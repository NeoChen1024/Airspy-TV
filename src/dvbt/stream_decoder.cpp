#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/tps_decoder.hpp"

#include <fftw3.h>
#include <liquid/liquid.h>
#include <volk/volk.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float minimum_power = 1.0e-12F;
constexpr float input_scale = 32768.0F;
constexpr std::size_t acquisition_samples = 350'000;
constexpr std::size_t buffer_duration_denominator = 5;
constexpr std::size_t initial_symbol_queue_capacity = 256;
constexpr std::size_t ts_packet_size = 188;
// Resampled-sample ring shared by the front-end thread (producer) and the
// demod thread (consumer): 1 Mi complex samples = 8 MiB, ~150 ms at the
// nominal 8K/guard-1/4 rate, enough slack for the acquisition window and
// scheduling jitter without ever blocking the front-end for long.
constexpr std::size_t ring_capacity_samples = 1'048'576;
// Demod statistics window: 400 OFDM symbols (~0.6 s at 8K/guard-1/4), the
// report cadence for the CLI (previously one per 7M-sample chunk).
constexpr std::size_t stats_window_symbols = 400;
// MER gate window: one TPS superframe (68 symbols). Windows whose mean MER
// falls below the constellation's decode floor are dropped and bracket a
// fresh FEC trellis at the region edges.
constexpr std::size_t gate_window_symbols = 68;
constexpr std::size_t acquisition_monitor_interval_ms = 2000;
constexpr std::size_t resampler_semi_length = 12;

constexpr std::array continual_2k{
    0,    48,   54,   87,   141,  156,  192,  201,  255,  279,  282,  333,
    432,  450,  483,  525,  531,  618,  636,  714,  759,  765,  780,  804,
    873,  888,  918,  939,  942,  969,  984,  1050, 1101, 1107, 1110, 1137,
    1140, 1146, 1206, 1269, 1323, 1377, 1491, 1683, 1704};
constexpr std::array tps_2k{34,  50,   209,  346,  413,  569,  595,  688, 790,
                            901, 1073, 1219, 1262, 1286, 1469, 1594, 1687};

[[nodiscard]] constexpr std::array<std::uint8_t, 6817> make_prbs() {
    std::array<std::uint8_t, 6817> result{};
    std::uint32_t state = 0x7ffU;
    for (auto &bit : result) {
        bit = static_cast<std::uint8_t>(state & 1U);
        state = (state >> 1U) | ((((state >> 2U) ^ state) & 1U) << 10U);
    }
    return result;
}
constexpr auto prbs = make_prbs();

[[nodiscard]] std::complex<float>
carrier(const std::span<const std::complex<float>> fft, const std::size_t index,
        const std::size_t maximum, const int offset) {
    const auto bin = static_cast<std::ptrdiff_t>(index) -
                     static_cast<std::ptrdiff_t>(maximum / 2) + offset;
    const auto wrapped = (bin + static_cast<std::ptrdiff_t>(fft.size())) %
                         static_cast<std::ptrdiff_t>(fft.size());
    return fft[static_cast<std::size_t>(wrapped)];
}

struct PilotLock {
    int phase{};
    int offset{};
};

[[nodiscard]] PilotLock
lock_pilots(const std::span<const std::complex<float>> fft,
            const std::size_t maximum, const int previous_offset) {
    PilotLock best;
    float best_score = -1.0F;
    const int radius =
        previous_offset == std::numeric_limits<int>::max() ? 48 : 2;
    const int center = previous_offset == std::numeric_limits<int>::max()
                           ? 0
                           : previous_offset;
    for (int offset = center - radius; offset <= center + radius; ++offset) {
        for (int phase = 0; phase < 4; ++phase) {
            std::complex<float> correlation{};
            float score = 0.0F;
            std::size_t chunk_count = 0;
            for (std::size_t pilot = static_cast<std::size_t>(phase * 3);
                 pilot <= maximum; pilot += 12) {
                const float value =
                    prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                correlation +=
                    value * std::conj(carrier(fft, pilot, maximum, offset));
                if (++chunk_count == 8) {
                    score += std::norm(correlation);
                    correlation = {};
                    chunk_count = 0;
                }
            }
            score += std::norm(correlation);
            if (score > best_score) {
                best_score = score;
                best = {phase, offset};
            }
        }
    }
    return best;
}

[[nodiscard]] bool listed(const std::span<const int> list,
                          const std::size_t value) {
    return std::ranges::binary_search(list, static_cast<int>(value));
}

struct PostprocessedSymbol {
    std::vector<std::complex<float>> carriers;
    std::vector<float> reliabilities;
    std::vector<std::uint8_t> mother_metrics;
    std::size_t symbol_index{};
    float mer_db{};
    float demap_time_ms{};
    float deinterleave_time_ms{};
    float depuncture_time_ms{};
};

struct WorkerAllocation {
    std::size_t symbol{};
    std::size_t viterbi{};
};

[[nodiscard]] WorkerAllocation
allocate_workers(const std::size_t requested_threads) noexcept {
    const std::size_t total = requested_threads == 0
                                  ? default_viterbi_worker_count()
                                  : requested_threads;
    if (total <= 1) {
        return {1, 1};
    }
    const std::size_t symbol = std::max<std::size_t>(1, total / 3);
    return {symbol, std::max<std::size_t>(1, total - symbol)};
}

[[nodiscard]] std::size_t
buffered_symbol_count(const std::uint32_t bandwidth,
                      const std::size_t symbol_samples) noexcept {
    // Nominal DVB-T sample rate is bandwidth * 8 / 7; retain one fifth of a
    // second at the current OFDM symbol duration.
    const std::uint64_t numerator = static_cast<std::uint64_t>(bandwidth) * 8U;
    const std::uint64_t denominator =
        7U * buffer_duration_denominator * symbol_samples;
    return std::max<std::size_t>(
        1,
        static_cast<std::size_t>((numerator + denominator - 1) / denominator));
}

[[nodiscard]] std::size_t
buffered_input_samples(const std::uint32_t sample_rate) noexcept {
    return std::max<std::size_t>(1, (static_cast<std::size_t>(sample_rate) +
                                     buffer_duration_denominator - 1) /
                                        buffer_duration_denominator);
}

[[nodiscard]] float
duration_ms(const std::chrono::steady_clock::time_point started_at) {
    return std::chrono::duration<float, std::milli>(
               std::chrono::steady_clock::now() - started_at)
        .count();
}

class SymbolPostprocessorPool {
  public:
    SymbolPostprocessorPool(const std::size_t requested_workers,
                            const TransmissionMode mode,
                            const Constellation constellation,
                            const CodeRate code_rate,
                            const std::size_t queue_capacity)
        : worker_count_(requested_workers == 0 ? default_viterbi_worker_count()
                                               : requested_workers),
          mode_(mode), constellation_(constellation), reference_(constellation),
          symbol_deinterleaver_(mode),
          bits_per_carrier_(bits_per_symbol(constellation)),
          code_rate_(code_rate), maximum_queued_(std::max<std::size_t>(
                                     queue_capacity, worker_count_ * 2)) {
        workers_.reserve(worker_count_);
        try {
            for (std::size_t index = 0; index < worker_count_; ++index) {
                workers_.emplace_back([this] { run_worker(); });
            }
        } catch (...) {
            stop_and_join();
            throw;
        }
    }

    ~SymbolPostprocessorPool() { stop_and_join(); }
    SymbolPostprocessorPool(const SymbolPostprocessorPool &) = delete;
    SymbolPostprocessorPool &
    operator=(const SymbolPostprocessorPool &) = delete;

    [[nodiscard]] bool
    compatible(const std::size_t worker_count, const TransmissionMode mode,
               const Constellation constellation, const CodeRate code_rate,
               const std::size_t queue_capacity) const noexcept {
        return worker_count_ == worker_count && mode_ == mode &&
               constellation_ == constellation && code_rate_ == code_rate &&
               maximum_queued_ ==
                   std::max<std::size_t>(queue_capacity, worker_count * 2);
    }

    void submit(std::vector<std::complex<float>> carriers,
                std::vector<float> equalizer_power,
                const std::size_t symbol_index) {
        Task task{.carriers = std::move(carriers),
                  .equalizer_power = std::move(equalizer_power),
                  .symbol_index = symbol_index};
        {
            std::unique_lock lock(mutex_);
            space_available_.wait(lock, [this] {
                return stopping_ || worker_error_ ||
                       tasks_.size() < maximum_queued_;
            });
            rethrow_worker_error();
            if (stopping_) {
                return;
            }
            task.sequence = next_sequence_++;
            ++outstanding_;
            tasks_.push_back(std::move(task));
        }
        task_ready_.notify_one();
    }

    [[nodiscard]] std::vector<PostprocessedSymbol> take_ready() {
        std::scoped_lock lock(mutex_);
        rethrow_worker_error();
        return take_ready_locked();
    }

    [[nodiscard]] std::vector<PostprocessedSymbol> flush() {
        std::unique_lock lock(mutex_);
        finished_.wait(lock,
                       [this] { return outstanding_ == 0 || worker_error_; });
        rethrow_worker_error();
        return take_ready_locked();
    }

  private:
    struct Task {
        std::uint64_t sequence{};
        std::vector<std::complex<float>> carriers;
        std::vector<float> equalizer_power;
        std::size_t symbol_index{};
    };

    [[nodiscard]] PostprocessedSymbol process(Task task) const {
        std::vector<std::complex<float>> nearest(task.carriers.size());
        for (int iteration = 0; iteration < 2; ++iteration) {
            reference_.slice_nearest(task.carriers, nearest);
            std::complex<double> numerator{};
            double denominator = 0.0;
            for (std::size_t index = 0; index < task.carriers.size(); ++index) {
                const auto value = task.carriers[index];
                const auto reference = nearest[index];
                numerator +=
                    std::conj(static_cast<std::complex<double>>(reference)) *
                    static_cast<std::complex<double>>(value);
                denominator += std::norm(reference);
            }
            const auto gain =
                static_cast<std::complex<float>>(numerator / denominator);
            if (std::abs(gain) > 1.0e-6F) {
                const std::complex<float> inverse_gain = 1.0F / gain;
                for (auto &value : task.carriers) {
                    value *= inverse_gain;
                }
            }
        }

        std::vector<float> errors;
        errors.reserve(task.carriers.size());
        reference_.slice_nearest(task.carriers, nearest);
        for (std::size_t index = 0; index < task.carriers.size(); ++index) {
            errors.push_back(std::norm(task.carriers[index] - nearest[index]));
        }
        const double mean_error =
            std::accumulate(errors.begin(), errors.end(), 0.0) /
            static_cast<double>(errors.size());
        auto middle =
            errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2);
        std::ranges::nth_element(errors, middle);
        const float reliability =
            1.0F / std::max(*middle / std::log(2.0F), 1.0e-4F);

        auto equalizer_power_order = task.equalizer_power;
        auto equalizer_middle =
            equalizer_power_order.begin() +
            static_cast<std::ptrdiff_t>(equalizer_power_order.size() / 2);
        std::ranges::nth_element(equalizer_power_order, equalizer_middle);
        const float median_equalizer_power =
            std::max(*equalizer_middle, minimum_power);
        std::vector<float> reliabilities(task.carriers.size());
        for (std::size_t index = 0; index < reliabilities.size(); ++index) {
            const float relative_channel_power =
                median_equalizer_power /
                std::max(task.equalizer_power[index], minimum_power);
            reliabilities[index] =
                reliability * std::clamp(relative_channel_power, 0.01F, 16.0F);
        }

        const std::size_t metric_count =
            task.carriers.size() * bits_per_carrier_;
        std::vector<float> demapped(metric_count);
        std::vector<float> symbol_metrics(metric_count);
        std::vector<float> bit_metrics(metric_count);
        const auto demap_started_at = std::chrono::steady_clock::now();
        reference_.demap(task.carriers, reliabilities, demapped);
        const auto deinterleave_started_at = std::chrono::steady_clock::now();
        symbol_deinterleaver_.process(demapped, bits_per_carrier_,
                                      task.symbol_index, symbol_metrics);
        bit_deinterleave(symbol_metrics, bits_per_carrier_, bit_metrics);
        const auto depuncture_started_at = std::chrono::steady_clock::now();
        std::vector<float> depunctured(
            depunctured_size(bit_metrics.size(), code_rate_));
        depuncture(bit_metrics, code_rate_, depunctured);
        std::vector<std::uint8_t> mother_metrics(depunctured.size());
        for (std::size_t index = 0; index < depunctured.size(); ++index) {
            const float soft =
                std::clamp(127.5F + (depunctured[index] * 8.0F), 0.0F, 255.0F);
            mother_metrics[index] = static_cast<std::uint8_t>(soft + 0.5F);
        }
        const auto finished_at = std::chrono::steady_clock::now();
        return {.carriers = std::move(task.carriers),
                .reliabilities = std::move(reliabilities),
                .mother_metrics = std::move(mother_metrics),
                .symbol_index = task.symbol_index,
                .mer_db = static_cast<float>(
                    -10.0 * std::log10(std::max(mean_error, 1.0e-12))),
                .demap_time_ms = std::chrono::duration<float, std::milli>(
                                     deinterleave_started_at - demap_started_at)
                                     .count(),
                .deinterleave_time_ms =
                    std::chrono::duration<float, std::milli>(
                        depuncture_started_at - deinterleave_started_at)
                        .count(),
                .depuncture_time_ms = std::chrono::duration<float, std::milli>(
                                          finished_at - depuncture_started_at)
                                          .count()};
    }

    void run_worker() {
        while (true) {
            Task task;
            {
                std::unique_lock lock(mutex_);
                task_ready_.wait(
                    lock, [this] { return stopping_ || !tasks_.empty(); });
                if (stopping_ && tasks_.empty()) {
                    return;
                }
                task = std::move(tasks_.front());
                tasks_.pop_front();
            }
            space_available_.notify_one();
            try {
                const std::uint64_t sequence = task.sequence;
                auto result = process(std::move(task));
                const std::scoped_lock lock(mutex_);
                completed_.emplace(sequence, std::move(result));
                --outstanding_;
            } catch (...) {
                const std::scoped_lock lock(mutex_);
                if (!worker_error_) {
                    worker_error_ = std::current_exception();
                }
                --outstanding_;
            }
            finished_.notify_all();
            space_available_.notify_all();
        }
    }

    [[nodiscard]] std::vector<PostprocessedSymbol> take_ready_locked() {
        std::vector<PostprocessedSymbol> output;
        auto found = completed_.find(next_result_);
        while (found != completed_.end()) {
            output.push_back(std::move(found->second));
            completed_.erase(found);
            ++next_result_;
            found = completed_.find(next_result_);
        }
        return output;
    }

    void stop_and_join() {
        {
            const std::scoped_lock lock(mutex_);
            stopping_ = true;
        }
        task_ready_.notify_all();
        space_available_.notify_all();
        for (auto &worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    void rethrow_worker_error() const {
        if (worker_error_) {
            std::rethrow_exception(worker_error_);
        }
    }

    const std::size_t worker_count_;
    const TransmissionMode mode_;
    const Constellation constellation_;
    const MaxLogDemapper reference_;
    const SymbolDeinterleaver symbol_deinterleaver_;
    const std::size_t bits_per_carrier_;
    const CodeRate code_rate_;
    const std::size_t maximum_queued_;
    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable task_ready_;
    std::condition_variable space_available_;
    std::condition_variable finished_;
    std::deque<Task> tasks_;
    std::map<std::uint64_t, PostprocessedSymbol> completed_;
    std::uint64_t next_sequence_{};
    std::uint64_t next_result_{};
    std::size_t outstanding_{};
    std::exception_ptr worker_error_;
    bool stopping_{};
};

// Single-threaded rational resampler with persistent filter state. The old
// chunked pipeline recreated the filter (and its 16-worker partition) on every
// 7M-sample chunk, forcing each chunk to re-warm the transient and re-run
// acquisition; this object streams one continuous filter state across the
// whole capture (resampling is memory-bandwidth-bound, so a thread split does
// not help — one thread is the right granularity).
class StreamingResampler {
  public:
    ~StreamingResampler() {
        if (filter_ != nullptr) {
            rresamp_crcf_destroy(filter_);
        }
    }
    StreamingResampler() = default;
    StreamingResampler(const StreamingResampler &) = delete;
    StreamingResampler &operator=(const StreamingResampler &) = delete;
    StreamingResampler(StreamingResampler &&other) noexcept {
        *this = std::move(other);
    }
    StreamingResampler &operator=(StreamingResampler &&other) noexcept {
        if (this != &other) {
            if (filter_ != nullptr) {
                rresamp_crcf_destroy(filter_);
            }
            filter_ = other.filter_;
            rate_ = other.rate_;
            bandwidth_ = other.bandwidth_;
            other.filter_ = nullptr;
        }
        return *this;
    }

    void configure(const std::uint32_t rate, const std::uint32_t bandwidth) {
        if (filter_ != nullptr && rate == rate_ && bandwidth == bandwidth_) {
            return;
        }
        if (filter_ != nullptr) {
            rresamp_crcf_destroy(filter_);
            filter_ = nullptr;
        }
        // Nominal DVB-T output rate is bandwidth * 8 / 7 samples/second.
        const std::uint64_t interpolation =
            static_cast<std::uint64_t>(bandwidth) * 8U;
        const std::uint64_t decimation = static_cast<std::uint64_t>(rate) * 7U;
        const std::uint64_t divisor = std::gcd(interpolation, decimation);
        const unsigned int p =
            static_cast<unsigned int>(interpolation / divisor);
        const unsigned int q = static_cast<unsigned int>(decimation / divisor);
        filter_ = rresamp_crcf_create_kaiser(p, q, resampler_semi_length, -1.0F,
                                             60.0F);
        if (filter_ == nullptr) {
            throw std::runtime_error("failed to create streaming resampler");
        }
        rate_ = rate;
        bandwidth_ = bandwidth;
        interpolation_ = p;
        decimation_ = q;
        accumulator_.clear();
    }

    // Consumes the input and appends the produced samples to `output`. The
    // input is accumulated until a full decimation block is available, then
    // executed through the persistent filter state (which carries the poly-
    // phase delay line across calls, so the output is a continuous stream
    // with no per-block transient). The residual input (< one block) stays
    // in the accumulator for the next call.
    void process(const std::span<const std::complex<float>> input,
                 std::vector<std::complex<float>> &output) {
        output.clear();
        if (!configured()) {
            return;
        }
        accumulator_.insert(accumulator_.end(), input.begin(), input.end());
        const std::size_t blocks = accumulator_.size() / decimation_;
        if (blocks == 0) {
            return;
        }
        output.resize(blocks * interpolation_);
        rresamp_crcf_execute_block(filter_, accumulator_.data(),
                                   static_cast<unsigned int>(blocks),
                                   output.data());
        accumulator_.erase(accumulator_.begin(),
                           accumulator_.begin() + static_cast<std::ptrdiff_t>(
                                                      blocks * decimation_));
    }

    [[nodiscard]] bool configured() const noexcept {
        return filter_ != nullptr;
    }
    [[nodiscard]] std::uint32_t rate() const noexcept { return rate_; }
    [[nodiscard]] std::uint32_t bandwidth() const noexcept {
        return bandwidth_;
    }

  private:
    rresamp_crcf filter_{};
    std::uint32_t rate_{};
    std::uint32_t bandwidth_{};
    std::size_t interpolation_{};
    std::size_t decimation_{};
    std::vector<std::complex<float>> accumulator_;
};

} // namespace

struct StreamDecoder::Impl {
    struct Block {
        std::vector<std::int16_t> samples;
        std::uint32_t rate{};
        std::uint32_t bandwidth{};
    };

    // Continuous front-end tracking state owned by the demod thread and
    // carried for the life of a stream. The resampled symbol stream is
    // contiguous (no overlap rewind), so the CFO loop, carrier search,
    // continual-carrier reference, AND TPS superframe decoder all carry
    // continuously; they are re-seeded only on cold starts (mode/guard
    // changes or resets). The TPS carry is the key weak-signal win: with the
    // old chunked pipeline every chunk re-locked TPS from scratch (~68
    // symbols) and the per-chunk CFO boundary overshoot perturbed tracking.
    struct FrontendState {
        bool valid{false};
        TransmissionMode mode{TransmissionMode::k8};
        GuardInterval guard{GuardInterval::gi_1_4};
        std::size_t fft_size{};
        std::size_t guard_size{};
        float tracked_cfo_phase{0.0F};
        float residual_phase_ema{0.0F};
        int carrier_offset{std::numeric_limits<int>::max()};
        // Values captured while the tracking was last healthy; a fade never
        // moves the carrier grid (the LO is stable), so a cold re-anchor
        // restores these instead of re-running the ambiguous wide pilot lock.
        int stable_carrier_offset{std::numeric_limits<int>::max()};
        int stable_phase{-1};
        std::vector<std::complex<float>> previous_continual;
        int previous_phase{-1};
        std::uint64_t phase_discontinuities{0};
        // Absolute stream position of the most recent processed symbol; the
        // CFO loop only updates from contiguous symbol pairs (start ==
        // last_symbol_start + period), so the first symbol after a re-anchor
        // is skipped exactly like the first symbol of the old chunks.
        std::uint64_t last_symbol_start{0};
        bool just_seeded{false};
        TpsDecoder tps_decoder;
        TpsSnapshot tps_snapshot;
    };

    // Acquisition result published by the front-end thread. `version` is
    // bumped only when the demod must act: mode/guard changes (cold re-anchor)
    // or a boundary shift beyond the alignment tolerance (the demod's grid
    // stays aligned otherwise, so a plain boundary refresh is silent).
    // An invalid publish (reset) abandons the current stream.
    struct SyncState {
        bool valid{false};
        std::uint64_t version{0};
        std::uint64_t start_pos{0};
        std::complex<float> phase{};
        float score{};
        TransmissionMode mode{TransmissionMode::k8};
        GuardInterval guard{GuardInterval::gi_1_4};
        std::size_t fft_size{};
        std::size_t guard_size{};
        std::uint32_t bandwidth{};
        float resampled_rate{};
    };

    struct FecItem {
        enum class Kind { begin, symbol, end, stats };

        Kind kind{Kind::symbol};
        std::uint64_t generation{};
        DecoderParameters parameters{};
        std::vector<std::uint8_t> mother_metrics;
        std::size_t symbol_index{};
    };

    mutable std::mutex mutex;
    std::condition_variable input_ready;
    std::condition_variable input_not_full;
    std::condition_variable ring_data;
    std::condition_variable ring_space;
    std::condition_variable fec_ready;
    std::condition_variable fec_not_full;
    std::condition_variable idle;
    std::deque<Block> queue;
    std::deque<FecItem> fec_queue;
    std::size_t queued_complex_samples{};
    std::size_t input_queue_capacity_samples{};
    std::size_t fec_queue_capacity{initial_symbol_queue_capacity};
    // Circular buffer of resampled samples. Positions are absolute uint64
    // stream offsets; the ring retains [ring_read_pos, ring_write_pos).
    std::vector<std::complex<float>> ring;
    std::uint64_t ring_write_pos{};
    std::uint64_t ring_read_pos{};
    bool ring_closed{};
    SyncState sync;
    TransportCallback callback;
    EqualizedCallback equalized_callback;
    ReceiverParameters parameters;
    dvbt::SignalAnalyzer analyzer;
    std::optional<TransmissionMode> stable_mode;
    std::optional<GuardInterval> stable_guard;
    FrontendState frontend;
    StreamDecoderStats latest;
    std::atomic<bool> cancel_requested{};
    bool stopping{};
    bool reset_requested{};
    bool flush_requested{};
    // Set by the demod thread when its tracking has been lost for a long
    // fade; the front-end responds with an immediate fresh acquisition whose
    // publish forces a cold re-anchor (CP-phase CFO seed + wide carrier lock
    // + TPS re-lock), mirroring the per-chunk re-acquisition of the old
    // chunked pipeline.
    bool reacquisition_requested{};
    std::uint64_t reacquisition_generation{};
    bool frontend_busy{};
    bool demod_busy{};
    bool fec_worker_busy{};
    std::atomic<std::uint64_t> latest_generation{};
    std::unique_ptr<SymbolPostprocessorPool> symbol_postprocessor;
    std::thread frontend_thread;
    std::thread demod_thread;
    std::thread fec_thread;

    Impl()
        : ring(ring_capacity_samples),
          frontend_thread([this] { run_frontend(); }),
          demod_thread([this] { run_demod(); }),
          fec_thread([this] { run_fec(); }) {}
    ~Impl() {
        cancel_requested = true;
        {
            const std::scoped_lock lock(mutex);
            stopping = true;
        }
        input_ready.notify_one();
        input_not_full.notify_all();
        ring_data.notify_all();
        ring_space.notify_all();
        fec_ready.notify_one();
        fec_not_full.notify_all();
        frontend_thread.join();
        demod_thread.join();
        fec_thread.join();
    }

    void reset_frontend_state() noexcept {
        frontend.valid = false;
        frontend.fft_size = 0;
        frontend.guard_size = 0;
        frontend.tracked_cfo_phase = 0.0F;
        frontend.residual_phase_ema = 0.0F;
        frontend.carrier_offset = std::numeric_limits<int>::max();
        frontend.previous_continual.clear();
        frontend.previous_phase = -1;
        frontend.phase_discontinuities = 0;
        frontend.last_symbol_start = 0;
        frontend.just_seeded = false;
        frontend.tps_decoder.reset();
        frontend.tps_snapshot = {};
    }

    [[nodiscard]] bool enqueue_fec(FecItem item) {
        std::unique_lock lock(mutex);
        fec_not_full.wait(lock, [this, generation = item.generation] {
            return stopping || cancel_requested ||
                   generation != latest_generation ||
                   fec_queue.size() < fec_queue_capacity;
        });
        if (stopping || cancel_requested ||
            item.generation != latest_generation) {
            return false;
        }
        fec_queue.push_back(std::move(item));
        fec_ready.notify_one();
        return true;
    }

    // ------------------------------------------------------------------ //
    // Front-end thread: cs16 -> resampled cfloat -> ring, plus the rolling
    // acquisition window. Runs independently of the demod so symbol extraction
    // is never blocked behind resampling (or vice versa); the 16-way partition
    // of the old resampler is gone because resampling is memory-bandwidth-
    // bound and a single streaming filter state is both simpler and correct.
    // ------------------------------------------------------------------ //
    void run_frontend() {
        StreamingResampler resampler;
        std::vector<std::complex<float>> convert_buffer;
        std::vector<std::complex<float>> resample_buffer;
        std::vector<std::complex<float>> acquisition_window;
        bool need_acquisition = true;
        auto last_monitor = std::chrono::steady_clock::now();
        while (true) {
            Block block;
            bool close_ring = false;
            {
                std::unique_lock lock(mutex);
                input_ready.wait(lock, [this] {
                    return stopping || reset_requested || flush_requested ||
                           !queue.empty();
                });
                if (stopping) {
                    return;
                }
                if (reset_requested) {
                    // Abandon the whole pipeline. The demod sees the invalid
                    // sync and drops its per-stream state; the FEC worker
                    // drops items whose generation is stale.
                    reset_requested = false;
                    cancel_requested = false;
                    queue.clear();
                    fec_queue.clear();
                    queued_complex_samples = 0;
                    ring_read_pos = 0;
                    ring_write_pos = 0;
                    ring_closed = false;
                    sync.valid = false;
                    ++sync.version;
                    stable_mode.reset();
                    stable_guard.reset();
                    latest = {};
                    ++latest_generation;
                    acquisition_window.clear();
                    need_acquisition = true;
                    resampler = StreamingResampler{};
                    input_not_full.notify_all();
                    fec_not_full.notify_all();
                    ring_data.notify_all();
                    idle.notify_all();
                    continue;
                }
                if (queue.empty()) {
                    if (flush_requested) {
                        flush_requested = false;
                        ring_closed = true;
                        close_ring = true;
                    } else {
                        idle.notify_all();
                        continue;
                    }
                } else {
                    block = std::move(queue.front());
                    queue.pop_front();
                    queued_complex_samples -= block.samples.size() / 2;
                    input_not_full.notify_one();
                    ++latest.input_blocks;
                    frontend_busy = true;
                }
            }
            if (close_ring) {
                ring_data.notify_all();
                frontend_busy = false;
                idle.notify_all();
                continue;
            }
            if (!block.samples.empty()) {
                // A flush followed by new submits resumes the same stream:
                // reopen the ring so the demod continues past the seam.
                if (ring_closed) {
                    {
                        const std::scoped_lock lock(mutex);
                        ring_closed = false;
                    }
                    ring_data.notify_all();
                }
                if (resampler.configured() &&
                    (resampler.rate() != block.rate ||
                     resampler.bandwidth() != block.bandwidth)) {
                    // Retune: drop the filter state and re-acquire from
                    // scratch (the demod will cold-start on the new sync).
                    resampler = StreamingResampler{};
                    acquisition_window.clear();
                    need_acquisition = true;
                }
                if (!resampler.configured()) {
                    resampler.configure(block.rate, block.bandwidth);
                }
                const std::size_t complex_count = block.samples.size() / 2;
                convert_buffer.resize(complex_count);
                volk_16i_s32f_convert_32f(
                    reinterpret_cast<float *>(convert_buffer.data()),
                    block.samples.data(), input_scale,
                    static_cast<unsigned int>(complex_count * 2));
                const auto resample_started_at =
                    std::chrono::steady_clock::now();
                resampler.process(convert_buffer, resample_buffer);
                const std::uint64_t block_base = [&] {
                    const std::scoped_lock lock(mutex);
                    return ring_write_pos;
                }();
                // First acquisition: the head of this resampled block, the
                // same window the old chunked pipeline acquired on. The demod
                // needs the samples at this position, so the sync is published
                // BEFORE the ring push; the ring push below is incremental
                // (the ring is much smaller than a block) and paces itself
                // against the demod's consumption.
                if (need_acquisition &&
                    resample_buffer.size() >= acquisition_samples) {
                    acquire_and_publish(
                        std::span(resample_buffer).first(acquisition_samples),
                        block_base, block.bandwidth);
                    need_acquisition = false;
                    last_monitor = std::chrono::steady_clock::now();
                }
                // Rolling acquisition window (for monitor re-acquisitions):
                // keep the most recent acquisition_samples of resampled data.
                acquisition_window.insert(acquisition_window.end(),
                                          resample_buffer.begin(),
                                          resample_buffer.end());
                if (acquisition_window.size() > acquisition_samples) {
                    const std::size_t excess =
                        acquisition_window.size() - acquisition_samples;
                    acquisition_window.erase(
                        acquisition_window.begin(),
                        acquisition_window.begin() +
                            static_cast<std::ptrdiff_t>(excess));
                }
                // Push to the ring incrementally: the ring (1 Mi samples) is
                // smaller than a block (~4.8 Mi), so each iteration pushes
                // only what fits and waits for the demod to free space.
                std::size_t pushed = 0;
                while (pushed < resample_buffer.size()) {
                    std::unique_lock lock(mutex);
                    ring_space.wait(lock, [this] {
                        return stopping ||
                               ring_write_pos - ring_read_pos < ring.size();
                    });
                    if (stopping) {
                        return;
                    }
                    const std::size_t used = ring_write_pos - ring_read_pos;
                    const std::size_t chunk = std::min(
                        ring.size() - used, resample_buffer.size() - pushed);
                    for (std::size_t i = 0; i < chunk; ++i) {
                        ring[(ring_write_pos + i) % ring.size()] =
                            resample_buffer[pushed + i];
                    }
                    ring_write_pos += chunk;
                    pushed += chunk;
                    ring_data.notify_all();
                }
                {
                    const std::scoped_lock lock(mutex);
                    latest.processed_input_samples += complex_count;
                    latest.resample_time_ms = duration_ms(resample_started_at);
                }
                const auto now = std::chrono::steady_clock::now();
                bool force_acquisition = false;
                {
                    const std::scoped_lock lock(mutex);
                    if (reacquisition_requested) {
                        force_acquisition = true;
                        reacquisition_requested = false;
                    }
                }
                if (force_acquisition ||
                    (!need_acquisition &&
                     now - last_monitor >=
                         std::chrono::milliseconds(
                             acquisition_monitor_interval_ms))) {
                    last_monitor = now;
                    const std::uint64_t window_base = [&] {
                        const std::scoped_lock lock(mutex);
                        return ring_write_pos - acquisition_window.size();
                    }();
                    acquire_and_publish(
                        std::span(acquisition_window).last(acquisition_samples),
                        window_base, block.bandwidth, force_acquisition);
                }
            }
            {
                const std::scoped_lock lock(mutex);
                frontend_busy = false;
            }
            idle.notify_all();
        }
    }

    void acquire_and_publish(const std::span<const std::complex<float>> window,
                             const std::uint64_t base_pos,
                             const std::uint32_t bandwidth,
                             const bool forced = false) {
        if (window.size() < acquisition_samples) {
            return;
        }
        const auto acquired_at = std::chrono::steady_clock::now();
        ReceiverParameters acquisition_parameters;
        {
            const std::scoped_lock lock(mutex);
            acquisition_parameters = parameters;
        }
        const auto span = window.first(acquisition_samples);
        const OfdmAcquisition acquisition =
            acquire_ofdm(span, acquisition_parameters);
        const float acquisition_ms = duration_ms(acquired_at);
        if (acquisition.score < 0.20F) {
            return; // keep the current sync; the demod keeps tracking
        }
        const std::uint64_t start_pos = base_pos + acquisition.start;
        const float resampled_rate =
            static_cast<float>(bandwidth) * (8.0F / 7.0F);
        std::uint64_t new_version = 0;
        {
            const std::scoped_lock lock(mutex);
            stable_mode = acquisition.mode;
            stable_guard = acquisition.guard;
            latest.acquisition_score = acquisition.score;
            latest.acquisition_time_ms = acquisition_ms;
            latest.fft_size = static_cast<std::uint32_t>(acquisition.fft_size);
            latest.guard_size =
                static_cast<std::uint32_t>(acquisition.guard_size);
            const bool same_parameters = sync.valid &&
                                         sync.mode == acquisition.mode &&
                                         sync.guard == acquisition.guard;
            const std::uint64_t period =
                acquisition.fft_size + acquisition.guard_size;
            const std::uint64_t aligned_distance = [&] {
                if (!sync.valid) {
                    return std::uint64_t{0};
                }
                const std::uint64_t delta = sync.start_pos > start_pos
                                                ? sync.start_pos - start_pos
                                                : start_pos - sync.start_pos;
                return std::min(delta % period, period - delta % period);
            }();
            sync.valid = true;
            sync.start_pos = start_pos;
            sync.phase = acquisition.phase;
            sync.score = acquisition.score;
            sync.mode = acquisition.mode;
            sync.guard = acquisition.guard;
            sync.fft_size = acquisition.fft_size;
            sync.guard_size = acquisition.guard_size;
            sync.bandwidth = bandwidth;
            sync.resampled_rate = resampled_rate;
            // The demod re-anchors only on parameter changes or boundary
            // moves beyond a symbol's alignment tolerance; a silent refresh
            // (drift + estimator noise) does not disturb continuous tracking.
            // A forced re-acquisition always publishes so the demod can cold
            // re-anchor from the fresh CP-phase CFO seed.
            if (!same_parameters || aligned_distance >= 64 || forced) {
                ++sync.version;
                if (forced) {
                    ++reacquisition_generation;
                }
                new_version = sync.version;
            }
        }
        if (new_version != 0) {
            ring_data.notify_all();
        }
    }

    // ------------------------------------------------------------------ //
    // Demod thread: contiguous symbol extraction, CFO/channel/TPS tracking,
    // symbol postprocessing, and the windowed MER gate. One continuous symbol
    // stream per sync; re-anchors and resets are handled at the loop heads.
    // ------------------------------------------------------------------ //
    void run_demod() {
        try {
            ReceiverParameters selected_parameters;
            std::size_t maximum = 6816;
            std::size_t fft_size = 8192;
            std::size_t guard_size = 2048;
            std::size_t period = 10240;
            std::vector<std::complex<float>> fft_in;
            std::vector<std::complex<float>> fft_out;
            fftwf_plan plan = nullptr;
            std::vector<std::size_t> continual_indices;
            std::array<std::vector<std::size_t>, 4> pilot_indices;
            std::array<std::vector<std::size_t>, 4> payload_indices;
            std::optional<DecoderParameters> decoder_parameters;
            WorkerAllocation workers{1, 1};
            SymbolPostprocessorPool *postprocessor = nullptr;
            struct PendingSymbol {
                std::vector<std::complex<float>> payload;
                std::vector<float> equalizer_power;
                std::size_t fallback_index{};
            };
            std::deque<PendingSymbol> pending_symbols;
            std::deque<PostprocessedSymbol> gate_buffer;
            bool in_hopeless_region = false;
            bool last_reanchor_carried = false;
            std::uint64_t seen_sync_version = 0;
            std::uint64_t seen_reacquisition_generation = 0;
            bool have_grid = false;
            std::uint64_t next_symbol_start = 0;
            float nco_phase = 0.0F;
            std::uint64_t symbol_count = 0;
            std::uint64_t frozen_symbol_count = 0;
            std::uint64_t fade_symbol_count = 0;
            // Uncapped fade duration mod 4: the scattered-pilot phase rotates
            // once per symbol, so a cold re-anchor must restore the phase
            // advanced by the full fade length, not the capped counter.
            std::uint64_t fade_phase_count = 0;
            int lock_hold = 0;
            std::uint64_t window_symbol_count = 0;
            double mer_sum = 0.0;
            float demap_time_sum = 0.0F;
            float deinterleave_time_sum = 0.0F;
            float depuncture_time_sum = 0.0F;
            double timing_acc = 0.0;
            std::uint64_t timing_count = 0;
            auto window_started_at = std::chrono::steady_clock::now();

            const auto cold_seed = [&]() {
                // Seed the continuous tracking state from the sync's CP-phase
                // estimate instead of resuming carried state.
                frontend.tracked_cfo_phase =
                    std::arg(sync.phase) / static_cast<float>(fft_size);
                frontend.residual_phase_ema = 0.0F;
                frontend.carrier_offset = std::numeric_limits<int>::max();
                frontend.previous_continual.clear();
                frontend.previous_phase = -1;
                frontend.tps_decoder.reset();
                frontend.tps_snapshot = {};
                frontend.just_seeded = true;
            };

            const auto build_grid = [&]() {
                maximum = frontend.fft_size == 8192 ? 6816 : 1704;
                fft_size = frontend.fft_size;
                guard_size = frontend.guard_size;
                period = fft_size + guard_size;
                continual_indices.clear();
                pilot_indices = {};
                payload_indices = {};
                for (std::size_t k = 0; k <= maximum; ++k) {
                    const std::size_t base = k % 1704;
                    const bool continual_carrier = listed(continual_2k, base);
                    const bool tps_carrier = listed(tps_2k, base);
                    if (continual_carrier) {
                        continual_indices.push_back(k);
                    }
                    for (std::size_t phase = 0; phase < 4; ++phase) {
                        const bool scattered = k % 12 == phase * 3;
                        if (scattered || continual_carrier) {
                            pilot_indices[phase].push_back(k);
                        }
                        if (!scattered && !continual_carrier && !tps_carrier) {
                            payload_indices[phase].push_back(k);
                        }
                    }
                }
                fft_in.resize(fft_size);
                fft_out.resize(fft_size);
                if (plan != nullptr) {
                    fftwf_destroy_plan(plan);
                    plan = nullptr;
                }
                plan = fftwf_plan_dft_1d(
                    static_cast<int>(fft_size),
                    reinterpret_cast<fftwf_complex *>(fft_in.data()),
                    reinterpret_cast<fftwf_complex *>(fft_out.data()),
                    FFTW_FORWARD, FFTW_ESTIMATE);
            };

            std::size_t symbol_queue_capacity = initial_symbol_queue_capacity;
            // Handles a sync version change (first anchor, re-anchor, or
            // reset). Returns true when the demod must act (grid (re)built or
            // abandoned). Called with the mutex held.
            const auto handle_sync_change = [&]() {
                seen_sync_version = sync.version;
                // A forced re-acquisition (long fade) always cold-reanchors
                // even when the boundary looks aligned.
                const bool forced =
                    reacquisition_generation != seen_reacquisition_generation;
                seen_reacquisition_generation = reacquisition_generation;
                if (!sync.valid) {
                    // Reset: abandon the stream. The FEC generation has already
                    // advanced, so any stale queued items are dropped by the
                    // worker.
                    if (postprocessor != nullptr) {
                        static_cast<void>(symbol_postprocessor->flush());
                    }
                    postprocessor = nullptr;
                    pending_symbols.clear();
                    gate_buffer.clear();
                    in_hopeless_region = false;
                    decoder_parameters.reset();
                    have_grid = false;
                    frontend.valid = false;
                    frontend.just_seeded = false;
                    return true;
                }
                const bool mode_changed =
                    !frontend.valid || sync.fft_size != frontend.fft_size ||
                    sync.guard_size != frontend.guard_size;
                if (!have_grid || mode_changed) {
                    frontend.mode = sync.mode;
                    frontend.guard = sync.guard;
                    frontend.fft_size = sync.fft_size;
                    frontend.guard_size = sync.guard_size;
                    frontend.valid = true;
                    build_grid();
                    cold_seed();
                    // The mutex is already held by the caller.
                    selected_parameters = parameters;
                    if (!selected_parameters.mode.has_value() &&
                        stable_mode.has_value()) {
                        selected_parameters.mode = stable_mode;
                    }
                    if (!selected_parameters.guard_interval.has_value() &&
                        stable_guard.has_value()) {
                        selected_parameters.guard_interval = stable_guard;
                    }
                    workers =
                        allocate_workers(selected_parameters.worker_threads);
                    symbol_queue_capacity = buffered_symbol_count(
                        sync.bandwidth, fft_size + guard_size);
                    fec_queue_capacity = symbol_queue_capacity;
                    if (selected_parameters.constellation.has_value() &&
                        selected_parameters.code_rate.has_value()) {
                        decoder_parameters = DecoderParameters{
                            frontend.mode, *selected_parameters.constellation,
                            *selected_parameters.code_rate, workers.viterbi};
                    } else {
                        decoder_parameters.reset();
                    }
                    if (postprocessor != nullptr) {
                        static_cast<void>(symbol_postprocessor->flush());
                        postprocessor = nullptr;
                    }
                    pending_symbols.clear();
                    gate_buffer.clear();
                    in_hopeless_region = false;
                    next_symbol_start = sync.start_pos + sync.guard_size;
                    nco_phase = frontend.tracked_cfo_phase *
                                static_cast<float>(next_symbol_start);
                    last_reanchor_carried = false;
                    have_grid = true;
                    demod_busy = true;
                    return true;
                }
                // Same mode/guard: is the new boundary on the current grid?
                const std::uint64_t new_start =
                    sync.start_pos + sync.guard_size;
                const std::uint64_t delta = next_symbol_start > new_start
                                                ? next_symbol_start - new_start
                                                : new_start - next_symbol_start;
                const std::uint64_t aligned_distance =
                    std::min(delta % period, period - delta % period);
                if (aligned_distance < 64 && !forced) {
                    return false; // aligned: keep the grid and the tracking
                }
                // Re-anchor: the boundary moved (signal drop/recovery or a
                // first anchor estimate error). Tracking carries when the
                // mode/guard are unchanged. Never read past the samples the
                // ring has freed. When the tracking was lost for a long fade
                // (the forced re-acquisition the front-end just published),
                // cold-start instead: re-seed the CFO from the fresh CP-phase
                // estimate, widen the carrier search, and re-lock the TPS
                // superframe — the same recovery the old chunked pipeline got
                // from its per-chunk re-acquisition.
                const bool carried = frontend.valid;
                const bool stale_tracking = forced;
                std::uint64_t new_next = new_start;
                while (new_next < ring_read_pos) {
                    new_next += period;
                }
                if (stale_tracking) {
                    cold_seed();
                    // Restore the pre-fade carrier grid instead of trusting
                    // the ambiguous wide pilot lock; the phase advances one
                    // step per symbol across the fade (the uncapped counter
                    // keeps the mod-4 phase information).
                    if (frontend.stable_carrier_offset !=
                        std::numeric_limits<int>::max()) {
                        frontend.carrier_offset =
                            frontend.stable_carrier_offset;
                        frontend.previous_phase =
                            (frontend.stable_phase +
                             static_cast<int>(fade_phase_count)) %
                            4;
                        // Hold the restored grid for one symbol so the lock
                        // cannot immediately flip the correct phase onto a
                        // noise-latched one; the channel estimate and CFO
                        // re-establish first.
                        lock_hold = 1;
                    }
                }
                nco_phase =
                    stale_tracking
                        ? frontend.tracked_cfo_phase *
                              static_cast<float>(new_next)
                        : std::remainder(
                              nco_phase +
                                  frontend.tracked_cfo_phase *
                                      static_cast<float>(
                                          static_cast<std::int64_t>(new_next) -
                                          static_cast<std::int64_t>(
                                              next_symbol_start)),
                              2.0F * std::numbers::pi_v<float>);
                next_symbol_start = new_next;
                last_reanchor_carried = carried && !stale_tracking;
                fade_symbol_count = 0;
                return true;
            };

            // MER gate floor per constellation.
            const auto fec_floor = [](const Constellation constellation) {
                switch (constellation) {
                case Constellation::qpsk:
                    return 5.0F;
                case Constellation::qam16:
                    return 10.0F;
                case Constellation::qam64:
                    return 14.0F;
                }
                return 14.0F;
            };

            // Forwards a postprocessed batch through the equalized callback and
            // the windowed MER gate into the FEC queue. Hopeless windows (deep
            // fades) are dropped and bracket end/begin FEC resets at the region
            // edges so the trellis never grinds through noise; tracking is
            // unaffected.
            const auto process_batch = [&](std::vector<PostprocessedSymbol>
                                               batch) {
                for (auto &symbol : batch) {
                    mer_sum += symbol.mer_db;
                    demap_time_sum += symbol.demap_time_ms;
                    deinterleave_time_sum += symbol.deinterleave_time_ms;
                    depuncture_time_sum += symbol.depuncture_time_ms;
                    EqualizedCallback equalized_sink;
                    {
                        const std::scoped_lock guard(mutex);
                        equalized_sink = equalized_callback;
                    }
                    if (equalized_sink) {
                        equalized_sink(symbol.carriers, symbol.reliabilities,
                                       symbol.symbol_index);
                    }
                    gate_buffer.push_back(std::move(symbol));
                }
                if (!decoder_parameters) {
                    return;
                }
                const float floor =
                    fec_floor(decoder_parameters->constellation) + 4.0F;
                while (gate_buffer.size() >= gate_window_symbols) {
                    double window_mer = 0.0;
                    for (std::size_t i = 0; i < gate_window_symbols; ++i) {
                        window_mer += gate_buffer[i].mer_db;
                    }
                    window_mer /= static_cast<double>(gate_window_symbols);
                    const bool hopeless =
                        static_cast<float>(window_mer) < floor;
                    if (hopeless && !in_hopeless_region) {
                        in_hopeless_region = true;
                        static_cast<void>(
                            enqueue_fec({.kind = FecItem::Kind::end,
                                         .generation = latest_generation,
                                         .parameters = {},
                                         .mother_metrics = {},
                                         .symbol_index = 0}));
                    } else if (!hopeless && in_hopeless_region) {
                        in_hopeless_region = false;
                        static_cast<void>(
                            enqueue_fec({.kind = FecItem::Kind::begin,
                                         .generation = latest_generation,
                                         .parameters = *decoder_parameters,
                                         .mother_metrics = {},
                                         .symbol_index = 0}));
                    }
                    if (!hopeless) {
                        for (std::size_t i = 0; i < gate_window_symbols; ++i) {
                            static_cast<void>(enqueue_fec(
                                {.kind = FecItem::Kind::symbol,
                                 .generation = latest_generation,
                                 .parameters = {},
                                 .mother_metrics =
                                     std::move(gate_buffer[i].mother_metrics),
                                 .symbol_index = gate_buffer[i].symbol_index}));
                        }
                    }
                    gate_buffer.erase(
                        gate_buffer.begin(),
                        gate_buffer.begin() +
                            static_cast<std::ptrdiff_t>(gate_window_symbols));
                }
            };

            const auto start_decoder = [&]() {
                if (!decoder_parameters || postprocessor != nullptr) {
                    return true;
                }
                if (!enqueue_fec({.kind = FecItem::Kind::begin,
                                  .generation = latest_generation,
                                  .parameters = *decoder_parameters,
                                  .mother_metrics = {},
                                  .symbol_index = 0})) {
                    return false;
                }
                if (!symbol_postprocessor ||
                    !symbol_postprocessor->compatible(
                        workers.symbol, frontend.mode,
                        decoder_parameters->constellation,
                        decoder_parameters->code_rate, symbol_queue_capacity)) {
                    symbol_postprocessor =
                        std::make_unique<SymbolPostprocessorPool>(
                            workers.symbol, frontend.mode,
                            decoder_parameters->constellation,
                            decoder_parameters->code_rate,
                            symbol_queue_capacity);
                } else {
                    // A cancelled stream can leave completed symbols behind.
                    // Drain and discard them before reuse so no stale result
                    // crosses a stream or region boundary.
                    static_cast<void>(symbol_postprocessor->flush());
                }
                postprocessor = symbol_postprocessor.get();
                return true;
            };

            // Applies the pending statistics-window accumulators to `latest`.
            const auto publish_stats_window = [&]() {
                const float window_wall = duration_ms(window_started_at);
                const std::uint64_t window_symbols = window_symbol_count;
                const float input_seconds =
                    sync.resampled_rate > 0.0F
                        ? static_cast<float>(window_symbols) *
                              static_cast<float>(period) / sync.resampled_rate
                        : 0.0F;
                const float timing_offset =
                    timing_count == 0
                        ? 0.0F
                        : static_cast<float>(
                              timing_acc / static_cast<double>(timing_count)) *
                              static_cast<float>(fft_size) /
                              (2.0F * std::numbers::pi_v<float>);
                const std::scoped_lock lock(mutex);
                latest.ofdm_locked = true;
                latest.tps_locked = frontend.tps_snapshot.locked;
                latest.tps_constellation =
                    frontend.tps_snapshot.parameters.constellation;
                latest.tps_code_rate =
                    frontend.tps_snapshot.parameters.high_priority_code_rate;
                latest.tps_guard_interval =
                    frontend.tps_snapshot.parameters.guard_interval;
                latest.tps_mode = frontend.tps_snapshot.parameters.mode;
                latest.tps_hierarchy =
                    frontend.tps_snapshot.parameters.hierarchy;
                latest.carrier_bin_offset = frontend.carrier_offset;
                latest.tracked_carrier_offset_hz =
                    frontend.tracked_cfo_phase * sync.resampled_rate /
                    (2.0F * std::numbers::pi_v<float>);
                latest.acquisition_start =
                    static_cast<std::size_t>(sync.start_pos % period);
                latest.timing_offset_samples = timing_offset;
                latest.mer_db =
                    mer_sum == 0.0 && window_symbols == 0
                        ? 0.0F
                        : static_cast<float>(
                              mer_sum /
                              static_cast<double>(
                                  std::max<std::uint64_t>(window_symbols, 1)));
                latest.residual_carrier_offset_hz =
                    frontend.residual_phase_ema * sync.resampled_rate /
                    (2.0F * std::numbers::pi_v<float> *
                     static_cast<float>(period));
                latest.pilot_phase_discontinuities =
                    frontend.phase_discontinuities;
                latest.ofdm_symbols = symbol_count;
                latest.processing_realtime_ratio =
                    input_seconds > 0.0F
                        ? (window_wall / 1000.0F) / input_seconds
                        : 0.0F;
                latest.equalization_time_ms = window_wall;
                latest.demap_time_ms = demap_time_sum;
                latest.deinterleave_time_ms = deinterleave_time_sum;
                latest.depuncture_time_ms = depuncture_time_sum;
                latest.symbol_workers = workers.symbol;
                latest.resample_workers = 1;
                latest.state_carried = last_reanchor_carried;
                latest.fec_skipped = in_hopeless_region;
                ++latest.processed_chunks;
            };

            while (true) {
                // --- wait for a sync publish (first anchor / re-anchor / the
                //     resume of a flushed stream) ---
                {
                    std::unique_lock lock(mutex);
                    ring_data.wait(lock, [this, &seen_sync_version, &have_grid,
                                          &next_symbol_start] {
                        return stopping || sync.version != seen_sync_version ||
                               (have_grid && !ring_closed &&
                                ring_write_pos > next_symbol_start);
                    });
                    if (stopping) {
                        return;
                    }
                }
                {
                    const std::scoped_lock lock(mutex);
                    if (sync.version != seen_sync_version) {
                        static_cast<void>(handle_sync_change());
                    }
                }
                if (!have_grid) {
                    continue; // reset: wait for the next stream
                }
                // --- contiguous symbol stream ---
                while (true) {
                    {
                        const std::scoped_lock lock(mutex);
                        if (sync.version != seen_sync_version) {
                            static_cast<void>(handle_sync_change());
                        }
                    }
                    if (!have_grid) {
                        break; // reset mid-stream: drain nothing, wait for a
                               // sync
                    }
                    const std::uint64_t needed =
                        next_symbol_start +
                        static_cast<std::uint64_t>(fft_size);
                    {
                        std::unique_lock lock(mutex);
                        ring_data.wait(lock, [this, needed] {
                            return stopping ||
                                   (ring_closed && ring_write_pos < needed) ||
                                   ring_write_pos >= needed;
                        });
                        if (stopping) {
                            return;
                        }
                        if (ring_closed && ring_write_pos < needed) {
                            break; // end of stream
                        }
                    }
                    for (std::size_t i = 0; i < fft_size; ++i) {
                        const std::uint64_t position = next_symbol_start + i;
                        fft_in[i] = ring[position % ring.size()];
                    }
                    {
                        const std::scoped_lock lock(mutex);
                        ring_read_pos = needed;
                        ring_space.notify_all();
                    }
                    const std::uint64_t start = next_symbol_start;
                    std::complex<float> nco = std::polar(1.0F, -nco_phase);
                    const std::complex<float> nco_step =
                        std::polar(1.0F, -frontend.tracked_cfo_phase);
                    for (std::size_t i = 0; i < fft_size; ++i) {
                        fft_in[i] *= nco;
                        nco *= nco_step;
                        if ((i & 511U) == 511U) {
                            nco *= 1.0F / std::sqrt(std::norm(nco));
                        }
                    }
                    fftwf_execute(plan);
                    std::vector<std::complex<float>> current_continual;
                    current_continual.reserve(continual_indices.size());
                    for (const std::size_t k : continual_indices) {
                        current_continual.push_back(carrier(
                            fft_out, k, maximum, frontend.carrier_offset));
                    }
                    float residual_phase = 0.0F;
                    float fade_indicator = 1.0F;
                    // Only update the CFO loop from a contiguous symbol pair.
                    // The first symbol after a re-anchor follows the previous
                    // symbol by a non-period step, so its temporal correlation
                    // would measure a spurious residual and overshoot; the
                    // carried frequency is already converged, so skip the
                    // update.
                    if (frontend.previous_continual.size() ==
                            current_continual.size() &&
                        start == frontend.last_symbol_start +
                                     static_cast<std::uint64_t>(period)) {
                        std::complex<float> temporal_correlation{};
                        double carrier_power = 0.0;
                        for (std::size_t i = 0; i < current_continual.size();
                             ++i) {
                            temporal_correlation +=
                                current_continual[i] *
                                std::conj(frontend.previous_continual[i]);
                            carrier_power +=
                                std::norm(current_continual[i]) +
                                std::norm(frontend.previous_continual[i]);
                        }
                        residual_phase = std::arg(temporal_correlation);
                        // Fade gate: the normalized temporal correlation of the
                        // continual carriers collapses when the channel fades
                        // (the carriers vanish into the noise). Updating the
                        // loop on that uncorrelated garbage would drive the
                        // converged tracked CFO away from the true offset, and
                        // a deep fade long enough to integrate the error would
                        // leave the demodulator rotated when the signal returns
                        // (the old chunked pipeline was immune because a failed
                        // chunk acquisition froze the tracking). Freeze the
                        // loop until the correlation returns.
                        const float normalized_correlation =
                            carrier_power > 0.0
                                ? static_cast<float>(
                                      std::abs(temporal_correlation)) /
                                      static_cast<float>(carrier_power)
                                : 0.0F;
                        fade_indicator = normalized_correlation;
                        if (fade_indicator > 0.25F) {
                            constexpr float loop_gain = 0.20F;
                            frontend.tracked_cfo_phase +=
                                loop_gain * residual_phase /
                                static_cast<float>(period);
                            frontend.residual_phase_ema =
                                (0.1F * residual_phase) +
                                (0.9F * frontend.residual_phase_ema);
                        }
                    }
                    frontend.last_symbol_start = start;
                    frontend.previous_continual = std::move(current_continual);
                    if (frontend.just_seeded) {
                        frontend.just_seeded = false;
                    }
                    // Pilot phase/carrier lock, refreshed only when the channel
                    // is alive. During a fade the lock latches a noise-driven
                    // phase/offset, and its narrow +/-2-bin search cannot
                    // escape a bad latch when the signal returns, permanently
                    // scrambling the channel estimate and payload (this was the
                    // permanent lock loss the old chunked pipeline avoided by
                    // skipping failed-acquisition chunks entirely). Freeze the
                    // carried values instead; the pilot phase rotates mod 4 and
                    // fades span whole 4-symbol cycles, so the carried phase
                    // stays valid across the fade. A long freeze (a fade longer
                    // than one TPS frame) periodically re-runs the WIDE lock so
                    // the demodulator can re-grab the true grid the moment the
                    // signal returns, instead of staying latched on stale or
                    // noise-latched values.
                    PilotLock lock;
                    if (lock_hold > 0) {
                        --lock_hold;
                        lock =
                            PilotLock{static_cast<int>(frontend.previous_phase),
                                      frontend.carrier_offset};
                    } else if (fade_indicator > 0.25F ||
                               frontend.previous_phase < 0) {
                        lock = lock_pilots(fft_out, maximum,
                                           frontend.carrier_offset);
                        if (frontend.previous_phase >= 0 &&
                            lock.phase != (frontend.previous_phase + 1) % 4) {
                            ++frontend.phase_discontinuities;
                        }
                        frontend.previous_phase = lock.phase;
                        frontend.carrier_offset = lock.offset;
                        frontend.stable_phase = lock.phase;
                        frontend.stable_carrier_offset = lock.offset;
                        frozen_symbol_count = 0;
                        fade_symbol_count = 0;
                        fade_phase_count = 0;
                    } else {
                        if (++frozen_symbol_count >= 68) {
                            frozen_symbol_count = 0;
                            // Wide re-lock: search the full pilot grid (the
                            // narrow +/-2-bin lock cannot escape a noise
                            // latch). Verify the candidate against the
                            // phase-coherent scattered-pilot correlation: the
                            // true grid shows a strong, phase-matched
                            // correlation even through multipath, while
                            // noise-latched grids do not. Accept only a
                            // verified grid, otherwise keep the carried values
                            // and retry next cycle.
                            const PilotLock fresh =
                                lock_pilots(fft_out, maximum,
                                            std::numeric_limits<int>::max());
                            std::complex<double> verify{};
                            std::size_t verify_count = 0;
                            for (std::size_t pilot =
                                     static_cast<std::size_t>(fresh.phase * 3);
                                 pilot <= maximum; pilot += 12) {
                                const float value = prbs[pilot] == 0U
                                                        ? 4.0F / 3.0F
                                                        : -4.0F / 3.0F;
                                verify +=
                                    static_cast<std::complex<double>>(value) *
                                    std::conj(static_cast<std::complex<double>>(
                                        carrier(fft_out, pilot, maximum,
                                                fresh.offset)));
                                ++verify_count;
                            }
                            const float verified =
                                std::abs(verify) /
                                static_cast<float>(
                                    std::max(verify_count, std::size_t{1}));
                            if (verified >= 0.40F) {
                                lock = fresh;
                                frontend.previous_phase = lock.phase;
                                frontend.carrier_offset = lock.offset;
                                frontend.stable_phase = lock.phase;
                                frontend.stable_carrier_offset = lock.offset;
                                // The TPS superframe sync is stale after a
                                // long fade; re-lock it (BCH frame sync +
                                // symbol index) so the deinterleaver aligns
                                // again. The following ~68 symbols decode on
                                // the fallback index and are rejected by the
                                // Viterbi/RS rather than silently scrambled.
                                frontend.tps_decoder.reset();
                                frontend.tps_snapshot = {};
                            } else {
                                lock = PilotLock{
                                    static_cast<int>(frontend.previous_phase),
                                    frontend.carrier_offset};
                            }
                        } else {
                            lock = PilotLock{
                                static_cast<int>(frontend.previous_phase),
                                frontend.carrier_offset};
                        }
                        if (++fade_symbol_count >= 1400) {
                            // ~2.1 s of continuous fade: request an immediate
                            // fresh acquisition. Re-arms after each attempt so
                            // a long fade keeps probing; a lower threshold
                            // re-locks TPS too often and loses more payload
                            // than the faster recovery gains.
                            fade_symbol_count = 0;
                            const std::scoped_lock guard(mutex);
                            reacquisition_requested = true;
                        }
                        fade_phase_count = (fade_phase_count + 1) % 4;
                    }
                    std::vector<std::complex<float>> channel(maximum + 1);
                    const auto &pilots =
                        pilot_indices[static_cast<std::size_t>(lock.phase)];
                    for (const std::size_t k : pilots) {
                        const float sent =
                            prbs[k] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                        const auto received = carrier(fft_out, k, maximum,
                                                      frontend.carrier_offset);
                        channel[k] =
                            std::norm(received) > minimum_power
                                ? std::complex<float>{sent, 0.0F} / received
                                : std::complex<float>{};
                    }
                    // Fractional timing estimate: an FFT-window shift of tau
                    // samples ramps arg(channel) linearly across carriers
                    // (2*pi*k*tau/N), so the mean phase difference between
                    // adjacent pilots, normalized by their spacing, estimates
                    // tau in samples. Multipath biases the absolute value but
                    // its drift over the capture is the sample-clock offset.
                    if (pilots.size() >= 2) {
                        double pair_sum = 0.0;
                        std::size_t pair_count = 0;
                        for (std::size_t pair = 1; pair < pilots.size();
                             ++pair) {
                            const std::size_t left = pilots[pair - 1];
                            const std::size_t right = pilots[pair];
                            const std::size_t spacing = right - left;
                            if (spacing == 0 || spacing > 64 ||
                                std::norm(channel[left]) == 0.0F ||
                                std::norm(channel[right]) == 0.0F) {
                                continue;
                            }
                            pair_sum += std::arg(channel[right] *
                                                 std::conj(channel[left])) /
                                        static_cast<double>(spacing);
                            ++pair_count;
                        }
                        if (pair_count != 0) {
                            timing_acc +=
                                pair_sum / static_cast<double>(pair_count);
                            ++timing_count;
                        }
                    }
                    for (std::size_t i = 1; i < pilots.size(); ++i) {
                        const std::size_t left = pilots[i - 1];
                        const std::size_t right = pilots[i];
                        for (std::size_t k = left; k <= right; ++k) {
                            const float f = static_cast<float>(k - left) /
                                            static_cast<float>(right - left);
                            channel[k] = channel[left] +
                                         (channel[right] - channel[left]) * f;
                        }
                    }
                    std::fill(channel.begin(),
                              channel.begin() +
                                  static_cast<std::ptrdiff_t>(pilots.front()),
                              channel[pilots.front()]);
                    std::fill(channel.begin() +
                                  static_cast<std::ptrdiff_t>(pilots.back()),
                              channel.end(), channel[pilots.back()]);
                    std::vector<std::complex<float>> tps_values;
                    tps_values.reserve(tps_2k.size() *
                                       (maximum == 6816 ? 4U : 1U));
                    for (std::size_t k = 0; k <= maximum; ++k) {
                        if (listed(tps_2k, k % 1704)) {
                            tps_values.push_back(
                                carrier(fft_out, k, maximum,
                                        frontend.carrier_offset) *
                                channel[k]);
                        }
                    }
                    // TPS state carries continuously across the whole stream
                    // (contiguous symbols), so the differential decoder locks
                    // once and stays locked through re-anchors.
                    frontend.tps_snapshot =
                        frontend.tps_decoder.process(tps_values);
                    const bool matching_tps =
                        frontend.tps_snapshot.locked &&
                        frontend.tps_snapshot.parameters.mode ==
                            frontend.mode &&
                        frontend.tps_snapshot.parameters.guard_interval ==
                            frontend.guard;
                    if (!decoder_parameters && matching_tps &&
                        frontend.tps_snapshot.parameters.hierarchy == 0U) {
                        decoder_parameters = DecoderParameters{
                            frontend.mode,
                            selected_parameters.constellation.value_or(
                                frontend.tps_snapshot.parameters.constellation),
                            selected_parameters.code_rate.value_or(
                                frontend.tps_snapshot.parameters
                                    .high_priority_code_rate),
                            workers.viterbi};
                        if (!start_decoder()) {
                            return;
                        }
                    }
                    std::vector<std::complex<float>> payload;
                    payload.reserve(payload_carrier_count(frontend.mode));
                    std::vector<float> equalizer_power;
                    equalizer_power.reserve(
                        payload_carrier_count(frontend.mode));
                    for (const std::size_t k :
                         payload_indices[static_cast<std::size_t>(
                             lock.phase)]) {
                        payload.push_back(carrier(fft_out, k, maximum,
                                                  frontend.carrier_offset) *
                                          channel[k]);
                        equalizer_power.push_back(std::norm(channel[k]));
                    }
                    if (payload.size() !=
                        payload_carrier_count(frontend.mode)) {
                        next_symbol_start += period;
                        ++symbol_count;
                        ++window_symbol_count;
                        nco_phase = std::remainder(
                            nco_phase + (frontend.tracked_cfo_phase *
                                         static_cast<float>(period)),
                            2.0F * std::numbers::pi_v<float>);
                        if (window_symbol_count >= stats_window_symbols) {
                            publish_stats_window();
                            window_started_at =
                                std::chrono::steady_clock::now();
                            window_symbol_count = 0;
                            mer_sum = 0.0;
                            demap_time_sum = 0.0F;
                            deinterleave_time_sum = 0.0F;
                            depuncture_time_sum = 0.0F;
                            timing_acc = 0.0;
                            timing_count = 0;
                        }
                        continue;
                    }
                    if (postprocessor == nullptr) {
                        pending_symbols.push_back(
                            {.payload = std::move(payload),
                             .equalizer_power = std::move(equalizer_power),
                             .fallback_index =
                                 static_cast<std::size_t>(lock.phase)});
                        if (pending_symbols.size() > 136) {
                            pending_symbols.pop_front();
                        }
                    } else {
                        if (!pending_symbols.empty()) {
                            const std::size_t count = pending_symbols.size();
                            for (std::size_t index = 0; index < count;
                                 ++index) {
                                auto pending =
                                    std::move(pending_symbols.front());
                                pending_symbols.pop_front();
                                const std::size_t distance = count - index;
                                const std::size_t symbol_index =
                                    matching_tps
                                        ? (frontend.tps_snapshot.symbol_index +
                                           68 - (distance % 68)) %
                                              68
                                        : pending.fallback_index;
                                postprocessor->submit(
                                    std::move(pending.payload),
                                    std::move(pending.equalizer_power),
                                    symbol_index);
                            }
                        }
                        const std::size_t symbol_index =
                            matching_tps ? frontend.tps_snapshot.symbol_index
                                         : static_cast<std::size_t>(lock.phase);
                        postprocessor->submit(std::move(payload),
                                              std::move(equalizer_power),
                                              symbol_index);
                        process_batch(postprocessor->take_ready());
                    }
                    ++symbol_count;
                    ++window_symbol_count;
                    nco_phase =
                        std::remainder(nco_phase + (frontend.tracked_cfo_phase *
                                                    static_cast<float>(period)),
                                       2.0F * std::numbers::pi_v<float>);
                    next_symbol_start += period;
                    if (window_symbol_count >= stats_window_symbols) {
                        publish_stats_window();
                        static_cast<void>(
                            enqueue_fec({.kind = FecItem::Kind::stats,
                                         .generation = latest_generation,
                                         .parameters = {},
                                         .mother_metrics = {},
                                         .symbol_index = 0}));
                        window_started_at = std::chrono::steady_clock::now();
                        window_symbol_count = 0;
                        mer_sum = 0.0;
                        demap_time_sum = 0.0F;
                        deinterleave_time_sum = 0.0F;
                        depuncture_time_sum = 0.0F;
                        timing_acc = 0.0;
                        timing_count = 0;
                    }
                }
                // --- end of stream: drain the postprocessor, the gate, and the
                //     FEC decoder, then wait for the next stream ---
                if (postprocessor != nullptr) {
                    process_batch(postprocessor->flush());
                }
                if (!gate_buffer.empty() && decoder_parameters) {
                    double window_mer = 0.0;
                    for (const auto &symbol : gate_buffer) {
                        window_mer += symbol.mer_db;
                    }
                    window_mer /= static_cast<double>(gate_buffer.size());
                    const float floor =
                        fec_floor(decoder_parameters->constellation) + 4.0F;
                    const bool hopeless =
                        static_cast<float>(window_mer) < floor;
                    if (hopeless && !in_hopeless_region) {
                        in_hopeless_region = true;
                    } else if (!hopeless && in_hopeless_region) {
                        in_hopeless_region = false;
                        static_cast<void>(
                            enqueue_fec({.kind = FecItem::Kind::begin,
                                         .generation = latest_generation,
                                         .parameters = *decoder_parameters,
                                         .mother_metrics = {},
                                         .symbol_index = 0}));
                    }
                    if (!hopeless) {
                        for (auto &symbol : gate_buffer) {
                            static_cast<void>(enqueue_fec(
                                {.kind = FecItem::Kind::symbol,
                                 .generation = latest_generation,
                                 .parameters = {},
                                 .mother_metrics =
                                     std::move(symbol.mother_metrics),
                                 .symbol_index = symbol.symbol_index}));
                        }
                    }
                    gate_buffer.clear();
                }
                if (postprocessor != nullptr) {
                    static_cast<void>(
                        enqueue_fec({.kind = FecItem::Kind::end,
                                     .generation = latest_generation,
                                     .parameters = {},
                                     .mother_metrics = {},
                                     .symbol_index = 0}));
                    // A flushed stream that is then resumed starts a fresh
                    // region so the replay's first symbols do not continue a
                    // flushed trellis.
                    if (decoder_parameters) {
                        static_cast<void>(
                            enqueue_fec({.kind = FecItem::Kind::begin,
                                         .generation = latest_generation,
                                         .parameters = *decoder_parameters,
                                         .mother_metrics = {},
                                         .symbol_index = 0}));
                    }
                }
                // Publish a final partial-window stats snapshot.
                if (window_symbol_count != 0 || mer_sum != 0.0) {
                    publish_stats_window();
                    window_started_at = std::chrono::steady_clock::now();
                    window_symbol_count = 0;
                    mer_sum = 0.0;
                    demap_time_sum = 0.0F;
                    deinterleave_time_sum = 0.0F;
                    depuncture_time_sum = 0.0F;
                    timing_acc = 0.0;
                    timing_count = 0;
                }
                {
                    const std::scoped_lock lock(mutex);
                    demod_busy = false;
                }
                idle.notify_all();
            }
        } catch (const std::exception &exception) {
            throw;
        } catch (...) {
            throw;
        }
    }

    // ------------------------------------------------------------------ //
    // FEC worker: consumes the continuous symbol stream through the stateful
    // TransportDecoder (Viterbi pool, RS, TS output). No per-chunk seams:
    // symbols flow straight through, and end/begin items only bracket gated
    // (hopeless) regions and stream boundaries.
    // ------------------------------------------------------------------ //
    void run_fec() {
        std::unique_ptr<Decoder> decoder;
        std::uint64_t decoder_generation = 0;
        float fec_work_ms = 0.0F;
        std::uint64_t window_transport_bytes = 0;
        while (true) {
            FecItem item;
            {
                std::unique_lock lock(mutex);
                fec_ready.wait(
                    lock, [this] { return stopping || !fec_queue.empty(); });
                if (stopping) {
                    return;
                }
                item = std::move(fec_queue.front());
                fec_queue.pop_front();
                fec_worker_busy = true;
            }
            fec_not_full.notify_one();

            if (item.generation == latest_generation) {
                if (item.kind == FecItem::Kind::begin) {
                    if (!decoder || decoder_generation != item.generation ||
                        decoder->parameters() != item.parameters) {
                        decoder = std::make_unique<Decoder>(item.parameters);
                    } else {
                        decoder->reset();
                    }
                    decoder_generation = item.generation;
                } else if (item.kind == FecItem::Kind::symbol && decoder &&
                           decoder_generation == item.generation) {
                    const auto fec_started_at =
                        std::chrono::steady_clock::now();
                    const auto ts =
                        decoder->process_soft_metrics(item.mother_metrics);
                    fec_work_ms += duration_ms(fec_started_at);
                    if (!ts.empty()) {
                        TransportCallback sink;
                        {
                            const std::scoped_lock guard(mutex);
                            sink = callback;
                            window_transport_bytes += ts.size();
                        }
                        if (sink) {
                            sink(ts);
                        }
                    }
                } else if (item.kind == FecItem::Kind::end && decoder &&
                           decoder_generation == item.generation) {
                    const auto fec_started_at =
                        std::chrono::steady_clock::now();
                    const auto ts = decoder->flush();
                    fec_work_ms += duration_ms(fec_started_at);
                    if (!ts.empty()) {
                        TransportCallback sink;
                        {
                            const std::scoped_lock guard(mutex);
                            sink = callback;
                            window_transport_bytes += ts.size();
                        }
                        if (sink) {
                            sink(ts);
                        }
                    }
                    const std::scoped_lock guard(mutex);
                    if (item.generation == latest_generation) {
                        latest.fec_time_ms = fec_work_ms;
                        latest.transport_bytes += window_transport_bytes;
                        latest.transport = decoder->stats();
                        latest.transport_time_ms =
                            decoder->timing().transport_time_ms;
                    }
                    fec_work_ms = 0.0F;
                    window_transport_bytes = 0;
                } else if (item.kind == FecItem::Kind::stats && decoder &&
                           decoder_generation == item.generation) {
                    const std::scoped_lock guard(mutex);
                    if (item.generation == latest_generation) {
                        latest.fec_time_ms = fec_work_ms;
                        latest.transport_bytes += window_transport_bytes;
                        latest.transport = decoder->stats();
                        latest.transport_time_ms =
                            decoder->timing().transport_time_ms;
                    }
                    fec_work_ms = 0.0F;
                    window_transport_bytes = 0;
                }
            }
            {
                const std::scoped_lock guard(mutex);
                fec_worker_busy = false;
            }
            idle.notify_all();
        }
    }
};

StreamDecoder::StreamDecoder() : impl_(std::make_unique<Impl>()) {}
StreamDecoder::~StreamDecoder() noexcept = default;

void StreamDecoder::submit(const std::span<const std::int16_t> interleaved_iq,
                           const std::uint32_t sample_rate_hz,
                           const std::uint32_t channel_bandwidth_hz) {
    if (interleaved_iq.empty() || sample_rate_hz == 0 ||
        (interleaved_iq.size() % 2) != 0) {
        return;
    }
    impl_->analyzer.submit(interleaved_iq, sample_rate_hz,
                           channel_bandwidth_hz);
    const std::scoped_lock lock(impl_->mutex);
    const std::size_t incoming_samples = interleaved_iq.size() / 2;
    impl_->input_queue_capacity_samples =
        std::max(buffered_input_samples(sample_rate_hz), incoming_samples);
    if (impl_->queued_complex_samples + incoming_samples >
        impl_->input_queue_capacity_samples) {
        ++impl_->latest.dropped_blocks;
        return;
    }
    impl_->queue.push_back({std::vector<std::int16_t>(interleaved_iq.begin(),
                                                      interleaved_iq.end()),
                            sample_rate_hz, channel_bandwidth_hz});
    impl_->queued_complex_samples += incoming_samples;
    impl_->input_ready.notify_one();
}

void StreamDecoder::submit_blocking(
    const std::span<const std::int16_t> interleaved_iq,
    const std::uint32_t sample_rate_hz,
    const std::uint32_t channel_bandwidth_hz) {
    if (interleaved_iq.empty() || sample_rate_hz == 0 ||
        (interleaved_iq.size() % 2) != 0) {
        return;
    }
    std::unique_lock lock(impl_->mutex);
    const std::size_t incoming_samples = interleaved_iq.size() / 2;
    impl_->input_queue_capacity_samples =
        std::max(buffered_input_samples(sample_rate_hz), incoming_samples);
    impl_->input_not_full.wait(lock, [this, incoming_samples] {
        return impl_->stopping ||
               impl_->queued_complex_samples + incoming_samples <=
                   impl_->input_queue_capacity_samples;
    });
    if (impl_->stopping) {
        return;
    }
    impl_->queue.push_back({std::vector<std::int16_t>(interleaved_iq.begin(),
                                                      interleaved_iq.end()),
                            sample_rate_hz, channel_bandwidth_hz});
    impl_->queued_complex_samples += incoming_samples;
    impl_->input_ready.notify_one();
}

void StreamDecoder::flush() {
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->flush_requested = true;
    }
    impl_->input_ready.notify_one();
    wait_until_idle();
}

void StreamDecoder::wait_until_idle() {
    std::unique_lock lock(impl_->mutex);
    impl_->idle.wait(lock, [this] {
        return impl_->queue.empty() && !impl_->frontend_busy &&
               !impl_->demod_busy && !impl_->fec_worker_busy &&
               !impl_->flush_requested && !impl_->reset_requested &&
               impl_->fec_queue.empty();
    });
}

void StreamDecoder::reset() {
    impl_->analyzer.reset();
    impl_->cancel_requested = true;
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->reset_requested = true;
    }
    impl_->fec_not_full.notify_all();
    impl_->input_ready.notify_one();
}

void StreamDecoder::set_parameters(const ReceiverParameters &parameters) {
    impl_->analyzer.set_parameters(parameters);
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->parameters = parameters;
    }
    reset();
    wait_until_idle();
}

void StreamDecoder::set_transport_callback(TransportCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->callback = std::move(callback);
}

void StreamDecoder::set_equalized_callback(EqualizedCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->equalized_callback = std::move(callback);
}

StreamDecoderStats StreamDecoder::stats() const {
    const std::scoped_lock lock(impl_->mutex);
    auto statistics = impl_->latest;
    statistics.queued_blocks = impl_->queue.size();
    statistics.queued_input_samples = impl_->queued_complex_samples;
    statistics.input_queue_capacity_samples =
        impl_->input_queue_capacity_samples;
    statistics.queued_symbols = impl_->fec_queue.size();
    statistics.symbol_queue_capacity = impl_->fec_queue_capacity;
    statistics.fec_processing = impl_->fec_worker_busy;
    statistics.processing = impl_->frontend_busy || impl_->demod_busy ||
                            impl_->fec_worker_busy || !impl_->queue.empty() ||
                            !impl_->fec_queue.empty();
    return statistics;
}

DemodulatorStats StreamDecoder::demodulator_stats() const {
    const auto statistics = stats();
    DemodulatorStats result;
    result.locked = statistics.ofdm_locked;
    result.mer_db = statistics.mer_db;
    if (statistics.transport.pre_viterbi_compared_bits != 0) {
        result.ber =
            static_cast<float>(statistics.transport.pre_viterbi_error_bits) /
            static_cast<float>(statistics.transport.pre_viterbi_compared_bits);
    }
    result.worker_threads = statistics.resample_workers +
                            statistics.symbol_workers +
                            statistics.transport.viterbi_workers;
    result.transport_bytes = statistics.transport_bytes;
    result.processing = statistics.processing;
    return result;
}

SignalAnalysisSnapshot StreamDecoder::analysis_snapshot() const {
    return impl_->analyzer.snapshot();
}

void StreamDecoder::set_snr_smoothing(const bool enabled, const int speed) {
    impl_->analyzer.set_snr_smoothing(enabled, speed);
}

} // namespace airspy_tv::dvbt
