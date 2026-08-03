#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/tps_decoder.hpp"

#include <fftw3.h>

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
#include <mutex>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float minimum_power = 1.0e-12F;
constexpr std::size_t acquisition_samples = 350'000;
constexpr std::size_t buffer_duration_denominator = 5;
constexpr std::size_t chunk_overlap_duration_denominator = 10;
constexpr std::size_t initial_symbol_queue_capacity = 256;
constexpr std::size_t ts_packet_size = 188;
constexpr std::size_t retained_ts_packets = 32'768;
constexpr std::size_t minimum_ts_overlap_packets = 32;

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

[[nodiscard]] std::size_t
chunk_overlap_samples(const std::uint32_t sample_rate) noexcept {
    const std::size_t duration_samples =
        (static_cast<std::size_t>(sample_rate) +
         chunk_overlap_duration_denominator - 1) /
        chunk_overlap_duration_denominator;
    return std::min<std::size_t>(StreamDecoder::processing_chunk_samples / 2,
                                 duration_samples);
}

[[nodiscard]] std::uint64_t
packet_hash(const std::span<const std::uint8_t> packet) noexcept {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const std::uint8_t byte : packet) {
        hash = (hash ^ byte) * 1099511628211ULL;
    }
    return hash;
}

[[nodiscard]] std::size_t
find_ts_overlap(const std::span<const std::uint8_t> history,
                const std::span<const std::uint8_t> current,
                const std::size_t expected_packets) {
    if (history.size() % ts_packet_size != 0 ||
        current.size() % ts_packet_size != 0 || history.empty() ||
        current.empty()) {
        return 0;
    }
    const std::size_t history_packets = history.size() / ts_packet_size;
    const std::size_t current_packets = current.size() / ts_packet_size;
    std::vector<std::uint64_t> pattern(current_packets);
    for (std::size_t packet = 0; packet < current_packets; ++packet) {
        pattern[packet] = packet_hash(
            current.subspan(packet * ts_packet_size, ts_packet_size));
    }
    std::vector<std::size_t> prefix(current_packets);
    for (std::size_t index = 1, matched = 0; index < current_packets; ++index) {
        while (matched != 0 && pattern[index] != pattern[matched]) {
            matched = prefix[matched - 1];
        }
        if (pattern[index] == pattern[matched]) {
            ++matched;
        }
        prefix[index] = matched;
    }
    std::size_t matched = 0;
    for (std::size_t packet = 0; packet < history_packets; ++packet) {
        const auto hash = packet_hash(
            history.subspan(packet * ts_packet_size, ts_packet_size));
        while (matched != 0 && hash != pattern[matched]) {
            matched = prefix[matched - 1];
        }
        if (hash == pattern[matched]) {
            ++matched;
        }
        if (matched == current_packets && packet + 1 != history_packets) {
            matched = prefix[matched - 1];
        }
    }
    std::size_t best = 0;
    std::size_t best_distance = std::numeric_limits<std::size_t>::max();
    while (matched >= minimum_ts_overlap_packets) {
        const auto history_suffix = history.last(matched * ts_packet_size);
        const auto current_prefix = current.first(matched * ts_packet_size);
        if (std::ranges::equal(history_suffix, current_prefix)) {
            const std::size_t distance = matched > expected_packets
                                             ? matched - expected_packets
                                             : expected_packets - matched;
            if (distance < best_distance) {
                best = matched;
                best_distance = distance;
            }
        }
        matched = prefix[matched - 1];
    }
    return best;
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

} // namespace

struct StreamDecoder::Impl {
    struct Block {
        std::vector<std::int16_t> samples;
        std::uint32_t rate{};
        std::uint32_t bandwidth{};
    };
    struct ChunkSummary {
        bool ofdm_locked{};
        bool state_carried{};
        bool fec_skipped{};
        bool tps_locked{};
        TpsParameters tps_parameters{};
        int carrier_bin_offset{};
        float mer_db{};
        float residual_carrier_offset_hz{};
        std::uint64_t pilot_phase_discontinuities{};
        std::uint64_t ofdm_symbols{};
        float input_seconds{};
        float overlap_input_seconds{};
        std::size_t input_samples{};
        float resample_time_ms{};
        float acquisition_time_ms{};
        float equalization_time_ms{};
        float demap_time_ms{};
        float deinterleave_time_ms{};
        float depuncture_time_ms{};
        std::size_t resample_workers{};
        std::size_t symbol_workers{};
        std::chrono::steady_clock::time_point started_at{};
    };
    // Continuous front-end tracking state carried across processing chunks.
    // The worker thread owns this; decode_chunk() reads and updates it
    // directly. A chunk whose acquisition agrees with the carried mode/guard
    // resumes tracking instead of cold-starting the CFO loop, carrier search,
    // continual-carrier reference, and TPS superframe from scratch.
    struct FrontendState {
        bool valid{false};
        TransmissionMode mode{TransmissionMode::k8};
        GuardInterval guard{GuardInterval::gi_1_4};
        std::size_t fft_size{};
        std::size_t guard_size{};
        float tracked_cfo_phase{0.0F};
        float residual_phase_ema{0.0F};
        int carrier_offset{std::numeric_limits<int>::max()};
        std::vector<std::complex<float>> previous_continual;
        int previous_phase{-1};
        std::uint64_t phase_discontinuities{0};
        std::size_t consecutive_acquisition_failures{0};
    };
    // NOTE: TPS superframe state is deliberately NOT carried across chunks.
    // The 100 ms overlap re-decodes ~65 symbols, so the first symbol of a
    // chunk is always EARLIER than the previous chunk's last symbol; feeding
    // that non-contiguous sequence to the differential TPS decoder corrupts
    // the frame sync and symbol index. Each chunk re-locks TPS (~68 symbols)
    // and the pending-symbol buffer absorbs the gap losslessly.
    struct FecItem {
        enum class Kind { begin, symbol, end };

        Kind kind{Kind::symbol};
        std::uint64_t generation{};
        DecoderParameters parameters{};
        std::vector<std::uint8_t> mother_metrics;
        std::size_t symbol_index{};
        ChunkSummary summary{};
    };
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::condition_variable idle;
    std::condition_variable input_not_full;
    std::condition_variable fec_ready;
    std::condition_variable fec_not_full;
    std::deque<Block> queue;
    std::deque<FecItem> fec_queue;
    std::size_t queued_complex_samples{};
    std::size_t input_queue_capacity_samples{};
    std::size_t fec_queue_capacity{initial_symbol_queue_capacity};
    std::vector<std::int16_t> accumulated;
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
    bool worker_busy{};
    bool fec_worker_busy{};
    bool overlap_active{};
    std::atomic<std::uint64_t> latest_generation{};
    std::uint32_t accumulated_rate{};
    std::uint32_t accumulated_bandwidth{};
    std::unique_ptr<Cs16Resampler> resampler;
    std::unique_ptr<SymbolPostprocessorPool> symbol_postprocessor;
    std::thread worker;
    std::thread fec_worker;

    Impl() : worker([this] { run(); }), fec_worker([this] { run_fec(); }) {}
    ~Impl() {
        cancel_requested = true;
        {
            const std::scoped_lock lock(mutex);
            stopping = true;
        }
        ready.notify_one();
        input_not_full.notify_all();
        fec_ready.notify_one();
        fec_not_full.notify_all();
        worker.join();
        fec_worker.join();
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
        frontend.consecutive_acquisition_failures = 0;
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

    void decode_chunk(const std::span<const std::int16_t> iq,
                      const std::uint32_t rate, const std::uint32_t bandwidth,
                      const std::size_t new_complex_samples) {
        ReceiverParameters selected_parameters;
        std::uint64_t generation = 0;
        const auto started_at = std::chrono::steady_clock::now();
        {
            const std::scoped_lock guard(mutex);
            selected_parameters = parameters;
            generation = latest_generation;
            if (!selected_parameters.mode.has_value() &&
                stable_mode.has_value()) {
                selected_parameters.mode = stable_mode;
            }
            if (!selected_parameters.guard_interval.has_value() &&
                stable_guard.has_value()) {
                selected_parameters.guard_interval = stable_guard;
            }
        }
        const std::size_t resample_workers =
            selected_parameters.worker_threads == 0
                ? default_viterbi_worker_count()
                : selected_parameters.worker_threads;
        if (!resampler || resampler->worker_count() != resample_workers) {
            resampler = std::make_unique<Cs16Resampler>(resample_workers);
        }
        auto samples = resampler->process(iq, rate, bandwidth);
        const auto resampled_at = std::chrono::steady_clock::now();
        if (cancel_requested || samples.size() < acquisition_samples) {
            return;
        }
        const OfdmAcquisition acquisition = acquire_ofdm(
            std::span(samples).first(acquisition_samples), selected_parameters);
        const auto acquired_at = std::chrono::steady_clock::now();
        if (cancel_requested || acquisition.score < 0.20F) {
            return;
        }
        {
            const std::scoped_lock guard(mutex);
            stable_mode = acquisition.mode;
            stable_guard = acquisition.guard;
            latest.acquisition_score = acquisition.score;
            latest.fft_size = static_cast<std::uint32_t>(acquisition.fft_size);
            latest.guard_size =
                static_cast<std::uint32_t>(acquisition.guard_size);
        }
        const bool carried = frontend.valid &&
                             frontend.mode == acquisition.mode &&
                             frontend.guard == acquisition.guard;
        frontend.mode = acquisition.mode;
        frontend.guard = acquisition.guard;
        frontend.fft_size = acquisition.fft_size;
        frontend.guard_size = acquisition.guard_size;
        frontend.consecutive_acquisition_failures = 0;
        frontend.valid = true;
        if (!carried) {
            // Cold start: seed the continuous tracking state from this
            // chunk's acquisition estimate instead of resuming carried state.
            frontend.tracked_cfo_phase =
                std::arg(acquisition.phase) /
                static_cast<float>(acquisition.fft_size);
            frontend.residual_phase_ema = 0.0F;
            frontend.carrier_offset = std::numeric_limits<int>::max();
            frontend.previous_continual.clear();
            frontend.previous_phase = -1;
        }
        const std::size_t maximum =
            acquisition.mode == TransmissionMode::k8 ? 6816 : 1704;
        const std::span<const int> continual = continual_2k;
        const std::span<const int> tps = tps_2k;
        std::optional<DecoderParameters> decoder_parameters;
        if (selected_parameters.constellation.has_value() &&
            selected_parameters.code_rate.has_value()) {
            decoder_parameters = DecoderParameters{
                acquisition.mode, *selected_parameters.constellation,
                *selected_parameters.code_rate,
                allocate_workers(selected_parameters.worker_threads).viterbi};
        }
        const std::size_t symbol_queue_capacity = buffered_symbol_count(
            bandwidth, acquisition.fft_size + acquisition.guard_size);
        {
            const std::scoped_lock guard(mutex);
            fec_queue_capacity = symbol_queue_capacity;
        }
        std::vector<std::complex<float>> fft_in(acquisition.fft_size);
        std::vector<std::complex<float>> fft_out(acquisition.fft_size);
        fftwf_plan plan =
            fftwf_plan_dft_1d(static_cast<int>(acquisition.fft_size),
                              reinterpret_cast<fftwf_complex *>(fft_in.data()),
                              reinterpret_cast<fftwf_complex *>(fft_out.data()),
                              FFTW_FORWARD, FFTW_ESTIMATE);
        if (plan == nullptr) {
            return;
        }
        float &tracked_cfo_phase = frontend.tracked_cfo_phase;
        const std::size_t period =
            acquisition.fft_size + acquisition.guard_size;
        std::vector<std::size_t> continual_indices;
        std::array<std::vector<std::size_t>, 4> pilot_indices;
        std::array<std::vector<std::size_t>, 4> payload_indices;
        continual_indices.reserve(continual_2k.size() *
                                  (maximum == 6816 ? 4U : 1U));
        for (std::size_t k = 0; k <= maximum; ++k) {
            const std::size_t base = k % 1704;
            const bool continual_carrier = listed(continual, base);
            const bool tps_carrier = listed(tps, base);
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
        int &carrier_offset = frontend.carrier_offset;
        int &previous_phase = frontend.previous_phase;
        std::uint64_t &phase_discontinuities = frontend.phase_discontinuities;
        double mer_sum = 0.0;
        float demap_time_sum = 0.0F;
        float deinterleave_time_sum = 0.0F;
        float depuncture_time_sum = 0.0F;
        const WorkerAllocation workers =
            allocate_workers(selected_parameters.worker_threads);
        SymbolPostprocessorPool *postprocessor = nullptr;
        const auto start_decoder = [&]() {
            if (!decoder_parameters || postprocessor) {
                return true;
            }
            if (!enqueue_fec({.kind = FecItem::Kind::begin,
                              .generation = generation,
                              .parameters = *decoder_parameters,
                              .mother_metrics = {},
                              .symbol_index = 0,
                              .summary = {}})) {
                return false;
            }
            if (!symbol_postprocessor ||
                !symbol_postprocessor->compatible(
                    workers.symbol, acquisition.mode,
                    decoder_parameters->constellation,
                    decoder_parameters->code_rate, symbol_queue_capacity)) {
                symbol_postprocessor =
                    std::make_unique<SymbolPostprocessorPool>(
                        workers.symbol, acquisition.mode,
                        decoder_parameters->constellation,
                        decoder_parameters->code_rate, symbol_queue_capacity);
            } else {
                // A cancelled chunk can leave completed symbols that belong to
                // its generation. Drain and discard them before reusing the
                // pool so no stale result crosses a chunk boundary.
                static_cast<void>(symbol_postprocessor->flush());
            }
            postprocessor = symbol_postprocessor.get();
            return true;
        };
        if (decoder_parameters && !start_decoder()) {
            return;
        }
        // MER gate: equalized symbols whose mean falls far below the
        // constellation's decode floor cannot be FEC-decoded (deep multipath
        // fades). Symbols are buffered until the whole chunk's MER is known,
        // so a chunk whose head is faded but whose tail recovers is decoded
        // normally, while a hopeless chunk discards its symbols and spares
        // the Viterbi from grinding them. Front-end tracking is unaffected.
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
        std::vector<PostprocessedSymbol> chunk_symbols;
        chunk_symbols.reserve(1024);
        const auto emit_postprocessed =
            [this, &mer_sum, &demap_time_sum, &deinterleave_time_sum,
             &depuncture_time_sum,
             &chunk_symbols](std::vector<PostprocessedSymbol> symbols) {
                for (auto &symbol : symbols) {
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
                    chunk_symbols.push_back(std::move(symbol));
                }
                return true;
            };
        // CP correlation locates the beginning of the guard interval.  The
        // FFT window must start at the useful symbol, after that guard.  Using
        // acquisition.start directly applies a large cyclic time shift and a
        // carrier phase ramp that sparse-pilot interpolation cannot unwrap.
        const std::size_t first_fft_start =
            acquisition.start + acquisition.guard_size;
        float nco_phase =
            tracked_cfo_phase * static_cast<float>(first_fft_start);
        std::vector<std::complex<float>> &previous_continual =
            frontend.previous_continual;
        // TPS state is per-chunk: the overlap re-decodes earlier symbols at
        // every chunk boundary, which would corrupt a carried differential
        // TPS decoder (see the FrontendState comment).
        TpsDecoder tps_decoder;
        TpsSnapshot tps_snapshot;
        struct PendingSymbol {
            std::vector<std::complex<float>> payload;
            std::vector<float> equalizer_power;
            std::size_t fallback_index{};
        };
        std::deque<PendingSymbol> pending_symbols;
        float &residual_phase_ema = frontend.residual_phase_ema;
        std::uint64_t symbol_count = 0;
        const std::uint64_t phase_discontinuity_base =
            frontend.phase_discontinuities;
        std::size_t previous_symbol_start =
            std::numeric_limits<std::size_t>::max();
        for (std::size_t start = first_fft_start;
             start + acquisition.fft_size <= samples.size() &&
             !cancel_requested;
             start += period) {
            std::complex<float> nco = std::polar(1.0F, -nco_phase);
            const std::complex<float> nco_step =
                std::polar(1.0F, -tracked_cfo_phase);
            for (std::size_t i = 0; i < acquisition.fft_size; ++i) {
                fft_in[i] = samples[start + i] * nco;
                nco *= nco_step;
                if ((i & 511U) == 511U) {
                    nco *= 1.0F / std::sqrt(std::norm(nco));
                }
            }
            fftwf_execute(plan);
            const PilotLock lock =
                lock_pilots(fft_out, maximum, carrier_offset);
            if (previous_phase >= 0 && lock.phase != (previous_phase + 1) % 4) {
                ++phase_discontinuities;
            }
            previous_phase = lock.phase;
            carrier_offset = lock.offset;
            std::vector<std::complex<float>> current_continual;
            current_continual.reserve(continual_indices.size());
            for (const std::size_t k : continual_indices) {
                current_continual.push_back(
                    carrier(fft_out, k, maximum, carrier_offset));
            }
            float residual_phase = 0.0F;
            // Only update the CFO loop from a contiguous symbol pair. The
            // first symbol of a chunk follows the previous chunk's last symbol
            // by the overlap (~65 symbols earlier), so its temporal
            // correlation would measure 65x the true residual and overshoot;
            // the carried frequency is already converged, so skip the update.
            if (previous_continual.size() == current_continual.size() &&
                start == previous_symbol_start + period) {
                std::complex<float> temporal_correlation{};
                for (std::size_t i = 0; i < current_continual.size(); ++i) {
                    temporal_correlation +=
                        current_continual[i] * std::conj(previous_continual[i]);
                }
                residual_phase = std::arg(temporal_correlation);
                constexpr float loop_gain = 0.20F;
                tracked_cfo_phase +=
                    loop_gain * residual_phase / static_cast<float>(period);
                residual_phase_ema =
                    (0.1F * residual_phase) + (0.9F * residual_phase_ema);
            }
            previous_symbol_start = start;
            previous_continual = std::move(current_continual);
            std::vector<std::complex<float>> channel(maximum + 1);
            const auto &pilots =
                pilot_indices[static_cast<std::size_t>(lock.phase)];
            for (const std::size_t k : pilots) {
                const float sent = prbs[k] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                const auto received =
                    carrier(fft_out, k, maximum, carrier_offset);
                channel[k] = std::norm(received) > minimum_power
                                 ? std::complex<float>{sent, 0.0F} / received
                                 : std::complex<float>{};
            }
            for (std::size_t i = 1; i < pilots.size(); ++i) {
                const std::size_t left = pilots[i - 1];
                const std::size_t right = pilots[i];
                for (std::size_t k = left; k <= right; ++k) {
                    const float f = static_cast<float>(k - left) /
                                    static_cast<float>(right - left);
                    channel[k] =
                        channel[left] + (channel[right] - channel[left]) * f;
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
            tps_values.reserve(tps.size() * (maximum == 6816 ? 4U : 1U));
            for (std::size_t k = 0; k <= maximum; ++k) {
                if (listed(tps, k % 1704)) {
                    tps_values.push_back(
                        carrier(fft_out, k, maximum, carrier_offset) *
                        channel[k]);
                }
            }
            tps_snapshot = tps_decoder.process(tps_values);
            const bool matching_tps =
                tps_snapshot.locked &&
                tps_snapshot.parameters.mode == acquisition.mode &&
                tps_snapshot.parameters.guard_interval == acquisition.guard;
            if (!decoder_parameters && matching_tps &&
                tps_snapshot.parameters.hierarchy == 0U) {
                decoder_parameters = DecoderParameters{
                    acquisition.mode,
                    selected_parameters.constellation.value_or(
                        tps_snapshot.parameters.constellation),
                    selected_parameters.code_rate.value_or(
                        tps_snapshot.parameters.high_priority_code_rate),
                    workers.viterbi};
                if (!start_decoder()) {
                    break;
                }
            }
            std::vector<std::complex<float>> payload;
            payload.reserve(payload_carrier_count(acquisition.mode));
            std::vector<float> equalizer_power;
            equalizer_power.reserve(payload_carrier_count(acquisition.mode));
            for (const std::size_t k :
                 payload_indices[static_cast<std::size_t>(lock.phase)]) {
                payload.push_back(carrier(fft_out, k, maximum, carrier_offset) *
                                  channel[k]);
                equalizer_power.push_back(std::norm(channel[k]));
            }
            if (payload.size() != payload_carrier_count(acquisition.mode)) {
                continue;
            }
            if (!postprocessor) {
                pending_symbols.push_back(
                    {.payload = std::move(payload),
                     .equalizer_power = std::move(equalizer_power),
                     .fallback_index = static_cast<std::size_t>(lock.phase)});
                if (pending_symbols.size() > 136) {
                    pending_symbols.pop_front();
                }
            } else {
                if (!pending_symbols.empty()) {
                    const std::size_t count = pending_symbols.size();
                    for (std::size_t index = 0; index < count; ++index) {
                        auto pending = std::move(pending_symbols.front());
                        pending_symbols.pop_front();
                        const std::size_t distance = count - index;
                        const std::size_t symbol_index =
                            matching_tps ? (tps_snapshot.symbol_index + 68 -
                                            (distance % 68)) %
                                               68
                                         : pending.fallback_index;
                        postprocessor->submit(
                            std::move(pending.payload),
                            std::move(pending.equalizer_power), symbol_index);
                    }
                }
                const std::size_t symbol_index =
                    matching_tps ? tps_snapshot.symbol_index
                                 : static_cast<std::size_t>(lock.phase);
                postprocessor->submit(std::move(payload),
                                      std::move(equalizer_power), symbol_index);
                if (!emit_postprocessed(postprocessor->take_ready())) {
                    break;
                }
            }
            ++symbol_count;
            nco_phase = std::remainder(
                nco_phase + (tracked_cfo_phase * static_cast<float>(period)),
                2.0F * std::numbers::pi_v<float>);
        }
        if (!cancel_requested && postprocessor &&
            !emit_postprocessed(postprocessor->flush())) {
            return;
        }
        // Decide the MER gate from the chunk's symbol quality, then enqueue
        // the buffered symbols (or discard them as hopeless). A faded-head /
        // recovered-tail chunk must still decode, so the gate skips the FEC
        // only when even the chunk's best symbols fall below the floor: a
        // uniformly hopeless chunk is spared the Viterbi grind entirely.
        bool fec_skipped = false;
        if (!cancel_requested && !chunk_symbols.empty() && decoder_parameters) {
            const float floor = fec_floor(decoder_parameters->constellation);
            std::vector<float> mers;
            mers.reserve(chunk_symbols.size());
            for (const auto &symbol : chunk_symbols) {
                mers.push_back(symbol.mer_db);
            }
            const std::size_t top = std::max<std::size_t>(1, mers.size() / 10);
            std::ranges::nth_element(
                mers,
                mers.begin() + static_cast<std::ptrdiff_t>(mers.size() - top));
            double best = 0.0;
            for (std::size_t index = mers.size() - top; index < mers.size();
                 ++index) {
                best += mers[index];
            }
            best /= static_cast<double>(top);
            fec_skipped = static_cast<float>(best) < floor + 4.0F;
            if (!fec_skipped) {
                for (auto &symbol : chunk_symbols) {
                    if (!enqueue_fec(
                            {.kind = FecItem::Kind::symbol,
                             .generation = generation,
                             .parameters = {},
                             .mother_metrics = std::move(symbol.mother_metrics),
                             .symbol_index = symbol.symbol_index,
                             .summary = {}})) {
                        break;
                    }
                }
            }
            chunk_symbols.clear();
        }
        fftwf_destroy_plan(plan);
        const auto equalized_at = std::chrono::steady_clock::now();
        if (cancel_requested) {
            return;
        }
        const ChunkSummary summary{
            .ofdm_locked = symbol_count != 0,
            .state_carried = carried,
            .fec_skipped = fec_skipped,
            .tps_locked = tps_snapshot.locked,
            .tps_parameters = tps_snapshot.parameters,
            .carrier_bin_offset = carrier_offset,
            .mer_db = symbol_count == 0
                          ? 0.0F
                          : static_cast<float>(
                                mer_sum / static_cast<double>(symbol_count)),
            .residual_carrier_offset_hz =
                residual_phase_ema *
                (static_cast<float>(bandwidth) * (8.0F / 7.0F)) /
                (2.0F * std::numbers::pi_v<float> * static_cast<float>(period)),
            .pilot_phase_discontinuities =
                phase_discontinuities - phase_discontinuity_base,
            .ofdm_symbols = symbol_count,
            .input_seconds = static_cast<float>(new_complex_samples) /
                             static_cast<float>(rate),
            .overlap_input_seconds =
                static_cast<float>((iq.size() / 2) - new_complex_samples) /
                static_cast<float>(rate),
            .input_samples = new_complex_samples,
            .resample_time_ms = std::chrono::duration<float, std::milli>(
                                    resampled_at - started_at)
                                    .count(),
            .acquisition_time_ms = std::chrono::duration<float, std::milli>(
                                       acquired_at - resampled_at)
                                       .count(),
            .equalization_time_ms = std::chrono::duration<float, std::milli>(
                                        equalized_at - acquired_at)
                                        .count(),
            .demap_time_ms = demap_time_sum,
            .deinterleave_time_ms = deinterleave_time_sum,
            .depuncture_time_ms = depuncture_time_sum,
            .resample_workers = resample_workers,
            .symbol_workers = workers.symbol,
            .started_at = started_at,
        };
        if (postprocessor) {
            static_cast<void>(enqueue_fec({.kind = FecItem::Kind::end,
                                           .generation = generation,
                                           .parameters = {},
                                           .mother_metrics = {},
                                           .symbol_index = 0,
                                           .summary = summary}));
        }
    }

    void run_fec() {
        std::unique_ptr<Decoder> decoder;
        std::uint64_t decoder_generation = 0;
        std::uint64_t byte_count = 0;
        float fec_work_ms = 0.0F;
        std::vector<std::uint8_t> chunk_transport;
        std::vector<std::uint8_t> transport_history;
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
                    const bool generation_changed =
                        decoder_generation != item.generation;
                    if (!decoder || decoder_generation != item.generation ||
                        decoder->parameters() != item.parameters) {
                        decoder = std::make_unique<Decoder>(item.parameters);
                    } else {
                        decoder->reset();
                    }
                    decoder_generation = item.generation;
                    byte_count = 0;
                    fec_work_ms = 0.0F;
                    chunk_transport.clear();
                    if (generation_changed) {
                        transport_history.clear();
                    }
                } else if (item.kind == FecItem::Kind::symbol && decoder &&
                           decoder_generation == item.generation) {
                    const auto fec_started_at =
                        std::chrono::steady_clock::now();
                    const auto ts =
                        decoder->process_soft_metrics(item.mother_metrics);
                    fec_work_ms +=
                        std::chrono::duration<float, std::milli>(
                            std::chrono::steady_clock::now() - fec_started_at)
                            .count();
                    if (!ts.empty() && item.generation == latest_generation) {
                        chunk_transport.insert(chunk_transport.end(),
                                               ts.begin(), ts.end());
                    }
                } else if (item.kind == FecItem::Kind::end && decoder &&
                           decoder_generation == item.generation) {
                    const auto fec_started_at =
                        std::chrono::steady_clock::now();
                    const auto ts = decoder->flush();
                    fec_work_ms +=
                        std::chrono::duration<float, std::milli>(
                            std::chrono::steady_clock::now() - fec_started_at)
                            .count();
                    if (!ts.empty() && item.generation == latest_generation) {
                        chunk_transport.insert(chunk_transport.end(),
                                               ts.begin(), ts.end());
                    }
                    const float chunk_seconds =
                        item.summary.input_seconds +
                        item.summary.overlap_input_seconds;
                    const std::size_t expected_overlap_packets =
                        chunk_seconds <= 0.0F
                            ? 0
                            : static_cast<std::size_t>(
                                  static_cast<float>(chunk_transport.size() /
                                                     ts_packet_size) *
                                  item.summary.overlap_input_seconds /
                                  chunk_seconds);
                    const std::size_t overlap_packets =
                        find_ts_overlap(transport_history, chunk_transport,
                                        expected_overlap_packets);
                    const bool join_failed =
                        !transport_history.empty() && overlap_packets == 0;
                    const auto emitted =
                        std::span<const std::uint8_t>{chunk_transport}.subspan(
                            overlap_packets * ts_packet_size);
                    TransportCallback sink;
                    {
                        const std::scoped_lock guard(mutex);
                        sink = callback;
                    }
                    if (sink && !emitted.empty()) {
                        sink(emitted);
                    }
                    byte_count = emitted.size();
                    if (chunk_transport.size() >=
                        retained_ts_packets * ts_packet_size) {
                        transport_history.assign(
                            chunk_transport.end() -
                                static_cast<std::ptrdiff_t>(
                                    retained_ts_packets * ts_packet_size),
                            chunk_transport.end());
                    } else {
                        transport_history.insert(transport_history.end(),
                                                 emitted.begin(),
                                                 emitted.end());
                        if (transport_history.size() >
                            retained_ts_packets * ts_packet_size) {
                            transport_history.erase(
                                transport_history.begin(),
                                transport_history.end() -
                                    static_cast<std::ptrdiff_t>(
                                        retained_ts_packets * ts_packet_size));
                        }
                    }
                    const float wall_seconds =
                        std::chrono::duration<float>(
                            std::chrono::steady_clock::now() -
                            item.summary.started_at)
                            .count();
                    const std::scoped_lock guard(mutex);
                    if (item.generation == latest_generation) {
                        latest.ofdm_locked = item.summary.ofdm_locked;
                        latest.tps_locked = item.summary.tps_locked;
                        latest.tps_constellation =
                            item.summary.tps_parameters.constellation;
                        latest.tps_code_rate =
                            item.summary.tps_parameters.high_priority_code_rate;
                        latest.tps_guard_interval =
                            item.summary.tps_parameters.guard_interval;
                        latest.tps_mode = item.summary.tps_parameters.mode;
                        latest.tps_hierarchy =
                            item.summary.tps_parameters.hierarchy;
                        latest.carrier_bin_offset =
                            item.summary.carrier_bin_offset;
                        latest.mer_db = item.summary.mer_db;
                        latest.residual_carrier_offset_hz =
                            item.summary.residual_carrier_offset_hz;
                        latest.pilot_phase_discontinuities +=
                            item.summary.pilot_phase_discontinuities;
                        ++latest.processed_chunks;
                        latest.processed_input_samples +=
                            item.summary.input_samples;
                        latest.ofdm_symbols += item.summary.ofdm_symbols;
                        latest.transport_bytes += byte_count;
                        latest.ts_overlap_packets += overlap_packets;
                        latest.ts_overlap_join_failures +=
                            join_failed ? 1U : 0U;
                        latest.transport = decoder->stats();
                        latest.processing_realtime_ratio =
                            wall_seconds / item.summary.input_seconds;
                        latest.resample_time_ms = item.summary.resample_time_ms;
                        latest.acquisition_time_ms =
                            item.summary.acquisition_time_ms;
                        latest.equalization_time_ms =
                            item.summary.equalization_time_ms;
                        latest.fec_time_ms = fec_work_ms;
                        latest.symbol_workers = item.summary.symbol_workers;
                        latest.demap_time_ms = item.summary.demap_time_ms;
                        latest.deinterleave_time_ms =
                            item.summary.deinterleave_time_ms;
                        latest.depuncture_time_ms =
                            item.summary.depuncture_time_ms;
                        latest.transport_time_ms =
                            decoder->timing().transport_time_ms;
                        latest.resample_workers = item.summary.resample_workers;
                        latest.state_carried = item.summary.state_carried;
                        latest.fec_skipped = item.summary.fec_skipped;
                    }
                }
            }
            {
                const std::scoped_lock guard(mutex);
                fec_worker_busy = false;
            }
            idle.notify_all();
        }
    }

    void run() {
        while (true) {
            Block block;
            {
                std::unique_lock lock(mutex);
                ready.wait(lock, [this] {
                    return stopping || reset_requested || flush_requested ||
                           !queue.empty();
                });
                if (stopping) {
                    return;
                }
                if (reset_requested) {
                    queue.clear();
                    fec_queue.clear();
                    queued_complex_samples = 0;
                    accumulated.clear();
                    overlap_active = false;
                    stable_mode.reset();
                    stable_guard.reset();
                    reset_frontend_state();
                    latest = {};
                    ++latest_generation;
                    reset_requested = false;
                    flush_requested = false;
                    cancel_requested = false;
                    input_not_full.notify_all();
                    fec_not_full.notify_all();
                }
                if (queue.empty()) {
                    if (flush_requested) {
                        flush_requested = false;
                        worker_busy = true;
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
                    worker_busy = true;
                }
            }
            if (!block.samples.empty()) {
                if (!accumulated.empty() &&
                    (accumulated_rate != block.rate ||
                     accumulated_bandwidth != block.bandwidth)) {
                    accumulated.clear();
                    overlap_active = false;
                    reset_frontend_state();
                }
                accumulated_rate = block.rate;
                accumulated_bandwidth = block.bandwidth;
                accumulated.insert(accumulated.end(), block.samples.begin(),
                                   block.samples.end());
            }
            const std::size_t scalar_chunk = processing_chunk_samples * 2;
            const std::size_t overlap = chunk_overlap_samples(accumulated_rate);
            const std::size_t scalar_step =
                (processing_chunk_samples - overlap) * 2;
            while (accumulated.size() >= scalar_chunk) {
                decode_chunk(std::span(accumulated).first(scalar_chunk),
                             accumulated_rate, accumulated_bandwidth,
                             overlap_active ? processing_chunk_samples - overlap
                                            : processing_chunk_samples);
                overlap_active = true;
                accumulated.erase(accumulated.begin(),
                                  accumulated.begin() +
                                      static_cast<std::ptrdiff_t>(scalar_step));
            }
            const std::size_t accumulated_complex = accumulated.size() / 2;
            const std::size_t pending_complex =
                overlap_active && accumulated_complex >= overlap
                    ? accumulated_complex - overlap
                    : accumulated_complex;
            if (block.samples.empty() && pending_complex != 0) {
                decode_chunk(accumulated, accumulated_rate,
                             accumulated_bandwidth, pending_complex);
                accumulated.clear();
                overlap_active = false;
            }
            {
                const std::scoped_lock guard(mutex);
                worker_busy = false;
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
    impl_->ready.notify_one();
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
    impl_->ready.notify_one();
}

void StreamDecoder::flush() {
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->flush_requested = true;
    }
    impl_->ready.notify_one();
    wait_until_idle();
}

void StreamDecoder::wait_until_idle() {
    std::unique_lock lock(impl_->mutex);
    impl_->idle.wait(lock, [this] {
        return impl_->queue.empty() && !impl_->worker_busy &&
               !impl_->flush_requested && !impl_->reset_requested &&
               impl_->fec_queue.empty() && !impl_->fec_worker_busy;
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
    impl_->ready.notify_one();
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
    statistics.processing = impl_->worker_busy || impl_->fec_worker_busy ||
                            !impl_->queue.empty() || !impl_->fec_queue.empty();
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
