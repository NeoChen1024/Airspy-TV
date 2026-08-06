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

// Event-driven debug dump: when AIRSPYTV_EVENT_DEBUG is set, key pipeline
// transitions (fade enter/exit, TPS lock/unlock, carrier drift, re-anchor,
// acquisition) print their internal state to stderr immediately so a 10 s
// polling diag does not miss the instant a transient event damages the grid.
[[nodiscard]] inline bool event_debug_enabled() {
    static const bool enabled =
        std::getenv("AIRSPYTV_EVENT_DEBUG") != nullptr;
    return enabled;
}
// Resampled-sample ring shared by the front-end thread (producer) and the
// demod thread (consumer). The ring is sized at run time from the input rate
// to retain ~0.2 s of baseband (the baseband rate is always <= the input
// rate), floored at the acquisition window plus a couple of symbols of
// lookahead and scheduling jitter — the proven 1 Mi behaviour for low input
// rates.
constexpr std::size_t ring_minimum_samples = 1'048'576;

[[nodiscard]] std::size_t
ring_capacity_for(const std::uint32_t sample_rate_hz) noexcept {
    return std::max(ring_minimum_samples,
                    (static_cast<std::size_t>(sample_rate_hz) +
                     buffer_duration_denominator - 1) /
                        buffer_duration_denominator);
}
// Demod statistics window: 400 OFDM symbols (~0.6 s at 8K/guard-1/4), the
// report cadence for the CLI (the demod no longer reports once per submitted
// input chunk).
constexpr std::size_t stats_window_symbols = 400;
// MER gate window: one TPS superframe (68 symbols). Windows whose mean MER
// falls below the constellation's decode floor are dropped and bracket a
// fresh FEC trellis at the region edges.
constexpr std::size_t gate_window_symbols = 68;
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

constexpr std::size_t timing_pilot_spacing = 12;
constexpr std::size_t timing_filter_history_size = 7;
constexpr double timing_outlier_limit_samples = 24.0;
// A one-sample FFT-window correction produces essentially a one-sample change
// in the unwrapped pilot-slope coordinate. Keep this explicit because the
// response is used both to de-bias the drift history and to convert the drift
// estimate back into the integer window-step accumulator.
constexpr double timing_window_shift_response = 1.0;

[[nodiscard]] std::optional<double>
estimate_scattered_timing_tau(
    const std::span<const std::complex<float>> channel,
    const std::size_t phase, const std::size_t maximum,
    const std::size_t fft_size) {
    // Use only the scattered pilots here.  `pilot_indices` also contains
    // continual carriers, whose non-uniform spacing gives each phase pair a
    // different unwrap period.  The scattered-pilot grid has one fixed
    // spacing, so all observations share the same N/12-sample ambiguity.
    std::array<double, 1024> estimates{};
    std::size_t estimate_count = 0;
    const std::size_t first = phase * 3;
    for (std::size_t left = first;
         left + timing_pilot_spacing <= maximum;
         left += timing_pilot_spacing) {
        const std::size_t right = left + timing_pilot_spacing;
        if (std::norm(channel[left]) <= minimum_power ||
            std::norm(channel[right]) <= minimum_power ||
            estimate_count == estimates.size()) {
            continue;
        }
        // `channel` is already normalized as sent / received below, so the
        // known +/- scattered-pilot polarity has already been removed.  Do
        // not multiply by the pilot signs again: doing so reintroduces a pi
        // jump for every polarity transition and turns it into a false
        // timing branch, which then poisons the phase verifier.
        const double slope = std::arg(channel[right] *
                                      std::conj(channel[left])) /
                             static_cast<double>(timing_pilot_spacing);
        if (std::isfinite(slope)) {
            estimates[estimate_count++] =
                slope * static_cast<double>(fft_size) /
                (2.0 * std::numbers::pi_v<double>);
        }
    }
    if (estimate_count == 0) {
        return std::nullopt;
    }
    std::ranges::sort(estimates.begin(),
                      estimates.begin() +
                          static_cast<std::ptrdiff_t>(estimate_count));
    const std::size_t middle = estimate_count / 2;
    if (estimate_count % 2 != 0) {
        return estimates[middle];
    }
    return 0.5 * (estimates[middle - 1] + estimates[middle]);
}

// The pilot phase slope is periodic in N/12 samples.  Keep its absolute
// branch continuous and reject isolated group-delay clicks before they reach
// either the long-term sample-clock loop or pilot phase verification.  The
// latter is particularly sensitive: one sample of ramp error is already
// several radians at the edge of the 8K carrier grid.
struct TimingSlopeTracker {
    void reset(const std::size_t fft_size = 0) noexcept {
        ambiguity_period = fft_size == 0
                                ? 0.0
                                : static_cast<double>(fft_size) /
                                      static_cast<double>(timing_pilot_spacing);
        history.fill(0.0);
        history_head = 0;
        accepted_count = 0;
        filtered_tau = 0.0;
        initialized = false;
    }

    [[nodiscard]] std::optional<double> observe(
        const double measured_tau) noexcept {
        if (!std::isfinite(measured_tau) || ambiguity_period <= 0.0) {
            return std::nullopt;
        }
        if (!initialized) {
            // Seed the whole short history with the first valid observation so
            // median_history() never has to sort a partially initialized
            // buffer.
            history.fill(measured_tau);
            history_head = 1 % history.size();
            filtered_tau = measured_tau;
            initialized = true;
            accepted_count = 1;
            return filtered_tau;
        }
        double candidate = measured_tau;
        candidate +=
            ambiguity_period *
            std::round((filtered_tau - candidate) / ambiguity_period);
        const double center = median_history();
        if (std::abs(candidate - center) > timing_outlier_limit_samples) {
            // A sample-clock drift cannot move the FFT boundary by dozens of
            // samples in a handful of OFDM symbols. A persistent jump here
            // is therefore a phase-slope ambiguity or a multipath outlier,
            // not a new timing branch to adopt. Keeping the last valid slope
            // is safe because the closed loop below prevents the true offset
            // from approaching this ambiguity in the first place. Adopting
            // the old branch after a few confirmations was what turned the
            // capture's 282 -> 405 -> 680 sequence into a phase storm.
            return std::nullopt;
        }
        history[history_head] = candidate;
        history_head = (history_head + 1) % history.size();
        const double robust_tau = median_history();
        filtered_tau = 0.25 * robust_tau + 0.75 * filtered_tau;
        ++accepted_count;
        return filtered_tau;
    }

    [[nodiscard]] std::optional<double> filtered() const noexcept {
        // A few accepted symbols make the shared verify ramp independent of
        // the first noisy pilot observation after acquisition.
        return accepted_count >= 4 && initialized
                   ? std::optional<double>{filtered_tau}
                   : std::nullopt;
    }

  private:
    [[nodiscard]] double median_history() const noexcept {
        std::array<double, timing_filter_history_size> sorted = history;
        std::ranges::sort(sorted);
        const std::size_t middle = sorted.size() / 2;
        if (sorted.size() % 2 != 0) {
            return sorted[middle];
        }
        return 0.5 * (sorted[middle - 1] + sorted[middle]);
    }

    double ambiguity_period{};
    std::array<double, timing_filter_history_size> history{};
    std::size_t history_head{};
    std::size_t accepted_count{};
    double filtered_tau{};
    bool initialized{};
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

// Phase-only re-lock at a fixed carrier offset, used while the fade
// indicator is marginal (0.25 < fi <= 0.5): the grid is still up but the
// channel is degraded enough that a full offset search is noise-driven and
// can latch a multipath alias (545's 0 -> 3, 557's 0 -> -1). Re-verify only
// the mod-4 scattered-pilot phase at the frozen offset; the offset itself is
// re-searched only when fi > 0.5 (healthy) or after a fade.
[[nodiscard]] int lock_phase_at_offset(
    const std::span<const std::complex<float>> fft, const std::size_t maximum,
    const int offset, const std::optional<double> timing_tau) {
    int best_phase = 0;
    float best_score = -1.0F;
    for (int phase = 0; phase < 4; ++phase) {
        // Window-offset phase ramp: an FFT-window shift of tau samples
        // rotates each carrier by exp(j 2 pi k tau / N), so a naive
        // correlation against the nominal pilot phases collapses after only
        // a few samples of window offset — the 545 phase-jump storms
        // (hundreds of jumps per second while fi reads 1.0, near every
        // ~38000-symbol badlock). Estimate the per-carrier phase ramp from
        // the adjacent-pilot phase differences (de-rotating the pilot sign
        // sequence), then verify with the ramp removed. Once the shared
        // timing tracker has four good observations, use its unwrapped and
        // robustly filtered channel slope instead of measuring the ramp again
        // in this phase decision. That prevents one multipath click from
        // changing both the timing loop and the phase lock at once.
        std::complex<float> correlation{};
        float score = 0.0F;
        std::size_t chunk_count = 0;
        // `timing_tau` is measured from sent / received channel estimates, so
        // it has the opposite sign of the raw FFT carrier ramp.  The positive
        // sign below therefore de-rotates the received pilots.  Before the
        // shared tracker is ready, retain the old local estimate as a cold
        // start fallback.
        double dephase_slope = 0.0;
        if (timing_tau.has_value()) {
            dephase_slope =
                2.0 * std::numbers::pi_v<double> * *timing_tau /
                static_cast<double>(fft.size());
        } else {
            double ramp_sum = 0.0;
            std::size_t ramp_count = 0;
            std::size_t previous_pilot =
                std::numeric_limits<std::size_t>::max();
            for (std::size_t pilot = static_cast<std::size_t>(phase * 3);
                 pilot <= maximum; pilot += 12) {
                if (previous_pilot != std::numeric_limits<std::size_t>::max()) {
                    const auto left =
                        carrier(fft, previous_pilot, maximum, offset);
                    const auto right = carrier(fft, pilot, maximum, offset);
                    const float left_value = prbs[previous_pilot] == 0U
                                                 ? 4.0F / 3.0F
                                                 : -4.0F / 3.0F;
                    const float right_value = prbs[pilot] == 0U
                                                  ? 4.0F / 3.0F
                                                  : -4.0F / 3.0F;
                    if (std::norm(left) > 0.0F &&
                        std::norm(right) > 0.0F) {
                        const double difference = std::arg(
                            right * std::conj(left) *
                            std::complex<float>(right_value * left_value,
                                                0.0F));
                        if (std::isfinite(difference)) {
                            ramp_sum += difference;
                            ++ramp_count;
                        }
                    }
                }
                previous_pilot = pilot;
            }
            // The local estimate comes from the received FFT values, while
            // the shared timing estimate comes from sent / received channel
            // values and therefore has the opposite sign.
            dephase_slope =
                ramp_count != 0
                    ? -ramp_sum /
                          static_cast<double>(ramp_count *
                                              timing_pilot_spacing)
                    : 0.0;
        }
        for (std::size_t pilot = static_cast<std::size_t>(phase * 3);
             pilot <= maximum; pilot += 12) {
            const float value =
                prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
            const float dephase = static_cast<float>(dephase_slope *
                                                     static_cast<double>(pilot));
            correlation +=
                value * std::conj(std::polar(1.0F, dephase) *
                                  carrier(fft, pilot, maximum, offset));
            if (++chunk_count == 8) {
                score += std::norm(correlation);
                correlation = {};
                chunk_count = 0;
            }
        }
        score += std::norm(correlation);
        if (score > best_score) {
            best_score = score;
            best_phase = phase;
        }
    }
    return best_phase;
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
    float preprocess_time_ms{};
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
        const auto preprocess_started_at = std::chrono::steady_clock::now();
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
                .preprocess_time_ms =
                    std::chrono::duration<float, std::milli>(
                        demap_started_at - preprocess_started_at)
                        .count(),
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
        // CIR / delay-spread estimation for adaptive FFT-window placement.
        // Updated once per TPS frame (68 symbols) from the scattered pilots,
        // so the added work is negligible; `cir_offset` is the smoothed FFT
        // window offset (samples, <= 0) relative to the effective symbol
        // start (start_pos + guard_size), sliding the window toward the
        // latest strong tap when the delay spread leaves guard margin.
        std::vector<std::complex<float>> cir_grid;
        std::vector<std::complex<float>> cir_response;
        fftwf_plan cir_plan{nullptr};
        std::size_t cir_n{0};
        float cir_offset{0.0F};
        int cir_symbol_count{0};
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
    // The most recent block's bandwidth, published by the front-end; the demod
    // reads it when it runs its event-driven acquisitions (the resampled rate
    // = bandwidth * 8/7 feeds the sync + realtime stats).
    std::uint32_t current_bandwidth{};
    TransportCallback callback;
    DiscontinuityCallback discontinuity_callback;
    // Deferred by handle_sync_change (which runs with the mutex held) and
    // fired by the caller once the lock is released, so a user callback is
    // never invoked under an internal lock.
    std::optional<TransportDiscontinuity> pending_discontinuity;
    EqualizedCallback equalized_callback;
    ReceiverParameters parameters;
    dvbt::SignalAnalyzer analyzer;
    std::optional<TransmissionMode> stable_mode;
    std::optional<GuardInterval> stable_guard;
    FrontendState frontend;
    StreamDecoderStats latest;
    std::atomic<bool> cancel_requested{};
    // Where each pipeline thread is parked, for diagnostics (see
    // WorkerState). Written by the owning thread, read lock-free by stats().
    std::atomic<int> frontend_state{static_cast<int>(WorkerState::idle)};
    std::atomic<int> demod_state{static_cast<int>(WorkerState::idle)};
    std::atomic<int> fec_state{static_cast<int>(WorkerState::idle)};
    // Demod symbol-processing wall time accumulated over the current stats
    // window (written by the demod thread, consumed by publish_stats_window
    // on the same thread).
    double demod_busy_time_sum_ms{};
    bool stopping{};
    bool reset_requested{};
    bool flush_requested{};
    bool frontend_busy{};
    bool demod_busy{};
    // True while the demod is at the stream head approaching a first-anchor
    // acquisition (including the wait for acquisition data): the front-end's
    // push-abandon keys off this in addition to !demod_busy so it never fires
    // in the window between the head wait waking and the acquisition marking
    // itself busy (a ring full of fresh data with a flush arriving mid-push
    // used to abandon the push there, dropping the first block's remainder
    // and starving small files). It is cleared when the demod parks in the
    // retry back-off, the end-of-stream wait, or the drain.
    bool acquisition_pending{};
    bool fec_worker_busy{};
    std::atomic<std::uint64_t> latest_generation{};
    std::unique_ptr<SymbolPostprocessorPool> symbol_postprocessor;
    std::thread frontend_thread;
    std::thread demod_thread;
    std::thread fec_thread;

    Impl()
        : ring(ring_minimum_samples),
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
        // The fade-recovery carrier grid is per-stream: a grid captured from
        // one frequency or file must never be restored in a later stream.
        frontend.stable_carrier_offset = std::numeric_limits<int>::max();
        frontend.stable_phase = -1;
    }

    void fire_discontinuity(const TransportDiscontinuity discontinuity) {
        DiscontinuityCallback sink;
        {
            const std::scoped_lock lock(mutex);
            sink = discontinuity_callback;
        }
        if (sink) {
            sink(discontinuity);
        }
    }

    void fire_pending_discontinuity() {
        std::optional<TransportDiscontinuity> pending;
        {
            const std::scoped_lock lock(mutex);
            pending = pending_discontinuity;
            pending_discontinuity.reset();
        }
        if (pending.has_value()) {
            fire_discontinuity(*pending);
        }
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
        while (true) {
            Block block;
            bool close_ring = false;
            {
                std::unique_lock lock(mutex);
                frontend_state.store(
                    static_cast<int>(WorkerState::waiting_input));
                input_ready.wait(lock, [this] {
                    return stopping || reset_requested || flush_requested ||
                           !queue.empty();
                });
                if (stopping) {
                    frontend_state.store(static_cast<int>(WorkerState::exited));
                    return;
                }
                if (reset_requested) {
                    // Abandon the whole pipeline. The demod sees the invalid
                    // sync and drops its per-stream state; the FEC worker
                    // drops items whose generation is stale.
                    reset_requested = false;
                    cancel_requested = false;
                    frontend_state.store(
                        static_cast<int>(WorkerState::processing));
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
                    current_bandwidth = 0;
                    demod_busy_time_sum_ms = 0.0;
                    resampler = StreamingResampler{};
                    reset_frontend_state();
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
                {
                    const std::scoped_lock lock(mutex);
                    // Size the ring for ~0.2 s of this block's rate. Only
                    // resize while the ring is empty: the read/write
                    // positions are absolute counters wrapped by the size,
                    // so a mid-stream resize would corrupt the wrap.
                    if (ring_read_pos == ring_write_pos &&
                        ring.size() != ring_capacity_for(block.rate)) {
                        ring.resize(ring_capacity_for(block.rate));
                    }
                }
                if (resampler.configured() &&
                    (resampler.rate() != block.rate ||
                     resampler.bandwidth() != block.bandwidth)) {
                    // Retune: drop the filter state and invalidate the sync;
                    // the demod re-acquires itself on the new rate.
                    resampler = StreamingResampler{};
                    {
                        const std::scoped_lock lock(mutex);
                        current_bandwidth = block.bandwidth;
                        sync.valid = false;
                        ++sync.version;
                        // A retune is a receiver reset: the TPS-fixed
                        // parameters and the fade-recovery grid captured from
                        // the previous frequency must not carry over.
                        stable_mode.reset();
                        stable_guard.reset();
                        frontend.stable_carrier_offset =
                            std::numeric_limits<int>::max();
                        frontend.stable_phase = -1;
                    }
                    ring_data.notify_all();
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
                // Push to the ring incrementally: the ring (sized to ~0.2 s
                // of the input rate) holds no more than a block, so each
                // iteration pushes only what fits and waits for the demod to
                // free space. A
                // flush or reset arriving mid-push abandons the push instead
                // of blocking forever behind a demod that is not consuming
                // (e.g. a stream whose acquisition can never succeed): the
                // flush then closes the ring and the demod drains what is
                // there.
                std::size_t pushed = 0;
                while (pushed < resample_buffer.size()) {
                    std::unique_lock lock(mutex);
                    frontend_state.store(
                        static_cast<int>(WorkerState::waiting_ring_space));
                    ring_space.wait(lock, [this] {
                        return stopping || reset_requested || flush_requested ||
                               ring_write_pos - ring_read_pos < ring.size();
                    });
                    if (stopping) {
                        return;
                    }
                    // Abandon the push only when the demod is genuinely not
                    // consuming (the ring is full AND it is not busy — the
                    // acquisition-retry stall): then the flush/reset would
                    // otherwise wait forever behind the push. If the demod is
                    // keeping up (it frees ring space as it decodes), finish
                    // the push so no submitted data is dropped at the end of
                    // a stream.
                    if (reset_requested ||
                        (flush_requested && !demod_busy &&
                         !acquisition_pending &&
                         ring_write_pos - ring_read_pos >= ring.size())) {
                        break;
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
                    latest.last_resample_block_time_ms =
                        duration_ms(resample_started_at);
                    current_bandwidth = block.bandwidth;
                }
            }
            {
                const std::scoped_lock lock(mutex);
                frontend_busy = false;
            }
            idle.notify_all();
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
            std::vector<std::size_t> tps_indices;
            std::vector<std::complex<float>> tps_values;
            std::array<std::vector<std::size_t>, 4> pilot_indices;
            std::array<std::vector<std::size_t>, 4> payload_indices;
            std::optional<DecoderParameters> decoder_parameters;
            WorkerAllocation workers{1, 1};
            SymbolPostprocessorPool *postprocessor = nullptr;
            struct PendingSymbol {
                std::vector<std::complex<float>> payload;
                std::vector<float> equalizer_power;
            };
            std::deque<PendingSymbol> pending_symbols;
            std::deque<PostprocessedSymbol> gate_buffer;
            bool in_hopeless_region = false;
            bool last_reanchor_carried = false;
            std::uint64_t seen_sync_version = 0;
            bool have_grid = false;
            std::uint64_t next_symbol_start = 0;
            float nco_phase = 0.0F;
            std::uint64_t symbol_count = 0;
            std::uint64_t frozen_symbol_count = 0;
            // Windows whose MER is below the decode floor, back to back.
            // Unlike fade_indicator (the continual-carrier temporal
            // correlation, which stays high while the signal is present
            // even if the carrier grid has drifted off), the MER floor
            // detects a genuinely undecodable stream, so sustained
            // hopelessness forces a re-acquisition even when fi reads
            // healthy (the 581 tail: MER stuck at ~7.8 dB, fi=0.9996,
            // never entering the fade branch).
            std::uint64_t hopeless_window_count = 0;
            // Confirmation state for the once-only stable-grid capture: the
            // first healthy lock is not trusted until the same offset has
            // held for a few consecutive symbols, so a marginal start cannot
            // pin the pre-fade reference (which a re-anchor restores) onto a
            // multipath alias.
            int stable_pending_offset = std::numeric_limits<int>::max();
            int stable_pending_count = 0;
            // Confirmation state for per-symbol offset drift: a single
            // lock_pilots result is not trusted — a degraded-but-not-yet-faded
            // channel (MER collapses before the continual-carrier correlation
            // does) makes the +/-2 offset search noise-driven and can latch a
            // multipath alias (545's 0 -> 3, 557's 0 -> -1) in one symbol.
            // Event-driven mode-change detection: a persistent TPS-locked
            // mismatch (a station switch without a fade) re-runs the
            // acquisition so the grid rebuilds for the new mode.
            std::uint64_t tps_mismatch_symbols = 0;
            int lock_hold = 0;
            std::uint64_t window_symbol_count = 0;
            double mer_sum = 0.0;
            float preprocess_time_sum = 0.0F;
            float demap_time_sum = 0.0F;
            float deinterleave_time_sum = 0.0F;
            float depuncture_time_sum = 0.0F;
            double timing_acc = 0.0;
            std::uint64_t timing_count = 0;
            double latest_raw_timing = 0.0;
            std::uint64_t timing_raw_count = 0;
            std::uint64_t timing_rejected_count = 0;
            TimingSlopeTracker timing_tracker;
            // Continual-carrier fade indicator for the current symbol
            // (persisted here so publish_stats_window, defined before the
            // symbol loop, can surface it).
            float fade_indicator = 1.0F;
            // Wall-clock start of the current symbol's processing segment
            // (FFT through payload), for the CPU-load estimate.
            std::chrono::steady_clock::time_point demod_busy_started_at{};
            // Closed-loop sample-clock tracking: the windowed mean tau is
            // dominated by the channel's mean group delay (multipath) plus a
            // slow sample-clock drift, so a P-loop on the absolute value
            // would chase the channel. Track only the slow DRIFT between
            // consecutive stats windows (the sample-clock offset), heavily
            // filtered (tau ~ 50 windows), and accumulate it into a
            // fractional timing that nudges the symbol period by +/-1 sample
            // when it crosses +/-0.5.
            double last_windowed_timing = 0.0;
            double smoothed_sample_clock_ppm = 0.0;
            double fractional_timing = 0.0;
            // Physical timing and nominal sample position for the robust
            // first-difference SRO estimate. Sample positions make the slope
            // independent of full versus partial statistics-window length.
            static constexpr std::size_t tau_history_n = 24;
            static constexpr std::size_t tau_history_min = 16;
            std::array<double, tau_history_n> tau_history{};
            std::array<double, tau_history_n> tau_sample_history{};
            std::size_t tau_history_head = 0;
            std::size_t tau_history_count = 0;
            double timing_elapsed_samples = 0.0;
            // Cumulative FFT-window displacement applied by the period steps
            // (each +/-1-sample step on next_symbol_start), retained for
            // diagnostics. The publish loop adds the calibrated response
            // back when estimating the physical drift.
            double accumulated_window_shift = 0.0;
            // Window-averaged CIR offset of the previous stats window: the
            // drift estimate is computed in window-position-invariant
            // coordinates so the adaptive FFT-window slides do not read as
            // sample-clock steps.
            double last_windowed_cir_avg = 0.0;
            // Baseline used to remove the timing loop's own integer steps
            // from the measured sample-clock drift.
            double last_timing_window_shift = 0.0;
            // Independent baseline for actuator-rate telemetry. It advances
            // even when no timing measurement is accepted; the control
            // baseline above deliberately waits for valid timing.
            double last_telemetry_window_shift = 0.0;
            double window_cir_offset_sum = 0.0;
            // Rolling actuator rate is diagnostics only. Keep the raw
            // per-window pulse rate too, since it proves integer-step timing.
            static constexpr std::size_t shift_rate_history_n = 64;
            std::array<double, shift_rate_history_n> shift_rate_steps{};
            std::array<double, shift_rate_history_n> shift_rate_samples{};
            std::size_t shift_rate_history_head = 0;
            std::size_t shift_rate_history_count = 0;
            double rolling_shift_steps = 0.0;
            double rolling_shift_samples = 0.0;
            // Integer CIR window offset currently applied to
            // `next_symbol_start`; the per-frame estimate only nudges the
            // anchor by the difference, so mid-stream updates move the FFT
            // window by a few samples at most instead of re-anchoring.
            int applied_cir_offset = 0;
            double cir_confidence = 0.0;
            float acquisition_time_ms = 0.0F;
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
                tps_indices.clear();
                pilot_indices = {};
                payload_indices = {};
                for (std::size_t k = 0; k <= maximum; ++k) {
                    const std::size_t base = k % 1704;
                    const bool continual_carrier = listed(continual_2k, base);
                    const bool tps_carrier = listed(tps_2k, base);
                    if (continual_carrier) {
                        continual_indices.push_back(k);
                    }
                    if (tps_carrier) {
                        tps_indices.push_back(k);
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
                tps_values.resize(tps_indices.size());
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
                // CIR estimation grid: one tap per scattered-pilot slot (12
                // sub-carriers), so the impulse response spans Tu/12 and tap i
                // is delayed i * fft_size / (12 * N) samples after the window.
                frontend.cir_n = fft_size == 8192 ? 1024 : 256;
                frontend.cir_grid.assign(frontend.cir_n,
                                         std::complex<float>{});
                frontend.cir_response.assign(frontend.cir_n,
                                             std::complex<float>{});
                if (frontend.cir_plan != nullptr) {
                    fftwf_destroy_plan(frontend.cir_plan);
                    frontend.cir_plan = nullptr;
                }
                frontend.cir_plan = fftwf_plan_dft_1d(
                    static_cast<int>(frontend.cir_n),
                    reinterpret_cast<fftwf_complex *>(
                        frontend.cir_grid.data()),
                    reinterpret_cast<fftwf_complex *>(
                        frontend.cir_response.data()),
                    FFTW_BACKWARD, FFTW_ESTIMATE);
            };

            std::size_t symbol_queue_capacity = initial_symbol_queue_capacity;
            const auto reset_timing_state =
                [&](const std::size_t tracker_fft_size,
                    const bool reset_window_cir) {
                    fractional_timing = 0.0;
                    smoothed_sample_clock_ppm = 0.0;
                    accumulated_window_shift = 0.0;
                    last_windowed_timing = 0.0;
                    last_windowed_cir_avg = 0.0;
                    last_timing_window_shift = 0.0;
                    last_telemetry_window_shift = 0.0;
                    tau_history.fill(0.0);
                    tau_sample_history.fill(0.0);
                    tau_history_head = 0;
                    tau_history_count = 0;
                    timing_elapsed_samples = 0.0;
                    if (reset_window_cir) {
                        window_cir_offset_sum = 0.0;
                    }
                    shift_rate_steps.fill(0.0);
                    shift_rate_samples.fill(0.0);
                    shift_rate_history_head = 0;
                    shift_rate_history_count = 0;
                    rolling_shift_steps = 0.0;
                    rolling_shift_samples = 0.0;
                    cir_confidence = 0.0;
                    timing_tracker.reset(tracker_fft_size);
                };
            // Handles a sync version change (first anchor, re-anchor, or
            // reset). Returns true when the demod must act (grid (re)built or
            // abandoned). Called with the mutex held.
            const auto handle_sync_change = [&]() {
                seen_sync_version = sync.version;
                // Effective symbol start (acquisition boundary + guard) plus
                // the adaptive CIR window offset (<= 0 samples, sliding the
                // FFT window toward the latest strong tap).
                const auto anchored_start = [&]() -> std::uint64_t {
                    return static_cast<std::uint64_t>(
                        static_cast<std::int64_t>(sync.start_pos +
                                                  sync.guard_size) +
                        static_cast<std::int64_t>(
                            std::lround(frontend.cir_offset)));
                };
                if (!sync.valid) {
                    // Reset: abandon the stream. The FEC generation has already
                    // advanced, so any stale queued items are dropped by the
                    // worker. A reset means a retune, a source switch, or a
                    // dropped-block recovery: the content may have changed
                    // entirely, so playback must restart rather than
                    // concatenate.
                    pending_discontinuity = TransportDiscontinuity::retune;
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
                    frontend.cir_offset = 0.0F;
                    frontend.cir_symbol_count = 0;
                    applied_cir_offset = 0;
                    reset_timing_state(0, true);
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
                    next_symbol_start = anchored_start();
                    nco_phase = frontend.tracked_cfo_phase *
                                static_cast<float>(next_symbol_start);
                    last_reanchor_carried = false;
                    applied_cir_offset = static_cast<int>(
                        std::lround(frontend.cir_offset));
                    // The timing loop's reference grid changed: its residual
                    // fraction and drift state have no meaning against the
                    // new boundary.
                    reset_timing_state(fft_size, true);
                    have_grid = true;
                    demod_busy = true;
                    return true;
                }
                // Same mode/guard: is the new boundary on the current grid?
                const std::uint64_t new_start = anchored_start();
                const std::uint64_t delta = next_symbol_start > new_start
                                                ? next_symbol_start - new_start
                                                : new_start - next_symbol_start;
                const std::uint64_t aligned_distance =
                    std::min(delta % period, period - delta % period);
                if (aligned_distance < 64) {
                    return false; // aligned: keep the grid and the tracking
                }
                // Re-anchor: the boundary moved (signal drop/recovery or a
                // first anchor estimate error). Tracking carries when the
                // mode/guard are unchanged. Never read past the samples the
                // ring has freed.
                const bool carried = frontend.valid;
                std::uint64_t new_next = new_start;
                while (new_next < ring_read_pos) {
                    new_next += period;
                }
                nco_phase = std::remainder(
                    nco_phase +
                        frontend.tracked_cfo_phase *
                            static_cast<float>(
                                static_cast<std::int64_t>(new_next) -
                                static_cast<std::int64_t>(next_symbol_start)),
                    2.0F * std::numbers::pi_v<float>);
                next_symbol_start = new_next;
                applied_cir_offset = static_cast<int>(
                    std::lround(frontend.cir_offset));
                last_reanchor_carried = carried;
                stable_pending_offset = std::numeric_limits<int>::max();
                stable_pending_count = 0;
                // The boundary moved: the timing loop restarts against the
                // newly anchored grid.
                reset_timing_state(fft_size, false);
                return true;
            };

            // Event-driven acquisition, owned by the demod (the front-end
            // no longer acquires at all). Runs on the ring's window
            // [ring_read_pos, ring_read_pos + acquisition_samples): read-side
            // positions are deterministic, and the window is AHEAD of the
            // demod's consumption, so the ring never frees it. Publishes into
            // the shared sync so handle_sync_change can act (first anchor:
            // grid build + cold seed; re-anchor: carried tracking + boundary
            // move; mode change: full rebuild).
            const auto run_event_acquisition = [&](bool wait_for_data = false)
                -> float {
                if (wait_for_data) {
                    // Re-anchor in a live stream: the ring is drained as fast
                    // as the frontend fills it (realtime decode keeps no
                    // backlog), so a snapshot sees little or nothing. Block
                    // (bounded) until a full acquisition window accumulates —
                    // that was the 545 failure: 309 re-anchor attempts, all
                    // score=0.000. A partial window (< 10 symbol periods) is
                    // just as useless: acquire_ofdm's periodic phase scan
                    // requires count >= 10 (10+ symbol periods) before it
                    // reports any non-zero score, so a 2-symbol snapshot
                    // still scores 0.000. While recovering there is no valid
                    // TS to lose, so the wait is free.
                    const auto wait_started =
                        std::chrono::steady_clock::now();
                    for (;;) {
                        bool cancel = false;
                        std::uint64_t now_available = 0;
                        {
                            const std::scoped_lock lock(mutex);
                            cancel = stopping || reset_requested ||
                                     cancel_requested;
                            now_available =
                                ring_write_pos - ring_read_pos;
                        }
                        if (cancel || duration_ms(wait_started) > 1000.0F) {
                            if (event_debug_enabled() && !cancel) {
                                std::fprintf(
                                    stderr,
                                    "[evt] acq wait-timeout avail=%llu\n",
                                    static_cast<unsigned long long>(
                                        now_available));
                            }
                            return 0.0F;
                        }
                        if (now_available >= acquisition_samples) {
                            break;
                        }
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(4));
                    }
                }
                std::vector<std::complex<float>> window;
                ReceiverParameters acquisition_parameters;
                std::uint64_t base = 0;
                std::uint64_t acquisition_generation = 0;
                {
                    const std::scoped_lock lock(mutex);
                    const std::uint64_t available =
                        ring_write_pos - ring_read_pos;
                    // Adaptive window: acquire on everything available when
                    // the stream is shorter than one acquisition window (the
                    // first block of a small fixture resamples to less than
                    // acquisition_samples). The CP correlation needs at most
                    // a couple of symbol periods.
                    const std::uint64_t window_size =
                        std::min<std::uint64_t>(available, acquisition_samples);
                    if (window_size < 2 * 10240) {
                        return 0.0F;
                    }
                    base = ring_read_pos;
                    acquisition_generation = latest_generation;
                    window.reserve(static_cast<std::size_t>(window_size));
                    for (std::uint64_t p = base; p < base + window_size; ++p) {
                        window.push_back(ring[p % ring.size()]);
                    }
                    acquisition_parameters = parameters;
                }
                const auto acquisition_started_at =
                    std::chrono::steady_clock::now();
                const OfdmAcquisition acquisition =
                    acquire_ofdm(std::span(window), acquisition_parameters);
                const float acquisition_elapsed_ms =
                    duration_ms(acquisition_started_at);
                if (acquisition.score < 0.20F) {
                    const std::scoped_lock lock(mutex);
                    if (acquisition_generation == latest_generation &&
                        !reset_requested) {
                        acquisition_time_ms = acquisition_elapsed_ms;
                    }
                    return 0.0F; // no signal: keep the current state and retry
                }
                std::uint32_t bandwidth{};
                float resampled_rate{};
                {
                    const std::scoped_lock lock(mutex);
                    // Acquisition runs outside the decoder mutex. A reset or
                    // bandwidth change may therefore invalidate the copied
                    // window while acquire_ofdm() is running. Never publish a
                    // stale result after the reset generation has advanced;
                    // doing so can resurrect the old grid and leave the GUI
                    // waiting forever for the reset to become idle.
                    if (stopping || reset_requested || cancel_requested ||
                        acquisition_generation != latest_generation) {
                        return 0.0F;
                    }
                    bandwidth = current_bandwidth;
                    resampled_rate =
                        static_cast<float>(bandwidth) * (8.0F / 7.0F);
                    acquisition_time_ms = acquisition_elapsed_ms;
                    stable_mode = acquisition.mode;
                    stable_guard = acquisition.guard;
                    sync.valid = true;
                    sync.start_pos = base + acquisition.start;
                    sync.phase = acquisition.phase;
                    sync.score = acquisition.score;
                    sync.mode = acquisition.mode;
                    sync.guard = acquisition.guard;
                    sync.fft_size = acquisition.fft_size;
                    sync.guard_size = acquisition.guard_size;
                    sync.bandwidth = bandwidth;
                    sync.resampled_rate = resampled_rate;
                    latest.acquisition_score = acquisition.score;
                    if (event_debug_enabled()) {
                        std::fprintf(
                            stderr,
                            "[evt] acq ok score=%.3f start=%llu mode=%d g=%d\n",
                            acquisition.score,
                            static_cast<unsigned long long>(acquisition.start),
                            static_cast<int>(acquisition.mode),
                            static_cast<int>(acquisition.guard));
                    }
                    ++sync.version;
                    static_cast<void>(handle_sync_change());
                }
                return acquisition.score;
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
                    preprocess_time_sum += symbol.preprocess_time_ms;
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
                    // Bound the gate buffer while the FEC is not running:
                    // symbols keep arriving when decoder_parameters is unset
                    // (no TPS lock yet, or after a reset), and an unbounded
                    // gate would grow ~100 KB per symbol until memory is
                    // exhausted — the "TS stops and the machine wedges"
                    // failure. With the FEC active the 68-symbol window loop
                    // below already drains the gate, so no cap is needed.
                    while (gate_buffer.size() > gate_window_symbols * 4) {
                        gate_buffer.pop_front();
                    }
                    return;
                }
                // Gate floor: the decision-directed MSE the MER is derived
                // from saturates near the constellation's quantization noise
                // (ideal 64-QAM reads ~21 dB, a healthy 64-QAM signal ~18
                // dB), so a generous +4 dB margin sat on top of exactly the
                // healthy operating point and gated healthy windows on MER
                // ripple — a live 64-QAM mux at a steady ~18 dB drained the
                // TS queue to zero over tens of seconds as half its windows
                // were dropped. The floor sits well below the constellation's
                // decode threshold instead (theory minus 6 dB: 64-QAM 12 dB,
                // 16-QAM 8 dB, QPSK 3 dB), so only windows that are genuinely
                // undecodable are gated and a healthy signal at its operating
                // point can never be dropped at the boundary.
                const float floor =
                    fec_floor(decoder_parameters->constellation) - 2.0F;
                while (gate_buffer.size() >= gate_window_symbols) {
                    double window_mer = 0.0;
                    for (std::size_t i = 0; i < gate_window_symbols; ++i) {
                        window_mer += gate_buffer[i].mer_db;
                    }
                    window_mer /= static_cast<double>(gate_window_symbols);
                    const bool hopeless =
                        static_cast<float>(window_mer) < floor;
                    // Back-to-back hopeless windows, counted at the window
                    // cadence (one per 68 symbols). A single bad window is
                    // ripple; sustained hopelessness is what forces the
                    // re-acquisition path even when fi reads healthy.
                    hopeless_window_count =
                        hopeless ? hopeless_window_count + 1 : 0;
                    if (event_debug_enabled() && hopeless) {
                        std::fprintf(stderr,
                                     "[evt] hopeless mer=%.2f floor=%.2f "
                                     "count=%llu sym=%llu\n",
                                     static_cast<float>(window_mer), floor,
                                     static_cast<unsigned long long>(
                                         hopeless_window_count),
                                     static_cast<unsigned long long>(
                                         symbol_count));
                    }
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
                const double window_sample_count =
                    static_cast<double>(window_symbols) *
                    static_cast<double>(period);
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
                const double window_cir_avg =
                    window_symbols == 0
                        ? static_cast<double>(applied_cir_offset)
                        : window_cir_offset_sum /
                              static_cast<double>(window_symbols);
                double observed_drift = 0.0;
                double corrected_drift = 0.0;
                double window_shift = 0.0;
                const double telemetry_window_shift =
                    accumulated_window_shift - last_telemetry_window_shift;
                last_telemetry_window_shift = accumulated_window_shift;
                if (shift_rate_history_count == shift_rate_history_n) {
                    rolling_shift_steps -=
                        shift_rate_steps[shift_rate_history_head];
                    rolling_shift_samples -=
                        shift_rate_samples[shift_rate_history_head];
                } else {
                    ++shift_rate_history_count;
                }
                shift_rate_steps[shift_rate_history_head] =
                    telemetry_window_shift;
                shift_rate_samples[shift_rate_history_head] =
                    window_sample_count;
                rolling_shift_steps += telemetry_window_shift;
                rolling_shift_samples += window_sample_count;
                shift_rate_history_head =
                    (shift_rate_history_head + 1) % shift_rate_history_n;
                const double rolling_shift_rate_ppm =
                    rolling_shift_samples > 0.0
                        ? rolling_shift_steps * 1.0e6 /
                              rolling_shift_samples
                        : 0.0;
                const double timing_sample_position =
                    timing_elapsed_samples + 0.5 * window_sample_count;
                timing_elapsed_samples += window_sample_count;
                double smoothed_timing_drift =
                    smoothed_sample_clock_ppm * window_sample_count / 1.0e6;
                if (timing_count != 0) {
                    // Closed-loop sample-clock tracking: the windowed mean
                    // tau is dominated by the channel's mean group delay
                    // (multipath), so a P-loop on the absolute value would
                    // chase the channel (a constant ~+35 samples at the 581
                    // capture). Track only the slow drift between consecutive
                    // windows — the sample-clock offset — with a long time
                    // constant, and accumulate it into the fractional timing
                    // the symbol advance consumes. Bounded so a pathological
                    // estimate cannot walk the window far off grid. The CIR
                    // window slides shift tau by exactly the slide, so both
                    // references are expressed relative to the window's
                    // average position, keeping the drift estimate blind to
                    // the adaptive placement. (A window slide of d samples
                    // moves the measured timing offset by -d, so the offset
                    // is rebased onto the window's average position by
                    // adding the slide back.)
                    observed_drift =
                        (static_cast<double>(timing_offset) +
                         window_cir_avg) -
                        (last_windowed_timing + last_windowed_cir_avg);
                    last_windowed_timing = static_cast<double>(timing_offset);
                    last_windowed_cir_avg = window_cir_avg;
                    window_shift =
                        accumulated_window_shift -
                        last_timing_window_shift;
                    last_timing_window_shift = accumulated_window_shift;
                    // A positive window step makes the measured pilot slope
                    // move negative by only the channel-dependent response
                    // fraction. Add that known response back so the loop
                    // estimates physical sample-clock drift instead of
                    // cancelling its own correction in the measurement.
                    corrected_drift =
                        observed_drift +
                        timing_window_shift_response * window_shift;
                    // The per-window drift is corrected for the loop's own
                    // period steps above: a step moves the measured tau by
                    // approximately one sample, so treating that step as
                    // physical drift would bias the compensation —
                    // the tau sawtooth (ramping 0 -> 62 samples until the
                    // pilot verify collapses, one badlock per ~38000
                    // symbols). Keep a physical-coordinate history and use a
                    // median first difference instead: a handful of outliers
                    // cannot move the estimate away from true clock drift.
                    // Store the timing coordinate after undoing the known
                    // response of all integer window corrections. The median
                    // first-difference estimator below must see physical
                    // sample-clock drift, not the loop's sawtooth response.
                    tau_history[tau_history_head] =
                        static_cast<double>(timing_offset) + window_cir_avg +
                        timing_window_shift_response *
                            accumulated_window_shift;
                    tau_sample_history[tau_history_head] =
                        timing_sample_position;
                    tau_history_head = (tau_history_head + 1) % tau_history_n;
                    if (tau_history_count < tau_history_n) {
                        ++tau_history_count;
                    }
                    if (tau_history_count >= tau_history_min) {
                        // Robust drift estimate: the least-squares slope is
                        // wrecked by a single outlier window. A multipath
                        // group-delay jump can flip the per-pair pilot phase
                        // difference past +/-pi and shift the measured tau by
                        // a hundred samples in one window (observed on 545:
                        // tau=171.11 -> -163.89 in one stats window, after
                        // which the slope estimate dropped to ~0 and the loop
                        // walked the window off grid). Take the median of the
                        // consecutive first differences instead: the true
                        // sample-clock drift is slow (0.36 samples/window on
                        // 545) and the per-window noise is symmetric, so a
                        // handful of outlier windows cannot move the median
                        // while the median still tracks the drift.
                        const std::size_t diffs_count = tau_history_count - 1;
                        std::array<double, tau_history_n - 1> diffs{};
                        for (std::size_t i = 0; i < diffs_count; ++i) {
                            const std::size_t idx0 =
                                (tau_history_head + tau_history_n -
                                 tau_history_count + i) %
                                tau_history_n;
                            const std::size_t idx1 = (idx0 + 1) % tau_history_n;
                            const double sample_span =
                                tau_sample_history[idx1] -
                                tau_sample_history[idx0];
                            diffs[i] =
                                sample_span > 0.0
                                    ? (tau_history[idx1] - tau_history[idx0]) *
                                          1.0e6 / sample_span
                                    : 0.0;
                        }
                        std::sort(
                            diffs.begin(),
                            diffs.begin() +
                                static_cast<std::ptrdiff_t>(diffs_count));
                        const double drift_estimate_ppm =
                            diffs_count % 2 != 0
                                ? diffs[diffs_count / 2]
                                : 0.5 * (diffs[diffs_count / 2 - 1] +
                                         diffs[diffs_count / 2]);
                        smoothed_sample_clock_ppm =
                            0.1 * drift_estimate_ppm +
                            0.9 * smoothed_sample_clock_ppm;
                    }
                    const double drift_limit_ppm =
                        4.0 * 1.0e6 /
                        (static_cast<double>(stats_window_symbols) *
                         static_cast<double>(period));
                    smoothed_sample_clock_ppm = std::clamp(
                        smoothed_sample_clock_ppm, -drift_limit_ppm,
                        drift_limit_ppm);
                    smoothed_timing_drift =
                        smoothed_sample_clock_ppm * window_sample_count /
                        1.0e6;
                    fractional_timing +=
                        smoothed_timing_drift / timing_window_shift_response;
                    fractional_timing =
                        std::clamp(fractional_timing, -4.0, 4.0);
                    if (event_debug_enabled() && timing_count != 0 &&
                        window_symbols >= stats_window_symbols) {
                        std::fprintf(stderr,
                                     "[evt] tloop raw=%.2f tau=%.2f "
                                     "phys=%.2f shift=%.1f drift=%.3f "
                                     "smooth=%.3f sro=%+.4fppm frac=%.2f "
                                     "act=%+.4f/%+.4fppm conf=%.3f "
                                     "cir=%.2f/%.3f ready=%d\n",
                                     latest_raw_timing,
                                     static_cast<double>(timing_offset),
                                     static_cast<double>(timing_offset) +
                                         window_cir_avg +
                                         timing_window_shift_response *
                                             accumulated_window_shift,
                                     accumulated_window_shift,
                                     corrected_drift,
                                     smoothed_timing_drift,
                                     smoothed_sample_clock_ppm,
                                     fractional_timing,
                                     window_sample_count > 0.0
                                         ? telemetry_window_shift * 1.0e6 /
                                               window_sample_count
                                         : 0.0,
                                     rolling_shift_rate_ppm,
                                     static_cast<double>(timing_count) /
                                         static_cast<double>(window_symbols),
                                     window_cir_avg, cir_confidence,
                                     tau_history_count >= tau_history_min);
                    }
                }
                const std::scoped_lock lock(mutex);
                latest.ofdm_locked = true;
                latest.fft_size = static_cast<std::uint32_t>(fft_size);
                latest.guard_size = static_cast<std::uint32_t>(guard_size);
                latest.tps_locked = frontend.tps_snapshot.currently_valid;
                latest.tps_ever_locked = frontend.tps_snapshot.ever_locked;
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
                latest.raw_timing_offset_samples =
                    static_cast<float>(latest_raw_timing);
                latest.timing_offset_samples = timing_offset;
                latest.physical_timing_offset_samples =
                    timing_count == 0
                        ? 0.0F
                        : static_cast<float>(
                              static_cast<double>(timing_offset) +
                              window_cir_avg +
                              timing_window_shift_response *
                                  accumulated_window_shift);
                latest.observed_timing_drift_samples =
                    static_cast<float>(observed_drift);
                latest.corrected_timing_drift_samples =
                    static_cast<float>(corrected_drift);
                latest.smoothed_timing_drift_samples =
                    static_cast<float>(smoothed_timing_drift);
                latest.sample_clock_offset_ppm =
                    static_cast<float>(smoothed_sample_clock_ppm);
                latest.cumulative_timing_shift_samples =
                    static_cast<float>(accumulated_window_shift);
                latest.timing_shift_rate_ppm =
                    window_sample_count > 0.0
                        ? static_cast<float>(telemetry_window_shift * 1.0e6 /
                                             window_sample_count)
                        : 0.0F;
                latest.rolling_timing_shift_rate_ppm =
                    static_cast<float>(rolling_shift_rate_ppm);
                latest.fractional_timing_samples =
                    static_cast<float>(fractional_timing);
                latest.cir_offset_samples = static_cast<float>(window_cir_avg);
                latest.timing_confidence =
                    window_symbols == 0
                        ? 0.0F
                        : std::clamp(
                              static_cast<float>(timing_count) /
                                  static_cast<float>(window_symbols),
                              0.0F, 1.0F);
                latest.cir_confidence = static_cast<float>(cir_confidence);
                latest.timing_measurements = timing_raw_count;
                latest.timing_accepted_measurements = timing_count;
                latest.timing_rejected_measurements = timing_rejected_count;
                latest.timing_drift_ready =
                    tau_history_count >= tau_history_min;
                latest.fade_indicator = fade_indicator;
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
                latest.demod_busy_fraction =
                    window_wall > 0.0F
                        ? static_cast<float>(
                              demod_busy_time_sum_ms /
                              static_cast<double>(window_wall))
                        : 0.0F;
                latest.demod_window_wall_time_ms = window_wall;
                latest.demod_busy_time_ms =
                    static_cast<float>(demod_busy_time_sum_ms);
                demod_busy_time_sum_ms = 0.0;
                latest.symbol_preprocess_work_time_ms = preprocess_time_sum;
                latest.symbol_demap_work_time_ms = demap_time_sum;
                latest.symbol_deinterleave_work_time_ms = deinterleave_time_sum;
                latest.symbol_depuncture_work_time_ms = depuncture_time_sum;
                latest.last_acquisition_time_ms = acquisition_time_ms;
                latest.symbol_workers = workers.symbol;
                latest.resample_workers = 1;
                latest.state_carried = last_reanchor_carried;
                latest.fec_skipped = in_hopeless_region;
                ++latest.processed_chunks;
                acquisition_time_ms = 0.0F;
            };

            while (true) {
                // --- wait for first-anchor data, a reset (the front-end
                //     only invalidates the sync on retune/reset), or the
                //     resume of a flushed stream ---
                {
                    std::unique_lock lock(mutex);
                    // While working toward a first anchor (including the wait
                    // for acquisition data), mark the demod as acquisition-
                    // pending: the front-end's push-abandon must not fire in
                    // the window between this wait waking and the acquisition
                    // marking itself busy — a ring full of fresh data with a
                    // flush arriving mid-push used to abandon the push there,
                    // dropping the first block's remainder and starving small
                    // files before the TPS could lock. It may fire only once
                    // the demod parks in the retry back-off (never-lockable
                    // streams) or the end-of-stream wait.
                    if (!have_grid) {
                        acquisition_pending = true;
                    }
                    demod_state.store(
                        static_cast<int>(WorkerState::waiting_sync));
                    ring_data.wait(lock, [this, &seen_sync_version,
                                          &have_grid] {
                        return stopping || sync.version != seen_sync_version ||
                               // A finite stream may close while the first
                               // acquisition is still running. Once the grid
                               // is published, drain the already-written
                               // samples even though the ring is closed.
                               (have_grid && ring_write_pos > ring_read_pos) ||
                               (!have_grid && ring_closed) ||
                               (!have_grid && ring_write_pos - ring_read_pos >=
                                                  acquisition_samples);
                    });
                    if (stopping) {
                        demod_state.store(
                            static_cast<int>(WorkerState::exited));
                        return;
                    }
                }
                {
                    const std::scoped_lock lock(mutex);
                    if (sync.version != seen_sync_version) {
                        static_cast<void>(handle_sync_change());
                    }
                }
                fire_pending_discontinuity();
                // Explicit constellation/code-rate parameters are sufficient
                // to start the FEC path; TPS is optional in that mode. This
                // also makes a manually configured synthetic/test signal
                // exercise the complete pipeline without manufacturing a
                // valid TPS frame.
                if (have_grid && decoder_parameters &&
                    postprocessor == nullptr && !start_decoder()) {
                    return;
                }
                if (!have_grid) {
                    // First anchor: event-driven acquisition (acquisition no
                    // longer runs on a fixed cadence; the only other events
                    // are the long-fade re-anchor and a TPS mode change). The
                    // demod is genuinely busy while it acquires, so
                    // wait_until_idle blocks until it finishes (bounded work),
                    // while the push-abandon still sees acquisition_pending
                    // and never fires mid-acquisition.
                    {
                        const std::scoped_lock lock(mutex);
                        demod_busy = true;
                    }
                    demod_state.store(
                        static_cast<int>(WorkerState::processing));
                    const float score = run_event_acquisition();
                    if (!have_grid) {
                        const std::scoped_lock lock(mutex);
                        demod_busy = false;
                        acquisition_pending = false;
                    }
                    if (!have_grid && ring_closed) {
                        {
                            const std::scoped_lock lock(mutex);
                            demod_busy = false;
                            // No acquisition means these samples cannot be
                            // consumed by the symbol loop. Drop the closed
                            // stream's dead data before parking for reset or a
                            // new source, so wait_until_idle can observe a
                            // genuinely drained decoder.
                            ring_read_pos = ring_write_pos;
                            idle.notify_all();
                        }
                        // The stream ended without a signal. Stay alive for
                        // the next stream: a reset bumps the sync version and
                        // a reopened ring refills the data.
                        std::unique_lock lock(mutex);
                        demod_state.store(
                            static_cast<int>(WorkerState::waiting_sync));
                        ring_data.wait(lock, [this, &seen_sync_version] {
                            return stopping ||
                                   sync.version != seen_sync_version ||
                                   (!ring_closed &&
                                    ring_write_pos > ring_read_pos);
                        });
                        if (stopping) {
                            demod_state.store(
                                static_cast<int>(WorkerState::exited));
                            return;
                        }
                    } else if (score == 0.0F) {
                        {
                            const std::scoped_lock lock(mutex);
                            demod_busy = false;
                            acquisition_pending = false;
                        }
                        // No signal yet (or the acquisition can never succeed
                        // for this stream): back off so the retry cannot
                        // busy-loop and stall the ring in front of the
                        // front-end. The flush/reset notifications still wake
                        // this wait. Parked here with demod_busy false, a
                        // flush can abandon the front-end push (ring full)
                        // and close the ring so the stream ends cleanly
                        // instead of deadlocking.
                        std::unique_lock lock(mutex);
                        demod_state.store(static_cast<int>(
                            WorkerState::waiting_acquisition));
                        ring_data.wait_for(
                            lock, std::chrono::milliseconds(100),
                            [this, &seen_sync_version] {
                                return stopping ||
                                       sync.version != seen_sync_version;
                            });
                        if (stopping) {
                            demod_state.store(
                                static_cast<int>(WorkerState::exited));
                            return;
                        }
                    }
                    continue;
                }
                // --- contiguous symbol stream ---
                while (true) {
                    {
                        const std::scoped_lock lock(mutex);
                        if (sync.version != seen_sync_version) {
                            static_cast<void>(handle_sync_change());
                        }
                    }
                    fire_pending_discontinuity();
                    if (have_grid && decoder_parameters &&
                        postprocessor == nullptr && !start_decoder()) {
                        return;
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
                        demod_state.store(static_cast<int>(
                            WorkerState::waiting_ring_data));
                        ring_data.wait(lock, [this, needed, &seen_sync_version] {
                            return stopping ||
                                   sync.version != seen_sync_version ||
                                   (ring_closed && ring_write_pos < needed) ||
                                   ring_write_pos >= needed;
                        });
                        if (stopping) {
                            return;
                        }
                        if (sync.version != seen_sync_version) {
                            // A reset or retune landed while this thread was
                            // parked on a stale stream position: the reset
                            // path rewinds the ring to zero, so the old
                            // `needed` may never be reachable again. Re-run
                            // the loop head, which sees the new sync version
                            // and drops the grid (handle_sync_change).
                            break;
                        }
                        if (ring_closed && ring_write_pos < needed) {
                            break; // end of stream
                        }
                    }
                    for (std::size_t i = 0; i < fft_size; ++i) {
                        const std::uint64_t position = next_symbol_start + i;
                        fft_in[i] = ring[position % ring.size()];
                    }
                    demod_state.store(
                        static_cast<int>(WorkerState::processing));
                    demod_busy_started_at = std::chrono::steady_clock::now();
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
                    fade_indicator = 1.0F;                    // Only update the CFO loop from a contiguous symbol pair.
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
                        double power_current = 0.0;
                        double power_previous = 0.0;
                        for (std::size_t i = 0; i < current_continual.size();
                             ++i) {
                            temporal_correlation +=
                                current_continual[i] *
                                std::conj(frontend.previous_continual[i]);
                            power_current +=
                                std::norm(current_continual[i]);
                            power_previous +=
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
                        // loop until the correlation returns. The denominator
                        // is the geometric mean of the two powers
                        // (sqrt(P_a * P_b)), not their sum: dividing by the
                        // sum halves a perfect correlation to 0.5, which made
                        // a healthy ~18 dB 64-QAM signal read as marginal
                        // (fi ~= 0.5) and let the pilot lock drift onto a
                        // multipath alias through the "healthy" path.
                        const float normalized_correlation =
                            power_current > 0.0 && power_previous > 0.0
                                ? static_cast<float>(
                                      std::abs(temporal_correlation)) /
                                      static_cast<float>(
                                          std::sqrt(power_current *
                                                    power_previous))
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
                    // Shared re-acquisition used by both the fade branch and
                    // the decode-stuck branch below: run the wideband CP
                    // acquisition and, on success, restore the stable carrier
                    // grid and the mod-4 phase at the absolute frame position
                    // (deterministic and immune to the rotation a stale
                    // channel estimate carries, which the MER cannot see),
                    // then re-lock the TPS on the healthy grid. Returns true
                    // when the caller must discard the in-flight symbol and
                    // restart from the newly published boundary.
                    const auto maybe_reacquire = [&]() -> bool {
                        if (event_debug_enabled()) {
                            std::fprintf(stderr,
                                         "[evt] re-anchor triggered fi=%.3f "
                                         "off=%d sym=%llu\n",
                                         fade_indicator,
                                         frontend.carrier_offset,
                                         static_cast<unsigned long long>(
                                             symbol_count));
                        }
                        const float reanchor_score =
                            run_event_acquisition(true);
                        if (event_debug_enabled()) {
                            std::fprintf(stderr,
                                         "[evt] re-anchor score=%.3f "
                                         "stable_off=%d\n",
                                         reanchor_score,
                                         frontend.stable_carrier_offset);
                        }
                        if (reanchor_score < 0.20F) {
                            return false;
                        }
                        if (frontend.stable_carrier_offset !=
                            std::numeric_limits<int>::max()) {
                            // Restore the carrier grid from the stable
                            // reference: the LO never moves, so the true
                            // offset is the pre-fade one, while the narrow
                            // lock can latch a multipath alias (the observed
                            // off=2 at the 581 fade) whose pilot pattern
                            // passes the scatter verify. Restore the offset
                            // and the scattered-pilot phase from the absolute
                            // frame position, then hold the restored grid for
                            // one symbol so the lock cannot immediately flip
                            // the correct phase onto a noise-latched one.
                            frontend.carrier_offset =
                                frontend.stable_carrier_offset;
                            frontend.previous_phase = static_cast<int>(
                                (frontend.stable_phase + symbol_count) % 4);
                            lock_hold = 1;
                        }
                        // The acquisition republished the grid: the in-flight
                        // symbol's FFT window was read from the pre-anchor
                        // boundary, so combining it with the newly anchored
                        // grid would corrupt its channel estimate and payload.
                        // Discard it and restart from the published
                        // next_symbol_start at the loop head.
                        hopeless_window_count = 0;
                        return true;
                    };
                    PilotLock lock;
                    if (lock_hold > 0) {
                        --lock_hold;
                        lock =
                            PilotLock{static_cast<int>(frontend.previous_phase),
                                      frontend.carrier_offset};
                    } else if ((fade_indicator <= 0.25F &&
                                frontend.previous_phase >= 0) ||
                               hopeless_window_count >= 4) {
                        if (event_debug_enabled() && frozen_symbol_count == 0) {
                            // Distinguish a real fade (fi collapsed) from a
                            // decode-stuck state (fi healthy but MER below the
                            // decode floor — the carrier grid drifted off,
                            // which the continual-carrier correlation cannot
                            // see).
                            if (fade_indicator > 0.25F) {
                                std::fprintf(
                                    stderr,
                                    "[evt] badlock enter fi=%.3f off=%d "
                                    "sym=%llu discont=%llu timing=%.2f "
                                    "hopeless=%llu\n",
                                    fade_indicator, frontend.carrier_offset,
                                    static_cast<unsigned long long>(
                                        symbol_count),
                                    static_cast<unsigned long long>(
                                        frontend.phase_discontinuities),
                                    timing_count == 0
                                        ? 0.0
                                        : (timing_acc /
                                           static_cast<double>(timing_count)) *
                                              static_cast<double>(fft_size) /
                                              (2.0 *
                                               std::numbers::pi_v<double>),
                                    static_cast<unsigned long long>(
                                        hopeless_window_count));
                            } else {
                                std::fprintf(
                                    stderr,
                                    "[evt] fade enter fi=%.3f off=%d sym=%llu\n",
                                    fade_indicator, frontend.carrier_offset,
                                    static_cast<unsigned long long>(
                                        symbol_count));
                            }
                        }
                        // Fade / decode-stuck: hold the frozen phase/offset
                        // and re-acquire directly. A fade (or a grid drift)
                        // can shift the true carrier offset (observed off=2 at
                        // the 581 fade) and disturb the mod-4 scattered-pilot
                        // phase, so a narrow re-lock on the frozen grid can
                        // only latch a multipath alias or stay stuck on a
                        // stale phase — the permanent lock loss seen on the
                        // 581 tail (MER stuck at ~7.8 dB with
                        // phase-discontinuities climbing). The wideband CP
                        // correlation, unlike the continual-carrier
                        // correlation, survives the multipath that keeps
                        // fade_indicator collapsed, so re-acquisition
                        // confirms the signal's return and rebuilds the whole
                        // grid (offset, phase, boundary, guard) at once.
                        // Retry every 68 symbols (~100 ms) until the signal
                        // returns; while faded there is no valid TS to lose,
                        // and a re-anchor that lands while the signal is
                        // still gone just scores low and retries.
                        lock = PilotLock{
                            static_cast<int>(frontend.previous_phase),
                            frontend.carrier_offset};
                        if (++frozen_symbol_count >= 68 &&
                            frozen_symbol_count % 68 == 0 &&
                            maybe_reacquire()) {
                            continue;
                        }
                    } else {
                        if (frontend.carrier_offset ==
                            std::numeric_limits<int>::max()) {
                            // First healthy lock after (re-)acquisition: the
                            // grid is unknown, so run the full offset search
                            // once to establish it.
                            lock = lock_pilots(fft_out, maximum,
                                               frontend.carrier_offset);
                        } else {
                            // Grid already established: freeze the carrier
                            // offset and re-verify only the rotating phase.
                            // The offset is a fixed physical property (the LO
                            // never moves; residual drift is sub-bin and owned
                            // by the CFO loop), so re-searching it every
                            // symbol only chases multipath-induced offset
                            // ambiguity — the 545 capture shows lock_pilots
                            // returning 0/±1/±2 at random from symbol to
                            // symbol even at fi=0.9996, and a 4-symbol
                            // confirmation window is short enough to accept
                            // one of those spurious offsets, permanently
                            // snapping the grid off the true carrier (the
                            // observed car 0->3 collapse). The grid is only
                            // re-established by a re-anchor after a real
                            // fade, which restores the pre-fade stable
                            // offset.
                            lock = PilotLock{
                                lock_phase_at_offset(
                                    fft_out, maximum, frontend.carrier_offset,
                                    timing_tracker.filtered()),
                                frontend.carrier_offset};
                        }
                        if (frontend.previous_phase >= 0 &&
                            lock.phase != (frontend.previous_phase + 1) % 4) {
                            ++frontend.phase_discontinuities;
                            if (event_debug_enabled()) {
                                std::fprintf(stderr,
                                             "[evt] phase-jump %d->%d sym=%llu "
                                             "fi=%.3f off=%d\n",
                                             frontend.previous_phase, lock.phase,
                                             static_cast<unsigned long long>(
                                                 symbol_count),
                                             fade_indicator,
                                             frontend.carrier_offset);
                            }
                        }
                        frontend.previous_phase = lock.phase;
                        frontend.carrier_offset = lock.offset;
                        // Capture the stable grid only once: the carrier grid
                        // is fixed for the life of the stream (the LO never
                        // moves), so the pre-fade reference the fade re-anchor
                        // restores must be a healthy lock, never a
                        // multipath-influenced one. A single first lock is not
                        // enough — the stream can start through a marginal
                        // channel, and a noise-latched alias captured here
                        // would be restored by every later re-anchor (the
                        // 545 capture's car 0->3: the stable reference was
                        // pinned to the alias, so each fade re-anchor snapped
                        // the grid to 3 and decoding never recovered). Require
                        // the same offset for a few consecutive healthy
                        // symbols before trusting it.
                        if (frontend.stable_carrier_offset ==
                            std::numeric_limits<int>::max()) {
                            if (lock.offset == stable_pending_offset) {
                                if (++stable_pending_count >= 4) {
                                    frontend.stable_phase = lock.phase;
                                    frontend.stable_carrier_offset =
                                        lock.offset;
                                }
                            } else {
                                stable_pending_offset = lock.offset;
                                stable_pending_count = 1;
                            }
                        }
                        const auto was_frozen = frozen_symbol_count;
                        frozen_symbol_count = 0;
                        if (event_debug_enabled() && was_frozen > 0) {
                            std::fprintf(stderr,
                                         "[evt] fade exit fi=%.3f off=%d "
                                         "sym=%llu\n",
                                         fade_indicator,
                                         frontend.carrier_offset,
                                         static_cast<unsigned long long>(
                                             symbol_count));
                        }
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
                    // (2*pi*k*tau/N). Estimate it from the fixed-spacing
                    // scattered-pilot grid, unwrap it against the previous
                    // filtered value, and reject isolated group-delay clicks
                    // before feeding either the long-term timing loop or the
                    // phase verifier.
                    if (const auto measured_tau =
                            estimate_scattered_timing_tau(
                                channel, static_cast<std::size_t>(lock.phase),
                                maximum, fft_size);
                        measured_tau.has_value()) {
                        latest_raw_timing = *measured_tau;
                        ++timing_raw_count;
                        const auto previous_filtered_tau =
                            timing_tracker.filtered();
                        const auto accepted_tau =
                            timing_tracker.observe(*measured_tau);
                        if (accepted_tau.has_value()) {
                            if (event_debug_enabled() &&
                                previous_filtered_tau.has_value() &&
                                std::abs(*accepted_tau -
                                         *previous_filtered_tau) >
                                    timing_outlier_limit_samples) {
                                std::fprintf(
                                    stderr,
                                    "[evt] timing-branch old=%.2f new=%.2f "
                                    "raw=%.2f sym=%llu\n",
                                    *previous_filtered_tau, *accepted_tau,
                                    *measured_tau,
                                    static_cast<unsigned long long>(
                                        symbol_count));
                            }
                            timing_acc +=
                                *accepted_tau *
                                (2.0 * std::numbers::pi_v<double>) /
                                static_cast<double>(fft_size);
                            ++timing_count;
                        } else if (event_debug_enabled()) {
                            ++timing_rejected_count;
                            const auto filtered_tau = timing_tracker.filtered();
                            std::fprintf(
                                stderr,
                                "[evt] timing-reject raw=%.2f filtered=%.2f "
                                "sym=%llu\n",
                                *measured_tau,
                                filtered_tau.value_or(*measured_tau),
                                static_cast<unsigned long long>(symbol_count));
                        } else {
                            ++timing_rejected_count;
                        }
                    }
                    // This symbol was demodulated with the current CIR anchor.
                    // Accumulate the exact applied position before the CIR
                    // update below can move the anchor for the next symbol.
                    window_cir_offset_sum +=
                        static_cast<double>(applied_cir_offset);
                    // CIR / delay-spread estimate (scattered pilots -> IFFT ->
                    // impulse response) for adaptive FFT-window placement.
                    // Once per TPS frame: negligible cost. When the measured
                    // delay spread is significant (>= guard/4) the window
                    // slides toward the middle of the ISI-free range
                    // [spread, guard], balancing the pre/post-ISI margins;
                    // the offset steps by at most +/-4 samples per frame so
                    // the timing loop is never perturbed by a jump.
                    if (++frontend.cir_symbol_count >= 68) {
                        frontend.cir_symbol_count = 0;
                        if (fade_indicator > 0.25F &&
                            frontend.cir_plan != nullptr) {
                            std::fill(frontend.cir_grid.begin(),
                                      frontend.cir_grid.end(),
                                      std::complex<float>{});
                            const std::size_t phase =
                                static_cast<std::size_t>(lock.phase);
                            std::size_t slot = 0;
                            for (std::size_t k = phase * 3; k <= maximum;
                                 k += 12) {
                                if (slot >= frontend.cir_grid.size()) {
                                    break;
                                }
                                frontend.cir_grid[slot] = channel[k];
                                ++slot;
                            }
                            fftwf_execute(frontend.cir_plan);
                            const std::size_t n =
                                frontend.cir_response.size();
                            std::vector<double> energy(n, 0.0);
                            double total = 0.0;
                            std::size_t peak = 0;
                            double peak_energy = -1.0;
                            for (std::size_t i = 0; i < n; ++i) {
                                energy[i] = static_cast<double>(std::norm(
                                    frontend.cir_response[i]));
                                total += energy[i];
                                if (energy[i] > peak_energy) {
                                    peak_energy = energy[i];
                                    peak = i;
                                }
                            }
                            cir_confidence =
                                total > 0.0 ? peak_energy / total : 0.0;
                            // Only trust a structured response: the peak tap
                            // must hold a meaningful share of the energy, so
                            // a noise-driven CIR cannot drag the window.
                            if (total > 0.0 &&
                                peak_energy / total > 0.05) {
                                // Contiguous main lobe: taps above 1% of the
                                // peak, wrapping the IFFT window (taps that
                                // arrive before the FFT window fold to its
                                // tail). Underestimating the spread is safe:
                                // the offset only moves the window inside
                                // the ISI-free range [d_max, G + d_min].
                                const double lobe_threshold =
                                    peak_energy * 0.01;
                                std::size_t lo = peak;
                                std::size_t hi = peak;
                                while (energy[(lo + n - 1) % n] >=
                                           lobe_threshold &&
                                       (lo + n - 1) % n != hi) {
                                    lo = (lo + n - 1) % n;
                                }
                                while (energy[(hi + 1) % n] >=
                                           lobe_threshold &&
                                       (hi + 1) % n != lo) {
                                    hi = (hi + 1) % n;
                                }
                                const std::size_t lobe_width =
                                    (hi + n - lo) % n + 1;
                                const double scale =
                                    static_cast<double>(fft_size) /
                                    static_cast<double>(12 * frontend.cir_n);
                                const double spread_samples =
                                    static_cast<double>(lobe_width) * scale;
                                // Slide the window to the middle of the
                                // ISI-free range [spread, guard] (in offset
                                // coordinates relative to the effective
                                // symbol start), balancing the pre- and
                                // post-ISI margins. Only act when the delay
                                // spread is significant: for a single-path /
                                // short-delay channel the current anchor is
                                // already inside the (wide) ISI-free range,
                                // and sliding the window there buys nothing
                                // while perturbing the timing loop.
                                const double spread_threshold =
                                    0.25 * static_cast<double>(guard_size);
                                const double target =
                                    spread_samples > spread_threshold
                                        ? std::clamp(
                                              (spread_samples -
                                               static_cast<double>(
                                                   guard_size)) /
                                                  2.0,
                                              -static_cast<double>(
                                                  guard_size),
                                              0.0)
                                        : 0.0;
                                frontend.cir_offset = static_cast<float>(
                                    0.9 * static_cast<double>(
                                              frontend.cir_offset) +
                                    0.1 * target);
                                const int applied = static_cast<int>(
                                    std::lround(frontend.cir_offset));
                                // Step the anchor by at most +/-4 samples per
                                // frame. The timing loop is now rebased onto
                                // the window's average position (so slides do
                                // not read as sample-clock steps), but the
                                // step bound still limits how often the CFO
                                // update is skipped by the boundary crossing
                                // and keeps a mis-estimated target from
                                // sliding far into the ISI region in one
                                // frame.
                                const int delta = std::clamp(
                                    applied - applied_cir_offset, -4, 4);
                                if (delta != 0) {
                                    if (delta > 0) {
                                        next_symbol_start +=
                                            static_cast<std::uint64_t>(delta);
                                    } else {
                                        next_symbol_start -=
                                            static_cast<std::uint64_t>(-delta);
                                    }
                                    applied_cir_offset += delta;
                                    // The window moved mid-stream: advance
                                    // the mixer phase by the skipped samples
                                    // and let the contiguous-pair gate skip
                                    // the CFO update for this boundary.
                                    nco_phase = std::remainder(
                                        nco_phase +
                                            (frontend.tracked_cfo_phase *
                                             static_cast<float>(delta)),
                                        2.0F * std::numbers::pi_v<float>);
                                }
                            }
                        } else {
                            cir_confidence = 0.0;
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
                    for (std::size_t i = 0; i < tps_indices.size(); ++i) {
                        const std::size_t k = tps_indices[i];
                        tps_values[i] =
                            carrier(fft_out, k, maximum,
                                    frontend.carrier_offset) *
                            channel[k];
                    }
                    // TPS differential bits carry continuously across the
                    // stream. Before synchronization the decoder checks each
                    // sliding 68-bit window; afterward it validates only the
                    // expected frame boundary and returns to the sliding search
                    // after a bad frame. The FEC configuration remains fixed
                    // after its first matching TPS frame; deinterleave parity
                    // comes from the measured pilot phase below, not the TPS
                    // frame index.
                    frontend.tps_snapshot =
                        frontend.tps_decoder.process(tps_values);
                    const bool matching_tps =
                        frontend.tps_snapshot.ever_locked &&
                        frontend.tps_snapshot.parameters.mode ==
                            frontend.mode &&
                        frontend.tps_snapshot.parameters.guard_interval ==
                            frontend.guard;
                    if (frontend.tps_snapshot.ever_locked && !matching_tps) {
                        if (++tps_mismatch_symbols >= 1400) {
                            tps_mismatch_symbols = 0;
                            if (run_event_acquisition(true) >= 0.20F) {
                                // The grid was republished (the TPS decoded a
                                // mode/guard that disagrees with the current
                                // grid): the in-flight symbol predates the new
                                // grid. Discard it and restart from the
                                // published boundary at the loop head.
                                continue;
                            }
                        }
                    } else {
                        tps_mismatch_symbols = 0;
                    }
                    if (!decoder_parameters && matching_tps &&
                        frontend.tps_snapshot.parameters.hierarchy == 0U) {
                        if (event_debug_enabled()) {
                            std::fprintf(stderr,
                                         "[evt] TPS lock mode=%d g=%d const=%d "
                                         "rate=%d sym=%llu\n",
                                         static_cast<int>(frontend.mode),
                                         static_cast<int>(frontend.guard),
                                         static_cast<int>(
                                             frontend.tps_snapshot.parameters
                                                 .constellation),
                                         static_cast<int>(
                                             frontend.tps_snapshot.parameters
                                                 .high_priority_code_rate),
                                         static_cast<unsigned long long>(
                                             symbol_count));
                        }
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
                    // Symbol-period advance with the closed-loop sample-clock
                    // correction: the accumulated fractional timing error
                    // carries into a +/-1-sample step on the period when it
                    // crosses +/-0.5, keeping the FFT window on the true
                    // symbol start.
                    const auto advance_symbol = [&]() {
                        next_symbol_start += period;
                        if (fractional_timing >= 0.5) {
                            ++next_symbol_start;
                            accumulated_window_shift += 1.0;
                            fractional_timing -= 1.0;
                        } else if (fractional_timing <= -0.5) {
                            --next_symbol_start;
                            accumulated_window_shift -= 1.0;
                            fractional_timing += 1.0;
                        }
                        nco_phase = std::remainder(
                            nco_phase + (frontend.tracked_cfo_phase *
                                         static_cast<float>(period)),
                            2.0F * std::numbers::pi_v<float>);
                    };

                    if (payload.size() !=
                        payload_carrier_count(frontend.mode)) {
                        advance_symbol();
                        ++symbol_count;
                        ++window_symbol_count;
                        if (window_symbol_count >= stats_window_symbols) {
                            publish_stats_window();
                            window_started_at =
                                std::chrono::steady_clock::now();
                            window_symbol_count = 0;
                            window_cir_offset_sum = 0.0;
                            mer_sum = 0.0;
                            preprocess_time_sum = 0.0F;
                            demap_time_sum = 0.0F;
                            deinterleave_time_sum = 0.0F;
                            depuncture_time_sum = 0.0F;
                            timing_acc = 0.0;
                            timing_count = 0;
                            latest_raw_timing = 0.0;
                            timing_raw_count = 0;
                            timing_rejected_count = 0;
                        }
                        demod_busy_time_sum_ms +=
                            duration_ms(demod_busy_started_at);
                        continue;
                    }
                    if (postprocessor == nullptr) {
                        pending_symbols.push_back(
                            {.payload = std::move(payload),
                             .equalizer_power = std::move(equalizer_power)});
                        if (pending_symbols.size() > 272) {
                            // The first TPS lock can take ~68-101 symbols on
                            // marginal signal; the back-computed parity
                            // indices can still decode the buffered symbols,
                            // so keep a generous margin before dropping.
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
                                // Back-computed from the measured pilot phase
                                // (the only thing the deinterleave needs is
                                // the parity), never from the TPS frame index.
                                const std::size_t symbol_index =
                                    (static_cast<std::size_t>(lock.phase) + 68 -
                                     (distance % 68)) %
                                    68;
                                postprocessor->submit(
                                    std::move(pending.payload),
                                    std::move(pending.equalizer_power),
                                    symbol_index);
                            }
                        }
                        const std::size_t symbol_index =
                            static_cast<std::size_t>(lock.phase);
                        postprocessor->submit(std::move(payload),
                                              std::move(equalizer_power),
                                              symbol_index);
                        process_batch(postprocessor->take_ready());
                    }
                    ++symbol_count;
                    ++window_symbol_count;
                    advance_symbol();
                    demod_busy_time_sum_ms +=
                        duration_ms(demod_busy_started_at);
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
                        window_cir_offset_sum = 0.0;
                        mer_sum = 0.0;
                        preprocess_time_sum = 0.0F;
                        demap_time_sum = 0.0F;
                        deinterleave_time_sum = 0.0F;
                        depuncture_time_sum = 0.0F;
                        timing_acc = 0.0;
                        timing_count = 0;
                        latest_raw_timing = 0.0;
                        timing_raw_count = 0;
                        timing_rejected_count = 0;
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
                        fec_floor(decoder_parameters->constellation) - 2.0F;
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
                    // The input ended: the FEC flush emits the trellis tail
                    // and the stream is over. Playback must not wait for more
                    // data — it plays out the queued tail and hits EOF.
                    fire_discontinuity(TransportDiscontinuity::stream_end);
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
                    window_cir_offset_sum = 0.0;
                    mer_sum = 0.0;
                    preprocess_time_sum = 0.0F;
                    demap_time_sum = 0.0F;
                    deinterleave_time_sum = 0.0F;
                    depuncture_time_sum = 0.0F;
                    timing_acc = 0.0;
                    timing_count = 0;
                    latest_raw_timing = 0.0;
                    timing_raw_count = 0;
                    timing_rejected_count = 0;
                }
                {
                    const std::scoped_lock lock(mutex);
                    // A finite stream can end with a partial next symbol (the
                    // guard prefix or a truncated FFT window). It is not
                    // decodable input and must not keep wait_until_idle from
                    // observing a drained stream after the valid tail flush.
                    ring_read_pos = ring_write_pos;
                    demod_busy = false;
                    acquisition_pending = false;
                }
                idle.notify_all();
            }
        } catch (const std::exception &exception) {
            // An unexpected exception here would terminate the process (the
            // thread is joined in ~Impl). Dump the pipeline state before the
            // crash so a mid-stream stall/exit is diagnosable.
            std::fprintf(stderr,
                         "[decoder] demod thread exception: %s "
                         "(sync=%llu locked=%d ring=%llu/%llu iq=%zu/%zu "
                         "fec=%zu/%zu)\n",
                         exception.what(),
                         static_cast<unsigned long long>(sync.version),
                         static_cast<int>(sync.valid),
                         static_cast<unsigned long long>(ring_write_pos -
                                                         ring_read_pos),
                         static_cast<unsigned long long>(ring.size()),
                         queue.size(), input_queue_capacity_samples,
                         fec_queue.size(), fec_queue_capacity);
            throw;
        } catch (...) {
            std::fprintf(stderr, "[decoder] demod thread exception: unknown\n");
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
        float decoder_transport_time_ms = 0.0F;
        while (true) {
            FecItem item;
            {
                std::unique_lock lock(mutex);
                fec_state.store(static_cast<int>(WorkerState::waiting_fec_item));
                fec_ready.wait(
                    lock, [this] { return stopping || !fec_queue.empty(); });
                if (stopping) {
                    fec_state.store(static_cast<int>(WorkerState::exited));
                    return;
                }
                fec_state.store(static_cast<int>(WorkerState::processing));
                item = std::move(fec_queue.front());
                fec_queue.pop_front();
                fec_worker_busy = true;
            }
            fec_not_full.notify_one();

            if (item.generation == latest_generation) {
                if (item.kind == FecItem::Kind::begin) {
                    const bool parameters_match =
                        decoder != nullptr &&
                        decoder->parameters() == item.parameters;
                    if (!parameters_match) {
                        // A mode/constellation/code-rate change requires a
                        // new Decoder configuration. A generation change by
                        // itself does not: the Decoder owns a long-lived
                        // Viterbi worker pool, and reset() is sufficient to
                        // discard the old stream state.
                        if (decoder != nullptr &&
                            decoder_generation == item.generation) {
                            fire_discontinuity(
                                TransportDiscontinuity::fec_region_reset);
                        }
                        decoder = std::make_unique<Decoder>(item.parameters);
                    } else {
                        // A new generation (retune/source reset) or a gated
                        // region ended. Keep the Viterbi threads and reset
                        // only the decoder state; generation filtering above
                        // already prevents stale FEC items from crossing the
                        // seam.
                        if (decoder_generation == item.generation) {
                            fire_discontinuity(
                                TransportDiscontinuity::fec_region_reset);
                        }
                        decoder->reset();
                    }
                    decoder_transport_time_ms = 0.0F;
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
                    const float total_transport_time_ms =
                        decoder->timing().transport_time_ms;
                    const float window_transport_time_ms =
                        std::max(0.0F, total_transport_time_ms -
                                           decoder_transport_time_ms);
                    decoder_transport_time_ms = total_transport_time_ms;
                    const std::scoped_lock guard(mutex);
                    if (item.generation == latest_generation) {
                        latest.fec_work_time_ms = fec_work_ms;
                        latest.transport_bytes += window_transport_bytes;
                        latest.transport = decoder->stats();
                        latest.transport_work_time_ms = window_transport_time_ms;
                    }
                    fec_work_ms = 0.0F;
                    window_transport_bytes = 0;
                } else if (item.kind == FecItem::Kind::stats && decoder &&
                           decoder_generation == item.generation) {
                    const float total_transport_time_ms =
                        decoder->timing().transport_time_ms;
                    const float window_transport_time_ms =
                        std::max(0.0F, total_transport_time_ms -
                                           decoder_transport_time_ms);
                    decoder_transport_time_ms = total_transport_time_ms;
                    const std::scoped_lock guard(mutex);
                    if (item.generation == latest_generation) {
                        latest.fec_work_time_ms = fec_work_ms;
                        latest.transport_bytes += window_transport_bytes;
                        latest.transport = decoder->stats();
                        latest.transport_work_time_ms = window_transport_time_ms;
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
    impl_->ring_space.notify_all();
    impl_->ring_data.notify_all();
    wait_until_idle();
}

void StreamDecoder::wait_until_idle() {
    std::unique_lock lock(impl_->mutex);
    impl_->idle.wait(lock, [this] {
        return impl_->queue.empty() && !impl_->frontend_busy &&
               !impl_->demod_busy && !impl_->fec_worker_busy &&
               !impl_->flush_requested && !impl_->reset_requested &&
               impl_->fec_queue.empty() &&
               (!impl_->ring_closed ||
                impl_->ring_read_pos == impl_->ring_write_pos);
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
    impl_->ring_space.notify_all();
    impl_->ring_data.notify_all();
    // Wait for the front-end to consume the reset. reset() is called when a
    // source is closed or re-opened, and the caller may start submitting the
    // next stream immediately after it returns: an asynchronous clear would
    // race with those fresh submits and sweep the new data away (the
    // queue.clear() in the reset path cannot tell pre-reset from post-reset
    // blocks). Blocking here also lets live sources drop nothing: by the time
    // the caller re-opens, the pipeline is already drained and parked.
    std::unique_lock lock(impl_->mutex);
    impl_->idle.wait(lock,
                     [this] { return impl_->stopping || !impl_->reset_requested; });
}

void StreamDecoder::set_parameters(const ReceiverParameters &parameters) {
    impl_->analyzer.set_parameters(parameters);
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->parameters = parameters;
    }

    // A live SDR producer never becomes globally idle: it may keep filling the
    // input queue/ring while the GUI changes a demodulator setting. Wait only
    // for the frontend reset generation to be acknowledged, not for the
    // entire live pipeline to drain. Finite streams still use flush() and the
    // full wait_until_idle() path.
    const auto generation = impl_->latest_generation.load();
    reset();
    std::unique_lock lock(impl_->mutex);
    impl_->idle.wait(lock, [this, generation] {
        return impl_->stopping || impl_->latest_generation.load() != generation;
    });
}

void StreamDecoder::set_transport_callback(TransportCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->callback = std::move(callback);
}

void StreamDecoder::set_discontinuity_callback(DiscontinuityCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->discontinuity_callback = std::move(callback);
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
    statistics.ring_used_samples = impl_->ring_write_pos - impl_->ring_read_pos;
    statistics.ring_capacity_samples = impl_->ring.size();
    statistics.frontend_state =
        static_cast<WorkerState>(impl_->frontend_state.load());
    statistics.demod_state =
        static_cast<WorkerState>(impl_->demod_state.load());
    statistics.fec_state = static_cast<WorkerState>(impl_->fec_state.load());
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
