#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "airspy_tv/dsp/resampler_timeline.hpp"
#include "airspy_tv/dsp/vector_ops.hpp"
#include "airspy_tv/dvbt/analysis_publisher.hpp"
#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/tps_decoder.hpp"
#include "airspy_tv/fftw_plan.hpp"
#include "airspy_tv/thread_name.hpp"
#include "solid_resampler/frequency_translating_resampler.hpp"

#include "absolute_sample_ring.hpp"
#include "ofdm_carrier.hpp"
#include "resampler_config.hpp"

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
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float minimum_power = 1.0e-12F;
constexpr std::size_t acquisition_samples = 350'000;
constexpr std::size_t buffer_duration_denominator = 5;
constexpr std::size_t initial_symbol_queue_capacity = 256;

void add_transport_counters(TransportDecoderStats &destination,
                            const TransportDecoderStats &source) noexcept {
    destination.viterbi_bits += source.viterbi_bits;
    destination.pre_viterbi_error_bits += source.pre_viterbi_error_bits;
    destination.pre_viterbi_compared_bits += source.pre_viterbi_compared_bits;
    destination.post_viterbi_error_bits += source.post_viterbi_error_bits;
    destination.post_viterbi_compared_bits += source.post_viterbi_compared_bits;
    destination.rs_packets += source.rs_packets;
    destination.rs_uncorrectable_packets += source.rs_uncorrectable_packets;
    destination.tei_packets += source.tei_packets;
    destination.ts_packets += source.ts_packets;
    destination.outer_bit_offset = source.outer_bit_offset;
    destination.outer_deinterleaver_phase = source.outer_deinterleaver_phase;
    destination.outer_sync_distance = source.outer_sync_distance;
    destination.outer_rs_evidence = source.outer_rs_evidence;
    destination.rs_synchronized = source.rs_synchronized;
    destination.energy_synchronized = source.energy_synchronized;
    destination.viterbi_workers = source.viterbi_workers;
}

[[nodiscard]] TransportDecoderStats
transport_counter_delta(const TransportDecoderStats &current,
                        const TransportDecoderStats &previous) noexcept {
    TransportDecoderStats result = current;
    result.viterbi_bits -= previous.viterbi_bits;
    result.pre_viterbi_error_bits -= previous.pre_viterbi_error_bits;
    result.pre_viterbi_compared_bits -= previous.pre_viterbi_compared_bits;
    result.post_viterbi_error_bits -= previous.post_viterbi_error_bits;
    result.post_viterbi_compared_bits -= previous.post_viterbi_compared_bits;
    result.rs_packets -= previous.rs_packets;
    result.rs_uncorrectable_packets -= previous.rs_uncorrectable_packets;
    result.tei_packets -= previous.tei_packets;
    result.ts_packets -= previous.ts_packets;
    return result;
}

[[nodiscard]] const char *event_mode_name(const TransmissionMode mode) {
    return mode == TransmissionMode::k8 ? "8k" : "2k";
}

[[nodiscard]] const char *event_guard_name(const GuardInterval guard) {
    switch (guard) {
    case GuardInterval::gi_1_32:
        return "1/32";
    case GuardInterval::gi_1_16:
        return "1/16";
    case GuardInterval::gi_1_8:
        return "1/8";
    case GuardInterval::gi_1_4:
        return "1/4";
    }
    return "unknown";
}

[[nodiscard]] const char *
event_constellation_name(const Constellation constellation) {
    switch (constellation) {
    case Constellation::qpsk:
        return "qpsk";
    case Constellation::qam16:
        return "qam16";
    case Constellation::qam64:
        return "qam64";
    }
    return "unknown";
}

[[nodiscard]] const char *event_code_rate_name(const CodeRate rate) {
    switch (rate) {
    case CodeRate::rate_1_2:
        return "1/2";
    case CodeRate::rate_2_3:
        return "2/3";
    case CodeRate::rate_3_4:
        return "3/4";
    case CodeRate::rate_5_6:
        return "5/6";
    case CodeRate::rate_7_8:
        return "7/8";
    }
    return "unknown";
}

// Resampled-sample ring shared by the front-end thread (producer) and the
// demod thread (consumer). The ring is sized at run time from the input rate
// to retain ~0.2 s of baseband (the baseband rate is always <= the input
// rate), floored at the acquisition window plus a couple of symbols of
// lookahead and scheduling jitter — the proven 1 Mi behaviour for low input
// rates.
constexpr std::size_t ring_minimum_samples = 1'048'576;
constexpr std::size_t resampler_quantum_denominator = 20;
constexpr std::size_t minimum_sro_delay_denominator = 2;

[[nodiscard]] std::size_t
ring_capacity_for(const std::uint32_t sample_rate_hz) noexcept {
    return std::max(ring_minimum_samples,
                    (static_cast<std::size_t>(sample_rate_hz) +
                     buffer_duration_denominator - 1) /
                        buffer_duration_denominator);
}

[[nodiscard]] std::size_t
resampler_quantum_samples(const std::uint32_t sample_rate_hz) noexcept {
    return std::max<std::size_t>(1, (static_cast<std::size_t>(sample_rate_hz) +
                                     resampler_quantum_denominator - 1) /
                                        resampler_quantum_denominator);
}

[[nodiscard]] std::size_t
bootstrap_input_samples(const std::uint32_t input_rate_hz,
                        const std::uint32_t bandwidth_hz) noexcept {
    if (input_rate_hz == 0 || bandwidth_hz == 0) {
        return 0;
    }
    // Leave enough preview output beyond the acquisition window to absorb the
    // FIR startup transient without making bootstrap depend on caller blocks.
    constexpr std::size_t preview_margin_samples = 16'384;
    const auto output_samples =
        static_cast<long double>(acquisition_samples + preview_margin_samples);
    const long double output_rate =
        static_cast<long double>(bandwidth_hz) * 8.0L / 7.0L;
    return static_cast<std::size_t>(
        std::ceil(output_samples * static_cast<long double>(input_rate_hz) /
                  output_rate));
}

[[nodiscard]] std::uint64_t
sro_fixed_delay_samples(const std::uint32_t input_rate_hz,
                        const std::uint32_t bandwidth_hz) noexcept {
    const std::uint64_t minimum = (static_cast<std::uint64_t>(input_rate_hz) +
                                   minimum_sro_delay_denominator - 1) /
                                  minimum_sro_delay_denominator;
    if (bandwidth_hz == 0) {
        return minimum;
    }
    const long double output_rate =
        static_cast<long double>(bandwidth_hz) * 8.0L / 7.0L;
    const long double ring_input_equivalent =
        static_cast<long double>(ring_capacity_for(input_rate_hz)) *
        static_cast<long double>(input_rate_hz) / output_rate;
    const long double bounded_lead =
        std::ceil(ring_input_equivalent) +
        (2.0L *
         static_cast<long double>(resampler_quantum_samples(input_rate_hz)));
    return std::max(minimum, static_cast<std::uint64_t>(bounded_lead));
}
// Demod statistics window: 400 OFDM symbols (~0.6 s at 8K/guard-1/4), the
// report cadence for the CLI (the demod no longer reports once per submitted
// input chunk).
constexpr std::size_t stats_window_symbols = 400;
constexpr std::size_t analysis_interval_symbols = 32;
constexpr float cfo_rebootstrap_phase_threshold =
    0.75F * std::numbers::pi_v<float>;
constexpr std::size_t cfo_rebootstrap_freeze_symbols =
    static_cast<std::size_t>(2) * 68U;
// MER gate window: one TPS superframe (68 symbols). Windows whose mean MER
// falls below the constellation's decode floor are dropped and bracket a
// fresh FEC trellis at the region edges.
constexpr std::size_t gate_window_symbols = 68;
constexpr std::array continual_2k{
    0,    48,   54,   87,   141,  156,  192,  201,  255,  279,  282,  333,
    432,  450,  483,  525,  531,  618,  636,  714,  759,  765,  780,  804,
    873,  888,  918,  939,  942,  969,  984,  1050, 1101, 1107, 1110, 1137,
    1140, 1146, 1206, 1269, 1323, 1377, 1491, 1683, 1704};
constexpr std::array tps_2k{34,  50,   209,  346,  413,  569,  595,  688, 790,
                            901, 1073, 1219, 1262, 1286, 1469, 1594, 1687};

constexpr std::size_t timing_pilot_spacing = 12;
constexpr std::size_t timing_filter_history_size = 7;
constexpr double timing_outlier_limit_samples = 24.0;

[[nodiscard]] std::optional<double> estimate_scattered_timing_tau(
    const std::span<const std::complex<float>> channel, const std::size_t phase,
    const std::size_t maximum, const std::size_t fft_size) {
    // Use only the scattered pilots here.  `pilot_indices` also contains
    // continual carriers, whose non-uniform spacing gives each phase pair a
    // different unwrap period.  The scattered-pilot grid has one fixed
    // spacing, so all observations share the same N/12-sample ambiguity.
    std::array<double, 1024> estimates{};
    std::size_t estimate_count = 0;
    const std::size_t first = phase * 3;
    for (std::size_t left = first; left + timing_pilot_spacing <= maximum;
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
        const double slope =
            std::arg(channel[right] * std::conj(channel[left])) /
            static_cast<double>(timing_pilot_spacing);
        if (std::isfinite(slope)) {
            estimates[estimate_count++] = slope *
                                          static_cast<double>(fft_size) /
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

[[nodiscard]] float estimate_channel_notch_db(
    const std::span<const std::complex<float>> inverse_channel,
    const std::size_t phase, const std::size_t maximum) {
    std::vector<float> channel_db;
    channel_db.reserve((maximum / timing_pilot_spacing) + 1);
    const std::size_t edge_guard = maximum / 32;
    for (std::size_t carrier_index = phase * 3; carrier_index <= maximum;
         carrier_index += timing_pilot_spacing) {
        const float inverse_power = std::norm(inverse_channel[carrier_index]);
        if (carrier_index >= edge_guard &&
            carrier_index + edge_guard <= maximum &&
            inverse_power > minimum_power) {
            channel_db.push_back(-10.0F * std::log10(inverse_power));
        }
    }
    if (channel_db.empty()) {
        return 0.0F;
    }
    auto baseline = channel_db;
    auto median =
        baseline.begin() + static_cast<std::ptrdiff_t>(baseline.size() / 2);
    std::ranges::nth_element(baseline, median);
    const std::size_t lower_index =
        std::min(channel_db.size() - 1,
                 std::max<std::size_t>(1, channel_db.size() / 100));
    auto lower = channel_db.begin() + static_cast<std::ptrdiff_t>(lower_index);
    std::ranges::nth_element(channel_db, lower);
    return *lower - *median;
}

// The pilot phase slope is periodic in N/12 samples.  Keep its absolute
// branch continuous and reject isolated group-delay clicks before they reach
// either the long-term sample-clock loop or pilot phase verification.  The
// latter is particularly sensitive: one sample of ramp error is already
// several radians at the edge of the 8K carrier grid.
#include "timing_slope_tracker.hpp"

// Phase-only re-lock at a fixed carrier offset, used while the fade
// indicator is marginal (0.25 < fi <= 0.5): the grid is still up but the
// channel is degraded enough that a full offset search is noise-driven and
// can latch a multipath alias (545's 0 -> 3, 557's 0 -> -1). Re-verify only
// the mod-4 scattered-pilot phase at the frozen offset; the offset itself is
// re-searched only when fi > 0.5 (healthy) or after a fade.
[[nodiscard]] int
lock_phase_at_offset(const std::span<const std::complex<float>> fft,
                     const std::size_t maximum, const int offset,
                     const std::optional<double> timing_tau) {
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
            dephase_slope = 2.0 * std::numbers::pi_v<double> * *timing_tau /
                            static_cast<double>(fft.size());
        } else {
            double ramp_sum = 0.0;
            std::size_t ramp_count = 0;
            std::size_t previous_pilot =
                std::numeric_limits<std::size_t>::max();
            for (auto pilot = static_cast<std::size_t>(phase) * 3U;
                 pilot <= maximum; pilot += 12) {
                if (previous_pilot != std::numeric_limits<std::size_t>::max()) {
                    const auto left =
                        active_carrier(fft, previous_pilot, maximum, offset);
                    const auto right =
                        active_carrier(fft, pilot, maximum, offset);
                    const float left_value = pilot_prbs[previous_pilot] == 0U
                                                 ? 4.0F / 3.0F
                                                 : -4.0F / 3.0F;
                    const float right_value =
                        pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                    if (std::norm(left) > 0.0F && std::norm(right) > 0.0F) {
                        const double difference =
                            std::arg(right * std::conj(left) *
                                     std::complex<float>(
                                         right_value * left_value, 0.0F));
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
                          static_cast<double>(ramp_count * timing_pilot_spacing)
                    : 0.0;
        }
        for (auto pilot = static_cast<std::size_t>(phase) * 3U;
             pilot <= maximum; pilot += 12) {
            const float value =
                pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
            const auto dephase =
                static_cast<float>(dephase_slope * static_cast<double>(pilot));
            correlation +=
                value * std::conj(std::polar(1.0F, dephase) *
                                  active_carrier(fft, pilot, maximum, offset));
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

#include "stream_decoder_components.hpp"
#include "stream_decoder_demod_state.hpp"
} // namespace

#include "fec_stage_item.hpp"
#include "ofdm_tracking_state.hpp"

struct StreamDecoder::Impl { // NOLINT(clang-analyzer-optin.performance.Padding)
    struct Block {
        std::vector<std::int16_t> samples;
        std::uint32_t rate{};
        std::uint32_t bandwidth{};
        std::uint64_t generation{};
        InputSampleStamp stamp;
    };

    struct ScheduledSroCommand {
        std::uint64_t generation{};
        std::uint64_t source_epoch{};
        std::uint64_t command_output_sample{};
        std::uint64_t command_input_sample{};
        std::uint64_t effective_input_sample{};
        std::uint64_t fixed_delay_samples{};
        double target_ppm{};
    };

    struct ScheduledCfoCommand {
        std::uint64_t generation{};
        std::uint64_t source_epoch{};
        std::uint64_t command_output_sample{};
        std::uint64_t command_input_sample{};
        std::uint64_t effective_input_sample{};
        std::uint64_t fixed_delay_samples{};
        double target_hz{};
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
        int acquisition_pilot_phase{};
        float score{};
        TransmissionMode mode{TransmissionMode::k8};
        GuardInterval guard{GuardInterval::gi_1_4};
        std::size_t fft_size{};
        std::size_t guard_size{};
        std::uint32_t bandwidth{};
        float resampled_rate{};
    };

    mutable std::mutex mutex;
    std::condition_variable input_ready;
    std::condition_variable input_not_full;
    std::condition_variable ring_data;
    std::condition_variable ring_space;
    std::condition_variable fec_ready;
    std::condition_variable fec_not_full;
    std::condition_variable idle;
    std::condition_variable reset_acknowledged;
    std::deque<Block> queue;
    std::deque<FecItem> fec_queue;
    std::size_t queued_complex_samples{};
    std::size_t input_queue_capacity_samples{};
    std::size_t fec_queue_capacity{initial_symbol_queue_capacity};
    InputSampleTimeline fallback_input_timeline;
    // Circular buffer of resampled samples. Positions are absolute uint64
    // stream offsets; the ring retains [ring_read_pos, ring_write_pos).
    AbsoluteSampleRing ring;
    std::uint64_t &ring_write_pos{ring.write_position};
    std::uint64_t &ring_read_pos{ring.read_position};
    bool &ring_closed{ring.closed};
    SyncState sync;
    dsp::ResamplerRateTimeline resampler_timeline;
    std::deque<ScheduledSroCommand> scheduled_sro_commands;
    std::deque<ScheduledCfoCommand> scheduled_cfo_commands;
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
    AnalysisPublisher analysis_publisher;
    std::optional<TransmissionMode> stable_mode;
    std::optional<GuardInterval> stable_guard;
    OfdmTrackingState frontend;
    StreamDecoderStats latest;
    std::atomic_bool telemetry_enabled;
    TelemetryClock::time_point telemetry_started_at{TelemetryClock::now()};
    std::deque<TelemetryRecord> telemetry_queue;
    std::uint64_t frontend_telemetry_sequence{};
    std::uint64_t fec_telemetry_sequence{};
    std::uint64_t event_telemetry_sequence{};
    std::atomic<bool> cancel_requested;
    // The demod estimates source SRO; the front-end owns and applies the
    // common resampler. These atomics are the only cross-thread control path.
    std::atomic<double> sro_resampler_command_ppm;
    std::atomic<double> sro_resampler_applied_ppm;
    std::atomic<bool> sro_resampler_ready;
    std::atomic<double> cfo_resampler_command_hz;
    std::atomic<double> cfo_resampler_applied_hz;
    std::atomic<bool> cfo_resampler_ready;
    std::atomic<bool> cfo_rebootstrap_requested{false};
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
    std::uint64_t reset_request_generation{};
    std::uint64_t demod_reset_generation{};
    std::uint64_t completed_reset_generation{};
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
    std::atomic<std::uint64_t> latest_generation;
    std::exception_ptr terminal_exception;
    std::string terminal_error;
    std::unique_ptr<SymbolPostprocessorPool> symbol_postprocessor;
    std::thread frontend_thread;
    std::thread demod_thread;
    std::thread fec_thread;

    [[nodiscard]] double telemetry_elapsed_ms() const noexcept {
        return std::chrono::duration<double, std::milli>(TelemetryClock::now() -
                                                         telemetry_started_at)
            .count();
    }

    [[nodiscard]] bool events_enabled() const noexcept {
        return telemetry_enabled.load(std::memory_order_relaxed);
    }

    void emit_event(std::string event, DecoderEventSeverity severity,
                    std::uint64_t generation,
                    std::optional<std::uint64_t> resampled_sample,
                    std::optional<std::uint64_t> ofdm_symbol,
                    DecoderEventFields fields) {
        if (!telemetry_enabled.load(std::memory_order_relaxed)) {
            return;
        }
        const std::scoped_lock lock(mutex);
        if (!telemetry_enabled.load(std::memory_order_relaxed)) {
            return;
        }
        std::uint64_t source_epoch = 0;
        std::optional<std::uint64_t> source_sample;
        if (resampled_sample.has_value()) {
            if (const auto mapped =
                    resampler_timeline.input_at_output(*resampled_sample)) {
                source_epoch = mapped->stream_epoch;
                source_sample = mapped->input_sample;
            }
        }
        DecoderEventTelemetry record;
        record.envelope = {
            .sequence = ++event_telemetry_sequence,
            .decoder_generation = generation,
            .source_epoch = source_epoch,
            .wall_elapsed_ms = telemetry_elapsed_ms(),
        };
        record.event = std::move(event);
        record.severity = severity;
        record.source_sample = source_sample;
        record.resampled_sample = resampled_sample;
        record.ofdm_symbol = ofdm_symbol;
        record.fields = std::move(fields);
        telemetry_queue.emplace_back(std::move(record));
    }

    void
    emit_diagnostic_event(DiagnosticEvent diagnostic,
                          const std::uint64_t generation,
                          const std::uint64_t source_epoch,
                          const std::uint64_t demod_window_sequence,
                          const std::uint64_t fec_session,
                          const std::optional<std::uint64_t> tps_symbol_index) {
        if (!telemetry_enabled.load(std::memory_order_relaxed)) {
            return;
        }
        diagnostic.fields.emplace("fec_session", fec_session);
        if (demod_window_sequence != 0) {
            diagnostic.fields.emplace("demod_window_sequence",
                                      demod_window_sequence);
        }
        if (tps_symbol_index.has_value()) {
            diagnostic.fields.emplace("tps_symbol_index", *tps_symbol_index);
        }
        const std::scoped_lock lock(mutex);
        if (!telemetry_enabled.load(std::memory_order_relaxed) ||
            generation != latest_generation.load(std::memory_order_relaxed)) {
            return;
        }
        DecoderEventTelemetry record;
        record.envelope = {
            .sequence = ++event_telemetry_sequence,
            .decoder_generation = generation,
            .source_epoch = source_epoch,
            .wall_elapsed_ms = telemetry_elapsed_ms(),
        };
        record.event = std::move(diagnostic.name);
        record.severity = diagnostic.severity;
        record.fields = std::move(diagnostic.fields);
        telemetry_queue.emplace_back(std::move(record));
    }

    Impl()
        : ring(ring_minimum_samples), frontend_thread([this] {
              set_current_thread_name("dvbt-frontend");
              run_guarded("frontend", [this] { run_frontend(); });
          }),
          demod_thread([this] {
              set_current_thread_name("dvbt-demod");
              run_guarded("demod", [this] { run_demod(); });
          }),
          fec_thread([this] {
              set_current_thread_name("dvbt-fec");
              run_guarded("fec", [this] { run_fec(); });
          }) {}
    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;
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
        reset_acknowledged.notify_all();
        frontend_thread.join();
        demod_thread.join();
        fec_thread.join();
    }

    void notify_all_waiters() noexcept {
        input_ready.notify_all();
        input_not_full.notify_all();
        ring_data.notify_all();
        ring_space.notify_all();
        fec_ready.notify_all();
        fec_not_full.notify_all();
        reset_acknowledged.notify_all();
        idle.notify_all();
    }

    template <typename Function>
    void run_guarded(const char *stage, Function &&function) noexcept {
        try {
            std::forward<Function>(function)();
        } catch (...) {
            const std::exception_ptr error = std::current_exception();
            std::string message = std::string(stage) + " worker failed";
            try {
                std::rethrow_exception(error);
            } catch (const std::exception &exception) {
                message += ": ";
                message += exception.what();
            } catch (...) {
                message += ": unknown exception";
            }
            {
                const std::scoped_lock lock(mutex);
                if (!terminal_exception) {
                    terminal_exception = error;
                    terminal_error = std::move(message);
                }
                stopping = true;
                cancel_requested = true;
            }
            notify_all_waiters();
        }
    }

    void reset_frontend_state() noexcept {
        frontend.valid = false;
        frontend.fft_size = 0;
        frontend.guard_size = 0;
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

    void demod_cold_seed(DemodRuntimeState &state);
    void demod_build_grid(DemodRuntimeState &state);
    void demod_reset_timing(DemodRuntimeState &state,
                            std::size_t tracker_fft_size,
                            bool reset_window_cir);
    [[nodiscard]] bool demod_handle_sync_change(DemodRuntimeState &state);
    [[nodiscard]] float demod_run_acquisition(DemodRuntimeState &state,
                                              bool wait_for_data = false);
    void demod_process_batch(DemodRuntimeState &state,
                             std::vector<PostprocessedSymbol> batch);
    [[nodiscard]] bool demod_start_decoder(DemodRuntimeState &state);
    [[nodiscard]] DemodWindowMetrics
    demod_update_timing_window(DemodRuntimeState &state);
    void demod_publish_stats_window(DemodRuntimeState &state);
    static void demod_reset_stats_window(DemodRuntimeState &state);
    void demod_advance_symbol(DemodRuntimeState &state);
    [[nodiscard]] bool demod_maybe_reacquire(DemodRuntimeState &state);
    [[nodiscard]] bool demod_request_cfo_rebootstrap(DemodRuntimeState &state,
                                                     const char *reason,
                                                     float residual_phase);
    void demod_execute_fft_and_measure_cfo(DemodRuntimeState &state);
    [[nodiscard]] std::optional<PilotLock>
    demod_lock_pilots(DemodRuntimeState &state);
    [[nodiscard]] std::vector<std::complex<float>>
    demod_estimate_channel(DemodRuntimeState &state, const PilotLock &lock);
    [[nodiscard]] DemodFlow demod_process_tps(DemodRuntimeState &state);
    [[nodiscard]] DemodFlow demod_prepare_stream(DemodRuntimeState &state);
    [[nodiscard]] DemodInputFlow demod_read_symbol(DemodRuntimeState &state);
    void demod_finish_stream(DemodRuntimeState &state);
    [[nodiscard]] bool
    demod_dispatch_payload(DemodRuntimeState &state, const PilotLock &lock,
                           const std::vector<std::complex<float>> &channel);
    [[nodiscard]] static float
    demod_fec_floor(Constellation constellation) noexcept;
    void run_frontend();
    void run_demod();
    void run_fec();
};

#include "stream_decoder_demod.hpp"
#include "stream_decoder_demod_output.hpp"
#include "stream_decoder_demod_session.hpp"
#include "stream_decoder_demod_symbol.hpp"
#include "stream_decoder_fec.hpp"
#include "stream_decoder_frontend.hpp"

StreamDecoder::StreamDecoder() : impl_(std::make_unique<Impl>()) {}
StreamDecoder::~StreamDecoder() noexcept = default;

void StreamDecoder::submit(const std::span<const std::int16_t> interleaved_iq,
                           const std::uint32_t sample_rate_hz,
                           const std::uint32_t channel_bandwidth_hz,
                           InputSampleStamp stamp) {
    if (interleaved_iq.empty() || sample_rate_hz == 0 ||
        (interleaved_iq.size() % 2) != 0) {
        return;
    }
    const std::size_t incoming_samples = interleaved_iq.size() / 2;
    if (!stamp.valid_for(incoming_samples, sample_rate_hz)) {
        stamp = impl_->fallback_input_timeline.stamp(incoming_samples,
                                                     sample_rate_hz);
    }
    if (!impl_->analysis_publisher.locked()) {
        impl_->analyzer.submit(interleaved_iq, sample_rate_hz,
                               channel_bandwidth_hz);
    }
    const std::scoped_lock lock(impl_->mutex);
    impl_->input_queue_capacity_samples =
        std::max(buffered_input_samples(sample_rate_hz), incoming_samples);
    if (impl_->queued_complex_samples + incoming_samples >
        impl_->input_queue_capacity_samples) {
        ++impl_->latest.dropped_blocks;
        return;
    }
    impl_->queue.push_back({std::vector<std::int16_t>(interleaved_iq.begin(),
                                                      interleaved_iq.end()),
                            sample_rate_hz, channel_bandwidth_hz,
                            impl_->latest_generation.load(), stamp});
    impl_->queued_complex_samples += incoming_samples;
    impl_->input_ready.notify_one();
}

void StreamDecoder::submit_blocking(
    const std::span<const std::int16_t> interleaved_iq,
    const std::uint32_t sample_rate_hz,
    const std::uint32_t channel_bandwidth_hz, InputSampleStamp stamp) {
    if (interleaved_iq.empty() || sample_rate_hz == 0 ||
        (interleaved_iq.size() % 2) != 0) {
        return;
    }
    const std::size_t incoming_samples = interleaved_iq.size() / 2;
    if (!stamp.valid_for(incoming_samples, sample_rate_hz)) {
        stamp = impl_->fallback_input_timeline.stamp(incoming_samples,
                                                     sample_rate_hz);
    }
    std::unique_lock lock(impl_->mutex);
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
                            sample_rate_hz, channel_bandwidth_hz,
                            impl_->latest_generation.load(), stamp});
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
        return impl_->stopping ||
               (impl_->queue.empty() && !impl_->frontend_busy &&
                !impl_->demod_busy && !impl_->fec_worker_busy &&
                !impl_->flush_requested && !impl_->reset_requested &&
                impl_->fec_queue.empty() &&
                (!impl_->ring_closed ||
                 impl_->ring_read_pos == impl_->ring_write_pos));
    });
}

void StreamDecoder::request_reset() {
    impl_->analyzer.reset();
    impl_->analysis_publisher.reset();
    impl_->fallback_input_timeline.mark_discontinuity();
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->cancel_requested = true;
        impl_->sro_resampler_command_ppm.store(0.0, std::memory_order_relaxed);
        impl_->sro_resampler_applied_ppm.store(0.0, std::memory_order_relaxed);
        impl_->sro_resampler_ready.store(false, std::memory_order_relaxed);
        impl_->cfo_resampler_command_hz.store(0.0, std::memory_order_relaxed);
        impl_->cfo_resampler_applied_hz.store(0.0, std::memory_order_relaxed);
        impl_->cfo_resampler_ready.store(false, std::memory_order_relaxed);
        impl_->cfo_rebootstrap_requested.store(false,
                                               std::memory_order_release);
        const std::uint64_t generation =
            impl_->latest_generation.fetch_add(1) + 1;
        impl_->reset_request_generation = generation;
        impl_->reset_requested = true;
        impl_->queue.clear();
        impl_->fec_queue.clear();
        impl_->scheduled_sro_commands.clear();
        impl_->scheduled_cfo_commands.clear();
        impl_->resampler_timeline.clear();
        impl_->queued_complex_samples = 0;
        impl_->sync.valid = false;
        ++impl_->sync.version;
        impl_->latest = {};
        impl_->current_bandwidth = 0;
        impl_->demod_busy_time_sum_ms = 0.0;
    }
    impl_->fec_not_full.notify_all();
    impl_->input_ready.notify_one();
    impl_->ring_space.notify_all();
    impl_->ring_data.notify_all();
}

void StreamDecoder::reset() {
    request_reset();
    // Wait for the front-end to consume the reset. reset() is called when a
    // source is closed or re-opened, and the caller may start submitting the
    // next stream immediately after it returns: an asynchronous clear would
    // race with those fresh submits and sweep the new data away (the
    // queue.clear() in the reset path cannot tell pre-reset from post-reset
    // blocks). Blocking here also lets live sources drop nothing: by the time
    // the caller re-opens, the pipeline is already drained and parked.
    std::unique_lock lock(impl_->mutex);
    const std::uint64_t generation = impl_->reset_request_generation;
    impl_->idle.wait(lock, [this, generation] {
        return impl_->stopping ||
               impl_->completed_reset_generation >= generation;
    });
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
    statistics.decoder_generation = impl_->latest_generation.load();
    statistics.failed = impl_->terminal_exception != nullptr;
    statistics.error = impl_->terminal_error;
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
    statistics.sro_resampler_command_ppm = static_cast<float>(
        impl_->sro_resampler_command_ppm.load(std::memory_order_relaxed));
    statistics.sro_resampler_applied_ppm = static_cast<float>(
        impl_->sro_resampler_applied_ppm.load(std::memory_order_relaxed));
    statistics.sro_resampler_ready =
        impl_->sro_resampler_ready.load(std::memory_order_relaxed);
    statistics.cfo_resampler_command_hz = static_cast<float>(
        impl_->cfo_resampler_command_hz.load(std::memory_order_relaxed));
    statistics.cfo_resampler_applied_hz = static_cast<float>(
        impl_->cfo_resampler_applied_hz.load(std::memory_order_relaxed));
    statistics.cfo_resampler_ready =
        impl_->cfo_resampler_ready.load(std::memory_order_relaxed);
    statistics.processing = impl_->frontend_busy || impl_->demod_busy ||
                            impl_->fec_worker_busy || !impl_->queue.empty() ||
                            !impl_->fec_queue.empty();
    return statistics;
}

void StreamDecoder::set_telemetry_enabled(
    const bool enabled, const TelemetryClock::time_point run_started_at) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->telemetry_enabled.store(enabled, std::memory_order_relaxed);
    impl_->telemetry_started_at = run_started_at;
    impl_->telemetry_queue.clear();
    impl_->frontend_telemetry_sequence = 0;
    impl_->fec_telemetry_sequence = 0;
    impl_->event_telemetry_sequence = 0;
}

std::vector<TelemetryRecord> StreamDecoder::drain_telemetry() {
    const std::scoped_lock lock(impl_->mutex);
    std::vector<TelemetryRecord> result;
    result.reserve(impl_->telemetry_queue.size());
    while (!impl_->telemetry_queue.empty()) {
        result.push_back(std::move(impl_->telemetry_queue.front()));
        impl_->telemetry_queue.pop_front();
    }
    return result;
}

SignalSnapshot StreamDecoder::signal_snapshot() const {
    const auto analysis = analysis_snapshot();
    const auto statistics = stats();
    SignalSnapshot result;
    result.constellation_count =
        std::min(analysis.point_count, result.constellation.size());
    std::copy_n(analysis.points.begin(), result.constellation_count,
                result.constellation.begin());
    result.mer_db = analysis.mer_db;
    result.snr_db = analysis.cp_snr_db;
    result.deepest_notch_db = analysis.deepest_notch_db;
    result.carrier_offset_hz = analysis.carrier_offset_hz;
    const float fft_size =
        analysis.mode == TransmissionMode::k8 ? 8192.0F : 2048.0F;
    std::uint32_t channel_bandwidth_hz = 0;
    {
        const std::scoped_lock lock(impl_->mutex);
        channel_bandwidth_hz = impl_->parameters.channel_bandwidth_hz;
    }
    const float sample_rate_hz =
        static_cast<float>(channel_bandwidth_hz) * (8.0F / 7.0F);
    result.carrier_offset_limit_hz = sample_rate_hz / (2.0F * fft_size);
    result.sequence = analysis.sequence;
    result.signal_locked = analysis.locked;
    result.transport_locked = statistics.transport.rs_synchronized &&
                              statistics.transport.ts_packets != 0;
    return result;
}

PipelineSnapshot StreamDecoder::pipeline_snapshot() const {
    const auto statistics = stats();
    const auto queue_fraction = [](const std::size_t used,
                                   const std::size_t capacity) {
        return capacity == 0 ? 0.0F
                             : std::clamp(static_cast<float>(used) /
                                              static_cast<float>(capacity),
                                          0.0F, 1.0F);
    };

    PipelineSnapshot result;
    result.stages[0] = PipelineStageSnapshot{
        .name = "IQ queue",
        .queue_fraction =
            queue_fraction(statistics.queued_input_samples,
                           statistics.input_queue_capacity_samples),
        .workers = statistics.resample_workers,
        .queue_valid = statistics.input_queue_capacity_samples != 0,
    };
    result.stages[1] = PipelineStageSnapshot{
        .name = "Demod",
        .busy_fraction = statistics.demod_busy_fraction,
        .workers = statistics.symbol_workers,
        .busy_valid = true,
    };
    result.stages[2] = PipelineStageSnapshot{
        .name = "FEC queue",
        .queue_fraction = queue_fraction(statistics.queued_symbols,
                                         statistics.symbol_queue_capacity),
        .workers = statistics.transport.viterbi_workers,
        .queue_valid = statistics.symbol_queue_capacity != 0,
    };
    result.stage_count = 3;
    result.processing_realtime_ratio = statistics.processing_realtime_ratio;
    result.dropped_blocks = statistics.dropped_blocks;
    result.transport_bytes = statistics.transport_bytes;
    result.sequence = statistics.processed_chunks;
    result.processing = statistics.processing;
    result.failed = statistics.failed;
    result.error = statistics.error;
    return result;
}

SignalAnalysisSnapshot StreamDecoder::analysis_snapshot() const {
    auto snapshot = impl_->analysis_publisher.snapshot();
    if (snapshot.locked) {
        return snapshot;
    }
    snapshot = impl_->analyzer.snapshot();
    snapshot.source = snapshot.locked ? SignalAnalysisSource::prelock_monitor
                                      : SignalAnalysisSource::none;
    return snapshot;
}

void StreamDecoder::set_signal_smoothing(const bool enabled, const int speed) {
    impl_->analyzer.set_snr_smoothing(enabled, speed);
    impl_->analysis_publisher.set_smoothing(enabled, speed);
}

} // namespace airspy_tv::dvbt
