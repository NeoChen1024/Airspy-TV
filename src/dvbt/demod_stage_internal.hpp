#pragma once

#include "demod_stage.hpp"

#include "airspy_tv/dvbt/analysis_publisher.hpp"
#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dvbt/tps_decoder.hpp"
#include "airspy_tv/fftw_plan.hpp"
#include "airspy_tv/thread_name.hpp"

#include "clock_control_timeline.hpp"
#include "demod_dsp.hpp"
#include "ofdm_carrier.hpp"
#include "pipeline_config.hpp"

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
#include <cstdio>
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
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {

inline constexpr float minimum_power = 1.0e-12F;
inline constexpr std::size_t stats_window_symbols = 400;
inline constexpr std::size_t analysis_interval_symbols = 32;
inline constexpr float cfo_rebootstrap_phase_threshold =
    0.75F * std::numbers::pi_v<float>;
inline constexpr std::size_t cfo_rebootstrap_freeze_symbols = 2U * 68U;
inline constexpr std::size_t gate_window_symbols = 68;
inline constexpr std::array continual_2k{
    0,    48,   54,   87,   141,  156,  192,  201,  255,  279,  282,  333,
    432,  450,  483,  525,  531,  618,  636,  714,  759,  765,  780,  804,
    873,  888,  918,  939,  942,  969,  984,  1050, 1101, 1107, 1110, 1137,
    1140, 1146, 1206, 1269, 1323, 1377, 1491, 1683, 1704};
inline constexpr std::array tps_2k{34,   50,   209,  346,  413,  569,
                                   595,  688,  790,  901,  1073, 1219,
                                   1262, 1286, 1469, 1594, 1687};
inline constexpr std::size_t timing_pilot_spacing = 12;
inline constexpr std::size_t timing_filter_history_size = 7;
inline constexpr double timing_outlier_limit_samples = 24.0;

// These internal fragments share the enclosing namespace and constants above.
// Keep dependency order stable until they become self-contained headers.
// clang-format off
#include "symbol_postprocessor.hpp"
#include "timing_slope_tracker.hpp"
#include "demod_stage_state.hpp"
#include "ofdm_tracking_state.hpp"
// clang-format on

struct DemodStage::Impl { // NOLINT(clang-analyzer-optin.performance.Padding)
    using EqualizedCallback = StreamDecoder::EqualizedCallback;

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

    Impl(SampleChannel &samples, ClockControlTimeline &clock, FecStage &fec,
         AnalysisPublisher &analysis, Callbacks callbacks);
    ~Impl() noexcept;

    void stop() noexcept;
    void run_guarded() noexcept;
    void run();

    [[nodiscard]] double telemetry_elapsed_ms() const noexcept;
    [[nodiscard]] bool events_enabled() const noexcept;
    void emit_event(std::string event, DecoderEventSeverity severity,
                    std::uint64_t generation,
                    std::optional<std::uint64_t> resampled_sample,
                    std::optional<std::uint64_t> ofdm_symbol,
                    DecoderEventFields fields);
    void emit_diagnostic_event(DiagnosticEvent diagnostic,
                               std::uint64_t generation,
                               std::uint64_t source_epoch,
                               std::uint64_t demod_window_sequence,
                               std::uint64_t fec_session,
                               std::optional<std::uint64_t> tps_symbol_index);
    void fire_pending_discontinuity();
    [[nodiscard]] bool enqueue_fec(FecItem item) const;
    void reset_frontend_state() noexcept;

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
    [[nodiscard]] std::span<const std::complex<float>>
    demod_estimate_channel(DemodRuntimeState &state, const PilotLock &lock);
    [[nodiscard]] DemodFlow demod_process_tps(DemodRuntimeState &state);
    [[nodiscard]] DemodFlow demod_prepare_stream(DemodRuntimeState &state);
    [[nodiscard]] DemodInputFlow demod_read_symbol(DemodRuntimeState &state);
    void demod_finish_stream(DemodRuntimeState &state);
    [[nodiscard]] bool
    demod_dispatch_payload(DemodRuntimeState &state, const PilotLock &lock,
                           std::span<const std::complex<float>> channel);
    [[nodiscard]] static float
    demod_fec_floor(Constellation constellation) noexcept;

    SampleChannel &sample_channel;
    ClockControlTimeline &clock_control;
    FecStage &fec_stage;
    AnalysisPublisher &analysis_publisher;
    Callbacks callbacks;
    mutable std::mutex mutex;
    SyncState sync;
    ReceiverParameters parameters;
    std::optional<TransmissionMode> stable_mode;
    std::optional<GuardInterval> stable_guard;
    OfdmTrackingState frontend;
    StreamDecoderStats latest;
    std::atomic_bool telemetry_enabled{};
    TelemetryClock::time_point telemetry_started_at{TelemetryClock::now()};
    std::deque<TelemetryRecord> telemetry_queue;
    std::uint64_t frontend_telemetry_sequence{};
    std::uint64_t fec_telemetry_sequence{};
    std::uint64_t event_telemetry_sequence{};
    std::optional<TransportDiscontinuity> pending_discontinuity;
    StreamDecoder::EqualizedCallback equalized_callback;
    std::unique_ptr<SymbolPostprocessorPool> symbol_postprocessor;
    std::atomic<int> demod_state{static_cast<int>(WorkerState::idle)};
    std::thread worker;
};

} // namespace airspy_tv::dvbt
