#pragma once

#include "airspy_tv/diagnostic_event.hpp"
#include "airspy_tv/dvbt/receiver_parameters.hpp"
#include "airspy_tv/dvbt/transport_decoder.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace airspy_tv::dvbt {

using TelemetryClock = std::chrono::steady_clock;
using TimingMap = std::map<std::string, double, std::less<>>;

struct TelemetryEnvelope {
    std::uint64_t sequence{};
    std::uint64_t decoder_generation{};
    std::uint64_t source_epoch{};
    double wall_elapsed_ms{};
};

struct FrontendBlockTelemetry {
    TelemetryEnvelope envelope;
    std::uint32_t input_sample_rate_hz{};
    std::uint32_t channel_bandwidth_hz{};
    std::uint64_t source_begin_sample{};
    std::uint64_t source_end_sample{};
    std::uint64_t resampled_begin_sample{};
    std::uint64_t resampled_end_sample{};
    std::uint64_t input_complex_samples{};
    std::uint64_t resampled_complex_samples{};
    bool discontinuity_before{};
    bool abandoned{};
    std::uint64_t bootstrap_attempts{};
    std::uint64_t bootstrap_replayed_input_samples{};
    std::uint64_t bootstrap_retained_peak_samples{};
    double requested_ratio{};
    double effective_ratio{};
    double commanded_sro_ppm{};
    double applied_sro_ppm{};
    double commanded_cfo_hz{};
    double applied_cfo_hz{};
    std::uint64_t command_output_sample{};
    std::uint64_t command_input_sample{};
    std::uint64_t effective_input_sample{};
    std::uint64_t applied_input_sample{};
    std::uint64_t fixed_delay_samples{};
    std::uint64_t late_samples{};
    std::size_t pending_commands{};
    std::uint64_t cfo_command_output_sample{};
    std::uint64_t cfo_command_input_sample{};
    std::uint64_t cfo_effective_input_sample{};
    std::uint64_t cfo_applied_effective_input_sample{};
    std::uint64_t cfo_applied_input_sample{};
    std::uint64_t cfo_applied_output_sample{};
    std::uint64_t cfo_fixed_delay_samples{};
    std::uint64_t cfo_late_samples{};
    std::uint32_t cfo_input_sample_rate_hz{};
    std::size_t cfo_pending_commands{};
    std::uint64_t cfo_rebootstrap_requests{};
    std::uint64_t cfo_rebootstrap_count{};
    double cfo_rebootstrap_last_residual_hz{};
    std::uint64_t cfo_rebootstrap_output_sample{};
    std::uint64_t cfo_rebootstrap_source_sample{};
    TimingMap serial_wall_ms;
};

struct DemodWindowTelemetry {
    TelemetryEnvelope envelope;
    std::uint64_t demod_window_sequence{};
    std::uint64_t source_begin_sample{};
    std::uint64_t source_end_sample{};
    std::uint64_t resampled_begin_sample{};
    std::uint64_t resampled_midpoint_sample{};
    std::uint64_t resampled_end_sample{};
    std::uint32_t source_sample_rate_hz{};
    std::uint64_t symbol_count{};
    double signal_duration_seconds{};
    bool ofdm_locked{};
    bool tps_locked{};
    bool tps_ever_locked{};
    bool state_carried{};
    bool fec_skipped{};
    TransmissionMode transmission_mode{TransmissionMode::k2};
    GuardInterval guard_interval{GuardInterval::gi_1_32};
    Constellation constellation{Constellation::qpsk};
    CodeRate code_rate{CodeRate::rate_1_2};
    std::uint8_t hierarchy{};
    std::uint32_t fft_size{};
    std::uint32_t guard_size{};
    int carrier_bin_offset{};
    float acquisition_score{};
    std::size_t acquisition_start{};
    std::optional<double> mer_db;
    std::optional<double> fade_indicator;
    std::optional<double> tracked_cfo_hz;
    std::optional<double> residual_cfo_hz;
    std::optional<double> acquisition_fractional_cfo_hz;
    std::optional<double> acquisition_cfo_hz;
    int acquisition_carrier_bin_offset{};
    bool cfo_resampler_ready{};
    double cfo_command_hz{};
    double cfo_applied_hz{};
    std::uint64_t cfo_command_output_sample{};
    std::uint64_t cfo_command_input_sample{};
    std::uint64_t cfo_effective_input_sample{};
    std::uint64_t cfo_applied_effective_input_sample{};
    std::uint64_t cfo_applied_input_sample{};
    std::uint64_t cfo_applied_output_sample{};
    std::uint64_t cfo_fixed_delay_samples{};
    std::uint64_t cfo_late_samples{};
    std::uint32_t cfo_input_sample_rate_hz{};
    std::size_t cfo_pending_commands{};
    std::uint64_t cfo_rebootstrap_requests{};
    std::uint64_t cfo_rebootstrap_count{};
    std::optional<double> cfo_rebootstrap_last_residual_hz;
    std::uint64_t cfo_rebootstrap_output_sample{};
    std::uint64_t cfo_rebootstrap_source_sample{};
    std::optional<double> raw_timing_samples;
    std::optional<double> filtered_timing_samples;
    std::optional<double> physical_timing_samples;
    std::optional<double> observed_timing_drift_samples;
    std::optional<double> smoothed_timing_drift_samples;
    std::optional<double> estimated_sro_ppm;
    std::optional<double> timing_confidence;
    std::optional<double> cir_offset_samples;
    std::optional<double> cir_confidence;
    std::uint64_t timing_measurements{};
    std::uint64_t timing_accepted_measurements{};
    std::uint64_t timing_rejected_measurements{};
    bool timing_drift_ready{};
    bool sro_resampler_ready{};
    double sro_command_ppm{};
    double sro_applied_ppm{};
    double requested_resample_ratio{};
    double effective_resample_ratio{};
    std::uint64_t sro_command_output_sample{};
    std::uint64_t sro_command_input_sample{};
    std::uint64_t sro_effective_input_sample{};
    std::uint64_t sro_applied_input_sample{};
    std::uint64_t sro_fixed_delay_samples{};
    std::uint64_t sro_late_samples{};
    std::size_t sro_pending_commands{};
    std::uint64_t phase_discontinuities{};
    std::uint64_t pilot_expected_phase_checks{};
    std::uint64_t pilot_expected_phase_fast_accepts{};
    std::uint64_t pilot_expected_phase_fallbacks{};
    std::optional<double> pilot_expected_phase_confidence_mean;
    std::optional<double> pilot_expected_phase_confidence_min;
    double fft_plan_time_ms{};
    double wall_time_ms{};
    double serial_busy_time_ms{};
    TimingMap serial_busy_ms;
    TimingMap wait_ms;
    TimingMap nested_ms;
    TimingMap aggregate_worker_work_ms;
};

struct FecWindowTelemetry {
    TelemetryEnvelope envelope;
    std::uint64_t demod_window_sequence{};
    std::uint64_t fec_session{};
    std::uint64_t output_bytes_delta{};
    std::uint64_t output_bytes_cumulative{};
    TransportDecoderStats session;
    TransportDecoderStats delta;
    TransportDecoderStats cumulative;
    double fec_total_ms{};
    double transport_nested_ms{};
    TimingMap wait_ms;
    TimingMap nested_ms;
    TimingMap thread_cpu_ms;
    TimingMap aggregate_worker_work_ms;
};

using DecoderEventSeverity = DiagnosticEventSeverity;
using DecoderEventValue = DiagnosticEventValue;
using DecoderEventFields = DiagnosticEventFields;

struct DecoderEventTelemetry {
    TelemetryEnvelope envelope;
    std::string event;
    DecoderEventSeverity severity{DecoderEventSeverity::info};
    std::optional<std::uint64_t> source_sample;
    std::optional<std::uint64_t> resampled_sample;
    std::optional<std::uint64_t> ofdm_symbol;
    DecoderEventFields fields;
};

using TelemetryRecord =
    std::variant<FrontendBlockTelemetry, DemodWindowTelemetry,
                 FecWindowTelemetry, DecoderEventTelemetry>;

} // namespace airspy_tv::dvbt
