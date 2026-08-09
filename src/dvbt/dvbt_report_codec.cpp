#include "dvbt_report_codec.hpp"

#include "airspy_tv/jsonl.hpp"

#include <nlohmann/json.hpp>

#include <format>
#include <optional>
#include <string_view>
#include <type_traits>

namespace airspy_tv {
namespace {

using dvbt::CodeRate;
using dvbt::Constellation;
using dvbt::GuardInterval;
using dvbt::TransmissionMode;
using dvbt::TransportDecoderStats;
using nlohmann::json;

[[nodiscard]] const char *mode_name(const TransmissionMode value) noexcept {
    return value == TransmissionMode::k8 ? "8k" : "2k";
}

[[nodiscard]] const char *guard_name(const GuardInterval value) noexcept {
    switch (value) {
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

[[nodiscard]] const char *constellation_name(const Constellation value) {
    switch (value) {
    case Constellation::qpsk:
        return "qpsk";
    case Constellation::qam16:
        return "qam16";
    case Constellation::qam64:
        return "qam64";
    }
    return "unknown";
}

[[nodiscard]] const char *code_rate_name(const CodeRate value) {
    switch (value) {
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

[[nodiscard]] json optional_number(const std::optional<double> value) {
    return value.has_value() ? json_finite_or_null(*value) : json(nullptr);
}

[[nodiscard]] json envelope(const dvbt::TelemetryEnvelope &value,
                            const std::string_view record_type) {
    return {{"schema_version", 0},
            {"record_type", record_type},
            {"mode", "dvbt"},
            {"sequence", value.sequence},
            {"decoder_generation", value.decoder_generation},
            {"source_epoch", value.source_epoch},
            {"wall_elapsed_ms", json_finite_or_null(value.wall_elapsed_ms)}};
}

[[nodiscard]] json timing_map(const dvbt::TimingMap &values) {
    json result = json::object();
    for (const auto &[name, value] : values) {
        result[name] = json_finite_or_null(value);
    }
    return result;
}

[[nodiscard]] json transport_stats(const TransportDecoderStats &stats) {
    return {{"viterbi_bits", stats.viterbi_bits},
            {"pre_viterbi_error_bits", stats.pre_viterbi_error_bits},
            {"pre_viterbi_compared_bits", stats.pre_viterbi_compared_bits},
            {"post_viterbi_error_bits", stats.post_viterbi_error_bits},
            {"post_viterbi_compared_bits", stats.post_viterbi_compared_bits},
            {"rs_packets", stats.rs_packets},
            {"rs_clean_packets", stats.rs_clean_packets},
            {"rs_corrected_packets", stats.rs_corrected_packets},
            {"rs_uncorrectable_packets", stats.rs_uncorrectable_packets},
            {"tei_packets", stats.tei_packets},
            {"ts_packets", stats.ts_packets},
            {"outer_bit_offset", stats.outer_bit_offset},
            {"outer_deinterleaver_phase", stats.outer_deinterleaver_phase},
            {"outer_sync_distance", stats.outer_sync_distance},
            {"outer_rs_evidence", stats.outer_rs_evidence},
            {"rs_synchronized", stats.rs_synchronized},
            {"energy_synchronized", stats.energy_synchronized},
            {"viterbi_workers", stats.viterbi_workers}};
}

[[nodiscard]] json encode(const dvbt::FrontendBlockTelemetry &record) {
    json result = envelope(record.envelope, "frontend_block");
    result["input"] = {
        {"sample_rate_hz", record.input_sample_rate_hz},
        {"channel_bandwidth_hz", record.channel_bandwidth_hz},
        {"source_begin_sample", record.source_begin_sample},
        {"source_end_sample", record.source_end_sample},
        {"complex_samples", record.input_complex_samples},
        {"discontinuity_before", record.discontinuity_before},
        {"abandoned", record.abandoned},
        {"bootstrap",
         {{"attempts", record.bootstrap_attempts},
          {"replayed_input_samples", record.bootstrap_replayed_input_samples},
          {"retained_peak_samples", record.bootstrap_retained_peak_samples}}}};
    result["resampler"] = {
        {"resampled_begin_sample", record.resampled_begin_sample},
        {"resampled_end_sample", record.resampled_end_sample},
        {"complex_samples", record.resampled_complex_samples},
        {"requested_ratio", json_finite_or_null(record.requested_ratio)},
        {"effective_ratio", json_finite_or_null(record.effective_ratio)},
        {"commanded_sro_ppm", json_finite_or_null(record.commanded_sro_ppm)},
        {"applied_sro_ppm", json_finite_or_null(record.applied_sro_ppm)},
        {"command_output_sample", record.command_output_sample},
        {"command_input_sample", record.command_input_sample},
        {"effective_input_sample", record.effective_input_sample},
        {"applied_input_sample", record.applied_input_sample},
        {"fixed_delay_samples", record.fixed_delay_samples},
        {"late_samples", record.late_samples},
        {"pending_commands", record.pending_commands},
        {"frequency",
         {{"commanded_cfo_hz", json_finite_or_null(record.commanded_cfo_hz)},
          {"applied_cfo_hz", json_finite_or_null(record.applied_cfo_hz)},
          {"command_output_sample", record.cfo_command_output_sample},
          {"command_input_sample", record.cfo_command_input_sample},
          {"effective_input_sample", record.cfo_effective_input_sample},
          {"applied_effective_input_sample",
           record.cfo_applied_effective_input_sample},
          {"applied_input_sample", record.cfo_applied_input_sample},
          {"applied_output_sample", record.cfo_applied_output_sample},
          {"fixed_delay_samples", record.cfo_fixed_delay_samples},
          {"late_samples", record.cfo_late_samples},
          {"input_sample_rate_hz", record.cfo_input_sample_rate_hz},
          {"pending_commands", record.cfo_pending_commands},
          {"rebootstrap",
           {{"requests", record.cfo_rebootstrap_requests},
            {"completed", record.cfo_rebootstrap_count},
            {"last_residual_hz",
             json_finite_or_null(record.cfo_rebootstrap_last_residual_hz)},
            {"output_sample", record.cfo_rebootstrap_output_sample},
            {"source_sample", record.cfo_rebootstrap_source_sample}}}}}};
    result["timing_ms"] = {{"serial_wall", timing_map(record.serial_wall_ms)}};
    return result;
}

[[nodiscard]] json encode(const dvbt::DemodWindowTelemetry &record) {
    json result = envelope(record.envelope, "demod_window");
    result["demod_window_sequence"] = record.demod_window_sequence;
    result["interval"] = {
        {"source_begin_sample", record.source_begin_sample},
        {"source_end_sample", record.source_end_sample},
        {"source_sample_rate_hz", record.source_sample_rate_hz},
        {"resampled_begin_sample", record.resampled_begin_sample},
        {"resampled_midpoint_sample", record.resampled_midpoint_sample},
        {"resampled_end_sample", record.resampled_end_sample},
        {"ofdm_symbols", record.symbol_count},
        {"signal_duration_seconds",
         json_finite_or_null(record.signal_duration_seconds)}};
    result["lock"] = {{"ofdm", record.ofdm_locked},
                      {"tps", record.tps_locked},
                      {"tps_ever", record.tps_ever_locked},
                      {"state_carried", record.state_carried},
                      {"fec_skipped", record.fec_skipped}};
    result["parameters"] = {
        {"transmission_mode", mode_name(record.transmission_mode)},
        {"guard_interval", guard_name(record.guard_interval)},
        {"constellation", constellation_name(record.constellation)},
        {"code_rate", code_rate_name(record.code_rate)},
        {"hierarchy", record.hierarchy},
        {"fft_size", record.fft_size},
        {"guard_size", record.guard_size}};
    result["acquisition"] = {
        {"carrier_bin_offset", record.carrier_bin_offset},
        {"initial_carrier_bin_offset", record.acquisition_carrier_bin_offset},
        {"fractional_cfo_hz",
         optional_number(record.acquisition_fractional_cfo_hz)},
        {"cfo_hz", optional_number(record.acquisition_cfo_hz)},
        {"score", json_finite_or_null(record.acquisition_score)},
        {"start", record.acquisition_start}};
    result["signal"] = {
        {"mer_db", optional_number(record.mer_db)},
        {"fade_indicator", optional_number(record.fade_indicator)},
        {"tracked_cfo_hz", optional_number(record.tracked_cfo_hz)},
        {"residual_cfo_hz", optional_number(record.residual_cfo_hz)}};
    result["frequency_tracking"] = {
        {"resampler_ready", record.cfo_resampler_ready},
        {"commanded_cfo_hz", json_finite_or_null(record.cfo_command_hz)},
        {"applied_cfo_hz", json_finite_or_null(record.cfo_applied_hz)},
        {"command_output_sample", record.cfo_command_output_sample},
        {"command_input_sample", record.cfo_command_input_sample},
        {"effective_input_sample", record.cfo_effective_input_sample},
        {"applied_effective_input_sample",
         record.cfo_applied_effective_input_sample},
        {"applied_input_sample", record.cfo_applied_input_sample},
        {"applied_output_sample", record.cfo_applied_output_sample},
        {"fixed_delay_samples", record.cfo_fixed_delay_samples},
        {"late_samples", record.cfo_late_samples},
        {"input_sample_rate_hz", record.cfo_input_sample_rate_hz},
        {"pending_commands", record.cfo_pending_commands},
        {"rebootstrap",
         {{"requests", record.cfo_rebootstrap_requests},
          {"completed", record.cfo_rebootstrap_count},
          {"last_residual_hz",
           optional_number(record.cfo_rebootstrap_last_residual_hz)},
          {"output_sample", record.cfo_rebootstrap_output_sample},
          {"source_sample", record.cfo_rebootstrap_source_sample}}}};
    result["clock_tracking"] = {
        {"raw_timing_samples", optional_number(record.raw_timing_samples)},
        {"filtered_timing_samples",
         optional_number(record.filtered_timing_samples)},
        {"physical_timing_samples",
         optional_number(record.physical_timing_samples)},
        {"observed_timing_drift_samples",
         optional_number(record.observed_timing_drift_samples)},
        {"smoothed_timing_drift_samples",
         optional_number(record.smoothed_timing_drift_samples)},
        {"estimated_sro_ppm", optional_number(record.estimated_sro_ppm)},
        {"timing_confidence", optional_number(record.timing_confidence)},
        {"cir_offset_samples", optional_number(record.cir_offset_samples)},
        {"cir_confidence", optional_number(record.cir_confidence)},
        {"measurements", record.timing_measurements},
        {"accepted_measurements", record.timing_accepted_measurements},
        {"rejected_measurements", record.timing_rejected_measurements},
        {"drift_ready", record.timing_drift_ready},
        {"resampler_ready", record.sro_resampler_ready},
        {"commanded_sro_ppm", json_finite_or_null(record.sro_command_ppm)},
        {"applied_sro_ppm", json_finite_or_null(record.sro_applied_ppm)},
        {"requested_ratio",
         json_finite_or_null(record.requested_resample_ratio)},
        {"effective_ratio",
         json_finite_or_null(record.effective_resample_ratio)},
        {"command_output_sample", record.sro_command_output_sample},
        {"command_input_sample", record.sro_command_input_sample},
        {"effective_input_sample", record.sro_effective_input_sample},
        {"applied_input_sample", record.sro_applied_input_sample},
        {"fixed_delay_samples", record.sro_fixed_delay_samples},
        {"late_samples", record.sro_late_samples},
        {"pending_commands", record.sro_pending_commands}};
    result["phase_discontinuities"] = record.phase_discontinuities;
    result["pilot_phase_fast_path"] = {
        {"checks", record.pilot_expected_phase_checks},
        {"fast_accepts", record.pilot_expected_phase_fast_accepts},
        {"fallbacks", record.pilot_expected_phase_fallbacks},
        {"confidence_mean",
         optional_number(record.pilot_expected_phase_confidence_mean)},
        {"confidence_min",
         optional_number(record.pilot_expected_phase_confidence_min)}};
    result["setup_ms"] = {
        {"fft_plan", json_finite_or_null(record.fft_plan_time_ms)}};
    result["timing_ms"] = {
        {"wall", json_finite_or_null(record.wall_time_ms)},
        {"serial_busy_total", json_finite_or_null(record.serial_busy_time_ms)},
        {"serial_busy", timing_map(record.serial_busy_ms)},
        {"wait", timing_map(record.wait_ms)},
        {"nested", timing_map(record.nested_ms)},
        {"aggregate_worker_work", timing_map(record.aggregate_worker_work_ms)}};
    return result;
}

[[nodiscard]] json encode(const dvbt::FecWindowTelemetry &record) {
    json result = envelope(record.envelope, "fec_window");
    result["demod_window_sequence"] = record.demod_window_sequence;
    result["fec_session"] = record.fec_session;
    result["output"] = {
        {"bytes_delta", record.output_bytes_delta},
        {"bytes_cumulative", record.output_bytes_cumulative},
        {"packets_cumulative", record.output_bytes_cumulative / 188},
        {"partial_bytes_cumulative", record.output_bytes_cumulative % 188}};
    result["session"] = transport_stats(record.session);
    result["delta"] = transport_stats(record.delta);
    result["cumulative"] = transport_stats(record.cumulative);
    json nested = timing_map(record.nested_ms);
    nested["fec::transport"] = json_finite_or_null(record.transport_nested_ms);
    result["timing_ms"] = {
        {"serial_wall", {{"fec::total", record.fec_total_ms}}},
        {"wait", timing_map(record.wait_ms)},
        {"nested", std::move(nested)},
        {"thread_cpu", timing_map(record.thread_cpu_ms)},
        {"aggregate_worker_work", timing_map(record.aggregate_worker_work_ms)}};
    return result;
}

[[nodiscard]] const char *
event_severity_name(const dvbt::DecoderEventSeverity severity) {
    return severity == dvbt::DecoderEventSeverity::warning ? "warning" : "info";
}

[[nodiscard]] json event_value(const dvbt::DecoderEventValue &value) {
    return std::visit(
        [](const auto &item) -> json {
            using Value = std::decay_t<decltype(item)>;
            if constexpr (std::is_same_v<Value, double>) {
                return json_finite_or_null(item);
            } else {
                return item;
            }
        },
        value);
}

[[nodiscard]] json encode(const dvbt::DecoderEventTelemetry &record) {
    json result = envelope(record.envelope, "decoder_event");
    result["event"] = record.event;
    result["severity"] = event_severity_name(record.severity);
    result["source_sample"] = record.source_sample.has_value()
                                  ? json(*record.source_sample)
                                  : json(nullptr);
    result["resampled_sample"] = record.resampled_sample.has_value()
                                     ? json(*record.resampled_sample)
                                     : json(nullptr);
    result["ofdm_symbol"] = record.ofdm_symbol.has_value()
                                ? json(*record.ofdm_symbol)
                                : json(nullptr);
    result["fields"] = json::object();
    for (const auto &[name, value] : record.fields) {
        result["fields"][name] = event_value(value);
    }
    return result;
}

} // namespace

const char *dvbt_worker_state_name(const dvbt::WorkerState state) {
    switch (state) {
    case dvbt::WorkerState::idle:
        return "idle";
    case dvbt::WorkerState::processing:
        return "processing";
    case dvbt::WorkerState::waiting_input:
        return "waiting_input";
    case dvbt::WorkerState::waiting_ring_space:
        return "waiting_ring_space";
    case dvbt::WorkerState::waiting_ring_data:
        return "waiting_ring_data";
    case dvbt::WorkerState::waiting_sync:
        return "waiting_sync";
    case dvbt::WorkerState::waiting_acquisition:
        return "waiting_acquisition";
    case dvbt::WorkerState::waiting_fec_item:
        return "waiting_fec_item";
    case dvbt::WorkerState::exited:
        return "exited";
    }
    return "unknown";
}

DvbTJsonRecord encode_dvbt_telemetry(const dvbt::TelemetryRecord &record) {
    return std::visit(
        [](const auto &value) -> DvbTJsonRecord {
            using Value = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Value, dvbt::FrontendBlockTelemetry>) {
                return {DvbTReportStream::frontend, encode(value)};
            } else if constexpr (std::is_same_v<Value,
                                                dvbt::DemodWindowTelemetry>) {
                return {DvbTReportStream::demod, encode(value)};
            } else if constexpr (std::is_same_v<Value,
                                                dvbt::FecWindowTelemetry>) {
                return {DvbTReportStream::fec, encode(value)};
            } else {
                return {DvbTReportStream::event, encode(value)};
            }
        },
        record);
}

nlohmann::json
encode_dvbt_decoder_config(const dvbt::ReceiverParameters &parameters) {
    const auto optional_parameter = [](const auto &value,
                                       const auto name_function) -> json {
        return value.has_value() ? json(name_function(*value)) : json("auto");
    };
    return {
        {"channel_bandwidth_hz", parameters.channel_bandwidth_hz},
        {"transmission_mode", optional_parameter(parameters.mode, mode_name)},
        {"guard_interval",
         optional_parameter(parameters.guard_interval, guard_name)},
        {"constellation",
         optional_parameter(parameters.constellation, constellation_name)},
        {"code_rate", optional_parameter(parameters.code_rate, code_rate_name)},
        {"worker_threads", parameters.worker_threads},
        {"queue_capacity_multiplier", parameters.queue_capacity_multiplier}};
}

void format_debug_telemetry(std::ostream &stream,
                            const dvbt::TelemetryRecord &record) {
    std::visit(
        [&stream](const auto &value) {
            using Value = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Value, dvbt::FrontendBlockTelemetry>) {
                stream << std::format(
                    "[frontend_block] seq={} source={}..{} resampled={}..{} "
                    "ratio={:.9f} sro={:+.4f}ppm cfo={:+.2f}Hz "
                    "total={:.3f}ms\n",
                    value.envelope.sequence, value.source_begin_sample,
                    value.source_end_sample, value.resampled_begin_sample,
                    value.resampled_end_sample, value.effective_ratio,
                    value.applied_sro_ppm, value.applied_cfo_hz,
                    value.serial_wall_ms.at("frontend::total"));
            } else if constexpr (std::is_same_v<Value,
                                                dvbt::DemodWindowTelemetry>) {
                stream << std::format(
                    "[demod_window] seq={} symbols={} MER={} CFO={}Hz "
                    "SRO={}ppm wall={:.3f}ms busy={:.3f}ms\n",
                    value.demod_window_sequence, value.symbol_count,
                    value.mer_db ? std::format("{:.2f}", *value.mer_db) : "--",
                    value.tracked_cfo_hz
                        ? std::format("{:+.2f}", *value.tracked_cfo_hz)
                        : "--",
                    value.estimated_sro_ppm
                        ? std::format("{:+.4f}", *value.estimated_sro_ppm)
                        : "--",
                    value.wall_time_ms, value.serial_busy_time_ms);
                if (value.raw_timing_samples.has_value()) {
                    stream << std::format(
                        "  clock::timing raw={:.2f} filtered={:.2f} "
                        "physical={:.2f} drift={:.3f} smooth={:.3f} "
                        "sro={:+.4f}ppm resampler={:+.4f}/{:+.4f}ppm "
                        "confidence={:.3f} cir={:.2f}/{:.3f} ready={}\n",
                        *value.raw_timing_samples,
                        value.filtered_timing_samples.value_or(0.0),
                        value.physical_timing_samples.value_or(0.0),
                        value.observed_timing_drift_samples.value_or(0.0),
                        value.smoothed_timing_drift_samples.value_or(0.0),
                        value.estimated_sro_ppm.value_or(0.0),
                        value.sro_command_ppm, value.sro_applied_ppm,
                        value.timing_confidence.value_or(0.0),
                        value.cir_offset_samples.value_or(0.0),
                        value.cir_confidence.value_or(0.0),
                        value.timing_drift_ready);
                }
                stream << std::format(
                    "  clock::frequency acquisition={:+.2f}Hz "
                    "fractional={:+.2f}Hz bins={} estimated={:+.2f}Hz "
                    "residual={:+.2f}Hz resampler={:+.2f}/{:+.2f}Hz "
                    "delay={} late={} pending={} rebootstrap={}/{} "
                    "last={:+.2f}Hz\n",
                    value.acquisition_cfo_hz.value_or(0.0),
                    value.acquisition_fractional_cfo_hz.value_or(0.0),
                    value.acquisition_carrier_bin_offset,
                    value.tracked_cfo_hz.value_or(0.0),
                    value.residual_cfo_hz.value_or(0.0), value.cfo_command_hz,
                    value.cfo_applied_hz, value.cfo_fixed_delay_samples,
                    value.cfo_late_samples, value.cfo_pending_commands,
                    value.cfo_rebootstrap_count, value.cfo_rebootstrap_requests,
                    value.cfo_rebootstrap_last_residual_hz.value_or(0.0));
                for (const auto &[name, timing] : value.serial_busy_ms) {
                    stream << std::format("  {}={:.3f}ms\n", name, timing);
                }
                for (const auto &[name, timing] : value.nested_ms) {
                    stream << std::format("  {}={:.3f}ms (nested)\n", name,
                                          timing);
                }
            } else if constexpr (std::is_same_v<Value,
                                                dvbt::FecWindowTelemetry>) {
                stream << std::format(
                    "[fec_window] seq={} demod={} session={} bytes=+{} "
                    "total={} RS=+{}/{} TEI=+{} fec={:.3f}ms "
                    "transport={:.3f}ms\n",
                    value.envelope.sequence, value.demod_window_sequence,
                    value.fec_session, value.output_bytes_delta,
                    value.output_bytes_cumulative, value.delta.rs_packets,
                    value.delta.rs_uncorrectable_packets,
                    value.delta.tei_packets, value.fec_total_ms,
                    value.transport_nested_ms);
            } else {
                stream << std::format("[evt] {} severity={}", value.event,
                                      event_severity_name(value.severity));
                if (value.ofdm_symbol.has_value()) {
                    stream << " sym=" << *value.ofdm_symbol;
                }
                for (const auto &[name, field] : value.fields) {
                    stream << ' ' << name << '=';
                    std::visit([&stream](const auto &item) { stream << item; },
                               field);
                }
                stream << '\n';
            }
        },
        record);
}

} // namespace airspy_tv
