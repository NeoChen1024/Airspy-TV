#include "decode_report.hpp"

#include "airspy_tv/jsonl.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

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

[[nodiscard]] const char *
worker_state_name_json(const dvbt::WorkerState state) {
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
    return {
        {"viterbi_bits", stats.viterbi_bits},
        {"pre_viterbi_error_bits", stats.pre_viterbi_error_bits},
        {"pre_viterbi_compared_bits", stats.pre_viterbi_compared_bits},
        {"post_viterbi_error_bits", stats.post_viterbi_error_bits},
        {"post_viterbi_compared_bits", stats.post_viterbi_compared_bits},
        {"rs_packets", stats.rs_packets},
        {"rs_uncorrectable_packets", stats.rs_uncorrectable_packets},
        {"tei_packets", stats.tei_packets},
        {"ts_packets", stats.ts_packets},
        {"outer_deinterleaver_phase", stats.outer_deinterleaver_phase},
        {"outer_sync_distance", stats.outer_sync_distance},
        {"outer_rs_evidence", stats.outer_rs_evidence},
        {"rs_synchronized", stats.rs_synchronized},
        {"energy_synchronized", stats.energy_synchronized},
        {"viterbi_workers", stats.viterbi_workers},
    };
}

[[nodiscard]] json to_json_record(const dvbt::FrontendBlockTelemetry &record) {
    json result = envelope(record.envelope, "frontend_block");
    result["input"] = {
        {"sample_rate_hz", record.input_sample_rate_hz},
        {"channel_bandwidth_hz", record.channel_bandwidth_hz},
        {"source_begin_sample", record.source_begin_sample},
        {"source_end_sample", record.source_end_sample},
        {"complex_samples", record.input_complex_samples},
        {"discontinuity_before", record.discontinuity_before},
        {"abandoned", record.abandoned},
    };
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
    };
    result["timing_ms"] = {{"serial_wall", timing_map(record.serial_wall_ms)}};
    return result;
}

[[nodiscard]] json to_json_record(const dvbt::DemodWindowTelemetry &record) {
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
         json_finite_or_null(record.signal_duration_seconds)},
    };
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
        {"guard_size", record.guard_size},
    };
    result["acquisition"] = {
        {"carrier_bin_offset", record.carrier_bin_offset},
        {"score", json_finite_or_null(record.acquisition_score)},
        {"start", record.acquisition_start},
    };
    result["signal"] = {
        {"mer_db", optional_number(record.mer_db)},
        {"fade_indicator", optional_number(record.fade_indicator)},
        {"tracked_cfo_hz", optional_number(record.tracked_cfo_hz)},
        {"residual_cfo_hz", optional_number(record.residual_cfo_hz)},
    };
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
        {"pending_commands", record.sro_pending_commands},
    };
    result["phase_discontinuities"] = record.phase_discontinuities;
    result["timing_ms"] = {
        {"wall", json_finite_or_null(record.wall_time_ms)},
        {"serial_busy_total", json_finite_or_null(record.serial_busy_time_ms)},
        {"serial_busy", timing_map(record.serial_busy_ms)},
        {"wait", timing_map(record.wait_ms)},
        {"nested", timing_map(record.nested_ms)},
        {"aggregate_worker_work", timing_map(record.aggregate_worker_work_ms)},
    };
    return result;
}

[[nodiscard]] json to_json_record(const dvbt::FecWindowTelemetry &record) {
    json result = envelope(record.envelope, "fec_window");
    result["demod_window_sequence"] = record.demod_window_sequence;
    result["fec_session"] = record.fec_session;
    result["output"] = {
        {"bytes_delta", record.output_bytes_delta},
        {"bytes_cumulative", record.output_bytes_cumulative},
        {"packets_cumulative", record.output_bytes_cumulative / 188},
        {"partial_bytes_cumulative", record.output_bytes_cumulative % 188},
    };
    result["session"] = transport_stats(record.session);
    result["delta"] = transport_stats(record.delta);
    result["cumulative"] = transport_stats(record.cumulative);
    result["timing_ms"] = {
        {"serial_wall", {{"fec::total", record.fec_total_ms}}},
        {"nested", {{"fec::transport", record.transport_nested_ms}}},
    };
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

[[nodiscard]] json to_json_record(const dvbt::DecoderEventTelemetry &record) {
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

struct Aggregate {
    std::uint64_t count{};
    double total{};
    double minimum{std::numeric_limits<double>::infinity()};
    double maximum{-std::numeric_limits<double>::infinity()};

    void add(const double value) {
        if (!std::isfinite(value)) {
            return;
        }
        ++count;
        total += value;
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
    }

    [[nodiscard]] json value() const {
        if (count == 0) {
            return {{"count", 0},
                    {"total", nullptr},
                    {"mean", nullptr},
                    {"min", nullptr},
                    {"max", nullptr}};
        }
        return {{"count", count},
                {"total", total},
                {"mean", total / static_cast<double>(count)},
                {"min", minimum},
                {"max", maximum}};
    }
};

[[nodiscard]] json optional_parameter(const auto &value,
                                      const auto name_function) {
    return value.has_value() ? json(name_function(*value)) : json("auto");
}

void write_json_atomic(const std::filesystem::path &path, const json &value) {
    auto temporary = path;
    temporary += ".tmp";
    {
        std::ofstream stream(temporary, std::ios::trunc);
        if (!stream) {
            throw std::runtime_error("Unable to open report summary: " +
                                     temporary.string());
        }
        stream << value.dump(2) << '\n';
        stream.flush();
        if (!stream) {
            throw std::runtime_error("Unable to write report summary: " +
                                     temporary.string());
        }
    }
    std::filesystem::rename(temporary, path);
}

} // namespace

struct DecodeReport::Impl {
    explicit Impl(DecodeReportConfig selected) : config(std::move(selected)) {
        if (config.directory.empty()) {
            throw std::runtime_error("Report directory path is empty");
        }
        if (std::filesystem::exists(config.directory)) {
            if (!std::filesystem::is_directory(config.directory)) {
                throw std::runtime_error("Report path is not a directory: " +
                                         config.directory.string());
            }
            if (!std::filesystem::is_empty(config.directory)) {
                throw std::runtime_error("Report directory is not empty: " +
                                         config.directory.string());
            }
        } else {
            std::filesystem::create_directories(config.directory);
        }

        open_stream(frontend_stream, frontend_writer, "frontend.jsonl");
        open_stream(pipeline_stream, pipeline_writer, "pipeline.jsonl");
        open_stream(demod_stream, demod_writer, "dvbt-demod.jsonl");
        open_stream(fec_stream, fec_writer, "dvbt-fec.jsonl");
        open_stream(event_stream, event_writer, "events.jsonl");

        const json manifest = {
            {"report_format_version", 0},
            {"mode", "dvbt"},
            {"tool", {{"name", "airspy-tv"}, {"version", "0.1.0"}}},
            {"source",
             {{"descriptor", config.source},
              {"sample_rate_hz", config.sample_rate_hz}}},
            {"destination", {{"descriptor", config.destination}}},
            {"decoder",
             {{"channel_bandwidth_hz", config.decoder.channel_bandwidth_hz},
              {"transmission_mode",
               optional_parameter(config.decoder.mode, mode_name)},
              {"guard_interval",
               optional_parameter(config.decoder.guard_interval, guard_name)},
              {"constellation", optional_parameter(config.decoder.constellation,
                                                   constellation_name)},
              {"code_rate",
               optional_parameter(config.decoder.code_rate, code_rate_name)},
              {"worker_threads", config.decoder.worker_threads}}},
            {"streams",
             {{{"path", "frontend.jsonl"},
               {"record_type", "frontend_block"},
               {"schema_version", 0}},
              {{"path", "pipeline.jsonl"},
               {"record_type", "pipeline_sample"},
               {"schema_version", 0}},
              {{"path", "dvbt-demod.jsonl"},
               {"record_type", "demod_window"},
               {"schema_version", 0}},
              {{"path", "dvbt-fec.jsonl"},
               {"record_type", "fec_window"},
               {"schema_version", 0}},
              {{"path", "events.jsonl"},
               {"record_type", "decoder_event"},
               {"schema_version", 0}}}},
        };
        std::ofstream manifest_stream(config.directory / "manifest.json",
                                      std::ios::trunc);
        if (!manifest_stream) {
            throw std::runtime_error("Unable to create report manifest");
        }
        manifest_stream << manifest.dump(2) << '\n';
        manifest_stream.flush();
        if (!manifest_stream) {
            throw std::runtime_error("Unable to write report manifest");
        }
        write_summary("running", 0, "", {}, 0, 0.0);
    }

    void open_stream(std::ofstream &stream,
                     std::unique_ptr<JsonlWriter> &writer,
                     const char *name) const {
        stream.open(config.directory / name,
                    std::ios::binary | std::ios::trunc);
        if (!stream) {
            throw std::runtime_error(
                std::string("Unable to create report stream: ") + name);
        }
        writer = std::make_unique<JsonlWriter>(stream);
    }

    void consume(const std::span<const dvbt::TelemetryRecord> records) {
        std::vector<json> frontend;
        std::vector<json> demod;
        std::vector<json> fec;
        std::vector<json> events;
        for (const auto &record : records) {
            std::visit(
                [this, &frontend, &demod, &fec, &events](const auto &value) {
                    using Value = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<
                                      Value, dvbt::FrontendBlockTelemetry>) {
                        for (const auto &[name, duration_ms] :
                             value.serial_wall_ms) {
                            timing_aggregates[name].add(duration_ms);
                        }
                        frontend.push_back(to_json_record(value));
                    } else if constexpr (std::is_same_v<
                                             Value,
                                             dvbt::DemodWindowTelemetry>) {
                        aggregate(value);
                        demod.push_back(to_json_record(value));
                    } else if constexpr (std::is_same_v<
                                             Value, dvbt::FecWindowTelemetry>) {
                        timing_aggregates["fec::total"].add(value.fec_total_ms);
                        timing_aggregates["fec::transport"].add(
                            value.transport_nested_ms);
                        fec.push_back(to_json_record(value));
                    } else {
                        ++event_counts[value.event];
                        events.push_back(to_json_record(value));
                    }
                },
                record);
        }
        frontend_writer->write_batch(frontend);
        demod_writer->write_batch(demod);
        fec_writer->write_batch(fec);
        event_writer->write_batch(events);
    }

    void aggregate(const dvbt::DemodWindowTelemetry &record) {
        if (record.mer_db) {
            mer.add(*record.mer_db);
        }
        if (record.estimated_sro_ppm) {
            sro.add(*record.estimated_sro_ppm);
        }
        if (record.tracked_cfo_hz) {
            cfo.add(*record.tracked_cfo_hz);
        }
        if (record.physical_timing_samples) {
            timing.add(*record.physical_timing_samples);
        }
        for (const auto *map :
             {&record.serial_busy_ms, &record.wait_ms, &record.nested_ms,
              &record.aggregate_worker_work_ms}) {
            for (const auto &[name, value] : *map) {
                timing_aggregates[name].add(value);
            }
        }
        ++demod_windows;
        if (record.ofdm_locked) {
            ++ofdm_locked_windows;
        }
        if (record.tps_locked) {
            ++tps_locked_windows;
        }
    }

    void write_pipeline(const dvbt::StreamDecoderStats &stats,
                        const std::uint64_t submitted_samples,
                        const double wall_elapsed_seconds) {
        const auto fraction = [](const std::uint64_t used,
                                 const std::uint64_t capacity) {
            return capacity == 0 ? 0.0
                                 : std::clamp(static_cast<double>(used) /
                                                  static_cast<double>(capacity),
                                              0.0, 1.0);
        };
        json record = {{"schema_version", 0},
                       {"record_type", "pipeline_sample"},
                       {"mode", "dvbt"},
                       {"sequence", ++pipeline_sequence},
                       {"wall_elapsed_ms", wall_elapsed_seconds * 1000.0},
                       {"decoder_generation", stats.decoder_generation},
                       {"source_epoch", stats.source_epoch}};
        record["progress"] = {
            {"submitted_samples", submitted_samples},
            {"processed_samples", stats.processed_input_samples},
            {"processed_chunks", stats.processed_chunks},
            {"ofdm_symbols", stats.ofdm_symbols},
            {"transport_bytes", stats.transport_bytes},
            {"dropped_blocks", stats.dropped_blocks},
            {"phase_discontinuities", stats.pilot_phase_discontinuities},
            {"overlap_packets", stats.ts_overlap_packets},
            {"overlap_join_failures", stats.ts_overlap_join_failures},
        };
        record["quality"] = {
            {"ofdm_locked", stats.ofdm_locked},
            {"tps_locked", stats.tps_locked},
            {"mer_db", stats.ofdm_locked ? json_finite_or_null(stats.mer_db)
                                         : json(nullptr)},
        };
        record["queues"] = {
            {"input",
             {{"used", stats.queued_input_samples},
              {"capacity", stats.input_queue_capacity_samples},
              {"fraction", fraction(stats.queued_input_samples,
                                    stats.input_queue_capacity_samples)}}},
            {"resampled_ring",
             {{"used", stats.ring_used_samples},
              {"capacity", stats.ring_capacity_samples},
              {"fraction", fraction(stats.ring_used_samples,
                                    stats.ring_capacity_samples)}}},
            {"fec",
             {{"used", stats.queued_symbols},
              {"capacity", stats.symbol_queue_capacity},
              {"fraction",
               fraction(stats.queued_symbols, stats.symbol_queue_capacity)}}},
        };
        record["workers"] = {
            {"frontend", worker_state_name_json(stats.frontend_state)},
            {"demod", worker_state_name_json(stats.demod_state)},
            {"fec", worker_state_name_json(stats.fec_state)},
            {"resample_count", stats.resample_workers},
            {"symbol_count", stats.symbol_workers},
            {"viterbi_count", stats.transport.viterbi_workers},
        };
        record["processing"] = {
            {"active", stats.processing},
            {"demod_busy_fraction",
             json_finite_or_null(stats.demod_busy_fraction)},
            {"realtime_ratio",
             json_finite_or_null(stats.processing_realtime_ratio)},
            {"fec_gated", stats.fec_skipped},
        };
        pipeline_writer->write(record);
    }

    void flush() const {
        frontend_writer->flush();
        pipeline_writer->flush();
        demod_writer->flush();
        fec_writer->flush();
        event_writer->flush();
    }

    void write_summary(const std::string_view status, const int exit_code,
                       const std::string_view error,
                       const dvbt::StreamDecoderStats &stats,
                       const std::uint64_t submitted_samples,
                       const double wall_elapsed_seconds) {
        json timing_summary = json::object();
        for (const auto &[name, value] : timing_aggregates) {
            timing_summary[name] = value.value();
        }
        json event_summary = json::object();
        for (const auto &[name, count] : event_counts) {
            event_summary[name] = count;
        }
        const double signal_seconds =
            config.sample_rate_hz == 0
                ? 0.0
                : static_cast<double>(submitted_samples) /
                      static_cast<double>(config.sample_rate_hz);
        const auto &fec = stats.cumulative_transport;
        json summary = {
            {"schema_version", 0},
            {"status", status},
            {"program_version", "0.1.0"},
            {"mode", "dvbt"},
            {"source",
             {{"descriptor", config.source},
              {"sample_rate_hz", config.sample_rate_hz}}},
            {"destination", {{"descriptor", config.destination}}},
            {"duration",
             {{"wall_seconds", wall_elapsed_seconds},
              {"signal_seconds", signal_seconds},
              {"average_realtime_speed",
               wall_elapsed_seconds > 0.0
                   ? signal_seconds / wall_elapsed_seconds
                   : 0.0}}},
            {"samples",
             {{"submitted", submitted_samples},
              {"processed", stats.processed_input_samples}}},
            {"transport",
             {{"bytes", stats.transport_bytes},
              {"emitted_packets", stats.transport_bytes / 188},
              {"emitted_partial_bytes", stats.transport_bytes % 188},
              {"decoded_packets", fec.ts_packets},
              {"tei_packets", fec.tei_packets},
              {"usable_packets", fec.ts_packets >= fec.tei_packets
                                     ? fec.ts_packets - fec.tei_packets
                                     : 0}}},
            {"counters",
             {{"dropped_blocks", stats.dropped_blocks},
              {"ofdm_symbols", stats.ofdm_symbols},
              {"phase_discontinuities", stats.pilot_phase_discontinuities},
              {"overlap_packets", stats.ts_overlap_packets},
              {"overlap_join_failures", stats.ts_overlap_join_failures},
              {"fec_sessions", stats.fec_sessions},
              {"pre_viterbi_error_bits", fec.pre_viterbi_error_bits},
              {"pre_viterbi_compared_bits", fec.pre_viterbi_compared_bits},
              {"post_viterbi_error_bits", fec.post_viterbi_error_bits},
              {"post_viterbi_compared_bits", fec.post_viterbi_compared_bits},
              {"rs_packets", fec.rs_packets},
              {"rs_uncorrectable_packets", fec.rs_uncorrectable_packets},
              {"tei_packets", fec.tei_packets}}},
            {"windows",
             {{"demod", demod_windows},
              {"ofdm_locked", ofdm_locked_windows},
              {"tps_locked", tps_locked_windows}}},
            {"measurements",
             {{"mer_db", mer.value()},
              {"sro_ppm", sro.value()},
              {"tracked_cfo_hz", cfo.value()},
              {"physical_timing_samples", timing.value()}}},
            {"timing_ms", timing_summary},
            {"events", event_summary},
            {"final_state",
             {{"ofdm_locked", stats.ofdm_locked},
              {"tps_locked", stats.tps_locked},
              {"rs_synchronized", stats.transport.rs_synchronized},
              {"energy_synchronized", stats.transport.energy_synchronized},
              {"estimated_sro_ppm",
               json_finite_or_null(stats.sample_clock_offset_ppm)},
              {"applied_sro_ppm",
               json_finite_or_null(stats.sro_resampler_applied_ppm)}}},
            {"exit_code", exit_code},
            {"error", error.empty() ? json(nullptr) : json(error)},
        };
        write_json_atomic(config.directory / "stats.json", summary);
    }

    DecodeReportConfig config;
    std::ofstream frontend_stream;
    std::ofstream pipeline_stream;
    std::ofstream demod_stream;
    std::ofstream fec_stream;
    std::ofstream event_stream;
    std::unique_ptr<JsonlWriter> frontend_writer;
    std::unique_ptr<JsonlWriter> pipeline_writer;
    std::unique_ptr<JsonlWriter> demod_writer;
    std::unique_ptr<JsonlWriter> fec_writer;
    std::unique_ptr<JsonlWriter> event_writer;
    std::uint64_t pipeline_sequence{};
    std::uint64_t demod_windows{};
    std::uint64_t ofdm_locked_windows{};
    std::uint64_t tps_locked_windows{};
    Aggregate mer;
    Aggregate sro;
    Aggregate cfo;
    Aggregate timing;
    std::map<std::string, Aggregate, std::less<>> timing_aggregates;
    std::map<std::string, std::uint64_t, std::less<>> event_counts;
};

DecodeReport::DecodeReport(DecodeReportConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

DecodeReport::~DecodeReport() noexcept = default;

void DecodeReport::consume(
    const std::span<const dvbt::TelemetryRecord> records) {
    impl_->consume(records);
}

void DecodeReport::write_pipeline(const dvbt::StreamDecoderStats &stats,
                                  const std::uint64_t submitted_samples,
                                  const double wall_elapsed_seconds) {
    impl_->write_pipeline(stats, submitted_samples, wall_elapsed_seconds);
}

void DecodeReport::flush() { impl_->flush(); }

void DecodeReport::finalize(const std::string_view status, const int exit_code,
                            const std::string_view error,
                            const dvbt::StreamDecoderStats &stats,
                            const std::uint64_t submitted_samples,
                            const double wall_elapsed_seconds) {
    std::exception_ptr flush_error;
    try {
        impl_->flush();
    } catch (...) {
        flush_error = std::current_exception();
    }
    impl_->write_summary(status, exit_code, error, stats, submitted_samples,
                         wall_elapsed_seconds);
    if (flush_error) {
        std::rethrow_exception(flush_error);
    }
}

void format_debug_telemetry(std::ostream &stream,
                            const dvbt::TelemetryRecord &record) {
    std::visit(
        [&stream](const auto &value) {
            using Value = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Value, dvbt::FrontendBlockTelemetry>) {
                stream << std::format(
                    "[frontend_block] seq={} source={}..{} resampled={}..{} "
                    "ratio={:.9f} sro={:+.4f}ppm total={:.3f}ms\n",
                    value.envelope.sequence, value.source_begin_sample,
                    value.source_end_sample, value.resampled_begin_sample,
                    value.resampled_end_sample, value.effective_ratio,
                    value.applied_sro_ppm,
                    value.serial_wall_ms.at("frontend::total"));
            } else if constexpr (std::is_same_v<Value,
                                                dvbt::DemodWindowTelemetry>) {
                stream << std::format(
                    "[demod_window] seq={} symbols={} MER={} CFO={}Hz "
                    "SRO={}ppm "
                    "wall={:.3f}ms busy={:.3f}ms\n",
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
                    "total={} "
                    "RS=+{}/{} TEI=+{} fec={:.3f}ms transport={:.3f}ms\n",
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
