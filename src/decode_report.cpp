#include "decode_report.hpp"
#include "dvbt/dvbt_report_codec.hpp"
#include "telemetry_stream_router.hpp"

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
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

using dvbt::TransportDecoderStats;
using nlohmann::json;

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

struct TelemetryAggregate {
    Aggregate mer;
    Aggregate sro;
    Aggregate acquisition_cfo;
    Aggregate cfo;
    Aggregate residual_cfo;
    Aggregate cfo_command;
    Aggregate cfo_applied;
    Aggregate timing;
    std::map<std::string, Aggregate, std::less<>> timing_aggregates;
    std::map<std::string, std::uint64_t, std::less<>> event_counts;
    TransportDecoderStats transport;
    std::uint64_t transport_bytes{};
    std::uint64_t demod_windows{};
    std::uint64_t ofdm_locked_windows{};
    std::uint64_t tps_locked_windows{};
    std::uint64_t ofdm_symbols{};

    void add(const dvbt::FrontendBlockTelemetry &record) {
        for (const auto &[name, duration_ms] : record.serial_wall_ms) {
            timing_aggregates[name].add(duration_ms);
        }
    }

    void add(const dvbt::DemodWindowTelemetry &record) {
        if (record.mer_db) {
            mer.add(*record.mer_db);
        }
        if (record.estimated_sro_ppm) {
            sro.add(*record.estimated_sro_ppm);
        }
        if (record.tracked_cfo_hz) {
            cfo.add(*record.tracked_cfo_hz);
        }
        if (record.acquisition_cfo_hz) {
            acquisition_cfo.add(*record.acquisition_cfo_hz);
        }
        if (record.residual_cfo_hz) {
            residual_cfo.add(*record.residual_cfo_hz);
        }
        cfo_command.add(record.cfo_command_hz);
        cfo_applied.add(record.cfo_applied_hz);
        if (record.physical_timing_samples) {
            timing.add(*record.physical_timing_samples);
        }
        for (const auto *values :
             {&record.serial_busy_ms, &record.wait_ms, &record.nested_ms,
              &record.aggregate_worker_work_ms}) {
            for (const auto &[name, value] : *values) {
                timing_aggregates[name].add(value);
            }
        }
        ++demod_windows;
        ofdm_symbols += record.symbol_count;
        if (record.ofdm_locked) {
            ++ofdm_locked_windows;
        }
        if (record.tps_locked) {
            ++tps_locked_windows;
        }
    }

    void add(const dvbt::FecWindowTelemetry &record) {
        timing_aggregates["fec::total"].add(record.fec_total_ms);
        timing_aggregates["fec::transport"].add(record.transport_nested_ms);
        transport_bytes += record.output_bytes_delta;
        add_transport(record.delta);
    }

    void add(const dvbt::DecoderEventTelemetry &record) {
        ++event_counts[record.event];
    }

    [[nodiscard]] json measurements() const {
        return {{"mer_db", mer.value()},
                {"sro_ppm", sro.value()},
                {"acquisition_cfo_hz", acquisition_cfo.value()},
                {"tracked_cfo_hz", cfo.value()},
                {"residual_cfo_hz", residual_cfo.value()},
                {"commanded_cfo_hz", cfo_command.value()},
                {"applied_cfo_hz", cfo_applied.value()},
                {"physical_timing_samples", timing.value()}};
    }

    [[nodiscard]] json timing_summary() const {
        json result = json::object();
        for (const auto &[name, value] : timing_aggregates) {
            result[name] = value.value();
        }
        return result;
    }

    [[nodiscard]] json event_summary() const {
        json result = json::object();
        for (const auto &[name, count] : event_counts) {
            result[name] = count;
        }
        return result;
    }

  private:
    void add_transport(const TransportDecoderStats &value) {
        transport.viterbi_bits += value.viterbi_bits;
        transport.pre_viterbi_error_bits += value.pre_viterbi_error_bits;
        transport.pre_viterbi_compared_bits += value.pre_viterbi_compared_bits;
        transport.post_viterbi_error_bits += value.post_viterbi_error_bits;
        transport.post_viterbi_compared_bits +=
            value.post_viterbi_compared_bits;
        transport.rs_packets += value.rs_packets;
        transport.rs_uncorrectable_packets += value.rs_uncorrectable_packets;
        transport.tei_packets += value.tei_packets;
        transport.ts_packets += value.ts_packets;
    }
};

template <typename Value>
[[nodiscard]] Value counter_delta(const Value current, const Value initial) {
    return current >= initial ? current - initial : current;
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

struct TransportOutputTotal {
    std::string type;
    bool active{};
    bool required{};
    bool failed{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_processed{};
    std::uint64_t bytes_processed{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::uint64_t errors{};
    std::size_t queue_capacity_bytes{};
    std::size_t maximum_queued_bytes{};
    std::string error;
};

struct TransportOutputAggregate {
    void set_baseline(const std::span<const TransportOutputTelemetry> outputs) {
        previous.clear();
        for (const auto &output : outputs) {
            previous[output.name] = output;
            update_state(output);
        }
        initialized = true;
    }

    void observe(const std::span<const TransportOutputTelemetry> outputs) {
        if (!initialized) {
            set_baseline(outputs);
            return;
        }
        for (const auto &output : outputs) {
            auto &total = update_state(output);
            const auto found = previous.find(output.name);
            if (found == previous.end()) {
                add_delta(total, output, {});
            } else {
                add_delta(total, output, found->second);
            }
            previous[output.name] = output;
        }
    }

    [[nodiscard]] json summary() const {
        json result = json::array();
        for (const auto &[name, value] : totals) {
            result.push_back({
                {"name", name},
                {"type", value.type},
                {"active", value.active},
                {"required", value.required},
                {"failed", value.failed},
                {"accepted",
                 {{"blocks", value.blocks_accepted},
                  {"bytes", value.bytes_accepted}}},
                {"processed",
                 {{"blocks", value.blocks_processed},
                  {"bytes", value.bytes_processed}}},
                {"dropped",
                 {{"blocks", value.dropped_blocks},
                  {"bytes", value.dropped_bytes}}},
                {"errors", value.errors},
                {"queue",
                 {{"capacity_bytes", value.queue_capacity_bytes},
                  {"maximum_used_bytes", value.maximum_queued_bytes}}},
                {"error",
                 value.error.empty() ? json(nullptr) : json(value.error)},
            });
        }
        return result;
    }

  private:
    TransportOutputTotal &update_state(const TransportOutputTelemetry &output) {
        auto &total = totals[output.name];
        total.type = output.type;
        total.active = output.active;
        total.required = output.required;
        total.failed |= output.failed;
        total.queue_capacity_bytes = output.queue_capacity_bytes;
        total.maximum_queued_bytes =
            std::max(total.maximum_queued_bytes, output.queued_bytes);
        if (!output.error.empty()) {
            total.error = output.error;
        }
        return total;
    }

    static void add_delta(TransportOutputTotal &total,
                          const TransportOutputTelemetry &current,
                          const TransportOutputTelemetry &initial) {
        total.blocks_accepted +=
            counter_delta(current.blocks_accepted, initial.blocks_accepted);
        total.bytes_accepted +=
            counter_delta(current.bytes_accepted, initial.bytes_accepted);
        total.blocks_processed +=
            counter_delta(current.blocks_processed, initial.blocks_processed);
        total.bytes_processed +=
            counter_delta(current.bytes_processed, initial.bytes_processed);
        total.dropped_blocks +=
            counter_delta(current.dropped_blocks, initial.dropped_blocks);
        total.dropped_bytes +=
            counter_delta(current.dropped_bytes, initial.dropped_bytes);
        total.errors += counter_delta(current.errors, initial.errors);
    }

    std::map<std::string, TransportOutputTelemetry, std::less<>> previous;
    std::map<std::string, TransportOutputTotal, std::less<>> totals;
    bool initialized{};
};

[[nodiscard]] json transport_output_record(
    const TransportOutputTelemetry &output, const std::uint64_t sequence,
    const std::uint64_t decoder_generation, const std::uint64_t source_epoch,
    const double wall_elapsed_seconds) {
    return {
        {"schema_version", 0},
        {"record_type", "transport_output_sample"},
        {"mode", "dvbt"},
        {"sequence", sequence},
        {"wall_elapsed_ms", wall_elapsed_seconds * 1000.0},
        {"decoder_generation", decoder_generation},
        {"source_epoch", source_epoch},
        {"sink", {{"name", output.name}, {"type", output.type}}},
        {"active", output.active},
        {"required", output.required},
        {"failed", output.failed},
        {"accepted",
         {{"blocks", output.blocks_accepted},
          {"bytes", output.bytes_accepted}}},
        {"processed",
         {{"blocks", output.blocks_processed},
          {"bytes", output.bytes_processed}}},
        {"dropped",
         {{"blocks", output.dropped_blocks}, {"bytes", output.dropped_bytes}}},
        {"errors", output.errors},
        {"queue",
         {{"used_bytes", output.queued_bytes},
          {"capacity_bytes", output.queue_capacity_bytes}}},
        {"error", output.error.empty() ? json(nullptr) : json(output.error)},
    };
}

} // namespace

struct DecodeReport::Impl {
    struct ActiveSource {
        DecodeSourceSessionConfig config;
        dvbt::StreamDecoderStats initial_stats;
        InputTimelineSnapshot initial_timeline;
        TelemetryAggregate aggregate;
        TransportOutputAggregate transport_outputs;
        double wall_started_seconds{};
        std::uint64_t sequence{};
        std::uint64_t first_source_epoch{};
        std::uint64_t last_source_epoch{};
        std::uint64_t first_decoder_generation{};
        std::uint64_t last_decoder_generation{};
    };

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

        router = std::make_unique<TelemetryStreamRouter>(
            config.directory, std::vector<TelemetryStreamSpec>{
                                  {.key = "source_sessions",
                                   .path = "source-sessions.jsonl",
                                   .record_type = "source_session"},
                                  {.key = "frontend",
                                   .path = "frontend.jsonl",
                                   .record_type = "frontend_block"},
                                  {.key = "pipeline",
                                   .path = "pipeline.jsonl",
                                   .record_type = "pipeline_sample"},
                                  {.key = "transport_outputs",
                                   .path = "transport-outputs.jsonl",
                                   .record_type = "transport_output_sample"},
                                  {.key = "demod",
                                   .path = "dvbt-demod.jsonl",
                                   .record_type = "demod_window"},
                                  {.key = "fec",
                                   .path = "dvbt-fec.jsonl",
                                   .record_type = "fec_window"},
                                  {.key = "events",
                                   .path = "events.jsonl",
                                   .record_type = "decoder_event"},
                              });

        json manifest_streams = json::array();
        for (const auto &stream : router->specs()) {
            manifest_streams.push_back(
                {{"path", stream.path.string()},
                 {"record_type", stream.record_type},
                 {"schema_version", stream.schema_version}});
        }

        const json manifest = {
            {"report_format_version", 0},
            {"mode", "dvbt"},
            {"tool", {{"name", "airspy-tv"}, {"version", "0.1.0"}}},
            {"context", config.context},
            {"streams", std::move(manifest_streams)},
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
        write_summary("running", 0, "", 0.0);
    }

    void consume(const std::span<const dvbt::TelemetryRecord> records) {
        std::vector<json> frontend;
        std::vector<json> demod;
        std::vector<json> fec;
        std::vector<json> events;
        for (const auto &record : records) {
            std::visit(
                [this](const auto &value) {
                    global_aggregate.add(value);
                    if (active_source) {
                        active_source->aggregate.add(value);
                        observe(*active_source, value.envelope);
                    }
                },
                record);
            auto encoded = encode_dvbt_telemetry(record);
            switch (encoded.stream) {
            case DvbTReportStream::frontend:
                frontend.push_back(std::move(encoded.value));
                break;
            case DvbTReportStream::demod:
                demod.push_back(std::move(encoded.value));
                break;
            case DvbTReportStream::fec:
                fec.push_back(std::move(encoded.value));
                break;
            case DvbTReportStream::event:
                events.push_back(std::move(encoded.value));
                break;
            }
        }
        router->write_batch("frontend", frontend);
        router->write_batch("demod", demod);
        router->write_batch("fec", fec);
        router->write_batch("events", events);
    }

    void begin_source(DecodeSourceSessionConfig selected,
                      const InputTimelineSnapshot &timeline,
                      const dvbt::StreamDecoderStats &stats,
                      const double wall_elapsed_seconds,
                      const std::span<const TransportOutputTelemetry> outputs) {
        if (finalized) {
            throw std::logic_error(
                "Cannot begin a source after report finalization");
        }
        if (active_source) {
            throw std::logic_error(
                "A decode report source session is already active");
        }
        active_source = ActiveSource{
            .config = std::move(selected),
            .initial_stats = stats,
            .initial_timeline = timeline,
            .aggregate = {},
            .transport_outputs = {},
            .wall_started_seconds = wall_elapsed_seconds,
            .sequence = ++source_sequence,
            .first_source_epoch = timeline.stream_epoch,
            .last_source_epoch = timeline.stream_epoch,
            .first_decoder_generation = stats.decoder_generation,
            .last_decoder_generation = stats.decoder_generation,
        };
        active_source->transport_outputs.set_baseline(outputs);
        if (!global_transport_outputs_initialized) {
            global_transport_outputs.set_baseline(outputs);
            global_transport_outputs_initialized = true;
        }
    }

    void write_transport_outputs(
        const std::span<const TransportOutputTelemetry> outputs,
        const std::uint64_t decoder_generation,
        const std::uint64_t source_epoch, const double wall_elapsed_seconds) {
        if (!active_source) {
            throw std::logic_error(
                "Transport output sample has no active source session");
        }
        active_source->transport_outputs.observe(outputs);
        global_transport_outputs.observe(outputs);
        std::vector<json> records;
        records.reserve(outputs.size());
        for (const auto &output : outputs) {
            if (!output.active && !output.failed &&
                output.blocks_accepted == 0 && output.dropped_blocks == 0 &&
                output.errors == 0) {
                continue;
            }
            records.push_back(transport_output_record(
                output, ++transport_output_sequence, decoder_generation,
                source_epoch, wall_elapsed_seconds));
        }
        router->write_batch("transport_outputs", records);
    }

    void write_pipeline(const dvbt::StreamDecoderStats &stats,
                        const std::uint64_t submitted_samples,
                        const double wall_elapsed_seconds) {
        if (!active_source) {
            throw std::logic_error(
                "Pipeline sample has no active source session");
        }
        observe(*active_source, stats);
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
            {"frontend", dvbt_worker_state_name(stats.frontend_state)},
            {"demod", dvbt_worker_state_name(stats.demod_state)},
            {"fec", dvbt_worker_state_name(stats.fec_state)},
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
        router->write("pipeline", record);
    }

    void flush() const { router->flush(); }

    void end_source(const std::string_view status, const std::string_view error,
                    const dvbt::StreamDecoderStats &stats,
                    const InputTimelineSnapshot &timeline,
                    const std::uint64_t submitted_samples,
                    const double wall_elapsed_seconds) {
        if (!active_source) {
            throw std::logic_error("No active decode report source session");
        }
        observe(*active_source, stats);
        observe(*active_source, timeline);
        const ActiveSource &source = *active_source;
        const auto &fec = source.aggregate.transport;
        const std::uint64_t processed_samples =
            counter_delta(stats.processed_input_samples,
                          source.initial_stats.processed_input_samples);
        const double signal_seconds =
            source.config.sample_rate_hz == 0
                ? 0.0
                : static_cast<double>(submitted_samples) /
                      source.config.sample_rate_hz;
        const double duration_seconds =
            std::max(0.0, wall_elapsed_seconds - source.wall_started_seconds);
        json record = {
            {"schema_version", 0},
            {"record_type", "source_session"},
            {"mode", "dvbt"},
            {"sequence", source.sequence},
            {"source_session_id", std::format("source-{:06}", source.sequence)},
            {"source",
             {{"descriptor", source.config.source},
              {"sample_rate_hz", source.config.sample_rate_hz},
              {"center_frequency_hz", source.config.center_frequency_hz}}},
            {"destination", {{"descriptor", source.config.destination}}},
            {"decoder", decoder_config(source.config.decoder)},
            {"correlation",
             {{"first_source_epoch", source.first_source_epoch},
              {"last_source_epoch", source.last_source_epoch},
              {"first_decoder_generation", source.first_decoder_generation},
              {"last_decoder_generation", source.last_decoder_generation}}},
            {"source_samples",
             {{"begin", source.initial_timeline.source_head_sample},
              {"end", timeline.source_head_sample}}},
            {"duration",
             {{"wall_started_seconds", source.wall_started_seconds},
              {"wall_ended_seconds", wall_elapsed_seconds},
              {"wall_seconds", duration_seconds},
              {"signal_seconds", signal_seconds},
              {"average_realtime_speed", duration_seconds > 0.0
                                             ? signal_seconds / duration_seconds
                                             : 0.0}}},
            {"samples",
             {{"submitted", submitted_samples},
              {"processed", processed_samples}}},
            {"transport",
             transport_summary(source.aggregate.transport_bytes, fec)},
            {"transport_outputs", source.transport_outputs.summary()},
            {"counters", counter_summary(source, stats, fec)},
            {"windows", window_summary(source.aggregate)},
            {"measurements", source.aggregate.measurements()},
            {"timing_ms", source.aggregate.timing_summary()},
            {"events", source.aggregate.event_summary()},
            {"final_state", final_state(stats)},
            {"status", status},
            {"error", error.empty() ? json(nullptr) : json(error)},
        };
        router->write("source_sessions", record);

        ++source_sessions_total;
        ++source_status_counts[std::string(status)];
        submitted_samples_total += submitted_samples;
        processed_samples_total += processed_samples;
        signal_seconds_total += signal_seconds;
        dropped_blocks_total += counter_delta(
            stats.dropped_blocks, source.initial_stats.dropped_blocks);
        phase_discontinuities_total +=
            counter_delta(stats.pilot_phase_discontinuities,
                          source.initial_stats.pilot_phase_discontinuities);
        overlap_packets_total += counter_delta(
            stats.ts_overlap_packets, source.initial_stats.ts_overlap_packets);
        overlap_join_failures_total +=
            counter_delta(stats.ts_overlap_join_failures,
                          source.initial_stats.ts_overlap_join_failures);
        fec_sessions_total += counter_delta(stats.fec_sessions,
                                            source.initial_stats.fec_sessions);
        bootstrap_attempts_total += counter_delta(
            stats.bootstrap_attempts, source.initial_stats.bootstrap_attempts);
        cfo_rebootstrap_requests_total +=
            counter_delta(stats.cfo_rebootstrap_requests,
                          source.initial_stats.cfo_rebootstrap_requests);
        cfo_rebootstrap_count_total +=
            counter_delta(stats.cfo_rebootstrap_count,
                          source.initial_stats.cfo_rebootstrap_count);
        active_source.reset();
    }

    void write_summary(const std::string_view status, const int exit_code,
                       const std::string_view error,
                       const double wall_elapsed_seconds) const {
        const auto &fec = global_aggregate.transport;
        json summary = {
            {"schema_version", 0},
            {"status", status},
            {"program_version", "0.1.0"},
            {"mode", "dvbt"},
            {"context", config.context},
            {"duration",
             {{"wall_seconds", wall_elapsed_seconds},
              {"signal_seconds", signal_seconds_total},
              {"average_realtime_speed",
               wall_elapsed_seconds > 0.0
                   ? signal_seconds_total / wall_elapsed_seconds
                   : 0.0}}},
            {"samples",
             {{"submitted", submitted_samples_total},
              {"processed", processed_samples_total}}},
            {"source_sessions",
             {{"total", source_sessions_total},
              {"completed", source_status_count("completed")},
              {"no_transport", source_status_count("no_transport")},
              {"failed", source_status_count("failed")}}},
            {"transport",
             transport_summary(global_aggregate.transport_bytes, fec)},
            {"transport_outputs", global_transport_outputs.summary()},
            {"counters",
             {{"dropped_blocks", dropped_blocks_total},
              {"ofdm_symbols", global_aggregate.ofdm_symbols},
              {"phase_discontinuities", phase_discontinuities_total},
              {"overlap_packets", overlap_packets_total},
              {"overlap_join_failures", overlap_join_failures_total},
              {"fec_sessions", fec_sessions_total},
              {"bootstrap_attempts", bootstrap_attempts_total},
              {"cfo_rebootstrap_requests", cfo_rebootstrap_requests_total},
              {"cfo_rebootstrap_count", cfo_rebootstrap_count_total},
              {"pre_viterbi_error_bits", fec.pre_viterbi_error_bits},
              {"pre_viterbi_compared_bits", fec.pre_viterbi_compared_bits},
              {"post_viterbi_error_bits", fec.post_viterbi_error_bits},
              {"post_viterbi_compared_bits", fec.post_viterbi_compared_bits},
              {"rs_packets", fec.rs_packets},
              {"rs_uncorrectable_packets", fec.rs_uncorrectable_packets},
              {"tei_packets", fec.tei_packets}}},
            {"windows", window_summary(global_aggregate)},
            {"measurements", global_aggregate.measurements()},
            {"timing_ms", global_aggregate.timing_summary()},
            {"events", global_aggregate.event_summary()},
            {"exit_code", exit_code},
            {"error", error.empty() ? json(nullptr) : json(error)},
        };
        write_json_atomic(config.directory / "stats.json", summary);
    }

    static json decoder_config(const dvbt::ReceiverParameters &decoder) {
        return encode_dvbt_decoder_config(decoder);
    }

    static json transport_summary(const std::uint64_t bytes,
                                  const TransportDecoderStats &fec) {
        return {{"bytes", bytes},
                {"emitted_packets", bytes / 188},
                {"emitted_partial_bytes", bytes % 188},
                {"decoded_packets", fec.ts_packets},
                {"tei_packets", fec.tei_packets},
                {"usable_packets", fec.ts_packets >= fec.tei_packets
                                       ? fec.ts_packets - fec.tei_packets
                                       : 0}};
    }

    static json counter_summary(const ActiveSource &source,
                                const dvbt::StreamDecoderStats &stats,
                                const TransportDecoderStats &fec) {
        return {
            {"dropped_blocks",
             counter_delta(stats.dropped_blocks,
                           source.initial_stats.dropped_blocks)},
            {"ofdm_symbols", source.aggregate.ofdm_symbols},
            {"phase_discontinuities",
             counter_delta(stats.pilot_phase_discontinuities,
                           source.initial_stats.pilot_phase_discontinuities)},
            {"overlap_packets",
             counter_delta(stats.ts_overlap_packets,
                           source.initial_stats.ts_overlap_packets)},
            {"overlap_join_failures",
             counter_delta(stats.ts_overlap_join_failures,
                           source.initial_stats.ts_overlap_join_failures)},
            {"fec_sessions", counter_delta(stats.fec_sessions,
                                           source.initial_stats.fec_sessions)},
            {"bootstrap_attempts",
             counter_delta(stats.bootstrap_attempts,
                           source.initial_stats.bootstrap_attempts)},
            {"cfo_rebootstrap_requests",
             counter_delta(stats.cfo_rebootstrap_requests,
                           source.initial_stats.cfo_rebootstrap_requests)},
            {"cfo_rebootstrap_count",
             counter_delta(stats.cfo_rebootstrap_count,
                           source.initial_stats.cfo_rebootstrap_count)},
            {"pre_viterbi_error_bits", fec.pre_viterbi_error_bits},
            {"pre_viterbi_compared_bits", fec.pre_viterbi_compared_bits},
            {"post_viterbi_error_bits", fec.post_viterbi_error_bits},
            {"post_viterbi_compared_bits", fec.post_viterbi_compared_bits},
            {"rs_packets", fec.rs_packets},
            {"rs_uncorrectable_packets", fec.rs_uncorrectable_packets},
            {"tei_packets", fec.tei_packets}};
    }

    static json window_summary(const TelemetryAggregate &aggregate) {
        return {{"demod", aggregate.demod_windows},
                {"ofdm_locked", aggregate.ofdm_locked_windows},
                {"tps_locked", aggregate.tps_locked_windows}};
    }

    static json final_state(const dvbt::StreamDecoderStats &stats) {
        return {{"ofdm_locked", stats.ofdm_locked},
                {"tps_locked", stats.tps_locked},
                {"rs_synchronized", stats.transport.rs_synchronized},
                {"energy_synchronized", stats.transport.energy_synchronized},
                {"estimated_sro_ppm",
                 json_finite_or_null(stats.sample_clock_offset_ppm)},
                {"applied_sro_ppm",
                 json_finite_or_null(stats.sro_resampler_applied_ppm)},
                {"acquisition_cfo_hz",
                 json_finite_or_null(stats.acquisition_cfo_hz)},
                {"acquisition_fractional_cfo_hz",
                 json_finite_or_null(stats.acquisition_fractional_cfo_hz)},
                {"acquisition_carrier_bin_offset",
                 stats.acquisition_carrier_bin_offset},
                {"estimated_cfo_hz",
                 json_finite_or_null(stats.tracked_carrier_offset_hz)},
                {"residual_cfo_hz",
                 json_finite_or_null(stats.residual_carrier_offset_hz)},
                {"commanded_cfo_hz",
                 json_finite_or_null(stats.cfo_resampler_command_hz)},
                {"applied_cfo_hz",
                 json_finite_or_null(stats.cfo_resampler_applied_hz)},
                {"cfo_rebootstrap_last_residual_hz",
                 json_finite_or_null(stats.cfo_rebootstrap_last_residual_hz)},
                {"cfo_rebootstrap_output_sample",
                 stats.cfo_rebootstrap_output_sample},
                {"cfo_rebootstrap_source_sample",
                 stats.cfo_rebootstrap_source_sample},
                {"bootstrap_replayed_input_samples",
                 stats.bootstrap_replayed_input_samples},
                {"bootstrap_retained_peak_samples",
                 stats.bootstrap_retained_peak_samples}};
    }

    static void update_range(std::uint64_t &first, std::uint64_t &last,
                             const std::uint64_t value) {
        if (value == 0) {
            return;
        }
        first = first == 0 ? value : std::min(first, value);
        last = std::max(last, value);
    }

    static void observe(ActiveSource &source,
                        const dvbt::TelemetryEnvelope &value) {
        update_range(source.first_source_epoch, source.last_source_epoch,
                     value.source_epoch);
        update_range(source.first_decoder_generation,
                     source.last_decoder_generation, value.decoder_generation);
    }

    static void observe(ActiveSource &source,
                        const dvbt::StreamDecoderStats &value) {
        update_range(source.first_source_epoch, source.last_source_epoch,
                     value.source_epoch);
        update_range(source.first_decoder_generation,
                     source.last_decoder_generation, value.decoder_generation);
    }

    static void observe(ActiveSource &source,
                        const InputTimelineSnapshot &value) {
        update_range(source.first_source_epoch, source.last_source_epoch,
                     value.stream_epoch);
    }

    [[nodiscard]] std::uint64_t
    source_status_count(const std::string_view status) const {
        const auto found = source_status_counts.find(status);
        return found == source_status_counts.end() ? 0 : found->second;
    }

    DecodeReportConfig config;
    std::unique_ptr<TelemetryStreamRouter> router;
    std::uint64_t pipeline_sequence{};
    std::uint64_t transport_output_sequence{};
    std::uint64_t source_sequence{};
    std::optional<ActiveSource> active_source;
    TelemetryAggregate global_aggregate;
    TransportOutputAggregate global_transport_outputs;
    std::map<std::string, std::uint64_t, std::less<>> source_status_counts;
    std::uint64_t source_sessions_total{};
    std::uint64_t submitted_samples_total{};
    std::uint64_t processed_samples_total{};
    std::uint64_t dropped_blocks_total{};
    std::uint64_t phase_discontinuities_total{};
    std::uint64_t overlap_packets_total{};
    std::uint64_t overlap_join_failures_total{};
    std::uint64_t fec_sessions_total{};
    std::uint64_t bootstrap_attempts_total{};
    std::uint64_t cfo_rebootstrap_requests_total{};
    std::uint64_t cfo_rebootstrap_count_total{};
    double signal_seconds_total{};
    bool finalized{};
    bool global_transport_outputs_initialized{};
};

DecodeReport::DecodeReport(DecodeReportConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

DecodeReport::~DecodeReport() noexcept = default;

void DecodeReport::begin_source(
    DecodeSourceSessionConfig config, const InputTimelineSnapshot &timeline,
    const dvbt::StreamDecoderStats &stats, const double wall_elapsed_seconds,
    const std::span<const TransportOutputTelemetry> outputs) {
    impl_->begin_source(std::move(config), timeline, stats,
                        wall_elapsed_seconds, outputs);
}

void DecodeReport::write_transport_outputs(
    const std::span<const TransportOutputTelemetry> outputs,
    const std::uint64_t decoder_generation, const std::uint64_t source_epoch,
    const double wall_elapsed_seconds) {
    impl_->write_transport_outputs(outputs, decoder_generation, source_epoch,
                                   wall_elapsed_seconds);
}

void DecodeReport::consume(
    const std::span<const dvbt::TelemetryRecord> records) {
    impl_->consume(records);
}

void DecodeReport::write_pipeline(const dvbt::StreamDecoderStats &stats,
                                  const std::uint64_t submitted_samples,
                                  const double wall_elapsed_seconds) {
    impl_->write_pipeline(stats, submitted_samples, wall_elapsed_seconds);
}

void DecodeReport::end_source(const std::string_view status,
                              const std::string_view error,
                              const dvbt::StreamDecoderStats &stats,
                              const InputTimelineSnapshot &timeline,
                              const std::uint64_t submitted_samples,
                              const double wall_elapsed_seconds) {
    impl_->end_source(status, error, stats, timeline, submitted_samples,
                      wall_elapsed_seconds);
}

void DecodeReport::flush() { impl_->flush(); }

void DecodeReport::finalize(const std::string_view status, const int exit_code,
                            const std::string_view error,
                            const double wall_elapsed_seconds) {
    if (impl_->active_source) {
        throw std::logic_error(
            "Cannot finalize a decode report with an active source session");
    }
    if (impl_->finalized) {
        throw std::logic_error("Decode report is already finalized");
    }
    std::exception_ptr flush_error;
    try {
        impl_->flush();
    } catch (...) {
        flush_error = std::current_exception();
    }
    impl_->write_summary(status, exit_code, error, wall_elapsed_seconds);
    impl_->finalized = true;
    if (flush_error) {
        std::rethrow_exception(flush_error);
    }
}

} // namespace airspy_tv
