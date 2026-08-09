#include "decode_report.hpp"

#include "airspy_tv/jsonl.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using airspy_tv::DecodeReport;
using airspy_tv::DecodeReportConfig;
using airspy_tv::DecodeSourceSessionConfig;
using airspy_tv::InputTimelineSnapshot;
using airspy_tv::JsonlReader;
using airspy_tv::TransportOutputTelemetry;
using airspy_tv::dvbt::FecWindowTelemetry;
using airspy_tv::dvbt::StreamDecoderStats;
using airspy_tv::dvbt::TelemetryRecord;
using nlohmann::json;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

struct TemporaryDirectory {
    TemporaryDirectory() {
        const auto suffix =
            std::chrono::steady_clock::now().time_since_epoch().count();
        path = std::filesystem::temp_directory_path() /
               ("airspy-tv-decode-report-test-" + std::to_string(suffix));
    }

    ~TemporaryDirectory() { std::filesystem::remove_all(path); }

    std::filesystem::path path;
};

json read_json(const std::filesystem::path &path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("Unable to open " + path.string());
    }
    return json::parse(stream);
}

FecWindowTelemetry
fec_record(const std::uint64_t sequence, const std::uint64_t generation,
           const std::uint64_t epoch, const std::uint64_t output_bytes,
           const std::uint64_t packets, const std::uint64_t tei_packets) {
    FecWindowTelemetry record;
    record.envelope = {.sequence = sequence,
                       .decoder_generation = generation,
                       .source_epoch = epoch,
                       .wall_elapsed_ms = static_cast<double>(sequence) * 10.0};
    record.output_bytes_delta = output_bytes;
    record.output_bytes_cumulative = output_bytes;
    record.delta.ts_packets = packets;
    record.delta.tei_packets = tei_packets;
    record.delta.rs_packets = packets;
    return record;
}

void test_multiple_source_sessions_share_one_report() {
    TemporaryDirectory directory;
    DecodeReport report({.directory = directory.path, .context = "test"});

    StreamDecoderStats initial_one;
    initial_one.decoder_generation = 1;
    initial_one.source_epoch = 1;
    report.begin_source(
        DecodeSourceSessionConfig{.source = "first.cs16",
                                  .destination = "first.ts",
                                  .sample_rate_hz = 10'000'000,
                                  .center_frequency_hz = 545'000'000,
                                  .decoder = {}},
        InputTimelineSnapshot{.stream_epoch = 1,
                              .source_head_sample = 0,
                              .delivered_samples = 0,
                              .sample_rate_hz = 10'000'000},
        initial_one, 0.0);
    const std::array<TelemetryRecord, 1> first_records{
        fec_record(1, 2, 2, 376, 2, 1)};
    report.consume(first_records);
    StreamDecoderStats final_one = initial_one;
    final_one.decoder_generation = 2;
    final_one.source_epoch = 2;
    final_one.processed_input_samples = 1'000;
    report.write_pipeline(final_one, 1'000, 1.0);
    report.end_source("completed", "", final_one,
                      InputTimelineSnapshot{.stream_epoch = 2,
                                            .source_head_sample = 1'000,
                                            .delivered_samples = 1'000,
                                            .sample_rate_hz = 10'000'000},
                      1'000, 1.0);

    StreamDecoderStats initial_two;
    initial_two.decoder_generation = 3;
    initial_two.source_epoch = 3;
    report.begin_source(
        DecodeSourceSessionConfig{.source = "second.cs16",
                                  .destination = "second.ts",
                                  .sample_rate_hz = 8'000'000,
                                  .center_frequency_hz = 557'000'000,
                                  .decoder = {}},
        InputTimelineSnapshot{.stream_epoch = 3,
                              .source_head_sample = 1'000,
                              .delivered_samples = 1'000,
                              .sample_rate_hz = 8'000'000},
        initial_two, 2.0);
    const std::array<TelemetryRecord, 1> second_records{
        fec_record(2, 4, 4, 188, 1, 0)};
    report.consume(second_records);
    StreamDecoderStats final_two = initial_two;
    final_two.decoder_generation = 4;
    final_two.source_epoch = 4;
    final_two.processed_input_samples = 800;
    report.end_source("completed", "", final_two,
                      InputTimelineSnapshot{.stream_epoch = 4,
                                            .source_head_sample = 1'800,
                                            .delivered_samples = 1'800,
                                            .sample_rate_hz = 8'000'000},
                      800, 3.0);
    report.finalize("completed", 0, "", 3.0);

    const json manifest = read_json(directory.path / "manifest.json");
    require(manifest.at("context") == "test", "manifest context is wrong");
    require(!manifest.contains("source"),
            "manifest retained per-source metadata");
    require(manifest.dump().find("source-sessions.jsonl") != std::string::npos,
            "manifest omitted source session stream");

    std::ifstream sessions_stream(directory.path / "source-sessions.jsonl");
    JsonlReader sessions(sessions_stream);
    const auto first = sessions.read();
    const auto second = sessions.read();
    require(first.has_value() && second.has_value(),
            "source session records are missing");
    require(!sessions.read().has_value(),
            "source session stream contains more than two records");
    require(first->value.at("source").at("descriptor") == "first.cs16",
            "first source descriptor is wrong");
    require(first->value.at("correlation").at("first_source_epoch") == 1,
            "first source epoch range is wrong");
    require(first->value.at("correlation").at("last_source_epoch") == 2,
            "last source epoch range is wrong");
    require(first->value.at("transport").at("bytes") == 376,
            "first source transport delta is wrong");
    require(second->value.at("source").at("descriptor") == "second.cs16",
            "second source descriptor is wrong");

    const json stats = read_json(directory.path / "stats.json");
    require(!stats.contains("source") && !stats.contains("sources"),
            "global stats retained per-source metadata");
    require(stats.at("source_sessions").at("total") == 2,
            "global source session count is wrong");
    require(stats.at("source_sessions").at("completed") == 2,
            "global source outcome count is wrong");
    require(stats.at("samples").at("submitted") == 1'800,
            "global submitted sample count is wrong");
    require(stats.at("transport").at("bytes") == 564,
            "global transport bytes did not aggregate FEC deltas");
    require(stats.at("transport").at("decoded_packets") == 3,
            "global decoded packet count is wrong");
    require(stats.at("transport").at("tei_packets") == 1,
            "global TEI packet count is wrong");
}

void test_non_empty_directory_is_rejected() {
    TemporaryDirectory directory;
    std::filesystem::create_directories(directory.path);
    std::ofstream(directory.path / "existing") << "occupied\n";
    try {
        DecodeReport report({.directory = directory.path, .context = "test"});
    } catch (const std::runtime_error &) {
        return;
    }
    throw std::runtime_error("non-empty report directory was accepted");
}

void test_transport_output_stream_and_totals() {
    TemporaryDirectory directory;
    DecodeReport report({.directory = directory.path, .context = "test"});
    StreamDecoderStats stats;
    stats.decoder_generation = 4;
    stats.source_epoch = 7;
    const std::array<TransportOutputTelemetry, 1> baseline{{
        {.name = "offline-ts-output",
         .type = "file",
         .active = true,
         .required = true,
         .blocks_accepted = 10,
         .bytes_accepted = 1'880,
         .blocks_processed = 8,
         .bytes_processed = 1'504,
         .queue_capacity_bytes = 24U << 20U,
         .error = {}},
    }};
    report.begin_source(
        DecodeSourceSessionConfig{.source = "stdin",
                                  .destination = "stdout",
                                  .sample_rate_hz = 10'000'000,
                                  .decoder = {}},
        InputTimelineSnapshot{.stream_epoch = 7, .sample_rate_hz = 10'000'000},
        stats, 0.0, baseline);
    const std::array<TransportOutputTelemetry, 1> sample{{
        {.name = "offline-ts-output",
         .type = "file",
         .active = true,
         .required = true,
         .blocks_accepted = 15,
         .bytes_accepted = 2'820,
         .blocks_processed = 13,
         .bytes_processed = 2'444,
         .dropped_blocks = 2,
         .dropped_bytes = 376,
         .queued_bytes = 376,
         .queue_capacity_bytes = 24U << 20U,
         .error = {}},
    }};
    report.write_transport_outputs(sample, 4, 7, 1.0);
    report.end_source("completed", "", stats,
                      InputTimelineSnapshot{.stream_epoch = 7,
                                            .source_head_sample = 1'000,
                                            .delivered_samples = 1'000,
                                            .sample_rate_hz = 10'000'000},
                      1'000, 1.0);
    report.finalize("completed", 0, "", 1.0);

    std::ifstream stream(directory.path / "transport-outputs.jsonl");
    JsonlReader reader(stream);
    const auto record = reader.read();
    require(record.has_value(), "transport output sample is missing");
    require(record->value.at("schema_version") == 0 &&
                record->value.at("mode") == "dvbt",
            "transport output envelope is wrong");
    require(record->value.at("source_epoch") == 7 &&
                record->value.at("decoder_generation") == 4,
            "transport output correlation is wrong");
    require(record->value.at("queue").at("capacity_bytes") == (24U << 20U),
            "transport output capacity is wrong");

    std::ifstream sessions_stream(directory.path / "source-sessions.jsonl");
    JsonlReader sessions(sessions_stream);
    const auto session = sessions.read();
    require(session.has_value(), "source session is missing");
    const auto &source_output = session->value.at("transport_outputs").at(0);
    require(source_output.at("accepted").at("bytes") == 940,
            "source output bytes did not use its baseline");
    require(source_output.at("dropped").at("blocks") == 2,
            "source output drops were not aggregated");

    const json summary = read_json(directory.path / "stats.json");
    const auto &run_output = summary.at("transport_outputs").at(0);
    require(run_output.at("processed").at("bytes") == 940,
            "run output totals are wrong");
}

} // namespace

int main() {
    test_multiple_source_sessions_share_one_report();
    test_transport_output_stream_and_totals();
    test_non_empty_directory_is_rejected();
}
