#pragma once

#include "airspy_tv/sample_timeline.hpp"
#include "airspy_tv/transport_telemetry.hpp"

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>

namespace airspy_tv {

class ReceiverSession;

struct DecodeReportConfig {
    std::filesystem::path directory;
    std::string context;
};

struct DecodeSourceSessionConfig {
    std::string source;
    std::string destination;
    std::uint32_t sample_rate_hz{};
    std::uint64_t center_frequency_hz{};
};

// The decoder facts the run lifecycle needs to classify a source session.
struct DecodeReportSample {
    bool failed{};
    std::string error;
    std::uint64_t transport_bytes{};
};

// Standard-specific half of a run report. DecodeRunReporter owns the source
// and run lifecycle; an implementation maps the active demodulator's typed
// parameters, statistics and telemetry onto its own report streams.
class DecodeReport {
  public:
    DecodeReport() = default;
    virtual ~DecodeReport() = default;
    DecodeReport(const DecodeReport &) = delete;
    DecodeReport &operator=(const DecodeReport &) = delete;

    // Remember the decoder counters that precede the next source session so
    // its totals exclude earlier sessions.
    virtual DecodeReportSample
    capture_baseline(const ReceiverSession &session) = 0;
    // Take one decoder snapshot. write_sample() and end_source() serialize
    // exactly this snapshot, so the caller's decisions match the records.
    virtual DecodeReportSample sample(const ReceiverSession &session) = 0;
    // Move pending telemetry out of the demodulator: into the report streams
    // when the report is open, and to stderr when echo is set.
    virtual void drain_telemetry(ReceiverSession &session, bool echo) = 0;

    [[nodiscard]] virtual bool is_open() const noexcept = 0;
    // Create the report directory and manifest.
    virtual void open(DecodeReportConfig config) = 0;
    // Drop the report streams without finalizing them.
    virtual void close() noexcept = 0;

    virtual void begin_source(
        const ReceiverSession &session, DecodeSourceSessionConfig config,
        const InputTimelineSnapshot &timeline, double wall_elapsed_seconds,
        std::span<const TransportOutputTelemetry> outputs) = 0;
    virtual void
    write_sample(std::uint64_t submitted_samples, double wall_elapsed_seconds,
                 std::span<const TransportOutputTelemetry> outputs) = 0;
    virtual void end_source(std::string_view status, std::string_view error,
                            const InputTimelineSnapshot &timeline,
                            std::uint64_t submitted_samples,
                            double wall_elapsed_seconds) = 0;
    virtual void flush() = 0;
    virtual void finalize(std::string_view status, int exit_code,
                          std::string_view error,
                          double wall_elapsed_seconds) = 0;
};

} // namespace airspy_tv
