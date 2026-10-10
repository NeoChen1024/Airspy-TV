#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/transport_telemetry.hpp"
#include "decode_report.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace airspy_tv {

struct DvbTSourceSessionConfig {
    std::string source;
    std::string destination;
    std::uint32_t sample_rate_hz{};
    std::uint64_t center_frequency_hz{};
    dvbt::ReceiverParameters decoder;
};

// Serializes typed DVB-T statistics and telemetry into one run report. It is
// driven with plain values so the output can be verified without a receiver.
class DvbTReportWriter {
  public:
    explicit DvbTReportWriter(DecodeReportConfig config);
    ~DvbTReportWriter() noexcept;

    DvbTReportWriter(const DvbTReportWriter &) = delete;
    DvbTReportWriter &operator=(const DvbTReportWriter &) = delete;

    void begin_source(DvbTSourceSessionConfig config,
                      const InputTimelineSnapshot &timeline,
                      const dvbt::StreamDecoderStats &stats,
                      double wall_elapsed_seconds,
                      std::span<const TransportOutputTelemetry> outputs = {});
    void consume(std::span<const dvbt::TelemetryRecord> records);
    void write_pipeline(const dvbt::StreamDecoderStats &stats,
                        std::uint64_t submitted_samples,
                        double wall_elapsed_seconds);
    void
    write_transport_outputs(std::span<const TransportOutputTelemetry> outputs,
                            std::uint64_t decoder_generation,
                            std::uint64_t source_epoch,
                            double wall_elapsed_seconds);
    void end_source(std::string_view status, std::string_view error,
                    const dvbt::StreamDecoderStats &stats,
                    const InputTimelineSnapshot &timeline,
                    std::uint64_t submitted_samples,
                    double wall_elapsed_seconds);
    void flush();
    void finalize(std::string_view status, int exit_code,
                  std::string_view error, double wall_elapsed_seconds);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
