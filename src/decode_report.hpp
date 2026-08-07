#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <ostream>
#include <span>
#include <string>
#include <string_view>

namespace airspy_tv {

struct DecodeReportConfig {
    std::filesystem::path directory;
    std::string source;
    std::string destination;
    std::uint32_t sample_rate_hz{};
    dvbt::ReceiverParameters decoder;
};

class DecodeReport {
  public:
    explicit DecodeReport(DecodeReportConfig config);
    ~DecodeReport() noexcept;

    DecodeReport(const DecodeReport &) = delete;
    DecodeReport &operator=(const DecodeReport &) = delete;

    void consume(std::span<const dvbt::TelemetryRecord> records);
    void write_pipeline(const dvbt::StreamDecoderStats &stats,
                        std::uint64_t submitted_samples,
                        double wall_elapsed_seconds);
    void flush();
    void finalize(std::string_view status, int exit_code,
                  std::string_view error, const dvbt::StreamDecoderStats &stats,
                  std::uint64_t submitted_samples, double wall_elapsed_seconds);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

void format_debug_telemetry(std::ostream &stream,
                            const dvbt::TelemetryRecord &record);

} // namespace airspy_tv
