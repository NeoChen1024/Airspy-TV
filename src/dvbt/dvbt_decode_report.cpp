#include "dvbt_decode_report.hpp"

#include "dvbt_report_codec.hpp"
#include "dvbt_report_writer.hpp"
#include "receiver_session.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

namespace airspy_tv {
namespace {

[[nodiscard]] DecodeReportSample
report_sample(const dvbt::StreamDecoderStats &stats) {
    return {.failed = stats.failed,
            .error = stats.error,
            .transport_bytes = stats.transport_bytes};
}

class DvbTDecodeReport final : public DecodeReport {
  public:
    DecodeReportSample
    capture_baseline(const ReceiverSession &session) override {
        baseline_ = session.dvbt_snapshot().decoder;
        return report_sample(baseline_);
    }

    DecodeReportSample sample(const ReceiverSession &session) override {
        stats_ = session.dvbt_snapshot().decoder;
        return report_sample(stats_);
    }

    void drain_telemetry(ReceiverSession &session, const bool echo) override {
        const auto records = session.drain_dvbt_telemetry();
        if (writer_ != nullptr) {
            writer_->consume(records);
        }
        if (echo) {
            for (const auto &record : records) {
                format_debug_telemetry(std::cerr, record);
            }
        }
    }

    [[nodiscard]] bool is_open() const noexcept override {
        return writer_ != nullptr;
    }

    void open(DecodeReportConfig config) override {
        writer_ = std::make_unique<DvbTReportWriter>(std::move(config));
    }

    void close() noexcept override { writer_.reset(); }

    void begin_source(
        const ReceiverSession &session, DecodeSourceSessionConfig config,
        const InputTimelineSnapshot &timeline,
        const double wall_elapsed_seconds,
        const std::span<const TransportOutputTelemetry> outputs) override {
        const auto current = session.dvbt_snapshot().decoder;
        auto initial = baseline_;
        initial.decoder_generation = current.decoder_generation;
        initial.source_epoch = current.source_epoch;
        writer().begin_source(
            DvbTSourceSessionConfig{
                .source = std::move(config.source),
                .destination = std::move(config.destination),
                .sample_rate_hz = config.sample_rate_hz,
                .center_frequency_hz = config.center_frequency_hz,
                .decoder = session.dvbt_parameters(),
            },
            timeline, initial, wall_elapsed_seconds, outputs);
    }

    void write_sample(
        const std::uint64_t submitted_samples,
        const double wall_elapsed_seconds,
        const std::span<const TransportOutputTelemetry> outputs) override {
        writer().write_pipeline(stats_, submitted_samples,
                                wall_elapsed_seconds);
        writer().write_transport_outputs(outputs, stats_.decoder_generation,
                                         stats_.source_epoch,
                                         wall_elapsed_seconds);
    }

    void end_source(const std::string_view status, const std::string_view error,
                    const InputTimelineSnapshot &timeline,
                    const std::uint64_t submitted_samples,
                    const double wall_elapsed_seconds) override {
        writer().end_source(status, error, stats_, timeline, submitted_samples,
                            wall_elapsed_seconds);
    }

    void flush() override { writer().flush(); }

    void finalize(const std::string_view status, const int exit_code,
                  const std::string_view error,
                  const double wall_elapsed_seconds) override {
        writer().finalize(status, exit_code, error, wall_elapsed_seconds);
    }

  private:
    [[nodiscard]] DvbTReportWriter &writer() {
        if (writer_ == nullptr) {
            throw std::logic_error("DVB-T decode report is not open");
        }
        return *writer_;
    }

    std::unique_ptr<DvbTReportWriter> writer_;
    dvbt::StreamDecoderStats baseline_;
    dvbt::StreamDecoderStats stats_;
};

} // namespace

std::unique_ptr<DecodeReport> make_dvbt_decode_report() {
    return std::make_unique<DvbTDecodeReport>();
}

} // namespace airspy_tv
