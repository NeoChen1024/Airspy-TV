#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "fec_stage.hpp"
#include "frontend_stage.hpp"
#include "sample_channel.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace airspy_tv::dvbt {

class AnalysisPublisher;
class ClockControlTimeline;
class SampleChannel;

class DemodStage {
  public:
    struct Callbacks {
        std::function<void(TransportDiscontinuity)> emit_discontinuity;
        std::function<void()> notify_idle;
        std::function<void(std::exception_ptr, std::string)> worker_failure;
    };

    DemodStage(SampleChannel &samples, ClockControlTimeline &clock,
               FecStage &fec, AnalysisPublisher &analysis, Callbacks callbacks);
    ~DemodStage() noexcept;

    DemodStage(const DemodStage &) = delete;
    DemodStage &operator=(const DemodStage &) = delete;
    DemodStage(DemodStage &&) = delete;
    DemodStage &operator=(DemodStage &&) = delete;

    void stop() noexcept;
    [[nodiscard]] WorkerState worker_state() const noexcept;

    void set_parameters(const ReceiverParameters &parameters);
    [[nodiscard]] ReceiverParameters parameters() const;
    void set_equalized_callback(StreamDecoder::EqualizedCallback callback);
    [[nodiscard]] std::uint64_t reset();

    void input_block_started(std::uint64_t generation);
    void input_block_dropped();
    [[nodiscard]] std::uint64_t invalidate_sync(std::uint64_t generation);
    [[nodiscard]] std::uint64_t
    publish_rebootstrap(const SampleChannel::RebootstrapResult &result);
    void publish_bootstrap_progress(const FrontendBootstrapProgress &progress);
    [[nodiscard]] std::uint64_t
    publish_acquisition(const FrontendAcquisitionPublication &publication);
    void publish_command(const FrontendCommandPublication &publication);
    void publish_frontend_block(const FrontendBlockPublication &publication);

    void publish_fec_session(const FecStageSession &session);
    void publish_fec_window(const FecStageWindow &window);
    [[nodiscard]] bool events_enabled() const noexcept;
    void emit_fec_diagnostic(DiagnosticEvent event,
                             const FecStageDiagnosticContext &context);

    [[nodiscard]] StreamDecoderStats stats() const;
    void set_telemetry_enabled(
        bool enabled,
        TelemetryClock::time_point run_started_at = TelemetryClock::now());
    [[nodiscard]] std::vector<TelemetryRecord> drain_telemetry();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
