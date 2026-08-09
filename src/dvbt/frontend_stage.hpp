#pragma once

#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "clock_control_timeline.hpp"
#include "sample_channel.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace airspy_tv::dvbt {

struct FrontendBootstrapProgress {
    std::uint64_t generation{};
    std::uint64_t attempts{};
    std::uint64_t retained_peak_samples{};
    std::optional<float> acquisition_time_ms;
};

struct FrontendAcquisitionPublication {
    std::uint64_t generation{};
    OfdmAcquisition acquisition;
    std::uint64_t output_base{};
    std::uint32_t bandwidth_hz{};
    float resampled_rate_hz{};
    float initial_cfo_hz{};
    float initial_fractional_cfo_hz{};
    std::uint64_t bootstrap_attempts{};
    std::uint64_t bootstrap_replayed_input_samples{};
    std::uint64_t bootstrap_retained_peak_samples{};
};

struct FrontendCommandPublication {
    std::uint64_t generation{};
    std::uint64_t input_sample{};
    std::uint64_t output_sample{};
    std::optional<ClockControlTimeline::SroCommand> sro;
    std::optional<ClockControlTimeline::CfoCommand> cfo;
    std::size_t pending_sro{};
    std::size_t pending_cfo{};
};

struct FrontendBlockPublication {
    std::uint64_t generation{};
    std::uint32_t input_rate_hz{};
    std::uint32_t bandwidth_hz{};
    InputSampleStamp stamp;
    std::size_t input_samples{};
    std::uint64_t resampled_begin_sample{};
    std::uint64_t resampled_end_sample{};
    std::uint64_t bootstrap_attempts{};
    std::uint64_t bootstrap_replayed_input_samples{};
    std::uint64_t bootstrap_retained_peak_samples{};
    ClockControlTimeline::Snapshot clock;
    float total_time_ms{};
    float convert_time_ms{};
    float resample_time_ms{};
    float ring_copy_time_ms{};
    float ring_wait_time_ms{};
    double requested_ratio{};
    double effective_ratio{};
    bool abandoned{};
};

class FrontendStage {
  public:
    struct Callbacks {
        std::function<ReceiverParameters()> parameters;
        std::function<void(std::uint64_t)> input_block_started;
        std::function<std::uint64_t(std::uint64_t)> invalidate_sync;
        std::function<std::uint64_t(const SampleChannel::RebootstrapResult &)>
            publish_rebootstrap;
        std::function<void(const FrontendBootstrapProgress &)>
            publish_bootstrap_progress;
        std::function<std::uint64_t(const FrontendAcquisitionPublication &)>
            publish_acquisition;
        std::function<void(const FrontendCommandPublication &)> publish_command;
        std::function<void(const FrontendBlockPublication &)> publish_block;
        std::function<void()> clear_fec;
        std::function<void()> notify_fec_cancelled;
        std::function<void(std::exception_ptr, std::string)> worker_failure;
    };

    FrontendStage(SampleChannel &samples, ClockControlTimeline &clock,
                  Callbacks callbacks);
    ~FrontendStage() noexcept;

    FrontendStage(const FrontendStage &) = delete;
    FrontendStage &operator=(const FrontendStage &) = delete;
    FrontendStage(FrontendStage &&) = delete;
    FrontendStage &operator=(FrontendStage &&) = delete;

    void stop() noexcept;
    [[nodiscard]] WorkerState worker_state() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
