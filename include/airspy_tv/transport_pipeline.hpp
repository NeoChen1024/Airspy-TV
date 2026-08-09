#pragma once

#include "airspy_tv/epg.hpp"
#include "airspy_tv/recorder.hpp"
#include "airspy_tv/rtp_udp_output.hpp"
#include "airspy_tv/transport_observer.hpp"
#include "airspy_tv/transport_stream.hpp"
#include "airspy_tv/transport_telemetry.hpp"

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace airspy_tv {

struct TransportPipelineConfig {
    bool metadata_observers_enabled{true};

    [[nodiscard]] static constexpr TransportPipelineConfig headless() noexcept {
        return {.metadata_observers_enabled = false};
    }
};

struct TransportPipelineSnapshot {
    TransportObserverStats service_observer;
    TransportObserverStats epg_observer;
    TransportRecordingStats recorder;
    RtpUdpStats rtp;
};

class TransportPipeline {
  public:
    using Sink = std::function<void(std::span<const std::uint8_t>)>;
    using DiscontinuitySink = std::function<void(TransportDiscontinuity)>;

    explicit TransportPipeline(TransportPipelineConfig config = {});
    ~TransportPipeline() noexcept;

    TransportPipeline(const TransportPipeline &) = delete;
    TransportPipeline &operator=(const TransportPipeline &) = delete;
    TransportPipeline(TransportPipeline &&) = delete;
    TransportPipeline &operator=(TransportPipeline &&) = delete;

    void consume(std::span<const std::uint8_t> transport_stream);
    void notify_discontinuity(TransportDiscontinuity discontinuity);
    void set_sink(Sink sink);
    void set_discontinuity_sink(DiscontinuitySink discontinuity_sink);

    bool start_recording(const std::filesystem::path &path, std::string &error);
    void stop_recording() noexcept;
    bool start_rtp(const RtpUdpEndpoint &endpoint, std::string &error);
    void stop_rtp() noexcept;
    void stop() noexcept;

    [[nodiscard]] std::vector<TransportService> services() const;
    [[nodiscard]] EpgSnapshot epg_snapshot(std::uint16_t service_id) const;
    [[nodiscard]] TransportPipelineSnapshot snapshot() const;
    [[nodiscard]] std::vector<TransportOutputTelemetry>
    output_telemetry() const;
    [[nodiscard]] std::string runtime_error() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
