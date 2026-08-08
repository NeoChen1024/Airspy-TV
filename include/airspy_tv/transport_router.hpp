#pragma once

#include "airspy_tv/recorder.hpp"
#include "airspy_tv/rtp_udp_output.hpp"
#include "airspy_tv/transport_stream.hpp"

#include <cstdint>
#include <functional>
#include <mutex>
#include <span>

namespace airspy_tv {

// Ordered MPEG-TS fanout owned by the receiver session, independent of source
// callback locks. Consumers run on the demod/FEC output thread and must remain
// bounded; changing the optional external sink is synchronized separately.
class TransportStreamRouter {
  public:
    using Sink = std::function<void(std::span<const std::uint8_t>)>;

    TransportStreamRouter(TransportStreamModel &model,
                          TransportStreamRecorder &recorder,
                          RtpUdpTransportOutput &rtp_output) noexcept;

    void consume(std::span<const std::uint8_t> transport_stream);
    void set_sink(Sink sink);

  private:
    TransportStreamModel &model_;
    TransportStreamRecorder &recorder_;
    RtpUdpTransportOutput &rtp_output_;
    mutable std::mutex sink_mutex_;
    Sink sink_;
};

} // namespace airspy_tv
