#pragma once

#include "airspy_tv/transport_output.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace airspy_tv {

struct RtpUdpEndpoint {
    std::string host;
    std::uint16_t port{};
};

struct RtpUdpStats {
    bool active{};
    bool failed{};
    std::uint64_t elapsed_milliseconds{};
    std::uint64_t datagrams_accepted{};
    std::uint64_t wire_bytes_accepted{};
    std::uint64_t wire_bytes_sent{};
    std::uint64_t datagrams_sent{};
    std::uint64_t dropped_datagrams{};
    std::uint64_t dropped_wire_bytes{};
    std::uint64_t write_errors{};
    std::size_t queued_bytes{};
    std::size_t queue_capacity_bytes{};
    std::string last_error;
};

[[nodiscard]] bool parse_rtp_udp_endpoint(std::string_view text,
                                          RtpUdpEndpoint &endpoint,
                                          std::string &error);
[[nodiscard]] std::string
format_rtp_udp_endpoint(const RtpUdpEndpoint &endpoint);

// RFC 2250 MPEG-TS over RTP/UDP. RTP uses static payload type 33 and a 90 kHz
// monotonic timestamp. Datagram payloads normally contain 7 x 188-byte TS
// packets (1316 bytes) to stay below the common Ethernet MTU.
class RtpUdpTransportOutput {
  public:
    RtpUdpTransportOutput();
    ~RtpUdpTransportOutput() noexcept;

    RtpUdpTransportOutput(const RtpUdpTransportOutput &) = delete;
    RtpUdpTransportOutput &operator=(const RtpUdpTransportOutput &) = delete;
    RtpUdpTransportOutput(RtpUdpTransportOutput &&) = delete;
    RtpUdpTransportOutput &operator=(RtpUdpTransportOutput &&) = delete;

    bool start(const RtpUdpEndpoint &endpoint, std::string &error);
    void submit(std::span<const std::uint8_t> transport_stream);
    void discard_queued() noexcept;
    void stop() noexcept;

    [[nodiscard]] RtpUdpStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
