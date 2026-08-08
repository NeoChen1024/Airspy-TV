#include "airspy_tv/rtp_udp_output.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <poll.h>
#include <span>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr std::size_t ts_packet_bytes = 188;
constexpr std::size_t rtp_payload_bytes = ts_packet_bytes * 7;
constexpr std::size_t rtp_datagram_bytes = 12 + rtp_payload_bytes;

bool require(const bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

class UdpReceiver {
  public:
    explicit UdpReceiver(const int family)
        : fd_(::socket(family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP)) {
        if (fd_ < 0) {
            return;
        }
        if (family == AF_INET) {
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            if (::bind(fd_, reinterpret_cast<const sockaddr *>(&address),
                       sizeof(address)) != 0) {
                close();
                return;
            }
            socklen_t size = sizeof(address);
            if (::getsockname(fd_, reinterpret_cast<sockaddr *>(&address),
                              &size) != 0) {
                close();
                return;
            }
            port_ = ntohs(address.sin_port);
        } else {
            sockaddr_in6 address{};
            address.sin6_family = AF_INET6;
            address.sin6_addr = in6addr_loopback;
            if (::bind(fd_, reinterpret_cast<const sockaddr *>(&address),
                       sizeof(address)) != 0) {
                close();
                return;
            }
            socklen_t size = sizeof(address);
            if (::getsockname(fd_, reinterpret_cast<sockaddr *>(&address),
                              &size) != 0) {
                close();
                return;
            }
            port_ = ntohs(address.sin6_port);
        }
    }

    ~UdpReceiver() { close(); }

    UdpReceiver(const UdpReceiver &) = delete;
    UdpReceiver &operator=(const UdpReceiver &) = delete;
    UdpReceiver(UdpReceiver &&) = delete;
    UdpReceiver &operator=(UdpReceiver &&) = delete;

    [[nodiscard]] bool valid() const { return fd_ >= 0 && port_ != 0; }
    [[nodiscard]] std::uint16_t port() const { return port_; }

    [[nodiscard]] std::vector<std::uint8_t> receive() const {
        pollfd input{.fd = fd_, .events = POLLIN, .revents = 0};
        if (::poll(&input, 1, 1000) <= 0) {
            return {};
        }
        std::vector<std::uint8_t> datagram(2048);
        const ssize_t received =
            ::recv(fd_, datagram.data(), datagram.size(), 0);
        if (received <= 0) {
            return {};
        }
        datagram.resize(static_cast<std::size_t>(received));
        return datagram;
    }

  private:
    void close() {
        if (fd_ >= 0) {
            static_cast<void>(::close(fd_));
            fd_ = -1;
        }
    }

    int fd_{-1};
    std::uint16_t port_{};
};

[[nodiscard]] std::uint16_t
read_u16(const std::span<const std::uint8_t> bytes) {
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
}

std::vector<std::uint8_t> make_transport_payload(const std::uint8_t tag) {
    std::vector<std::uint8_t> payload(rtp_payload_bytes);
    for (std::size_t packet = 0; packet < 7; ++packet) {
        const std::size_t offset = packet * ts_packet_bytes;
        payload[offset] = 0x47;
        std::fill(payload.begin() + static_cast<std::ptrdiff_t>(offset + 1),
                  payload.begin() +
                      static_cast<std::ptrdiff_t>(offset + ts_packet_bytes),
                  static_cast<std::uint8_t>(tag + packet));
    }
    return payload;
}

bool check_datagram(const std::vector<std::uint8_t> &datagram,
                    const std::vector<std::uint8_t> &payload) {
    return require(datagram.size() == rtp_datagram_bytes,
                   "RTP datagram has a 12-byte header and 1316-byte payload") &&
           require(datagram[0] == 0x80, "RTP version is 2") &&
           require(datagram[1] == 33, "RTP payload type is MPEG-TS (33)") &&
           require(std::equal(payload.begin(), payload.end(),
                              datagram.begin() + 12),
                   "RTP payload preserves all seven TS packets");
}

bool test_endpoint_parser() {
    airspy_tv::RtpUdpEndpoint endpoint;
    std::string error;
    return require(airspy_tv::parse_rtp_udp_endpoint("127.0.0.1:5004", endpoint,
                                                     error) &&
                       endpoint.host == "127.0.0.1" && endpoint.port == 5004,
                   "IPv4 endpoint parses") &&
           require(airspy_tv::parse_rtp_udp_endpoint("[::1]:5006", endpoint,
                                                     error) &&
                       endpoint.host == "::1" && endpoint.port == 5006,
                   "bracketed IPv6 endpoint parses") &&
           require(
               !airspy_tv::parse_rtp_udp_endpoint("::1:5006", endpoint, error),
               "unbracketed IPv6 endpoint is rejected as ambiguous") &&
           require(airspy_tv::format_rtp_udp_endpoint(
                       {.host = "::1", .port = 5006}) == "[::1]:5006",
                   "IPv6 endpoint formatting restores brackets");
}

bool test_loopback(const int family, const std::string &host) {
    const UdpReceiver receiver(family);
    if (family == AF_INET6 && !receiver.valid()) {
        std::cerr << "SKIP: IPv6 loopback is unavailable\n";
        return true;
    }
    if (!require(receiver.valid(), "UDP loopback receiver starts")) {
        return false;
    }

    airspy_tv::RtpUdpTransportOutput output;
    std::string error;
    if (!require(output.start({.host = host, .port = receiver.port()}, error),
                 "RTP/UDP output starts: " + error)) {
        return false;
    }

    const auto first_payload = make_transport_payload(0x10);
    output.submit(std::span(first_payload).first(500));
    output.submit(std::span(first_payload).subspan(500));
    const auto first = receiver.receive();
    if (!check_datagram(first, first_payload)) {
        return false;
    }

    const auto second_payload = make_transport_payload(0x20);
    output.submit(second_payload);
    const auto second = receiver.receive();
    const bool sequence_ok =
        check_datagram(second, second_payload) &&
        require(static_cast<std::uint16_t>(
                    read_u16(std::span(first).subspan(2, 2)) + 1U) ==
                    read_u16(std::span(second).subspan(2, 2)),
                "RTP sequence increments once per datagram");
    output.stop();
    const auto stats = output.stats();
    return sequence_ok &&
           require(stats.datagrams_sent == 2,
                   "RTP stats count sent datagrams") &&
           require(stats.dropped_datagrams == 0,
                   "loopback RTP sends do not drop");
}

bool test_partial_group_flush() {
    const UdpReceiver receiver(AF_INET);
    if (!require(receiver.valid(), "partial-flush receiver starts")) {
        return false;
    }

    airspy_tv::RtpUdpTransportOutput output;
    std::string error;
    if (!require(
            output.start({.host = "127.0.0.1", .port = receiver.port()}, error),
            "partial-flush output starts: " + error)) {
        return false;
    }
    std::vector<std::uint8_t> payload((ts_packet_bytes * 2) + 3, 0x5A);
    payload[0] = 0x47;
    payload[ts_packet_bytes] = 0x47;
    output.submit(payload);
    output.stop();

    const auto datagram = receiver.receive();
    const std::size_t expected_size = 12 + (ts_packet_bytes * 2);
    return require(datagram.size() == expected_size,
                   "stop flushes two complete TS packets") &&
           require(std::equal(payload.begin(),
                              payload.begin() + static_cast<std::ptrdiff_t>(
                                                    ts_packet_bytes * 2),
                              datagram.begin() + 12),
                   "partial flush preserves complete packets and drops tail");
}

} // namespace

int main() {
    const bool ok =
        test_endpoint_parser() && test_loopback(AF_INET, "127.0.0.1") &&
        test_loopback(AF_INET6, "::1") && test_partial_group_flush();
    return ok ? 0 : 1;
}
