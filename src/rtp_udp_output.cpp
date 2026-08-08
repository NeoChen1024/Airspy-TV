#include "airspy_tv/rtp_udp_output.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <memory>
#include <mutex>
#include <netdb.h>
#include <span>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

constexpr std::size_t transport_packet_bytes = 188;
constexpr std::size_t transport_packets_per_datagram = 7;
constexpr std::size_t rtp_payload_bytes =
    transport_packet_bytes * transport_packets_per_datagram;
constexpr std::size_t rtp_header_bytes = 12;
constexpr std::uint8_t mpeg_ts_payload_type = 33;
constexpr std::uint64_t rtp_clock_hz = 90'000;

void put_u16(std::span<std::uint8_t> output, const std::uint16_t value) {
    output[0] = static_cast<std::uint8_t>(value >> 8U);
    output[1] = static_cast<std::uint8_t>(value);
}

void put_u32(std::span<std::uint8_t> output, const std::uint32_t value) {
    output[0] = static_cast<std::uint8_t>(value >> 24U);
    output[1] = static_cast<std::uint8_t>(value >> 16U);
    output[2] = static_cast<std::uint8_t>(value >> 8U);
    output[3] = static_cast<std::uint8_t>(value);
}

[[nodiscard]] std::uint32_t initial_identifier(const std::uintptr_t salt) {
    const auto ticks = static_cast<std::uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    std::uint64_t value = ticks ^ (static_cast<std::uint64_t>(salt) << 17U);
    value ^= value >> 12U;
    value ^= value << 25U;
    value ^= value >> 27U;
    return static_cast<std::uint32_t>(value * 0x2545F4914F6CDD1DULL);
}

} // namespace

bool parse_rtp_udp_endpoint(const std::string_view text,
                            RtpUdpEndpoint &endpoint, std::string &error) {
    std::string_view host;
    std::string_view port_text;
    if (!text.empty() && text.front() == '[') {
        const std::size_t close = text.find(']');
        if (close == std::string_view::npos || close + 1 >= text.size() ||
            text[close + 1] != ':') {
            error = "IPv6 RTP destination must use [address]:port";
            return false;
        }
        host = text.substr(1, close - 1);
        port_text = text.substr(close + 2);
    } else {
        const std::size_t separator = text.rfind(':');
        if (separator == std::string_view::npos ||
            text.find(':') != separator) {
            error = "RTP destination must use host:port or [IPv6]:port";
            return false;
        }
        host = text.substr(0, separator);
        port_text = text.substr(separator + 1);
    }
    std::uint32_t port = 0;
    const auto parsed =
        std::from_chars(port_text.begin(), port_text.end(), port);
    if (host.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != port_text.end() || port == 0 || port > 65'535) {
        error = "RTP destination has an invalid host or port";
        return false;
    }
    endpoint = {.host = std::string(host),
                .port = static_cast<std::uint16_t>(port)};
    return true;
}

std::string format_rtp_udp_endpoint(const RtpUdpEndpoint &endpoint) {
    const bool ipv6_literal = endpoint.host.find(':') != std::string::npos;
    return ipv6_literal ? std::format("[{}]:{}", endpoint.host, endpoint.port)
                        : std::format("{}:{}", endpoint.host, endpoint.port);
}

struct RtpUdpTransportOutput::Impl {
    Impl()
        : output(TransportOutputConfig{
              .queue_capacity_bytes = 1U << 20U,
              .overflow_policy = TransportOverflowPolicy::drop_oldest,
              .criticality = TransportSinkCriticality::optional,
              .write_error_policy = TransportWriteErrorPolicy::drop_block,
              .thread_name = "rtp-udp-output",
          }) {
        pending.reserve(rtp_payload_bytes);
    }

    [[nodiscard]] std::vector<std::uint8_t>
    make_datagram(const std::span<const std::uint8_t> payload) {
        std::vector<std::uint8_t> datagram(rtp_header_bytes + payload.size());
        datagram[0] = 0x80;
        datagram[1] = mpeg_ts_payload_type;
        put_u16(std::span(datagram).subspan(2, 2), sequence++);
        const auto elapsed = std::chrono::steady_clock::now() - started_at;
        const auto timestamp_ticks =
            std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)
                .count();
        const auto timestamp_delta = static_cast<std::uint64_t>(
            (static_cast<long double>(timestamp_ticks) * rtp_clock_hz) /
            1'000'000'000.0L);
        put_u32(std::span(datagram).subspan(4, 4),
                timestamp_base + static_cast<std::uint32_t>(timestamp_delta));
        put_u32(std::span(datagram).subspan(8, 4), ssrc);
        std::ranges::copy(payload,
                          datagram.begin() +
                              static_cast<std::ptrdiff_t>(rtp_header_bytes));
        return datagram;
    }

    mutable std::mutex mutex;
    AsyncTransportOutput output;
    std::vector<std::uint8_t> pending;
    std::chrono::steady_clock::time_point started_at;
    RtpUdpEndpoint endpoint;
    std::uint16_t sequence{};
    std::uint32_t timestamp_base{};
    std::uint32_t ssrc{};
    bool active{};
};

RtpUdpTransportOutput::RtpUdpTransportOutput()
    : impl_(std::make_unique<Impl>()) {}

RtpUdpTransportOutput::~RtpUdpTransportOutput() noexcept { stop(); }

bool RtpUdpTransportOutput::start(const RtpUdpEndpoint &endpoint,
                                  std::string &error) {
    stop();
    if (endpoint.host.empty() || endpoint.port == 0) {
        error = "RTP/UDP destination host and port are required";
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo *raw_addresses = nullptr;
    const std::string service = std::to_string(endpoint.port);
    const int resolve_result = ::getaddrinfo(
        endpoint.host.c_str(), service.c_str(), &hints, &raw_addresses);
    if (resolve_result != 0) {
        error = "Unable to resolve RTP/UDP destination: " +
                std::string(::gai_strerror(resolve_result));
        return false;
    }
    const std::unique_ptr<addrinfo, decltype(&::freeaddrinfo)> addresses(
        raw_addresses, &::freeaddrinfo);

    int socket_fd = -1;
    std::string socket_error;
    for (const addrinfo *address = addresses.get(); address != nullptr;
         address = address->ai_next) {
        socket_fd =
            ::socket(address->ai_family,
                     address->ai_socktype | SOCK_CLOEXEC | SOCK_NONBLOCK,
                     address->ai_protocol);
        if (socket_fd < 0) {
            socket_error = std::strerror(errno);
            continue;
        }
        if (::connect(socket_fd, address->ai_addr, address->ai_addrlen) == 0) {
            break;
        }
        socket_error = std::strerror(errno);
        static_cast<void>(::close(socket_fd));
        socket_fd = -1;
    }
    if (socket_fd < 0) {
        error = "Unable to connect RTP/UDP socket: " + socket_error;
        return false;
    }
    if (!impl_->output.start_fd(socket_fd, true,
                                "RTP/UDP " + format_rtp_udp_endpoint(endpoint),
                                error)) {
        return false;
    }

    const std::scoped_lock lock(impl_->mutex);
    impl_->endpoint = endpoint;
    impl_->pending.clear();
    impl_->started_at = std::chrono::steady_clock::now();
    const auto salt = reinterpret_cast<std::uintptr_t>(impl_.get());
    impl_->ssrc = initial_identifier(salt);
    impl_->timestamp_base = initial_identifier(salt ^ 0x9E3779B9U);
    impl_->sequence = static_cast<std::uint16_t>(
        initial_identifier(salt ^ static_cast<std::uintptr_t>(0x85EBCA6BU)));
    impl_->active = true;
    return true;
}

void RtpUdpTransportOutput::submit(
    const std::span<const std::uint8_t> transport_stream) {
    if (transport_stream.empty()) {
        return;
    }
    const std::scoped_lock lock(impl_->mutex);
    if (!impl_->active) {
        return;
    }

    std::size_t offset = 0;
    if (!impl_->pending.empty()) {
        const std::size_t copied = std::min(
            rtp_payload_bytes - impl_->pending.size(), transport_stream.size());
        impl_->pending.insert(impl_->pending.end(), transport_stream.begin(),
                              transport_stream.begin() +
                                  static_cast<std::ptrdiff_t>(copied));
        offset += copied;
        if (impl_->pending.size() == rtp_payload_bytes) {
            static_cast<void>(
                impl_->output.submit(impl_->make_datagram(impl_->pending)));
            impl_->pending.clear();
        }
    }
    while (transport_stream.size() - offset >= rtp_payload_bytes) {
        const auto payload =
            transport_stream.subspan(offset, rtp_payload_bytes);
        static_cast<void>(impl_->output.submit(impl_->make_datagram(payload)));
        offset += rtp_payload_bytes;
    }
    impl_->pending.insert(impl_->pending.end(),
                          transport_stream.begin() +
                              static_cast<std::ptrdiff_t>(offset),
                          transport_stream.end());
}

void RtpUdpTransportOutput::stop() noexcept {
    std::vector<std::uint8_t> final_datagram;
    {
        const std::scoped_lock lock(impl_->mutex);
        if (!impl_->active) {
            return;
        }
        const std::size_t complete_bytes =
            (impl_->pending.size() / transport_packet_bytes) *
            transport_packet_bytes;
        if (complete_bytes != 0) {
            final_datagram = impl_->make_datagram(
                std::span(impl_->pending).first(complete_bytes));
        }
        impl_->pending.clear();
        impl_->active = false;
    }
    if (!final_datagram.empty()) {
        static_cast<void>(impl_->output.submit(final_datagram));
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    while (impl_->output.stats().queued_bytes != 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    impl_->output.stop(false);
}

RtpUdpStats RtpUdpTransportOutput::stats() const {
    const auto output = impl_->output.stats();
    return {
        .active = output.active,
        .elapsed_milliseconds = output.elapsed_milliseconds,
        .wire_bytes_sent = output.bytes_written,
        .datagrams_sent = output.blocks_written,
        .dropped_datagrams = output.dropped_blocks,
        .dropped_wire_bytes = output.dropped_bytes,
        .write_errors = output.write_errors,
        .queued_bytes = output.queued_bytes,
        .last_error = output.error,
    };
}

} // namespace airspy_tv
