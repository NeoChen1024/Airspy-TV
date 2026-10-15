#include "airspy_tv/transport_clock_monitor.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::TransportClockMonitor;
using Clock = TransportClockMonitor::Clock;
using Packet = std::array<std::uint8_t, 188>;

constexpr std::uint64_t ticks_per_second = 27'000'000;
constexpr std::uint64_t pcr_base_modulus = std::uint64_t{1} << 33U;

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

Packet null_packet() {
    Packet packet{};
    packet.fill(0xFF);
    packet[0] = 0x47;
    packet[1] = 0x1F;
    packet[2] = 0xFF;
    packet[3] = 0x10;
    return packet;
}

Packet pcr_packet(const std::uint16_t pid, const std::uint64_t pcr,
                  const bool discontinuity) {
    Packet packet{};
    packet.fill(0xFF);
    packet[0] = 0x47;
    packet[1] = static_cast<std::uint8_t>(pid >> 8U);
    packet[2] = static_cast<std::uint8_t>(pid & 0xFFU);
    packet[3] = 0x30;
    packet[4] = 7;
    packet[5] = static_cast<std::uint8_t>(0x10U | (discontinuity ? 0x80U : 0U));
    const std::uint64_t base = (pcr / 300) % pcr_base_modulus;
    const std::uint64_t extension = pcr % 300;
    packet[6] = static_cast<std::uint8_t>(base >> 25U);
    packet[7] = static_cast<std::uint8_t>(base >> 17U);
    packet[8] = static_cast<std::uint8_t>(base >> 9U);
    packet[9] = static_cast<std::uint8_t>(base >> 1U);
    packet[10] = static_cast<std::uint8_t>(((base & 1U) << 7U) | 0x7EU |
                                           (extension >> 8U));
    packet[11] = static_cast<std::uint8_t>(extension & 0xFFU);
    return packet;
}

void write_timestamp(std::uint8_t *field, const unsigned int prefix,
                     const std::uint64_t value) {
    field[0] = static_cast<std::uint8_t>((prefix << 4U) |
                                         (((value >> 30U) & 0x7U) << 1U) | 1U);
    field[1] = static_cast<std::uint8_t>((value >> 22U) & 0xFFU);
    field[2] = static_cast<std::uint8_t>((((value >> 15U) & 0x7FU) << 1U) | 1U);
    field[3] = static_cast<std::uint8_t>((value >> 7U) & 0xFFU);
    field[4] = static_cast<std::uint8_t>(((value & 0x7FU) << 1U) | 1U);
}

Packet pes_packet(const std::uint16_t pid, const std::uint64_t pts,
                  const bool with_dts) {
    Packet packet{};
    packet.fill(0xFF);
    packet[0] = 0x47;
    packet[1] = static_cast<std::uint8_t>(0x40U | (pid >> 8U));
    packet[2] = static_cast<std::uint8_t>(pid & 0xFFU);
    packet[3] = 0x10;
    std::uint8_t *pes = packet.data() + 4;
    pes[0] = 0;
    pes[1] = 0;
    pes[2] = 1;
    pes[3] = 0xE0;
    pes[4] = 0;
    pes[5] = 0;
    pes[6] = 0x80;
    pes[7] = with_dts ? 0xC0 : 0x80;
    pes[8] = with_dts ? 10 : 5;
    write_timestamp(pes + 9, with_dts ? 3U : 2U, pts);
    if (with_dts) {
        write_timestamp(pes + 14, 1U, pts - 3'000);
    }
    return packet;
}

// One PCR packet followed by `fill` null packets every 40 ms of PCR time;
// arrival advances 40 ms / (1 + offset_ppm * 1e-6).
void feed(TransportClockMonitor &monitor, std::uint64_t &pcr,
          Clock::time_point &arrival, const unsigned int count,
          const unsigned int fill, const double offset_ppm) {
    const std::uint64_t pcr_step = ticks_per_second * 40 / 1000;
    const auto arrival_step = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(0.040 / (1.0 + (offset_ppm * 1e-6))));
    for (unsigned int index = 0; index < count; ++index) {
        std::vector<std::uint8_t> block;
        const auto head = pcr_packet(0x100, pcr, false);
        block.insert(block.end(), head.begin(), head.end());
        for (unsigned int extra = 0; extra < fill; ++extra) {
            const auto filler = null_packet();
            block.insert(block.end(), filler.begin(), filler.end());
        }
        monitor.consume(block, arrival);
        pcr = (pcr + pcr_step) % (pcr_base_modulus * 300);
        arrival += arrival_step;
    }
}

bool test_rate_and_clock_offset() {
    TransportClockMonitor monitor;
    std::uint64_t pcr = 1'000'000;
    auto arrival = Clock::time_point{} + std::chrono::seconds(10);
    feed(monitor, pcr, arrival, 800, 9, 120.0);
    const auto snapshot = monitor.snapshot();
    const double expected_bitrate = 10.0 * 188.0 * 8.0 / 0.040;
    return require(snapshot.packets == 8000 && snapshot.pcr_count == 800 &&
                       snapshot.pcr_pids == 1,
                   "packet and PCR counts") &&
           require(snapshot.pcr_unflagged_discontinuities == 0 &&
                       snapshot.pcr_flagged_discontinuities == 0,
                   "steady PCR has no seams") &&
           require(std::abs(snapshot.pcr_interval_max_ms - 40.0) < 1e-6,
                   "PCR interval") &&
           require(snapshot.bitrate_bps.has_value() &&
                       std::abs(*snapshot.bitrate_bps - expected_bitrate) <
                           1e-3,
                   "bit rate from bytes per PCR second") &&
           require(snapshot.clock_offset_ppm.has_value() &&
                       std::abs(*snapshot.clock_offset_ppm - 120.0) < 1.0,
                   "clock offset in ppm over the anchored span") &&
           require(snapshot.clock_span_s > 19.0 && snapshot.clock_span_s < 31.0,
                   "clock span covers the closed windows");
}

bool test_catch_up_burst_is_not_a_clock_offset() {
    TransportClockMonitor monitor;
    std::uint64_t pcr = 5'000'000;
    auto arrival = Clock::time_point{} + std::chrono::seconds(10);
    feed(monitor, pcr, arrival, 375, 3, 0.0);
    // Three seconds of stream delivered within 100 ms: a backlog catch-up.
    const auto before_burst = arrival;
    feed(monitor, pcr, arrival, 75, 3, 0.0);
    arrival = before_burst + std::chrono::milliseconds(100);
    feed(monitor, pcr, arrival, 1000, 3, 0.0);
    const auto snapshot = monitor.snapshot();
    return require(snapshot.pcr_unflagged_discontinuities == 0,
                   "a burst is not a PCR seam") &&
           require(snapshot.clock_offset_ppm.has_value() &&
                       std::abs(*snapshot.clock_offset_ppm) < 5.0,
                   "clock offset measured after the catch-up burst");
}

bool test_wrap_and_seams() {
    TransportClockMonitor monitor;
    std::uint64_t pcr = (pcr_base_modulus - 30) * 300;
    auto arrival = Clock::time_point{} + std::chrono::seconds(10);
    feed(monitor, pcr, arrival, 750, 4, 0.0);
    auto snapshot = monitor.snapshot();
    if (!require(snapshot.pcr_unflagged_discontinuities == 0 &&
                     snapshot.bitrate_bps.has_value() &&
                     snapshot.clock_offset_ppm.has_value(),
                 "PCR base wrap is not a seam")) {
        return false;
    }
    const double wrapped_span = snapshot.clock_span_s;

    // An unannounced jump of 2 s re-anchors; a flagged jump is counted on
    // its own and re-anchors as well.
    pcr = (pcr + (2 * ticks_per_second)) % (pcr_base_modulus * 300);
    feed(monitor, pcr, arrival, 1, 4, 0.0);
    snapshot = monitor.snapshot();
    if (!require(snapshot.pcr_unflagged_discontinuities == 1 &&
                     !snapshot.clock_offset_ppm.has_value() &&
                     snapshot.clock_span_s < wrapped_span,
                 "unflagged PCR jump re-anchors the clock")) {
        return false;
    }
    const auto flagged = pcr_packet(0x100, pcr + (5 * ticks_per_second), true);
    monitor.consume(flagged, arrival);
    snapshot = monitor.snapshot();
    if (!require(snapshot.pcr_flagged_discontinuities == 1 &&
                     snapshot.pcr_unflagged_discontinuities == 1,
                 "flagged PCR discontinuity is counted separately")) {
        return false;
    }
    monitor.reset();
    snapshot = monitor.snapshot();
    return require(snapshot.packets == 0 && snapshot.pcr_count == 0 &&
                       snapshot.pcr_pids == 0,
                   "reset forgets the stream");
}

bool test_pts_dts_and_corruption() {
    TransportClockMonitor monitor;
    const auto arrival = Clock::time_point{} + std::chrono::seconds(1);
    std::vector<std::uint8_t> block;
    const auto video = pes_packet(0x101, 90'000, true);
    const auto audio = pes_packet(0x102, 90'000, false);
    auto corrupt = pes_packet(0x103, 90'000, false);
    corrupt[1] |= 0x80U; // transport_error_indicator
    auto bad_marker = pes_packet(0x104, 90'000, false);
    bad_marker[4 + 9] &= 0xFEU;
    for (const auto &packet : {video, audio, corrupt, bad_marker}) {
        block.insert(block.end(), packet.begin(), packet.end());
    }
    monitor.consume(block, arrival);
    const auto snapshot = monitor.snapshot();
    return require(snapshot.packets == 3, "errored packets are skipped") &&
           require(snapshot.pts_count == 2 && snapshot.dts_count == 1,
                   "PTS and DTS counted at PES starts with valid markers") &&
           require(snapshot.pcr_count == 0 && !snapshot.bitrate_bps,
                   "no PCR leaves the rate unknown");
}

} // namespace

int main() {
    return test_rate_and_clock_offset() &&
                   test_catch_up_burst_is_not_a_clock_offset() &&
                   test_wrap_and_seams() && test_pts_dts_and_corruption()
               ? 0
               : 1;
}
