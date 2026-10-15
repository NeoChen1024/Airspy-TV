#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace airspy_tv {

// Timing facts of a transport stream as it passes a point in the pipeline:
// PCR/PTS/DTS activity, the stream's own bit rate and how the broadcast clock
// runs against the clock that timestamps the arrivals.
struct TransportClockSnapshot {
    std::uint64_t packets{};
    std::uint64_t pcr_count{};
    std::uint64_t pts_count{};
    std::uint64_t dts_count{};
    // PIDs that have carried a PCR since the last reset.
    std::uint32_t pcr_pids{};
    // PCR jumps announced by the adaptation-field discontinuity indicator and
    // jumps that were not announced (a backward step, or more than a second
    // forward between consecutive PCRs of one PID).
    std::uint64_t pcr_flagged_discontinuities{};
    std::uint64_t pcr_unflagged_discontinuities{};
    // Longest gap between consecutive PCRs of one PID (EN 300 468 and
    // ISO/IEC 13818-1 bound it at 100 ms and 40 ms respectively).
    double pcr_interval_max_ms{};
    // Stream bytes per broadcast second, measured between the clock anchor
    // and the latest PCR of the reference PID. For a filtered stream it is
    // the rate of the packets that were kept.
    std::optional<double> bitrate_bps;
    // Broadcast clock against the arrival clock: positive when the stream's
    // PCR advances faster than the arrival timestamps, in parts per million.
    // Measured between the minimum-delay samples of 10 s arrival windows so
    // pipeline batching and catch-up bursts do not bias it; available after
    // two windows. Meaningful only when the arrivals are paced by the
    // broadcast (live reception); for offline decoding it reports the
    // processing speed.
    std::optional<double> clock_offset_ppm;
    // Arrival time the clock offset was measured over.
    double clock_span_s{};
};

// Parses PCR, PTS and DTS from 188-byte packets without buffering them; a
// cheap inline observer for the transport stream path. Not thread-safe
// between consume() calls; snapshot() may be taken from another thread.
class TransportClockMonitor {
  public:
    using Clock = std::chrono::steady_clock;

    TransportClockMonitor();
    ~TransportClockMonitor() noexcept;

    TransportClockMonitor(const TransportClockMonitor &) = delete;
    TransportClockMonitor &operator=(const TransportClockMonitor &) = delete;
    TransportClockMonitor(TransportClockMonitor &&) = delete;
    TransportClockMonitor &operator=(TransportClockMonitor &&) = delete;

    // `arrival` timestamps every packet of the block.
    void consume(std::span<const std::uint8_t> transport_stream,
                 Clock::time_point arrival = Clock::now());
    // Forget every PID and the clock anchor (retune, new stream).
    void reset() noexcept;

    [[nodiscard]] TransportClockSnapshot snapshot() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
