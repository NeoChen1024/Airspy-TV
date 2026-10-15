#include "airspy_tv/transport_clock_monitor.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>

namespace airspy_tv {
namespace {

constexpr std::size_t transport_packet_size = 188;
constexpr double pcr_ticks_per_second = 27'000'000.0;
constexpr std::uint64_t pcr_modulus = (std::uint64_t{1} << 33U) * 300U;
// A forward PCR step beyond this without a discontinuity indicator is a
// stream seam (the spec bounds PCR spacing at 100 ms).
constexpr double unflagged_jump_seconds = 1.0;
// The arrival of a packet lags its PCR by the pipeline's queueing delay,
// which varies by batching and jumps when the decoder catches up after
// acquisition or a stall. The clock offset is therefore measured between the
// minimum-delay samples of fixed arrival windows (the samples where the
// pipeline was emptiest), and the anchor moves forward when a later window
// is earlier than any clock mismatch could explain.
constexpr double clock_window_seconds = 10.0;
constexpr double catch_up_tolerance_seconds = 0.05;
constexpr double catch_up_tolerance_ratio = 1e-3;
// A corrupt or hostile stream must not grow the per-PID table without bound.
constexpr std::size_t max_pcr_pids = 256;

[[nodiscard]] std::uint16_t packet_pid(const std::uint8_t *packet) noexcept {
    return static_cast<std::uint16_t>(((packet[1] & 0x1FU) << 8U) | packet[2]);
}

[[nodiscard]] std::uint64_t read_pcr(const std::uint8_t *field) noexcept {
    const std::uint64_t base = (static_cast<std::uint64_t>(field[0]) << 25U) |
                               (static_cast<std::uint64_t>(field[1]) << 17U) |
                               (static_cast<std::uint64_t>(field[2]) << 9U) |
                               (static_cast<std::uint64_t>(field[3]) << 1U) |
                               (static_cast<std::uint64_t>(field[4]) >> 7U);
    const std::uint64_t extension =
        ((static_cast<std::uint64_t>(field[4]) & 1U) << 8U) | field[5];
    return (base * 300U) + extension;
}

// The 33-bit timestamp layout shared by PTS and DTS: marker bits interleave
// the value in five bytes.
[[nodiscard]] bool valid_timestamp(const std::uint8_t *field) noexcept {
    return (field[0] & 1U) != 0U && (field[2] & 1U) != 0U &&
           (field[4] & 1U) != 0U;
}

} // namespace

struct TransportClockMonitor::Impl {
    struct PidState {
        std::uint64_t last_pcr{};
        bool have_pcr{};
    };

    // One PCR of the reference PID: broadcast seconds and arrival seconds
    // since the origin, and the stream position.
    struct Sample {
        double pcr_s{};
        double arrival_s{};
        std::uint64_t bytes{};
        [[nodiscard]] double delay() const noexcept {
            return arrival_s - pcr_s;
        }
    };

    mutable std::mutex mutex;
    std::map<std::uint16_t, PidState> pids;
    TransportClockSnapshot totals;
    std::uint64_t stream_bytes{};

    std::optional<std::uint16_t> reference_pid;
    bool anchored{};
    // Origin of the unwrapped PCR and arrival axes (the first reference PCR
    // after a seam).
    Clock::time_point origin_arrival;
    std::uint64_t latest_pcr{};
    std::uint64_t latest_pcr_offset{};
    // Current arrival window and its minimum-delay sample.
    double window_start_s{};
    std::optional<Sample> window_best;
    // Minimum-delay samples of the anchor window and the latest closed one.
    std::optional<Sample> clock_anchor;
    std::optional<Sample> clock_latest;

    void anchor(const std::uint64_t pcr, const Clock::time_point arrival) {
        origin_arrival = arrival;
        latest_pcr = pcr;
        latest_pcr_offset = 0;
        window_start_s = 0.0;
        window_best.reset();
        clock_anchor.reset();
        clock_latest.reset();
        anchored = true;
        totals.bitrate_bps.reset();
        totals.clock_offset_ppm.reset();
        totals.clock_span_s = 0.0;
        observe_reference(
            Sample{.pcr_s = 0.0, .arrival_s = 0.0, .bytes = stream_bytes});
    }

    void close_window(const Sample &best) {
        if (!clock_anchor.has_value()) {
            clock_anchor = best;
            return;
        }
        const double drop = best.delay() - clock_anchor->delay();
        const double elapsed = best.arrival_s - clock_anchor->arrival_s;
        if (drop < -(catch_up_tolerance_seconds +
                     (catch_up_tolerance_ratio * elapsed))) {
            // The pipeline delivered earlier than any clock mismatch could
            // make it: it caught up a backlog. Measure from here on.
            clock_anchor = best;
            clock_latest.reset();
            totals.clock_offset_ppm.reset();
            totals.clock_span_s = 0.0;
            return;
        }
        clock_latest = best;
        totals.clock_span_s = elapsed;
        if (elapsed > 0.0) {
            totals.clock_offset_ppm =
                (((best.pcr_s - clock_anchor->pcr_s) / elapsed) - 1.0) * 1e6;
        }
    }

    void observe_reference(const Sample &sample) {
        if (window_best.has_value() &&
            sample.arrival_s - window_start_s >= clock_window_seconds) {
            close_window(*window_best);
            window_start_s = sample.arrival_s;
            window_best.reset();
        }
        if (!window_best.has_value() || sample.delay() < window_best->delay()) {
            window_best = sample;
        }
        if (clock_anchor.has_value() && sample.pcr_s > clock_anchor->pcr_s) {
            totals.bitrate_bps =
                static_cast<double>(sample.bytes - clock_anchor->bytes) * 8.0 /
                (sample.pcr_s - clock_anchor->pcr_s);
        }
    }

    void observe_pcr(const std::uint16_t pid, const std::uint64_t pcr,
                     const bool flagged, const Clock::time_point arrival) {
        ++totals.pcr_count;
        auto found = pids.find(pid);
        if (found == pids.end()) {
            if (pids.size() >= max_pcr_pids) {
                return;
            }
            found = pids.try_emplace(pid).first;
            ++totals.pcr_pids;
        }
        auto &state = found->second;
        bool seam = flagged;
        if (flagged) {
            ++totals.pcr_flagged_discontinuities;
        } else if (state.have_pcr) {
            const std::uint64_t forward =
                (pcr + pcr_modulus - state.last_pcr) % pcr_modulus;
            const double seconds =
                static_cast<double>(forward) / pcr_ticks_per_second;
            // A backward step shows up as a forward distance close to the
            // modulus (about 26.5 hours); treat anything beyond the bound as
            // a seam either way.
            if (seconds > unflagged_jump_seconds) {
                ++totals.pcr_unflagged_discontinuities;
                seam = true;
            } else {
                totals.pcr_interval_max_ms =
                    std::max(totals.pcr_interval_max_ms, seconds * 1e3);
            }
        }
        state.last_pcr = pcr;
        state.have_pcr = true;

        if (!reference_pid.has_value()) {
            reference_pid = pid;
        }
        if (pid != *reference_pid) {
            return;
        }
        if (!anchored || seam) {
            anchor(pcr, arrival);
            return;
        }
        latest_pcr_offset += (pcr + pcr_modulus - latest_pcr) % pcr_modulus;
        latest_pcr = pcr;
        observe_reference(Sample{
            .pcr_s =
                static_cast<double>(latest_pcr_offset) / pcr_ticks_per_second,
            .arrival_s =
                std::chrono::duration<double>(arrival - origin_arrival).count(),
            .bytes = stream_bytes});
    }

    void observe_packet(const std::uint8_t *packet,
                        const Clock::time_point arrival) {
        ++totals.packets;
        const bool payload_start = (packet[1] & 0x40U) != 0U;
        const unsigned int adaptation =
            (static_cast<unsigned int>(packet[3]) >> 4U) & 0x3U;
        std::size_t payload_offset = 4;
        if ((adaptation & 2U) != 0U) {
            const std::size_t length = packet[4];
            payload_offset = 5 + length;
            if (length >= 7 && (packet[5] & 0x10U) != 0U) {
                observe_pcr(packet_pid(packet), read_pcr(packet + 6),
                            (packet[5] & 0x80U) != 0U, arrival);
            }
        }
        // Count the stream bytes after the PCR so the anchor's byte position
        // and PCR refer to the same packet.
        stream_bytes += transport_packet_size;
        if ((adaptation & 1U) == 0U || !payload_start ||
            payload_offset + 14 > transport_packet_size) {
            return;
        }
        const std::uint8_t *pes = packet + payload_offset;
        if (pes[0] != 0U || pes[1] != 0U || pes[2] != 1U ||
            (pes[6] & 0xC0U) != 0x80U) {
            return;
        }
        const unsigned int flags =
            (static_cast<unsigned int>(pes[7]) >> 6U) & 0x3U;
        if (flags == 0U || !valid_timestamp(pes + 9)) {
            return;
        }
        ++totals.pts_count;
        if (flags == 3U && payload_offset + 19 <= transport_packet_size &&
            valid_timestamp(pes + 14)) {
            ++totals.dts_count;
        }
    }
};

TransportClockMonitor::TransportClockMonitor()
    : impl_(std::make_unique<Impl>()) {}

TransportClockMonitor::~TransportClockMonitor() noexcept = default;

void TransportClockMonitor::consume(
    const std::span<const std::uint8_t> transport_stream,
    const Clock::time_point arrival) {
    const std::scoped_lock lock(impl_->mutex);
    for (std::size_t offset = 0;
         offset + transport_packet_size <= transport_stream.size();
         offset += transport_packet_size) {
        const std::uint8_t *packet = transport_stream.data() + offset;
        if (packet[0] == 0x47U && (packet[1] & 0x80U) == 0U) {
            impl_->observe_packet(packet, arrival);
        }
    }
}

void TransportClockMonitor::reset() noexcept {
    const std::scoped_lock lock(impl_->mutex);
    impl_->pids.clear();
    impl_->totals = {};
    impl_->stream_bytes = 0;
    impl_->reference_pid.reset();
    impl_->anchored = false;
}

TransportClockSnapshot TransportClockMonitor::snapshot() const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->totals;
}

} // namespace airspy_tv
