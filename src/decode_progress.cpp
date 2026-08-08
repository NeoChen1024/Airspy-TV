#include "decode_progress.hpp"

#include <algorithm>
#include <format>
#include <iomanip>

namespace airspy_tv {
namespace {

[[nodiscard]] int percent(const std::uint64_t used,
                          const std::uint64_t capacity) {
    if (capacity == 0) {
        return 0;
    }
    return static_cast<int>(std::clamp(100.0 * static_cast<double>(used) /
                                           static_cast<double>(capacity),
                                       0.0, 100.0));
}

} // namespace

void format_decode_progress(std::ostream &stream,
                            const dvbt::StreamDecoderStats &stats,
                            const std::uint64_t submitted_samples,
                            const std::uint32_t sample_rate_hz,
                            const double wall_seconds) {
    const double input_seconds = sample_rate_hz == 0
                                     ? 0.0
                                     : static_cast<double>(submitted_samples) /
                                           static_cast<double>(sample_rate_hz);
    const double speed =
        wall_seconds > 0.0 ? input_seconds / wall_seconds : 0.0;
    const int iq =
        percent(stats.queued_input_samples, stats.input_queue_capacity_samples);
    const int demod = static_cast<int>(
        std::clamp(100.0F * stats.demod_busy_fraction, 0.0F, 100.0F));
    const int fec = percent(stats.queued_symbols, stats.symbol_queue_capacity);
    stream << std::fixed << std::setprecision(1) << "wall=" << wall_seconds
           << "s input=" << input_seconds << "s speed=" << speed << "x MER=";
    if (stats.ofdm_locked) {
        stream << stats.mer_db << "dB";
    } else {
        stream << "--";
    }
    stream << " OFDM=" << (stats.ofdm_locked ? "lock" : "search")
           << " TPS=" << (stats.tps_locked ? "lock" : "search")
           << " TS=" << std::setprecision(1)
           << static_cast<double>(stats.transport_bytes) / (1024.0 * 1024.0)
           << "MiB TEI=" << stats.cumulative_transport.tei_packets
           << std::format(" IQ={:3d}% Demod={:3d}% FEC={:3d}%", iq, demod, fec)
           << '\n';
}

} // namespace airspy_tv
