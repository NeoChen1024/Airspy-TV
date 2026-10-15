#pragma once

#include <complex>
#include <cstdint>
#include <span>

namespace airspy_tv::dvbt {

struct ChannelLocation {
    // Centre of the located channel relative to the source centre, in hertz.
    // Zero when no channel was located.
    double offset_hz{};
    // In-channel level above the level just outside both channel edges.
    float edge_contrast_db{};
    bool found{};
};

// Finds where a DVB-T channel of the configured bandwidth sits inside the
// source band from the averaged power spectrum: the window of the channel's
// occupied width holding the most power, accepted only when both of its edges
// stand clearly above the band beside them and lie inside the source band.
// The centre is then read from the half-power crossings at the two edges so a
// tilted channel does not pull the estimate towards its stronger side.
// Recordings made beside the channel (gqrx, SDR#) and radios tuned far off
// need this; a centred channel reports a small offset the acquisition
// integer-bin search already covers.
[[nodiscard]] ChannelLocation
locate_channel(std::span<const std::complex<float>> samples,
               std::uint32_t sample_rate_hz,
               std::uint32_t channel_bandwidth_hz);

// A located offset inside the acquisition integer-bin search (48 carriers)
// leaves the stream untouched, so a centred source keeps its unchanged path.
[[nodiscard]] inline double
coarse_offset_threshold_hz(const std::uint32_t bandwidth_hz) noexcept {
    constexpr double integer_search_bins = 48.0;
    return integer_search_bins * bandwidth_hz * 8.0 / (7.0 * 8192.0);
}

// The mirrored scattered-pilot pattern must beat the upright one by this
// factor before a source is treated as having I and Q swapped.
inline constexpr float mirror_score_ratio = 1.5F;

} // namespace airspy_tv::dvbt
