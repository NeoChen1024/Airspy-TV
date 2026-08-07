#pragma once

#include "solid_resampler/frequency_translating_resampler.hpp"

#include <algorithm>
#include <cstdint>

namespace airspy_tv::dvbt {

[[nodiscard]] inline solid_resampler::ResamplerConfig
make_resampler_config(const std::uint32_t input_rate_hz,
                      const std::uint32_t channel_bandwidth_hz) {
    constexpr double active_carrier_edge_fraction = 3408.0 / 8192.0;
    const double output_rate_hz =
        static_cast<double>(channel_bandwidth_hz) * 8.0 / 7.0;
    return {
        .input_rate_hz = static_cast<double>(input_rate_hz),
        .output_rate_hz = output_rate_hz,
        .passband_edge_hz = output_rate_hz * active_carrier_edge_fraction,
        .stopband_edge_hz =
            0.5 * std::min(static_cast<double>(input_rate_hz), output_rate_hz),
        .stopband_attenuation_db = 80.0F,
    };
}

} // namespace airspy_tv::dvbt
