#pragma once

#include "airspy_tv/demodulator.hpp"

#include <cstdint>
#include <ostream>
#include <string_view>

namespace airspy_tv {

// decoder_status is the active standard's own lock and quality text.
void format_decode_progress(std::ostream &stream,
                            const PipelineSnapshot &pipeline,
                            std::string_view decoder_status,
                            std::uint64_t submitted_samples,
                            std::uint32_t sample_rate_hz, double wall_seconds);

} // namespace airspy_tv
