#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"

#include <cstdint>
#include <ostream>

namespace airspy_tv {

void format_decode_progress(std::ostream &stream,
                            const dvbt::StreamDecoderStats &stats,
                            std::uint64_t submitted_samples,
                            std::uint32_t sample_rate_hz, double wall_seconds);

} // namespace airspy_tv
