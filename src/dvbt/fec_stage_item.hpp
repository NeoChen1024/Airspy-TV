#pragma once

#include "airspy_tv/dvbt/decoder.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace airspy_tv::dvbt {

// Ordered events crossing from symbol postprocessing into serial FEC output.
struct FecItem {
    enum class Kind { begin, symbol, end, stream_end, stats };

    Kind kind{Kind::symbol};
    std::uint64_t generation{};
    DecoderParameters parameters{};
    std::vector<std::uint8_t> mother_metrics;
    std::size_t symbol_index{};
    std::uint64_t demod_window_sequence{};
    std::uint64_t source_epoch{};
};

} // namespace airspy_tv::dvbt
