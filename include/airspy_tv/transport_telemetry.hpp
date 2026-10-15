#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace airspy_tv {

// A sink that paces its own consumption (playback) and steers that pace from
// its queue level.
struct TransportOutputPacing {
    double rate{1.0};
    std::optional<double> queued_seconds;
    std::uint64_t stalls{};
    std::optional<double> clock_offset_ppm;
};

struct TransportOutputTelemetry {
    std::string name;
    std::string type;
    bool active{};
    bool required{};
    bool failed{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_processed{};
    std::uint64_t bytes_processed{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::uint64_t errors{};
    std::size_t queued_bytes{};
    std::size_t queue_capacity_bytes{};
    std::string error;
    std::optional<TransportOutputPacing> pacing;
};

} // namespace airspy_tv
