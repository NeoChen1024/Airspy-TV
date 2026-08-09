#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace airspy_tv {

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
};

} // namespace airspy_tv
