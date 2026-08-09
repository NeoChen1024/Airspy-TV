#pragma once

#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/soft_demapper.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace airspy_tv::dvbt {

enum class GuardInterval { gi_1_32, gi_1_16, gi_1_8, gi_1_4 };

struct ReceiverParameters {
    std::uint32_t channel_bandwidth_hz{6'000'000};
    std::optional<TransmissionMode> mode;
    std::optional<GuardInterval> guard_interval;
    std::optional<Constellation> constellation;
    std::optional<CodeRate> code_rate;
    // Zero chooses the logical CPU count. The budget is divided between the
    // symbol postprocessor and Viterbi pools and is fixed before opening a
    // source because changing it reconstructs both pools.
    std::size_t worker_threads{};
    // Capacity multiplier for the demod symbol-worker and FEC queues. Live and
    // GUI receivers leave this at one; decoder-paced offline input may
    // increase it to absorb stage-to-stage bursts.
    std::size_t queue_capacity_multiplier{1};
};

} // namespace airspy_tv::dvbt
