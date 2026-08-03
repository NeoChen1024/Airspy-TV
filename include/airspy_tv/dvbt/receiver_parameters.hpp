#pragma once

#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/soft_demapper.hpp"

#include <cstddef>
#include <optional>

namespace airspy_tv::dvbt {

enum class GuardInterval { gi_1_32, gi_1_16, gi_1_8, gi_1_4 };

struct ReceiverParameters {
    std::optional<TransmissionMode> mode;
    std::optional<GuardInterval> guard_interval;
    std::optional<Constellation> constellation;
    std::optional<CodeRate> code_rate;
    // Zero chooses the logical CPU count. The budget is divided between the
    // symbol postprocessor and Viterbi pools and is fixed before opening a
    // source because changing it reconstructs both pools.
    std::size_t worker_threads{};
};

} // namespace airspy_tv::dvbt
