#pragma once

#include "airspy_tv/dvbt/receiver_parameters.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

struct OfdmAcquisition {
    std::size_t start{};
    std::size_t fft_size{};
    std::size_t guard_size{};
    std::complex<float> phase{};
    float score{};
    TransmissionMode mode{TransmissionMode::k8};
    GuardInterval guard{GuardInterval::gi_1_4};
};

// Common CS16 frontend used by both the interactive monitor and the complete
// streaming decoder. DVB-T's nominal baseband rate is bandwidth * 8 / 7.
[[nodiscard]] std::vector<std::complex<float>>
resample_cs16(std::span<const std::int16_t> interleaved_iq,
              std::uint32_t sample_rate_hz, std::uint32_t channel_bandwidth_hz,
              std::size_t worker_count = 1);

// Searches CP periodicity across all allowed (or explicitly selected) DVB-T
// transmission modes and guard intervals.
[[nodiscard]] OfdmAcquisition
acquire_ofdm(std::span<const std::complex<float>> samples,
             const ReceiverParameters &parameters);

} // namespace airspy_tv::dvbt
