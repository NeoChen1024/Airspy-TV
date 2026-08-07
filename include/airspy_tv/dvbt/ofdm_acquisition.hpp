#pragma once

#include "airspy_tv/dvbt/receiver_parameters.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

struct OfdmAcquisition {
    std::size_t start{};
    std::size_t fft_size{};
    std::size_t guard_size{};
    std::complex<float> phase{};
    float fractional_cfo_phase_per_sample{};
    float total_cfo_phase_per_sample{};
    int carrier_offset{};
    int pilot_phase{};
    float score{};
    TransmissionMode mode{TransmissionMode::k8};
    GuardInterval guard{GuardInterval::gi_1_4};
};

// Reusable partitioned CS16 resampler. Worker threads and filter state remain
// alive across process() calls; process() itself is synchronous and must only
// be called by one producer at a time.
class Cs16Resampler {
  public:
    explicit Cs16Resampler(std::size_t worker_count = 1);
    ~Cs16Resampler() noexcept;
    Cs16Resampler(const Cs16Resampler &) = delete;
    Cs16Resampler &operator=(const Cs16Resampler &) = delete;
    Cs16Resampler(Cs16Resampler &&) = delete;
    Cs16Resampler &operator=(Cs16Resampler &&) = delete;

    [[nodiscard]] std::vector<std::complex<float>>
    process(std::span<const std::int16_t> interleaved_iq,
            std::uint32_t sample_rate_hz, std::uint32_t channel_bandwidth_hz);
    [[nodiscard]] std::size_t worker_count() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Common CS16 frontend used by both the interactive monitor and the complete
// streaming decoder. DVB-T's nominal baseband rate is bandwidth * 8 / 7.
[[nodiscard]] std::vector<std::complex<float>>
resample_cs16(std::span<const std::int16_t> interleaved_iq,
              std::uint32_t sample_rate_hz, std::uint32_t channel_bandwidth_hz,
              std::size_t requested_workers = 1);

// Searches CP periodicity across all allowed (or explicitly selected) DVB-T
// transmission modes and guard intervals.
[[nodiscard]] OfdmAcquisition
acquire_ofdm(std::span<const std::complex<float>> samples,
             const ReceiverParameters &parameters,
             bool search_carrier_offset = true);

} // namespace airspy_tv::dvbt
