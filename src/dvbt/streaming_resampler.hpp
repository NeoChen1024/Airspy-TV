#pragma once

#include "solid_resampler/frequency_translating_resampler.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>

namespace airspy_tv::dvbt {

class StreamingResampler {
  public:
    explicit StreamingResampler(std::size_t worker_count);

    [[nodiscard]] std::size_t worker_count() const noexcept;
    void reset();
    void configure(std::uint32_t rate, std::uint32_t bandwidth);
    [[nodiscard]] std::span<const std::complex<float>>
    process(std::span<const std::complex<float>> input);
    void set_sro_correction_ppm(double correction_ppm);
    void set_cfo_correction_hz(double correction_hz);
    [[nodiscard]] double applied_sro_correction_ppm() const noexcept;
    [[nodiscard]] double applied_cfo_correction_hz() const noexcept;
    [[nodiscard]] double requested_cfo_correction_hz() const noexcept;
    [[nodiscard]] double requested_ratio() const noexcept;
    [[nodiscard]] double effective_ratio() const noexcept;
    [[nodiscard]] bool configured() const noexcept;
    [[nodiscard]] std::uint32_t rate() const noexcept;
    [[nodiscard]] std::uint32_t bandwidth() const noexcept;

  private:
    // Reach the documented +/-20 ppm receiver sanity bound before an 8K
    // acquisition can walk out of its timing ambiguity, while still slewing
    // over multiple input blocks rather than stepping the sample timeline.
    static constexpr double sro_slew_rate_ppm_per_second = 20.0;
    solid_resampler::FrequencyTranslatingResampler resampler_;
    std::uint32_t rate_{};
    std::uint32_t bandwidth_{};
};

} // namespace airspy_tv::dvbt
