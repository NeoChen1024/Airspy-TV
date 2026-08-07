#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace solid_resampler {

struct ResamplerConfig {
    double input_rate_hz{};
    double output_rate_hz{};
    double passband_edge_hz{};
    double stopband_edge_hz{};
    float stopband_attenuation_db{80.0F};
    unsigned int polyphase_filters{1024};
};

// Streaming arbitrary-rate complex resampler with optional frequency
// translation. process() has one producer; control setters may be called
// concurrently and are sampled at a block boundary.
class FrequencyTranslatingResampler {
  public:
    explicit FrequencyTranslatingResampler(
        std::size_t worker_count = 1, std::string worker_name = "resample");
    ~FrequencyTranslatingResampler() noexcept;

    FrequencyTranslatingResampler(const FrequencyTranslatingResampler &) =
        delete;
    FrequencyTranslatingResampler &
    operator=(const FrequencyTranslatingResampler &) = delete;
    FrequencyTranslatingResampler(FrequencyTranslatingResampler &&) = delete;
    FrequencyTranslatingResampler &
    operator=(FrequencyTranslatingResampler &&) = delete;

    void configure(const ResamplerConfig &config);
    void reset() noexcept;

    // Ratio is output samples per input sample. Updating it preserves stream
    // phase and FIR history. With no slew limit, the new Q32.32 step applies
    // on the next block.
    void set_ratio(double output_per_input);
    // Limit block-boundary phase-step changes in ppm of the nominal step per
    // second of input. Zero (the default) applies ratio changes immediately.
    void set_max_slew_rate(double ppm_per_second);
    // Translate the complex output by the requested signed frequency. The
    // oscillator phase remains continuous across updates and process() calls.
    // Zero disables the mixer without changing the resampler output.
    void set_frequency_shift(double frequency_hz);

    [[nodiscard]] std::span<const std::complex<float>>
    process(std::span<const std::complex<float>> input);

    [[nodiscard]] bool configured() const noexcept;
    [[nodiscard]] std::size_t worker_count() const noexcept;
    [[nodiscard]] double nominal_ratio() const noexcept;
    [[nodiscard]] double requested_ratio() const noexcept;
    [[nodiscard]] double effective_ratio() const noexcept;
    [[nodiscard]] double max_slew_rate() const noexcept;
    [[nodiscard]] double requested_frequency_shift() const noexcept;
    [[nodiscard]] double effective_frequency_shift() const noexcept;
    [[nodiscard]] std::uint64_t phase_step_q32() const noexcept;
    [[nodiscard]] std::size_t filter_taps() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace solid_resampler
