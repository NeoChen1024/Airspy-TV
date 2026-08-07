#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace liquid_resampler {

struct ResamplerConfig {
    double input_rate_hz{};
    double output_rate_hz{};
    double passband_edge_hz{};
    double stopband_edge_hz{};
    float stopband_attenuation_db{80.0F};
    unsigned int polyphase_filters{1024};
};

// Streaming arbitrary-rate complex resampler. process() has one producer;
// set_ratio() may be called concurrently and is sampled at a block boundary.
class ArbitraryResampler {
  public:
    explicit ArbitraryResampler(std::size_t worker_count = 1,
                                std::string worker_name = "resample");
    ~ArbitraryResampler() noexcept;

    ArbitraryResampler(const ArbitraryResampler &) = delete;
    ArbitraryResampler &operator=(const ArbitraryResampler &) = delete;
    ArbitraryResampler(ArbitraryResampler &&) = delete;
    ArbitraryResampler &operator=(ArbitraryResampler &&) = delete;

    void configure(const ResamplerConfig &config);
    void reset() noexcept;

    // Ratio is output samples per input sample. Updating it preserves stream
    // phase and FIR history. With no slew limit, the new Q32.32 step applies
    // on the next block.
    void set_ratio(double output_per_input);
    // Limit block-boundary phase-step changes in ppm of the nominal step per
    // second of input. Zero (the default) applies ratio changes immediately.
    void set_max_slew_rate(double ppm_per_second);

    [[nodiscard]] std::span<const std::complex<float>>
    process(std::span<const std::complex<float>> input);

    [[nodiscard]] bool configured() const noexcept;
    [[nodiscard]] std::size_t worker_count() const noexcept;
    [[nodiscard]] double nominal_ratio() const noexcept;
    [[nodiscard]] double requested_ratio() const noexcept;
    [[nodiscard]] double effective_ratio() const noexcept;
    [[nodiscard]] double max_slew_rate() const noexcept;
    [[nodiscard]] std::uint64_t phase_step_q32() const noexcept;
    [[nodiscard]] std::size_t filter_taps() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace liquid_resampler
