#pragma once

#include <complex>
#include <cstddef>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

enum class Constellation {
    qpsk,
    qam16,
    qam64,
};

[[nodiscard]] std::size_t bits_per_symbol(Constellation constellation);

class MaxLogDemapper {
  public:
    explicit MaxLogDemapper(Constellation constellation);

    [[nodiscard]] Constellation constellation() const noexcept;
    [[nodiscard]] std::size_t bits_per_symbol() const noexcept;
    [[nodiscard]] std::span<const std::complex<float>>
    constellation_points() const noexcept;

    // Output is sample-major and MSB-first. A positive LLR favours bit 1.
    // reliability is the effective post-equalization inverse noise variance,
    // |H|^2 / sigma^2. A value of zero produces neutral LLRs for that carrier.
    void demap(std::span<const std::complex<float>> equalized_carriers,
               std::span<const float> reliability,
               std::span<float> output_llrs) const;

  private:
    Constellation constellation_;
    std::size_t bits_per_symbol_;
    std::vector<std::complex<float>> points_;
};

} // namespace airspy_tv::dvbt
