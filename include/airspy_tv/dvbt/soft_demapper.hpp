#pragma once

#include <array>
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
    [[nodiscard]] std::complex<float>
    nearest_constellation_point(std::complex<float> value) const noexcept;
    void slice_nearest(std::span<const std::complex<float>> input,
                       std::span<std::complex<float>> output) const;

    // Output is sample-major and MSB-first. A positive LLR favours bit 1.
    // reliability is the effective post-equalization inverse noise variance,
    // |H|^2 / sigma^2. A value of zero produces neutral LLRs for that carrier.
    void demap(std::span<const std::complex<float>> equalized_carriers,
               std::span<const float> reliability,
               std::span<float> output_llrs) const;

  private:
    enum class Axis {
        in_phase,
        quadrature,
    };

    struct BitDecision {
        Axis axis{Axis::in_phase};
        std::array<float, 4> zero_levels{};
        std::array<float, 4> one_levels{};
        std::size_t zero_count{};
        std::size_t one_count{};
    };

    Constellation constellation_;
    std::size_t bits_per_symbol_;
    float axis_gain_;
    float half_inverse_axis_gain_;
    int maximum_axis_index_;
    std::vector<std::complex<float>> points_;
    std::array<BitDecision, 6> decisions_{};
};

} // namespace airspy_tv::dvbt
