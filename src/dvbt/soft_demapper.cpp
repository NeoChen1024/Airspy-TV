#include "airspy_tv/dvbt/soft_demapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

[[nodiscard]] std::size_t
constellation_size(const Constellation constellation) {
    return std::size_t{1} << bits_per_symbol(constellation);
}

[[nodiscard]] float normalization(const Constellation constellation) {
    switch (constellation) {
    case Constellation::qpsk:
        return 1.0F / std::sqrt(2.0F);
    case Constellation::qam16:
        return 1.0F / std::sqrt(10.0F);
    case Constellation::qam64:
        return 1.0F / std::sqrt(42.0F);
    }
    throw std::invalid_argument("unsupported DVB-T constellation");
}

[[nodiscard]] std::vector<std::complex<float>>
make_constellation(const Constellation constellation) {
    const std::size_t bit_count = bits_per_symbol(constellation);
    const std::size_t bits_per_axis = bit_count / 2;
    const std::size_t size = constellation_size(constellation);
    const int steps_per_axis = static_cast<int>(std::sqrt(size) / 2) - 1;
    const float gain = normalization(constellation);
    std::vector<std::complex<float>> points(size);

    for (std::size_t source = 0; source < size; ++source) {
        const std::size_t quadrant = source >> (2 * (bits_per_axis - 1));
        const float sign_i = (quadrant & 2U) != 0U ? -1.0F : 1.0F;
        const float sign_q = (quadrant & 1U) != 0U ? -1.0F : 1.0F;
        const std::size_t inner_mask =
            (std::size_t{1} << (bits_per_axis - 1)) - 1;
        const std::size_t axis_i = (source >> (bits_per_axis - 1)) & inner_mask;
        const std::size_t axis_q = source & inner_mask;
        const float value_i =
            1.0F +
            ((static_cast<float>(steps_per_axis) - static_cast<float>(axis_i)) *
             2.0F);
        const float value_q =
            1.0F +
            ((static_cast<float>(steps_per_axis) - static_cast<float>(axis_q)) *
             2.0F);

        const std::size_t gray_i = (axis_i >> 1) ^ axis_i;
        const std::size_t gray_q = (axis_q >> 1) ^ axis_q;
        const std::size_t gray = (gray_i << (bits_per_axis - 1)) + gray_q;
        std::size_t label_i = 0;
        std::size_t label_q = 0;
        for (std::size_t bit = 0; bit < bits_per_axis - 1; ++bit) {
            label_i |= ((gray >> (1 + (2 * bit))) & 1U) << bit;
            label_q |= ((gray >> (2 * bit)) & 1U) << bit;
        }
        const std::size_t label = (quadrant << (2 * (bits_per_axis - 1))) +
                                  (label_i << (bits_per_axis - 1)) + label_q;
        points[label] =
            gain * std::complex<float>{sign_i * value_i, sign_q * value_q};
    }
    return points;
}

} // namespace

std::size_t bits_per_symbol(const Constellation constellation) {
    switch (constellation) {
    case Constellation::qpsk:
        return 2;
    case Constellation::qam16:
        return 4;
    case Constellation::qam64:
        return 6;
    }
    throw std::invalid_argument("unsupported DVB-T constellation");
}

MaxLogDemapper::MaxLogDemapper(const Constellation constellation)
    : constellation_(constellation),
      bits_per_symbol_(dvbt::bits_per_symbol(constellation)),
      axis_gain_(normalization(constellation)),
      half_inverse_axis_gain_(0.5F / axis_gain_),
      maximum_axis_index_(
          static_cast<int>((std::size_t{1} << (bits_per_symbol_ / 2 - 1)) - 1)),
      points_(make_constellation(constellation)) {
    for (std::size_t bit = 0; bit < bits_per_symbol_; ++bit) {
        const std::size_t mask = std::size_t{1} << (bits_per_symbol_ - bit - 1);
        bool found_axis = false;
        for (const Axis axis : {Axis::in_phase, Axis::quadrature}) {
            std::vector<std::pair<float, bool>> levels;
            bool separable = true;
            for (std::size_t label = 0; label < points_.size(); ++label) {
                const float coordinate = axis == Axis::in_phase
                                             ? points_[label].real()
                                             : points_[label].imag();
                const bool one = (label & mask) != 0U;
                const auto existing = std::ranges::find_if(
                    levels, [coordinate](const auto &level) {
                        return level.first == coordinate;
                    });
                if (existing == levels.end()) {
                    levels.emplace_back(coordinate, one);
                } else if (existing->second != one) {
                    separable = false;
                    break;
                }
            }
            if (!separable) {
                continue;
            }

            auto &decision = decisions_[bit];
            decision.axis = axis;
            for (const auto [level, one] : levels) {
                auto &destination =
                    one ? decision.one_levels : decision.zero_levels;
                auto &count = one ? decision.one_count : decision.zero_count;
                if (count >= destination.size()) {
                    throw std::logic_error("unsupported square-QAM PAM order");
                }
                destination[count++] = level;
            }
            found_axis = decision.zero_count != 0 && decision.one_count != 0;
            if (found_axis) {
                break;
            }
        }
        if (!found_axis) {
            throw std::logic_error("DVB-T constellation bit is not separable");
        }
    }
}

Constellation MaxLogDemapper::constellation() const noexcept {
    return constellation_;
}

std::size_t MaxLogDemapper::bits_per_symbol() const noexcept {
    return bits_per_symbol_;
}

std::span<const std::complex<float>>
MaxLogDemapper::constellation_points() const noexcept {
    return points_;
}

std::complex<float> MaxLogDemapper::nearest_constellation_point(
    const std::complex<float> value) const noexcept {
    const auto slice_axis = [this](const float coordinate) {
        const int index = std::min(
            static_cast<int>(std::abs(coordinate) * half_inverse_axis_gain_),
            maximum_axis_index_);
        const float level = static_cast<float>((2 * index) + 1) * axis_gain_;
        return std::copysign(level, coordinate);
    };
    return {slice_axis(value.real()), slice_axis(value.imag())};
}

void MaxLogDemapper::slice_nearest(
    const std::span<const std::complex<float>> input,
    const std::span<std::complex<float>> output) const {
    if (output.size() != input.size()) {
        throw std::invalid_argument("nearest-point output size mismatch");
    }
    const float gain = axis_gain_;
    const float half_inverse_gain = half_inverse_axis_gain_;
    const int maximum_index = maximum_axis_index_;
    for (std::size_t index = 0; index < input.size(); ++index) {
        const float real = input[index].real();
        const float imaginary = input[index].imag();
        const int real_index =
            std::min(static_cast<int>(std::abs(real) * half_inverse_gain),
                     maximum_index);
        const int imaginary_index =
            std::min(static_cast<int>(std::abs(imaginary) * half_inverse_gain),
                     maximum_index);
        output[index] = {
            std::copysign(static_cast<float>((2 * real_index) + 1) * gain,
                          real),
            std::copysign(static_cast<float>((2 * imaginary_index) + 1) * gain,
                          imaginary)};
    }
}

void MaxLogDemapper::demap(
    const std::span<const std::complex<float>> equalized_carriers,
    const std::span<const float> reliability,
    const std::span<float> output_llrs) const {
    if (reliability.size() != equalized_carriers.size()) {
        throw std::invalid_argument("carrier reliability size mismatch");
    }
    if (output_llrs.size() != equalized_carriers.size() * bits_per_symbol_) {
        throw std::invalid_argument("LLR output size mismatch");
    }

    constexpr std::size_t tile_size = 256;
    alignas(64) std::array<float, tile_size> coordinates{};
    alignas(64) std::array<float, tile_size> minimum_zero{};
    alignas(64) std::array<float, tile_size> minimum_one{};

    // Square QAM labels are separable into Gray-coded PAM decisions on I or Q.
    // The nearest distance on the other axis is identical for bit zero and bit
    // one and cancels from their Max-Log difference.  Keeping carriers in the
    // innermost, contiguous loops lets ordinary C++20 compilers vectorize the
    // distance kernels without an architecture-specific implementation.
    for (std::size_t bit = 0; bit < bits_per_symbol_; ++bit) {
        const BitDecision &decision = decisions_[bit];
        for (std::size_t base = 0; base < equalized_carriers.size();
             base += tile_size) {
            const std::size_t count =
                std::min(tile_size, equalized_carriers.size() - base);
            for (std::size_t index = 0; index < count; ++index) {
                coordinates[index] =
                    decision.axis == Axis::in_phase
                        ? equalized_carriers[base + index].real()
                        : equalized_carriers[base + index].imag();
            }

            for (std::size_t index = 0; index < count; ++index) {
                const float delta =
                    coordinates[index] - decision.zero_levels.front();
                minimum_zero[index] = delta * delta;
            }
            for (std::size_t level = 1; level < decision.zero_count; ++level) {
                for (std::size_t index = 0; index < count; ++index) {
                    const float delta =
                        coordinates[index] - decision.zero_levels[level];
                    minimum_zero[index] =
                        std::min(minimum_zero[index], delta * delta);
                }
            }

            for (std::size_t index = 0; index < count; ++index) {
                const float delta =
                    coordinates[index] - decision.one_levels.front();
                minimum_one[index] = delta * delta;
            }
            for (std::size_t level = 1; level < decision.one_count; ++level) {
                for (std::size_t index = 0; index < count; ++index) {
                    const float delta =
                        coordinates[index] - decision.one_levels[level];
                    minimum_one[index] =
                        std::min(minimum_one[index], delta * delta);
                }
            }

            for (std::size_t index = 0; index < count; ++index) {
                const float carrier_reliability =
                    std::max(reliability[base + index], 0.0F);
                output_llrs[((base + index) * bits_per_symbol_) + bit] =
                    carrier_reliability *
                    (minimum_zero[index] - minimum_one[index]);
            }
        }
    }
}

} // namespace airspy_tv::dvbt
