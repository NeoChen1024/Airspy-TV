#include "airspy_tv/dvbt/soft_demapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <stdexcept>

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
      points_(make_constellation(constellation)) {}

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

    for (std::size_t carrier = 0; carrier < equalized_carriers.size();
         ++carrier) {
        const float carrier_reliability = std::max(reliability[carrier], 0.0F);
        for (std::size_t bit = 0; bit < bits_per_symbol_; ++bit) {
            float minimum_zero = std::numeric_limits<float>::infinity();
            float minimum_one = std::numeric_limits<float>::infinity();
            const std::size_t mask = std::size_t{1}
                                     << (bits_per_symbol_ - bit - 1);
            for (std::size_t label = 0; label < points_.size(); ++label) {
                const float distance =
                    std::norm(equalized_carriers[carrier] - points_[label]);
                auto &minimum =
                    (label & mask) == 0U ? minimum_zero : minimum_one;
                minimum = std::min(minimum, distance);
            }
            output_llrs[(carrier * bits_per_symbol_) + bit] =
                carrier_reliability * (minimum_zero - minimum_one);
        }
    }
}

} // namespace airspy_tv::dvbt
