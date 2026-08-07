#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace airspy_tv::dsp {

inline void
convert_cs16_to_cf32(const std::span<const std::int16_t> interleaved_iq,
                     const std::span<std::complex<float>> output) {
    if ((interleaved_iq.size() % 2) != 0 ||
        interleaved_iq.size() / 2 != output.size()) {
        throw std::invalid_argument("CS16 conversion size mismatch");
    }
    static_assert(sizeof(std::complex<float>) == 2 * sizeof(float));
    auto *const scalars = reinterpret_cast<float *>(output.data());
    constexpr float scale = 1.0F / 32768.0F;
    for (std::size_t index = 0; index < interleaved_iq.size(); ++index) {
        scalars[index] = static_cast<float>(interleaved_iq[index]) * scale;
    }
}

inline void magnitude_squared(const std::span<const std::complex<float>> input,
                              const std::span<float> output) {
    if (input.size() != output.size()) {
        throw std::invalid_argument("magnitude output size mismatch");
    }
    for (std::size_t index = 0; index < input.size(); ++index) {
        const float real = input[index].real();
        const float imag = input[index].imag();
        output[index] = (real * real) + (imag * imag);
    }
}

inline void multiply_real(const std::span<const std::complex<float>> input,
                          const std::span<const float> factors,
                          const std::span<std::complex<float>> output) {
    if (input.size() != factors.size() || input.size() != output.size()) {
        throw std::invalid_argument("real-vector multiply size mismatch");
    }
    for (std::size_t index = 0; index < input.size(); ++index) {
        output[index] = input[index] * factors[index];
    }
}

[[nodiscard]] inline float sum(const std::span<const float> input) noexcept {
    std::size_t index = 0;
    std::array<float, 8> lanes{};
    for (; input.size() - index >= 8; index += 8) {
        lanes[0] += input[index];
        lanes[1] += input[index + 1];
        lanes[2] += input[index + 2];
        lanes[3] += input[index + 3];
        lanes[4] += input[index + 4];
        lanes[5] += input[index + 5];
        lanes[6] += input[index + 6];
        lanes[7] += input[index + 7];
    }
    float result = lanes[0] + lanes[1] + lanes[2] + lanes[3] + lanes[4] +
                   lanes[5] + lanes[6] + lanes[7];
    for (; index < input.size(); ++index) {
        result += input[index];
    }
    return result;
}

} // namespace airspy_tv::dsp
