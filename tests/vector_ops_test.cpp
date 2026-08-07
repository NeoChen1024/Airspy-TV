#include "airspy_tv/dsp/vector_ops.hpp"

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using airspy_tv::dsp::convert_cs16_to_cf32;
using airspy_tv::dsp::magnitude_squared;
using airspy_tv::dsp::multiply_real;
using airspy_tv::dsp::sum;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

void test_conversion() {
    constexpr std::array<std::int16_t, 8> input{-32768, 32767, -16384, 16384,
                                                -1,     1,     0,      8192};
    std::array<std::complex<float>, 4> output{};
    convert_cs16_to_cf32(input, output);
    constexpr float scale = 1.0F / 32768.0F;
    for (std::size_t index = 0; index < output.size(); ++index) {
        require(output[index].real() ==
                    static_cast<float>(input[index * 2]) * scale,
                "CS16 real conversion mismatch");
        require(output[index].imag() ==
                    static_cast<float>(input[(index * 2) + 1]) * scale,
                "CS16 imaginary conversion mismatch");
    }
}

void test_spectrum_operations() {
    constexpr std::array<std::complex<float>, 4> input{
        std::complex<float>{3.0F, 4.0F},
        {1.0F, -2.0F},
        {-2.0F, -2.0F},
        {0.5F, 0.25F}};
    constexpr std::array<float, 4> factors{0.0F, 0.25F, 0.5F, 2.0F};
    std::array<float, 4> power{};
    std::array<std::complex<float>, 4> multiplied{};
    magnitude_squared(input, power);
    multiply_real(input, factors, multiplied);

    constexpr std::array<float, 4> expected_power{25.0F, 5.0F, 8.0F, 0.3125F};
    for (std::size_t index = 0; index < input.size(); ++index) {
        require(power[index] == expected_power[index],
                "magnitude squared mismatch");
        require(multiplied[index] == input[index] * factors[index],
                "real-vector multiply mismatch");
    }
}

void test_sum() {
    std::array<float, 19> input{};
    for (std::size_t index = 0; index < input.size(); ++index) {
        input[index] = static_cast<float>(index + 1);
    }
    require(sum(input) == 190.0F, "vector sum mismatch");
    require(sum(std::span<const float>{}) == 0.0F, "empty vector sum mismatch");
}

void test_size_validation() {
    std::array<std::int16_t, 4> input{};
    std::array<std::complex<float>, 1> complex_output{};
    bool threw = false;
    try {
        convert_cs16_to_cf32(input, complex_output);
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    require(threw, "conversion accepted mismatched spans");
}

} // namespace

int main() {
    try {
        test_conversion();
        test_spectrum_operations();
        test_sum();
        test_size_validation();
        std::cout << "Vector operation tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Vector operation test failure: " << error.what() << '\n';
        return 1;
    }
}
