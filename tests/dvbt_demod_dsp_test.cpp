#include "demod_dsp.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <optional>
#include <random>
#include <span>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::dvbt::select_exact_median;
using airspy_tv::dvbt::timing_measurement_confidence;

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

std::optional<double> sorted_median(std::vector<double> values) {
    if (values.empty()) {
        return std::nullopt;
    }
    std::ranges::sort(values);
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 != 0) {
        return values[middle];
    }
    return 0.5 * (values[middle - 1] + values[middle]);
}

bool same(const std::optional<double> left,
          const std::optional<double> right) {
    if (left.has_value() != right.has_value()) {
        return false;
    }
    return !left.has_value() || *left == *right;
}

bool test_exact_cases() {
    std::vector<std::vector<double>> cases{
        {}, {4.0}, {4.0, 1.0}, {9.0, 1.0, 4.0},
        {9.0, 1.0, 4.0, 7.0}, {3.0, 3.0, 3.0, 3.0},
        {-8.0, -2.0, 7.0, 12.0, 12.0, 19.0}};
    for (auto values : cases) {
        const auto expected = sorted_median(values);
        if (!require(same(select_exact_median(values), expected),
                     "selection median matches sorted reference")) {
            return false;
        }
    }
    return true;
}

bool test_randomized_exactness() {
    std::mt19937_64 random(0x44564254U);
    std::uniform_int_distribution<int> size_distribution(0, 1024);
    std::uniform_int_distribution<int> value_distribution(-200, 200);
    for (std::size_t iteration = 0; iteration < 2'000; ++iteration) {
        std::vector<double> values(
            static_cast<std::size_t>(size_distribution(random)));
        for (auto &value : values) {
            value = static_cast<double>(value_distribution(random)) / 8.0;
        }
        const auto expected = sorted_median(values);
        if (!require(same(select_exact_median(values), expected),
                     "randomized selection median is numerically exact")) {
            return false;
        }
    }
    return true;
}

bool test_timing_measurement_confidence() {
    return require(timing_measurement_confidence(0, 0) == 0.0F,
                   "no timing attempts have zero confidence") &&
           require(timing_measurement_confidence(100, 100) == 1.0F,
                   "cadenced timing accepts all attempted measurements") &&
           require(timing_measurement_confidence(75, 100) == 0.75F,
                   "timing confidence reports the acceptance ratio") &&
           require(timing_measurement_confidence(101, 100) == 1.0F,
                   "timing confidence is capped at one");
}

} // namespace

int main() {
    return test_exact_cases() && test_randomized_exactness() &&
                   test_timing_measurement_confidence()
               ? 0
               : 1;
}
