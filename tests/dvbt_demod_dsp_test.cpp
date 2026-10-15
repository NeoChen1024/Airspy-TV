#include "demod_dsp.hpp"

#include <iostream>
#include <string_view>

namespace {

using airspy_tv::dvbt::timing_measurement_confidence;

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool test_timing_measurement_confidence() {
    return require(timing_measurement_confidence(0, 0) == 0.0F,
                   "timing confidence is zero without measurements") &&
           require(timing_measurement_confidence(100, 100) == 1.0F,
                   "timing confidence is one when every measurement counts") &&
           require(timing_measurement_confidence(75, 100) == 0.75F,
                   "timing confidence is the accepted fraction") &&
           require(timing_measurement_confidence(101, 100) == 1.0F,
                   "timing confidence is capped at one");
}

} // namespace

int main() { return test_timing_measurement_confidence() ? 0 : 1; }
