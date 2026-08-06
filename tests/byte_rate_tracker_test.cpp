#include "byte_rate_tracker.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace {

using airspy_tv::ByteRateTracker;
using namespace std::chrono_literals;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "Byte rate tracker test failed: " << message << '\n';
        std::exit(1);
    }
}

void require_near(const double actual, const double expected,
                  const std::string_view message) {
    require(std::abs(actual - expected) < 1.0e-9, message);
}

void test_windowed_rate_and_idle_reset() {
    ByteRateTracker tracker;
    const ByteRateTracker::Clock::time_point start{};
    constexpr std::uint64_t mib = 1024U * 1024U;

    require_near(tracker.update(true, 0, start), 0.0,
                 "first active sample should establish a baseline");
    require_near(tracker.update(true, mib, start + 500ms), 0.0,
                 "sub-second callback bursts should remain hidden");
    require_near(tracker.update(true, 2 * mib, start + 1s), 2.0,
                 "one-second sample should report MiB per second");
    require_near(tracker.update(true, 2 * mib, start + 2s), 0.0,
                 "a full window without writes should report zero");
    require_near(tracker.update(false, 2 * mib, start + 3s), 0.0,
                 "inactive recorder should reset the displayed rate");
}

void test_counter_restart_rebaselines() {
    ByteRateTracker tracker;
    const ByteRateTracker::Clock::time_point start{};
    constexpr std::uint64_t mib = 1024U * 1024U;

    static_cast<void>(tracker.update(true, 4 * mib, start));
    require_near(tracker.update(true, mib, start + 1s), 0.0,
                 "a restarted byte counter should not underflow");
    require_near(tracker.update(true, 2 * mib, start + 2s), 1.0,
                 "rate should resume from the new recording baseline");
}

} // namespace

int main() {
    test_windowed_rate_and_idle_reset();
    test_counter_restart_rebaselines();
    std::cout << "Byte rate tracker tests passed\n";
    return 0;
}
