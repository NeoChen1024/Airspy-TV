#include "airspy_tv/dvbt/signal_analyzer.hpp"

#include <barrier>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <string_view>
#include <thread>
#include <vector>

namespace {

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool test_concurrent_submit_reset_and_snapshot() {
    constexpr std::size_t scalar_count = 700'000;
    std::vector<std::int16_t> iq(scalar_count);
    std::uint32_t state = 0x13579BDFU;
    for (auto &sample : iq) {
        state = (state * 1664525U) + 1013904223U;
        sample = static_cast<std::int16_t>(state >> 16U);
    }

    airspy_tv::dvbt::SignalAnalyzer analyzer;
    std::barrier iteration_start{2};
    std::barrier iteration_finished{2};
    constexpr int iterations = 128;
    std::thread producer([&] {
        for (int iteration = 0; iteration < iterations; ++iteration) {
            iteration_start.arrive_and_wait();
            analyzer.submit(iq, 10'000'000,
                            iteration % 2 == 0 ? 6'000'000 : 8'000'000);
            iteration_finished.arrive_and_wait();
        }
    });
    std::thread controller([&] {
        for (int iteration = 0; iteration < iterations; ++iteration) {
            iteration_start.arrive_and_wait();
            if (iteration % 4 == 0) {
                analyzer.reset();
            } else if (iteration % 4 == 1) {
                airspy_tv::dvbt::ReceiverParameters parameters;
                parameters.channel_bandwidth_hz = 6'000'000;
                parameters.mode = airspy_tv::dvbt::TransmissionMode::k8;
                parameters.guard_interval =
                    airspy_tv::dvbt::GuardInterval::gi_1_4;
                analyzer.set_parameters(parameters);
            } else if (iteration % 4 == 2) {
                airspy_tv::dvbt::ReceiverParameters parameters;
                parameters.channel_bandwidth_hz = 8'000'000;
                analyzer.set_parameters(parameters);
            } else {
                analyzer.set_snr_smoothing(iteration % 8 == 3,
                                           (iteration % 31) + 1);
                const auto snapshot = analyzer.snapshot();
                if (snapshot.point_count >
                    airspy_tv::dvbt::constellation_snapshot_size) {
                    std::terminate();
                }
            }
            iteration_finished.arrive_and_wait();
        }
    });
    producer.join();
    controller.join();

    analyzer.reset();
    const auto snapshot = analyzer.snapshot();
    return require(snapshot.point_count == 0 && !snapshot.locked,
                   "final reset publishes an empty unlocked snapshot") &&
           require(std::isfinite(snapshot.mer_db) &&
                       std::isfinite(snapshot.cp_snr_db),
                   "snapshot fields remain initialized after stress");
}

} // namespace

int main() { return test_concurrent_submit_reset_and_snapshot() ? 0 : 1; }
