#include "airspy_tv/transport_observer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <ranges>
#include <span>
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

bool test_ordering_and_retune() {
    std::mutex mutex;
    std::condition_variable ready;
    std::vector<int> events;
    airspy_tv::AsyncTransportObserver observer(
        [&](const std::span<const std::uint8_t> block) {
            const std::scoped_lock lock(mutex);
            events.push_back(block.front());
            ready.notify_all();
        },
        [&](const airspy_tv::TransportDiscontinuity discontinuity) {
            const std::scoped_lock lock(mutex);
            events.push_back(discontinuity ==
                                     airspy_tv::TransportDiscontinuity::retune
                                 ? -2
                                 : -1);
            ready.notify_all();
        },
        {.queue_capacity_bytes = 1024, .thread_name = "ts-obs-test"});
    const std::vector<std::uint8_t> first(188, 1);
    const std::vector<std::uint8_t> second(188, 2);
    observer.submit(first);
    observer.notify_discontinuity(airspy_tv::TransportDiscontinuity::retune);
    observer.submit(second);
    {
        std::unique_lock lock(mutex);
        ready.wait_for(lock, std::chrono::seconds(1),
                       [&] { return events.size() >= 2; });
    }
    observer.stop(true);
    const auto stats = observer.stats();
    return require(!events.empty() && events.back() == 2,
                   "post-retune data reaches the observer") &&
           require(std::ranges::find(events, -2) != events.end(),
                   "retune marker reaches the observer") &&
           require(stats.queue_capacity_bytes == 1024,
                   "observer reports its configured capacity");
}

bool test_overflow_is_local_and_recovers() {
    std::mutex mutex;
    std::condition_variable release;
    bool consume_allowed{};
    std::uint64_t local_gaps{};
    airspy_tv::AsyncTransportObserver observer(
        [&](std::span<const std::uint8_t>) {
            std::unique_lock lock(mutex);
            release.wait(lock, [&] { return consume_allowed; });
        },
        [&](const airspy_tv::TransportDiscontinuity discontinuity) {
            if (discontinuity ==
                airspy_tv::TransportDiscontinuity::fec_region_reset) {
                ++local_gaps;
            }
        },
        {.queue_capacity_bytes = 188, .thread_name = "ts-obs-test"});
    const std::vector<std::uint8_t> packet(188, 0x47);
    observer.submit(packet);
    observer.submit(packet);
    observer.submit(packet);
    {
        const std::scoped_lock lock(mutex);
        consume_allowed = true;
    }
    release.notify_all();
    observer.stop(true);
    const auto stats = observer.stats();
    return require(stats.dropped_blocks != 0,
                   "slow observer drops only its queued blocks") &&
           require(local_gaps != 0,
                   "observer receives a local gap after overflow") &&
           require(!stats.failed, "overflow does not fail the observer");
}

bool test_single_oversized_block_is_processed() {
    std::mutex mutex;
    std::condition_variable ready;
    std::size_t processed_bytes{};
    airspy_tv::AsyncTransportObserver observer(
        [&](const std::span<const std::uint8_t> block) {
            const std::scoped_lock lock(mutex);
            processed_bytes += block.size();
            ready.notify_all();
        },
        [](airspy_tv::TransportDiscontinuity) {},
        {.queue_capacity_bytes = 188, .thread_name = "ts-obs-test"});
    const std::vector<std::uint8_t> oversized(188 * 4, 0x47);
    const bool accepted = observer.submit(oversized);
    {
        std::unique_lock lock(mutex);
        ready.wait_for(lock, std::chrono::seconds(1),
                       [&] { return processed_bytes == oversized.size(); });
    }
    observer.stop(true);
    const auto stats = observer.stats();
    return require(accepted, "single oversized observer block is accepted") &&
           require(processed_bytes == oversized.size(),
                   "single oversized observer block is processed") &&
           require(stats.blocks_accepted == 1 && stats.dropped_blocks == 0,
                   "oversized observer block is not reported as dropped");
}

bool test_stats_are_safe_during_callback_completion() {
    std::mutex mutex;
    std::condition_variable ready;
    bool consume_started{};
    bool consume_allowed{};
    airspy_tv::AsyncTransportObserver observer(
        [&](std::span<const std::uint8_t>) {
            std::unique_lock lock(mutex);
            consume_started = true;
            ready.notify_all();
            ready.wait(lock, [&] { return consume_allowed; });
        },
        [](airspy_tv::TransportDiscontinuity) {},
        {.queue_capacity_bytes = 188, .thread_name = "ts-obs-test"});
    const std::vector<std::uint8_t> packet(188, 0x47);
    observer.submit(packet);
    {
        std::unique_lock lock(mutex);
        ready.wait_for(lock, std::chrono::seconds(1),
                       [&] { return consume_started; });
    }

    std::atomic_bool keep_reading{true};
    std::thread reader([&] {
        while (keep_reading.load(std::memory_order_relaxed)) {
            static_cast<void>(observer.stats());
        }
    });
    {
        const std::scoped_lock lock(mutex);
        consume_allowed = true;
    }
    ready.notify_all();
    for (int attempt = 0;
         attempt < 1000 && observer.stats().blocks_processed == 0; ++attempt) {
        std::this_thread::yield();
    }
    keep_reading.store(false, std::memory_order_relaxed);
    reader.join();
    observer.stop(true);
    return require(observer.stats().blocks_processed == 1,
                   "stats observe callback completion without a data race");
}

} // namespace

int main() {
    return test_ordering_and_retune() &&
                   test_overflow_is_local_and_recovers() &&
                   test_single_oversized_block_is_processed() &&
                   test_stats_are_safe_during_callback_completion()
               ? 0
               : 1;
}
