#include "playback_stream_buffer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <ranges>
#include <span>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::PlaybackBufferAction;
using airspy_tv::PlaybackStreamBuffer;
using namespace std::chrono_literals;

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

std::future<std::vector<std::uint8_t>>
read_async(PlaybackStreamBuffer &buffer, PlaybackStreamBuffer::Reader &reader,
           const std::size_t count) {
    return std::async(std::launch::async, [&buffer, &reader, count] {
        std::vector<std::uint8_t> bytes(count);
        bytes.resize(buffer.read(reader, bytes));
        return bytes;
    });
}

bool test_hysteresis_and_fake_reader() {
    PlaybackStreamBuffer buffer;
    if (!require(buffer.set_source_active(true) == PlaybackBufferAction::load,
                 "activating playback requests a load")) {
        return false;
    }
    auto reader = buffer.open_reader();
    const std::vector<std::uint8_t> first(PlaybackStreamBuffer::resume_watermark -
                                              188,
                                          0x11);
    buffer.submit(first);
    auto pending = read_async(buffer, reader, 512U << 10U);
    if (!require(pending.wait_for(20ms) == std::future_status::timeout,
                 "reader waits below the resume watermark")) {
        buffer.cancel(reader);
        return false;
    }
    buffer.submit(std::vector<std::uint8_t>(188, 0x11));
    if (!require(pending.wait_for(1s) == std::future_status::ready,
                 "reader wakes at the resume watermark") ||
        !require(pending.get() == std::vector<std::uint8_t>(512U << 10U, 0x11),
                 "reader receives ordered bytes")) {
        return false;
    }

    std::vector<std::uint8_t> drain(512U << 10U);
    static_cast<void>(buffer.read(reader, drain));
    const auto low = buffer.snapshot();
    if (!require(low.queued_bytes <= PlaybackStreamBuffer::low_watermark &&
                     low.buffering,
                 "draining through the low watermark re-enters buffering")) {
        return false;
    }
    auto rebuffered = read_async(buffer, reader, 188);
    if (!require(rebuffered.wait_for(20ms) == std::future_status::timeout,
                 "reader remains blocked while rebuffering")) {
        buffer.cancel(reader);
        return false;
    }
    const std::size_t refill = PlaybackStreamBuffer::resume_watermark -
                               buffer.snapshot().queued_bytes;
    buffer.submit(std::vector<std::uint8_t>(refill, 0x22));
    return require(rebuffered.wait_for(1s) == std::future_status::ready,
                   "reader resumes after refill") &&
           require(rebuffered.get().size() == 188,
                   "resumed reader receives its request");
}

bool test_discontinuity_ordering_and_tail_eof() {
    PlaybackStreamBuffer buffer;
    static_cast<void>(buffer.set_source_active(true));
    auto first_reader = buffer.open_reader();
    const std::vector<std::uint8_t> tail(188 * 7, 0x31);
    buffer.submit(tail);
    buffer.fec_region_reset();
    if (!require(buffer.snapshot().queued_bytes == tail.size(),
                 "FEC reset preserves queued bytes")) {
        return false;
    }
    buffer.stream_end();
    std::vector<std::uint8_t> output(tail.size());
    const std::size_t copied = buffer.read(first_reader, output);
    std::vector<std::uint8_t> eof(188);
    const std::size_t after_tail = buffer.read(first_reader, eof);
    const auto deactivated = buffer.set_source_active(false);
    if (!require(copied == tail.size() && output == tail,
                 "stream end drains a sub-watermark tail") ||
        !require(after_tail == 0, "stream end returns EOF after the tail") ||
        !require(deactivated == PlaybackBufferAction::none,
                 "GUI deactivation does not discard a graceful tail")) {
        return false;
    }

    static_cast<void>(buffer.set_source_active(true));
    auto stale_reader = buffer.open_reader();
    buffer.submit(std::vector<std::uint8_t>(
        PlaybackStreamBuffer::resume_watermark, 0x41));
    if (!require(buffer.retune() == PlaybackBufferAction::load,
                 "live retune requests a reload")) {
        return false;
    }
    std::vector<std::uint8_t> stale(188);
    if (!require(buffer.read(stale_reader, stale) == 0,
                 "retune terminates the old reader generation") ||
        !require(buffer.snapshot().queued_bytes == 0,
                 "retune discards pre-boundary bytes")) {
        return false;
    }
    auto new_reader = buffer.open_reader();
    buffer.submit(std::vector<std::uint8_t>(
        PlaybackStreamBuffer::resume_watermark, 0x42));
    std::vector<std::uint8_t> fresh(188);
    return require(buffer.read(new_reader, fresh) == fresh.size(),
                   "new reader receives post-retune bytes") &&
           require(std::ranges::all_of(fresh,
                                       [](const auto byte) { return byte == 0x42; }),
                   "post-retune data cannot include the old generation") &&
           require(buffer.snapshot().discontinuities == 3,
                   "all discontinuities are counted");
}

bool test_hard_capacity_drops_oldest() {
    PlaybackStreamBuffer buffer;
    static_cast<void>(buffer.set_source_active(true));
    const std::vector<std::uint8_t> old(5U << 20U, 0x51);
    const std::vector<std::uint8_t> fresh(5U << 20U, 0x52);
    buffer.submit(old);
    buffer.submit(fresh);
    const auto snapshot = buffer.snapshot();
    return require(snapshot.queued_bytes == fresh.size(),
                   "overflow retains the newest complete block") &&
           require(snapshot.dropped_blocks == 1 &&
                       snapshot.dropped_bytes == old.size(),
                   "overflow accounts for the discarded oldest block") &&
           require(snapshot.queued_bytes <= snapshot.queue_capacity,
                   "playback queue remains bounded");
}

} // namespace

int main() {
    return test_hysteresis_and_fake_reader() &&
                   test_discontinuity_ordering_and_tail_eof() &&
                   test_hard_capacity_drops_oldest()
               ? 0
               : 1;
}
