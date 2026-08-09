#include "sample_channel.hpp"
#include "absolute_sample_ring.hpp"

#include <array>
#include <atomic>
#include <complex>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::InputSampleStamp;
using airspy_tv::dvbt::InputBlock;
using airspy_tv::dvbt::SampleChannel;

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

InputBlock block(const std::size_t complex_samples,
                 const std::uint64_t begin_sample = 0) {
    InputBlock result;
    result.samples.resize(complex_samples * 2U);
    result.rate = 10'000'000;
    result.bandwidth = 6'000'000;
    result.generation = 0;
    result.stamp = InputSampleStamp{.stream_epoch = 3,
                                    .begin_sample = begin_sample,
                                    .sample_count = complex_samples,
                                    .discontinuity_before = false};
    return result;
}

bool test_queue_ring_and_lifecycle() {
    std::atomic_bool rebootstrap_requested{false};
    SampleChannel channel(4, {.rebootstrap_requested =
                                  [&] {
                                      return rebootstrap_requested.load(
                                          std::memory_order_acquire);
                                  },
                              .notify_idle = [] {}});

    if (!require(channel.submit(block(4), 4, false),
                 "bounded input block is accepted") ||
        !require(!channel.submit(block(1, 4), 4, false),
                 "full input queue rejects live overflow")) {
        return false;
    }
    auto work = channel.wait_frontend();
    if (!require(work.event == SampleChannel::FrontendEvent::block &&
                     work.block.has_value(),
                 "frontend receives the queued block")) {
        return false;
    }
    const auto prepared = channel.prepare_block(work.block->generation, 4);
    if (!require(prepared.accepted && prepared.write_position == 0,
                 "frontend prepares an empty ring")) {
        return false;
    }

    const std::array<std::complex<float>, 4> samples{
        std::complex<float>{1.0F, 0.0F}, std::complex<float>{2.0F, 0.0F},
        std::complex<float>{3.0F, 0.0F}, std::complex<float>{4.0F, 0.0F}};
    const auto pushed = channel.push(work.block->generation, samples);
    channel.finish_frontend_work();
    channel.publish_sync_version(1);
    if (!require(pushed.status == SampleChannel::PushStatus::written &&
                     pushed.written == samples.size(),
                 "frontend writes one complete ring span") ||
        !require(channel.wait_for_stream(1, true, 4) ==
                     SampleChannel::WaitStatus::ready,
                 "demod observes available ring data")) {
        return false;
    }

    std::array<std::complex<float>, 4> recovered{};
    const auto read = channel.read_symbol(1, 0, recovered.size(), 0, false,
                                          std::span{recovered});
    if (!require(read.status == SampleChannel::WaitStatus::ready &&
                     recovered == samples,
                 "ring read preserves sample order") ||
        !require(channel.snapshot().ring_used_samples == 0,
                 "ring read advances the absolute consumer position")) {
        return false;
    }

    channel.request_flush();
    work = channel.wait_frontend();
    if (!require(work.event == SampleChannel::FrontendEvent::close_ring &&
                     channel.snapshot().ring_closed,
                 "flush closes a drained finite ring")) {
        return false;
    }

    const std::uint64_t reset_generation = channel.request_reset();
    channel.publish_sync_version(2);
    channel.acknowledge_demod_reset(reset_generation);
    work = channel.wait_frontend();
    if (!require(work.event == SampleChannel::FrontendEvent::reset &&
                     work.reset_generation == reset_generation,
                 "frontend observes the acknowledged reset")) {
        return false;
    }
    channel.begin_frontend_reset(reset_generation);
    channel.complete_frontend_reset(reset_generation);
    channel.wait_reset_complete(reset_generation);
    if (!require(!channel.cancelled() &&
                     channel.generation() == reset_generation,
                 "completed reset reopens generation submission") ||
        !require(channel.push(reset_generation - 1, samples).status ==
                     SampleChannel::PushStatus::reset,
                 "stale ring writer is rejected")) {
        return false;
    }

    InputBlock active = block(4);
    active.generation = reset_generation;
    rebootstrap_requested.store(true, std::memory_order_release);
    channel.notify_frontend();
    work = channel.wait_frontend();
    if (!require(work.event == SampleChannel::FrontendEvent::rebootstrap,
                 "frontend wakes for CFO rebootstrap")) {
        return false;
    }
    rebootstrap_requested.store(false, std::memory_order_release);
    const auto rebased = channel.rebootstrap(&active, 2);
    const auto snapshot = channel.snapshot();
    const bool ok =
        require(rebased.new_generation == reset_generation + 1 &&
                    rebased.consumed_input_samples == 2,
                "rebootstrap advances generation and records consumed input") &&
        require(snapshot.queued_blocks == 1 &&
                    snapshot.queued_input_samples == 2,
                "rebootstrap preserves the active input suffix") &&
        require(snapshot.ring_read_position == 0 &&
                    snapshot.ring_write_position == 0 && !snapshot.ring_closed,
                "rebootstrap rewinds ring coordinates");
    channel.stop();
    return ok;
}

bool test_absolute_ring_wrapping_copy() {
    AbsoluteSampleRing ring(6);
    const std::array<std::complex<float>, 6> first{
        std::complex<float>{1.0F, 0.0F}, std::complex<float>{2.0F, 0.0F},
        std::complex<float>{3.0F, 0.0F}, std::complex<float>{4.0F, 0.0F},
        std::complex<float>{5.0F, 0.0F}, std::complex<float>{6.0F, 0.0F}};
    std::copy(first.begin(), first.end(), ring.data());
    ring.write_position = first.size();
    ring.read_position = 4;

    const std::array<std::complex<float>, 2> wrapped{
        std::complex<float>{7.0F, 0.0F}, std::complex<float>{8.0F, 0.0F}};
    std::copy(wrapped.begin(), wrapped.end(), ring.data());
    ring.write_position += wrapped.size();

    std::array<std::complex<float>, 4> recovered{};
    ring.copy_absolute(4, recovered);
    const std::array<std::complex<float>, 4> expected{
        std::complex<float>{5.0F, 0.0F}, std::complex<float>{6.0F, 0.0F},
        std::complex<float>{7.0F, 0.0F}, std::complex<float>{8.0F, 0.0F}};
    return require(recovered == expected,
                   "two-span ring copy preserves wrapping sample order");
}

} // namespace

int main() {
    try {
        return test_queue_ring_and_lifecycle() &&
                       test_absolute_ring_wrapping_copy()
                   ? 0
                   : 1;
    } catch (const std::exception &exception) {
        std::cerr << "Sample channel test failed: " << exception.what() << '\n';
        return 1;
    }
}
