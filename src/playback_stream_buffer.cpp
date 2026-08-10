#include "playback_stream_buffer.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <vector>

namespace airspy_tv {
namespace {
constexpr std::size_t transport_packet_size = 188;
}

struct PlaybackStreamBuffer::Impl {
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::deque<std::vector<std::uint8_t>> queue;
    std::size_t front_offset{};
    std::size_t queued_bytes{};
    std::uint64_t generation{};
    bool source_active{};
    bool end_of_stream{};
    bool buffering{true};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_processed{};
    std::uint64_t bytes_processed{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::uint64_t discontinuities{};

    void clear_locked() {
        dropped_blocks += queue.size();
        dropped_bytes += queued_bytes;
        queue.clear();
        front_offset = 0;
        queued_bytes = 0;
        buffering = true;
    }

    PlaybackBufferAction restart_locked() {
        ++generation;
        clear_locked();
        end_of_stream = false;
        return source_active ? PlaybackBufferAction::load
                             : PlaybackBufferAction::stop;
    }
};

PlaybackStreamBuffer::PlaybackStreamBuffer() : impl_(std::make_unique<Impl>()) {}
PlaybackStreamBuffer::~PlaybackStreamBuffer() noexcept = default;

PlaybackStreamBuffer::Reader PlaybackStreamBuffer::open_reader() const {
    const std::scoped_lock lock(impl_->mutex);
    return {.generation = impl_->generation};
}

std::size_t PlaybackStreamBuffer::read(Reader &reader,
                                       const std::span<std::uint8_t> output) {
    if (output.empty()) {
        return 0;
    }
    std::unique_lock lock(impl_->mutex);
    impl_->ready.wait(lock, [&] {
        return reader.canceled->load(std::memory_order_relaxed) ||
               reader.generation != impl_->generation ||
               (!impl_->source_active && !impl_->end_of_stream) ||
               impl_->end_of_stream ||
               (!impl_->buffering && !impl_->queue.empty());
    });
    if (reader.canceled->load(std::memory_order_relaxed) ||
        reader.generation != impl_->generation ||
        (!impl_->source_active && !impl_->end_of_stream) ||
        (impl_->end_of_stream && impl_->queue.empty())) {
        return 0;
    }

    std::size_t copied = 0;
    while (copied < output.size() && !impl_->queue.empty()) {
        const auto &block = impl_->queue.front();
        const std::size_t count =
            std::min(output.size() - copied, block.size() - impl_->front_offset);
        std::memcpy(output.data() + copied, block.data() + impl_->front_offset,
                    count);
        copied += count;
        impl_->front_offset += count;
        impl_->queued_bytes -= count;
        if (impl_->front_offset == block.size()) {
            impl_->queue.pop_front();
            impl_->front_offset = 0;
            ++impl_->blocks_processed;
        }
    }
    impl_->bytes_processed += copied;
    if (!impl_->end_of_stream &&
        impl_->queued_bytes <= PlaybackStreamBuffer::low_watermark) {
        impl_->buffering = true;
    }
    return copied;
}

void PlaybackStreamBuffer::cancel(Reader &reader) noexcept {
    reader.canceled->store(true, std::memory_order_relaxed);
    impl_->ready.notify_all();
}

PlaybackBufferAction PlaybackStreamBuffer::set_source_active(const bool active) {
    PlaybackBufferAction action = PlaybackBufferAction::none;
    {
        const std::scoped_lock lock(impl_->mutex);
        if (impl_->source_active == active) {
            return action;
        }
        impl_->source_active = active;
        if (active) {
            action = impl_->restart_locked();
        } else if (!impl_->end_of_stream) {
            action = impl_->restart_locked();
        }
    }
    impl_->ready.notify_all();
    return action;
}

PlaybackBufferAction PlaybackStreamBuffer::restart() {
    PlaybackBufferAction action;
    {
        const std::scoped_lock lock(impl_->mutex);
        action = impl_->restart_locked();
    }
    impl_->ready.notify_all();
    return action;
}

void PlaybackStreamBuffer::submit(const std::span<const std::uint8_t> bytes) {
    if (bytes.empty()) {
        return;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        if (!impl_->source_active || impl_->end_of_stream) {
            return;
        }
        while (!impl_->queue.empty() &&
               impl_->queued_bytes + bytes.size() > capacity) {
            const std::size_t dropped =
                impl_->queue.front().size() - impl_->front_offset;
            ++impl_->dropped_blocks;
            impl_->dropped_bytes += dropped;
            impl_->queued_bytes -= dropped;
            impl_->queue.pop_front();
            impl_->front_offset = 0;
        }
        std::span<const std::uint8_t> accepted = bytes;
        if (accepted.size() > capacity) {
            const std::size_t keep =
                capacity - (capacity % transport_packet_size);
            ++impl_->dropped_blocks;
            impl_->dropped_bytes += accepted.size() - keep;
            accepted = accepted.last(keep);
        }
        ++impl_->blocks_accepted;
        impl_->bytes_accepted += accepted.size();
        impl_->queued_bytes += accepted.size();
        impl_->queue.emplace_back(accepted.begin(), accepted.end());
        if (impl_->buffering && impl_->queued_bytes >= resume_watermark) {
            impl_->buffering = false;
        }
    }
    impl_->ready.notify_all();
}

void PlaybackStreamBuffer::fec_region_reset() noexcept {
    const std::scoped_lock lock(impl_->mutex);
    ++impl_->discontinuities;
}

void PlaybackStreamBuffer::stream_end() noexcept {
    {
        const std::scoped_lock lock(impl_->mutex);
        ++impl_->discontinuities;
        impl_->end_of_stream = true;
        impl_->buffering = false;
    }
    impl_->ready.notify_all();
}

PlaybackBufferAction PlaybackStreamBuffer::retune() {
    PlaybackBufferAction action;
    {
        const std::scoped_lock lock(impl_->mutex);
        ++impl_->discontinuities;
        action = impl_->restart_locked();
    }
    impl_->ready.notify_all();
    return action;
}

void PlaybackStreamBuffer::shutdown() noexcept {
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->source_active = false;
        impl_->end_of_stream = false;
        ++impl_->generation;
        impl_->clear_locked();
    }
    impl_->ready.notify_all();
}

PlaybackBufferSnapshot PlaybackStreamBuffer::snapshot() const {
    const std::scoped_lock lock(impl_->mutex);
    return {.source_active = impl_->source_active,
            .buffering = impl_->source_active && impl_->buffering,
            .queued_bytes = impl_->queued_bytes,
            .queue_capacity = capacity,
            .blocks_accepted = impl_->blocks_accepted,
            .bytes_accepted = impl_->bytes_accepted,
            .blocks_processed = impl_->blocks_processed,
            .bytes_processed = impl_->bytes_processed,
            .dropped_blocks = impl_->dropped_blocks,
            .dropped_bytes = impl_->dropped_bytes,
            .discontinuities = impl_->discontinuities};
}

} // namespace airspy_tv
