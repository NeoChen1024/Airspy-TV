#include "transport_write_all.hpp"

#include "airspy_tv/thread_name.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::detail {

bool transport_write_all(const std::span<const std::uint8_t> block,
                         const std::atomic<bool> &abort_requested,
                         const TransportWriteStep &write_step,
                         std::string &write_error) {
    std::size_t offset = 0;
    while (offset < block.size()) {
        if (abort_requested.load(std::memory_order_relaxed)) {
            return false;
        }
        auto step = write_step(block.subspan(offset));
        switch (step.kind) {
        case TransportWriteStepKind::progress:
            if (step.bytes == 0 || step.bytes > block.size() - offset) {
                write_error = "Transport writer reported invalid progress";
                return false;
            }
            offset += step.bytes;
            break;
        case TransportWriteStepKind::retry:
            break;
        case TransportWriteStepKind::failure:
            write_error = std::move(step.error);
            if (write_error.empty()) {
                write_error = "Transport writer failed without an error";
            }
            return false;
        }
    }
    return true;
}

struct TransportWriterCore::Impl {
    struct Entry {
        std::vector<std::uint8_t> data;
        std::uint64_t logical_blocks{1};
    };

    Impl(TransportOutputConfig selected, TransportWriteStep selected_write_step)
        : config(std::move(selected)), write_step(std::move(selected_write_step)) {
        if (config.queue_capacity_bytes == 0) {
            config.queue_capacity_bytes = 1;
        }
        config.write_batch_bytes =
            std::min(config.write_batch_bytes, config.queue_capacity_bytes);
        config.write_batch_delay =
            std::max(config.write_batch_delay, std::chrono::microseconds{0});
    }

    void fail_locked(std::string message) {
        failed = true;
        accepting = false;
        stop_requested = true;
        abort_writes.store(true, std::memory_order_relaxed);
        error = std::move(message);
        space_available.notify_all();
    }

    void run() {
        set_current_thread_name(config.thread_name);
        while (true) {
            Entry block;
            {
                std::unique_lock lock(mutex);
                ready.wait(lock,
                           [this] { return stop_requested || !queue.empty(); });
                if (queue.empty() && stop_requested) {
                    break;
                }
                if (!stop_requested && queue.size() == 1 &&
                    config.write_batch_bytes != 0 &&
                    queue.front().data.size() < config.write_batch_bytes &&
                    config.write_batch_delay.count() != 0) {
                    ready.wait_for(lock, config.write_batch_delay, [this] {
                        return stop_requested || queue.size() != 1 ||
                               queue.front().data.size() >=
                                   config.write_batch_bytes;
                    });
                }
                if (queue.empty()) {
                    if (stop_requested) {
                        break;
                    }
                    continue;
                }
                block = std::move(queue.front());
                queue.pop_front();
                queued_bytes -= block.data.size();
                in_flight_bytes = block.data.size();
                in_flight_blocks = block.logical_blocks;
                space_available.notify_all();
            }

            std::string write_error;
            if (!transport_write_all(block.data, abort_writes, write_step,
                                     write_error)) {
                const std::scoped_lock lock(mutex);
                in_flight_bytes = 0;
                in_flight_blocks = 0;
                dropped_blocks += block.logical_blocks;
                dropped_bytes += block.data.size();
                if (write_error.empty()) {
                    break;
                }
                ++write_errors;
                error = write_error;
                if (config.write_error_policy ==
                    TransportWriteErrorPolicy::drop_block) {
                    continue;
                }
                fail_locked(std::move(write_error));
                for (const auto &queued : queue) {
                    dropped_bytes += queued.data.size();
                    dropped_blocks += queued.logical_blocks;
                }
                queue.clear();
                queued_bytes = 0;
                space_available.notify_all();
                break;
            }
            {
                const std::scoped_lock lock(mutex);
                in_flight_bytes = 0;
                in_flight_blocks = 0;
                bytes_written += block.data.size();
                blocks_written += block.logical_blocks;
            }
        }
    }

    TransportOutputConfig config;
    TransportWriteStep write_step;
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::condition_variable space_available;
    std::deque<Entry> queue;
    std::thread worker;
    std::chrono::steady_clock::time_point started_at;
    std::size_t queued_bytes{};
    std::size_t in_flight_bytes{};
    std::uint64_t in_flight_blocks{};
    bool accepting{};
    bool stop_requested{};
    bool failed{};
    std::string error;
    std::uint64_t elapsed_milliseconds{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t bytes_written{};
    std::uint64_t blocks_written{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::uint64_t write_errors{};
    std::atomic<bool> abort_writes{};
};

TransportWriterCore::TransportWriterCore(TransportOutputConfig config,
                                         TransportWriteStep write_step)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(write_step))) {}

TransportWriterCore::~TransportWriterCore() noexcept { stop(false); }

bool TransportWriterCore::start(std::string &error) {
    {
        const std::scoped_lock lock(impl_->mutex);
        if (impl_->worker.joinable() || impl_->accepting) {
            error = "Transport writer is already active";
            return false;
        }
        impl_->queue.clear();
        impl_->queued_bytes = 0;
        impl_->in_flight_bytes = 0;
        impl_->in_flight_blocks = 0;
        impl_->accepting = true;
        impl_->stop_requested = false;
        impl_->failed = false;
        impl_->error.clear();
        impl_->started_at = std::chrono::steady_clock::now();
        impl_->elapsed_milliseconds = 0;
        impl_->blocks_accepted = 0;
        impl_->bytes_accepted = 0;
        impl_->bytes_written = 0;
        impl_->blocks_written = 0;
        impl_->dropped_blocks = 0;
        impl_->dropped_bytes = 0;
        impl_->write_errors = 0;
        impl_->abort_writes.store(false, std::memory_order_relaxed);
    }
    try {
        impl_->worker = std::thread([this] { impl_->run(); });
    } catch (const std::exception &exception) {
        error = "Unable to start transport output worker: " +
                std::string(exception.what());
        const std::scoped_lock lock(impl_->mutex);
        impl_->accepting = false;
        return false;
    }
    return true;
}

bool TransportWriterCore::submit(
    const std::span<const std::uint8_t> transport_stream) noexcept {
    if (transport_stream.empty()) {
        return true;
    }
    std::unique_lock lock(impl_->mutex);
    if (!impl_->accepting) {
        return false;
    }

    const auto drop_block = [this, &transport_stream] {
        ++impl_->dropped_blocks;
        impl_->dropped_bytes += transport_stream.size();
    };
    if (transport_stream.size() > impl_->config.queue_capacity_bytes) {
        drop_block();
        if (impl_->config.overflow_policy ==
            TransportOverflowPolicy::fail_sink) {
            impl_->fail_locked("Transport output block exceeds queue capacity");
            impl_->ready.notify_one();
        }
        return false;
    }

    if (impl_->config.overflow_policy ==
        TransportOverflowPolicy::block_producer) {
        impl_->space_available.wait(lock, [this, &transport_stream] {
            return !impl_->accepting || impl_->stop_requested ||
                   impl_->queued_bytes + transport_stream.size() <=
                       impl_->config.queue_capacity_bytes;
        });
        if (!impl_->accepting || impl_->stop_requested) {
            return false;
        }
    }

    if (impl_->config.overflow_policy == TransportOverflowPolicy::drop_oldest) {
        while (!impl_->queue.empty() &&
               impl_->queued_bytes + transport_stream.size() >
                   impl_->config.queue_capacity_bytes) {
            impl_->queued_bytes -= impl_->queue.front().data.size();
            impl_->dropped_bytes += impl_->queue.front().data.size();
            impl_->dropped_blocks += impl_->queue.front().logical_blocks;
            impl_->queue.pop_front();
        }
    } else if (impl_->queued_bytes + transport_stream.size() >
               impl_->config.queue_capacity_bytes) {
        drop_block();
        if (impl_->config.overflow_policy ==
            TransportOverflowPolicy::fail_sink) {
            impl_->fail_locked("Transport output queue overflow");
            impl_->ready.notify_one();
        }
        return false;
    }

    const bool queue_was_empty = impl_->queue.empty();
    bool batch_reached_target = false;
    if (impl_->config.write_batch_bytes != 0 && !impl_->queue.empty() &&
        impl_->queue.back().data.size() + transport_stream.size() <=
            impl_->config.write_batch_bytes) {
        auto &batch = impl_->queue.back();
        batch.data.insert(batch.data.end(), transport_stream.begin(),
                          transport_stream.end());
        ++batch.logical_blocks;
        batch_reached_target =
            batch.data.size() >= impl_->config.write_batch_bytes;
    } else {
        impl_->queue.push_back(
            {.data = {transport_stream.begin(), transport_stream.end()},
             .logical_blocks = 1});
    }
    impl_->queued_bytes += transport_stream.size();
    ++impl_->blocks_accepted;
    impl_->bytes_accepted += transport_stream.size();
    if (queue_was_empty || batch_reached_target) {
        impl_->ready.notify_one();
    }
    return true;
}

void TransportWriterCore::discard_queued() noexcept {
    const std::scoped_lock lock(impl_->mutex);
    for (const auto &block : impl_->queue) {
        impl_->dropped_bytes += block.data.size();
        impl_->dropped_blocks += block.logical_blocks;
    }
    impl_->queue.clear();
    impl_->queued_bytes = 0;
    impl_->space_available.notify_all();
}

void TransportWriterCore::stop(const bool drain) noexcept {
    if (!impl_->worker.joinable()) {
        return;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->accepting = false;
        impl_->stop_requested = true;
        if (!drain) {
            impl_->abort_writes.store(true, std::memory_order_relaxed);
            for (const auto &block : impl_->queue) {
                impl_->dropped_bytes += block.data.size();
                impl_->dropped_blocks += block.logical_blocks;
            }
            impl_->queue.clear();
            impl_->queued_bytes = 0;
        }
    }
    impl_->ready.notify_one();
    impl_->space_available.notify_all();
    impl_->worker.join();
    const std::scoped_lock lock(impl_->mutex);
    impl_->elapsed_milliseconds = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->started_at)
            .count());
}

TransportOutputStats TransportWriterCore::stats() const {
    const std::scoped_lock lock(impl_->mutex);
    std::uint64_t elapsed = impl_->elapsed_milliseconds;
    if (impl_->accepting) {
        elapsed = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - impl_->started_at)
                .count());
    }
    return {
        .active = impl_->accepting,
        .failed = impl_->failed,
        .required = impl_->config.criticality ==
                    TransportSinkCriticality::required,
        .elapsed_milliseconds = elapsed,
        .blocks_accepted = impl_->blocks_accepted,
        .bytes_accepted = impl_->bytes_accepted,
        .bytes_written = impl_->bytes_written,
        .blocks_written = impl_->blocks_written,
        .dropped_blocks = impl_->dropped_blocks,
        .dropped_bytes = impl_->dropped_bytes,
        .write_errors = impl_->write_errors,
        .queued_bytes = impl_->queued_bytes + impl_->in_flight_bytes,
        .queue_capacity_bytes = impl_->config.queue_capacity_bytes,
        .error = impl_->error,
    };
}

bool TransportWriterCore::abort_requested() const noexcept {
    return impl_->abort_writes.load(std::memory_order_relaxed);
}

} // namespace airspy_tv::detail
