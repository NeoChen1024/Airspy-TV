#include "airspy_tv/transport_observer.hpp"

#include "airspy_tv/thread_name.hpp"

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <ranges>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv {

struct AsyncTransportObserver::Impl {
    struct Entry {
        std::vector<std::uint8_t> data;
        TransportDiscontinuity discontinuity{};
        bool control{};
    };

    Impl(Consume selected_consume, Discontinuity selected_discontinuity,
         TransportObserverConfig selected_config)
        : consume(std::move(selected_consume)),
          discontinuity(std::move(selected_discontinuity)),
          config(std::move(selected_config)) {
        if (!consume || !discontinuity) {
            throw std::invalid_argument(
                "incomplete transport observer callbacks");
        }
        config.queue_capacity_bytes =
            std::max<std::size_t>(1, config.queue_capacity_bytes);
        active = true;
        worker = std::thread([this] { run(); });
    }

    void record_drop(const std::vector<std::uint8_t> &data) noexcept {
        ++dropped_blocks;
        dropped_bytes += data.size();
        queued_bytes -= data.size();
    }

    void insert_local_gap_locked() {
        const auto already_pending =
            std::ranges::any_of(queue, [](const Entry &entry) {
                return entry.control &&
                       entry.discontinuity ==
                           TransportDiscontinuity::fec_region_reset;
            });
        if (!already_pending) {
            queue.push_front(
                {.data = {},
                 .discontinuity = TransportDiscontinuity::fec_region_reset,
                 .control = true});
        }
    }

    void discard_data_locked() noexcept {
        for (const auto &entry : queue) {
            if (!entry.control) {
                ++dropped_blocks;
                dropped_bytes += entry.data.size();
            }
        }
        queue.clear();
        queued_bytes = 0;
    }

    void fail(std::string message) noexcept {
        const std::scoped_lock lock(mutex);
        failed = true;
        active = false;
        stopping = true;
        error = std::move(message);
        discard_data_locked();
        ready.notify_all();
    }

    void run() noexcept {
        set_current_thread_name(config.thread_name);
        try {
            while (true) {
                Entry entry;
                {
                    std::unique_lock lock(mutex);
                    ready.wait(lock,
                               [this] { return stopping || !queue.empty(); });
                    if (queue.empty() && stopping) {
                        break;
                    }
                    entry = std::move(queue.front());
                    queue.pop_front();
                    if (!entry.control) {
                        queued_bytes -= entry.data.size();
                    }
                }
                if (entry.control) {
                    discontinuity(entry.discontinuity);
                } else {
                    consume(entry.data);
                    const std::scoped_lock lock(mutex);
                    ++blocks_processed;
                    bytes_processed += entry.data.size();
                }
            }
        } catch (const std::exception &exception) {
            fail(exception.what());
        } catch (...) {
            fail("transport observer callback failed");
        }
    }

    Consume consume;
    Discontinuity discontinuity;
    TransportObserverConfig config;
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::deque<Entry> queue;
    std::thread worker;
    std::size_t queued_bytes{};
    bool active{};
    bool stopping{};
    bool failed{};
    std::string error;
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_processed{};
    std::uint64_t bytes_processed{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
};

AsyncTransportObserver::AsyncTransportObserver(Consume consume,
                                               Discontinuity discontinuity,
                                               TransportObserverConfig config)
    : impl_(std::make_unique<Impl>(std::move(consume), std::move(discontinuity),
                                   std::move(config))) {}

AsyncTransportObserver::~AsyncTransportObserver() noexcept { stop(false); }

bool AsyncTransportObserver::submit(
    const std::span<const std::uint8_t> transport_stream) noexcept {
    if (transport_stream.empty()) {
        return true;
    }
    const std::scoped_lock lock(impl_->mutex);
    if (!impl_->active || impl_->stopping) {
        return false;
    }
    const std::size_t effective_capacity =
        std::max(impl_->config.queue_capacity_bytes, transport_stream.size());
    bool dropped = false;
    while (impl_->queued_bytes + transport_stream.size() >
           effective_capacity) {
        const auto data =
            std::ranges::find_if(impl_->queue, [](const Impl::Entry &entry) {
                return !entry.control;
            });
        if (data == impl_->queue.end()) {
            break;
        }
        impl_->record_drop(data->data);
        impl_->queue.erase(data);
        dropped = true;
    }
    if (dropped) {
        impl_->insert_local_gap_locked();
    }
    impl_->queue.push_back(
        {.data = {transport_stream.begin(), transport_stream.end()},
         .discontinuity = {},
         .control = false});
    impl_->queued_bytes += transport_stream.size();
    ++impl_->blocks_accepted;
    impl_->bytes_accepted += transport_stream.size();
    impl_->ready.notify_one();
    return true;
}

void AsyncTransportObserver::notify_discontinuity(
    const TransportDiscontinuity discontinuity) noexcept {
    const std::scoped_lock lock(impl_->mutex);
    if (!impl_->active || impl_->stopping) {
        return;
    }
    if (discontinuity == TransportDiscontinuity::retune) {
        impl_->discard_data_locked();
    }
    impl_->queue.push_back(
        {.data = {}, .discontinuity = discontinuity, .control = true});
    impl_->ready.notify_one();
}

void AsyncTransportObserver::stop(const bool drain) noexcept {
    if (!impl_->worker.joinable()) {
        return;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->active = false;
        impl_->stopping = true;
        if (!drain) {
            impl_->discard_data_locked();
        }
    }
    impl_->ready.notify_all();
    impl_->worker.join();
}

TransportObserverStats AsyncTransportObserver::stats() const {
    const std::scoped_lock lock(impl_->mutex);
    return {
        .active = impl_->active,
        .failed = impl_->failed,
        .blocks_accepted = impl_->blocks_accepted,
        .bytes_accepted = impl_->bytes_accepted,
        .blocks_processed = impl_->blocks_processed,
        .bytes_processed = impl_->bytes_processed,
        .dropped_blocks = impl_->dropped_blocks,
        .dropped_bytes = impl_->dropped_bytes,
        .queued_bytes = impl_->queued_bytes,
        .queue_capacity_bytes = impl_->config.queue_capacity_bytes,
        .error = impl_->error,
    };
}

} // namespace airspy_tv
