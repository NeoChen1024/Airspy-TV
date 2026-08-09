#include "airspy_tv/transport_output.hpp"

#include "airspy_tv/thread_name.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <string_view>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

[[nodiscard]] std::string system_error(const std::string_view operation) {
    return std::string(operation) + ": " + std::strerror(errno);
}

} // namespace

struct AsyncTransportOutput::Impl {
    explicit Impl(TransportOutputConfig selected)
        : config(std::move(selected)) {
        if (config.queue_capacity_bytes == 0) {
            config.queue_capacity_bytes = 1;
        }
    }

    ~Impl() { close_output(); }

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;

    void close_output() noexcept {
        if (fd >= 0 && restore_fd_flags) {
            static_cast<void>(::fcntl(fd, F_SETFL, original_fd_flags));
        }
        if (fd >= 0 && close_fd) {
            static_cast<void>(::close(fd));
        }
        fd = -1;
        close_fd = false;
        original_fd_flags = -1;
        restore_fd_flags = false;
    }

    void fail_locked(std::string message) {
        failed = true;
        accepting = false;
        stop_requested = true;
        abort_writes.store(true, std::memory_order_relaxed);
        error = std::move(message);
        space_available.notify_all();
    }

    [[nodiscard]] bool write_all(const std::span<const std::uint8_t> block,
                                 std::string &write_error) const {
        std::size_t offset = 0;
        while (offset < block.size()) {
            if (abort_writes.load(std::memory_order_relaxed)) {
                return false;
            }
            const ssize_t count =
                ::write(fd, block.data() + offset, block.size() - offset);
            if (count > 0) {
                offset += static_cast<std::size_t>(count);
                continue;
            }
            if (count < 0 && errno == EINTR) {
                continue;
            }
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd output_fd{.fd = fd, .events = POLLOUT, .revents = 0};
                int poll_result = 0;
                do {
                    poll_result = ::poll(&output_fd, 1, 100);
                } while (poll_result < 0 && errno == EINTR &&
                         !abort_writes.load(std::memory_order_relaxed));
                if (abort_writes.load(std::memory_order_relaxed)) {
                    return false;
                }
                if (poll_result >= 0) {
                    continue;
                }
            }
            write_error = system_error("Unable to write " + descriptor);
            return false;
        }
        return true;
    }

    void run() {
        set_current_thread_name(config.thread_name);
        while (true) {
            std::vector<std::uint8_t> block;
            {
                std::unique_lock lock(mutex);
                ready.wait(lock,
                           [this] { return stop_requested || !queue.empty(); });
                if (queue.empty() && stop_requested) {
                    break;
                }
                block = std::move(queue.front());
                queue.pop_front();
                queued_bytes -= block.size();
                in_flight_bytes = block.size();
                space_available.notify_all();
            }

            std::string write_error;
            if (!write_all(block, write_error)) {
                const std::scoped_lock lock(mutex);
                in_flight_bytes = 0;
                if (write_error.empty()) {
                    ++dropped_blocks;
                    dropped_bytes += block.size();
                    break;
                }
                ++write_errors;
                if (config.write_error_policy ==
                    TransportWriteErrorPolicy::drop_block) {
                    ++dropped_blocks;
                    dropped_bytes += block.size();
                    error = std::move(write_error);
                    continue;
                }
                fail_locked(std::move(write_error));
                for (const auto &queued : queue) {
                    dropped_bytes += queued.size();
                }
                dropped_blocks += queue.size();
                queue.clear();
                queued_bytes = 0;
                space_available.notify_all();
                break;
            }
            {
                const std::scoped_lock lock(mutex);
                in_flight_bytes = 0;
            }
            bytes_written += block.size();
            ++blocks_written;
        }
    }

    TransportOutputConfig config;
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::condition_variable space_available;
    std::deque<std::vector<std::uint8_t>> queue;
    std::thread worker;
    std::chrono::steady_clock::time_point started_at;
    int fd{-1};
    bool close_fd{};
    int original_fd_flags{-1};
    bool restore_fd_flags{};
    std::string descriptor;
    std::size_t queued_bytes{};
    std::size_t in_flight_bytes{};
    bool accepting{};
    bool stop_requested{};
    bool failed{};
    std::string error;
    std::atomic<std::uint64_t> elapsed_milliseconds;
    std::atomic<std::uint64_t> blocks_accepted;
    std::atomic<std::uint64_t> bytes_accepted;
    std::atomic<std::uint64_t> bytes_written;
    std::atomic<std::uint64_t> blocks_written;
    std::atomic<std::uint64_t> dropped_blocks;
    std::atomic<std::uint64_t> dropped_bytes;
    std::atomic<std::uint64_t> write_errors;
    std::atomic<bool> abort_writes;
};

AsyncTransportOutput::AsyncTransportOutput(TransportOutputConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

AsyncTransportOutput::~AsyncTransportOutput() noexcept { stop(); }

bool AsyncTransportOutput::start_file(const std::filesystem::path &path,
                                      std::string &error) {
    stop();
    if (path.empty()) {
        error = "Transport output path is empty";
        return false;
    }
    const int fd =
        ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC,
               S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd < 0) {
        error =
            system_error("Unable to open transport output " + path.string());
        return false;
    }
    if (!start_fd(fd, true, path.string(), error)) {
        return false;
    }
    return true;
}

bool AsyncTransportOutput::start_fd(const int fd, const bool close_fd,
                                    std::string descriptor,
                                    std::string &error) {
    stop();
    if (fd < 0) {
        error = "Transport output file descriptor is invalid";
        return false;
    }
    const int original_flags = ::fcntl(fd, F_GETFL);
    if (original_flags < 0 ||
        ::fcntl(fd, F_SETFL, original_flags | O_NONBLOCK) < 0) {
        error =
            system_error("Unable to configure nonblocking transport output");
        if (close_fd) {
            static_cast<void>(::close(fd));
        }
        return false;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->fd = fd;
        impl_->close_fd = close_fd;
        impl_->original_fd_flags = original_flags;
        impl_->restore_fd_flags = !close_fd;
        impl_->descriptor = std::move(descriptor);
        impl_->queue.clear();
        impl_->queued_bytes = 0;
        impl_->in_flight_bytes = 0;
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
        impl_->close_output();
        return false;
    }
    return true;
}

bool AsyncTransportOutput::submit(
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
            impl_->queued_bytes -= impl_->queue.front().size();
            impl_->dropped_bytes += impl_->queue.front().size();
            ++impl_->dropped_blocks;
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

    impl_->queue.emplace_back(transport_stream.begin(), transport_stream.end());
    impl_->queued_bytes += transport_stream.size();
    ++impl_->blocks_accepted;
    impl_->bytes_accepted += transport_stream.size();
    impl_->ready.notify_one();
    return true;
}

void AsyncTransportOutput::discard_queued() noexcept {
    const std::scoped_lock lock(impl_->mutex);
    for (const auto &block : impl_->queue) {
        impl_->dropped_bytes += block.size();
    }
    impl_->dropped_blocks += impl_->queue.size();
    impl_->queue.clear();
    impl_->queued_bytes = 0;
    impl_->space_available.notify_all();
}

void AsyncTransportOutput::stop(const bool drain) noexcept {
    if (!impl_->worker.joinable()) {
        impl_->close_output();
        return;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->accepting = false;
        impl_->stop_requested = true;
        if (!drain) {
            impl_->abort_writes.store(true, std::memory_order_relaxed);
            for (const auto &block : impl_->queue) {
                impl_->dropped_bytes += block.size();
            }
            impl_->dropped_blocks += impl_->queue.size();
            impl_->queue.clear();
            impl_->queued_bytes = 0;
        }
    }
    impl_->ready.notify_one();
    impl_->space_available.notify_all();
    impl_->worker.join();
    impl_->elapsed_milliseconds = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->started_at)
            .count());
    impl_->close_output();
}

TransportOutputStats AsyncTransportOutput::stats() const {
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
        .required =
            impl_->config.criticality == TransportSinkCriticality::required,
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

TransportOutputTelemetry
transport_output_telemetry(std::string name, std::string type,
                           const TransportOutputStats &stats) {
    return {.name = std::move(name),
            .type = std::move(type),
            .active = stats.active,
            .required = stats.required,
            .failed = stats.failed,
            .blocks_accepted = stats.blocks_accepted,
            .bytes_accepted = stats.bytes_accepted,
            .blocks_processed = stats.blocks_written,
            .bytes_processed = stats.bytes_written,
            .dropped_blocks = stats.dropped_blocks,
            .dropped_bytes = stats.dropped_bytes,
            .errors = stats.write_errors,
            .queued_bytes = stats.queued_bytes,
            .queue_capacity_bytes = stats.queue_capacity_bytes,
            .error = stats.error};
}

} // namespace airspy_tv
