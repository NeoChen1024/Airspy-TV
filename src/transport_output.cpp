#include "airspy_tv/transport_output.hpp"

#include "transport_write_all.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string_view>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace airspy_tv {
namespace {

[[nodiscard]] std::string system_error(const std::string_view operation) {
    return std::string(operation) + ": " + std::strerror(errno);
}

} // namespace

struct AsyncTransportOutput::Impl {
    explicit Impl(TransportOutputConfig selected)
        : config(std::move(selected)) {}

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

    TransportOutputConfig config;
    std::unique_ptr<detail::TransportWriterCore> writer;
    int fd{-1};
    bool close_fd{};
    int original_fd_flags{-1};
    bool restore_fd_flags{};
    std::string descriptor;
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
    impl_->fd = fd;
    impl_->close_fd = close_fd;
    impl_->original_fd_flags = original_flags;
    impl_->restore_fd_flags = !close_fd;
    impl_->descriptor = std::move(descriptor);
    impl_->writer = std::make_unique<detail::TransportWriterCore>(
        impl_->config, [impl = impl_.get()](
                           const std::span<const std::uint8_t> remaining) {
            const ssize_t count =
                ::write(impl->fd, remaining.data(), remaining.size());
            if (count > 0) {
                return detail::TransportWriteStepResult{
                    .kind = detail::TransportWriteStepKind::progress,
                    .bytes = static_cast<std::size_t>(count),
                    .error = {}};
            }
            if (count < 0 && errno == EINTR) {
                return detail::TransportWriteStepResult{
                    .kind = detail::TransportWriteStepKind::retry,
                    .bytes = 0,
                    .error = {}};
            }
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd output_fd{.fd = impl->fd,
                                 .events = POLLOUT,
                                 .revents = 0};
                int poll_result = 0;
                do {
                    poll_result = ::poll(&output_fd, 1, 100);
                } while (poll_result < 0 && errno == EINTR && impl->writer &&
                         !impl->writer->abort_requested());
                if (impl->writer && impl->writer->abort_requested()) {
                    return detail::TransportWriteStepResult{
                        .kind = detail::TransportWriteStepKind::retry,
                        .bytes = 0,
                        .error = {}};
                }
                if (poll_result >= 0) {
                    return detail::TransportWriteStepResult{
                        .kind = detail::TransportWriteStepKind::retry,
                        .bytes = 0,
                        .error = {}};
                }
            }
            return detail::TransportWriteStepResult{
                .kind = detail::TransportWriteStepKind::failure,
                .error = system_error("Unable to write " + impl->descriptor)};
        });
    if (!impl_->writer->start(error)) {
        impl_->writer.reset();
        impl_->close_output();
        return false;
    }
    return true;
}

bool AsyncTransportOutput::submit(
    const std::span<const std::uint8_t> transport_stream) noexcept {
    return impl_->writer && impl_->writer->submit(transport_stream);
}

void AsyncTransportOutput::discard_queued() noexcept {
    if (impl_->writer) {
        impl_->writer->discard_queued();
    }
}

void AsyncTransportOutput::stop(const bool drain) noexcept {
    if (impl_->writer) {
        impl_->writer->stop(drain);
    }
    impl_->close_output();
}

TransportOutputStats AsyncTransportOutput::stats() const {
    if (impl_->writer) {
        return impl_->writer->stats();
    }
    return {.active = false,
            .failed = false,
            .required = impl_->config.criticality ==
                        TransportSinkCriticality::required,
            .elapsed_milliseconds = 0,
            .blocks_accepted = 0,
            .bytes_accepted = 0,
            .bytes_written = 0,
            .blocks_written = 0,
            .dropped_blocks = 0,
            .dropped_bytes = 0,
            .write_errors = 0,
            .queued_bytes = 0,
            .queue_capacity_bytes =
                std::max<std::size_t>(1, impl_->config.queue_capacity_bytes),
            .error = {}};
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
