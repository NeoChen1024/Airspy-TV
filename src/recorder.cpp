#include "airspy_tv/recorder.hpp"
#include "airspy_tv/transport_output.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

constexpr std::size_t recorder_buffer_seconds = 5;
// 24 MiB holds more than five seconds at DVB-T's maximum useful
// transport-stream rate while keeping the capacity independent of callback
// block boundaries.

} // namespace

struct RawIqRecorder::Impl {
    explicit Impl(RawIqRecorderConfig selected) : config(selected) {}

    RawIqRecorderConfig config;
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::deque<std::vector<std::int16_t>> queue;
    std::ofstream output;
    std::thread worker;
    std::filesystem::path path;
    RecordingMetadata metadata;
    std::chrono::steady_clock::time_point started_at;
    std::size_t queued_complex_samples{};
    std::size_t queue_capacity_samples{};
    bool stopping{};
    bool failed{};
    std::string error;
    std::atomic<bool> active;
    std::atomic<std::uint64_t> elapsed_milliseconds;
    std::atomic<std::uint64_t> complex_samples;
    std::atomic<std::uint64_t> bytes_written;
    std::atomic<std::uint64_t> dropped_blocks;
    std::atomic<std::uint64_t> source_dropped_samples;
    std::atomic<std::uint64_t> write_errors;

    void fail(std::string message) {
        const std::scoped_lock lock(mutex);
        failed = true;
        error = std::move(message);
        active = false;
        stopping = true;
        ++write_errors;
        dropped_blocks += queue.size();
        queue.clear();
        queued_complex_samples = 0;
    }

    void run() {
        while (true) {
            std::vector<std::int16_t> block;
            {
                std::unique_lock lock(mutex);
                ready.wait(lock, [this] { return stopping || !queue.empty(); });
                if (queue.empty() && stopping) {
                    break;
                }
                block = std::move(queue.front());
                queue.pop_front();
                queued_complex_samples -= block.size() / 2;
            }

            const auto byte_count = static_cast<std::streamsize>(
                block.size() * sizeof(std::int16_t));
            output.write(reinterpret_cast<const char *>(block.data()),
                         byte_count);
            if (!output) {
                fail("Unable to write raw I/Q recording: " + path.string());
                break;
            }
            bytes_written += static_cast<std::uint64_t>(byte_count);
            complex_samples += static_cast<std::uint64_t>(block.size() / 2);
        }
        if (!failed) {
            output.flush();
            if (!output) {
                fail("Unable to flush raw I/Q recording: " + path.string());
            }
        }
    }

    void write_sidecar() const noexcept {
        try {
            std::ofstream sidecar(path.string() + ".json", std::ios::trunc);
            if (!sidecar) {
                return;
            }
            const nlohmann::json metadata_json{
                {"data_file", path.filename().string()},
                {"datatype", "ci16_le"},
                {"iq_order", "IQ"},
                {"sample_rate", metadata.sample_rate_hz},
                {"center_frequency", metadata.center_frequency_hz},
                {"source", metadata.source},
                {"complex_samples", complex_samples.load()},
                {"duration_ms", elapsed_milliseconds.load()},
                {"dropped_blocks", dropped_blocks.load()},
                {"source_dropped_samples", source_dropped_samples.load()},
                {"write_errors", write_errors.load()},
                {"failed", failed},
                {"error", error},
            };
            sidecar << metadata_json.dump(2) << '\n';
        } catch (...) { // NOLINT(bugprone-empty-catch)
            // Recording shutdown must remain noexcept if metadata allocation
            // fails.
        }
    }
};

RawIqRecorder::RawIqRecorder(RawIqRecorderConfig config)
    : impl_(std::make_unique<Impl>(config)) {}

RawIqRecorder::~RawIqRecorder() noexcept { stop(); }

bool RawIqRecorder::start(const std::filesystem::path &path,
                          RecordingMetadata metadata, std::string &error) {
    stop();
    if (path.empty()) {
        error = "Recording path is empty";
        return false;
    }

    impl_->output.clear();
    impl_->output.open(path, std::ios::binary | std::ios::trunc);
    if (!impl_->output) {
        error = "Unable to open recording file: " + path.string();
        return false;
    }

    impl_->path = path;
    impl_->metadata = std::move(metadata);
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->stopping = false;
        impl_->failed = false;
        impl_->error.clear();
        impl_->queue.clear();
        impl_->queued_complex_samples = 0;
        impl_->queue_capacity_samples = std::max<std::size_t>(
            1, impl_->config.queue_capacity_samples == 0
                   ? static_cast<std::size_t>(impl_->metadata.sample_rate_hz) *
                         recorder_buffer_seconds
                   : impl_->config.queue_capacity_samples);
    }
    impl_->started_at = std::chrono::steady_clock::now();
    impl_->complex_samples = 0;
    impl_->elapsed_milliseconds = 0;
    impl_->bytes_written = 0;
    impl_->dropped_blocks = 0;
    impl_->source_dropped_samples = 0;
    impl_->write_errors = 0;
    impl_->active = true;
    try {
        impl_->worker = std::thread([this] { impl_->run(); });
    } catch (const std::exception &exception) {
        impl_->active = false;
        impl_->output.close();
        error = "Unable to start raw I/Q recorder worker: " +
                std::string(exception.what());
        return false;
    }
    return true;
}

void RawIqRecorder::submit(std::span<const std::int16_t> interleaved_iq) {
    if (!impl_->active || interleaved_iq.empty()) {
        return;
    }

    const std::scoped_lock lock(impl_->mutex);
    if (!impl_->active) {
        return;
    }
    const std::size_t incoming_samples = interleaved_iq.size() / 2;
    if (impl_->queued_complex_samples + incoming_samples >
        impl_->queue_capacity_samples) {
        ++impl_->dropped_blocks;
        return;
    }
    impl_->queue.emplace_back(interleaved_iq.begin(), interleaved_iq.end());
    impl_->queued_complex_samples += incoming_samples;
    impl_->ready.notify_one();
}

void RawIqRecorder::add_source_dropped_samples(const std::uint64_t count) {
    impl_->source_dropped_samples += count;
}

void RawIqRecorder::stop() noexcept {
    if (!impl_->worker.joinable()) {
        return;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->stopping = true;
        impl_->active = false;
    }
    impl_->ready.notify_one();
    impl_->worker.join();
    impl_->elapsed_milliseconds = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->started_at)
            .count());
    impl_->output.close();
    impl_->write_sidecar();
    impl_->queue.clear();
    impl_->queued_complex_samples = 0;
}

RecordingStats RawIqRecorder::stats() const {
    const std::scoped_lock lock(impl_->mutex);
    std::uint64_t elapsed_milliseconds = impl_->elapsed_milliseconds;
    if (impl_->active) {
        elapsed_milliseconds = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - impl_->started_at)
                .count());
    }
    return {
        .active = impl_->active,
        .failed = impl_->failed,
        .elapsed_milliseconds = elapsed_milliseconds,
        .complex_samples = impl_->complex_samples,
        .bytes_written = impl_->bytes_written,
        .dropped_blocks = impl_->dropped_blocks,
        .source_dropped_samples = impl_->source_dropped_samples,
        .write_errors = impl_->write_errors,
        .error = impl_->error,
    };
}

struct TransportStreamRecorder::Impl {
    AsyncTransportOutput output{TransportOutputConfig{
        .queue_capacity_bytes = 24U << 20U,
        .overflow_policy = TransportOverflowPolicy::drop_oldest,
        .criticality = TransportSinkCriticality::optional,
        .write_batch_bytes = 256U << 10U,
        .write_batch_delay = std::chrono::milliseconds(2),
        .thread_name = "ts-recorder",
    }};
};

TransportStreamRecorder::TransportStreamRecorder()
    : impl_(std::make_unique<Impl>()) {}

TransportStreamRecorder::~TransportStreamRecorder() noexcept { stop(); }

bool TransportStreamRecorder::start(const std::filesystem::path &path,
                                    std::string &error) {
    if (path.empty()) {
        error = "Transport-stream recording path is empty";
        return false;
    }
    return impl_->output.start_file(path, error);
}

void TransportStreamRecorder::submit(
    const std::span<const std::uint8_t> transport_stream) {
    static_cast<void>(impl_->output.submit(transport_stream));
}

void TransportStreamRecorder::discard_queued() noexcept {
    impl_->output.discard_queued();
}

void TransportStreamRecorder::stop() noexcept { impl_->output.stop(); }

TransportRecordingStats TransportStreamRecorder::stats() const {
    const auto stats = impl_->output.stats();
    return {.active = stats.active,
            .failed = stats.failed,
            .elapsed_milliseconds = stats.elapsed_milliseconds,
            .blocks_accepted = stats.blocks_accepted,
            .bytes_written = stats.bytes_written,
            .bytes_accepted = stats.bytes_accepted,
            .blocks_written = stats.blocks_written,
            .dropped_blocks = stats.dropped_blocks,
            .dropped_bytes = stats.dropped_bytes,
            .write_errors = stats.write_errors,
            .queued_bytes = stats.queued_bytes,
            .queue_capacity_bytes = stats.queue_capacity_bytes,
            .error = stats.error};
}

} // namespace airspy_tv
