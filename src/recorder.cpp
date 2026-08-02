#include "airspy_tv/recorder.hpp"

#include <nlohmann/json.hpp>

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

constexpr std::size_t max_queued_blocks = 64;

} // namespace

struct RawIqRecorder::Impl {
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::deque<std::vector<std::int16_t>> queue;
    std::ofstream output;
    std::thread worker;
    std::filesystem::path path;
    RecordingMetadata metadata;
    std::chrono::steady_clock::time_point started_at;
    bool stopping{};
    std::atomic<bool> active;
    std::atomic<std::uint64_t> elapsed_milliseconds;
    std::atomic<std::uint64_t> complex_samples;
    std::atomic<std::uint64_t> bytes_written;
    std::atomic<std::uint64_t> dropped_blocks;
    std::atomic<std::uint64_t> source_dropped_samples;

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
            }

            const auto byte_count = static_cast<std::streamsize>(
                block.size() * sizeof(std::int16_t));
            output.write(reinterpret_cast<const char *>(block.data()),
                         byte_count);
            if (!output) {
                active = false;
                continue;
            }
            bytes_written += static_cast<std::uint64_t>(byte_count);
            complex_samples += static_cast<std::uint64_t>(block.size() / 2);
        }
        output.flush();
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
            };
            sidecar << metadata_json.dump(2) << '\n';
        } catch (...) { // NOLINT(bugprone-empty-catch)
            // Recording shutdown must remain noexcept if metadata allocation
            // fails.
        }
    }
};

RawIqRecorder::RawIqRecorder() : impl_(std::make_unique<Impl>()) {}

RawIqRecorder::~RawIqRecorder() noexcept { stop(); }

bool RawIqRecorder::start(const std::filesystem::path &path,
                          RecordingMetadata metadata, std::string &error) {
    stop();
    if (path.empty()) {
        error = "Recording path is empty";
        return false;
    }

    impl_->output.open(path, std::ios::binary | std::ios::trunc);
    if (!impl_->output) {
        error = "Unable to open recording file: " + path.string();
        return false;
    }

    impl_->path = path;
    impl_->metadata = std::move(metadata);
    impl_->stopping = false;
    impl_->started_at = std::chrono::steady_clock::now();
    impl_->complex_samples = 0;
    impl_->elapsed_milliseconds = 0;
    impl_->bytes_written = 0;
    impl_->dropped_blocks = 0;
    impl_->source_dropped_samples = 0;
    impl_->active = true;
    impl_->worker = std::thread([this] { impl_->run(); });
    return true;
}

void RawIqRecorder::submit(std::span<const std::int16_t> interleaved_iq) {
    if (!impl_->active || interleaved_iq.empty()) {
        return;
    }

    const std::scoped_lock lock(impl_->mutex);
    if (impl_->queue.size() >= max_queued_blocks) {
        ++impl_->dropped_blocks;
        return;
    }
    impl_->queue.emplace_back(interleaved_iq.begin(), interleaved_iq.end());
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
}

RecordingStats RawIqRecorder::stats() const {
    std::uint64_t elapsed_milliseconds = impl_->elapsed_milliseconds;
    if (impl_->active) {
        elapsed_milliseconds = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - impl_->started_at)
                .count());
    }
    return {
        .active = impl_->active,
        .elapsed_milliseconds = elapsed_milliseconds,
        .complex_samples = impl_->complex_samples,
        .bytes_written = impl_->bytes_written,
        .dropped_blocks = impl_->dropped_blocks,
        .source_dropped_samples = impl_->source_dropped_samples,
    };
}

struct TransportStreamRecorder::Impl {
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::deque<std::vector<std::uint8_t>> queue;
    std::ofstream output;
    std::thread worker;
    std::chrono::steady_clock::time_point started_at;
    bool stopping{};
    std::atomic<bool> active;
    std::atomic<std::uint64_t> elapsed_milliseconds;
    std::atomic<std::uint64_t> bytes_written;
    std::atomic<std::uint64_t> dropped_blocks;

    void run() {
        while (true) {
            std::vector<std::uint8_t> block;
            {
                std::unique_lock lock(mutex);
                ready.wait(lock, [this] { return stopping || !queue.empty(); });
                if (queue.empty() && stopping) {
                    break;
                }
                block = std::move(queue.front());
                queue.pop_front();
            }
            output.write(reinterpret_cast<const char *>(block.data()),
                         static_cast<std::streamsize>(block.size()));
            if (!output) {
                active = false;
                continue;
            }
            bytes_written += block.size();
        }
        output.flush();
    }
};

TransportStreamRecorder::TransportStreamRecorder()
    : impl_(std::make_unique<Impl>()) {}

TransportStreamRecorder::~TransportStreamRecorder() noexcept { stop(); }

bool TransportStreamRecorder::start(const std::filesystem::path &path,
                                    std::string &error) {
    stop();
    if (path.empty()) {
        error = "Transport-stream recording path is empty";
        return false;
    }
    impl_->output.open(path, std::ios::binary | std::ios::trunc);
    if (!impl_->output) {
        error = "Unable to open transport-stream recording: " + path.string();
        return false;
    }
    impl_->stopping = false;
    impl_->started_at = std::chrono::steady_clock::now();
    impl_->elapsed_milliseconds = 0;
    impl_->bytes_written = 0;
    impl_->dropped_blocks = 0;
    impl_->active = true;
    impl_->worker = std::thread([this] { impl_->run(); });
    return true;
}

void TransportStreamRecorder::submit(
    const std::span<const std::uint8_t> transport_stream) {
    if (!impl_->active || transport_stream.empty()) {
        return;
    }
    const std::scoped_lock lock(impl_->mutex);
    if (impl_->queue.size() >= max_queued_blocks) {
        ++impl_->dropped_blocks;
        return;
    }
    impl_->queue.emplace_back(transport_stream.begin(), transport_stream.end());
    impl_->ready.notify_one();
}

void TransportStreamRecorder::stop() noexcept {
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
    impl_->queue.clear();
}

TransportRecordingStats TransportStreamRecorder::stats() const {
    std::uint64_t elapsed = impl_->elapsed_milliseconds;
    if (impl_->active) {
        elapsed = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - impl_->started_at)
                .count());
    }
    return {.active = impl_->active,
            .elapsed_milliseconds = elapsed,
            .bytes_written = impl_->bytes_written,
            .dropped_blocks = impl_->dropped_blocks};
}

} // namespace airspy_tv
