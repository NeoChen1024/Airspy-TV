#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace airspy_tv {

struct RecordingMetadata {
    std::string source;
    std::uint64_t center_frequency_hz{};
    std::uint32_t sample_rate_hz{};
};

struct RecordingStats {
    bool active{};
    bool failed{};
    std::uint64_t elapsed_milliseconds{};
    std::uint64_t complex_samples{};
    std::uint64_t bytes_written{};
    std::uint64_t dropped_blocks{};
    std::uint64_t source_dropped_samples{};
    std::uint64_t write_errors{};
    std::string error;
};

struct RawIqRecorderConfig {
    // Zero derives a five-second queue from the recording sample rate.
    std::size_t queue_capacity_samples{};
};

class RawIqRecorder {
  public:
    explicit RawIqRecorder(RawIqRecorderConfig config = {});
    ~RawIqRecorder() noexcept;

    RawIqRecorder(const RawIqRecorder &) = delete;
    RawIqRecorder &operator=(const RawIqRecorder &) = delete;
    RawIqRecorder(RawIqRecorder &&) = delete;
    RawIqRecorder &operator=(RawIqRecorder &&) = delete;

    bool start(const std::filesystem::path &path, RecordingMetadata metadata,
               std::string &error);
    void submit(std::span<const std::int16_t> interleaved_iq);
    void add_source_dropped_samples(std::uint64_t count);
    void stop() noexcept;

    [[nodiscard]] RecordingStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct TransportRecordingStats {
    bool active{};
    bool failed{};
    std::uint64_t elapsed_milliseconds{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_written{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_written{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::uint64_t write_errors{};
    std::size_t queued_bytes{};
    std::size_t queue_capacity_bytes{};
    std::string error;
};

class TransportStreamRecorder {
  public:
    TransportStreamRecorder();
    ~TransportStreamRecorder() noexcept;

    TransportStreamRecorder(const TransportStreamRecorder &) = delete;
    TransportStreamRecorder &
    operator=(const TransportStreamRecorder &) = delete;
    TransportStreamRecorder(TransportStreamRecorder &&) = delete;
    TransportStreamRecorder &operator=(TransportStreamRecorder &&) = delete;

    bool start(const std::filesystem::path &path, std::string &error);
    void submit(std::span<const std::uint8_t> transport_stream);
    void discard_queued() noexcept;
    void stop() noexcept;
    [[nodiscard]] TransportRecordingStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
