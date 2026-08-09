#pragma once

#include "airspy_tv/transport_telemetry.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>

namespace airspy_tv {

enum class TransportOverflowPolicy {
    fail_sink,
    drop_newest,
    drop_oldest,
    block_producer,
};
enum class TransportSinkCriticality { optional, required };
enum class TransportWriteErrorPolicy { fail_sink, drop_block };

struct TransportOutputConfig {
    std::size_t queue_capacity_bytes{8U << 20U};
    TransportOverflowPolicy overflow_policy{TransportOverflowPolicy::fail_sink};
    TransportSinkCriticality criticality{TransportSinkCriticality::required};
    TransportWriteErrorPolicy write_error_policy{
        TransportWriteErrorPolicy::fail_sink};
    std::string thread_name{"ts-output"};
};

struct TransportOutputStats {
    bool active{};
    bool failed{};
    bool required{};
    std::uint64_t elapsed_milliseconds{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t bytes_written{};
    std::uint64_t blocks_written{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::uint64_t write_errors{};
    std::size_t queued_bytes{};
    std::size_t queue_capacity_bytes{};
    std::string error;
};

[[nodiscard]] TransportOutputTelemetry
transport_output_telemetry(std::string name, std::string type,
                           const TransportOutputStats &stats);

// Bounded asynchronous byte sink for ordered MPEG-TS output. submit() never
// performs I/O and is safe to call from the decoder transport callback.
class AsyncTransportOutput {
  public:
    explicit AsyncTransportOutput(TransportOutputConfig config = {});
    ~AsyncTransportOutput() noexcept;

    AsyncTransportOutput(const AsyncTransportOutput &) = delete;
    AsyncTransportOutput &operator=(const AsyncTransportOutput &) = delete;
    AsyncTransportOutput(AsyncTransportOutput &&) = delete;
    AsyncTransportOutput &operator=(AsyncTransportOutput &&) = delete;

    bool start_file(const std::filesystem::path &path, std::string &error);
    bool start_fd(int fd, bool close_fd, std::string descriptor,
                  std::string &error);
    bool submit(std::span<const std::uint8_t> transport_stream) noexcept;
    void discard_queued() noexcept;
    void stop(bool drain = true) noexcept;

    [[nodiscard]] TransportOutputStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
