#pragma once

#include "airspy_tv/transport_output.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>

namespace airspy_tv::detail {

enum class TransportWriteStepKind { progress, retry, failure };

struct TransportWriteStepResult {
    TransportWriteStepKind kind{TransportWriteStepKind::failure};
    std::size_t bytes{};
    std::string error;
};

using TransportWriteStep = std::function<TransportWriteStepResult(
    std::span<const std::uint8_t>)>;

// Complete one logical output block through a backend that may make partial
// progress or ask to retry. An empty error means cancellation, not I/O failure.
[[nodiscard]] bool
transport_write_all(std::span<const std::uint8_t> block,
                    const std::atomic<bool> &abort_requested,
                    const TransportWriteStep &write_step,
                    std::string &write_error);

// Internal asynchronous queue worker. The production fd adapter and tests both
// supply the same single-step writer seam, so retries, queue accounting, drain,
// and abort semantics are exercised without replacing the worker itself.
class TransportWriterCore {
  public:
    TransportWriterCore(TransportOutputConfig config,
                        TransportWriteStep write_step);
    ~TransportWriterCore() noexcept;

    TransportWriterCore(const TransportWriterCore &) = delete;
    TransportWriterCore &operator=(const TransportWriterCore &) = delete;

    [[nodiscard]] bool start(std::string &error);
    [[nodiscard]] bool
    submit(std::span<const std::uint8_t> transport_stream) noexcept;
    void discard_queued() noexcept;
    void stop(bool drain = true) noexcept;

    [[nodiscard]] TransportOutputStats stats() const;
    [[nodiscard]] bool abort_requested() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::detail
