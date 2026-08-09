#pragma once

#include "airspy_tv/demodulator.hpp"

#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

struct InputBlock {
    std::vector<std::int16_t> samples;
    std::uint32_t rate{};
    std::uint32_t bandwidth{};
    std::uint64_t generation{};
    InputSampleStamp stamp;
};

class SampleChannel {
  public:
    enum class FrontendEvent { stop, reset, rebootstrap, close_ring, block };
    enum class PushStatus {
        written,
        stop,
        reset,
        rebootstrap,
        flush_abandoned
    };
    enum class WaitStatus { ready, stop, sync_changed, closed, timeout };

    struct FrontendWork {
        FrontendEvent event{FrontendEvent::stop};
        std::optional<InputBlock> block;
        std::uint64_t reset_generation{};
    };

    struct RebootstrapResult {
        std::uint64_t old_generation{};
        std::uint64_t new_generation{};
        std::uint64_t consumed_input_samples{};
        std::uint64_t ring_write_position{};
    };

    struct PrepareResult {
        bool accepted{};
        bool reopened{};
        std::uint64_t write_position{};
    };

    struct PushResult {
        PushStatus status{PushStatus::stop};
        std::size_t written{};
        std::uint64_t write_begin{};
        std::uint64_t write_end{};
        float wait_time_ms{};
        float copy_time_ms{};
    };

    struct Snapshot {
        std::uint64_t generation{};
        std::uint64_t sync_version{};
        std::size_t queued_blocks{};
        std::size_t queued_input_samples{};
        std::size_t input_capacity_samples{};
        std::size_t ring_used_samples{};
        std::size_t ring_capacity_samples{};
        std::uint64_t ring_read_position{};
        std::uint64_t ring_write_position{};
        bool ring_closed{};
        bool stopping{};
        bool reset_requested{};
        bool flush_requested{};
        bool frontend_busy{};
        bool demod_busy{};
        bool acquisition_pending{};
    };

    struct AcquisitionWindow {
        std::uint64_t generation{};
        std::uint64_t base{};
        std::vector<std::complex<float>> samples;
    };

    struct CyclicPrefixMeasurement {
        bool valid{};
        std::complex<float> correlation{};
        double prefix_power{};
        double suffix_power{};
    };

    struct SymbolReadResult {
        WaitStatus status{WaitStatus::stop};
        CyclicPrefixMeasurement cyclic_prefix;
        float wait_time_ms{};
        float copy_time_ms{};
    };

    struct Callbacks {
        std::function<bool()> rebootstrap_requested;
        std::function<void()> notify_idle;
    };

    explicit SampleChannel(std::size_t ring_capacity, Callbacks callbacks);
    ~SampleChannel() noexcept;

    SampleChannel(const SampleChannel &) = delete;
    SampleChannel &operator=(const SampleChannel &) = delete;
    SampleChannel(SampleChannel &&) = delete;
    SampleChannel &operator=(SampleChannel &&) = delete;

    [[nodiscard]] bool submit(InputBlock block, std::size_t capacity_samples,
                              bool blocking);
    [[nodiscard]] FrontendWork wait_frontend();
    void finish_frontend_work() noexcept;
    [[nodiscard]] PrepareResult prepare_block(std::uint64_t generation,
                                              std::size_t ring_capacity);
    [[nodiscard]] PushResult push(std::uint64_t generation,
                                  std::span<const std::complex<float>> samples);

    void request_flush();
    [[nodiscard]] std::uint64_t request_reset();
    void acknowledge_demod_reset(std::uint64_t generation);
    void begin_frontend_reset(std::uint64_t generation);
    void complete_frontend_reset(std::uint64_t generation);
    void wait_reset_complete(std::uint64_t generation);

    [[nodiscard]] RebootstrapResult rebootstrap(InputBlock *active_block,
                                                std::size_t input_offset);

    void publish_sync_version(std::uint64_t version);
    void notify_frontend() noexcept;
    void notify_ring() noexcept;
    void stop() noexcept;

    void set_demod_busy(bool busy) noexcept;
    void set_acquisition_pending(bool pending) noexcept;
    [[nodiscard]] bool cancelled() const noexcept;
    [[nodiscard]] std::uint64_t generation() const noexcept;
    [[nodiscard]] Snapshot snapshot() const;

    [[nodiscard]] WaitStatus wait_for_stream(std::uint64_t sync_version,
                                             bool have_grid,
                                             std::size_t acquisition_samples);
    [[nodiscard]] WaitStatus wait_for_rebootstrap(std::uint64_t sync_version);
    [[nodiscard]] WaitStatus wait_for_reopen(std::uint64_t sync_version);
    [[nodiscard]] WaitStatus wait_for_retry(std::uint64_t sync_version,
                                            std::chrono::milliseconds timeout);
    [[nodiscard]] WaitStatus
    wait_for_available(std::size_t samples, std::uint64_t generation,
                       std::chrono::milliseconds timeout);
    [[nodiscard]] AcquisitionWindow
    acquisition_window(std::size_t maximum_samples,
                       std::size_t minimum_samples) const;
    [[nodiscard]] SymbolReadResult
    read_symbol(std::uint64_t sync_version, std::uint64_t next_symbol_start,
                std::size_t fft_size, std::size_t guard_size,
                bool measure_cyclic_prefix,
                std::span<std::complex<float>> output);

    [[nodiscard]] std::uint64_t align_at_or_after(std::uint64_t position,
                                                  std::uint64_t period) const;
    [[nodiscard]] std::uint64_t read_position() const;
    [[nodiscard]] std::uint64_t write_position() const;
    void discard_all() noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
