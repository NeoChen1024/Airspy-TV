#pragma once

#include "airspy_tv/diagnostic_event.hpp"
#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/dvbt/transport_decoder.hpp"

#include "fec_stage_item.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace airspy_tv::dvbt {

struct FecStageDiagnosticContext {
    std::uint64_t generation{};
    std::uint64_t source_epoch{};
    std::uint64_t demod_window_sequence{};
    std::uint64_t fec_session{};
    std::optional<std::uint64_t> tps_symbol_index;
};

struct FecStageSession {
    std::uint64_t generation{};
    std::uint64_t fec_session{};
    TransportDecoderStats cumulative;
};

struct FecStageWindow {
    std::uint64_t generation{};
    std::uint64_t source_epoch{};
    std::uint64_t demod_window_sequence{};
    std::uint64_t fec_session{};
    std::uint64_t output_bytes_delta{};
    TransportDecoderStats session;
    TransportDecoderStats delta;
    TransportDecoderStats cumulative;
    float fec_total_ms{};
    float transport_nested_ms{};
};

class FecStage {
  public:
    struct Callbacks {
        std::function<bool(std::uint64_t)> generation_current;
        std::function<void(const FecStageSession &)> publish_session;
        std::function<void(const FecStageWindow &)> publish_window;
        std::function<bool(std::uint64_t, std::span<const std::uint8_t>)>
            emit_transport;
        std::function<void(std::uint64_t, TransportDiscontinuity)>
            emit_discontinuity;
        std::function<bool()> diagnostics_enabled;
        std::function<void(DiagnosticEvent, const FecStageDiagnosticContext &)>
            emit_diagnostic;
        std::function<void()> notify_idle;
        std::function<void(std::exception_ptr, std::string)> worker_failed;
    };

    struct Snapshot {
        std::size_t queued_items{};
        std::size_t capacity{};
        WorkerState worker_state{WorkerState::idle};
        bool processing{};
    };

    explicit FecStage(Callbacks callbacks, std::size_t capacity);
    ~FecStage() noexcept;

    FecStage(const FecStage &) = delete;
    FecStage &operator=(const FecStage &) = delete;
    FecStage(FecStage &&) = delete;
    FecStage &operator=(FecStage &&) = delete;

    [[nodiscard]] bool enqueue(FecItem item);
    void clear();
    void set_capacity(std::size_t capacity);
    void notify_cancelled() noexcept;
    void request_stop() noexcept;
    void stop() noexcept;
    [[nodiscard]] Snapshot snapshot() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
