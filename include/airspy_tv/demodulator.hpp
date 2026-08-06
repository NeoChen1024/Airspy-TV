#pragma once

#include "airspy_tv/transport_stream.hpp"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <string_view>

namespace airspy_tv {

// Standard-neutral summary of a demodulator, suitable for generic UI panels.
// Standard-specific detail remains available through the concrete demodulator
// type (the UI constructs the concrete demodulator and injects it).
struct DemodulatorStats {
    bool locked{};
    float mer_db{};
    float ber{};
    std::size_t worker_threads{};
    std::uint64_t transport_bytes{};
    bool processing{};
};

inline constexpr std::size_t common_constellation_snapshot_size = 2048;
inline constexpr std::size_t common_pipeline_stage_count = 8;

// Standard-neutral signal telemetry. The common GUI can render this without
// knowing whether the active engine is OFDM- or QAM-based; standard-specific
// acquisition/TPS/PLP detail stays in the concrete demodulator snapshot.
struct SignalSnapshot {
    std::array<std::complex<float>, common_constellation_snapshot_size>
        constellation{};
    std::size_t constellation_count{};
    float mer_db{};
    float snr_db{};
    float deepest_notch_db{};
    float carrier_offset_hz{};
    float carrier_offset_limit_hz{};
    std::uint64_t sequence{};
    bool signal_locked{};
    bool transport_locked{};
};

struct PipelineStageSnapshot {
    // Stage names must refer to static storage owned by the demodulator.
    std::string_view name;
    float busy_fraction{};
    float queue_fraction{};
    std::size_t workers{};
    bool busy_valid{};
    bool queue_valid{};
};

struct PipelineSnapshot {
    std::array<PipelineStageSnapshot, common_pipeline_stage_count> stages{};
    std::size_t stage_count{};
    float processing_realtime_ratio{};
    std::uint64_t dropped_blocks{};
    std::uint64_t transport_bytes{};
    std::uint64_t sequence{};
    bool processing{};
    bool failed{};
    std::string error;
};

// The broadcast standard being decoded. Each implemented standard maps to a
// concrete Demodulator implementation (only DvbT today); the remaining values
// are reserved for the roadmap standards (DVB-C, DVB-T2, DTMB, ATSC).
enum class ReceiveStandard {
    DvbT,
    DvbC,
    DvbT2,
    Dtmb,
    Atsc,
};

// Standard-agnostic I/Q-to-MPEG-TS demodulator seam. Concrete standards
// (dvbt::StreamDecoder today; future dvbc/dvbt2/dtmb modules) implement this,
// and the receiver layer interacts only with this interface. Standard-specific
// configuration happens on the concrete type before injection, because
// parameters are inherently standard-specific.
class Demodulator {
  public:
    using TransportCallback =
        std::function<void(std::span<const std::uint8_t>)>;
    using DiscontinuityCallback = std::function<void(TransportDiscontinuity)>;

    virtual ~Demodulator() = default;

    // Publish a discontinuity and return immediately. Source callbacks use
    // this path so dropped-sample recovery never waits for decoder workers.
    virtual void request_reset() = 0;
    virtual void reset() = 0;
    virtual void submit(std::span<const std::int16_t> interleaved_iq,
                        std::uint32_t sample_rate_hz,
                        std::uint32_t channel_bandwidth_hz) = 0;
    // Decoder-paced file input: wait for queue capacity instead of dropping an
    // input block. Live SDR callbacks should continue to use submit().
    virtual void submit_blocking(std::span<const std::int16_t> interleaved_iq,
                                 std::uint32_t sample_rate_hz,
                                 std::uint32_t channel_bandwidth_hz) = 0;
    // Process any final partial chunk, then wait until all queued input has
    // completed. This is intended for finite, decoder-paced file input.
    virtual void flush() = 0;
    virtual void wait_until_idle() = 0;
    virtual void set_transport_callback(TransportCallback callback) = 0;
    // Out-of-band stream-level events (see TransportDiscontinuity), fired by
    // the demod threads as they happen. The callback must not block on the
    // decoder or call back into it.
    virtual void set_discontinuity_callback(DiscontinuityCallback callback) = 0;
    virtual void set_signal_smoothing(bool enabled, int speed) = 0;
    [[nodiscard]] virtual DemodulatorStats demodulator_stats() const = 0;
    [[nodiscard]] virtual SignalSnapshot signal_snapshot() const = 0;
    [[nodiscard]] virtual PipelineSnapshot pipeline_snapshot() const = 0;
};

} // namespace airspy_tv
