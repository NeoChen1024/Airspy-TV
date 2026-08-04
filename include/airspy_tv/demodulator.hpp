#pragma once

#include "airspy_tv/transport_stream.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

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
    using DiscontinuityCallback =
        std::function<void(TransportDiscontinuity)>;

    virtual ~Demodulator() = default;

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
    virtual void set_discontinuity_callback(DiscontinuityCallback callback) =
        0;
    [[nodiscard]] virtual DemodulatorStats demodulator_stats() const = 0;
};

} // namespace airspy_tv
