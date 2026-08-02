#pragma once

#include "airspy_tv/dvbt/transport_decoder.hpp"

#include <complex>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>

namespace airspy_tv::dvbt {

struct StreamDecoderStats {
    bool ofdm_locked{};
    float acquisition_score{};
    std::uint32_t fft_size{};
    std::uint32_t guard_size{};
    int carrier_bin_offset{};
    float mer_db{};
    float residual_carrier_offset_hz{};
    std::uint64_t pilot_phase_discontinuities{};
    std::uint64_t input_blocks{};
    std::uint64_t dropped_blocks{};
    std::uint64_t ofdm_symbols{};
    std::uint64_t transport_bytes{};
    TransportDecoderStats transport{};
};

// Asynchronous CS16-to-TS receiver. submit() only copies into a bounded queue;
// OFDM and FEC work always runs on the private worker thread.
class StreamDecoder {
  public:
    using TransportCallback =
        std::function<void(std::span<const std::uint8_t>)>;
    using EqualizedCallback =
        std::function<void(std::span<const std::complex<float>>,
                           std::span<const float>, std::size_t)>;

    StreamDecoder();
    ~StreamDecoder() noexcept;
    StreamDecoder(const StreamDecoder &) = delete;
    StreamDecoder &operator=(const StreamDecoder &) = delete;

    void submit(std::span<const std::int16_t> interleaved_iq,
                std::uint32_t sample_rate_hz,
                std::uint32_t channel_bandwidth_hz = 6'000'000);
    void reset();
    void set_transport_callback(TransportCallback callback);
    void set_equalized_callback(EqualizedCallback callback);
    [[nodiscard]] StreamDecoderStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
