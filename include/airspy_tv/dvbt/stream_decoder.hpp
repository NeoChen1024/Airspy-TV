#pragma once

#include "airspy_tv/dvbt/receiver_parameters.hpp"
#include "airspy_tv/dvbt/transport_decoder.hpp"

#include <complex>
#include <cstddef>
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
    float processing_realtime_ratio{};
    float resample_time_ms{};
    float acquisition_time_ms{};
    float equalization_time_ms{};
    float fec_time_ms{};
    float demap_time_ms{};
    float deinterleave_time_ms{};
    float depuncture_time_ms{};
    float transport_time_ms{};
    std::size_t resample_workers{};
    std::size_t symbol_workers{};
    std::uint64_t pilot_phase_discontinuities{};
    std::uint64_t input_blocks{};
    std::uint64_t dropped_blocks{};
    std::uint64_t ofdm_symbols{};
    std::uint64_t transport_bytes{};
    std::size_t queued_blocks{};
    std::size_t queued_input_samples{};
    std::size_t input_queue_capacity_samples{};
    std::size_t queued_symbols{};
    std::size_t symbol_queue_capacity{};
    bool processing{};
    bool fec_processing{};
    TransportDecoderStats transport{};
};

// Asynchronous CS16-to-TS receiver. submit() only copies into a bounded queue;
// the front-end uses partitioned resampling and ordered symbol workers, while a
// stateful transport worker feeds the Viterbi pool, RS decoder, and TS output.
class StreamDecoder {
  public:
    static constexpr std::size_t processing_chunk_samples = 7'000'000;

    using TransportCallback =
        std::function<void(std::span<const std::uint8_t>)>;
    using EqualizedCallback =
        std::function<void(std::span<const std::complex<float>>,
                           std::span<const float>, std::size_t)>;

    StreamDecoder();
    ~StreamDecoder() noexcept;
    StreamDecoder(const StreamDecoder &) = delete;
    StreamDecoder &operator=(const StreamDecoder &) = delete;
    StreamDecoder(StreamDecoder &&) = delete;
    StreamDecoder &operator=(StreamDecoder &&) = delete;

    void submit(std::span<const std::int16_t> interleaved_iq,
                std::uint32_t sample_rate_hz,
                std::uint32_t channel_bandwidth_hz = 6'000'000);
    // Decoder-paced file input: wait for queue capacity instead of dropping an
    // input block. Live SDR callbacks should continue to use submit().
    void submit_blocking(std::span<const std::int16_t> interleaved_iq,
                         std::uint32_t sample_rate_hz,
                         std::uint32_t channel_bandwidth_hz = 6'000'000);
    // Process any final partial chunk, then wait until all queued input has
    // completed. This is intended for finite, decoder-paced file input.
    void flush();
    void wait_until_idle();
    void reset();
    void set_parameters(const ReceiverParameters &parameters);
    void set_transport_callback(TransportCallback callback);
    void set_equalized_callback(EqualizedCallback callback);
    [[nodiscard]] StreamDecoderStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
