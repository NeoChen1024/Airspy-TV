#pragma once

#include "airspy_tv/demodulator.hpp"
#include "airspy_tv/dvbt/receiver_parameters.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
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
    bool tps_locked{};
    Constellation tps_constellation{Constellation::qpsk};
    CodeRate tps_code_rate{CodeRate::rate_1_2};
    GuardInterval tps_guard_interval{GuardInterval::gi_1_32};
    TransmissionMode tps_mode{TransmissionMode::k2};
    std::uint8_t tps_hierarchy{};
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
    std::uint64_t processed_chunks{};
    std::uint64_t processed_input_samples{};
    std::uint64_t dropped_blocks{};
    std::uint64_t ofdm_symbols{};
    std::uint64_t transport_bytes{};
    std::uint64_t ts_overlap_packets{};
    std::uint64_t ts_overlap_join_failures{};
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
// The demodulator owns its GUI analysis path (SignalAnalyzer), which is fed
// from the same input on submit().
class StreamDecoder : public Demodulator {
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

    void reset() override;
    void submit(std::span<const std::int16_t> interleaved_iq,
                std::uint32_t sample_rate_hz,
                std::uint32_t channel_bandwidth_hz = 6'000'000) override;
    // Decoder-paced file input: wait for queue capacity instead of dropping an
    // input block. Live SDR callbacks should continue to use submit().
    void submit_blocking(std::span<const std::int16_t> interleaved_iq,
                         std::uint32_t sample_rate_hz,
                         std::uint32_t channel_bandwidth_hz = 6'000'000)
        override;
    // Process any final partial chunk, then wait until all queued input has
    // completed. This is intended for finite, decoder-paced file input.
    void flush() override;
    void wait_until_idle() override;
    void set_parameters(const ReceiverParameters &parameters);
    void set_transport_callback(TransportCallback callback) override;
    void set_equalized_callback(EqualizedCallback callback);
    [[nodiscard]] DemodulatorStats demodulator_stats() const override;
    // DVB-T-specific GUI analysis (constellation, MER, CP SNR, TPS state),
    // computed by the demodulator's own monitoring path.
    [[nodiscard]] SignalAnalysisSnapshot analysis_snapshot() const;
    void set_snr_smoothing(bool enabled, int speed);
    [[nodiscard]] StreamDecoderStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
