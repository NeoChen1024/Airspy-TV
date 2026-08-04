#pragma once

#include "airspy_tv/demodulator.hpp"
#include "airspy_tv/dvbt/receiver_parameters.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/transport_decoder.hpp"

#include <algorithm>
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
    bool state_carried{};
    bool fec_skipped{};
    float tracked_carrier_offset_hz{};
    std::size_t acquisition_start{};
    float timing_offset_samples{};
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

// Asynchronous CS16-to-TS receiver on a continuous three-stage pipeline:
// a front-end thread runs the streaming resampler (one persistent liquid
// filter state, no per-chunk warmup) and an event-driven acquisition monitor;
// a demod thread extracts a fully contiguous symbol stream (carried CFO,
// carrier, continual-reference, and TPS superframe state — re-seeded only on
// cold starts) through ordered symbol workers and a windowed MER gate; a
// stateful transport worker feeds the Viterbi pool, RS decoder, and TS output.
// The demodulator owns its GUI analysis path (SignalAnalyzer), which is fed
// from the same input on submit().
class StreamDecoder : public Demodulator {
  public:
    // Ingestion budget: input is submitted in ~0.2 s spans and the input
    // queue retains about that much, bounding end-to-end latency for live
    // streams. Chunk sizes are derived from the sample rate — the historical
    // fixed 7 M-sample (0.7 s at 10 MHz) constant was removed so chunking
    // never defines DSP latency or queue behaviour.
    static constexpr std::uint32_t input_budget_denominator = 5;
    static std::size_t
    chunk_samples_for(const std::uint32_t sample_rate_hz) noexcept {
        return std::max<std::size_t>(
            1, (static_cast<std::size_t>(sample_rate_hz) +
                input_budget_denominator - 1) /
                   input_budget_denominator);
    }

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
    void
    submit_blocking(std::span<const std::int16_t> interleaved_iq,
                    std::uint32_t sample_rate_hz,
                    std::uint32_t channel_bandwidth_hz = 6'000'000) override;
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
