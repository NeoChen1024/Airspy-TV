#pragma once

#include "airspy_tv/dvbt/telemetry.hpp"
#include "airspy_tv/fftw_plan.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

// Pilot-based channel impulse response and Doppler profiler.
//
// Four consecutive OFDM symbols place scattered pilots on every third
// carrier. Their channel responses, Hann-windowed and zero-padded, are
// inverse-transformed into one impulse response per four symbols that is
// unambiguous over Tu/3 of delay (about 400 us at 8K / 6 MHz). Over a TPS
// frame the responses are averaged into a power delay profile, and the
// complex correlation of consecutive responses yields a Doppler estimate
// and a coherence figure per delay bin. The common frequency offset is
// removed by the CFO loop, so per-tap Doppler is relative to the tracked
// carrier. Per-symbol gain mismatch between the four interleaved symbols
// folds into images at multiples of Tu/12.
class ChannelProfiler {
  public:
    void configure(std::size_t fft_size, std::size_t maximum,
                   double sample_rate_hz, std::size_t symbol_period_samples);
    // Forget the partial response and the frame accumulators (sync loss).
    void reset() noexcept;

    // inverse_channel[k] = sent / received on the scattered pilots of
    // `phase` (zero elsewhere). `contiguous` says the symbol directly
    // follows the previous one in the sample stream.
    void add_symbol(std::span<const std::complex<float>> inverse_channel,
                    int phase, bool contiguous, std::uint64_t symbol_start);

    // After a full frame of symbols, the profile without envelope; empty
    // when the frame produced no response.
    [[nodiscard]] std::optional<ChannelProfileTelemetry> take_frame();

  private:
    void compute_response();
    void reset_frame() noexcept;

    std::size_t fft_size_{};
    std::size_t slots_{};
    double sample_rate_hz_{};
    std::size_t period_{};
    std::vector<float> window_;
    std::vector<std::complex<float>> grid_;
    std::vector<std::complex<float>> response_;
    std::vector<std::complex<float>> previous_;
    std::vector<float> power_;
    std::vector<std::complex<float>> correlation_;
    std::vector<float> pair_power_;
    FftwfPlan plan_;
    unsigned phase_mask_{};
    bool have_previous_{};
    bool pair_contiguous_{};
    std::uint32_t frame_symbols_{};
    std::uint32_t response_count_{};
    std::uint32_t pair_count_{};
    std::uint64_t frame_begin_{};
    std::uint64_t frame_end_{};
};

} // namespace airspy_tv::dvbt
