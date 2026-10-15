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
    void configure(std::size_t fft_size, std::size_t guard_size,
                   std::size_t maximum, double sample_rate_hz);
    // Forget the partial response and the frame accumulators (sync loss).
    void reset() noexcept;

    // H per scattered-pilot slot (k = 3 j, zero where unknown) after this
    // symbol's pilots were added; `complete` says all four phases are
    // present and `contiguous` that the symbol directly follows the
    // previous one in the sample stream.
    void add_symbol(std::span<const std::complex<float>> grid_values,
                    bool complete, bool contiguous, std::uint64_t symbol_start);

    // Window and inverse-transform the current grid; the per-bin energy of
    // the resulting impulse response (delay bin = Tu / (3 * size)). Used
    // for FFT-window placement independently of the frame statistics.
    [[nodiscard]] std::span<const float>
    response_energy(std::span<const std::complex<float>> grid_values);

    // Delay structure of the current grid's impulse response, in bins
    // relative to the FFT window start (negative = a path that arrives
    // before the window), unwrapped around the strongest tap. `first`
    // and `last` bound the taps within 20 dB of the strongest one and
    // `centroid` is their energy-weighted mean delay; `valid` is false
    // when the response has no dominant structure (noise or no lock).
    // `first_bin`/`last_bin` span the paths within 20 dB of the strongest
    // (window placement); `extent_first_bin`/`extent_last_bin` span every
    // path above the noise floor (equalizer passband). Bins are relative to
    // the FFT window start and may be negative.
    struct ResponseSummary {
        bool valid{};
        double peak_share{};
        double peak_bin{};
        double first_bin{};
        double last_bin{};
        double extent_first_bin{};
        double extent_last_bin{};
        double centroid_bin{};
    };
    [[nodiscard]] ResponseSummary
    response_summary(std::span<const std::complex<float>> grid_values);

    // Samples per response bin for the given FFT size: Tu / (3 * bins).
    [[nodiscard]] static double bin_samples(std::size_t fft_size) noexcept;

    // After a full frame of symbols, the profile without envelope; empty
    // when the frame produced no response.
    [[nodiscard]] std::optional<ChannelProfileTelemetry> take_frame();

  private:
    void compute_response();
    void reset_frame() noexcept;

    std::size_t fft_size_{};
    std::size_t guard_size_{};
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
    std::vector<float> energy_;
    std::vector<float> scratch_;
    FftwfPlan plan_;
    unsigned symbols_since_response_{};
    bool have_previous_{};
    bool pair_contiguous_{};
    std::uint32_t frame_symbols_{};
    std::uint32_t response_count_{};
    std::uint32_t pair_count_{};
    std::uint64_t frame_begin_{};
    std::uint64_t frame_end_{};
};

} // namespace airspy_tv::dvbt
