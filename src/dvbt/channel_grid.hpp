#pragma once

#include "airspy_tv/fftw_plan.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

// Channel frequency response H on the scattered-pilot grid. Each OFDM symbol
// carries scattered pilots on every twelfth carrier (3 * phase + 12 m); the
// four consecutive pilot phases together sample H on every third carrier.
// The grid keeps the most recent H per slot (k = 3 j) and interpolates the
// equalizer response between the slots that have been observed since the
// last discontinuity.
//
// A complete grid is interpolated through the delay domain. The slots are
// de-ramped by the centre of the passband (the delay range the significant
// paths occupy, plus a margin), extended beyond both band edges by linear
// prediction so the band limits do not ring into the response, and
// transformed into an impulse response over Tu/3; every path outside the
// passband is suppressed and the response is transformed back at
// single-carrier spacing and re-ramped. This is exact for any delay spread
// the guard interval covers, including steep phase ramps from a window
// placed in the middle of the guard, and it averages the pilot noise over
// the suppressed delays. The passband starts as the ISI-free range of the
// window ([0, guard] plus a margin) and follows the measured delay spread
// once it is known. An incomplete grid (fewer than four contiguous symbols)
// falls back to linear interpolation between the known slots.
class ScatteredPilotGrid {
  public:
    void configure(std::size_t maximum, std::size_t fft_size,
                   std::size_t guard_size);
    // Forget the slots and restore the default passband (sync loss, phase
    // jump or a window whose position is no longer known).
    void reset() noexcept;

    // Delay range (samples relative to the FFT window start) the
    // interpolation passes; a range wider than the unambiguous Tu/3 is
    // narrowed symmetrically. The default is the ISI-free range of the
    // window with a margin of guard/16 on both sides.
    void set_passband(double first_delay_samples, double last_delay_samples);
    void reset_passband();

    // inverse_channel[k] = sent / received on the scattered pilots of
    // `phase`, zero where the pilot was unusable. A non-contiguous symbol
    // invalidates the other phases first.
    void update(std::span<const std::complex<float>> inverse_channel, int phase,
                bool contiguous);

    // The FFT window of the following symbols starts `delta_samples` later
    // than the window the grid was measured with: every path appears that
    // much earlier, which is a known phase ramp across the slots. Re-ramping
    // keeps the grid complete across an intentional window move; the
    // passband moves with it.
    void shift_window(int delta_samples) noexcept;

    // H per slot (zero where unknown) and whether all four phases are
    // present.
    [[nodiscard]] std::span<const std::complex<float>> values() const noexcept {
        return values_;
    }
    [[nodiscard]] bool complete() const noexcept { return phase_mask_ == 0xFU; }
    [[nodiscard]] bool has_phase(const int phase) const noexcept {
        return phase >= 0 && phase < 4 &&
               (phase_mask_ & (1U << static_cast<unsigned>(phase))) != 0U;
    }

    // Equalizer response 1/H for every carrier 0..maximum (zero at nulls).
    // `dominant_delay_samples` is the current timing estimate (the delay of
    // the channel's energy centroid relative to the FFT window); only the
    // incomplete-grid fallback needs it.
    void interpolate_inverse(std::span<std::complex<float>> inverse_channel,
                             double dominant_delay_samples);

  private:
    void interpolate_linear(std::span<std::complex<float>> inverse_channel,
                            double dominant_delay_samples) const;
    void
    interpolate_delay_domain(std::span<std::complex<float>> inverse_channel);
    // Extend the de-ramped slots in delay_in_ beyond both band edges.
    void extend_band_edges();

    std::size_t maximum_{};
    std::size_t fft_size_{};
    std::size_t guard_size_{};
    std::vector<std::complex<float>> values_;
    std::vector<std::uint8_t> known_;
    unsigned phase_mask_{};

    // Delay-domain interpolation: de-ramped and extrapolated slots ->
    // delay_bins_ (IFFT), masked and placed into a 3x longer array ->
    // carriers (FFT), re-ramped.
    std::size_t delay_bins_{};
    double bin_samples_{};
    double passband_margin_{};
    double passband_first_{};
    double passband_last_{};
    std::vector<std::complex<float>> deramp_;
    std::vector<std::complex<float>> reramp_;
    std::vector<float> taper_;
    std::vector<std::complex<float>> prediction_sequence_;
    std::vector<std::complex<float>> prediction_forward_;
    std::vector<std::complex<float>> prediction_backward_;
    std::vector<std::complex<float>> prediction_forward_coefficients_;
    std::vector<std::complex<float>> prediction_backward_coefficients_;
    unsigned interpolations_since_refit_{};
    std::vector<std::complex<float>> delay_in_;
    std::vector<std::complex<float>> delay_;
    std::vector<float> delay_mask_;
    std::vector<std::uint32_t> delay_position_;
    std::vector<std::complex<float>> carrier_in_;
    std::vector<std::complex<float>> carrier_;
    FftwfPlan delay_plan_;
    FftwfPlan carrier_plan_;
};

} // namespace airspy_tv::dvbt
