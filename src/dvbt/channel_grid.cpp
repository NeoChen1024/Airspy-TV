#include "channel_grid.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>

namespace airspy_tv::dvbt {

namespace {

constexpr float minimum_power = 1.0e-12F;
// The passband extends the delay range of the paths by this fraction of the
// guard interval on both sides (never less than a few samples), with a
// raised-cosine roll-off of the same width so the mask's spectral kernel
// decays quickly.
constexpr double passband_margin_fraction = 1.0 / 16.0;
constexpr double passband_margin_minimum_samples = 8.0;
// Band-edge extension: an autoregressive model of this order, fitted to the
// de-ramped slots with Burg's method, predicts up to this many slots beyond
// each edge; the prediction is faded out with a raised cosine. A few
// dominant paths are modelled exactly, and the taper hides the rest. The
// model is refitted once per pilot cycle; between refits the previous
// coefficients predict the current slots.
constexpr std::size_t prediction_order = 8;
constexpr std::size_t prediction_length_limit = 256;
constexpr unsigned prediction_refit_interval = 4;

// Burg's method: coefficients a[1..p] (a[0] = 1) of the autoregressive
// model x[n] ~ -sum a[i] x[n - i] that minimise the forward and backward
// prediction errors; the model is always stable, so extrapolating with it
// cannot blow up. `forward` and `backward` are scratch of x's size.
void burg_coefficients(const std::span<const std::complex<float>> x,
                       const std::span<std::complex<float>> forward,
                       const std::span<std::complex<float>> backward,
                       const std::span<std::complex<float>> a) {
    const std::size_t n = x.size();
    const std::size_t order = a.size() - 1;
    std::ranges::copy(x, forward.begin());
    std::ranges::copy(x, backward.begin());
    a[0] = {1.0F, 0.0F};
    std::fill(a.begin() + 1, a.end(), std::complex<float>{});
    for (std::size_t m = 1; m <= order && m < n; ++m) {
        std::complex<double> numerator{};
        double denominator = 0.0;
        for (std::size_t i = m; i < n; ++i) {
            const std::complex<double> f = forward[i];
            const std::complex<double> b = backward[i - 1];
            numerator += std::conj(b) * f;
            denominator += std::norm(f) + std::norm(b);
        }
        if (denominator <= 0.0) {
            break;
        }
        const auto k =
            static_cast<std::complex<float>>(-2.0 * numerator / denominator);
        for (std::size_t i = 1; i <= m / 2; ++i) {
            const std::complex<float> low = a[i];
            const std::complex<float> high = a[m - i];
            a[i] = low + k * std::conj(high);
            if (i != m - i) {
                a[m - i] = high + k * std::conj(low);
            }
        }
        a[m] = k;
        for (std::size_t i = n - 1; i >= m; --i) {
            const std::complex<float> f = forward[i];
            const std::complex<float> b = backward[i - 1];
            forward[i] = f + k * b;
            backward[i] = b + std::conj(k) * f;
        }
    }
}

// x[n] = -sum a[i] x[n - i] for the `extension` samples following `x`.
void extrapolate(const std::span<const std::complex<float>> x,
                 const std::span<const std::complex<float>> a,
                 const std::span<std::complex<float>> extension) {
    const std::size_t order = a.size() - 1;
    const auto sample = [&](const std::size_t index) {
        return index < x.size() ? x[index] : extension[index - x.size()];
    };
    for (std::size_t e = 0; e < extension.size(); ++e) {
        const std::size_t n = x.size() + e;
        std::complex<float> value{};
        for (std::size_t i = 1; i <= order && i <= n; ++i) {
            value -= a[i] * sample(n - i);
        }
        extension[e] = value;
    }
}

std::complex<float> invert(const std::complex<float> response) {
    const float power = std::norm(response);
    return power > minimum_power ? std::conj(response) / power
                                 : std::complex<float>{};
}

} // namespace

void ScatteredPilotGrid::configure(const std::size_t maximum,
                                   const std::size_t fft_size,
                                   const std::size_t guard_size) {
    maximum_ = maximum;
    fft_size_ = fft_size;
    guard_size_ = guard_size;
    values_.assign((maximum / 3) + 1, {});
    known_.assign(values_.size(), 0U);
    phase_mask_ = 0;

    // Delay resolution of 1.5x oversampling or better keeps the mask edges
    // smooth; the unambiguous delay range of the three-carrier grid is Tu/3.
    const std::size_t slots = values_.size();
    delay_bins_ = std::bit_ceil(((slots * 3) + 1) / 2);
    bin_samples_ = static_cast<double>(fft_size) /
                   (3.0 * static_cast<double>(delay_bins_));
    passband_margin_ =
        std::max(static_cast<double>(guard_size) * passband_margin_fraction,
                 passband_margin_minimum_samples);
    delay_in_.assign(delay_bins_, {});
    delay_.assign(delay_bins_, {});
    delay_mask_.assign(delay_bins_, 0.0F);
    delay_position_.assign(delay_bins_, 0U);
    const std::size_t carrier_bins = 3 * delay_bins_;
    carrier_in_.assign(carrier_bins, {});
    carrier_.assign(carrier_bins, {});
    deramp_.assign(slots, {});
    reramp_.assign(maximum + 1, {});
    // Beyond each band edge the de-ramped response is extrapolated and
    // faded out within the free half of the zero padding.
    const std::size_t extension =
        std::min(prediction_length_limit, (delay_bins_ - slots) / 2);
    taper_.assign(extension, 0.0F);
    for (std::size_t m = 0; m < extension; ++m) {
        taper_[m] = static_cast<float>(
            0.5 *
            (1.0 + std::cos(std::numbers::pi * static_cast<double>(m + 1) /
                            static_cast<double>(extension + 1))));
    }
    prediction_sequence_.assign(slots, {});
    prediction_forward_.assign(slots, {});
    prediction_backward_.assign(slots, {});
    prediction_forward_coefficients_.assign(
        std::min(prediction_order, slots) + 1, {});
    prediction_backward_coefficients_.assign(
        prediction_forward_coefficients_.size(), {});
    interpolations_since_refit_ = 0;
    for (std::size_t n = 0; n < delay_bins_; ++n) {
        // Bin n holds the relative delay n (n <= bins/2) or n - bins.
        delay_position_[n] = static_cast<std::uint32_t>(
            n <= delay_bins_ / 2 ? n : carrier_bins - (delay_bins_ - n));
    }
    reset_passband();
    delay_plan_ =
        FftwfPlan::dft_1d(static_cast<int>(delay_bins_),
                          reinterpret_cast<fftwf_complex *>(delay_in_.data()),
                          reinterpret_cast<fftwf_complex *>(delay_.data()),
                          FFTW_BACKWARD, FFTW_MEASURE);
    carrier_plan_ =
        FftwfPlan::dft_1d(static_cast<int>(carrier_bins),
                          reinterpret_cast<fftwf_complex *>(carrier_in_.data()),
                          reinterpret_cast<fftwf_complex *>(carrier_.data()),
                          FFTW_FORWARD, FFTW_MEASURE);
    // FFTW_MEASURE scribbles on the arrays; every use rewrites them.
    std::ranges::fill(delay_in_, std::complex<float>{});
    std::ranges::fill(carrier_in_, std::complex<float>{});
}

void ScatteredPilotGrid::reset_passband() {
    set_passband(-passband_margin_,
                 static_cast<double>(guard_size_) + passband_margin_);
}

void ScatteredPilotGrid::set_passband(const double first_delay_samples,
                                      const double last_delay_samples) {
    if (delay_bins_ == 0) {
        return;
    }
    passband_first_ = first_delay_samples;
    passband_last_ = last_delay_samples;
    const double unambiguous = bin_samples_ * static_cast<double>(delay_bins_);
    const double margin = passband_margin_;
    const double centre = 0.5 * (first_delay_samples + last_delay_samples);
    const double half_width =
        std::clamp(0.5 * (last_delay_samples - first_delay_samples), 0.0,
                   (unambiguous / 2.0) - margin);
    for (std::size_t n = 0; n < delay_bins_; ++n) {
        const double relative =
            (n <= delay_bins_ / 2
                 ? static_cast<double>(n)
                 : static_cast<double>(n) - static_cast<double>(delay_bins_)) *
            bin_samples_;
        const double outside = std::abs(relative) - half_width;
        float weight = 0.0F;
        if (outside <= 0.0) {
            weight = 1.0F;
        } else if (outside < margin) {
            weight = static_cast<float>(
                0.5 * (1.0 + std::cos(std::numbers::pi * outside / margin)));
        }
        delay_mask_[n] = weight;
    }
    // A path at the passband centre is flat across the de-ramped slots.
    const double ramp =
        2.0 * std::numbers::pi * centre / static_cast<double>(fft_size_);
    for (std::size_t j = 0; j < deramp_.size(); ++j) {
        const double angle = ramp * static_cast<double>(3 * j);
        deramp_[j] = {static_cast<float>(std::cos(angle)),
                      static_cast<float>(std::sin(angle))};
    }
    for (std::size_t k = 0; k < reramp_.size(); ++k) {
        const double angle = -ramp * static_cast<double>(k);
        reramp_[k] = {static_cast<float>(std::cos(angle)),
                      static_cast<float>(std::sin(angle))};
    }
}

void ScatteredPilotGrid::reset() noexcept {
    std::ranges::fill(values_, std::complex<float>{});
    std::ranges::fill(known_, 0U);
    phase_mask_ = 0;
    interpolations_since_refit_ = 0;
    reset_passband();
}

void ScatteredPilotGrid::shift_window(const int delta_samples) noexcept {
    if (delta_samples == 0 || fft_size_ == 0) {
        return;
    }
    // A path at delay d contributes exp(-j 2 pi f d / N) at carrier
    // frequency f = k - maximum / 2 (the centre carrier sits at DC); moving
    // the window start by delta turns d into d - delta.
    const double ramp = 2.0 * std::numbers::pi *
                        static_cast<double>(delta_samples) /
                        static_cast<double>(fft_size_);
    const double centre = static_cast<double>(maximum_) / 2.0;
    for (std::size_t j = 0; j < values_.size(); ++j) {
        const double angle = ramp * (static_cast<double>(3 * j) - centre);
        values_[j] *= std::complex<float>{static_cast<float>(std::cos(angle)),
                                          static_cast<float>(std::sin(angle))};
    }
    set_passband(passband_first_ - delta_samples,
                 passband_last_ - delta_samples);
}

void ScatteredPilotGrid::update(
    const std::span<const std::complex<float>> inverse_channel, const int phase,
    const bool contiguous) {
    if (phase < 0 || phase > 3 || values_.empty()) {
        return;
    }
    if (!contiguous) {
        reset();
    }
    for (auto j = static_cast<std::size_t>(phase); j < values_.size(); j += 4) {
        const std::size_t k = 3 * j;
        const std::complex<float> inverse = k < inverse_channel.size()
                                                ? inverse_channel[k]
                                                : std::complex<float>{};
        const float power = std::norm(inverse);
        const bool usable = power > 0.0F;
        values_[j] =
            usable ? std::conj(inverse) / power : std::complex<float>{};
        known_[j] = usable ? 1U : 0U;
    }
    phase_mask_ |= 1U << static_cast<unsigned>(phase);
}

void ScatteredPilotGrid::interpolate_inverse(
    const std::span<std::complex<float>> inverse_channel,
    const double dominant_delay_samples) {
    if (complete() && delay_plan_ && carrier_plan_) {
        interpolate_delay_domain(inverse_channel);
    } else {
        interpolate_linear(inverse_channel, dominant_delay_samples);
    }
}

void ScatteredPilotGrid::interpolate_delay_domain(
    const std::span<std::complex<float>> inverse_channel) {
    // Pilots lost to a null are zero in values_; a zero slot is a notch,
    // which the delay-domain transform spreads over its neighbours rather
    // than treating as unknown. That matches a deep fade well enough.
    const std::size_t slots = values_.size();
    std::ranges::fill(delay_in_, std::complex<float>{});
    for (std::size_t j = 0; j < slots; ++j) {
        delay_in_[j] = values_[j] * deramp_[j];
    }
    extend_band_edges();
    delay_plan_.execute();
    std::ranges::fill(carrier_in_, std::complex<float>{});
    const float scale = 1.0F / static_cast<float>(delay_bins_);
    for (std::size_t n = 0; n < delay_bins_; ++n) {
        if (delay_mask_[n] != 0.0F) {
            carrier_in_[delay_position_[n]] +=
                delay_[n] * (delay_mask_[n] * scale);
        }
    }
    carrier_plan_.execute();
    const std::size_t count = std::min(inverse_channel.size(), maximum_ + 1);
    for (std::size_t k = 0; k < count; ++k) {
        inverse_channel[k] = invert(carrier_[k] * reramp_[k]);
    }
    std::fill(inverse_channel.begin() + static_cast<std::ptrdiff_t>(count),
              inverse_channel.end(), std::complex<float>{});
}

void ScatteredPilotGrid::extend_band_edges() {
    const std::size_t slots = values_.size();
    const std::size_t extension = taper_.size();
    if (extension == 0 || prediction_forward_coefficients_.size() < 2) {
        return;
    }
    const std::span<std::complex<float>> in_band{delay_in_.data(), slots};
    std::ranges::reverse_copy(in_band, prediction_sequence_.begin());
    if (interpolations_since_refit_ == 0) {
        burg_coefficients(in_band, prediction_forward_, prediction_backward_,
                          prediction_forward_coefficients_);
        burg_coefficients(prediction_sequence_, prediction_forward_,
                          prediction_backward_,
                          prediction_backward_coefficients_);
    }
    interpolations_since_refit_ =
        (interpolations_since_refit_ + 1) % prediction_refit_interval;
    // Above the band: predict forward from the last slots.
    const std::span<std::complex<float>> above{delay_in_.data() + slots,
                                               extension};
    extrapolate(in_band, prediction_forward_coefficients_, above);
    for (std::size_t m = 0; m < extension; ++m) {
        above[m] *= taper_[m];
    }
    // Below the band: the same on the reversed slots, written backwards
    // from the end of the (circular) array.
    const std::span<std::complex<float>> below{prediction_forward_.data(),
                                               extension};
    extrapolate(prediction_sequence_, prediction_backward_coefficients_, below);
    for (std::size_t m = 0; m < extension; ++m) {
        delay_in_[delay_bins_ - 1 - m] = below[m] * taper_[m];
    }
}

void ScatteredPilotGrid::interpolate_linear(
    const std::span<std::complex<float>> inverse_channel,
    const double dominant_delay_samples) const {
    // Linear interpolation of H between the known slots. The window may sit
    // anywhere inside the guard, so the phase ramp of the dominant delay is
    // removed before interpolating and restored afterwards; without that a
    // window in the middle of the guard rotates H by 3/8 of a turn per slot
    // and the straight line between slots is meaningless.
    const double ramp =
        2.0 * std::numbers::pi * dominant_delay_samples /
        static_cast<double>(std::max<std::size_t>(fft_size_, 1));
    const auto derotated = [&](const std::size_t j) {
        const double angle = ramp * static_cast<double>(3 * j);
        return values_[j] *
               std::complex<float>{static_cast<float>(std::cos(angle)),
                                   static_cast<float>(std::sin(angle))};
    };
    const auto restore = [&](const std::size_t k,
                             const std::complex<float> value) {
        const double angle = -ramp * static_cast<double>(k);
        return invert(value *
                      std::complex<float>{static_cast<float>(std::cos(angle)),
                                          static_cast<float>(std::sin(angle))});
    };
    std::size_t previous = values_.size();
    for (std::size_t j = 0; j < values_.size(); ++j) {
        if (known_[j] == 0U) {
            continue;
        }
        const std::size_t right = 3 * j;
        const auto current = derotated(j);
        if (previous == values_.size()) {
            // Hold the first known value down to carrier 0.
            for (std::size_t k = 0; k <= right && k < inverse_channel.size();
                 ++k) {
                inverse_channel[k] = restore(k, current);
            }
        } else {
            const std::size_t left = 3 * previous;
            const auto start = derotated(previous);
            const std::complex<float> step =
                (current - start) / static_cast<float>(right - left);
            for (std::size_t k = left; k <= right && k < inverse_channel.size();
                 ++k) {
                inverse_channel[k] =
                    restore(k, start + step * static_cast<float>(k - left));
            }
        }
        previous = j;
    }
    if (previous == values_.size()) {
        std::ranges::fill(inverse_channel, std::complex<float>{});
        return;
    }
    const auto last = derotated(previous);
    for (std::size_t k = 3 * previous; k < inverse_channel.size(); ++k) {
        inverse_channel[k] = restore(k, last);
    }
}

} // namespace airspy_tv::dvbt
