#include "channel_profile.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace airspy_tv::dvbt {

namespace {

constexpr std::size_t response_bins = 4096;
constexpr std::uint32_t frame_symbols = 68;
constexpr std::size_t maximum_taps = 16;
constexpr double tap_floor_relative = 1e-3;  // -30 dB below the peak
constexpr double tap_floor_over_noise = 4.0; // +6 dB above the median bin

} // namespace

void ChannelProfiler::configure(const std::size_t fft_size,
                                const std::size_t maximum,
                                const double sample_rate_hz,
                                const std::size_t symbol_period_samples) {
    fft_size_ = fft_size;
    slots_ = (maximum / 3) + 1;
    sample_rate_hz_ = sample_rate_hz;
    period_ = symbol_period_samples;
    window_.resize(slots_);
    for (std::size_t j = 0; j < slots_; ++j) {
        window_[j] = static_cast<float>(
            0.5 -
            (0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double>(j) /
                            static_cast<double>(slots_ - 1))));
    }
    grid_.assign(response_bins, {});
    response_.assign(response_bins, {});
    previous_.assign(response_bins, {});
    power_.assign(response_bins, 0.0F);
    correlation_.assign(response_bins, {});
    pair_power_.assign(response_bins, 0.0F);
    plan_ =
        FftwfPlan::dft_1d(static_cast<int>(response_bins),
                          reinterpret_cast<fftwf_complex *>(grid_.data()),
                          reinterpret_cast<fftwf_complex *>(response_.data()),
                          FFTW_BACKWARD, FFTW_MEASURE);
    // FFTW_MEASURE scribbles on both arrays; reset() clears the grid.
    reset();
}

void ChannelProfiler::reset() noexcept {
    std::ranges::fill(grid_, std::complex<float>{});
    phase_mask_ = 0;
    have_previous_ = false;
    pair_contiguous_ = false;
    reset_frame();
}

void ChannelProfiler::reset_frame() noexcept {
    std::ranges::fill(power_, 0.0F);
    std::ranges::fill(correlation_, std::complex<float>{});
    std::ranges::fill(pair_power_, 0.0F);
    frame_symbols_ = 0;
    response_count_ = 0;
    pair_count_ = 0;
    frame_begin_ = 0;
    frame_end_ = 0;
}

void ChannelProfiler::add_symbol(
    const std::span<const std::complex<float>> inverse_channel, const int phase,
    const bool contiguous, const std::uint64_t symbol_start) {
    if (!plan_ || phase < 0 || phase > 3) {
        return;
    }
    if (!contiguous) {
        phase_mask_ = 0;
        pair_contiguous_ = false;
    }
    if (frame_symbols_ == 0) {
        frame_begin_ = symbol_start;
    }
    frame_end_ = symbol_start + period_;
    ++frame_symbols_;
    for (auto j = static_cast<std::size_t>(phase); j < slots_; j += 4) {
        const std::size_t k = 3 * j;
        const std::complex<float> inverse = k < inverse_channel.size()
                                                ? inverse_channel[k]
                                                : std::complex<float>{};
        grid_[j] =
            std::norm(inverse) > 0.0F
                ? window_[j] * (std::complex<float>{1.0F, 0.0F} / inverse)
                : std::complex<float>{};
    }
    phase_mask_ |= 1U << static_cast<unsigned>(phase);
    if (phase_mask_ == 0xFU) {
        compute_response();
        phase_mask_ = 0;
    }
}

void ChannelProfiler::compute_response() {
    plan_.execute();
    ++response_count_;
    if (have_previous_ && pair_contiguous_) {
        ++pair_count_;
        for (std::size_t i = 0; i < response_bins; ++i) {
            const std::complex<float> current = response_[i];
            const std::complex<float> previous = previous_[i];
            const float current_power = std::norm(current);
            power_[i] += current_power;
            correlation_[i] += current * std::conj(previous);
            pair_power_[i] += 0.5F * (current_power + std::norm(previous));
        }
    } else {
        for (std::size_t i = 0; i < response_bins; ++i) {
            power_[i] += std::norm(response_[i]);
        }
    }
    std::ranges::copy(response_, previous_.begin());
    have_previous_ = true;
    pair_contiguous_ = true;
}

std::optional<ChannelProfileTelemetry> ChannelProfiler::take_frame() {
    if (frame_symbols_ < frame_symbols) {
        return std::nullopt;
    }
    if (response_count_ == 0 || sample_rate_hz_ <= 0.0) {
        reset_frame();
        return std::nullopt;
    }
    ChannelProfileTelemetry profile;
    profile.resampled_begin_sample = frame_begin_;
    profile.resampled_end_sample = frame_end_;
    profile.symbol_count = frame_symbols_;
    profile.response_count = response_count_;
    profile.response_pair_count = pair_count_;
    const double useful_period_s =
        static_cast<double>(fft_size_) / sample_rate_hz_;
    const double window_s = useful_period_s / 3.0;
    const double bin_s = window_s / static_cast<double>(response_bins);
    profile.delay_window_us = window_s * 1e6;
    profile.delay_resolution_us = bin_s * 1e6;
    const double response_interval_s =
        4.0 * static_cast<double>(period_) / sample_rate_hz_;
    profile.response_interval_us = response_interval_s * 1e6;

    std::vector<double> pdp(response_bins);
    double peak = 0.0;
    std::size_t peak_bin = 0;
    for (std::size_t i = 0; i < response_bins; ++i) {
        pdp[i] = static_cast<double>(power_[i]) /
                 static_cast<double>(response_count_);
        if (pdp[i] > peak) {
            peak = pdp[i];
            peak_bin = i;
        }
    }
    if (peak <= 0.0) {
        reset_frame();
        return std::nullopt;
    }
    std::vector<double> sorted(pdp);
    std::ranges::nth_element(sorted, sorted.begin() + response_bins / 2);
    const double noise_floor = sorted[response_bins / 2];
    profile.noise_floor_db = 10.0 * std::log10(noise_floor / peak);
    const double threshold =
        std::max(peak * tap_floor_relative, noise_floor * tap_floor_over_noise);

    const auto delay_of = [](const std::size_t bin) {
        return bin <= response_bins / 2
                   ? static_cast<double>(bin)
                   : static_cast<double>(bin) -
                         static_cast<double>(response_bins);
    };
    const double peak_delay = delay_of(peak_bin);

    // Delay statistics over every bin above the tap threshold; the main
    // tap share covers the Hann main lobe around the strongest bin.
    const auto lobe_half_width = static_cast<std::size_t>(
        std::ceil(2.0 * static_cast<double>(response_bins) /
                  static_cast<double>(slots_)));
    double total = 0.0;
    double main = 0.0;
    double mean_delay = 0.0;
    double mean_square_delay = 0.0;
    double max_delay = peak_delay;
    for (std::size_t i = 0; i < response_bins; ++i) {
        if (pdp[i] < threshold) {
            continue;
        }
        const double delay = delay_of(i);
        total += pdp[i];
        mean_delay += pdp[i] * delay;
        mean_square_delay += pdp[i] * delay * delay;
        max_delay = std::max(max_delay, delay);
        const std::size_t distance =
            std::min((i + response_bins - peak_bin) % response_bins,
                     (peak_bin + response_bins - i) % response_bins);
        if (distance <= lobe_half_width) {
            main += pdp[i];
        }
    }
    mean_delay /= total;
    mean_square_delay /= total;
    profile.rms_delay_spread_us =
        std::sqrt(
            std::max(0.0, mean_square_delay - (mean_delay * mean_delay))) *
        bin_s * 1e6;
    profile.max_excess_delay_us = (max_delay - peak_delay) * bin_s * 1e6;
    profile.main_tap_energy_share = main / total;

    // Local maxima above the threshold, strongest first.
    std::vector<std::size_t> peaks;
    for (std::size_t i = 0; i < response_bins; ++i) {
        const double left = pdp[(i + response_bins - 1) % response_bins];
        const double right = pdp[(i + 1) % response_bins];
        if (pdp[i] >= threshold && pdp[i] >= left && pdp[i] > right) {
            peaks.push_back(i);
        }
    }
    std::ranges::sort(peaks, [&pdp](const std::size_t a, const std::size_t b) {
        return pdp[a] > pdp[b];
    });
    if (peaks.size() > maximum_taps) {
        peaks.resize(maximum_taps);
    }
    for (const std::size_t bin : peaks) {
        ChannelProfileTap tap;
        tap.delay_us = delay_of(bin) * bin_s * 1e6;
        tap.power_db = 10.0 * std::log10(pdp[bin] / peak);
        if (pair_count_ != 0 && pair_power_[bin] > 0.0F) {
            tap.doppler_hz = static_cast<double>(std::arg(correlation_[bin])) /
                             (2.0 * std::numbers::pi * response_interval_s);
            tap.coherence = static_cast<double>(std::abs(correlation_[bin]) /
                                                pair_power_[bin]);
        }
        profile.taps.push_back(tap);
    }
    reset_frame();
    return profile;
}

} // namespace airspy_tv::dvbt
