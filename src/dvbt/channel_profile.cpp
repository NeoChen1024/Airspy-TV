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
constexpr double placement_peak_share = 0.05;
constexpr float placement_tap_relative = 1e-2F; // -20 dB below the peak
constexpr float extent_tap_relative = 1e-5F;    // -50 dB below the peak
constexpr float extent_tap_over_noise = 32.0F;  // +15 dB above the median bin

} // namespace

void ChannelProfiler::configure(const std::size_t fft_size,
                                const std::size_t guard_size,
                                const std::size_t maximum,
                                const double sample_rate_hz) {
    fft_size_ = fft_size;
    guard_size_ = guard_size;
    slots_ = (maximum / 3) + 1;
    sample_rate_hz_ = sample_rate_hz;
    period_ = fft_size + guard_size;
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
    symbols_since_response_ = 0;
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
    const std::span<const std::complex<float>> grid_values, const bool complete,
    const bool contiguous, const std::uint64_t symbol_start) {
    if (!plan_) {
        return;
    }
    if (!contiguous) {
        symbols_since_response_ = 0;
        pair_contiguous_ = false;
    }
    if (frame_symbols_ == 0) {
        frame_begin_ = symbol_start;
    }
    frame_end_ = symbol_start + period_;
    ++frame_symbols_;
    if (!complete || ++symbols_since_response_ < 4) {
        return;
    }
    symbols_since_response_ = 0;
    for (std::size_t j = 0; j < slots_ && j < grid_values.size(); ++j) {
        grid_[j] = window_[j] * grid_values[j];
    }
    compute_response();
}

std::span<const float> ChannelProfiler::response_energy(
    const std::span<const std::complex<float>> grid_values) {
    if (!plan_) {
        return {};
    }
    for (std::size_t j = 0; j < slots_ && j < grid_values.size(); ++j) {
        grid_[j] = window_[j] * grid_values[j];
    }
    plan_.execute();
    if (energy_.size() != response_bins) {
        energy_.resize(response_bins);
    }
    for (std::size_t i = 0; i < response_bins; ++i) {
        energy_[i] = std::norm(response_[i]);
    }
    return energy_;
}

ChannelProfiler::ResponseSummary ChannelProfiler::response_summary(
    const std::span<const std::complex<float>> grid_values) {
    ResponseSummary summary;
    const auto energy = response_energy(grid_values);
    const std::size_t n = energy.size();
    if (n == 0) {
        return summary;
    }
    double total = 0.0;
    std::size_t peak = 0;
    float peak_energy = -1.0F;
    for (std::size_t i = 0; i < n; ++i) {
        total += energy[i];
        if (energy[i] > peak_energy) {
            peak_energy = energy[i];
            peak = i;
        }
    }
    if (total <= 0.0) {
        return summary;
    }
    summary.peak_share = static_cast<double>(peak_energy) / total;
    // Only trust a structured response: the peak tap must hold a
    // meaningful share of the energy, so a noise-driven response cannot
    // drag the window or the timing loop.
    if (summary.peak_share <= placement_peak_share) {
        return summary;
    }
    const float threshold = peak_energy * placement_tap_relative;
    if (scratch_.size() != n) {
        scratch_.resize(n);
    }
    std::ranges::copy(energy, scratch_.begin());
    const auto median = scratch_.begin() + static_cast<std::ptrdiff_t>(n / 2);
    std::ranges::nth_element(scratch_, median);
    const float extent_threshold = std::max(peak_energy * extent_tap_relative,
                                            *median * extent_tap_over_noise);
    const auto half = static_cast<std::ptrdiff_t>(n / 2);
    double weighted = 0.0;
    double weight = 0.0;
    std::ptrdiff_t first = 0;
    std::ptrdiff_t last = 0;
    std::ptrdiff_t extent_first = 0;
    std::ptrdiff_t extent_last = 0;
    for (std::ptrdiff_t offset = -half + 1; offset <= half; ++offset) {
        const std::size_t i =
            (peak + n + static_cast<std::size_t>(offset + half)) % n;
        const std::size_t index = (i + n - static_cast<std::size_t>(half)) % n;
        const float value = energy[index];
        if (value < extent_threshold) {
            continue;
        }
        extent_first = std::min(extent_first, offset);
        extent_last = std::max(extent_last, offset);
        if (value < threshold) {
            continue;
        }
        weighted += static_cast<double>(value) * static_cast<double>(offset);
        weight += value;
        first = std::min(first, offset);
        last = std::max(last, offset);
    }
    const double peak_bin =
        peak > n / 2 ? static_cast<double>(peak) - static_cast<double>(n)
                     : static_cast<double>(peak);
    summary.valid = true;
    summary.peak_bin = peak_bin;
    summary.first_bin = peak_bin + static_cast<double>(first);
    summary.last_bin = peak_bin + static_cast<double>(last);
    summary.extent_first_bin = peak_bin + static_cast<double>(extent_first);
    summary.extent_last_bin = peak_bin + static_cast<double>(extent_last);
    summary.centroid_bin = peak_bin + (weighted / weight);
    return summary;
}

double ChannelProfiler::bin_samples(const std::size_t fft_size) noexcept {
    return static_cast<double>(fft_size) /
           static_cast<double>(3 * response_bins);
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

    // The response is periodic in Tu/3; report delays in the period centred
    // on the middle of the guard interval, where the window placement keeps
    // the significant paths, so a late echo inside the guard does not show
    // up as a pre-echo.
    const double centre_bin =
        static_cast<double>(guard_size_) / (2.0 * bin_samples(fft_size_));
    const auto delay_of = [centre_bin](const std::size_t bin) {
        double delay = static_cast<double>(bin);
        const auto bins = static_cast<double>(response_bins);
        while (delay - centre_bin > bins / 2.0) {
            delay -= bins;
        }
        while (delay - centre_bin <= -bins / 2.0) {
            delay += bins;
        }
        return delay;
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
