#include "channel_locator.hpp"

#include "airspy_tv/fftw_plan.hpp"

#include <fftw3.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr std::size_t fft_size = 4096;
// Fewer frames than this say nothing reliable about a noisy band.
constexpr std::size_t minimum_frames = 8;
constexpr float minimum_power = 1.0e-20F;
// The occupied width is the active carriers: 6817 of 8192 at 8/7 the channel
// bandwidth in sample rate.
constexpr double occupied_fraction = 6817.0 * 8.0 / (7.0 * 8192.0);
// Bins averaged on either side of an edge when checking the drop and when
// reading the half-power crossing.
constexpr std::size_t side_bins = 12;
constexpr std::size_t crossing_bins = 3;
// The band beside each edge must sit at least this far below the channel.
constexpr float minimum_edge_contrast_db = 6.0F;

[[nodiscard]] float median_of(std::vector<float> values) {
    if (values.empty()) {
        return 0.0F;
    }
    auto middle =
        values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::ranges::nth_element(values, middle);
    return *middle;
}

[[nodiscard]] float mean_of(const std::span<const float> values) {
    float sum = 0.0F;
    for (const float value : values) {
        sum += value;
    }
    return values.empty() ? 0.0F : sum / static_cast<float>(values.size());
}

[[nodiscard]] float to_db(const float power) {
    return 10.0F * std::log10(std::max(power, minimum_power));
}

} // namespace

ChannelLocation
locate_channel(const std::span<const std::complex<float>> samples,
               const std::uint32_t sample_rate_hz,
               const std::uint32_t channel_bandwidth_hz) {
    ChannelLocation result;
    if (sample_rate_hz == 0 || channel_bandwidth_hz == 0 ||
        samples.size() < fft_size * minimum_frames) {
        return result;
    }
    const double bin_hz = static_cast<double>(sample_rate_hz) / fft_size;
    const auto width = static_cast<std::size_t>(
        std::lround(occupied_fraction * channel_bandwidth_hz / bin_hz));
    // Room for the channel plus the side bands the edge test looks at.
    if (width < 4 * side_bins || width + (2 * side_bins) >= fft_size) {
        return result;
    }

    // Averaged power spectrum, Hann windowed, bin 0 at -rate/2.
    std::vector<std::complex<float>> fft_input(fft_size);
    std::vector<std::complex<float>> fft_output(fft_size);
    static_assert(sizeof(std::complex<float>) == sizeof(fftwf_complex));
    auto plan =
        FftwfPlan::dft_1d(static_cast<int>(fft_size),
                          reinterpret_cast<fftwf_complex *>(fft_input.data()),
                          reinterpret_cast<fftwf_complex *>(fft_output.data()),
                          FFTW_FORWARD, FFTW_ESTIMATE);
    std::vector<float> window(fft_size);
    for (std::size_t index = 0; index < fft_size; ++index) {
        window[index] =
            0.5F - (0.5F * std::cos((2.0F * std::numbers::pi_v<float> *
                                     static_cast<float>(index)) /
                                    static_cast<float>(fft_size)));
    }
    std::vector<float> power(fft_size, 0.0F);
    const std::size_t frames = samples.size() / fft_size;
    for (std::size_t frame = 0; frame < frames; ++frame) {
        const auto block = samples.subspan(frame * fft_size, fft_size);
        for (std::size_t index = 0; index < fft_size; ++index) {
            fft_input[index] = block[index] * window[index];
        }
        plan.execute();
        for (std::size_t index = 0; index < fft_size; ++index) {
            power[(index + (fft_size / 2)) % fft_size] +=
                std::norm(fft_output[index]);
        }
    }

    // The window of the occupied width holding the most power.
    float window_sum = 0.0F;
    for (std::size_t index = 0; index < width; ++index) {
        window_sum += power[index];
    }
    float best_sum = window_sum;
    std::size_t best_start = 0;
    for (std::size_t start = 1; start + width <= fft_size; ++start) {
        window_sum += power[(start + width) - 1] - power[start - 1];
        if (window_sum > best_sum) {
            best_sum = window_sum;
            best_start = start;
        }
    }
    const std::size_t best_end = best_start + width;
    // Both side bands must fit inside the source band.
    if (best_start < side_bins || best_end + side_bins > fft_size) {
        return result;
    }

    const float level = median_of(std::vector<float>(
        power.begin() + static_cast<std::ptrdiff_t>(best_start),
        power.begin() + static_cast<std::ptrdiff_t>(best_end)));
    const float left_side =
        mean_of(std::span{power}.subspan(best_start - side_bins, side_bins));
    const float right_side =
        mean_of(std::span{power}.subspan(best_end, side_bins));
    const float contrast_db =
        to_db(level) - std::max(to_db(left_side), to_db(right_side));
    if (contrast_db < minimum_edge_contrast_db) {
        return result;
    }

    // Half-power crossings nearest the window edges, by short averages.
    const auto crossing = [&](const std::size_t edge, const bool rising) {
        const float half = 0.5F * level;
        const std::ptrdiff_t step = rising ? 1 : -1;
        auto index = static_cast<std::ptrdiff_t>(edge);
        const auto limit_low = static_cast<std::ptrdiff_t>(crossing_bins);
        const auto limit_high =
            static_cast<std::ptrdiff_t>(fft_size - crossing_bins);
        const auto local = [&](const std::ptrdiff_t at) {
            return mean_of(std::span{power}.subspan(
                static_cast<std::size_t>(
                    at - static_cast<std::ptrdiff_t>(crossing_bins / 2)),
                crossing_bins));
        };
        // Move inward while below half power, outward while above it.
        if (local(index) < half) {
            while (index + step > limit_low && index + step < limit_high &&
                   local(index + step) < half) {
                index += step;
            }
        } else {
            while (index - step > limit_low && index - step < limit_high &&
                   local(index - step) >= half) {
                index -= step;
            }
        }
        return static_cast<double>(index);
    };
    const double left = crossing(best_start, true);
    const double right = crossing(best_end, false);
    const double centre_bin = 0.5 * (left + right);
    constexpr double centre_of_band = static_cast<double>(fft_size) / 2.0;
    result.offset_hz = (centre_bin - centre_of_band) * bin_hz;
    result.edge_contrast_db = contrast_db;
    result.found = true;
    return result;
}

} // namespace airspy_tv::dvbt
