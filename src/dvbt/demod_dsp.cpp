#include "demod_dsp.hpp"

#include "ofdm_carrier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float minimum_power = 1.0e-12F;
constexpr std::size_t timing_pilot_spacing = 12;

} // namespace

const char *event_mode_name(const TransmissionMode mode) {
    return mode == TransmissionMode::k8 ? "8k" : "2k";
}

const char *event_guard_name(const GuardInterval guard) {
    switch (guard) {
    case GuardInterval::gi_1_32:
        return "1/32";
    case GuardInterval::gi_1_16:
        return "1/16";
    case GuardInterval::gi_1_8:
        return "1/8";
    case GuardInterval::gi_1_4:
        return "1/4";
    }
    return "unknown";
}

const char *event_constellation_name(const Constellation constellation) {
    switch (constellation) {
    case Constellation::qpsk:
        return "qpsk";
    case Constellation::qam16:
        return "qam16";
    case Constellation::qam64:
        return "qam64";
    }
    return "unknown";
}

const char *event_code_rate_name(const CodeRate rate) {
    switch (rate) {
    case CodeRate::rate_1_2:
        return "1/2";
    case CodeRate::rate_2_3:
        return "2/3";
    case CodeRate::rate_3_4:
        return "3/4";
    case CodeRate::rate_5_6:
        return "5/6";
    case CodeRate::rate_7_8:
        return "7/8";
    }
    return "unknown";
}

std::optional<double> estimate_scattered_timing_tau(
    const std::span<const std::complex<float>> channel, const std::size_t phase,
    const std::size_t maximum, const std::size_t fft_size) {
    std::array<double, 1024> estimates{};
    std::size_t estimate_count = 0;
    const std::size_t first = phase * 3;
    for (std::size_t left = first; left + timing_pilot_spacing <= maximum;
         left += timing_pilot_spacing) {
        const std::size_t right = left + timing_pilot_spacing;
        if (std::norm(channel[left]) <= minimum_power ||
            std::norm(channel[right]) <= minimum_power ||
            estimate_count == estimates.size()) {
            continue;
        }
        const double slope =
            std::arg(channel[right] * std::conj(channel[left])) /
            static_cast<double>(timing_pilot_spacing);
        if (std::isfinite(slope)) {
            estimates[estimate_count++] = slope *
                                          static_cast<double>(fft_size) /
                                          (2.0 * std::numbers::pi_v<double>);
        }
    }
    if (estimate_count == 0) {
        return std::nullopt;
    }
    std::ranges::sort(estimates.begin(),
                      estimates.begin() +
                          static_cast<std::ptrdiff_t>(estimate_count));
    const std::size_t middle = estimate_count / 2;
    if (estimate_count % 2 != 0) {
        return estimates[middle];
    }
    return 0.5 * (estimates[middle - 1] + estimates[middle]);
}

float estimate_channel_notch_db(
    const std::span<const std::complex<float>> inverse_channel,
    const std::size_t phase, const std::size_t maximum) {
    std::vector<float> channel_db;
    channel_db.reserve((maximum / timing_pilot_spacing) + 1);
    const std::size_t edge_guard = maximum / 32;
    for (std::size_t carrier_index = phase * 3; carrier_index <= maximum;
         carrier_index += timing_pilot_spacing) {
        const float inverse_power = std::norm(inverse_channel[carrier_index]);
        if (carrier_index >= edge_guard &&
            carrier_index + edge_guard <= maximum &&
            inverse_power > minimum_power) {
            channel_db.push_back(-10.0F * std::log10(inverse_power));
        }
    }
    if (channel_db.empty()) {
        return 0.0F;
    }
    auto baseline = channel_db;
    auto median =
        baseline.begin() + static_cast<std::ptrdiff_t>(baseline.size() / 2);
    std::ranges::nth_element(baseline, median);
    const std::size_t lower_index =
        std::min(channel_db.size() - 1,
                 std::max<std::size_t>(1, channel_db.size() / 100));
    auto lower = channel_db.begin() + static_cast<std::ptrdiff_t>(lower_index);
    std::ranges::nth_element(channel_db, lower);
    return *lower - *median;
}

int lock_phase_at_offset(const std::span<const std::complex<float>> fft,
                         const std::size_t maximum, const int offset,
                         const std::optional<double> timing_tau) {
    int best_phase = 0;
    float best_score = -1.0F;
    for (int phase = 0; phase < 4; ++phase) {
        std::complex<float> correlation{};
        float score = 0.0F;
        std::size_t chunk_count = 0;
        double dephase_slope = 0.0;
        if (timing_tau.has_value()) {
            dephase_slope = 2.0 * std::numbers::pi_v<double> * *timing_tau /
                            static_cast<double>(fft.size());
        } else {
            double ramp_sum = 0.0;
            std::size_t ramp_count = 0;
            std::size_t previous_pilot =
                std::numeric_limits<std::size_t>::max();
            for (auto pilot = static_cast<std::size_t>(phase) * 3U;
                 pilot <= maximum; pilot += 12) {
                if (previous_pilot != std::numeric_limits<std::size_t>::max()) {
                    const auto left =
                        active_carrier(fft, previous_pilot, maximum, offset);
                    const auto right =
                        active_carrier(fft, pilot, maximum, offset);
                    const float left_value = pilot_prbs[previous_pilot] == 0U
                                                 ? 4.0F / 3.0F
                                                 : -4.0F / 3.0F;
                    const float right_value =
                        pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                    if (std::norm(left) > 0.0F && std::norm(right) > 0.0F) {
                        const double difference =
                            std::arg(right * std::conj(left) *
                                     std::complex<float>(
                                         right_value * left_value, 0.0F));
                        if (std::isfinite(difference)) {
                            ramp_sum += difference;
                            ++ramp_count;
                        }
                    }
                }
                previous_pilot = pilot;
            }
            dephase_slope =
                ramp_count != 0
                    ? -ramp_sum /
                          static_cast<double>(ramp_count * timing_pilot_spacing)
                    : 0.0;
        }
        for (auto pilot = static_cast<std::size_t>(phase) * 3U;
             pilot <= maximum; pilot += 12) {
            const float value =
                pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
            const auto dephase =
                static_cast<float>(dephase_slope * static_cast<double>(pilot));
            correlation +=
                value * std::conj(std::polar(1.0F, dephase) *
                                  active_carrier(fft, pilot, maximum, offset));
            if (++chunk_count == 8) {
                score += std::norm(correlation);
                correlation = {};
                chunk_count = 0;
            }
        }
        score += std::norm(correlation);
        if (score > best_score) {
            best_score = score;
            best_phase = phase;
        }
    }
    return best_phase;
}

bool listed(const std::span<const int> list, const std::size_t value) {
    return std::ranges::binary_search(list, static_cast<int>(value));
}

} // namespace airspy_tv::dvbt
