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
#include <stdexcept>
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

float timing_measurement_confidence(
    const std::uint64_t accepted_measurements,
    const std::uint64_t attempted_measurements) noexcept {
    if (attempted_measurements == 0) {
        return 0.0F;
    }
    return std::clamp(static_cast<float>(accepted_measurements) /
                          static_cast<float>(attempted_measurements),
                      0.0F, 1.0F);
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

namespace {

// Coherent pilot correlation in chunks of eight pilots: a correct phase sums
// coherently inside each chunk, a wrong one (data carriers) does not.
template <typename Sampler>
PilotPhaseScore accumulate_pilot_score(const std::size_t phase,
                                       const std::size_t maximum,
                                       Sampler &&sample_at) {
    std::complex<float> correlation{};
    float score = 0.0F;
    double normalization = 0.0;
    double chunk_power = 0.0;
    std::size_t chunk_count = 0;
    for (std::size_t pilot = phase * 3U; pilot <= maximum;
         pilot += timing_pilot_spacing) {
        const std::complex<float> sample = sample_at(pilot);
        correlation += sample;
        chunk_power += std::norm(sample);
        if (++chunk_count == 8) {
            score += std::norm(correlation);
            normalization += static_cast<double>(chunk_count) * chunk_power;
            correlation = {};
            chunk_power = 0.0;
            chunk_count = 0;
        }
    }
    score += std::norm(correlation);
    normalization += static_cast<double>(chunk_count) * chunk_power;
    return {.score = score,
            .confidence = normalization > 0.0
                              ? static_cast<float>(static_cast<double>(score) /
                                                   normalization)
                              : 0.0F};
}

} // namespace

PilotPhaseScore score_pilot_phase_at_offset(
    const std::span<const std::complex<float>> fft, const std::size_t maximum,
    const int offset, const int phase, const std::optional<double> timing_tau) {
    if (phase < 0 || phase >= 4) {
        throw std::invalid_argument("pilot phase is outside [0, 4)");
    }
    double dephase_slope = 0.0;
    if (timing_tau.has_value()) {
        dephase_slope = 2.0 * std::numbers::pi_v<double> * *timing_tau /
                        static_cast<double>(fft.size());
    } else {
        double ramp_sum = 0.0;
        std::size_t ramp_count = 0;
        std::size_t previous_pilot = std::numeric_limits<std::size_t>::max();
        for (auto pilot = static_cast<std::size_t>(phase) * 3U;
             pilot <= maximum; pilot += timing_pilot_spacing) {
            if (previous_pilot != std::numeric_limits<std::size_t>::max()) {
                const auto left =
                    active_carrier(fft, previous_pilot, maximum, offset);
                const auto right = active_carrier(fft, pilot, maximum, offset);
                const float left_value = pilot_prbs[previous_pilot] == 0U
                                             ? 4.0F / 3.0F
                                             : -4.0F / 3.0F;
                const float right_value =
                    pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                if (std::norm(left) > 0.0F && std::norm(right) > 0.0F) {
                    const double difference = std::arg(
                        right * std::conj(left) *
                        std::complex<float>(right_value * left_value, 0.0F));
                    if (std::isfinite(difference)) {
                        ramp_sum += difference;
                        ++ramp_count;
                    }
                }
            }
            previous_pilot = pilot;
        }
        dephase_slope = ramp_count != 0
                            ? -ramp_sum / static_cast<double>(
                                              ramp_count * timing_pilot_spacing)
                            : 0.0;
    }

    return accumulate_pilot_score(
        static_cast<std::size_t>(phase), maximum, [&](const std::size_t pilot) {
            const float value =
                pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
            const auto dephase =
                static_cast<float>(dephase_slope * static_cast<double>(pilot));
            return value *
                   std::conj(std::polar(1.0F, dephase) *
                             active_carrier(fft, pilot, maximum, offset));
        });
}

PilotPhaseScore score_pilot_phase_with_reference(
    const std::span<const std::complex<float>> fft, const std::size_t maximum,
    const int offset, const int phase,
    const std::span<const std::complex<float>> grid_values) {
    if (phase < 0 || phase >= 4) {
        throw std::invalid_argument("pilot phase is outside [0, 4)");
    }
    return accumulate_pilot_score(
        static_cast<std::size_t>(phase), maximum, [&](const std::size_t pilot) {
            const std::size_t slot = pilot / 3;
            const std::complex<float> response = slot < grid_values.size()
                                                     ? grid_values[slot]
                                                     : std::complex<float>{};
            if (std::norm(response) <= 0.0F) {
                return std::complex<float>{};
            }
            const float value =
                pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
            // Matched filter: conj(H) weights each pilot by its channel
            // power, so notched pilots do not dominate the chunk.
            return value * active_carrier(fft, pilot, maximum, offset) *
                   std::conj(response);
        });
}

bool listed(const std::span<const int> list, const std::size_t value) {
    return std::ranges::binary_search(list, static_cast<int>(value));
}

} // namespace airspy_tv::dvbt
