#pragma once

#include "airspy_tv/dvbt/receiver_parameters.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace airspy_tv::dvbt {

struct PilotPhaseScore {
    float score{};
    float confidence{};
};

[[nodiscard]] const char *event_mode_name(TransmissionMode mode);
[[nodiscard]] const char *event_guard_name(GuardInterval guard);
[[nodiscard]] const char *event_constellation_name(Constellation constellation);
[[nodiscard]] const char *event_code_rate_name(CodeRate rate);

[[nodiscard]] std::size_t generate_scattered_timing_estimates(
    std::span<const std::complex<float>> channel, std::size_t phase,
    std::size_t maximum, std::size_t fft_size, std::span<double> estimates);
[[nodiscard]] std::optional<double>
select_exact_median(std::span<double> values);
[[nodiscard]] float timing_measurement_confidence(
    std::uint64_t accepted_measurements,
    std::uint64_t attempted_measurements) noexcept;
[[nodiscard]] float
estimate_channel_notch_db(std::span<const std::complex<float>> inverse_channel,
                          std::size_t phase, std::size_t maximum);
[[nodiscard]] int lock_phase_at_offset(std::span<const std::complex<float>> fft,
                                       std::size_t maximum, int offset,
                                       std::optional<double> timing_tau);
[[nodiscard]] PilotPhaseScore score_pilot_phase_at_offset(
    std::span<const std::complex<float>> fft, std::size_t maximum, int offset,
    int phase, std::optional<double> timing_tau);
[[nodiscard]] bool listed(std::span<const int> list, std::size_t value);

} // namespace airspy_tv::dvbt
