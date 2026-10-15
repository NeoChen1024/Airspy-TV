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

[[nodiscard]] float
timing_measurement_confidence(std::uint64_t accepted_measurements,
                              std::uint64_t attempted_measurements) noexcept;
[[nodiscard]] float
estimate_channel_notch_db(std::span<const std::complex<float>> inverse_channel,
                          std::size_t phase, std::size_t maximum);
[[nodiscard]] PilotPhaseScore
score_pilot_phase_at_offset(std::span<const std::complex<float>> fft,
                            std::size_t maximum, int offset, int phase,
                            std::optional<double> timing_tau);
// Pilot phase verification against the scattered-pilot grid: grid_values[j]
// is H at carrier 3 j as measured when the candidate phase last occurred
// (zero where unknown). Matching the received pilots to the known channel
// instead of to a timing ramp keeps the coherent sum intact in multipath,
// where H rotates several turns across a chunk of pilots and a single ramp
// cannot follow it.
[[nodiscard]] PilotPhaseScore score_pilot_phase_with_reference(
    std::span<const std::complex<float>> fft, std::size_t maximum, int offset,
    int phase, std::span<const std::complex<float>> grid_values);
[[nodiscard]] bool listed(std::span<const int> list, std::size_t value);

} // namespace airspy_tv::dvbt
