#pragma once

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace airspy_tv::dvbt {

struct PilotLock {
    int phase{};
    int offset{};
};

[[nodiscard]] constexpr std::array<std::uint8_t, 6817> make_pilot_prbs() {
    std::array<std::uint8_t, 6817> result{};
    std::uint32_t state = 0x7ffU;
    for (auto &bit : result) {
        bit = static_cast<std::uint8_t>(state & 1U);
        state = (state >> 1U) | ((((state >> 2U) ^ state) & 1U) << 10U);
    }
    return result;
}

inline constexpr auto pilot_prbs = make_pilot_prbs();

[[nodiscard]] inline std::complex<float>
active_carrier(const std::span<const std::complex<float>> fft,
               const std::size_t index, const std::size_t maximum,
               const int offset = 0) {
    const auto bin = static_cast<std::ptrdiff_t>(index) -
                     static_cast<std::ptrdiff_t>(maximum / 2) + offset;
    const auto wrapped = (bin + static_cast<std::ptrdiff_t>(fft.size())) %
                         static_cast<std::ptrdiff_t>(fft.size());
    return fft[static_cast<std::size_t>(wrapped)];
}

[[nodiscard]] inline PilotLock
lock_pilots_at_offset(const std::span<const std::complex<float>> fft,
                      const std::size_t maximum, const int offset,
                      const std::size_t maximum_pilots =
                          std::numeric_limits<std::size_t>::max()) {
    PilotLock best{0, offset};
    float best_score = -1.0F;
    for (int phase = 0; phase < 4; ++phase) {
        std::complex<float> correlation{};
        float score = 0.0F;
        std::size_t chunk_count = 0;
        std::size_t pilot_count = 0;
        for (auto pilot = static_cast<std::size_t>(phase) * 3U;
             pilot <= maximum && pilot_count < maximum_pilots;
             pilot += 12, ++pilot_count) {
            const float value =
                pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
            correlation +=
                value * std::conj(active_carrier(fft, pilot, maximum, offset));
            if (++chunk_count == 8) {
                score += std::norm(correlation);
                correlation = {};
                chunk_count = 0;
            }
        }
        score += std::norm(correlation);
        if (score > best_score) {
            best_score = score;
            best.phase = phase;
        }
    }
    return best;
}

[[nodiscard]] inline PilotLock
lock_pilots(const std::span<const std::complex<float>> fft,
            const std::size_t maximum,
            const int previous_offset = std::numeric_limits<int>::max(),
            const std::size_t maximum_pilots =
                std::numeric_limits<std::size_t>::max()) {
    PilotLock best;
    float best_score = -1.0F;
    const int radius =
        previous_offset == std::numeric_limits<int>::max() ? 48 : 2;
    const int center = previous_offset == std::numeric_limits<int>::max()
                           ? 0
                           : previous_offset;
    for (int offset = center - radius; offset <= center + radius; ++offset) {
        const PilotLock candidate =
            lock_pilots_at_offset(fft, maximum, offset, maximum_pilots);
        std::complex<float> correlation{};
        float score = 0.0F;
        std::size_t chunk_count = 0;
        std::size_t pilot_count = 0;
        for (auto pilot = static_cast<std::size_t>(candidate.phase) * 3U;
             pilot <= maximum && pilot_count < maximum_pilots;
             pilot += 12, ++pilot_count) {
            const float value =
                pilot_prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
            correlation +=
                value * std::conj(active_carrier(fft, pilot, maximum, offset));
            if (++chunk_count == 8) {
                score += std::norm(correlation);
                correlation = {};
                chunk_count = 0;
            }
        }
        score += std::norm(correlation);
        if (score > best_score) {
            best_score = score;
            best = candidate;
        }
    }
    return best;
}

} // namespace airspy_tv::dvbt
