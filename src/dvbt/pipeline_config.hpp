#pragma once

#include "airspy_tv/dvbt/transport_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace airspy_tv::dvbt {

inline constexpr std::size_t acquisition_samples = 350'000;
inline constexpr std::size_t initial_symbol_queue_capacity = 256;
inline constexpr std::size_t ring_minimum_samples = 1'048'576;
inline constexpr std::size_t buffer_duration_denominator = 5;
inline constexpr std::size_t resampler_quantum_denominator = 20;
inline constexpr std::size_t minimum_sro_delay_denominator = 2;

struct WorkerAllocation {
    std::size_t resample{};
    std::size_t symbol{};
    std::size_t viterbi{};
};

[[nodiscard]] inline WorkerAllocation
allocate_workers(const std::size_t requested_threads) noexcept {
    const std::size_t total = requested_threads == 0
                                  ? default_viterbi_worker_count()
                                  : requested_threads;
    if (total <= 2) {
        return {1, 1, 1};
    }
    const std::size_t resample = std::max<std::size_t>(1, total / 4);
    const std::size_t remaining = total - resample;
    const std::size_t symbol = std::max<std::size_t>(1, remaining / 2);
    return {resample, symbol, std::max<std::size_t>(1, remaining - symbol)};
}

[[nodiscard]] inline std::size_t
buffered_symbol_count(const std::uint32_t bandwidth,
                      const std::size_t symbol_samples) noexcept {
    const std::uint64_t numerator = static_cast<std::uint64_t>(bandwidth) * 8U;
    const std::uint64_t denominator =
        7U * buffer_duration_denominator * symbol_samples;
    return std::max<std::size_t>(
        1,
        static_cast<std::size_t>((numerator + denominator - 1) / denominator));
}

[[nodiscard]] inline std::size_t
buffered_input_samples(const std::uint32_t sample_rate) noexcept {
    return std::max<std::size_t>(1, (static_cast<std::size_t>(sample_rate) +
                                     buffer_duration_denominator - 1) /
                                        buffer_duration_denominator);
}

[[nodiscard]] inline std::size_t
ring_capacity_for(const std::uint32_t sample_rate_hz) noexcept {
    return std::max(ring_minimum_samples,
                    (static_cast<std::size_t>(sample_rate_hz) +
                     buffer_duration_denominator - 1) /
                        buffer_duration_denominator);
}

[[nodiscard]] inline std::size_t
resampler_quantum_samples(const std::uint32_t sample_rate_hz) noexcept {
    return std::max<std::size_t>(1, (static_cast<std::size_t>(sample_rate_hz) +
                                     resampler_quantum_denominator - 1) /
                                        resampler_quantum_denominator);
}

[[nodiscard]] inline std::size_t
bootstrap_input_samples(const std::uint32_t input_rate_hz,
                        const std::uint32_t bandwidth_hz) noexcept {
    if (input_rate_hz == 0 || bandwidth_hz == 0) {
        return 0;
    }
    constexpr std::size_t preview_margin_samples = 16'384;
    const auto output_samples =
        static_cast<long double>(acquisition_samples + preview_margin_samples);
    const long double output_rate =
        static_cast<long double>(bandwidth_hz) * 8.0L / 7.0L;
    return static_cast<std::size_t>(
        std::ceil(output_samples * static_cast<long double>(input_rate_hz) /
                  output_rate));
}

[[nodiscard]] inline std::uint64_t
sro_fixed_delay_samples(const std::uint32_t input_rate_hz,
                        const std::uint32_t bandwidth_hz) noexcept {
    const std::uint64_t minimum = (static_cast<std::uint64_t>(input_rate_hz) +
                                   minimum_sro_delay_denominator - 1) /
                                  minimum_sro_delay_denominator;
    if (bandwidth_hz == 0) {
        return minimum;
    }
    const long double output_rate =
        static_cast<long double>(bandwidth_hz) * 8.0L / 7.0L;
    const long double ring_input_equivalent =
        static_cast<long double>(ring_capacity_for(input_rate_hz)) *
        static_cast<long double>(input_rate_hz) / output_rate;
    const long double bounded_lead =
        std::ceil(ring_input_equivalent) +
        (2.0L *
         static_cast<long double>(resampler_quantum_samples(input_rate_hz)));
    return std::max(minimum, static_cast<std::uint64_t>(bounded_lead));
}

} // namespace airspy_tv::dvbt
