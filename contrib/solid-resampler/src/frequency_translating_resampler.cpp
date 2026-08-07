/*
 * Filter-design and dot-product portions are derived from liquid-dsp.
 * Copyright (c) 2007 - 2026 Joseph Gaeddert
 * SPDX-License-Identifier: MIT
 */

#include "solid_resampler/frequency_translating_resampler.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <pthread.h>
#endif

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif

namespace solid_resampler {
namespace {

constexpr std::uint64_t q32_one = std::uint64_t{1} << 32U;
constexpr long double q64_turn = 18446744073709551616.0L;
constexpr std::size_t minimum_parallel_outputs = 4096;
constexpr std::size_t filter_tap_alignment = 16;
constexpr std::size_t minimum_filter_taps = 16;
constexpr std::size_t maximum_filter_taps = 4096;

[[nodiscard]] bool is_power_of_two(const unsigned int value) noexcept {
    return value != 0U && (value & (value - 1U)) == 0U;
}

[[nodiscard]] unsigned int log2_power_of_two(unsigned int value) noexcept {
    unsigned int bits = 0;
    while (value > 1U) {
        value >>= 1U;
        ++bits;
    }
    return bits;
}

[[nodiscard]] std::size_t round_up(const std::size_t value,
                                   const std::size_t alignment) {
    if (value > std::numeric_limits<std::size_t>::max() - (alignment - 1U)) {
        throw std::length_error("arbitrary resampler filter is too large");
    }
    return (value + alignment - 1U) / alignment * alignment;
}

[[nodiscard]] std::size_t estimate_filter_taps(const ResamplerConfig &config) {
    const double transition =
        (config.stopband_edge_hz - config.passband_edge_hz) /
        config.input_rate_hz;
    const double delta_omega = 2.0 * std::numbers::pi * transition;
    const double estimated_order =
        (static_cast<double>(config.stopband_attenuation_db) - 8.0) /
        (2.285 * delta_omega);
    if (!std::isfinite(estimated_order) || estimated_order < 0.0 ||
        estimated_order >= static_cast<double>(maximum_filter_taps)) {
        throw std::length_error(
            "arbitrary resampler transition requires too many filter taps");
    }
    const auto raw_taps =
        static_cast<std::size_t>(std::ceil(estimated_order)) + 1U;
    const std::size_t aligned_taps =
        round_up(std::max(raw_taps, minimum_filter_taps), filter_tap_alignment);
    if (aligned_taps > maximum_filter_taps - filter_tap_alignment) {
        throw std::length_error(
            "arbitrary resampler transition requires too many filter taps");
    }
    // Kaiser order estimates are approximate. One additional SIMD block keeps
    // polyphase and coefficient-quantization variation away from the target.
    return aligned_taps + filter_tap_alignment;
}

[[nodiscard]] double normalized_sinc(const double value) noexcept {
    if (std::abs(value) < 1.0e-12) {
        return 1.0;
    }
    const double angle = std::numbers::pi * value;
    return std::sin(angle) / angle;
}

[[nodiscard]] double kaiser_beta(const float attenuation_db) noexcept {
    const double attenuation = std::abs(static_cast<double>(attenuation_db));
    if (attenuation > 50.0) {
        return 0.1102 * (attenuation - 8.7);
    }
    if (attenuation > 21.0) {
        return 0.5842 * std::pow(attenuation - 21.0, 0.4) +
               0.07886 * (attenuation - 21.0);
    }
    return 0.0;
}

[[nodiscard]] std::vector<float>
design_polyphase_bank(const unsigned int semi_length,
                      const unsigned int phase_count, const float cutoff,
                      const float attenuation_db) {
    const std::size_t tap_count = std::size_t{2} * semi_length;
    const std::size_t prototype_count = tap_count * phase_count + 1U;
    std::vector<double> prototype(prototype_count);
    const double beta = kaiser_beta(attenuation_db);
    const double denominator = std::cyl_bessel_i(0.0, beta);
    const double prototype_cutoff =
        static_cast<double>(cutoff) / static_cast<double>(phase_count);
    double gain = 0.0;
    for (std::size_t index = 0; index < prototype_count; ++index) {
        const double centered = static_cast<double>(index) -
                                static_cast<double>(prototype_count - 1U) / 2.0;
        const double radius =
            2.0 * centered / static_cast<double>(prototype_count - 1U);
        const double window =
            std::cyl_bessel_i(
                0.0, beta * std::sqrt(std::max(0.0, 1.0 - radius * radius))) /
            denominator;
        const double coefficient =
            normalized_sinc(2.0 * prototype_cutoff * centered) * window;
        prototype[index] = coefficient;
        gain += coefficient;
    }
    const double normalization = static_cast<double>(phase_count) / gain;

    // Coefficients are repeated for I/Q so SIMD kernels can multiply the
    // interleaved complex input directly. Each phase is time-reversed to match
    // a window ordered from oldest to newest sample.
    std::vector<float> bank(phase_count * tap_count * 2U);
    for (std::size_t phase = 0; phase < phase_count; ++phase) {
        float *const destination = bank.data() + phase * tap_count * 2U;
        for (std::size_t tap = 0; tap < tap_count; ++tap) {
            const std::size_t source =
                phase + (tap_count - tap - 1U) * phase_count;
            const float coefficient =
                static_cast<float>(prototype[source] * normalization);
            destination[2U * tap] = coefficient;
            destination[2U * tap + 1U] = coefficient;
        }
    }
    return bank;
}

using DotProduct = std::complex<float> (*)(const std::complex<float> *,
                                           const float *, std::size_t) noexcept;

[[nodiscard]] std::complex<float>
dot_product_portable(const std::complex<float> *const input,
                     const float *const coefficients,
                     const std::size_t tap_count) noexcept {
    const float *const values = reinterpret_cast<const float *>(input);
    float real = 0.0F;
    float imag = 0.0F;
    for (std::size_t tap = 0; tap < tap_count; ++tap) {
        real += values[2U * tap] * coefficients[2U * tap];
        imag += values[2U * tap + 1U] * coefficients[2U * tap + 1U];
    }
    return {real, imag};
}

#if defined(__x86_64__) || defined(__i386__)
__attribute__((target("sse4.1"))) [[nodiscard]] std::complex<float>
dot_product_sse41(const std::complex<float> *const input,
                  const float *const coefficients,
                  const std::size_t tap_count) noexcept {
    const float *const values = reinterpret_cast<const float *>(input);
    const std::size_t scalar_count = tap_count * 2U;
    const std::size_t vector_count = scalar_count & ~std::size_t{3};
    __m128 sum = _mm_setzero_ps();
    for (std::size_t index = 0; index < vector_count; index += 4U) {
        sum = _mm_add_ps(sum, _mm_mul_ps(_mm_loadu_ps(values + index),
                                         _mm_loadu_ps(coefficients + index)));
    }
    alignas(16) float lanes[4];
    _mm_store_ps(lanes, sum);
    float real = lanes[0] + lanes[2];
    float imag = lanes[1] + lanes[3];
    for (std::size_t index = vector_count; index < scalar_count; index += 2U) {
        real += values[index] * coefficients[index];
        imag += values[index + 1U] * coefficients[index + 1U];
    }
    return {real, imag};
}

__attribute__((target("avx2,fma"))) [[nodiscard]] std::complex<float>
dot_product_avx2(const std::complex<float> *const input,
                 const float *const coefficients,
                 const std::size_t tap_count) noexcept {
    const float *const values = reinterpret_cast<const float *>(input);
    const std::size_t scalar_count = tap_count * 2U;
    const std::size_t vector_count = scalar_count & ~std::size_t{7};
    __m256 sum = _mm256_setzero_ps();
    for (std::size_t index = 0; index < vector_count; index += 8U) {
        sum = _mm256_fmadd_ps(_mm256_loadu_ps(values + index),
                              _mm256_loadu_ps(coefficients + index), sum);
    }
    alignas(32) float lanes[8];
    _mm256_store_ps(lanes, sum);
    float real = lanes[0] + lanes[2] + lanes[4] + lanes[6];
    float imag = lanes[1] + lanes[3] + lanes[5] + lanes[7];
    for (std::size_t index = vector_count; index < scalar_count; index += 2U) {
        real += values[index] * coefficients[index];
        imag += values[index + 1U] * coefficients[index + 1U];
    }
    return {real, imag};
}
#endif

#if defined(__aarch64__) || defined(__ARM_NEON)
[[nodiscard]] std::complex<float>
dot_product_neon(const std::complex<float> *const input,
                 const float *const coefficients,
                 const std::size_t tap_count) noexcept {
    const float *const values = reinterpret_cast<const float *>(input);
    const std::size_t scalar_count = tap_count * 2U;
    const std::size_t vector_count = scalar_count & ~std::size_t{3};
    float32x4_t sum = vdupq_n_f32(0.0F);
    for (std::size_t index = 0; index < vector_count; index += 4U) {
        sum = vmlaq_f32(sum, vld1q_f32(values + index),
                        vld1q_f32(coefficients + index));
    }
    alignas(16) float lanes[4];
    vst1q_f32(lanes, sum);
    float real = lanes[0] + lanes[2];
    float imag = lanes[1] + lanes[3];
    for (std::size_t index = vector_count; index < scalar_count; index += 2U) {
        real += values[index] * coefficients[index];
        imag += values[index + 1U] * coefficients[index + 1U];
    }
    return {real, imag};
}
#endif

[[nodiscard]] DotProduct select_dot_product() noexcept {
#if defined(__x86_64__) || defined(__i386__)
#if defined(__GNUC__) || defined(__clang__)
    __builtin_cpu_init();
    if (__builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma")) {
        return dot_product_avx2;
    }
    if (__builtin_cpu_supports("sse4.1")) {
        return dot_product_sse41;
    }
#endif
#elif defined(__aarch64__) || defined(__ARM_NEON)
    return dot_product_neon;
#endif
    return dot_product_portable;
}

void set_thread_name(const std::string &prefix,
                     const std::size_t index) noexcept {
#if defined(__linux__)
    std::string name = prefix + std::to_string(index);
    name.resize(std::min<std::size_t>(name.size(), 15U));
    (void)pthread_setname_np(pthread_self(), name.c_str());
#else
    (void)prefix;
    (void)index;
#endif
}

class OutputBuffer {
  public:
    ~OutputBuffer() {
        if (data_ != nullptr) {
            ::operator delete(data_, alignment);
        }
    }

    OutputBuffer() = default;
    OutputBuffer(const OutputBuffer &) = delete;
    OutputBuffer &operator=(const OutputBuffer &) = delete;

    void resize_for_overwrite(const std::size_t size) {
        static_assert(std::is_trivially_copyable_v<std::complex<float>>);
        if (size > capacity_) {
            if (size > std::numeric_limits<std::size_t>::max() /
                           sizeof(std::complex<float>)) {
                throw std::bad_array_new_length{};
            }
            auto *const replacement = static_cast<std::complex<float> *>(
                ::operator new(size * sizeof(std::complex<float>), alignment));
            if (data_ != nullptr) {
                ::operator delete(data_, alignment);
            }
            data_ = replacement;
            capacity_ = size;
        }
        size_ = size;
    }

    [[nodiscard]] std::complex<float> *data() noexcept { return data_; }
    [[nodiscard]] std::span<const std::complex<float>> view() const noexcept {
        return {data_, size_};
    }

  private:
    static constexpr std::align_val_t alignment{64};
    std::complex<float> *data_{};
    std::size_t size_{};
    std::size_t capacity_{};
};

} // namespace

struct FrequencyTranslatingResampler::Impl {
    explicit Impl(const std::size_t requested_workers, std::string name)
        : workers_requested(std::max<std::size_t>(requested_workers, 1U)),
          worker_name(std::move(name)), dot_product(select_dot_product()) {
        if (workers_requested <= 1U) {
            return;
        }
        workers.reserve(workers_requested);
        try {
            for (std::size_t index = 0; index < workers_requested; ++index) {
                workers.emplace_back([this, index] { run_worker(index); });
            }
        } catch (...) {
            stop_and_join();
            throw;
        }
    }

    ~Impl() { stop_and_join(); }

    void configure(const ResamplerConfig &next) {
        if (!(next.input_rate_hz > 0.0) || !std::isfinite(next.input_rate_hz) ||
            !(next.output_rate_hz > 0.0) ||
            !std::isfinite(next.output_rate_hz) ||
            !(next.passband_edge_hz > 0.0) ||
            !std::isfinite(next.passband_edge_hz) ||
            !(next.stopband_edge_hz > next.passband_edge_hz) ||
            !std::isfinite(next.stopband_edge_hz) ||
            next.stopband_edge_hz > 0.5 * next.input_rate_hz ||
            !(next.stopband_attenuation_db >= 20.0F) ||
            next.stopband_attenuation_db > 160.0F ||
            !std::isfinite(next.stopband_attenuation_db) ||
            !is_power_of_two(next.polyphase_filters) ||
            next.polyphase_filters < 2U || next.polyphase_filters > 65536U) {
            throw std::invalid_argument(
                "invalid arbitrary resampler configuration");
        }
        if (configured_ && next.input_rate_hz == config.input_rate_hz &&
            next.output_rate_hz == config.output_rate_hz &&
            next.passband_edge_hz == config.passband_edge_hz &&
            next.stopband_edge_hz == config.stopband_edge_hz &&
            next.stopband_attenuation_db == config.stopband_attenuation_db &&
            next.polyphase_filters == config.polyphase_filters) {
            return;
        }

        const double ratio = next.output_rate_hz / next.input_rate_hz;
        const std::size_t next_tap_count = estimate_filter_taps(next);
        const float cutoff = static_cast<float>(
            0.5 * (next.passband_edge_hz + next.stopband_edge_hz) /
            next.input_rate_hz);
        coefficients = design_polyphase_bank(
            static_cast<unsigned int>(next_tap_count / 2U),
            next.polyphase_filters, cutoff, next.stopband_attenuation_db);
        config = next;
        tap_count = next_tap_count;
        phase_bits = log2_power_of_two(config.polyphase_filters);
        history.assign(tap_count - 1U, std::complex<float>{});
        boundary.resize(2U * (tap_count - 1U));
        configured_ = true;
        set_ratio(ratio);
        set_frequency_shift(0.0);
        reset();
    }

    void reset() noexcept {
        phase_q32 = 0;
        frequency_phase_q64 = 0;
        slew_step_credit = 0.0L;
        applied_step_q32.store(0U, std::memory_order_relaxed);
        applied_frequency_step_q64.store(0, std::memory_order_relaxed);
        applied_frequency_shift_hz.store(0.0, std::memory_order_relaxed);
        if (configured_) {
            std::ranges::fill(history, std::complex<float>{});
        }
        output.resize_for_overwrite(0);
    }

    void set_ratio(const double output_per_input) {
        if (!(output_per_input > 0.0) || !std::isfinite(output_per_input)) {
            throw std::invalid_argument(
                "resampling ratio must be finite and positive");
        }
        const long double step =
            static_cast<long double>(q32_one) / output_per_input;
        const long double rounded = std::round(step);
        if (rounded < 1.0L || rounded >= std::ldexp(1.0L, 64)) {
            throw std::out_of_range("resampling ratio exceeds Q32.32 range");
        }
        const auto quantized = static_cast<std::uint64_t>(rounded);
        requested_ratio_.store(output_per_input, std::memory_order_relaxed);
        target_step_q32.store(quantized, std::memory_order_release);
    }

    void set_max_slew_rate(const double ppm_per_second) {
        if (!(ppm_per_second >= 0.0) || !std::isfinite(ppm_per_second)) {
            throw std::invalid_argument(
                "resampler slew rate must be finite and non-negative");
        }
        max_slew_rate_ppm_per_second.store(ppm_per_second,
                                           std::memory_order_relaxed);
    }

    void set_frequency_shift(const double frequency_hz) {
        if (!configured_) {
            if (frequency_hz == 0.0) {
                requested_frequency_shift_hz.store(0.0,
                                                   std::memory_order_relaxed);
                target_frequency_step_q64.store(0, std::memory_order_release);
                return;
            }
            throw std::logic_error(
                "frequency shift requires a configured resampler");
        }
        if (!std::isfinite(frequency_hz) ||
            std::abs(frequency_hz) >= 0.5 * config.output_rate_hz) {
            throw std::invalid_argument(
                "frequency shift must be finite and inside output Nyquist");
        }
        const long double normalized =
            static_cast<long double>(frequency_hz) /
            static_cast<long double>(config.output_rate_hz);
        const long double rounded = std::round(normalized * q64_turn);
        if (rounded < static_cast<long double>(
                          std::numeric_limits<std::int64_t>::min()) ||
            rounded > static_cast<long double>(
                          std::numeric_limits<std::int64_t>::max())) {
            throw std::out_of_range("frequency shift exceeds Q0.64 range");
        }
        requested_frequency_shift_hz.store(frequency_hz,
                                           std::memory_order_relaxed);
        target_frequency_step_q64.store(static_cast<std::int64_t>(rounded),
                                        std::memory_order_release);
    }

    [[nodiscard]] std::uint64_t
    next_applied_step(const std::size_t input_samples) noexcept {
        const std::uint64_t target =
            target_step_q32.load(std::memory_order_acquire);
        const std::uint64_t current =
            applied_step_q32.load(std::memory_order_relaxed);
        const double slew =
            max_slew_rate_ppm_per_second.load(std::memory_order_relaxed);
        if (current == 0U || current == target || slew == 0.0) {
            slew_step_credit = 0.0L;
            return target;
        }

        const long double nominal_step =
            static_cast<long double>(q32_one) *
            static_cast<long double>(config.input_rate_hz) /
            static_cast<long double>(config.output_rate_hz);
        slew_step_credit += nominal_step * static_cast<long double>(slew) *
                            static_cast<long double>(input_samples) /
                            static_cast<long double>(config.input_rate_hz) /
                            1.0e6L;
        const long double whole_steps = std::floor(slew_step_credit);
        if (whole_steps < 1.0L) {
            return current;
        }
        const auto allowance = static_cast<std::uint64_t>(std::min(
            whole_steps, static_cast<long double>(
                             std::numeric_limits<std::uint64_t>::max())));
        slew_step_credit -= static_cast<long double>(allowance);

        if (target > current) {
            const std::uint64_t distance = target - current;
            return current + std::min(distance, allowance);
        }
        const std::uint64_t distance = current - target;
        return current - std::min(distance, allowance);
    }

    [[nodiscard]] std::span<const std::complex<float>>
    process(const std::span<const std::complex<float>> input) {
        if (!configured_ || input.empty()) {
            output.resize_for_overwrite(0);
            return {};
        }
        static_assert(sizeof(std::complex<float>) == 2U * sizeof(float));

        const std::uint64_t step = next_applied_step(input.size());
        applied_step_q32.store(step, std::memory_order_relaxed);
        const std::int64_t frequency_step =
            target_frequency_step_q64.load(std::memory_order_acquire);
        applied_frequency_step_q64.store(frequency_step,
                                         std::memory_order_relaxed);
        const double frequency_shift = static_cast<double>(
            static_cast<long double>(frequency_step) *
            static_cast<long double>(config.output_rate_hz) / q64_turn);
        applied_frequency_shift_hz.store(frequency_shift,
                                         std::memory_order_relaxed);
        const std::uint64_t phase_at_block_start = phase_q32;
        const std::uint64_t frequency_phase_at_block_start =
            frequency_phase_q64;
        if (input.size() > std::numeric_limits<std::uint32_t>::max()) {
            throw std::length_error(
                "arbitrary resampler input exceeds Q32.32 block range");
        }
        const std::uint64_t limit = static_cast<std::uint64_t>(input.size())
                                    << 32U;
        std::size_t output_count = 0;
        if (phase_q32 < limit) {
            const std::uint64_t distance = limit - phase_q32;
            const std::uint64_t quotient = distance / step;
            const std::uint64_t remainder = distance % step;
            const std::uint64_t count = quotient + (remainder != 0U ? 1U : 0U);
            if (count > std::numeric_limits<std::size_t>::max()) {
                throw std::length_error(
                    "arbitrary resampler output is too large");
            }
            output_count = static_cast<std::size_t>(count);
            phase_q32 = remainder == 0U ? 0U : step - remainder;
        } else {
            phase_q32 -= limit;
        }
        output.resize_for_overwrite(output_count);

        std::copy(history.begin(), history.end(), boundary.begin());
        const std::size_t prefix = std::min(history.size(), input.size());
        std::copy_n(input.begin(), prefix,
                    boundary.begin() +
                        static_cast<std::ptrdiff_t>(history.size()));
        if (prefix < history.size()) {
            std::fill(boundary.begin() +
                          static_cast<std::ptrdiff_t>(history.size() + prefix),
                      boundary.end(), std::complex<float>{});
        }

        task_input = input.data();
        task_output = output.data();
        task_output_count = output_count;
        task_phase_q32 = phase_at_block_start;
        task_step_q32 = step;
        task_frequency_phase_q64 = frequency_phase_at_block_start;
        task_frequency_step_q64 = frequency_step;

        const std::size_t active =
            output_count < minimum_parallel_outputs
                ? 1U
                : std::min(workers_requested, output_count);
        if (active == 1U || workers.empty()) {
            process_range(0, output_count);
        } else {
            {
                const std::scoped_lock lock(worker_mutex);
                active_workers = active;
                remaining_workers = active;
                ++generation;
            }
            work_ready.notify_all();
            std::unique_lock lock(worker_mutex);
            work_done.wait(lock, [this] { return remaining_workers == 0U; });
        }

        update_history(input);
        frequency_phase_q64 +=
            static_cast<std::uint64_t>(frequency_step) * output_count;
        return output.view();
    }

    void process_range(const std::size_t begin,
                       const std::size_t end) const noexcept {
        const std::size_t phase_stride = tap_count * 2U;
        const bool translate = task_frequency_step_q64 != 0;
        std::complex<float> oscillator{1.0F, 0.0F};
        std::complex<float> oscillator_step{1.0F, 0.0F};
        const auto phase_radians = [](const std::uint64_t word) {
            return static_cast<float>(2.0L * std::numbers::pi_v<long double> *
                                      static_cast<long double>(word) /
                                      q64_turn);
        };
        if (translate) {
            const std::uint64_t phase =
                task_frequency_phase_q64 +
                static_cast<std::uint64_t>(task_frequency_step_q64) * begin;
            oscillator = std::polar(1.0F, phase_radians(phase));
            oscillator_step = std::polar(
                1.0F, static_cast<float>(
                          2.0L * std::numbers::pi_v<long double> *
                          static_cast<long double>(task_frequency_step_q64) /
                          q64_turn));
        }
        for (std::size_t output_index = begin; output_index < end;
             ++output_index) {
            const std::uint64_t position =
                task_phase_q32 +
                static_cast<std::uint64_t>(output_index) * task_step_q32;
            const auto source = static_cast<std::size_t>(position >> 32U);
            const auto fraction = static_cast<std::uint32_t>(position);
            const std::size_t phase =
                static_cast<std::size_t>(fraction >> (32U - phase_bits));
            const std::complex<float> *window = nullptr;
            if (source < history.size()) {
                window = boundary.data() + source;
            } else {
                window = task_input + source - history.size();
            }
            const auto filtered = dot_product(
                window, coefficients.data() + phase * phase_stride, tap_count);
            task_output[output_index] =
                translate ? filtered * oscillator : filtered;
            if (translate) {
                oscillator *= oscillator_step;
                if (((output_index - begin) & 511U) == 511U) {
                    const std::uint64_t next_phase =
                        task_frequency_phase_q64 +
                        static_cast<std::uint64_t>(task_frequency_step_q64) *
                            (output_index + 1U);
                    oscillator = std::polar(1.0F, phase_radians(next_phase));
                }
            }
        }
    }

    void update_history(const std::span<const std::complex<float>> input) {
        if (input.size() >= history.size()) {
            std::copy(input.end() - static_cast<std::ptrdiff_t>(history.size()),
                      input.end(), history.begin());
            return;
        }
        const std::size_t retained = history.size() - input.size();
        std::move(history.end() - static_cast<std::ptrdiff_t>(retained),
                  history.end(), history.begin());
        std::copy(input.begin(), input.end(),
                  history.begin() + static_cast<std::ptrdiff_t>(retained));
    }

    void run_worker(const std::size_t index) noexcept {
        set_thread_name(worker_name, index);
        std::uint64_t observed_generation = 0;
        while (true) {
            std::size_t begin = 0;
            std::size_t end = 0;
            {
                std::unique_lock lock(worker_mutex);
                work_ready.wait(lock, [this, observed_generation] {
                    return stopping || generation != observed_generation;
                });
                if (stopping) {
                    return;
                }
                observed_generation = generation;
                if (index >= active_workers) {
                    continue;
                }
                begin = task_output_count * index / active_workers;
                end = task_output_count * (index + 1U) / active_workers;
            }
            process_range(begin, end);
            {
                const std::scoped_lock lock(worker_mutex);
                --remaining_workers;
                if (remaining_workers == 0U) {
                    work_done.notify_one();
                }
            }
        }
    }

    void stop_and_join() noexcept {
        {
            const std::scoped_lock lock(worker_mutex);
            stopping = true;
        }
        work_ready.notify_all();
        for (auto &worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    const std::size_t workers_requested;
    const std::string worker_name;
    const DotProduct dot_product;
    ResamplerConfig config{};
    bool configured_{};
    std::size_t tap_count{};
    unsigned int phase_bits{};
    std::vector<float> coefficients;
    std::vector<std::complex<float>> history;
    std::vector<std::complex<float>> boundary;
    OutputBuffer output;
    std::atomic<double> requested_ratio_{};
    std::atomic<double> max_slew_rate_ppm_per_second{};
    std::atomic<double> requested_frequency_shift_hz{};
    std::atomic<double> applied_frequency_shift_hz{};
    std::atomic<std::uint64_t> target_step_q32{};
    std::atomic<std::uint64_t> applied_step_q32{};
    std::atomic<std::int64_t> target_frequency_step_q64{};
    std::atomic<std::int64_t> applied_frequency_step_q64{};
    std::uint64_t phase_q32{};
    std::uint64_t frequency_phase_q64{};
    long double slew_step_credit{};

    std::vector<std::thread> workers;
    std::mutex worker_mutex;
    std::condition_variable work_ready;
    std::condition_variable work_done;
    std::uint64_t generation{};
    std::size_t active_workers{};
    std::size_t remaining_workers{};
    bool stopping{};

    const std::complex<float> *task_input{};
    std::complex<float> *task_output{};
    std::size_t task_output_count{};
    std::uint64_t task_phase_q32{};
    std::uint64_t task_step_q32{};
    std::uint64_t task_frequency_phase_q64{};
    std::int64_t task_frequency_step_q64{};
};

FrequencyTranslatingResampler::FrequencyTranslatingResampler(
    const std::size_t worker_count, std::string worker_name)
    : impl_(std::make_unique<Impl>(worker_count, std::move(worker_name))) {}

FrequencyTranslatingResampler::~FrequencyTranslatingResampler() noexcept =
    default;

void FrequencyTranslatingResampler::configure(const ResamplerConfig &config) {
    impl_->configure(config);
}

void FrequencyTranslatingResampler::reset() noexcept { impl_->reset(); }

void FrequencyTranslatingResampler::set_ratio(const double output_per_input) {
    impl_->set_ratio(output_per_input);
}

void FrequencyTranslatingResampler::set_max_slew_rate(
    const double ppm_per_second) {
    impl_->set_max_slew_rate(ppm_per_second);
}

void FrequencyTranslatingResampler::set_frequency_shift(
    const double frequency_hz) {
    impl_->set_frequency_shift(frequency_hz);
}

std::span<const std::complex<float>> FrequencyTranslatingResampler::process(
    const std::span<const std::complex<float>> input) {
    return impl_->process(input);
}

bool FrequencyTranslatingResampler::configured() const noexcept {
    return impl_->configured_;
}

std::size_t FrequencyTranslatingResampler::worker_count() const noexcept {
    return impl_->workers_requested;
}

double FrequencyTranslatingResampler::nominal_ratio() const noexcept {
    return impl_->configured_
               ? impl_->config.output_rate_hz / impl_->config.input_rate_hz
               : 0.0;
}

double FrequencyTranslatingResampler::requested_ratio() const noexcept {
    return impl_->requested_ratio_.load(std::memory_order_relaxed);
}

double FrequencyTranslatingResampler::effective_ratio() const noexcept {
    const std::uint64_t applied =
        impl_->applied_step_q32.load(std::memory_order_relaxed);
    const std::uint64_t step =
        applied != 0U ? applied
                      : impl_->target_step_q32.load(std::memory_order_relaxed);
    return step == 0U
               ? 0.0
               : static_cast<double>(q32_one) / static_cast<double>(step);
}

double FrequencyTranslatingResampler::max_slew_rate() const noexcept {
    return impl_->max_slew_rate_ppm_per_second.load(std::memory_order_relaxed);
}

double
FrequencyTranslatingResampler::requested_frequency_shift() const noexcept {
    return impl_->requested_frequency_shift_hz.load(std::memory_order_relaxed);
}

double
FrequencyTranslatingResampler::effective_frequency_shift() const noexcept {
    return impl_->applied_frequency_shift_hz.load(std::memory_order_relaxed);
}

std::uint64_t FrequencyTranslatingResampler::phase_step_q32() const noexcept {
    const std::uint64_t applied =
        impl_->applied_step_q32.load(std::memory_order_relaxed);
    return applied != 0U
               ? applied
               : impl_->target_step_q32.load(std::memory_order_relaxed);
}

std::size_t FrequencyTranslatingResampler::filter_taps() const noexcept {
    return impl_->tap_count;
}

} // namespace solid_resampler
