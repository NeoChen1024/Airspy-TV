#include "airspy_tv/dvbt/ofdm_acquisition.hpp"

#include <liquid.h>
#include <volk/volk.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <numeric>
#include <span>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float input_scale = 32768.0F;
constexpr float minimum_power = 1.0e-12F;
constexpr unsigned int resampler_semi_length = 12;

} // namespace

std::vector<std::complex<float>>
resample_cs16(const std::span<const std::int16_t> interleaved_iq,
              const std::uint32_t sample_rate_hz,
              const std::uint32_t channel_bandwidth_hz,
              const std::size_t requested_workers) {
    if (interleaved_iq.empty() || interleaved_iq.size() % 2 != 0 ||
        sample_rate_hz == 0 || channel_bandwidth_hz == 0) {
        return {};
    }
    const std::uint64_t interpolation =
        static_cast<std::uint64_t>(channel_bandwidth_hz) * 8U;
    const std::uint64_t decimation =
        static_cast<std::uint64_t>(sample_rate_hz) * 7U;
    const std::uint64_t divisor = std::gcd(interpolation, decimation);
    const unsigned int p = static_cast<unsigned int>(interpolation / divisor);
    const unsigned int q = static_cast<unsigned int>(decimation / divisor);
    const std::size_t complex_count = interleaved_iq.size() / 2;
    const std::size_t blocks = complex_count / q;
    if (blocks == 0) {
        return {};
    }

    std::vector<std::complex<float>> input(blocks * q);
    volk_16i_s32f_convert_32f(reinterpret_cast<float *>(input.data()),
                              interleaved_iq.data(), input_scale,
                              static_cast<unsigned int>(input.size() * 2));
    std::vector<std::complex<float>> output(blocks * p);
    const std::size_t worker_count =
        std::min(std::max<std::size_t>(requested_workers, 1), blocks);
    std::vector<std::exception_ptr> errors(worker_count);
    std::vector<std::jthread> workers;
    workers.reserve(worker_count);
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        const std::size_t begin = blocks * worker / worker_count;
        const std::size_t end = blocks * (worker + 1) / worker_count;
        workers.emplace_back([&, worker, begin, end] {
            try {
                rresamp_crcf filter = rresamp_crcf_create_kaiser(
                    p, q, resampler_semi_length, -1.0F, 60.0F);
                if (filter == nullptr) {
                    throw std::runtime_error("failed to create CS16 resampler");
                }
                const std::size_t warmup =
                    std::min<std::size_t>(resampler_semi_length, begin);
                for (std::size_t block = begin - warmup; block < begin;
                     ++block) {
                    rresamp_crcf_write(filter, input.data() + (block * q));
                }
                rresamp_crcf_execute_block(
                    filter, input.data() + (begin * q),
                    static_cast<unsigned int>(end - begin),
                    output.data() + (begin * p));
                rresamp_crcf_destroy(filter);
            } catch (...) {
                errors[worker] = std::current_exception();
            }
        });
    }
    workers.clear();
    for (const auto &error : errors) {
        if (error) {
            std::rethrow_exception(error);
        }
    }
    return output;
}

OfdmAcquisition acquire_ofdm(const std::span<const std::complex<float>> samples,
                             const ReceiverParameters &parameters) {
    OfdmAcquisition best;
    float best_periodic_score = 0.0F;
    for (const auto mode : {TransmissionMode::k8, TransmissionMode::k2}) {
        if (parameters.mode.has_value() && mode != *parameters.mode) {
            continue;
        }
        const std::size_t fft_size = mode == TransmissionMode::k8 ? 8192 : 2048;
        for (const auto [divisor, guard_interval] :
             {std::pair{32U, GuardInterval::gi_1_32},
              std::pair{16U, GuardInterval::gi_1_16},
              std::pair{8U, GuardInterval::gi_1_8},
              std::pair{4U, GuardInterval::gi_1_4}}) {
            if (parameters.guard_interval.has_value() &&
                guard_interval != *parameters.guard_interval) {
                continue;
            }
            const std::size_t guard = fft_size / divisor;
            if (samples.size() <= fft_size + guard) {
                continue;
            }
            std::complex<float> correlation{};
            float first_power = 0.0F;
            float second_power = 0.0F;
            for (std::size_t index = 0; index < guard; ++index) {
                correlation +=
                    std::conj(samples[index]) * samples[index + fft_size];
                first_power += std::norm(samples[index]);
                second_power += std::norm(samples[index + fft_size]);
            }
            const std::size_t last = samples.size() - fft_size - guard;
            std::vector<float> scores(last + 1);
            for (std::size_t start = 0; start <= last; ++start) {
                scores[start] =
                    std::norm(correlation) /
                    std::max(first_power * second_power, minimum_power);
                if (start == last) {
                    break;
                }
                correlation -=
                    std::conj(samples[start]) * samples[start + fft_size];
                first_power -= std::norm(samples[start]);
                second_power -= std::norm(samples[start + fft_size]);
                const std::size_t entering = start + guard;
                correlation +=
                    std::conj(samples[entering]) * samples[entering + fft_size];
                first_power += std::norm(samples[entering]);
                second_power += std::norm(samples[entering + fft_size]);
            }

            const std::size_t period = fft_size + guard;
            std::size_t best_phase = 0;
            float candidate_score = 0.0F;
            for (std::size_t phase = 0; phase < period; ++phase) {
                float sum = 0.0F;
                std::size_t count = 0;
                for (std::size_t start = phase; start < scores.size();
                     start += period) {
                    sum += scores[start];
                    ++count;
                }
                if (count >= 10 &&
                    sum / static_cast<float>(count) > candidate_score) {
                    candidate_score = sum / static_cast<float>(count);
                    best_phase = phase;
                }
            }
            if (candidate_score <= best_periodic_score) {
                continue;
            }

            std::size_t selected = best_phase;
            for (std::size_t start = best_phase; start < scores.size();
                 start += period) {
                if (scores[start] > scores[selected]) {
                    selected = start;
                }
            }
            std::complex<float> selected_correlation{};
            for (std::size_t symbol_start = best_phase;
                 symbol_start + fft_size + guard <= samples.size();
                 symbol_start += period) {
                for (std::size_t index = 0; index < guard; ++index) {
                    selected_correlation +=
                        std::conj(samples[symbol_start + index]) *
                        samples[symbol_start + index + fft_size];
                }
            }
            best_periodic_score = candidate_score;
            best = {
                selected,         fft_size, guard,         selected_correlation,
                scores[selected], mode,     guard_interval};
        }
    }
    return best;
}

} // namespace airspy_tv::dvbt
