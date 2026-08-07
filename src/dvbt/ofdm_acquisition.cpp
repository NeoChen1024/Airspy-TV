#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dsp/vector_ops.hpp"
#include "airspy_tv/fftw_plan.hpp"
#include "solid_resampler/frequency_translating_resampler.hpp"

#include "ofdm_carrier.hpp"
#include "resampler_config.hpp"

#include <fftw3.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float minimum_power = 1.0e-12F;

} // namespace

struct Cs16Resampler::Impl {
    explicit Impl(const std::size_t requested_workers)
        : resampler(std::max<std::size_t>(requested_workers, 1U),
                    "an-resamp-") {}

    [[nodiscard]] std::vector<std::complex<float>>
    process(const std::span<const std::int16_t> interleaved_iq,
            const std::uint32_t sample_rate_hz,
            const std::uint32_t channel_bandwidth_hz) {
        if (interleaved_iq.empty() || interleaved_iq.size() % 2 != 0 ||
            sample_rate_hz == 0 || channel_bandwidth_hz == 0) {
            return {};
        }
        const std::size_t complex_count = interleaved_iq.size() / 2;
        std::vector<std::complex<float>> converted(complex_count);
        dsp::convert_cs16_to_cf32(interleaved_iq, converted);
        resampler.configure(
            make_resampler_config(sample_rate_hz, channel_bandwidth_hz));
        resampler.reset();
        const auto output = resampler.process(converted);
        return {output.begin(), output.end()};
    }

    solid_resampler::FrequencyTranslatingResampler resampler;
};

Cs16Resampler::Cs16Resampler(const std::size_t worker_count)
    : impl_(std::make_unique<Impl>(worker_count)) {}

Cs16Resampler::~Cs16Resampler() noexcept = default;

std::vector<std::complex<float>>
Cs16Resampler::process(const std::span<const std::int16_t> interleaved_iq,
                       const std::uint32_t sample_rate_hz,
                       const std::uint32_t channel_bandwidth_hz) {
    return impl_->process(interleaved_iq, sample_rate_hz, channel_bandwidth_hz);
}

std::size_t Cs16Resampler::worker_count() const noexcept {
    return impl_->resampler.worker_count();
}

std::vector<std::complex<float>>
resample_cs16(const std::span<const std::int16_t> interleaved_iq,
              const std::uint32_t sample_rate_hz,
              const std::uint32_t channel_bandwidth_hz,
              const std::size_t requested_workers) {
    Cs16Resampler resampler(requested_workers);
    return resampler.process(interleaved_iq, sample_rate_hz,
                             channel_bandwidth_hz);
}

OfdmAcquisition acquire_ofdm(const std::span<const std::complex<float>> samples,
                             const ReceiverParameters &parameters,
                             const bool search_carrier_offset) {
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
                .start = selected,
                .fft_size = fft_size,
                .guard_size = guard,
                .phase = selected_correlation,
                .score = scores[selected],
                .mode = mode,
                .guard = guard_interval,
            };
        }
    }
    if (best.score <= 0.0F || best.fft_size == 0U ||
        best.start + best.guard_size + best.fft_size > samples.size()) {
        return best;
    }

    best.fractional_cfo_phase_per_sample =
        std::arg(best.phase) / static_cast<float>(best.fft_size);
    std::vector<std::complex<float>> fft_input(best.fft_size);
    std::vector<std::complex<float>> fft_output(best.fft_size);
    const std::size_t data_start = best.start + best.guard_size;
    std::complex<float> nco =
        std::polar(1.0F, -best.fractional_cfo_phase_per_sample *
                             static_cast<float>(data_start));
    const std::complex<float> nco_step =
        std::polar(1.0F, -best.fractional_cfo_phase_per_sample);
    for (std::size_t index = 0; index < best.fft_size; ++index) {
        fft_input[index] = samples[data_start + index] * nco;
        nco *= nco_step;
        if ((index & 511U) == 511U) {
            const float power = std::norm(nco);
            if (power > 0.0F) {
                nco *= 1.0F / std::sqrt(power);
            }
        }
    }
    static_assert(sizeof(std::complex<float>) == sizeof(fftwf_complex));
    auto plan =
        FftwfPlan::dft_1d(static_cast<int>(best.fft_size),
                          reinterpret_cast<fftwf_complex *>(fft_input.data()),
                          reinterpret_cast<fftwf_complex *>(fft_output.data()),
                          FFTW_FORWARD, FFTW_ESTIMATE);
    plan.execute();
    const std::size_t maximum = best.fft_size == 8192U ? 6816U : 1704U;
    const PilotLock lock = search_carrier_offset
                               ? lock_pilots(fft_output, maximum)
                               : lock_pilots_at_offset(fft_output, maximum, 0);
    best.carrier_offset = lock.offset;
    best.pilot_phase = lock.phase;
    best.total_cfo_phase_per_sample = best.fractional_cfo_phase_per_sample +
                                      (2.0F * std::numbers::pi_v<float> *
                                       static_cast<float>(best.carrier_offset) /
                                       static_cast<float>(best.fft_size));
    return best;
}

} // namespace airspy_tv::dvbt
