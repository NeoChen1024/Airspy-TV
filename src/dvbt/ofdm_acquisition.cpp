#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dsp/vector_ops.hpp"
#include "liquid_resampler/arbitrary_resampler.hpp"

#include "resampler_config.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
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

    liquid_resampler::ArbitraryResampler resampler;
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
