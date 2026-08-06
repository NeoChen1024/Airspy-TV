#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dsp/vector_ops.hpp"
#include "airspy_tv/thread_name.hpp"

#include <liquid.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float minimum_power = 1.0e-12F;
constexpr unsigned int resampler_semi_length = 12;

} // namespace

struct Cs16Resampler::Impl {
    explicit Impl(const std::size_t requested_workers)
        : worker_count(std::max<std::size_t>(requested_workers, 1)),
          errors(worker_count) {
        workers.reserve(worker_count);
        try {
            for (std::size_t index = 0; index < worker_count; ++index) {
                workers.emplace_back([this, index] {
                    set_current_thread_name("an-resample-" +
                                            std::to_string(index));
                    run(index);
                });
            }
        } catch (...) {
            stop_and_join();
            throw;
        }
    }

    ~Impl() { stop_and_join(); }

    [[nodiscard]] std::vector<std::complex<float>>
    process(const std::span<const std::int16_t> interleaved_iq,
            const std::uint32_t sample_rate_hz,
            const std::uint32_t channel_bandwidth_hz) {
        if (interleaved_iq.empty() || interleaved_iq.size() % 2 != 0 ||
            sample_rate_hz == 0 || channel_bandwidth_hz == 0) {
            return {};
        }
        const std::uint64_t interpolation =
            static_cast<std::uint64_t>(channel_bandwidth_hz) * 8U;
        const std::uint64_t decimation =
            static_cast<std::uint64_t>(sample_rate_hz) * 7U;
        const std::uint64_t divisor = std::gcd(interpolation, decimation);
        const unsigned int next_p =
            static_cast<unsigned int>(interpolation / divisor);
        const unsigned int next_q =
            static_cast<unsigned int>(decimation / divisor);
        const std::size_t complex_count = interleaved_iq.size() / 2;
        const std::size_t next_blocks = complex_count / next_q;
        if (next_blocks == 0) {
            return {};
        }

        std::vector<std::complex<float>> next_input(next_blocks * next_q);
        dsp::convert_cs16_to_cf32(
            interleaved_iq.first(next_input.size() * 2), next_input);
        std::vector<std::complex<float>> next_output(next_blocks * next_p);

        {
            std::scoped_lock lock(mutex);
            input = next_input.data();
            output = next_output.data();
            p = next_p;
            q = next_q;
            blocks = next_blocks;
            active_workers = std::min(worker_count, blocks);
            remaining_workers = worker_count;
            std::ranges::fill(errors, nullptr);
            ++generation;
        }
        work_ready.notify_all();

        {
            std::unique_lock lock(mutex);
            work_done.wait(lock, [this] { return remaining_workers == 0; });
            input = nullptr;
            output = nullptr;
        }
        for (const auto &error : errors) {
            if (error) {
                std::rethrow_exception(error);
            }
        }
        return next_output;
    }

    void run(const std::size_t index) noexcept {
        std::uint64_t observed_generation = 0;
        rresamp_crcf filter = nullptr;
        unsigned int filter_p = 0;
        unsigned int filter_q = 0;
        while (true) {
            std::complex<float> *job_input = nullptr;
            std::complex<float> *job_output = nullptr;
            unsigned int job_p = 0;
            unsigned int job_q = 0;
            std::size_t begin = 0;
            std::size_t end = 0;
            {
                std::unique_lock lock(mutex);
                work_ready.wait(lock, [this, observed_generation] {
                    return stopping || generation != observed_generation;
                });
                if (stopping) {
                    break;
                }
                observed_generation = generation;
                job_input = input;
                job_output = output;
                job_p = p;
                job_q = q;
                if (index < active_workers) {
                    begin = blocks * index / active_workers;
                    end = blocks * (index + 1) / active_workers;
                }
            }

            if (index < active_workers) {
                try {
                    if (filter == nullptr || filter_p != job_p ||
                        filter_q != job_q) {
                        if (filter != nullptr) {
                            rresamp_crcf_destroy(filter);
                        }
                        filter = rresamp_crcf_create_kaiser(
                            job_p, job_q, resampler_semi_length, -1.0F, 60.0F);
                        if (filter == nullptr) {
                            throw std::runtime_error(
                                "failed to create CS16 resampler");
                        }
                        filter_p = job_p;
                        filter_q = job_q;
                    } else {
                        rresamp_crcf_reset(filter);
                    }
                    const std::size_t warmup =
                        std::min<std::size_t>(resampler_semi_length, begin);
                    for (std::size_t block = begin - warmup; block < begin;
                         ++block) {
                        rresamp_crcf_write(filter, job_input + (block * job_q));
                    }
                    rresamp_crcf_execute_block(
                        filter, job_input + (begin * job_q),
                        static_cast<unsigned int>(end - begin),
                        job_output + (begin * job_p));
                } catch (...) {
                    errors[index] = std::current_exception();
                }
            }
            {
                std::scoped_lock lock(mutex);
                if (--remaining_workers == 0) {
                    work_done.notify_one();
                }
            }
        }
        if (filter != nullptr) {
            rresamp_crcf_destroy(filter);
        }
    }

    void stop_and_join() noexcept {
        {
            const std::scoped_lock lock(mutex);
            stopping = true;
        }
        work_ready.notify_all();
        for (auto &worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    const std::size_t worker_count;
    std::vector<std::thread> workers;
    std::vector<std::exception_ptr> errors;
    std::mutex mutex;
    std::condition_variable work_ready;
    std::condition_variable work_done;
    std::complex<float> *input{};
    std::complex<float> *output{};
    unsigned int p{};
    unsigned int q{};
    std::size_t blocks{};
    std::size_t active_workers{};
    std::size_t remaining_workers{};
    std::uint64_t generation{};
    bool stopping{};
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
    return impl_->worker_count;
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
