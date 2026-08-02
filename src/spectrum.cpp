#include "airspy_tv/spectrum.hpp"

#include <fftw3.h>
#include <volk/volk.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <numbers>
#include <ranges>
#include <thread>
#include <utility>

namespace airspy_tv {
namespace {

constexpr std::size_t scalar_count = spectrum_fft_size * 2;
constexpr auto capture_interval = std::chrono::milliseconds(5);
constexpr float input_scale = 32768.0F;
constexpr float minimum_power = 1.0e-14F;
constexpr float spectrum_ema_alpha = 0.22F;
constexpr float signal_power_ema_alpha = 0.12F;

struct VolkDeleter {
    template <typename Value> void operator()(Value *pointer) const noexcept {
        volk_free(pointer);
    }
};

template <typename Value>
// NOLINTBEGIN(modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
using VolkBuffer = std::unique_ptr<Value[], VolkDeleter>;
// NOLINTEND(modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

template <typename Value>
VolkBuffer<Value> make_volk_buffer(const std::size_t count) {
    auto *pointer = static_cast<Value *>(
        volk_malloc(sizeof(Value) * count, volk_get_alignment()));
    if (pointer == nullptr) {
        throw std::bad_alloc();
    }
    return VolkBuffer<Value>(pointer);
}

} // namespace

struct SpectrumAnalyzer::Impl {
    std::array<std::int16_t, scalar_count> pending_samples{};
    std::array<std::int16_t, scalar_count> staging_samples{};
    std::size_t staging_count{};
    VolkBuffer<std::int16_t> worker_samples;
    mutable std::mutex pending_mutex;
    std::condition_variable pending_ready;
    bool pending{};
    bool stopping{};
    std::uint32_t pending_sample_rate{};
    std::chrono::steady_clock::time_point next_capture;

    mutable std::mutex snapshot_mutex;
    SpectrumSnapshot latest;
    std::atomic<bool> reset_requested;

    VolkBuffer<lv_32fc_t> input;
    VolkBuffer<lv_32fc_t> windowed;
    VolkBuffer<lv_32fc_t> fft_output;
    VolkBuffer<float> window;
    VolkBuffer<float> power;
    VolkBuffer<float> averaged_power;
    float window_sum{};
    float averaged_signal_power{};
    bool average_initialized{};
    fftwf_plan plan{};
    std::thread worker;

    Impl()
        : worker_samples(make_volk_buffer<std::int16_t>(scalar_count)),
          input(make_volk_buffer<lv_32fc_t>(spectrum_fft_size)),
          windowed(make_volk_buffer<lv_32fc_t>(spectrum_fft_size)),
          fft_output(make_volk_buffer<lv_32fc_t>(spectrum_fft_size)),
          window(make_volk_buffer<float>(spectrum_fft_size)),
          power(make_volk_buffer<float>(spectrum_fft_size)),
          averaged_power(make_volk_buffer<float>(spectrum_fft_size)) {
        for (std::size_t index = 0; index < spectrum_fft_size; ++index) {
            const float phase =
                (2.0F * std::numbers::pi_v<float> * static_cast<float>(index)) /
                static_cast<float>(spectrum_fft_size);
            window[index] = 0.35875F - (0.48829F * std::cos(phase)) +
                            (0.14128F * std::cos(2.0F * phase)) -
                            (0.01168F * std::cos(3.0F * phase));
            window_sum += window[index];
        }

        static_assert(sizeof(lv_32fc_t) == sizeof(fftwf_complex));
        plan = fftwf_plan_dft_1d(
            static_cast<int>(spectrum_fft_size),
            reinterpret_cast<fftwf_complex *>(windowed.get()),
            reinterpret_cast<fftwf_complex *>(fft_output.get()), FFTW_FORWARD,
            FFTW_ESTIMATE);
        if (plan == nullptr) {
            throw std::bad_alloc();
        }
        worker = std::thread([this] { run(); });
    }

    ~Impl() noexcept {
        {
            const std::scoped_lock lock(pending_mutex);
            stopping = true;
        }
        pending_ready.notify_one();
        if (worker.joinable()) {
            worker.join();
        }
        if (plan != nullptr) {
            fftwf_destroy_plan(plan);
        }
    }

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;

    void run() {
        while (true) {
            std::uint32_t sample_rate = 0;
            {
                std::unique_lock lock(pending_mutex);
                pending_ready.wait(lock,
                                   [this] { return stopping || pending; });
                if (stopping) {
                    break;
                }
                std::ranges::copy(pending_samples, worker_samples.get());
                sample_rate = pending_sample_rate;
                pending = false;
            }

            if (reset_requested.exchange(false)) {
                average_initialized = false;
            }
            process(sample_rate);
        }
    }

    void process(const std::uint32_t sample_rate) {
        volk_16i_s32f_convert_32f(reinterpret_cast<float *>(input.get()),
                                  worker_samples.get(), input_scale,
                                  static_cast<unsigned int>(scalar_count));

        volk_32fc_magnitude_squared_32f(
            power.get(), input.get(),
            static_cast<unsigned int>(spectrum_fft_size));
        float total_power = 0.0F;
        volk_32f_accumulator_s32f(&total_power, power.get(),
                                  static_cast<unsigned int>(spectrum_fft_size));
        const float signal_power =
            total_power / static_cast<float>(spectrum_fft_size);
        if (!average_initialized) {
            averaged_signal_power = signal_power;
        } else {
            averaged_signal_power =
                (signal_power_ema_alpha * signal_power) +
                ((1.0F - signal_power_ema_alpha) * averaged_signal_power);
        }

        volk_32fc_32f_multiply_32fc(
            windowed.get(), input.get(), window.get(),
            static_cast<unsigned int>(spectrum_fft_size));
        fftwf_execute(plan);
        volk_32fc_magnitude_squared_32f(
            power.get(), fft_output.get(),
            static_cast<unsigned int>(spectrum_fft_size));

        const float normalization = window_sum * window_sum;
        SpectrumSnapshot next;
        next.signal_power_dbfs = std::clamp(
            10.0F * std::log10(std::max(averaged_signal_power, minimum_power)),
            -140.0F, 0.0F);
        next.sample_rate_hz = sample_rate;
        next.valid = true;

        for (std::size_t output_index = 0; output_index < spectrum_fft_size;
             ++output_index) {
            const std::size_t fft_index =
                (output_index + (spectrum_fft_size / 2)) % spectrum_fft_size;
            const float normalized_power = power[fft_index] / normalization;
            if (!average_initialized) {
                averaged_power[output_index] = normalized_power;
            } else {
                averaged_power[output_index] =
                    (spectrum_ema_alpha * normalized_power) +
                    ((1.0F - spectrum_ema_alpha) *
                     averaged_power[output_index]);
            }
            next.bins_dbfs[output_index] = std::clamp(
                10.0F * std::log10(std::max(averaged_power[output_index],
                                            minimum_power)),
                -140.0F, 0.0F);
        }
        average_initialized = true;

        const std::scoped_lock lock(snapshot_mutex);
        next.sequence = latest.sequence + 1;
        latest = next;
    }
};

SpectrumAnalyzer::SpectrumAnalyzer() : impl_(std::make_unique<Impl>()) {}

SpectrumAnalyzer::~SpectrumAnalyzer() noexcept = default;

void SpectrumAnalyzer::submit(
    const std::span<const std::int16_t> interleaved_iq,
    const std::uint32_t sample_rate_hz) {
    if (interleaved_iq.empty()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < impl_->next_capture) {
        return;
    }
    std::unique_lock lock(impl_->pending_mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;
    }

    if (interleaved_iq.size() >= scalar_count) {
        std::ranges::copy(interleaved_iq.last(scalar_count),
                          impl_->pending_samples.begin());
        impl_->staging_count = 0;
    } else {
        const std::size_t required = scalar_count - impl_->staging_count;
        const std::size_t copied = std::min(required, interleaved_iq.size());
        std::ranges::copy(
            interleaved_iq.first(copied),
            impl_->staging_samples.begin() +
                static_cast<std::ptrdiff_t>(impl_->staging_count));
        impl_->staging_count += copied;
        if (impl_->staging_count < scalar_count) {
            return;
        }
        impl_->pending_samples = impl_->staging_samples;
        impl_->staging_count = 0;
    }
    impl_->pending_sample_rate = sample_rate_hz;
    impl_->pending = true;
    impl_->next_capture = now + capture_interval;
    lock.unlock();
    impl_->pending_ready.notify_one();
}

void SpectrumAnalyzer::reset() {
    impl_->reset_requested = true;
    {
        const std::scoped_lock lock(impl_->pending_mutex);
        impl_->pending = false;
        impl_->staging_count = 0;
        impl_->next_capture = {};
    }
    const std::scoped_lock lock(impl_->snapshot_mutex);
    impl_->latest.valid = false;
}

SpectrumSnapshot SpectrumAnalyzer::snapshot() const {
    const std::scoped_lock lock(impl_->snapshot_mutex);
    return impl_->latest;
}

} // namespace airspy_tv
