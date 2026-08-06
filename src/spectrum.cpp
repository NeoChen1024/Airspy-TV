#include "airspy_tv/spectrum.hpp"
#include "airspy_tv/fftw_plan.hpp"

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
#include <vector>

namespace airspy_tv {
namespace {

constexpr std::size_t scalar_count = spectrum_fft_size * 2;
constexpr auto capture_interval = std::chrono::milliseconds(5);
constexpr float input_scale = 32768.0F;
constexpr float minimum_power = 1.0e-14F;
constexpr float fft_rate_hz =
    1000.0F / static_cast<float>(capture_interval.count());
constexpr std::size_t notch_smoothing_radius = 4;
constexpr std::size_t notch_lower_percentile_divisor = 100;

[[nodiscard]] float lower_percentile(std::vector<float> &values) {
    const std::size_t index =
        std::min(values.size() - 1,
                 std::max<std::size_t>(1, values.size() /
                                              notch_lower_percentile_divisor));
    auto percentile = values.begin() + static_cast<std::ptrdiff_t>(index);
    std::ranges::nth_element(values, percentile);
    return *percentile;
}

void update_channel_metrics(SpectrumSnapshot &snapshot,
                            const std::span<const float> averaged_power,
                            const std::uint32_t channel_bandwidth_hz) {
    const auto sample_rate = static_cast<float>(snapshot.sample_rate_hz);
    const auto bandwidth = static_cast<float>(channel_bandwidth_hz);
    if (bandwidth <= 0.0F || sample_rate <= bandwidth * 1.15F) {
        return;
    }
    const float in_channel_edge = bandwidth * (2.9F / 6.0F);
    const float noise_start = bandwidth * (3.5F / 6.0F);
    const float notch_edge = bandwidth * (2.7F / 6.0F);

    float in_channel_power = 0.0F;
    std::size_t in_channel_bins = 0;
    float noise_power = 0.0F;
    std::size_t noise_bins = 0;
    std::vector<float> smoothed_channel;

    const auto frequency_at = [sample_rate](const std::size_t index) {
        return ((static_cast<float>(index) /
                 static_cast<float>(spectrum_fft_size)) -
                0.5F) *
               sample_rate;
    };
    for (std::size_t index = 0; index < spectrum_fft_size; ++index) {
        const float frequency = std::abs(frequency_at(index));
        if (frequency <= in_channel_edge) {
            in_channel_power += averaged_power[index];
            ++in_channel_bins;
        } else if (frequency >= noise_start &&
                   frequency <= sample_rate * 0.47F) {
            noise_power += averaged_power[index];
            ++noise_bins;
        }
    }
    if (in_channel_bins == 0 || noise_bins == 0) {
        return;
    }

    const float mean_channel =
        in_channel_power / static_cast<float>(in_channel_bins);
    const float mean_noise = noise_power / static_cast<float>(noise_bins);
    const float signal_excess =
        std::max(mean_channel - mean_noise, minimum_power);
    snapshot.rf_snr_db = std::clamp(
        10.0F * std::log10(signal_excess / std::max(mean_noise, minimum_power)),
        -20.0F, 60.0F);

    for (std::size_t index = notch_smoothing_radius;
         index + notch_smoothing_radius < spectrum_fft_size; ++index) {
        if (std::abs(frequency_at(index)) > notch_edge) {
            continue;
        }
        float sum = 0.0F;
        for (std::size_t offset = index - notch_smoothing_radius;
             offset <= index + notch_smoothing_radius; ++offset) {
            sum += averaged_power[offset];
        }
        smoothed_channel.push_back(
            10.0F *
            std::log10(std::max(
                sum / static_cast<float>((2 * notch_smoothing_radius) + 1),
                minimum_power)));
    }
    if (smoothed_channel.empty()) {
        return;
    }
    auto baseline_values = smoothed_channel;
    auto middle = baseline_values.begin() +
                  static_cast<std::ptrdiff_t>(baseline_values.size() / 2);
    std::ranges::nth_element(baseline_values, middle);
    const float median = *middle;
    const float lower = lower_percentile(smoothed_channel);
    snapshot.deepest_notch_db = std::clamp(lower - median, -80.0F, 0.0F);
    snapshot.channel_metrics_valid = true;
}

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
    std::uint32_t pending_channel_bandwidth{6'000'000};
    std::chrono::steady_clock::time_point next_capture;

    mutable std::mutex snapshot_mutex;
    SpectrumSnapshot latest;
    std::atomic<bool> reset_requested;
    std::atomic<bool> fft_smoothing{true};
    std::atomic<int> fft_smoothing_speed{100};
    std::atomic<bool> snr_smoothing{true};
    std::atomic<int> snr_smoothing_speed{20};

    VolkBuffer<lv_32fc_t> input;
    VolkBuffer<lv_32fc_t> windowed;
    VolkBuffer<lv_32fc_t> fft_output;
    VolkBuffer<float> window;
    VolkBuffer<float> power;
    VolkBuffer<float> shifted_power;
    VolkBuffer<float> smoothed_bins_dbfs;
    float window_sum{};
    float smoothed_signal_power_dbfs{};
    float smoothed_rf_snr_db{};
    float smoothed_notch_db{};
    bool average_initialized{};
    FftwfPlan plan;
    std::thread worker;

    Impl()
        : worker_samples(make_volk_buffer<std::int16_t>(scalar_count)),
          input(make_volk_buffer<lv_32fc_t>(spectrum_fft_size)),
          windowed(make_volk_buffer<lv_32fc_t>(spectrum_fft_size)),
          fft_output(make_volk_buffer<lv_32fc_t>(spectrum_fft_size)),
          window(make_volk_buffer<float>(spectrum_fft_size)),
          power(make_volk_buffer<float>(spectrum_fft_size)),
          shifted_power(make_volk_buffer<float>(spectrum_fft_size)),
          smoothed_bins_dbfs(make_volk_buffer<float>(spectrum_fft_size)) {
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
        plan = FftwfPlan::dft_1d(
            static_cast<int>(spectrum_fft_size),
            reinterpret_cast<fftwf_complex *>(windowed.get()),
            reinterpret_cast<fftwf_complex *>(fft_output.get()), FFTW_FORWARD,
            FFTW_ESTIMATE);
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
    }

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;

    void run() {
        while (true) {
            std::uint32_t sample_rate = 0;
            std::uint32_t channel_bandwidth = 0;
            {
                std::unique_lock lock(pending_mutex);
                pending_ready.wait(lock,
                                   [this] { return stopping || pending; });
                if (stopping) {
                    break;
                }
                std::ranges::copy(pending_samples, worker_samples.get());
                sample_rate = pending_sample_rate;
                channel_bandwidth = pending_channel_bandwidth;
                pending = false;
            }

            if (reset_requested.exchange(false)) {
                average_initialized = false;
            }
            process(sample_rate, channel_bandwidth);
        }
    }

    void process(const std::uint32_t sample_rate,
                 const std::uint32_t channel_bandwidth) {
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

        volk_32fc_32f_multiply_32fc(
            windowed.get(), input.get(), window.get(),
            static_cast<unsigned int>(spectrum_fft_size));
        plan.execute();
        volk_32fc_magnitude_squared_32f(
            power.get(), fft_output.get(),
            static_cast<unsigned int>(spectrum_fft_size));

        const float normalization = window_sum * window_sum;
        SpectrumSnapshot next;
        const float raw_signal_power_dbfs = std::clamp(
            10.0F * std::log10(std::max(signal_power, minimum_power)), -140.0F,
            0.0F);
        next.sample_rate_hz = sample_rate;
        next.valid = true;

        const float fft_alpha =
            fft_smoothing ? std::min(static_cast<float>(std::max(
                                         fft_smoothing_speed.load(), 1)) /
                                         (fft_rate_hz * 10.0F),
                                     1.0F)
                          : 1.0F;

        for (std::size_t output_index = 0; output_index < spectrum_fft_size;
             ++output_index) {
            const std::size_t fft_index =
                (output_index + (spectrum_fft_size / 2)) % spectrum_fft_size;
            const float normalized_power = power[fft_index] / normalization;
            shifted_power[output_index] = normalized_power;
            const float raw_dbfs = std::clamp(
                10.0F * std::log10(std::max(normalized_power, minimum_power)),
                -140.0F, 0.0F);
            next.waterfall_bins_dbfs[output_index] = raw_dbfs;
            if (!average_initialized) {
                smoothed_bins_dbfs[output_index] = raw_dbfs;
            } else {
                smoothed_bins_dbfs[output_index] =
                    (fft_alpha * raw_dbfs) +
                    ((1.0F - fft_alpha) * smoothed_bins_dbfs[output_index]);
            }
            next.bins_dbfs[output_index] = smoothed_bins_dbfs[output_index];
        }
        update_channel_metrics(
            next,
            std::span<const float>{shifted_power.get(), spectrum_fft_size},
            channel_bandwidth);
        const float snr_alpha =
            snr_smoothing ? std::min(static_cast<float>(std::max(
                                         snr_smoothing_speed.load(), 1)) /
                                         (fft_rate_hz * 10.0F),
                                     1.0F)
                          : 1.0F;
        if (!average_initialized) {
            smoothed_signal_power_dbfs = raw_signal_power_dbfs;
            smoothed_rf_snr_db = next.rf_snr_db;
            smoothed_notch_db = next.deepest_notch_db;
        } else {
            smoothed_signal_power_dbfs =
                (snr_alpha * raw_signal_power_dbfs) +
                ((1.0F - snr_alpha) * smoothed_signal_power_dbfs);
            if (next.channel_metrics_valid) {
                smoothed_rf_snr_db = (snr_alpha * next.rf_snr_db) +
                                     ((1.0F - snr_alpha) * smoothed_rf_snr_db);
                smoothed_notch_db = (snr_alpha * next.deepest_notch_db) +
                                    ((1.0F - snr_alpha) * smoothed_notch_db);
            }
        }
        next.signal_power_dbfs = smoothed_signal_power_dbfs;
        if (next.channel_metrics_valid) {
            next.rf_snr_db = smoothed_rf_snr_db;
            next.deepest_notch_db = smoothed_notch_db;
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
    const std::uint32_t sample_rate_hz,
    const std::uint32_t channel_bandwidth_hz) {
    if (interleaved_iq.empty()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::unique_lock lock(impl_->pending_mutex, std::try_to_lock);
    if (!lock.owns_lock() || now < impl_->next_capture) {
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
    impl_->pending_channel_bandwidth = channel_bandwidth_hz;
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

void SpectrumAnalyzer::set_smoothing(const bool fft_enabled,
                                     const int fft_speed,
                                     const bool snr_enabled,
                                     const int snr_speed) {
    impl_->fft_smoothing = fft_enabled;
    impl_->fft_smoothing_speed = std::max(fft_speed, 1);
    impl_->snr_smoothing = snr_enabled;
    impl_->snr_smoothing_speed = std::max(snr_speed, 1);
}

SpectrumSnapshot SpectrumAnalyzer::snapshot() const {
    const std::scoped_lock lock(impl_->snapshot_mutex);
    return impl_->latest;
}

} // namespace airspy_tv
