#include "airspy_tv/dvbt/signal_analyzer.hpp"

#include "airspy_tv/dvbt/ofdm_acquisition.hpp"

#include <fftw3.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numbers>
#include <ranges>
#include <span>
#include <thread>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr std::size_t notch_lower_percentile_divisor = 100;

constexpr std::size_t raw_snapshot_samples = 350'000;
constexpr auto analysis_interval = std::chrono::milliseconds(100);
constexpr float analysis_rate_hz =
    1000.0F / static_cast<float>(analysis_interval.count());
constexpr float minimum_power = 1.0e-12F;

struct PilotLock {
    int phase{};
    int carrier_offset{};
};

[[nodiscard]] std::array<std::uint8_t, 6817> pilot_prbs() noexcept {
    std::array<std::uint8_t, 6817> result{};
    std::uint32_t state = (1U << 11U) - 1U;
    for (auto &value : result) {
        value = static_cast<std::uint8_t>(state & 1U);
        const std::uint32_t next = ((state >> 2U) ^ state) & 1U;
        state = (state >> 1U) | (next << 10U);
    }
    return result;
}

const auto prbs = pilot_prbs();

[[nodiscard]] std::vector<std::complex<float>>
transform_symbol(const std::span<const std::complex<float>> samples,
                 const OfdmAcquisition &acquisition) {
    std::vector<std::complex<float>> input(acquisition.fft_size);
    std::vector<std::complex<float>> output(acquisition.fft_size);
    const float phase_per_sample =
        std::arg(acquisition.phase) / static_cast<float>(acquisition.fft_size);
    // Match the complete streaming decoder: the shared CP acquisition returns
    // a guard start and the useful-symbol FFT begins after that guard.
    const std::size_t data_start = acquisition.start + acquisition.guard_size;
    for (std::size_t index = 0; index < acquisition.fft_size; ++index) {
        input[index] =
            samples[data_start + index] *
            std::polar(1.0F, -phase_per_sample *
                                 static_cast<float>(data_start + index));
    }
    static_assert(sizeof(std::complex<float>) == sizeof(fftwf_complex));
    fftwf_plan plan =
        fftwf_plan_dft_1d(static_cast<int>(acquisition.fft_size),
                          reinterpret_cast<fftwf_complex *>(input.data()),
                          reinterpret_cast<fftwf_complex *>(output.data()),
                          FFTW_FORWARD, FFTW_ESTIMATE);
    if (plan == nullptr) {
        return {};
    }
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);
    return output;
}

[[nodiscard]] std::complex<float>
active_carrier(const std::span<const std::complex<float>> fft,
               const std::size_t carrier, const std::size_t carrier_max,
               const int carrier_offset = 0) {
    const auto signed_bin = static_cast<std::ptrdiff_t>(carrier) -
                            static_cast<std::ptrdiff_t>(carrier_max / 2) +
                            static_cast<std::ptrdiff_t>(carrier_offset);
    const auto wrapped =
        (signed_bin + static_cast<std::ptrdiff_t>(fft.size())) %
        static_cast<std::ptrdiff_t>(fft.size());
    return fft[static_cast<std::size_t>(wrapped)];
}

[[nodiscard]] PilotLock
scattered_pilot_lock(const std::span<const std::complex<float>> fft,
                     const std::size_t carrier_max) {
    float best_score = -1.0F;
    PilotLock best;
    for (int carrier_offset = -48; carrier_offset <= 48; ++carrier_offset) {
        for (int phase = 0; phase < 4; ++phase) {
            const auto first_pilot = static_cast<std::size_t>(phase) * 3;
            float score = 0.0F;
            std::complex<float> correlation{};
            std::size_t chunk_count = 0;
            for (std::size_t pilot = first_pilot, count = 0;
                 pilot <= carrier_max && count < 192; pilot += 12, ++count) {
                const float transmitted =
                    prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                correlation += transmitted *
                               std::conj(active_carrier(fft, pilot, carrier_max,
                                                        carrier_offset));
                if (++chunk_count == 8) {
                    score += std::norm(correlation);
                    correlation = {};
                    chunk_count = 0;
                }
            }
            score += std::norm(correlation);
            if (score > best_score) {
                best_score = score;
                best = {phase, carrier_offset};
            }
        }
    }
    return best;
}

[[nodiscard]] std::vector<std::complex<float>>
estimate_channel(const std::span<const std::complex<float>> fft,
                 const std::size_t carrier_max, const PilotLock lock) {
    std::vector<std::complex<float>> channel(carrier_max + 1);
    std::vector<std::size_t> pilots;
    for (auto pilot = static_cast<std::size_t>(lock.phase) * 3;
         pilot <= carrier_max; pilot += 12) {
        const float transmitted =
            prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
        channel[pilot] =
            active_carrier(fft, pilot, carrier_max, lock.carrier_offset) /
            transmitted;
        pilots.push_back(pilot);
    }
    if (pilots.empty()) {
        return channel;
    }
    std::ranges::fill(channel.begin(),
                      channel.begin() +
                          static_cast<std::ptrdiff_t>(pilots.front()),
                      channel[pilots.front()]);
    for (std::size_t pilot_index = 1; pilot_index < pilots.size();
         ++pilot_index) {
        const std::size_t left = pilots[pilot_index - 1];
        const std::size_t right = pilots[pilot_index];
        for (std::size_t carrier = left; carrier <= right; ++carrier) {
            const float fraction = static_cast<float>(carrier - left) /
                                   static_cast<float>(right - left);
            channel[carrier] =
                channel[left] + ((channel[right] - channel[left]) * fraction);
        }
    }
    std::ranges::fill(channel.begin() +
                          static_cast<std::ptrdiff_t>(pilots.back()),
                      channel.end(), channel[pilots.back()]);
    return channel;
}

[[nodiscard]] Constellation
classify_constellation(const std::span<const std::complex<float>> points,
                       float &mer_db,
                       const std::optional<Constellation> fixed) {
    Constellation best = Constellation::qam64;
    float best_error = std::numeric_limits<float>::infinity();
    for (const auto candidate :
         {Constellation::qpsk, Constellation::qam16, Constellation::qam64}) {
        if (fixed.has_value() && candidate != *fixed) {
            continue;
        }
        const MaxLogDemapper demapper(candidate);
        float error = 0.0F;
        float power = 0.0F;
        for (const auto point : points) {
            float nearest = std::numeric_limits<float>::infinity();
            for (const auto ideal : demapper.constellation_points()) {
                nearest = std::min(nearest, std::norm(point - ideal));
            }
            error += nearest;
            power += std::norm(point);
        }
        const float normalized_error = error / std::max(power, minimum_power);
        if (normalized_error < best_error) {
            best_error = normalized_error;
            best = candidate;
        }
    }
    mer_db =
        std::clamp(-10.0F * std::log10(std::max(best_error, minimum_power)),
                   -10.0F, 60.0F);
    return best;
}

[[nodiscard]] SignalAnalysisSnapshot
analyze(const std::span<const std::int16_t> scalars,
        const std::uint32_t sample_rate, const std::uint32_t bandwidth,
        const ReceiverParameters &parameters) {
    SignalAnalysisSnapshot snapshot;
    const double target_rate = static_cast<double>(bandwidth) * 8.0 / 7.0;
    const auto resampled = resample_cs16(scalars, sample_rate, bandwidth);
    const OfdmAcquisition acquisition = acquire_ofdm(resampled, parameters);
    if (acquisition.score < 0.20F) {
        return snapshot;
    }
    const auto fft = transform_symbol(resampled, acquisition);
    if (fft.empty()) {
        return snapshot;
    }
    const std::size_t carrier_max =
        acquisition.mode == TransmissionMode::k8 ? 6816 : 1704;
    const PilotLock pilot_lock = scattered_pilot_lock(fft, carrier_max);
    const auto channel = estimate_channel(fft, carrier_max, pilot_lock);

    std::vector<float> channel_db;
    channel_db.reserve((carrier_max / 12) + 1);
    std::vector<std::complex<float>> equalized;
    equalized.reserve(carrier_max + 1);
    for (std::size_t carrier = 0; carrier <= carrier_max; ++carrier) {
        if (carrier % 12 == (static_cast<std::size_t>(pilot_lock.phase) * 3)) {
            // The outer active carriers include transmitter/receiver channel
            // filter skirts.  They are not multipath notches and would make
            // an otherwise flat channel look tens of dB worse.
            const std::size_t edge_guard = carrier_max / 32;
            if (carrier >= edge_guard && carrier + edge_guard <= carrier_max) {
                channel_db.push_back(
                    10.0F * std::log10(std::max(std::norm(channel[carrier]),
                                                minimum_power)));
            }
            continue;
        }
        if (std::norm(channel[carrier]) > minimum_power) {
            equalized.push_back(active_carrier(fft, carrier, carrier_max,
                                               pilot_lock.carrier_offset) /
                                channel[carrier]);
        }
    }
    if (equalized.empty() || channel_db.empty()) {
        return snapshot;
    }
    const std::size_t stride =
        std::max<std::size_t>(1, equalized.size() / snapshot.points.size());
    for (std::size_t index = 0; index < equalized.size() &&
                                snapshot.point_count < snapshot.points.size();
         index += stride) {
        snapshot.points[snapshot.point_count++] = equalized[index];
    }
    snapshot.constellation = classify_constellation(
        std::span<const std::complex<float>>{snapshot.points.data(),
                                             snapshot.point_count},
        snapshot.mer_db, parameters.constellation);
    auto baseline_values = channel_db;
    auto middle = baseline_values.begin() +
                  static_cast<std::ptrdiff_t>(baseline_values.size() / 2);
    std::ranges::nth_element(baseline_values, middle);
    const float median = *middle;
    const std::size_t lower_index =
        std::min(channel_db.size() - 1,
                 std::max<std::size_t>(1, channel_db.size() /
                                              notch_lower_percentile_divisor));
    auto lower = channel_db.begin() + static_cast<std::ptrdiff_t>(lower_index);
    std::ranges::nth_element(channel_db, lower);
    snapshot.deepest_notch_db = *lower - median;
    const float rho = std::sqrt(acquisition.score);
    snapshot.cp_snr_db =
        10.0F * std::log10(std::max(rho / std::max(1.0F - rho, 1.0e-4F),
                                    minimum_power));
    // CP correlation estimates fractional CFO modulo one subcarrier. The
    // scattered-pilot search also finds an integer bin displacement, but that
    // result is not a trustworthy physical-frequency estimate until continual
    // pilots/TPS confirm it. Keep the public metric fractional-only.
    snapshot.carrier_offset_hz = std::arg(acquisition.phase) *
                                 static_cast<float>(target_rate) /
                                 (2.0F * std::numbers::pi_v<float> *
                                  static_cast<float>(acquisition.fft_size));
    snapshot.mode = acquisition.mode;
    snapshot.guard_interval = acquisition.guard;
    snapshot.locked = true;
    return snapshot;
}

} // namespace

struct SignalAnalyzer::Impl {
    std::array<std::int16_t, raw_snapshot_samples * 2> pending_samples{};
    std::size_t pending_count{};
    std::uint32_t sample_rate{};
    std::uint32_t bandwidth{};
    ReceiverParameters parameters;
    std::chrono::steady_clock::time_point next_analysis{};
    mutable std::mutex pending_mutex;
    std::condition_variable ready;
    bool pending{};
    bool stopping{};
    std::atomic<bool> snr_smoothing{true};
    std::atomic<int> snr_smoothing_speed{20};
    std::atomic<bool> tracking_reset_requested{};
    mutable std::mutex snapshot_mutex;
    SignalAnalysisSnapshot latest;
    std::array<std::size_t, 8> configuration_votes{};
    std::size_t configuration_vote_count{};
    TransmissionMode stable_mode{TransmissionMode::k8};
    GuardInterval stable_guard{GuardInterval::gi_1_4};
    std::size_t missed_count{};
    bool configuration_locked{};
    std::thread worker;

    Impl() : worker([this] { run(); }) {}

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;

    ~Impl() noexcept {
        {
            const std::scoped_lock lock(pending_mutex);
            stopping = true;
        }
        ready.notify_one();
        if (worker.joinable()) {
            worker.join();
        }
    }

    void run() {
        std::array<std::int16_t, raw_snapshot_samples * 2> samples{};
        while (true) {
            std::uint32_t current_rate = 0;
            std::uint32_t current_bandwidth = 0;
            ReceiverParameters current_parameters;
            {
                std::unique_lock lock(pending_mutex);
                ready.wait(lock, [this] { return stopping || pending; });
                if (stopping) {
                    return;
                }
                samples = pending_samples;
                current_rate = sample_rate;
                current_bandwidth = bandwidth;
                current_parameters = parameters;
                pending = false;
            }
            ReceiverParameters acquisition_parameters = current_parameters;
            {
                const std::scoped_lock lock(snapshot_mutex);
                if (tracking_reset_requested.exchange(false)) {
                    configuration_votes.fill(0);
                    configuration_vote_count = 0;
                    missed_count = 0;
                    configuration_locked = false;
                } else if (configuration_locked) {
                    if (!acquisition_parameters.mode.has_value()) {
                        acquisition_parameters.mode = stable_mode;
                    }
                    if (!acquisition_parameters.guard_interval.has_value()) {
                        acquisition_parameters.guard_interval = stable_guard;
                    }
                }
            }
            auto next = analyze(samples, current_rate, current_bandwidth,
                                acquisition_parameters);
            const std::scoped_lock lock(snapshot_mutex);
            if (tracking_reset_requested.exchange(false)) {
                configuration_votes.fill(0);
                configuration_vote_count = 0;
                missed_count = 0;
                configuration_locked = false;
                latest = {};
                continue;
            }
            if (next.locked && current_parameters.mode.has_value() &&
                current_parameters.guard_interval.has_value()) {
                stable_mode = *current_parameters.mode;
                stable_guard = *current_parameters.guard_interval;
                configuration_locked = true;
                configuration_votes.fill(0);
                configuration_vote_count = 0;
                missed_count = 0;
            }
            if (!next.locked) {
                ++missed_count;
                if (configuration_locked && latest.locked && missed_count < 3) {
                    next = latest;
                }
            } else if (!configuration_locked) {
                missed_count = 0;
                const std::size_t mode_index =
                    next.mode == TransmissionMode::k8 ? 4 : 0;
                const std::size_t guard_index =
                    static_cast<std::size_t>(next.guard_interval);
                ++configuration_votes[mode_index + guard_index];
                ++configuration_vote_count;
                const auto winner =
                    std::ranges::max_element(configuration_votes);
                const std::size_t winner_votes = *winner;
                const std::size_t winner_index = static_cast<std::size_t>(
                    std::distance(configuration_votes.begin(), winner));
                if (configuration_vote_count >= 3 && winner_votes == 3) {
                    stable_mode = winner_index >= 4 ? TransmissionMode::k8
                                                    : TransmissionMode::k2;
                    stable_guard = static_cast<GuardInterval>(winner_index % 4);
                    configuration_locked = true;
                    configuration_votes.fill(0);
                    configuration_vote_count = 0;
                    if (next.mode != stable_mode ||
                        next.guard_interval != stable_guard) {
                        next.locked = false;
                        next.point_count = 0;
                    }
                } else {
                    next.locked = false;
                    next.point_count = 0;
                    if (configuration_vote_count >= 16) {
                        configuration_votes.fill(0);
                        configuration_vote_count = 0;
                    }
                }
            } else if (next.mode != stable_mode ||
                       next.guard_interval != stable_guard) {
                ++missed_count;
                if (latest.locked && missed_count < 3) {
                    next = latest;
                } else {
                    next.locked = false;
                    next.point_count = 0;
                }
            } else {
                missed_count = 0;
            }
            if (next.locked && latest.locked && snr_smoothing) {
                const float alpha =
                    std::min(static_cast<float>(
                                 std::max(snr_smoothing_speed.load(), 1)) /
                                 (analysis_rate_hz * 10.0F),
                             1.0F);
                next.cp_snr_db = (alpha * next.cp_snr_db) +
                                 ((1.0F - alpha) * latest.cp_snr_db);
                next.mer_db =
                    (alpha * next.mer_db) + ((1.0F - alpha) * latest.mer_db);
                next.deepest_notch_db =
                    (alpha * next.deepest_notch_db) +
                    ((1.0F - alpha) * latest.deepest_notch_db);
                next.carrier_offset_hz =
                    (alpha * next.carrier_offset_hz) +
                    ((1.0F - alpha) * latest.carrier_offset_hz);
            }
            next.sequence = latest.sequence + 1;
            latest = next;
        }
    }
};

SignalAnalyzer::SignalAnalyzer() : impl_(std::make_unique<Impl>()) {}
SignalAnalyzer::~SignalAnalyzer() noexcept = default;

void SignalAnalyzer::submit(const std::span<const std::int16_t> interleaved_iq,
                            const std::uint32_t sample_rate_hz,
                            const std::uint32_t channel_bandwidth_hz) {
    if (interleaved_iq.empty() || sample_rate_hz == 0) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < impl_->next_analysis) {
        return;
    }
    std::unique_lock lock(impl_->pending_mutex, std::try_to_lock);
    if (!lock.owns_lock() || impl_->pending) {
        return;
    }
    if (interleaved_iq.size() >= impl_->pending_samples.size()) {
        std::ranges::copy(interleaved_iq.last(impl_->pending_samples.size()),
                          impl_->pending_samples.begin());
        impl_->pending_count = impl_->pending_samples.size();
    } else {
        const std::size_t overflow =
            impl_->pending_count + interleaved_iq.size() >
                    impl_->pending_samples.size()
                ? (impl_->pending_count + interleaved_iq.size()) -
                      impl_->pending_samples.size()
                : 0;
        if (overflow != 0) {
            std::ranges::move(
                impl_->pending_samples.begin() +
                    static_cast<std::ptrdiff_t>(overflow),
                impl_->pending_samples.begin() +
                    static_cast<std::ptrdiff_t>(impl_->pending_count),
                impl_->pending_samples.begin());
            impl_->pending_count -= overflow;
        }
        std::ranges::copy(interleaved_iq, impl_->pending_samples.begin() +
                                              static_cast<std::ptrdiff_t>(
                                                  impl_->pending_count));
        impl_->pending_count += interleaved_iq.size();
        if (impl_->pending_count < impl_->pending_samples.size()) {
            return;
        }
    }
    impl_->sample_rate = sample_rate_hz;
    impl_->bandwidth = channel_bandwidth_hz;
    impl_->pending = true;
    impl_->next_analysis = now + analysis_interval;
    lock.unlock();
    impl_->ready.notify_one();
}

void SignalAnalyzer::reset() {
    {
        const std::scoped_lock lock(impl_->pending_mutex);
        impl_->pending = false;
        impl_->pending_count = 0;
        impl_->next_analysis = {};
    }
    impl_->tracking_reset_requested = true;
    const std::scoped_lock lock(impl_->snapshot_mutex);
    impl_->latest = {};
}

void SignalAnalyzer::set_parameters(const ReceiverParameters &parameters) {
    {
        const std::scoped_lock lock(impl_->pending_mutex);
        impl_->parameters = parameters;
    }
    reset();
}

void SignalAnalyzer::set_snr_smoothing(const bool enabled, const int speed) {
    impl_->snr_smoothing = enabled;
    impl_->snr_smoothing_speed = std::max(speed, 1);
}

SignalAnalysisSnapshot SignalAnalyzer::snapshot() const {
    const std::scoped_lock lock(impl_->snapshot_mutex);
    return impl_->latest;
}

} // namespace airspy_tv::dvbt
