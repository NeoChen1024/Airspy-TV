#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"

#include <fftw3.h>
#include <liquid.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <numbers>
#include <numeric>
#include <span>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr float input_scale = 32768.0F;
constexpr float minimum_power = 1.0e-12F;
constexpr std::size_t decode_chunk_samples = 1'400'000;
constexpr std::size_t acquisition_samples = 350'000;
constexpr std::size_t max_queued_blocks = 64;

constexpr std::array continual_2k{
    0,    48,   54,   87,   141,  156,  192,  201,  255,  279,  282,  333,
    432,  450,  483,  525,  531,  618,  636,  714,  759,  765,  780,  804,
    873,  888,  918,  939,  942,  969,  984,  1050, 1101, 1107, 1110, 1137,
    1140, 1146, 1206, 1269, 1323, 1377, 1491, 1683, 1704};
constexpr std::array tps_2k{34,  50,   209,  346,  413,  569,  595,  688, 790,
                            901, 1073, 1219, 1262, 1286, 1469, 1594, 1687};

[[nodiscard]] std::array<std::uint8_t, 6817> make_prbs() {
    std::array<std::uint8_t, 6817> result{};
    std::uint32_t state = 0x7ffU;
    for (auto &bit : result) {
        bit = static_cast<std::uint8_t>(state & 1U);
        state = (state >> 1U) | ((((state >> 2U) ^ state) & 1U) << 10U);
    }
    return result;
}
const auto prbs = make_prbs();

struct Acquisition {
    std::size_t start{};
    std::size_t fft_size{};
    std::size_t guard_size{};
    std::complex<float> phase{};
    float score{};
    TransmissionMode mode{TransmissionMode::k8};
};

[[nodiscard]] Acquisition
acquire(const std::span<const std::complex<float>> samples) {
    Acquisition best;
    float best_periodic_score = 0.0F;
    for (const auto mode : {TransmissionMode::k8, TransmissionMode::k2}) {
        const std::size_t fft_size = mode == TransmissionMode::k8 ? 8192 : 2048;
        for (const std::size_t divisor : {32U, 16U, 8U, 4U}) {
            const std::size_t guard = fft_size / divisor;
            if (samples.size() <= fft_size + guard) {
                continue;
            }
            std::complex<float> corr{};
            float p0 = 0.0F;
            float p1 = 0.0F;
            for (std::size_t i = 0; i < guard; ++i) {
                corr += std::conj(samples[i]) * samples[i + fft_size];
                p0 += std::norm(samples[i]);
                p1 += std::norm(samples[i + fft_size]);
            }
            const std::size_t last = samples.size() - fft_size - guard;
            std::vector<float> scores(last + 1);
            for (std::size_t start = 0; start <= last; ++start) {
                const float score =
                    std::norm(corr) / std::max(p0 * p1, minimum_power);
                scores[start] = score;
                if (start == last) {
                    break;
                }
                corr -= std::conj(samples[start]) * samples[start + fft_size];
                p0 -= std::norm(samples[start]);
                p1 -= std::norm(samples[start + fft_size]);
                const std::size_t next = start + guard;
                corr += std::conj(samples[next]) * samples[next + fft_size];
                p0 += std::norm(samples[next]);
                p1 += std::norm(samples[next + fft_size]);
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
                for (std::size_t i = 0; i < guard; ++i) {
                    selected_correlation +=
                        std::conj(samples[symbol_start + i]) *
                        samples[symbol_start + i + fft_size];
                }
            }
            best_periodic_score = candidate_score;
            best = {selected,         fft_size, guard, selected_correlation,
                    scores[selected], mode};
        }
    }
    return best;
}

[[nodiscard]] std::complex<float>
carrier(const std::span<const std::complex<float>> fft, const std::size_t index,
        const std::size_t maximum, const int offset) {
    const auto bin = static_cast<std::ptrdiff_t>(index) -
                     static_cast<std::ptrdiff_t>(maximum / 2) + offset;
    const auto wrapped = (bin + static_cast<std::ptrdiff_t>(fft.size())) %
                         static_cast<std::ptrdiff_t>(fft.size());
    return fft[static_cast<std::size_t>(wrapped)];
}

struct PilotLock {
    int phase{};
    int offset{};
};

[[nodiscard]] PilotLock
lock_pilots(const std::span<const std::complex<float>> fft,
            const std::size_t maximum, const int previous_offset) {
    PilotLock best;
    float best_score = -1.0F;
    const int radius =
        previous_offset == std::numeric_limits<int>::max() ? 48 : 2;
    const int center = previous_offset == std::numeric_limits<int>::max()
                           ? 0
                           : previous_offset;
    for (int offset = center - radius; offset <= center + radius; ++offset) {
        for (int phase = 0; phase < 4; ++phase) {
            std::complex<float> correlation{};
            float score = 0.0F;
            std::size_t chunk_count = 0;
            for (std::size_t pilot = static_cast<std::size_t>(phase * 3);
                 pilot <= maximum; pilot += 12) {
                const float value =
                    prbs[pilot] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                correlation +=
                    value * std::conj(carrier(fft, pilot, maximum, offset));
                if (++chunk_count == 8) {
                    score += std::norm(correlation);
                    correlation = {};
                    chunk_count = 0;
                }
            }
            score += std::norm(correlation);
            if (score > best_score) {
                best_score = score;
                best = {phase, offset};
            }
        }
    }
    return best;
}

[[nodiscard]] bool listed(const std::span<const int> list,
                          const std::size_t value) {
    return std::ranges::binary_search(list, static_cast<int>(value));
}

[[nodiscard]] std::vector<std::complex<float>>
resample(const std::span<const std::int16_t> iq,
         const std::uint32_t sample_rate, const std::uint32_t bandwidth) {
    const std::uint64_t interpolation =
        static_cast<std::uint64_t>(bandwidth) * 8U;
    const std::uint64_t decimation =
        static_cast<std::uint64_t>(sample_rate) * 7U;
    const std::uint64_t divisor = std::gcd(interpolation, decimation);
    const unsigned int p = static_cast<unsigned int>(interpolation / divisor);
    const unsigned int q = static_cast<unsigned int>(decimation / divisor);
    const std::size_t complex_count = iq.size() / 2;
    const std::size_t blocks = complex_count / q;
    std::vector<std::complex<float>> input(blocks * q);
    for (std::size_t i = 0; i < input.size(); ++i) {
        input[i] = {static_cast<float>(iq[i * 2]) / input_scale,
                    static_cast<float>(iq[i * 2 + 1]) / input_scale};
    }
    std::vector<std::complex<float>> output(blocks * p);
    rresamp_crcf filter = rresamp_crcf_create_kaiser(p, q, 12, -1.0F, 60.0F);
    if (filter == nullptr) {
        return {};
    }
    rresamp_crcf_execute_block(
        filter, input.data(), static_cast<unsigned int>(blocks), output.data());
    rresamp_crcf_destroy(filter);
    return output;
}

} // namespace

struct StreamDecoder::Impl {
    struct Block {
        std::vector<std::int16_t> samples;
        std::uint32_t rate{};
        std::uint32_t bandwidth{};
    };
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::deque<Block> queue;
    std::vector<std::int16_t> accumulated;
    TransportCallback callback;
    StreamDecoderStats latest;
    bool stopping{};
    bool reset_requested{};
    std::thread worker;

    Impl() : worker([this] { run(); }) {}
    ~Impl() {
        {
            const std::scoped_lock lock(mutex);
            stopping = true;
        }
        ready.notify_one();
        worker.join();
    }

    void decode_chunk(const std::span<const std::int16_t> iq,
                      const std::uint32_t rate, const std::uint32_t bandwidth) {
        auto samples = resample(iq, rate, bandwidth);
        if (samples.size() < acquisition_samples) {
            return;
        }
        const Acquisition acquisition =
            acquire(std::span(samples).first(acquisition_samples));
        if (acquisition.score < 0.20F) {
            return;
        }
        {
            const std::scoped_lock guard(mutex);
            latest.acquisition_score = acquisition.score;
            latest.fft_size = static_cast<std::uint32_t>(acquisition.fft_size);
            latest.guard_size =
                static_cast<std::uint32_t>(acquisition.guard_size);
        }
        const std::size_t maximum =
            acquisition.mode == TransmissionMode::k8 ? 6816 : 1704;
        const std::span<const int> continual = continual_2k;
        const std::span<const int> tps = tps_2k;
        Decoder decoder(
            {acquisition.mode, Constellation::qam64, CodeRate::rate_2_3});
        MaxLogDemapper reference{Constellation::qam64};
        std::vector<std::complex<float>> fft_in(acquisition.fft_size);
        std::vector<std::complex<float>> fft_out(acquisition.fft_size);
        fftwf_plan plan =
            fftwf_plan_dft_1d(static_cast<int>(acquisition.fft_size),
                              reinterpret_cast<fftwf_complex *>(fft_in.data()),
                              reinterpret_cast<fftwf_complex *>(fft_out.data()),
                              FFTW_FORWARD, FFTW_ESTIMATE);
        if (plan == nullptr) {
            return;
        }
        const float cfo_phase = std::arg(acquisition.phase) /
                                static_cast<float>(acquisition.fft_size);
        const std::size_t period =
            acquisition.fft_size + acquisition.guard_size;
        int carrier_offset = std::numeric_limits<int>::max();
        int previous_phase = -1;
        std::uint64_t phase_discontinuities = 0;
        double mer_sum = 0.0;
        std::uint64_t symbol_count = 0;
        std::uint64_t byte_count = 0;
        for (std::size_t start = acquisition.start;
             start + acquisition.fft_size <= samples.size(); start += period) {
            for (std::size_t i = 0; i < acquisition.fft_size; ++i) {
                fft_in[i] = samples[start + i] *
                            std::polar(1.0F, -cfo_phase *
                                                 static_cast<float>(start + i));
            }
            fftwf_execute(plan);
            const PilotLock lock =
                lock_pilots(fft_out, maximum, carrier_offset);
            if (previous_phase >= 0 && lock.phase != (previous_phase + 1) % 4) {
                ++phase_discontinuities;
            }
            previous_phase = lock.phase;
            carrier_offset = lock.offset;
            std::vector<std::complex<float>> channel(maximum + 1);
            std::vector<std::size_t> pilots;
            for (std::size_t k = static_cast<std::size_t>(lock.phase * 3);
                 k <= maximum; k += 12) {
                const float sent = prbs[k] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
                channel[k] =
                    carrier(fft_out, k, maximum, carrier_offset) / sent;
                pilots.push_back(k);
            }
            for (std::size_t i = 1; i < pilots.size(); ++i) {
                const std::size_t left = pilots[i - 1];
                const std::size_t right = pilots[i];
                for (std::size_t k = left; k <= right; ++k) {
                    const float f = static_cast<float>(k - left) /
                                    static_cast<float>(right - left);
                    channel[k] =
                        channel[left] + (channel[right] - channel[left]) * f;
                }
            }
            std::fill(channel.begin(),
                      channel.begin() +
                          static_cast<std::ptrdiff_t>(pilots.front()),
                      channel[pilots.front()]);
            std::fill(channel.begin() +
                          static_cast<std::ptrdiff_t>(pilots.back()),
                      channel.end(), channel[pilots.back()]);
            std::vector<std::complex<float>> payload;
            payload.reserve(payload_carrier_count(acquisition.mode));
            for (std::size_t k = 0; k <= maximum; ++k) {
                const bool scattered =
                    k % 12 == static_cast<std::size_t>(lock.phase * 3);
                const std::size_t base = k % 1704;
                if (scattered || listed(continual, base) || listed(tps, base)) {
                    continue;
                }
                payload.push_back(carrier(fft_out, k, maximum, carrier_offset) /
                                  channel[k]);
            }
            if (payload.size() != payload_carrier_count(acquisition.mode)) {
                continue;
            }
            for (int iteration = 0; iteration < 2; ++iteration) {
                std::complex<double> numerator{};
                double denominator = 0.0;
                for (const auto value : payload) {
                    auto nearest = reference.constellation_points().front();
                    float distance = std::numeric_limits<float>::infinity();
                    for (const auto point : reference.constellation_points()) {
                        if (const float d = std::norm(value - point);
                            d < distance) {
                            distance = d;
                            nearest = point;
                        }
                    }
                    numerator +=
                        std::conj(static_cast<std::complex<double>>(nearest)) *
                        static_cast<std::complex<double>>(value);
                    denominator += std::norm(nearest);
                }
                const auto gain =
                    static_cast<std::complex<float>>(numerator / denominator);
                if (std::abs(gain) > 1.0e-6F) {
                    for (auto &value : payload) {
                        value /= gain;
                    }
                }
            }
            std::vector<float> errors;
            errors.reserve(payload.size());
            for (const auto value : payload) {
                float error = std::numeric_limits<float>::infinity();
                for (const auto point : reference.constellation_points()) {
                    error = std::min(error, std::norm(value - point));
                }
                errors.push_back(error);
            }
            const double mean_error =
                std::accumulate(errors.begin(), errors.end(), 0.0) /
                static_cast<double>(errors.size());
            mer_sum += -10.0 * std::log10(std::max(mean_error, 1.0e-12));
            auto middle =
                errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2);
            std::ranges::nth_element(errors, middle);
            const float reliability =
                1.0F / std::max(*middle / std::log(2.0F), 1.0e-4F);
            std::vector<float> reliabilities(payload.size(), reliability);
            const auto ts = decoder.process_symbol(
                payload, reliabilities, static_cast<std::size_t>(lock.phase));
            if (!ts.empty()) {
                TransportCallback sink;
                {
                    const std::scoped_lock guard(mutex);
                    sink = callback;
                }
                if (sink) {
                    sink(ts);
                }
                byte_count += ts.size();
            }
            ++symbol_count;
        }
        fftwf_destroy_plan(plan);
        const std::scoped_lock guard(mutex);
        latest.ofdm_locked = symbol_count != 0;
        latest.carrier_bin_offset = carrier_offset;
        latest.mer_db = symbol_count == 0
                            ? 0.0F
                            : static_cast<float>(
                                  mer_sum / static_cast<double>(symbol_count));
        latest.pilot_phase_discontinuities += phase_discontinuities;
        latest.ofdm_symbols += symbol_count;
        latest.transport_bytes += byte_count;
        latest.transport = decoder.stats();
    }

    void run() {
        while (true) {
            Block block;
            {
                std::unique_lock lock(mutex);
                ready.wait(lock, [this] {
                    return stopping || reset_requested || !queue.empty();
                });
                if (stopping) {
                    return;
                }
                if (reset_requested) {
                    queue.clear();
                    accumulated.clear();
                    latest = {};
                    reset_requested = false;
                }
                if (queue.empty()) {
                    continue;
                }
                block = std::move(queue.front());
                queue.pop_front();
                ++latest.input_blocks;
            }
            accumulated.insert(accumulated.end(), block.samples.begin(),
                               block.samples.end());
            const std::size_t scalar_chunk = decode_chunk_samples * 2;
            while (accumulated.size() >= scalar_chunk) {
                decode_chunk(std::span(accumulated).first(scalar_chunk),
                             block.rate, block.bandwidth);
                accumulated.erase(
                    accumulated.begin(),
                    accumulated.begin() +
                        static_cast<std::ptrdiff_t>(scalar_chunk));
            }
        }
    }
};

StreamDecoder::StreamDecoder() : impl_(std::make_unique<Impl>()) {}
StreamDecoder::~StreamDecoder() noexcept = default;

void StreamDecoder::submit(const std::span<const std::int16_t> iq,
                           const std::uint32_t rate,
                           const std::uint32_t bandwidth) {
    if (iq.empty() || rate == 0 || (iq.size() % 2) != 0) {
        return;
    }
    const std::scoped_lock lock(impl_->mutex);
    if (impl_->queue.size() >= max_queued_blocks) {
        ++impl_->latest.dropped_blocks;
        return;
    }
    impl_->queue.push_back(
        {std::vector<std::int16_t>(iq.begin(), iq.end()), rate, bandwidth});
    impl_->ready.notify_one();
}

void StreamDecoder::reset() {
    const std::scoped_lock lock(impl_->mutex);
    impl_->reset_requested = true;
    impl_->ready.notify_one();
}

void StreamDecoder::set_transport_callback(TransportCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->callback = std::move(callback);
}

StreamDecoderStats StreamDecoder::stats() const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->latest;
}

} // namespace airspy_tv::dvbt
