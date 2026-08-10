#include "sample_channel.hpp"

#include "absolute_sample_ring.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace airspy_tv::dvbt {
namespace {

[[nodiscard]] float
duration_ms(const std::chrono::steady_clock::time_point started_at) noexcept {
    return std::chrono::duration<float, std::milli>(
               std::chrono::steady_clock::now() - started_at)
        .count();
}

} // namespace

struct SampleChannel::Impl {
    explicit Impl(const std::size_t ring_capacity, Callbacks selected_callbacks)
        : ring(ring_capacity), callbacks(std::move(selected_callbacks)) {
        if (!callbacks.rebootstrap_requested || !callbacks.notify_idle) {
            throw std::invalid_argument("incomplete sample channel callbacks");
        }
    }

    [[nodiscard]] bool submit(InputBlock block,
                              const std::size_t capacity_samples,
                              const bool blocking) {
        const std::size_t incoming = block.samples.size() / 2U;
        std::unique_lock lock(mutex);
        input_capacity_samples = std::max(capacity_samples, incoming);
        if (blocking) {
            input_not_full.wait(lock, [this, incoming] {
                return stopping || queued_input_samples + incoming <=
                                       input_capacity_samples;
            });
        } else if (queued_input_samples + incoming > input_capacity_samples) {
            return false;
        }
        if (stopping) {
            return false;
        }
        block.generation = generation_value.load(std::memory_order_relaxed);
        queue.push_back(std::move(block));
        queued_input_samples += incoming;
        input_ready.notify_one();
        return true;
    }

    [[nodiscard]] FrontendWork wait_frontend() {
        std::unique_lock lock(mutex);
        input_ready.wait(lock, [this] {
            return stopping || reset_requested || flush_requested ||
                   callbacks.rebootstrap_requested() || !queue.empty();
        });
        if (stopping) {
            return {.event = FrontendEvent::stop,
                    .block = std::nullopt,
                    .reset_generation = 0};
        }
        if (reset_requested) {
            reset_acknowledged.wait(lock, [this] {
                return stopping ||
                       demod_reset_generation >= reset_request_generation;
            });
            if (stopping) {
                return {.event = FrontendEvent::stop,
                        .block = std::nullopt,
                        .reset_generation = 0};
            }
            return {.event = FrontendEvent::reset,
                    .block = std::nullopt,
                    .reset_generation = reset_request_generation};
        }
        if (callbacks.rebootstrap_requested()) {
            return {.event = FrontendEvent::rebootstrap,
                    .block = std::nullopt,
                    .reset_generation = 0};
        }
        if (queue.empty()) {
            flush_requested = false;
            ring.closed = true;
            return {.event = FrontendEvent::close_ring,
                    .block = std::nullopt,
                    .reset_generation = 0};
        }

        InputBlock block = std::move(queue.front());
        queue.pop_front();
        queued_input_samples -= block.samples.size() / 2U;
        frontend_busy = true;
        input_not_full.notify_one();
        return {.event = FrontendEvent::block,
                .block = std::move(block),
                .reset_generation = 0};
    }

    void finish_frontend_work() noexcept {
        {
            const std::scoped_lock lock(mutex);
            frontend_busy = false;
        }
        callbacks.notify_idle();
    }

    [[nodiscard]] PrepareResult prepare_block(const std::uint64_t generation,
                                              const std::size_t ring_capacity) {
        std::scoped_lock lock(mutex);
        if (generation != generation_value.load(std::memory_order_relaxed) ||
            reset_requested || stopping) {
            frontend_busy = false;
            return {};
        }
        const bool reopened = ring.closed;
        ring.closed = false;
        if (ring.empty() && ring.size() != ring_capacity) {
            ring.resize(ring_capacity);
        }
        if (reopened) {
            ring_data.notify_all();
        }
        return {.accepted = true,
                .reopened = reopened,
                .write_position = ring.write_position};
    }

    [[nodiscard]] PushResult
    push(const std::uint64_t generation,
         const std::span<const std::complex<float>> samples) {
        std::unique_lock lock(mutex);
        const auto wait_started_at = std::chrono::steady_clock::now();
        ring_space.wait(lock, [this] {
            return stopping || reset_requested || flush_requested ||
                   callbacks.rebootstrap_requested() ||
                   ring.used() < ring.size();
        });
        const float wait_time_ms = duration_ms(wait_started_at);
        const auto result = [this, wait_time_ms](const PushStatus status) {
            return PushResult{.status = status,
                              .written = 0,
                              .write_begin = ring.write_position,
                              .write_end = ring.write_position,
                              .wait_time_ms = wait_time_ms,
                              .copy_time_ms = 0.0F};
        };
        if (stopping) {
            return result(PushStatus::stop);
        }
        if (callbacks.rebootstrap_requested()) {
            return result(PushStatus::rebootstrap);
        }
        if (reset_requested ||
            generation != generation_value.load(std::memory_order_relaxed)) {
            return result(PushStatus::reset);
        }
        if (flush_requested && !demod_busy && !acquisition_pending &&
            ring.used() >= ring.size()) {
            return result(PushStatus::flush_abandoned);
        }

        const std::size_t chunk =
            std::min(ring.size() - ring.used(), samples.size());
        const std::uint64_t write_begin = ring.write_position;
        const auto copy_started_at = std::chrono::steady_clock::now();
        const std::size_t write_index = ring.write_position % ring.size();
        const std::size_t first = std::min(chunk, ring.size() - write_index);
        std::copy_n(samples.data(), first, ring.data() + write_index);
        if (first < chunk) {
            std::copy_n(samples.data() + first, chunk - first, ring.data());
        }
        const float copy_time_ms = duration_ms(copy_started_at);
        ring.write_position += chunk;
        ring_data.notify_all();
        return {.status = PushStatus::written,
                .written = chunk,
                .write_begin = write_begin,
                .write_end = ring.write_position,
                .wait_time_ms = wait_time_ms,
                .copy_time_ms = copy_time_ms};
    }

    void request_flush() {
        {
            const std::scoped_lock lock(mutex);
            flush_requested = true;
        }
        input_ready.notify_one();
        ring_space.notify_all();
        ring_data.notify_all();
    }

    [[nodiscard]] std::uint64_t request_reset() {
        std::uint64_t generation = 0;
        {
            const std::scoped_lock lock(mutex);
            cancelled_value.store(true, std::memory_order_release);
            generation =
                generation_value.fetch_add(1, std::memory_order_acq_rel) + 1;
            reset_request_generation = generation;
            reset_requested = true;
            queue.clear();
            queued_input_samples = 0;
        }
        input_not_full.notify_all();
        input_ready.notify_one();
        ring_space.notify_all();
        ring_data.notify_all();
        return generation;
    }

    void acknowledge_demod_reset(const std::uint64_t generation) {
        {
            const std::scoped_lock lock(mutex);
            demod_reset_generation =
                std::max(demod_reset_generation, generation);
        }
        reset_acknowledged.notify_all();
    }

    void begin_frontend_reset(const std::uint64_t generation) {
        const std::scoped_lock lock(mutex);
        if (reset_requested && generation == reset_request_generation) {
            ring.reset();
        }
    }

    void complete_frontend_reset(const std::uint64_t generation) {
        {
            const std::scoped_lock lock(mutex);
            if (generation != reset_request_generation) {
                return;
            }
            reset_requested = false;
            cancelled_value.store(false, std::memory_order_release);
            completed_reset_generation = generation;
            frontend_busy = false;
        }
        input_not_full.notify_all();
        ring_data.notify_all();
        reset_acknowledged.notify_all();
        callbacks.notify_idle();
    }

    void wait_reset_complete(const std::uint64_t generation) {
        std::unique_lock lock(mutex);
        reset_acknowledged.wait(lock, [this, generation] {
            return stopping || completed_reset_generation >= generation;
        });
    }

    [[nodiscard]] RebootstrapResult
    rebootstrap(InputBlock *active_block, const std::size_t input_offset) {
        const std::scoped_lock lock(mutex);
        const std::uint64_t old_generation =
            generation_value.load(std::memory_order_relaxed);
        const std::uint64_t new_generation =
            generation_value.fetch_add(1, std::memory_order_acq_rel) + 1;
        for (auto &queued : queue) {
            queued.generation = new_generation;
        }

        std::uint64_t consumed_input_samples = 0;
        if (active_block != nullptr &&
            active_block->generation == old_generation) {
            const std::size_t complex_count = active_block->samples.size() / 2U;
            consumed_input_samples = input_offset;
            if (input_offset < complex_count) {
                InputBlock suffix;
                suffix.samples.assign(
                    std::make_move_iterator(
                        active_block->samples.begin() +
                        static_cast<std::ptrdiff_t>(input_offset * 2U)),
                    std::make_move_iterator(active_block->samples.end()));
                suffix.rate = active_block->rate;
                suffix.bandwidth = active_block->bandwidth;
                suffix.generation = new_generation;
                suffix.stamp = active_block->stamp;
                suffix.stamp.begin_sample += input_offset;
                suffix.stamp.sample_count -= input_offset;
                suffix.stamp.discontinuity_before = true;
                queued_input_samples += suffix.stamp.sample_count;
                queue.push_front(std::move(suffix));
            }
        } else if (!queue.empty()) {
            queue.front().stamp.discontinuity_before = true;
        }
        const std::uint64_t write_position = ring.write_position;
        ring.reset();
        input_not_full.notify_all();
        ring_data.notify_all();
        ring_space.notify_all();
        return {.old_generation = old_generation,
                .new_generation = new_generation,
                .consumed_input_samples = consumed_input_samples,
                .ring_write_position = write_position};
    }

    void publish_sync_version(const std::uint64_t version) {
        {
            const std::scoped_lock lock(mutex);
            sync_version = version;
        }
        ring_data.notify_all();
    }

    void notify_frontend() noexcept { input_ready.notify_all(); }

    void notify_ring() noexcept {
        ring_data.notify_all();
        ring_space.notify_all();
    }

    void stop() noexcept {
        {
            const std::scoped_lock lock(mutex);
            stopping = true;
            cancelled_value.store(true, std::memory_order_release);
        }
        input_ready.notify_all();
        input_not_full.notify_all();
        ring_data.notify_all();
        ring_space.notify_all();
        reset_acknowledged.notify_all();
        callbacks.notify_idle();
    }

    void set_demod_busy(const bool busy) noexcept {
        {
            const std::scoped_lock lock(mutex);
            demod_busy = busy;
        }
        if (!busy) {
            ring_space.notify_all();
            callbacks.notify_idle();
        }
    }

    void set_acquisition_pending(const bool pending) noexcept {
        {
            const std::scoped_lock lock(mutex);
            acquisition_pending = pending;
        }
        if (!pending) {
            ring_space.notify_all();
        }
    }

    [[nodiscard]] Snapshot snapshot() const {
        const std::scoped_lock lock(mutex);
        return {.generation = generation_value.load(std::memory_order_relaxed),
                .sync_version = sync_version,
                .queued_blocks = queue.size(),
                .queued_input_samples = queued_input_samples,
                .input_capacity_samples = input_capacity_samples,
                .ring_used_samples = ring.used(),
                .ring_capacity_samples = ring.size(),
                .ring_read_position = ring.read_position,
                .ring_write_position = ring.write_position,
                .ring_closed = ring.closed,
                .stopping = stopping,
                .reset_requested = reset_requested,
                .flush_requested = flush_requested,
                .frontend_busy = frontend_busy,
                .demod_busy = demod_busy,
                .acquisition_pending = acquisition_pending};
    }

    [[nodiscard]] WaitStatus
    wait_for_stream(const std::uint64_t expected_sync_version,
                    const bool have_grid,
                    const std::size_t acquisition_samples) {
        std::unique_lock lock(mutex);
        ring_data.wait(lock, [this, expected_sync_version, have_grid,
                              acquisition_samples] {
            return stopping || sync_version != expected_sync_version ||
                   (have_grid && !ring.empty()) ||
                   (!have_grid && ring.closed) ||
                   (!have_grid && ring.used() >= acquisition_samples);
        });
        if (stopping) {
            return WaitStatus::stop;
        }
        if (sync_version != expected_sync_version) {
            return WaitStatus::sync_changed;
        }
        if (!have_grid && ring.closed) {
            return WaitStatus::closed;
        }
        return WaitStatus::ready;
    }

    [[nodiscard]] WaitStatus
    wait_for_rebootstrap(const std::uint64_t expected_sync_version) {
        std::unique_lock lock(mutex);
        ring_data.wait(lock, [this, expected_sync_version] {
            return stopping || sync_version != expected_sync_version ||
                   !callbacks.rebootstrap_requested();
        });
        if (stopping) {
            return WaitStatus::stop;
        }
        return sync_version != expected_sync_version ? WaitStatus::sync_changed
                                                     : WaitStatus::ready;
    }

    [[nodiscard]] WaitStatus
    wait_for_reopen(const std::uint64_t expected_sync_version) {
        std::unique_lock lock(mutex);
        ring_data.wait(lock, [this, expected_sync_version] {
            return stopping || sync_version != expected_sync_version ||
                   (!ring.closed && !ring.empty());
        });
        if (stopping) {
            return WaitStatus::stop;
        }
        return sync_version != expected_sync_version ? WaitStatus::sync_changed
                                                     : WaitStatus::ready;
    }

    [[nodiscard]] WaitStatus
    wait_for_retry(const std::uint64_t expected_sync_version,
                   const std::chrono::milliseconds timeout) {
        std::unique_lock lock(mutex);
        const bool ready =
            ring_data.wait_for(lock, timeout, [this, expected_sync_version] {
                return stopping || sync_version != expected_sync_version;
            });
        if (!ready) {
            return WaitStatus::timeout;
        }
        if (stopping) {
            return WaitStatus::stop;
        }
        return WaitStatus::sync_changed;
    }

    [[nodiscard]] WaitStatus
    wait_for_available(const std::size_t samples,
                       const std::uint64_t expected_generation,
                       const std::chrono::milliseconds timeout) {
        std::unique_lock lock(mutex);
        const bool ready = ring_data.wait_for(
            lock, timeout, [this, samples, expected_generation] {
                return stopping || reset_requested ||
                       generation_value.load(std::memory_order_relaxed) !=
                           expected_generation ||
                       ring.used() >= samples;
            });
        if (!ready) {
            return WaitStatus::timeout;
        }
        if (stopping) {
            return WaitStatus::stop;
        }
        if (reset_requested ||
            generation_value.load(std::memory_order_relaxed) !=
                expected_generation) {
            return WaitStatus::sync_changed;
        }
        return WaitStatus::ready;
    }

    [[nodiscard]] AcquisitionWindow
    acquisition_window(const std::size_t maximum_samples,
                       const std::size_t minimum_samples) const {
        const std::scoped_lock lock(mutex);
        const std::size_t window_size = std::min(ring.used(), maximum_samples);
        AcquisitionWindow result{
            .generation = generation_value.load(std::memory_order_relaxed),
            .base = ring.read_position,
            .samples = {},
        };
        if (window_size < minimum_samples) {
            return result;
        }
        result.samples.resize(window_size);
        ring.copy_absolute(ring.read_position, result.samples);
        return result;
    }

    [[nodiscard]] SymbolReadResult
    read_symbol(const std::uint64_t expected_sync_version,
                const std::uint64_t next_symbol_start,
                const std::size_t fft_size, const std::size_t guard_size,
                const bool measure_cyclic_prefix,
                const std::span<std::complex<float>> output) {
        if (output.size() != fft_size) {
            throw std::invalid_argument("symbol output size mismatch");
        }
        const std::uint64_t needed = next_symbol_start + fft_size;
        std::unique_lock lock(mutex);
        const auto wait_started_at = std::chrono::steady_clock::now();
        ring_data.wait(lock, [this, expected_sync_version, needed] {
            return stopping || sync_version != expected_sync_version ||
                   (ring.closed && ring.write_position < needed) ||
                   ring.write_position >= needed;
        });
        const float wait_time_ms = duration_ms(wait_started_at);
        if (stopping) {
            return {.status = WaitStatus::stop,
                    .cyclic_prefix = {},
                    .wait_time_ms = wait_time_ms,
                    .copy_time_ms = 0.0F};
        }
        if (sync_version != expected_sync_version) {
            return {.status = WaitStatus::sync_changed,
                    .cyclic_prefix = {},
                    .wait_time_ms = wait_time_ms,
                    .copy_time_ms = 0.0F};
        }
        if (ring.closed && ring.write_position < needed) {
            return {.status = WaitStatus::closed,
                    .cyclic_prefix = {},
                    .wait_time_ms = wait_time_ms,
                    .copy_time_ms = 0.0F};
        }

        const auto copy_started_at = std::chrono::steady_clock::now();
        ring.copy_absolute(next_symbol_start, output);
        CyclicPrefixMeasurement cyclic_prefix;
        if (measure_cyclic_prefix && next_symbol_start >= guard_size) {
            const std::uint64_t prefix_start = next_symbol_start - guard_size;
            if (prefix_start >= ring.read_position) {
                cyclic_prefix.valid = true;
                for (std::size_t index = 0; index < guard_size; ++index) {
                    const auto prefix =
                        ring[(prefix_start + index) % ring.size()];
                    const auto suffix =
                        ring[(prefix_start + fft_size + index) % ring.size()];
                    cyclic_prefix.correlation += std::conj(prefix) * suffix;
                    cyclic_prefix.prefix_power += std::norm(prefix);
                    cyclic_prefix.suffix_power += std::norm(suffix);
                }
            }
        }
        // Keep one guard interval behind the current FFT. Large but valid
        // startup SRO can require the demodulator to recenter the next FFT
        // window toward earlier samples; retaining this bounded history makes
        // that correction possible without weakening the ring invariants.
        ring.read_position =
            needed >= guard_size ? needed - guard_size : std::uint64_t{0};
        const float copy_time_ms = duration_ms(copy_started_at);
        lock.unlock();
        ring_space.notify_all();
        return {.status = WaitStatus::ready,
                .cyclic_prefix = cyclic_prefix,
                .wait_time_ms = wait_time_ms,
                .copy_time_ms = copy_time_ms};
    }

    [[nodiscard]] std::uint64_t
    align_at_or_after(std::uint64_t position,
                      const std::uint64_t period) const {
        const std::scoped_lock lock(mutex);
        while (position < ring.read_position) {
            position += period;
        }
        return position;
    }

    [[nodiscard]] std::uint64_t read_position() const {
        const std::scoped_lock lock(mutex);
        return ring.read_position;
    }

    [[nodiscard]] std::uint64_t write_position() const {
        const std::scoped_lock lock(mutex);
        return ring.write_position;
    }

    void discard_all() noexcept {
        {
            const std::scoped_lock lock(mutex);
            ring.read_position = ring.write_position;
        }
        ring_space.notify_all();
        callbacks.notify_idle();
    }

    mutable std::mutex mutex;
    std::condition_variable input_ready;
    std::condition_variable input_not_full;
    std::condition_variable ring_data;
    std::condition_variable ring_space;
    std::condition_variable reset_acknowledged;
    std::deque<InputBlock> queue;
    std::size_t queued_input_samples{};
    std::size_t input_capacity_samples{};
    AbsoluteSampleRing ring;
    Callbacks callbacks;
    std::atomic<std::uint64_t> generation_value{};
    std::atomic_bool cancelled_value{};
    std::uint64_t sync_version{};
    std::uint64_t reset_request_generation{};
    std::uint64_t demod_reset_generation{};
    std::uint64_t completed_reset_generation{};
    bool stopping{};
    bool reset_requested{};
    bool flush_requested{};
    bool frontend_busy{};
    bool demod_busy{};
    bool acquisition_pending{};
};

SampleChannel::SampleChannel(const std::size_t ring_capacity,
                             Callbacks callbacks)
    : impl_(std::make_unique<Impl>(ring_capacity, std::move(callbacks))) {}

SampleChannel::~SampleChannel() noexcept = default;

bool SampleChannel::submit(InputBlock block, const std::size_t capacity_samples,
                           const bool blocking) {
    return impl_->submit(std::move(block), capacity_samples, blocking);
}

SampleChannel::FrontendWork SampleChannel::wait_frontend() {
    return impl_->wait_frontend();
}

void SampleChannel::finish_frontend_work() noexcept {
    impl_->finish_frontend_work();
}

SampleChannel::PrepareResult
SampleChannel::prepare_block(const std::uint64_t generation,
                             const std::size_t ring_capacity) {
    return impl_->prepare_block(generation, ring_capacity);
}

SampleChannel::PushResult
SampleChannel::push(const std::uint64_t generation,
                    const std::span<const std::complex<float>> samples) {
    return impl_->push(generation, samples);
}

void SampleChannel::request_flush() { impl_->request_flush(); }

std::uint64_t SampleChannel::request_reset() { return impl_->request_reset(); }

void SampleChannel::acknowledge_demod_reset(const std::uint64_t generation) {
    impl_->acknowledge_demod_reset(generation);
}

void SampleChannel::begin_frontend_reset(const std::uint64_t generation) {
    impl_->begin_frontend_reset(generation);
}

void SampleChannel::complete_frontend_reset(const std::uint64_t generation) {
    impl_->complete_frontend_reset(generation);
}

void SampleChannel::wait_reset_complete(const std::uint64_t generation) {
    impl_->wait_reset_complete(generation);
}

SampleChannel::RebootstrapResult
SampleChannel::rebootstrap(InputBlock *active_block,
                           const std::size_t input_offset) {
    return impl_->rebootstrap(active_block, input_offset);
}

void SampleChannel::publish_sync_version(const std::uint64_t version) {
    impl_->publish_sync_version(version);
}

void SampleChannel::notify_frontend() noexcept { impl_->notify_frontend(); }

void SampleChannel::notify_ring() noexcept { impl_->notify_ring(); }

void SampleChannel::stop() noexcept { impl_->stop(); }

void SampleChannel::set_demod_busy(const bool busy) noexcept {
    impl_->set_demod_busy(busy);
}

void SampleChannel::set_acquisition_pending(const bool pending) noexcept {
    impl_->set_acquisition_pending(pending);
}

bool SampleChannel::cancelled() const noexcept {
    return impl_->cancelled_value.load(std::memory_order_acquire);
}

std::uint64_t SampleChannel::generation() const noexcept {
    return impl_->generation_value.load(std::memory_order_acquire);
}

SampleChannel::Snapshot SampleChannel::snapshot() const {
    return impl_->snapshot();
}

SampleChannel::WaitStatus
SampleChannel::wait_for_stream(const std::uint64_t sync_version,
                               const bool have_grid,
                               const std::size_t acquisition_samples) {
    return impl_->wait_for_stream(sync_version, have_grid, acquisition_samples);
}

SampleChannel::WaitStatus
SampleChannel::wait_for_rebootstrap(const std::uint64_t sync_version) {
    return impl_->wait_for_rebootstrap(sync_version);
}

SampleChannel::WaitStatus
SampleChannel::wait_for_reopen(const std::uint64_t sync_version) {
    return impl_->wait_for_reopen(sync_version);
}

SampleChannel::WaitStatus
SampleChannel::wait_for_retry(const std::uint64_t sync_version,
                              const std::chrono::milliseconds timeout) {
    return impl_->wait_for_retry(sync_version, timeout);
}

SampleChannel::WaitStatus
SampleChannel::wait_for_available(const std::size_t samples,
                                  const std::uint64_t generation,
                                  const std::chrono::milliseconds timeout) {
    return impl_->wait_for_available(samples, generation, timeout);
}

SampleChannel::AcquisitionWindow
SampleChannel::acquisition_window(const std::size_t maximum_samples,
                                  const std::size_t minimum_samples) const {
    return impl_->acquisition_window(maximum_samples, minimum_samples);
}

SampleChannel::SymbolReadResult SampleChannel::read_symbol(
    const std::uint64_t sync_version, const std::uint64_t next_symbol_start,
    const std::size_t fft_size, const std::size_t guard_size,
    const bool measure_cyclic_prefix,
    const std::span<std::complex<float>> output) {
    return impl_->read_symbol(sync_version, next_symbol_start, fft_size,
                              guard_size, measure_cyclic_prefix, output);
}

std::uint64_t
SampleChannel::align_at_or_after(const std::uint64_t position,
                                 const std::uint64_t period) const {
    return impl_->align_at_or_after(position, period);
}

std::uint64_t SampleChannel::read_position() const {
    return impl_->read_position();
}

std::uint64_t SampleChannel::write_position() const {
    return impl_->write_position();
}

void SampleChannel::discard_all() noexcept { impl_->discard_all(); }

} // namespace airspy_tv::dvbt
