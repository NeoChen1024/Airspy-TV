#include "fec_stage.hpp"

#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/thread_name.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

void add_transport_counters(TransportDecoderStats &destination,
                            const TransportDecoderStats &source) noexcept {
    destination.viterbi_bits += source.viterbi_bits;
    destination.pre_viterbi_error_bits += source.pre_viterbi_error_bits;
    destination.pre_viterbi_compared_bits += source.pre_viterbi_compared_bits;
    destination.post_viterbi_error_bits += source.post_viterbi_error_bits;
    destination.post_viterbi_compared_bits += source.post_viterbi_compared_bits;
    destination.rs_packets += source.rs_packets;
    destination.rs_clean_packets += source.rs_clean_packets;
    destination.rs_corrected_packets += source.rs_corrected_packets;
    destination.rs_uncorrectable_packets += source.rs_uncorrectable_packets;
    destination.tei_packets += source.tei_packets;
    destination.ts_packets += source.ts_packets;
    destination.outer_bit_offset = source.outer_bit_offset;
    destination.outer_deinterleaver_phase = source.outer_deinterleaver_phase;
    destination.outer_sync_distance = source.outer_sync_distance;
    destination.outer_rs_evidence = source.outer_rs_evidence;
    destination.rs_synchronized = source.rs_synchronized;
    destination.energy_synchronized = source.energy_synchronized;
    destination.viterbi_workers = source.viterbi_workers;
}

[[nodiscard]] TransportDecoderStats
transport_counter_delta(const TransportDecoderStats &current,
                        const TransportDecoderStats &previous) noexcept {
    TransportDecoderStats result = current;
    result.viterbi_bits -= previous.viterbi_bits;
    result.pre_viterbi_error_bits -= previous.pre_viterbi_error_bits;
    result.pre_viterbi_compared_bits -= previous.pre_viterbi_compared_bits;
    result.post_viterbi_error_bits -= previous.post_viterbi_error_bits;
    result.post_viterbi_compared_bits -= previous.post_viterbi_compared_bits;
    result.rs_packets -= previous.rs_packets;
    result.rs_clean_packets -= previous.rs_clean_packets;
    result.rs_corrected_packets -= previous.rs_corrected_packets;
    result.rs_uncorrectable_packets -= previous.rs_uncorrectable_packets;
    result.tei_packets -= previous.tei_packets;
    result.ts_packets -= previous.ts_packets;
    return result;
}

[[nodiscard]] double
duration_ms(const std::chrono::steady_clock::time_point started_at) noexcept {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - started_at)
        .count();
}

[[nodiscard]] double counter_delta(const double current,
                                   const double previous) noexcept {
    return std::max(0.0, current - previous);
}

[[nodiscard]] double current_thread_cpu_ms() noexcept {
#if defined(CLOCK_THREAD_CPUTIME_ID)
    timespec value{};
    if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) == 0) {
        return (static_cast<double>(value.tv_sec) * 1'000.0) +
               (static_cast<double>(value.tv_nsec) / 1'000'000.0);
    }
#endif
    return 0.0;
}

[[nodiscard]] TransportDecoderTiming
timing_delta(const TransportDecoderTiming &current,
             const TransportDecoderTiming &previous) noexcept {
    return {
        .viterbi_wall_ms =
            counter_delta(current.viterbi_wall_ms, previous.viterbi_wall_ms),
        .viterbi_submit_ms = counter_delta(current.viterbi_submit_ms,
                                           previous.viterbi_submit_ms),
        .viterbi_queue_wait_ms = counter_delta(current.viterbi_queue_wait_ms,
                                               previous.viterbi_queue_wait_ms),
        .viterbi_collect_ms = counter_delta(current.viterbi_collect_ms,
                                            previous.viterbi_collect_ms),
        .viterbi_flush_wait_ms = counter_delta(current.viterbi_flush_wait_ms,
                                               previous.viterbi_flush_wait_ms),
        .viterbi_worker_work_ms = counter_delta(
            current.viterbi_worker_work_ms, previous.viterbi_worker_work_ms),
        .outer_wall_ms =
            counter_delta(current.outer_wall_ms, previous.outer_wall_ms),
        .outer_alignment_ms = counter_delta(current.outer_alignment_ms,
                                            previous.outer_alignment_ms),
        .outer_bit_repack_ms = counter_delta(current.outer_bit_repack_ms,
                                             previous.outer_bit_repack_ms),
        .outer_byte_deinterleave_ms =
            counter_delta(current.outer_byte_deinterleave_ms,
                          previous.outer_byte_deinterleave_ms),
        .outer_rs_decode_ms = counter_delta(current.outer_rs_decode_ms,
                                            previous.outer_rs_decode_ms),
        .outer_rs_codeword_copy_ms = counter_delta(
            current.outer_rs_codeword_copy_ms,
            previous.outer_rs_codeword_copy_ms),
        .outer_rs_syndrome_ms = counter_delta(
            current.outer_rs_syndrome_ms, previous.outer_rs_syndrome_ms),
        .outer_rs_error_locator_ms = counter_delta(
            current.outer_rs_error_locator_ms,
            previous.outer_rs_error_locator_ms),
        .outer_rs_correction_ms = counter_delta(
            current.outer_rs_correction_ms,
            previous.outer_rs_correction_ms),
        .outer_rs_payload_copy_ms = counter_delta(
            current.outer_rs_payload_copy_ms,
            previous.outer_rs_payload_copy_ms),
        .outer_energy_tei_ms = counter_delta(current.outer_energy_tei_ms,
                                             previous.outer_energy_tei_ms),
        .outer_buffer_ms =
            counter_delta(current.outer_buffer_ms, previous.outer_buffer_ms),
        .outer_output_ms =
            counter_delta(current.outer_output_ms, previous.outer_output_ms),
        .decoded_handoff_ms = counter_delta(current.decoded_handoff_ms,
                                            previous.decoded_handoff_ms),
        .transport_output_ms = counter_delta(current.transport_output_ms,
                                             previous.transport_output_ms),
    };
}

[[nodiscard]] TimingMap nested_timing(const TransportDecoderTiming &timing,
                                      const double transport_wall_ms) {
    const double viterbi_accounted =
        timing.viterbi_submit_ms + timing.viterbi_queue_wait_ms +
        timing.viterbi_collect_ms + timing.viterbi_flush_wait_ms;
    const double outer_accounted =
        timing.outer_alignment_ms + timing.outer_bit_repack_ms +
        timing.outer_byte_deinterleave_ms + timing.outer_rs_decode_ms +
        timing.outer_energy_tei_ms + timing.outer_buffer_ms +
        timing.outer_output_ms;
    const double transport_accounted =
        timing.viterbi_wall_ms + timing.outer_wall_ms +
        timing.decoded_handoff_ms + timing.transport_output_ms;
    return {
        {"fec::viterbi", timing.viterbi_wall_ms},
        {"fec::viterbi::submit", timing.viterbi_submit_ms},
        {"fec::viterbi::collect", timing.viterbi_collect_ms},
        {"fec::viterbi::other",
         std::max(0.0, timing.viterbi_wall_ms - viterbi_accounted)},
        {"fec::outer", timing.outer_wall_ms},
        {"fec::outer::alignment", timing.outer_alignment_ms},
        {"fec::outer::bit_repack", timing.outer_bit_repack_ms},
        {"fec::outer::deinterleave", timing.outer_byte_deinterleave_ms},
        {"fec::outer::rs_decode", timing.outer_rs_decode_ms},
        {"fec::outer::rs_decode::codeword_copy",
         timing.outer_rs_codeword_copy_ms},
        {"fec::outer::rs_decode::syndrome", timing.outer_rs_syndrome_ms},
        {"fec::outer::rs_decode::error_locator",
         timing.outer_rs_error_locator_ms},
        {"fec::outer::rs_decode::correction",
         timing.outer_rs_correction_ms},
        {"fec::outer::rs_decode::payload_copy",
         timing.outer_rs_payload_copy_ms},
        {"fec::outer::energy_tei", timing.outer_energy_tei_ms},
        {"fec::outer::buffer", timing.outer_buffer_ms},
        {"fec::outer::output", timing.outer_output_ms},
        {"fec::outer::other",
         std::max(0.0, timing.outer_wall_ms - outer_accounted)},
        {"fec::transport::decoded_handoff", timing.decoded_handoff_ms},
        {"fec::transport::output", timing.transport_output_ms},
        {"fec::transport::other",
         std::max(0.0, transport_wall_ms - transport_accounted)},
    };
}

} // namespace

struct FecStage::Impl {
    explicit Impl(Callbacks selected_callbacks, const std::size_t capacity)
        : callbacks(std::move(selected_callbacks)),
          queue_capacity(std::max<std::size_t>(1, capacity)) {
        if (!callbacks.generation_current || !callbacks.publish_session ||
            !callbacks.publish_window || !callbacks.emit_transport ||
            !callbacks.emit_discontinuity || !callbacks.telemetry_enabled ||
            !callbacks.emit_diagnostic || !callbacks.notify_idle ||
            !callbacks.worker_failed) {
            throw std::invalid_argument("incomplete FEC stage callbacks");
        }
        worker = std::thread([this] {
            set_current_thread_name("dvbt-fec");
            run_guarded();
        });
    }

    ~Impl() noexcept { stop(); }

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;

    [[nodiscard]] bool enqueue(FecItem item) {
        std::unique_lock lock(mutex);
        not_full.wait(lock, [this, generation = item.generation] {
            return stopping || !callbacks.generation_current(generation) ||
                   queue.size() < queue_capacity;
        });
        if (stopping || !callbacks.generation_current(item.generation)) {
            return false;
        }
        queue.push_back(std::move(item));
        ready.notify_one();
        return true;
    }

    void clear() {
        {
            const std::scoped_lock lock(mutex);
            queue.clear();
        }
        not_full.notify_all();
    }

    void set_capacity(const std::size_t capacity) {
        {
            const std::scoped_lock lock(mutex);
            queue_capacity = std::max<std::size_t>(1, capacity);
        }
        not_full.notify_all();
    }

    void notify_cancelled() noexcept { not_full.notify_all(); }

    void request_stop() noexcept {
        {
            const std::scoped_lock lock(mutex);
            stopping = true;
        }
        ready.notify_all();
        not_full.notify_all();
    }

    void stop() noexcept {
        request_stop();
        if (worker.joinable()) {
            worker.join();
        }
    }

    [[nodiscard]] Snapshot snapshot() const {
        const std::scoped_lock lock(mutex);
        return {.queued_items = queue.size(),
                .capacity = queue_capacity,
                .worker_state = static_cast<WorkerState>(
                    state.load(std::memory_order_relaxed)),
                .processing = processing};
    }

    void run_guarded() noexcept {
        try {
            run();
        } catch (...) {
            const std::exception_ptr error = std::current_exception();
            std::string message = "fec worker failed";
            try {
                std::rethrow_exception(error);
            } catch (const std::exception &exception) {
                message += ": ";
                message += exception.what();
            } catch (...) {
                message += ": unknown exception";
            }
            {
                const std::scoped_lock lock(mutex);
                stopping = true;
                processing = false;
                state.store(static_cast<int>(WorkerState::exited),
                            std::memory_order_relaxed);
            }
            ready.notify_all();
            not_full.notify_all();
            try {
                callbacks.worker_failed(error, std::move(message));
            } catch (...) {
                std::terminate();
            }
            try {
                callbacks.notify_idle();
            } catch (...) {
                std::terminate();
            }
        }
    }

    void publish_window(const FecItem &item) {
        if (!decoder) {
            return;
        }
        const DecoderTiming current_timing = decoder->timing();
        const double window_transport_time_ms =
            counter_delta(current_timing.transport_time_ms,
                          previous_decoder_timing.transport_time_ms);
        const TransportDecoderTiming detailed_timing = timing_delta(
            current_timing.transport, previous_decoder_timing.transport);
        previous_decoder_timing = current_timing;
        const TransportDecoderStats current = decoder->stats();
        const TransportDecoderStats delta =
            transport_counter_delta(current, previous_marker);
        previous_marker = current;
        TransportDecoderStats cumulative = completed_sessions;
        add_transport_counters(cumulative, current);
        double coordinator_cpu_ms = 0.0;
        if (detailed_timing_active) {
            const double current_cpu_ms = current_thread_cpu_ms();
            coordinator_cpu_ms =
                counter_delta(current_cpu_ms, previous_thread_cpu_ms);
            previous_thread_cpu_ms = current_cpu_ms;
        }
        callbacks.publish_window(
            {.generation = item.generation,
             .source_epoch = item.source_epoch,
             .demod_window_sequence = item.demod_window_sequence,
             .fec_session = fec_session,
             .output_bytes_delta = window_transport_bytes,
             .session = current,
             .delta = delta,
             .cumulative = cumulative,
             .fec_total_ms = fec_work_ms,
             .transport_nested_ms = window_transport_time_ms,
             .wait_ms = {{"fec::viterbi::queue_wait",
                          detailed_timing.viterbi_queue_wait_ms},
                         {"fec::viterbi::flush_wait",
                          detailed_timing.viterbi_flush_wait_ms}},
             .nested_ms =
                 nested_timing(detailed_timing, window_transport_time_ms),
             .thread_cpu_ms = {{"fec::coordinator", coordinator_cpu_ms}},
             .aggregate_worker_work_ms = {
                 {"fec::viterbi::worker",
                  detailed_timing.viterbi_worker_work_ms}}});
        fec_work_ms = 0.0;
        window_transport_bytes = 0;
    }

    void emit_output(const std::uint64_t generation,
                     const std::span<const std::uint8_t> output) {
        if (!output.empty() && callbacks.emit_transport(generation, output)) {
            window_transport_bytes += output.size();
        }
    }

    void begin_session(const FecItem &item) {
        const bool parameters_match =
            decoder != nullptr && decoder->parameters() == item.parameters;
        if (decoder != nullptr) {
            add_transport_counters(completed_sessions, decoder->stats());
        }
        if (!parameters_match) {
            if (decoder != nullptr && decoder_generation == item.generation) {
                callbacks.emit_discontinuity(
                    item.generation, TransportDiscontinuity::fec_region_reset);
            }
            decoder = std::make_unique<Decoder>(item.parameters);
            decoder->set_diagnostic_handler(
                {.enabled = [this] { return callbacks.telemetry_enabled(); },
                 .emit =
                     [this](DiagnosticEvent event) {
                         callbacks.emit_diagnostic(std::move(event),
                                                   diagnostic_context);
                     }});
        } else {
            if (decoder_generation == item.generation) {
                callbacks.emit_discontinuity(
                    item.generation, TransportDiscontinuity::fec_region_reset);
            }
            (*decoder).reset();
        }
        previous_decoder_timing = {};
        detailed_timing_active = callbacks.telemetry_enabled();
        decoder->set_detailed_timing_enabled(detailed_timing_active);
        previous_thread_cpu_ms =
            detailed_timing_active ? current_thread_cpu_ms() : 0.0;
        decoder_generation = item.generation;
        previous_marker = {};
        ++fec_session;
        callbacks.publish_session({.generation = item.generation,
                                   .fec_session = fec_session,
                                   .cumulative = completed_sessions});
    }

    void process(FecItem &item) {
        diagnostic_context = {
            .generation = item.generation,
            .source_epoch = item.source_epoch,
            .demod_window_sequence = item.demod_window_sequence,
            .fec_session = fec_session,
            .tps_symbol_index =
                item.kind == FecItem::Kind::symbol
                    ? std::optional<std::uint64_t>{item.symbol_index}
                    : std::nullopt,
        };
        if (!callbacks.generation_current(item.generation)) {
            return;
        }
        if (item.kind == FecItem::Kind::begin) {
            begin_session(item);
            diagnostic_context.fec_session = fec_session;
            return;
        }
        if (!decoder || decoder_generation != item.generation) {
            return;
        }
        const bool detailed_timing_enabled = callbacks.telemetry_enabled();
        if (detailed_timing_enabled != detailed_timing_active) {
            detailed_timing_active = detailed_timing_enabled;
            decoder->set_detailed_timing_enabled(detailed_timing_active);
            previous_thread_cpu_ms =
                detailed_timing_active ? current_thread_cpu_ms() : 0.0;
        }
        if (item.kind == FecItem::Kind::symbol) {
            const auto started_at = std::chrono::steady_clock::now();
            auto output = decoder->process_soft_metrics(item.mother_metrics);
            fec_work_ms += duration_ms(started_at);
            emit_output(item.generation, output);
            return;
        }
        if (item.kind == FecItem::Kind::end ||
            item.kind == FecItem::Kind::stream_end) {
            const auto started_at = std::chrono::steady_clock::now();
            auto output = decoder->flush();
            fec_work_ms += duration_ms(started_at);
            emit_output(item.generation, output);
            const bool stream_end =
                item.kind == FecItem::Kind::stream_end &&
                callbacks.generation_current(item.generation);
            publish_window(item);
            if (stream_end) {
                callbacks.emit_discontinuity(
                    item.generation, TransportDiscontinuity::stream_end);
            }
            return;
        }
        if (item.kind == FecItem::Kind::stats) {
            publish_window(item);
        }
    }

    void run() {
        while (true) {
            FecItem item;
            {
                std::unique_lock lock(mutex);
                state.store(static_cast<int>(WorkerState::waiting_fec_item),
                            std::memory_order_relaxed);
                ready.wait(lock, [this] { return stopping || !queue.empty(); });
                if (stopping) {
                    state.store(static_cast<int>(WorkerState::exited),
                                std::memory_order_relaxed);
                    return;
                }
                state.store(static_cast<int>(WorkerState::processing),
                            std::memory_order_relaxed);
                item = std::move(queue.front());
                queue.pop_front();
                processing = true;
            }
            not_full.notify_one();
            process(item);
            {
                const std::scoped_lock lock(mutex);
                processing = false;
            }
            callbacks.notify_idle();
        }
    }

    Callbacks callbacks;
    mutable std::mutex mutex;
    std::condition_variable ready;
    std::condition_variable not_full;
    std::deque<FecItem> queue;
    std::size_t queue_capacity{};
    std::atomic<int> state{static_cast<int>(WorkerState::idle)};
    bool stopping{};
    bool processing{};
    std::thread worker;

    FecStageDiagnosticContext diagnostic_context;
    std::uint64_t fec_session{};
    std::unique_ptr<Decoder> decoder;
    std::uint64_t decoder_generation{};
    double fec_work_ms{};
    std::uint64_t window_transport_bytes{};
    DecoderTiming previous_decoder_timing;
    double previous_thread_cpu_ms{};
    bool detailed_timing_active{};
    TransportDecoderStats completed_sessions;
    TransportDecoderStats previous_marker;
};

FecStage::FecStage(Callbacks callbacks, const std::size_t capacity)
    : impl_(std::make_unique<Impl>(std::move(callbacks), capacity)) {}

FecStage::~FecStage() noexcept = default;

bool FecStage::enqueue(FecItem item) { return impl_->enqueue(std::move(item)); }

void FecStage::clear() { impl_->clear(); }

void FecStage::set_capacity(const std::size_t capacity) {
    impl_->set_capacity(capacity);
}

void FecStage::notify_cancelled() noexcept { impl_->notify_cancelled(); }

void FecStage::request_stop() noexcept { impl_->request_stop(); }

void FecStage::stop() noexcept { impl_->stop(); }

FecStage::Snapshot FecStage::snapshot() const { return impl_->snapshot(); }

} // namespace airspy_tv::dvbt
