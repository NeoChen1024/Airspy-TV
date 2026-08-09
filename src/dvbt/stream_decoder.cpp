#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "airspy_tv/dsp/vector_ops.hpp"
#include "airspy_tv/dvbt/analysis_publisher.hpp"
#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/tps_decoder.hpp"
#include "airspy_tv/fftw_plan.hpp"
#include "airspy_tv/thread_name.hpp"
#include "clock_control_timeline.hpp"
#include "demod_stage.hpp"
#include "fec_stage.hpp"
#include "frontend_stage.hpp"
#include "ofdm_carrier.hpp"
#include "pipeline_config.hpp"
#include "sample_channel.hpp"

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
#include <deque>
#include <exception>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {

struct StreamDecoder::Impl { // NOLINT(clang-analyzer-optin.performance.Padding)
    mutable std::mutex mutex;
    std::condition_variable idle;
    InputSampleTimeline fallback_input_timeline;
    ClockControlTimeline clock_control;
    TransportCallback callback;
    DiscontinuityCallback discontinuity_callback;
    dvbt::SignalAnalyzer analyzer;
    AnalysisPublisher analysis_publisher;
    SampleChannel sample_channel;
    // True while the demod is at the stream head approaching a first-anchor
    // acquisition (including the wait for acquisition data): the front-end's
    // push-abandon keys off this in addition to !demod_busy so it never fires
    // in the window between the head wait waking and the acquisition marking
    // itself busy (a ring full of fresh data with a flush arriving mid-push
    // used to abandon the push there, dropping the first block's remainder
    // and starving small files). It is cleared when the demod parks in the
    // retry back-off, the end-of-stream wait, or the drain.
    std::exception_ptr terminal_exception;
    std::string terminal_error;
    std::unique_ptr<FecStage> fec_stage;
    std::unique_ptr<DemodStage> demod_stage;
    std::unique_ptr<FrontendStage> frontend_stage;

    [[nodiscard]] bool
    emit_fec_transport(const std::uint64_t generation,
                       const std::span<const std::uint8_t> output) {
        TransportCallback sink;
        {
            const std::scoped_lock lock(mutex);
            if (generation != sample_channel.generation()) {
                return false;
            }
            sink = callback;
        }
        if (sink) {
            sink(output);
        }
        return true;
    }

    void emit_fec_discontinuity(const std::uint64_t generation,
                                const TransportDiscontinuity discontinuity) {
        DiscontinuityCallback sink;
        {
            const std::scoped_lock lock(mutex);
            if (generation != sample_channel.generation()) {
                return;
            }
            sink = discontinuity_callback;
        }
        if (sink) {
            sink(discontinuity);
        }
    }

    void handle_worker_failure(std::exception_ptr error,
                               std::string message) noexcept {
        {
            const std::scoped_lock lock(mutex);
            if (!terminal_exception) {
                terminal_exception = std::move(error);
                terminal_error = std::move(message);
            }
        }
        sample_channel.stop();
        notify_all_waiters();
    }

    Impl()
        : sample_channel(
              ring_minimum_samples,
              {.rebootstrap_requested =
                   [this] { return clock_control.rebootstrap_requested(); },
               .notify_idle = [this] { idle.notify_all(); }}),
          fec_stage(std::make_unique<FecStage>(
              FecStage::Callbacks{
                  .generation_current =
                      [this](const std::uint64_t generation) {
                          return generation == sample_channel.generation();
                      },
                  .publish_session =
                      [this](const FecStageSession &session) {
                          if (demod_stage) {
                              demod_stage->publish_fec_session(session);
                          }
                      },
                  .publish_window =
                      [this](const FecStageWindow &window) {
                          if (demod_stage) {
                              demod_stage->publish_fec_window(window);
                          }
                      },
                  .emit_transport =
                      [this](const std::uint64_t generation,
                             const std::span<const std::uint8_t> output) {
                          return emit_fec_transport(generation, output);
                      },
                  .emit_discontinuity =
                      [this](const std::uint64_t generation,
                             const TransportDiscontinuity discontinuity) {
                          emit_fec_discontinuity(generation, discontinuity);
                      },
                  .diagnostics_enabled =
                      [this] {
                          return demod_stage && demod_stage->events_enabled();
                      },
                  .emit_diagnostic =
                      [this](DiagnosticEvent event,
                             const FecStageDiagnosticContext &context) {
                          if (demod_stage) {
                              demod_stage->emit_fec_diagnostic(std::move(event),
                                                               context);
                          }
                      },
                  .notify_idle = [this] { idle.notify_all(); },
                  .worker_failed =
                      [this](std::exception_ptr error, std::string message) {
                          handle_worker_failure(std::move(error),
                                                std::move(message));
                      },
              },
              initial_symbol_queue_capacity)),
          demod_stage(std::make_unique<DemodStage>(
              sample_channel, clock_control, *fec_stage, analysis_publisher,
              DemodStage::Callbacks{
                  .emit_discontinuity =
                      [this](const TransportDiscontinuity discontinuity) {
                          fire_discontinuity(discontinuity);
                      },
                  .notify_idle = [this] { idle.notify_all(); },
                  .worker_failure =
                      [this](std::exception_ptr error, std::string message) {
                          handle_worker_failure(std::move(error),
                                                std::move(message));
                      },
              })),
          frontend_stage(std::make_unique<FrontendStage>(
              sample_channel, clock_control,
              FrontendStage::Callbacks{
                  .parameters = [this] { return demod_stage->parameters(); },
                  .input_block_started =
                      [this](const std::uint64_t generation) {
                          demod_stage->input_block_started(generation);
                      },
                  .invalidate_sync =
                      [this](const std::uint64_t generation) {
                          return demod_stage->invalidate_sync(generation);
                      },
                  .publish_rebootstrap =
                      [this](const SampleChannel::RebootstrapResult &result) {
                          return demod_stage->publish_rebootstrap(result);
                      },
                  .publish_bootstrap_progress =
                      [this](const FrontendBootstrapProgress &progress) {
                          demod_stage->publish_bootstrap_progress(progress);
                      },
                  .publish_acquisition =
                      [this](
                          const FrontendAcquisitionPublication &publication) {
                          return demod_stage->publish_acquisition(publication);
                      },
                  .publish_command =
                      [this](const FrontendCommandPublication &publication) {
                          demod_stage->publish_command(publication);
                      },
                  .publish_block =
                      [this](const FrontendBlockPublication &publication) {
                          demod_stage->publish_frontend_block(publication);
                      },
                  .clear_fec = [this] { fec_stage->clear(); },
                  .notify_fec_cancelled =
                      [this] { fec_stage->notify_cancelled(); },
                  .worker_failure =
                      [this](std::exception_ptr error, std::string message) {
                          handle_worker_failure(std::move(error),
                                                std::move(message));
                      },
              })) {}
    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;
    ~Impl() {
        sample_channel.stop();
        fec_stage->request_stop();
        frontend_stage->stop();
        demod_stage->stop();
        fec_stage->stop();
    }

    void notify_all_waiters() noexcept {
        sample_channel.stop();
        fec_stage->request_stop();
        idle.notify_all();
    }

    void fire_discontinuity(const TransportDiscontinuity discontinuity) {
        DiscontinuityCallback sink;
        {
            const std::scoped_lock lock(mutex);
            sink = discontinuity_callback;
        }
        if (sink) {
            sink(discontinuity);
        }
    }
};

StreamDecoder::StreamDecoder() : impl_(std::make_unique<Impl>()) {}
StreamDecoder::~StreamDecoder() noexcept = default;

void StreamDecoder::submit(const std::span<const std::int16_t> interleaved_iq,
                           const std::uint32_t sample_rate_hz,
                           const std::uint32_t channel_bandwidth_hz,
                           InputSampleStamp stamp) {
    if (interleaved_iq.empty() || sample_rate_hz == 0 ||
        (interleaved_iq.size() % 2) != 0) {
        return;
    }
    const std::size_t incoming_samples = interleaved_iq.size() / 2;
    if (!stamp.valid_for(incoming_samples, sample_rate_hz)) {
        stamp = impl_->fallback_input_timeline.stamp(incoming_samples,
                                                     sample_rate_hz);
    }
    if (!impl_->analysis_publisher.locked()) {
        impl_->analyzer.submit(interleaved_iq, sample_rate_hz,
                               channel_bandwidth_hz);
    }
    InputBlock block{.samples = std::vector<std::int16_t>(
                         interleaved_iq.begin(), interleaved_iq.end()),
                     .rate = sample_rate_hz,
                     .bandwidth = channel_bandwidth_hz,
                     .generation = 0,
                     .stamp = stamp};
    if (!impl_->sample_channel.submit(
            std::move(block), buffered_input_samples(sample_rate_hz), false)) {
        impl_->demod_stage->input_block_dropped();
    }
}

void StreamDecoder::submit_blocking(
    const std::span<const std::int16_t> interleaved_iq,
    const std::uint32_t sample_rate_hz,
    const std::uint32_t channel_bandwidth_hz, InputSampleStamp stamp) {
    if (interleaved_iq.empty() || sample_rate_hz == 0 ||
        (interleaved_iq.size() % 2) != 0) {
        return;
    }
    const std::size_t incoming_samples = interleaved_iq.size() / 2;
    if (!stamp.valid_for(incoming_samples, sample_rate_hz)) {
        stamp = impl_->fallback_input_timeline.stamp(incoming_samples,
                                                     sample_rate_hz);
    }
    InputBlock block{.samples = std::vector<std::int16_t>(
                         interleaved_iq.begin(), interleaved_iq.end()),
                     .rate = sample_rate_hz,
                     .bandwidth = channel_bandwidth_hz,
                     .generation = 0,
                     .stamp = stamp};
    static_cast<void>(impl_->sample_channel.submit(
        std::move(block), buffered_input_samples(sample_rate_hz), true));
}

void StreamDecoder::flush() {
    impl_->sample_channel.request_flush();
    wait_until_idle();
}

void StreamDecoder::wait_until_idle() {
    std::unique_lock lock(impl_->mutex);
    impl_->idle.wait(lock, [this] {
        const auto channel = impl_->sample_channel.snapshot();
        const auto fec = impl_->fec_stage->snapshot();
        return channel.stopping ||
               (channel.queued_blocks == 0 && !channel.frontend_busy &&
                !channel.demod_busy && !fec.processing &&
                !channel.flush_requested && !channel.reset_requested &&
                fec.queued_items == 0 &&
                (!channel.ring_closed || channel.ring_used_samples == 0));
    });
}

void StreamDecoder::request_reset() {
    impl_->analyzer.reset();
    impl_->analysis_publisher.reset();
    impl_->fallback_input_timeline.mark_discontinuity();
    static_cast<void>(impl_->sample_channel.request_reset());
    impl_->fec_stage->clear();
    impl_->clock_control.reset();
    const std::uint64_t sync_version = impl_->demod_stage->reset();
    impl_->sample_channel.publish_sync_version(sync_version);
    impl_->fec_stage->notify_cancelled();
}

void StreamDecoder::reset() {
    request_reset();
    // Wait for the front-end to consume the reset. reset() is called when a
    // source is closed or re-opened, and the caller may start submitting the
    // next stream immediately after it returns: an asynchronous clear would
    // race with those fresh submits and sweep the new data away (the
    // queue.clear() in the reset path cannot tell pre-reset from post-reset
    // blocks). Blocking here also lets live sources drop nothing: by the time
    // the caller re-opens, the pipeline is already drained and parked.
    impl_->sample_channel.wait_reset_complete(
        impl_->sample_channel.generation());
}

void StreamDecoder::set_parameters(const ReceiverParameters &parameters) {
    impl_->analyzer.set_parameters(parameters);
    impl_->demod_stage->set_parameters(parameters);

    // A live SDR producer never becomes globally idle: it may keep filling the
    // input queue/ring while the GUI changes a demodulator setting. Wait only
    // for the frontend reset generation to be acknowledged, not for the
    // entire live pipeline to drain. Finite streams still use flush() and the
    // full wait_until_idle() path.
    reset();
}

void StreamDecoder::set_transport_callback(TransportCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->callback = std::move(callback);
}

void StreamDecoder::set_discontinuity_callback(DiscontinuityCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->discontinuity_callback = std::move(callback);
}

void StreamDecoder::set_equalized_callback(EqualizedCallback callback) {
    impl_->demod_stage->set_equalized_callback(std::move(callback));
}

StreamDecoderStats StreamDecoder::stats() const {
    const auto channel = impl_->sample_channel.snapshot();
    const auto clock = impl_->clock_control.snapshot();
    const auto fec = impl_->fec_stage->snapshot();
    auto statistics = impl_->demod_stage->stats();
    statistics.decoder_generation = channel.generation;
    {
        const std::scoped_lock lock(impl_->mutex);
        statistics.failed = impl_->terminal_exception != nullptr;
        statistics.error = impl_->terminal_error;
    }
    statistics.queued_blocks = channel.queued_blocks;
    statistics.queued_input_samples = channel.queued_input_samples;
    statistics.input_queue_capacity_samples = channel.input_capacity_samples;
    statistics.queued_symbols = fec.queued_items;
    statistics.symbol_queue_capacity = fec.capacity;
    statistics.ring_used_samples = channel.ring_used_samples;
    statistics.ring_capacity_samples = channel.ring_capacity_samples;
    statistics.frontend_state = impl_->frontend_stage->worker_state();
    statistics.demod_state = impl_->demod_stage->worker_state();
    statistics.fec_state = fec.worker_state;
    statistics.fec_processing = fec.processing;
    statistics.sro_resampler_command_ppm =
        static_cast<float>(clock.sro_command_ppm);
    statistics.sro_resampler_applied_ppm =
        static_cast<float>(clock.sro_applied_ppm);
    statistics.sro_resampler_ready = clock.sro_ready;
    statistics.cfo_resampler_command_hz =
        static_cast<float>(clock.cfo_command_hz);
    statistics.cfo_resampler_applied_hz =
        static_cast<float>(clock.cfo_applied_hz);
    statistics.cfo_resampler_ready = clock.cfo_ready;
    statistics.processing = channel.frontend_busy || channel.demod_busy ||
                            fec.processing || channel.queued_blocks != 0 ||
                            fec.queued_items != 0;
    return statistics;
}

void StreamDecoder::set_telemetry_enabled(
    const bool enabled, const TelemetryClock::time_point run_started_at) {
    impl_->demod_stage->set_telemetry_enabled(enabled, run_started_at);
}

std::vector<TelemetryRecord> StreamDecoder::drain_telemetry() {
    return impl_->demod_stage->drain_telemetry();
}

SignalSnapshot StreamDecoder::signal_snapshot() const {
    const auto analysis = analysis_snapshot();
    const auto statistics = stats();
    SignalSnapshot result;
    result.constellation_count =
        std::min(analysis.point_count, result.constellation.size());
    std::copy_n(analysis.points.begin(), result.constellation_count,
                result.constellation.begin());
    result.mer_db = analysis.mer_db;
    result.snr_db = analysis.cp_snr_db;
    result.deepest_notch_db = analysis.deepest_notch_db;
    result.carrier_offset_hz = analysis.carrier_offset_hz;
    const float fft_size =
        analysis.mode == TransmissionMode::k8 ? 8192.0F : 2048.0F;
    std::uint32_t channel_bandwidth_hz = 0;
    channel_bandwidth_hz =
        impl_->demod_stage->parameters().channel_bandwidth_hz;
    const float sample_rate_hz =
        static_cast<float>(channel_bandwidth_hz) * (8.0F / 7.0F);
    result.carrier_offset_limit_hz = sample_rate_hz / (2.0F * fft_size);
    result.sequence = analysis.sequence;
    result.signal_locked = analysis.locked;
    result.transport_locked = statistics.transport.rs_synchronized &&
                              statistics.transport.ts_packets != 0;
    return result;
}

PipelineSnapshot StreamDecoder::pipeline_snapshot() const {
    const auto statistics = stats();
    const auto queue_fraction = [](const std::size_t used,
                                   const std::size_t capacity) {
        return capacity == 0 ? 0.0F
                             : std::clamp(static_cast<float>(used) /
                                              static_cast<float>(capacity),
                                          0.0F, 1.0F);
    };

    PipelineSnapshot result;
    result.stages[0] = PipelineStageSnapshot{
        .name = "IQ queue",
        .queue_fraction =
            queue_fraction(statistics.queued_input_samples,
                           statistics.input_queue_capacity_samples),
        .workers = statistics.resample_workers,
        .queue_valid = statistics.input_queue_capacity_samples != 0,
    };
    result.stages[1] = PipelineStageSnapshot{
        .name = "Demod",
        .busy_fraction = statistics.demod_busy_fraction,
        .workers = statistics.symbol_workers,
        .busy_valid = true,
    };
    result.stages[2] = PipelineStageSnapshot{
        .name = "FEC queue",
        .queue_fraction = queue_fraction(statistics.queued_symbols,
                                         statistics.symbol_queue_capacity),
        .workers = statistics.transport.viterbi_workers,
        .queue_valid = statistics.symbol_queue_capacity != 0,
    };
    result.stage_count = 3;
    result.processing_realtime_ratio = statistics.processing_realtime_ratio;
    result.dropped_blocks = statistics.dropped_blocks;
    result.transport_bytes = statistics.transport_bytes;
    result.sequence = statistics.processed_chunks;
    result.processing = statistics.processing;
    result.failed = statistics.failed;
    result.error = statistics.error;
    return result;
}

SignalAnalysisSnapshot StreamDecoder::analysis_snapshot() const {
    auto snapshot = impl_->analysis_publisher.snapshot();
    if (snapshot.locked) {
        return snapshot;
    }
    snapshot = impl_->analyzer.snapshot();
    snapshot.source = snapshot.locked ? SignalAnalysisSource::prelock_monitor
                                      : SignalAnalysisSource::none;
    return snapshot;
}

void StreamDecoder::set_signal_smoothing(const bool enabled, const int speed) {
    impl_->analyzer.set_snr_smoothing(enabled, speed);
    impl_->analysis_publisher.set_smoothing(enabled, speed);
}

} // namespace airspy_tv::dvbt
