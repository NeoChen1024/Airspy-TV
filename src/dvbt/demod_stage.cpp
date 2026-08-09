#include "demod_stage_internal.hpp"

namespace airspy_tv::dvbt {

DemodStage::Impl::Impl(SampleChannel &selected_samples,
                       ClockControlTimeline &selected_clock,
                       FecStage &selected_fec,
                       AnalysisPublisher &selected_analysis,
                       Callbacks selected_callbacks)
    : sample_channel(selected_samples), clock_control(selected_clock),
      fec_stage(selected_fec), analysis_publisher(selected_analysis),
      callbacks(std::move(selected_callbacks)) {
    if (!callbacks.emit_discontinuity || !callbacks.notify_idle ||
        !callbacks.worker_failure) {
        throw std::invalid_argument("incomplete demod stage callbacks");
    }
    worker = std::thread([this] {
        set_current_thread_name("dvbt-demod");
        run_guarded();
    });
}

DemodStage::Impl::~Impl() noexcept { stop(); }

void DemodStage::Impl::stop() noexcept {
    sample_channel.stop();
    if (worker.joinable()) {
        worker.join();
    }
}

void DemodStage::Impl::run_guarded() noexcept {
    try {
        run();
    } catch (...) {
        const std::exception_ptr error = std::current_exception();
        std::string message = "demod worker failed";
        try {
            std::rethrow_exception(error);
        } catch (const std::exception &exception) {
            message += ": ";
            message += exception.what();
        } catch (...) {
            message += ": unknown exception";
        }
        callbacks.worker_failure(error, std::move(message));
    }
}

double DemodStage::Impl::telemetry_elapsed_ms() const noexcept {
    return std::chrono::duration<double, std::milli>(TelemetryClock::now() -
                                                     telemetry_started_at)
        .count();
}

bool DemodStage::Impl::events_enabled() const noexcept {
    return telemetry_enabled.load(std::memory_order_relaxed);
}

void DemodStage::Impl::emit_event(
    std::string event, const DecoderEventSeverity severity,
    const std::uint64_t generation,
    const std::optional<std::uint64_t> resampled_sample,
    const std::optional<std::uint64_t> ofdm_symbol, DecoderEventFields fields) {
    if (!events_enabled()) {
        return;
    }
    const std::scoped_lock lock(mutex);
    if (!events_enabled()) {
        return;
    }
    std::uint64_t source_epoch = 0;
    std::optional<std::uint64_t> source_sample;
    if (resampled_sample.has_value()) {
        if (const auto mapped =
                clock_control.input_at_output(*resampled_sample)) {
            source_epoch = mapped->stream_epoch;
            source_sample = mapped->input_sample;
        }
    }
    DecoderEventTelemetry record;
    record.envelope = {
        .sequence = ++event_telemetry_sequence,
        .decoder_generation = generation,
        .source_epoch = source_epoch,
        .wall_elapsed_ms = telemetry_elapsed_ms(),
    };
    record.event = std::move(event);
    record.severity = severity;
    record.source_sample = source_sample;
    record.resampled_sample = resampled_sample;
    record.ofdm_symbol = ofdm_symbol;
    record.fields = std::move(fields);
    telemetry_queue.emplace_back(std::move(record));
}

void DemodStage::Impl::emit_diagnostic_event(
    DiagnosticEvent diagnostic, const std::uint64_t generation,
    const std::uint64_t source_epoch, const std::uint64_t demod_window_sequence,
    const std::uint64_t fec_session,
    const std::optional<std::uint64_t> tps_symbol_index) {
    if (!events_enabled()) {
        return;
    }
    diagnostic.fields.emplace("fec_session", fec_session);
    if (demod_window_sequence != 0) {
        diagnostic.fields.emplace("demod_window_sequence",
                                  demod_window_sequence);
    }
    if (tps_symbol_index.has_value()) {
        diagnostic.fields.emplace("tps_symbol_index", *tps_symbol_index);
    }
    const std::scoped_lock lock(mutex);
    if (!events_enabled() || generation != sample_channel.generation()) {
        return;
    }
    DecoderEventTelemetry record;
    record.envelope = {
        .sequence = ++event_telemetry_sequence,
        .decoder_generation = generation,
        .source_epoch = source_epoch,
        .wall_elapsed_ms = telemetry_elapsed_ms(),
    };
    record.event = std::move(diagnostic.name);
    record.severity = diagnostic.severity;
    record.fields = std::move(diagnostic.fields);
    telemetry_queue.emplace_back(std::move(record));
}

void DemodStage::Impl::fire_pending_discontinuity() {
    std::optional<TransportDiscontinuity> pending;
    {
        const std::scoped_lock lock(mutex);
        pending = pending_discontinuity;
        pending_discontinuity.reset();
    }
    if (pending.has_value()) {
        callbacks.emit_discontinuity(*pending);
    }
}

bool DemodStage::Impl::enqueue_fec(FecItem item) const {
    if (sample_channel.cancelled()) {
        return false;
    }
    return fec_stage.enqueue(std::move(item));
}

void DemodStage::Impl::reset_frontend_state() noexcept {
    frontend.valid = false;
    frontend.fft_size = 0;
    frontend.guard_size = 0;
    frontend.residual_phase_ema = 0.0F;
    frontend.carrier_offset = std::numeric_limits<int>::max();
    frontend.previous_continual.clear();
    frontend.previous_phase = -1;
    frontend.phase_discontinuities = 0;
    frontend.last_symbol_start = 0;
    frontend.just_seeded = false;
    frontend.tps_decoder.reset();
    frontend.tps_snapshot = {};
    frontend.stable_carrier_offset = std::numeric_limits<int>::max();
    frontend.stable_phase = -1;
}

DemodStage::DemodStage(SampleChannel &samples, ClockControlTimeline &clock,
                       FecStage &fec, AnalysisPublisher &analysis,
                       Callbacks callbacks)
    : impl_(std::make_unique<Impl>(samples, clock, fec, analysis,
                                   std::move(callbacks))) {}

DemodStage::~DemodStage() noexcept = default;

void DemodStage::stop() noexcept { impl_->stop(); }

WorkerState DemodStage::worker_state() const noexcept {
    return static_cast<WorkerState>(
        impl_->demod_state.load(std::memory_order_relaxed));
}

void DemodStage::set_parameters(const ReceiverParameters &parameters) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->parameters = parameters;
}

ReceiverParameters DemodStage::parameters() const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->parameters;
}

void DemodStage::set_equalized_callback(
    StreamDecoder::EqualizedCallback callback) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->equalized_callback = std::move(callback);
}

std::uint64_t DemodStage::reset() {
    const std::scoped_lock lock(impl_->mutex);
    impl_->sync.valid = false;
    ++impl_->sync.version;
    impl_->latest = {};
    return impl_->sync.version;
}

void DemodStage::input_block_started(const std::uint64_t generation) {
    const std::scoped_lock lock(impl_->mutex);
    if (generation == impl_->sample_channel.generation()) {
        ++impl_->latest.input_blocks;
    }
}

void DemodStage::input_block_dropped() {
    const std::scoped_lock lock(impl_->mutex);
    ++impl_->latest.dropped_blocks;
}

std::uint64_t DemodStage::invalidate_sync(const std::uint64_t generation) {
    const std::scoped_lock lock(impl_->mutex);
    if (generation == impl_->sample_channel.generation()) {
        impl_->sync.valid = false;
        ++impl_->sync.version;
    }
    return impl_->sync.version;
}

std::uint64_t DemodStage::publish_rebootstrap(
    const SampleChannel::RebootstrapResult &result) {
    const std::scoped_lock lock(impl_->mutex);
    if (result.new_generation != impl_->sample_channel.generation()) {
        return impl_->sync.version;
    }
    auto &latest = impl_->latest;
    latest.processed_input_samples += result.consumed_input_samples;
    impl_->sync.valid = false;
    ++impl_->sync.version;
    latest.decoder_generation = result.new_generation;
    latest.ofdm_locked = false;
    latest.tps_locked = false;
    latest.cfo_resampler_ready = false;
    latest.cfo_resampler_command_hz = 0.0F;
    latest.cfo_resampler_applied_hz = 0.0F;
    latest.sro_resampler_ready = false;
    latest.sro_resampler_command_ppm = 0.0F;
    latest.sro_resampler_applied_ppm = 0.0F;
    latest.cfo_pending_commands = 0;
    latest.sro_pending_commands = 0;
    latest.bootstrap_attempts = 0;
    latest.bootstrap_replayed_input_samples = 0;
    latest.bootstrap_retained_peak_samples = 0;
    ++latest.cfo_rebootstrap_count;
    impl_->callbacks.notify_idle();
    return impl_->sync.version;
}

void DemodStage::publish_bootstrap_progress(
    const FrontendBootstrapProgress &progress) {
    const std::scoped_lock lock(impl_->mutex);
    if (progress.generation != impl_->sample_channel.generation()) {
        return;
    }
    impl_->latest.bootstrap_attempts = progress.attempts;
    impl_->latest.bootstrap_retained_peak_samples =
        progress.retained_peak_samples;
    if (progress.acquisition_time_ms.has_value()) {
        impl_->latest.last_acquisition_time_ms = *progress.acquisition_time_ms;
    }
}

std::uint64_t DemodStage::publish_acquisition(
    const FrontendAcquisitionPublication &publication) {
    const std::scoped_lock lock(impl_->mutex);
    if (publication.generation != impl_->sample_channel.generation()) {
        return impl_->sync.version;
    }
    const auto &acquisition = publication.acquisition;
    impl_->stable_mode = acquisition.mode;
    impl_->stable_guard = acquisition.guard;
    impl_->sync.valid = true;
    impl_->sync.start_pos = publication.output_base + acquisition.start;
    impl_->sync.acquisition_pilot_phase = acquisition.pilot_phase;
    impl_->sync.score = acquisition.score;
    impl_->sync.mode = acquisition.mode;
    impl_->sync.guard = acquisition.guard;
    impl_->sync.fft_size = acquisition.fft_size;
    impl_->sync.guard_size = acquisition.guard_size;
    impl_->sync.bandwidth = publication.bandwidth_hz;
    impl_->sync.resampled_rate = publication.resampled_rate_hz;
    auto &latest = impl_->latest;
    latest.acquisition_score = acquisition.score;
    latest.acquisition_cfo_hz = publication.initial_cfo_hz;
    latest.acquisition_fractional_cfo_hz =
        publication.initial_fractional_cfo_hz;
    latest.acquisition_carrier_bin_offset = acquisition.carrier_offset;
    latest.bootstrap_attempts = publication.bootstrap_attempts;
    latest.bootstrap_replayed_input_samples =
        publication.bootstrap_replayed_input_samples;
    latest.bootstrap_retained_peak_samples =
        publication.bootstrap_retained_peak_samples;
    latest.carrier_bin_offset = 0;
    latest.tracked_carrier_offset_hz = publication.initial_cfo_hz;
    latest.cfo_resampler_ready = true;
    latest.cfo_resampler_command_hz = publication.initial_cfo_hz;
    return ++impl_->sync.version;
}

void DemodStage::publish_command(
    const FrontendCommandPublication &publication) {
    const std::scoped_lock lock(impl_->mutex);
    if (publication.generation != impl_->sample_channel.generation()) {
        return;
    }
    auto &latest = impl_->latest;
    latest.sro_pending_commands = publication.pending_sro;
    latest.cfo_pending_commands = publication.pending_cfo;
    if (publication.sro.has_value()) {
        latest.sro_applied_input_sample = publication.input_sample;
        latest.sro_schedule_late_samples =
            publication.input_sample - publication.sro->effective_input_sample;
    }
    if (publication.cfo.has_value()) {
        latest.cfo_applied_effective_input_sample =
            publication.cfo->effective_input_sample;
        latest.cfo_applied_input_sample = publication.input_sample;
        latest.cfo_schedule_late_samples =
            publication.input_sample - publication.cfo->effective_input_sample;
        latest.cfo_applied_output_sample = publication.output_sample;
    }
}

void DemodStage::publish_frontend_block(
    const FrontendBlockPublication &publication) {
    const std::scoped_lock lock(impl_->mutex);
    if (publication.generation != impl_->sample_channel.generation()) {
        return;
    }
    auto &latest = impl_->latest;
    latest.processed_input_samples += publication.input_samples;
    latest.decoder_generation = publication.generation;
    latest.source_epoch = publication.stamp.stream_epoch;
    latest.last_frontend_block_wall_time_ms = publication.total_time_ms;
    latest.last_frontend_convert_time_ms = publication.convert_time_ms;
    latest.last_frontend_resample_time_ms = publication.resample_time_ms;
    latest.last_frontend_ring_copy_time_ms = publication.ring_copy_time_ms;
    latest.last_frontend_ring_wait_time_ms = publication.ring_wait_time_ms;
    latest.sro_resampler_applied_ppm =
        static_cast<float>(publication.clock.sro_applied_ppm);
    latest.cfo_resampler_applied_hz =
        static_cast<float>(publication.clock.cfo_applied_hz);
    latest.resampler_requested_ratio = publication.requested_ratio;
    latest.resampler_effective_ratio = publication.effective_ratio;
    if (!impl_->events_enabled()) {
        return;
    }

    const double accounted =
        static_cast<double>(publication.convert_time_ms) +
        static_cast<double>(publication.resample_time_ms) +
        static_cast<double>(publication.ring_copy_time_ms) +
        static_cast<double>(publication.ring_wait_time_ms);
    FrontendBlockTelemetry record;
    record.envelope = {
        .sequence = ++impl_->frontend_telemetry_sequence,
        .decoder_generation = publication.generation,
        .source_epoch = publication.stamp.stream_epoch,
        .wall_elapsed_ms = impl_->telemetry_elapsed_ms(),
    };
    record.input_sample_rate_hz = publication.input_rate_hz;
    record.channel_bandwidth_hz = publication.bandwidth_hz;
    record.source_begin_sample = publication.stamp.begin_sample;
    record.source_end_sample = publication.stamp.end_sample();
    record.resampled_begin_sample = publication.resampled_begin_sample;
    record.resampled_end_sample = publication.resampled_end_sample;
    record.input_complex_samples = publication.input_samples;
    record.resampled_complex_samples =
        publication.resampled_end_sample - publication.resampled_begin_sample;
    record.discontinuity_before = publication.stamp.discontinuity_before;
    record.abandoned = publication.abandoned;
    record.bootstrap_attempts = publication.bootstrap_attempts;
    record.bootstrap_replayed_input_samples =
        publication.bootstrap_replayed_input_samples;
    record.bootstrap_retained_peak_samples =
        publication.bootstrap_retained_peak_samples;
    record.requested_ratio = publication.requested_ratio;
    record.effective_ratio = publication.effective_ratio;
    record.commanded_sro_ppm = publication.clock.sro_command_ppm;
    record.applied_sro_ppm = publication.clock.sro_applied_ppm;
    record.commanded_cfo_hz = publication.clock.cfo_command_hz;
    record.applied_cfo_hz = publication.clock.cfo_applied_hz;
    record.command_output_sample = latest.sro_command_output_sample;
    record.command_input_sample = latest.sro_command_input_sample;
    record.effective_input_sample = latest.sro_effective_input_sample;
    record.applied_input_sample = latest.sro_applied_input_sample;
    record.fixed_delay_samples = latest.sro_fixed_delay_samples;
    record.late_samples = latest.sro_schedule_late_samples;
    record.pending_commands = publication.clock.pending_sro;
    record.cfo_command_output_sample = latest.cfo_command_output_sample;
    record.cfo_command_input_sample = latest.cfo_command_input_sample;
    record.cfo_effective_input_sample = latest.cfo_effective_input_sample;
    record.cfo_applied_effective_input_sample =
        latest.cfo_applied_effective_input_sample;
    record.cfo_applied_input_sample = latest.cfo_applied_input_sample;
    record.cfo_applied_output_sample = latest.cfo_applied_output_sample;
    record.cfo_fixed_delay_samples = latest.cfo_fixed_delay_samples;
    record.cfo_late_samples = latest.cfo_schedule_late_samples;
    record.cfo_input_sample_rate_hz = latest.cfo_input_sample_rate_hz;
    record.cfo_pending_commands = publication.clock.pending_cfo;
    record.cfo_rebootstrap_requests = latest.cfo_rebootstrap_requests;
    record.cfo_rebootstrap_count = latest.cfo_rebootstrap_count;
    record.cfo_rebootstrap_last_residual_hz =
        latest.cfo_rebootstrap_last_residual_hz;
    record.cfo_rebootstrap_output_sample = latest.cfo_rebootstrap_output_sample;
    record.cfo_rebootstrap_source_sample = latest.cfo_rebootstrap_source_sample;
    record.serial_wall_ms = {
        {"frontend::total", publication.total_time_ms},
        {"frontend::convert", publication.convert_time_ms},
        {"frontend::resample", publication.resample_time_ms},
        {"frontend::ring_copy", publication.ring_copy_time_ms},
        {"frontend::ring_wait", publication.ring_wait_time_ms},
        {"frontend::other",
         std::max(0.0,
                  static_cast<double>(publication.total_time_ms) - accounted)},
    };
    impl_->telemetry_queue.emplace_back(std::move(record));
}

void DemodStage::publish_fec_session(const FecStageSession &session) {
    const std::scoped_lock lock(impl_->mutex);
    if (session.generation != impl_->sample_channel.generation()) {
        return;
    }
    impl_->latest.fec_sessions = session.fec_session;
    impl_->latest.cumulative_transport = session.cumulative;
}

void DemodStage::publish_fec_window(const FecStageWindow &window) {
    const std::scoped_lock lock(impl_->mutex);
    if (window.generation != impl_->sample_channel.generation()) {
        return;
    }
    auto &latest = impl_->latest;
    latest.fec_work_time_ms = window.fec_total_ms;
    latest.transport_bytes += window.output_bytes_delta;
    latest.transport = window.session;
    latest.cumulative_transport = window.cumulative;
    latest.fec_sessions = window.fec_session;
    latest.transport_work_time_ms = window.transport_nested_ms;
    if (impl_->events_enabled() && window.demod_window_sequence != 0) {
        FecWindowTelemetry record;
        record.envelope = {
            .sequence = ++impl_->fec_telemetry_sequence,
            .decoder_generation = window.generation,
            .source_epoch = window.source_epoch,
            .wall_elapsed_ms = impl_->telemetry_elapsed_ms(),
        };
        record.demod_window_sequence = window.demod_window_sequence;
        record.fec_session = window.fec_session;
        record.output_bytes_delta = window.output_bytes_delta;
        record.output_bytes_cumulative = latest.transport_bytes;
        record.session = window.session;
        record.delta = window.delta;
        record.cumulative = window.cumulative;
        record.fec_total_ms = window.fec_total_ms;
        record.transport_nested_ms = window.transport_nested_ms;
        impl_->telemetry_queue.emplace_back(record);
    }
}

bool DemodStage::events_enabled() const noexcept {
    return impl_->events_enabled();
}

void DemodStage::emit_fec_diagnostic(DiagnosticEvent event,
                                     const FecStageDiagnosticContext &context) {
    impl_->emit_diagnostic_event(std::move(event), context.generation,
                                 context.source_epoch,
                                 context.demod_window_sequence,
                                 context.fec_session, context.tps_symbol_index);
}

StreamDecoderStats DemodStage::stats() const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->latest;
}

void DemodStage::set_telemetry_enabled(
    const bool enabled, const TelemetryClock::time_point run_started_at) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->telemetry_enabled.store(enabled, std::memory_order_relaxed);
    impl_->telemetry_started_at = run_started_at;
    impl_->telemetry_queue.clear();
    impl_->frontend_telemetry_sequence = 0;
    impl_->fec_telemetry_sequence = 0;
    impl_->event_telemetry_sequence = 0;
}

std::vector<TelemetryRecord> DemodStage::drain_telemetry() {
    const std::scoped_lock lock(impl_->mutex);
    std::vector<TelemetryRecord> result;
    result.reserve(impl_->telemetry_queue.size());
    while (!impl_->telemetry_queue.empty()) {
        result.push_back(std::move(impl_->telemetry_queue.front()));
        impl_->telemetry_queue.pop_front();
    }
    return result;
}

} // namespace airspy_tv::dvbt
