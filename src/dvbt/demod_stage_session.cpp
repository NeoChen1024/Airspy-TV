#include "demod_stage_internal.hpp"

namespace airspy_tv::dvbt {

// Demod session, acquisition, and ring-coordination helpers.

void DemodStage::Impl::demod_cold_seed(DemodRuntimeState &state) {
    // DemodRuntimeState lives for the worker thread rather than for one
    // source. A new acquisition must not inherit a recovery decision from a
    // previous file / retune: before the first pilot lock, previous_phase and
    // carrier_offset deliberately hold sentinels, so carrying a stale
    // hopeless-window count into the freeze path would turn those sentinels
    // into an out-of-range PilotLock.
    state.lock_hold = 0;
    state.frozen_symbol_count = 0;
    state.cfo_recovery_symbol_count = 0;
    state.cfo_healthy_symbol_count = 0;
    state.hopeless_window_count = 0;
    state.stable_pending_offset = std::numeric_limits<int>::max();
    state.stable_pending_count = 0;
    state.tps_mismatch_symbols = 0;
    frontend.residual_phase_ema = 0.0F;
    frontend.carrier_offset = 0;
    frontend.previous_continual.clear();
    frontend.have_previous_continual = false;
    // The bootstrap acquisition FFT already established the scattered-pilot
    // phase at carrier offset zero. Seed the previous phase so the first
    // production symbol validates the predicted next phase instead of
    // entering any wide carrier-search path.
    frontend.previous_phase = (sync.acquisition_pilot_phase + 3) % 4;
    frontend.tps_decoder.reset();
    frontend.tps_snapshot = {};
    frontend.just_seeded = true;
}

void DemodStage::Impl::demod_build_grid(DemodRuntimeState &state) {
    state.maximum = frontend.fft_size == 8192 ? 6816 : 1704;
    state.fft_size = frontend.fft_size;
    state.guard_size = frontend.guard_size;
    state.period = state.fft_size + state.guard_size;
    state.continual_indices.clear();
    state.tps_indices.clear();
    state.pilot_indices = {};
    state.payload_indices = {};
    for (std::size_t k = 0; k <= state.maximum; ++k) {
        const std::size_t base = k % 1704;
        const bool continual_carrier = listed(continual_2k, base);
        const bool tps_carrier = listed(tps_2k, base);
        if (continual_carrier) {
            state.continual_indices.push_back(k);
        }
        if (tps_carrier) {
            state.tps_indices.push_back(k);
        }
        for (std::size_t phase = 0; phase < 4; ++phase) {
            const bool scattered = k % 12 == phase * 3;
            if (scattered || continual_carrier) {
                state.pilot_indices[phase].push_back(k);
            }
            if (!scattered && !continual_carrier && !tps_carrier) {
                state.payload_indices[phase].push_back(k);
            }
        }
    }
    state.tps_values.resize(state.tps_indices.size());
    state.fft_in.resize(state.fft_size);
    state.fft_out.resize(state.fft_size);
    state.channel_scratch.resize(state.maximum + 1);
    frontend.previous_continual.resize(state.continual_indices.size());
    frontend.current_continual.resize(state.continual_indices.size());
    frontend.have_previous_continual = false;
    const auto fft_plan_started_at = std::chrono::steady_clock::now();
    state.plan = FftwfPlan::dft_1d(
        static_cast<int>(state.fft_size),
        reinterpret_cast<fftwf_complex *>(state.fft_in.data()),
        reinterpret_cast<fftwf_complex *>(state.fft_out.data()), FFTW_FORWARD,
        FFTW_MEASURE);
    state.fft_plan_time_ms = duration_ms(fft_plan_started_at);
    // FFTW_MEASURE is allowed to overwrite both arrays while selecting a
    // plan. Restore deterministic contents; the addresses remain stable for
    // the entire plan lifetime and each later symbol overwrites fft_in.
    std::ranges::fill(state.fft_in, std::complex<float>{});
    std::ranges::fill(state.fft_out, std::complex<float>{});

    // One CIR tap per scattered-pilot slot. The response spans Tu/12.
    frontend.cir_n = state.fft_size == 8192 ? 1024 : 256;
    frontend.cir_grid.assign(frontend.cir_n, std::complex<float>{});
    frontend.cir_response.assign(frontend.cir_n, std::complex<float>{});
    frontend.cir_energy.assign(frontend.cir_n, 0.0);
    frontend.cir_plan = FftwfPlan::dft_1d(
        static_cast<int>(frontend.cir_n),
        reinterpret_cast<fftwf_complex *>(frontend.cir_grid.data()),
        reinterpret_cast<fftwf_complex *>(frontend.cir_response.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
}

void DemodStage::Impl::demod_reset_timing(DemodRuntimeState &state,
                                          const std::size_t tracker_fft_size,
                                          const bool reset_window_cir) {
    state.smoothed_sample_clock_ppm = 0.0;
    state.latest_cp_snr_db = 0.0F;
    state.latest_deepest_notch_db = 0.0F;
    state.last_windowed_timing = 0.0;
    state.last_windowed_cir_avg = 0.0;
    state.tau_history.fill(0.0);
    state.tau_sample_history.fill(0);
    state.tau_interval_correction_history.fill(0.0);
    state.tau_history_head = 0;
    state.tau_history_count = 0;
    state.last_timing_sample_position.reset();
    if (reset_window_cir) {
        state.window_cir_offset_sum = 0.0;
    }
    state.cir_confidence = 0.0;
    state.timing_tracker.reset(tracker_fft_size);
}

bool DemodStage::Impl::demod_handle_sync_change(DemodRuntimeState &state) {
    state.seen_sync_version = sync.version;
    state.demod_generation = sample_channel.generation();
    const auto anchored_start = [this]() -> std::uint64_t {
        return static_cast<std::uint64_t>(
            static_cast<std::int64_t>(sync.start_pos + sync.guard_size) +
            static_cast<std::int64_t>(std::lround(frontend.cir_offset)));
    };

    if (!sync.valid) {
        // The content may have changed entirely, so playback must restart
        // rather than concatenate across the abandoned generation.
        pending_discontinuity = TransportDiscontinuity::retune;
        if (state.postprocessor != nullptr) {
            static_cast<void>(symbol_postprocessor->flush());
        }
        state.postprocessor = nullptr;
        state.postprocessor_pending_symbols = 0;
        state.pending_symbols.clear();
        state.gate_buffer.clear();
        state.in_hopeless_region = false;
        state.decoder_parameters.reset();
        state.have_grid = false;
        frontend.valid = false;
        frontend.just_seeded = false;
        frontend.cir_offset = 0.0F;
        frontend.cir_symbol_count = 0;
        state.applied_cir_offset = 0;
        demod_reset_timing(state, 0, true);
        demod_reset_stats_window(state);
        reset_frontend_state();
        stable_mode.reset();
        stable_guard.reset();
        if (sample_channel.snapshot().reset_requested) {
            sample_channel.acknowledge_demod_reset(state.demod_generation);
        }
        return true;
    }

    const bool mode_changed = !frontend.valid ||
                              sync.fft_size != frontend.fft_size ||
                              sync.guard_size != frontend.guard_size;
    if (!state.have_grid || mode_changed) {
        frontend.mode = sync.mode;
        frontend.guard = sync.guard;
        frontend.fft_size = sync.fft_size;
        frontend.guard_size = sync.guard_size;
        frontend.valid = true;
        demod_build_grid(state);
        demod_cold_seed(state);

        // The caller already holds the coordinator mutex.
        state.selected_parameters = parameters;
        if (!state.selected_parameters.mode.has_value() &&
            stable_mode.has_value()) {
            state.selected_parameters.mode = stable_mode;
        }
        if (!state.selected_parameters.guard_interval.has_value() &&
            stable_guard.has_value()) {
            state.selected_parameters.guard_interval = stable_guard;
        }
        state.workers =
            allocate_workers(state.selected_parameters.worker_threads);
        state.symbol_queue_capacity = buffered_symbol_count(
            sync.bandwidth, state.fft_size + state.guard_size);
        fec_stage.set_capacity(state.symbol_queue_capacity);
        if (state.selected_parameters.constellation.has_value() &&
            state.selected_parameters.code_rate.has_value()) {
            state.decoder_parameters = DecoderParameters{
                frontend.mode, *state.selected_parameters.constellation,
                *state.selected_parameters.code_rate, state.workers.viterbi};
        } else {
            state.decoder_parameters.reset();
        }
        if (state.postprocessor != nullptr) {
            static_cast<void>(symbol_postprocessor->flush());
            state.postprocessor = nullptr;
        }
        state.postprocessor_pending_symbols = 0;
        state.pending_symbols.clear();
        state.gate_buffer.clear();
        state.in_hopeless_region = false;
        state.next_symbol_start = anchored_start();
        state.last_reanchor_carried = false;
        state.applied_cir_offset =
            static_cast<int>(std::lround(frontend.cir_offset));
        demod_reset_timing(state, state.fft_size, true);
        state.have_grid = true;
        sample_channel.set_demod_busy(true);
        return true;
    }

    const std::uint64_t new_start = anchored_start();
    const std::uint64_t delta = state.next_symbol_start > new_start
                                    ? state.next_symbol_start - new_start
                                    : new_start - state.next_symbol_start;
    const std::uint64_t aligned_distance =
        std::min(delta % state.period, state.period - delta % state.period);
    if (aligned_distance < 64) {
        return false;
    }

    // The boundary moved, but mode and guard are unchanged, so carrier
    // tracking carries while the timing loop restarts on the new grid.
    const bool carried = frontend.valid;
    const std::uint64_t new_next =
        sample_channel.align_at_or_after(new_start, state.period);
    state.next_symbol_start = new_next;
    state.applied_cir_offset =
        static_cast<int>(std::lround(frontend.cir_offset));
    state.last_reanchor_carried = carried;
    state.stable_pending_offset = std::numeric_limits<int>::max();
    state.stable_pending_count = 0;
    demod_reset_timing(state, state.fft_size, false);
    return true;
}

float DemodStage::Impl::demod_run_acquisition(DemodRuntimeState &state,
                                              const bool wait_for_data) {
    if (wait_for_data) {
        // A live decoder keeps little backlog. Wait for a useful acquisition
        // window while recovering, but bound the wait so reset remains prompt.
        const auto wait_started = std::chrono::steady_clock::now();
        for (;;) {
            const auto channel = sample_channel.snapshot();
            const bool cancel = channel.stopping || channel.reset_requested ||
                                sample_channel.cancelled();
            const std::uint64_t now_available = channel.ring_used_samples;
            if (cancel || duration_ms(wait_started) > 1000.0F) {
                if (!cancel && events_enabled()) {
                    emit_event(
                        "acquisition_wait_timeout",
                        DecoderEventSeverity::warning, state.demod_generation,
                        state.next_symbol_start, state.symbol_count,
                        {{"available_resampled_samples", now_available}});
                }
                return 0.0F;
            }
            if (now_available >= acquisition_samples) {
                break;
            }
            static_cast<void>(sample_channel.wait_for_available(
                acquisition_samples, state.demod_generation,
                std::chrono::milliseconds(4)));
        }
    }

    const auto acquisition_work_started_at = std::chrono::steady_clock::now();
    ReceiverParameters acquisition_parameters;
    const auto acquisition_window =
        sample_channel.acquisition_window(acquisition_samples, 2U * 10240U);
    if (acquisition_window.samples.empty()) {
        return 0.0F;
    }
    {
        const std::scoped_lock lock(mutex);
        acquisition_parameters = parameters;
    }
    const std::uint64_t base = acquisition_window.base;
    const std::uint64_t acquisition_generation = acquisition_window.generation;

    const auto acquisition_started_at = std::chrono::steady_clock::now();
    const OfdmAcquisition acquisition = acquire_ofdm(
        std::span(acquisition_window.samples), acquisition_parameters, false);
    const float acquisition_elapsed_ms = duration_ms(acquisition_started_at);
    if (state.demod_busy_active) {
        state.reacquisition_time_sum_ms +=
            duration_ms(acquisition_work_started_at);
    }
    if (acquisition.score < 0.20F) {
        if (acquisition_generation == sample_channel.generation() &&
            !sample_channel.cancelled()) {
            state.acquisition_time_ms = acquisition_elapsed_ms;
        }
        return 0.0F;
    }

    std::uint64_t published_sync_version = 0;
    {
        const std::scoped_lock lock(mutex);
        // Acquisition runs without the coordinator mutex. Never publish a
        // copied window after reset or generation advance.
        if (sample_channel.cancelled() ||
            acquisition_generation != sample_channel.generation()) {
            return 0.0F;
        }
        const std::uint32_t bandwidth = clock_control.snapshot().bandwidth_hz;
        const float resampled_rate =
            static_cast<float>(bandwidth) * (8.0F / 7.0F);
        state.acquisition_time_ms = acquisition_elapsed_ms;
        stable_mode = acquisition.mode;
        stable_guard = acquisition.guard;
        sync.valid = true;
        sync.start_pos = base + acquisition.start;
        sync.acquisition_pilot_phase = acquisition.pilot_phase;
        sync.score = acquisition.score;
        sync.mode = acquisition.mode;
        sync.guard = acquisition.guard;
        sync.fft_size = acquisition.fft_size;
        sync.guard_size = acquisition.guard_size;
        sync.bandwidth = bandwidth;
        sync.resampled_rate = resampled_rate;
        latest.acquisition_score = acquisition.score;
        published_sync_version = ++sync.version;
        static_cast<void>(demod_handle_sync_change(state));
    }
    sample_channel.publish_sync_version(published_sync_version);
    if (events_enabled()) {
        emit_event("acquisition_succeeded", DecoderEventSeverity::info,
                   acquisition_generation, base + acquisition.start,
                   state.symbol_count,
                   {{"score", static_cast<double>(acquisition.score)},
                    {"window_start", acquisition.start},
                    {"transmission_mode",
                     std::string(event_mode_name(acquisition.mode))},
                    {"guard_interval",
                     std::string(event_guard_name(acquisition.guard))}});
    }
    return acquisition.score;
}

DemodFlow DemodStage::Impl::demod_prepare_stream(DemodRuntimeState &state) {
    auto &have_grid = state.have_grid;
    auto &seen_sync_version = state.seen_sync_version;
    auto &decoder_parameters = state.decoder_parameters;
    auto &postprocessor = state.postprocessor;

    // --- wait for first-anchor data, a reset (the front-end
    //     only invalidates the sync on retune/reset), or the
    //     resume of a flushed stream ---
    // While working toward a first anchor, keep acquisition_pending set so a
    // flush cannot abandon a full-ring push between wakeup and acquisition.
    if (!have_grid) {
        sample_channel.set_acquisition_pending(true);
    }
    demod_state.store(static_cast<int>(WorkerState::waiting_sync));
    if (sample_channel.wait_for_stream(seen_sync_version, have_grid,
                                       acquisition_samples) ==
        SampleChannel::WaitStatus::stop) {
        demod_state.store(static_cast<int>(WorkerState::exited));
        return DemodFlow::stop;
    }
    bool stream_abandoned = false;
    {
        const std::scoped_lock lock(mutex);
        if (sync.version != seen_sync_version) {
            static_cast<void>(demod_handle_sync_change(state));
            stream_abandoned = !have_grid;
        }
    }
    fire_pending_discontinuity();
    if (stream_abandoned) {
        return DemodFlow::restart;
    }
    // Explicit constellation/code-rate parameters are sufficient
    // to start the FEC path; TPS is optional in that mode. This
    // also makes a manually configured synthetic/test signal
    // exercise the complete pipeline without manufacturing a
    // valid TPS frame.
    if (have_grid && decoder_parameters && postprocessor == nullptr &&
        !demod_start_decoder(state)) {
        return DemodFlow::stop;
    }
    if (!have_grid) {
        // First anchor: event-driven acquisition (acquisition no
        // longer runs on a fixed cadence; the only other events
        // are the long-fade re-anchor and a TPS mode change). The
        // demod is genuinely busy while it acquires, so
        // wait_until_idle blocks until it finishes (bounded work),
        // while the push-abandon still sees acquisition_pending
        // and never fires mid-acquisition.
        sample_channel.set_demod_busy(true);
        demod_state.store(static_cast<int>(WorkerState::processing));
        const float score = demod_run_acquisition(state);
        bool closed_without_grid = false;
        if (!have_grid) {
            sample_channel.set_demod_busy(false);
            sample_channel.set_acquisition_pending(false);
            closed_without_grid = sample_channel.snapshot().ring_closed;
            if (closed_without_grid) {
                // No acquisition means these samples cannot be consumed by
                // the symbol loop. Drop the closed stream's dead data before
                // parking for reset or a new source.
                sample_channel.discard_all();
            }
        }
        if (closed_without_grid) {
            // The stream ended without a signal. Stay alive for
            // the next stream: a reset bumps the sync version and
            // a reopened ring refills the data.
            demod_state.store(static_cast<int>(WorkerState::waiting_sync));
            if (sample_channel.wait_for_reopen(seen_sync_version) ==
                SampleChannel::WaitStatus::stop) {
                demod_state.store(static_cast<int>(WorkerState::exited));
                return DemodFlow::stop;
            }
        } else if (score == 0.0F) {
            sample_channel.set_demod_busy(false);
            sample_channel.set_acquisition_pending(false);
            // No signal yet (or the acquisition can never succeed
            // for this stream): back off so the retry cannot
            // busy-loop and stall the ring in front of the
            // front-end. The flush/reset notifications still wake
            // this wait. Parked here with demod_busy false, a
            // flush can abandon the front-end push (ring full)
            // and close the ring so the stream ends cleanly
            // instead of deadlocking.
            demod_state.store(
                static_cast<int>(WorkerState::waiting_acquisition));
            if (sample_channel.wait_for_retry(seen_sync_version,
                                              std::chrono::milliseconds(100)) ==
                SampleChannel::WaitStatus::stop) {
                demod_state.store(static_cast<int>(WorkerState::exited));
                return DemodFlow::stop;
            }
        }
        return DemodFlow::restart;
    }
    return DemodFlow::proceed;
}

DemodInputFlow DemodStage::Impl::demod_read_symbol(DemodRuntimeState &state) {
    auto &next_symbol_start = state.next_symbol_start;
    auto &fft_size = state.fft_size;
    auto &seen_sync_version = state.seen_sync_version;
    auto &fft_in = state.fft_in;
    auto &symbol_count = state.symbol_count;
    auto &guard_size = state.guard_size;
    auto &latest_cp_snr_db = state.latest_cp_snr_db;
    auto &demod_busy_started_at = state.demod_busy_started_at;

    demod_state.store(static_cast<int>(WorkerState::waiting_ring_data));
    const auto read = sample_channel.read_symbol(
        seen_sync_version, next_symbol_start, fft_size, guard_size,
        (symbol_count % analysis_interval_symbols) == 0U, fft_in);
    state.ring_wait_time_sum_ms += read.wait_time_ms;
    state.ring_copy_time_sum_ms += read.copy_time_ms;
    if (read.status == SampleChannel::WaitStatus::stop) {
        return DemodInputFlow::stop;
    }
    if (read.status == SampleChannel::WaitStatus::sync_changed ||
        read.status == SampleChannel::WaitStatus::closed) {
        return DemodInputFlow::end;
    }
    demod_busy_started_at = std::chrono::steady_clock::now();
    state.demod_busy_active = true;
    state.demod_busy_time_sum_ms += read.copy_time_ms;
    if (read.cyclic_prefix.valid) {
        const float rho =
            read.cyclic_prefix.prefix_power > 0.0 &&
                    read.cyclic_prefix.suffix_power > 0.0
                ? static_cast<float>(std::abs(read.cyclic_prefix.correlation) /
                                     std::sqrt(read.cyclic_prefix.prefix_power *
                                               read.cyclic_prefix.suffix_power))
                : 0.0F;
        latest_cp_snr_db =
            10.0F * std::log10(std::max(rho / std::max(1.0F - rho, 1.0e-4F),
                                        minimum_power));
    }
    demod_state.store(static_cast<int>(WorkerState::processing));
    return DemodInputFlow::ready;
}

} // namespace airspy_tv::dvbt
