// Demod session, acquisition, and ring-coordination helpers.

void StreamDecoder::Impl::demod_cold_seed(DemodRuntimeState &state) {
    frontend.tracked_cfo_phase =
        std::arg(sync.phase) / static_cast<float>(state.fft_size);
    frontend.residual_phase_ema = 0.0F;
    frontend.carrier_offset = std::numeric_limits<int>::max();
    frontend.previous_continual.clear();
    frontend.previous_phase = -1;
    frontend.tps_decoder.reset();
    frontend.tps_snapshot = {};
    frontend.just_seeded = true;
}

void StreamDecoder::Impl::demod_build_grid(DemodRuntimeState &state) {
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
    state.plan = FftwfPlan::dft_1d(
        static_cast<int>(state.fft_size),
        reinterpret_cast<fftwf_complex *>(state.fft_in.data()),
        reinterpret_cast<fftwf_complex *>(state.fft_out.data()), FFTW_FORWARD,
        FFTW_ESTIMATE);

    // One CIR tap per scattered-pilot slot. The response spans Tu/12.
    frontend.cir_n = state.fft_size == 8192 ? 1024 : 256;
    frontend.cir_grid.assign(frontend.cir_n, std::complex<float>{});
    frontend.cir_response.assign(frontend.cir_n, std::complex<float>{});
    frontend.cir_plan = FftwfPlan::dft_1d(
        static_cast<int>(frontend.cir_n),
        reinterpret_cast<fftwf_complex *>(frontend.cir_grid.data()),
        reinterpret_cast<fftwf_complex *>(frontend.cir_response.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
}

void StreamDecoder::Impl::demod_reset_timing(DemodRuntimeState &state,
                                             const std::size_t tracker_fft_size,
                                             const bool reset_window_cir) {
    state.fractional_timing = 0.0;
    state.smoothed_sample_clock_ppm = 0.0;
    state.latest_cp_snr_db = 0.0F;
    state.latest_deepest_notch_db = 0.0F;
    state.accumulated_window_shift = 0.0;
    state.last_windowed_timing = 0.0;
    state.last_windowed_cir_avg = 0.0;
    state.last_timing_window_shift = 0.0;
    state.last_telemetry_window_shift = 0.0;
    state.tau_history.fill(0.0);
    state.tau_sample_history.fill(0.0);
    state.tau_history_head = 0;
    state.tau_history_count = 0;
    state.timing_elapsed_samples = 0.0;
    if (reset_window_cir) {
        state.window_cir_offset_sum = 0.0;
    }
    state.shift_rate_steps.fill(0.0);
    state.shift_rate_samples.fill(0.0);
    state.shift_rate_history_head = 0;
    state.shift_rate_history_count = 0;
    state.rolling_shift_steps = 0.0;
    state.rolling_shift_samples = 0.0;
    state.cir_confidence = 0.0;
    state.timing_tracker.reset(tracker_fft_size);
}

bool StreamDecoder::Impl::demod_handle_sync_change(DemodRuntimeState &state) {
    state.seen_sync_version = sync.version;
    state.demod_generation = latest_generation.load();
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
        reset_frontend_state();
        stable_mode.reset();
        stable_guard.reset();
        if (reset_requested) {
            demod_reset_generation = state.demod_generation;
            reset_acknowledged.notify_all();
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
        fec_queue_capacity = state.symbol_queue_capacity;
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
        state.pending_symbols.clear();
        state.gate_buffer.clear();
        state.in_hopeless_region = false;
        state.next_symbol_start = anchored_start();
        state.nco_phase = frontend.tracked_cfo_phase *
                          static_cast<float>(state.next_symbol_start);
        state.last_reanchor_carried = false;
        state.applied_cir_offset =
            static_cast<int>(std::lround(frontend.cir_offset));
        demod_reset_timing(state, state.fft_size, true);
        state.have_grid = true;
        demod_busy = true;
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
    std::uint64_t new_next = new_start;
    while (new_next < ring_read_pos) {
        new_next += state.period;
    }
    state.nco_phase = std::remainder(
        state.nco_phase +
            frontend.tracked_cfo_phase *
                static_cast<float>(
                    static_cast<std::int64_t>(new_next) -
                    static_cast<std::int64_t>(state.next_symbol_start)),
        2.0F * std::numbers::pi_v<float>);
    state.next_symbol_start = new_next;
    state.applied_cir_offset =
        static_cast<int>(std::lround(frontend.cir_offset));
    state.last_reanchor_carried = carried;
    state.stable_pending_offset = std::numeric_limits<int>::max();
    state.stable_pending_count = 0;
    demod_reset_timing(state, state.fft_size, false);
    return true;
}

float StreamDecoder::Impl::demod_run_acquisition(DemodRuntimeState &state,
                                                 const bool wait_for_data) {
    if (wait_for_data) {
        // A live decoder keeps little backlog. Wait for a useful acquisition
        // window while recovering, but bound the wait so reset remains prompt.
        const auto wait_started = std::chrono::steady_clock::now();
        for (;;) {
            bool cancel = false;
            std::uint64_t now_available = 0;
            {
                const std::scoped_lock lock(mutex);
                cancel = stopping || reset_requested || cancel_requested;
                now_available = ring_write_pos - ring_read_pos;
            }
            if (cancel || duration_ms(wait_started) > 1000.0F) {
                if (airspy_tv::is_debug_enabled() && !cancel) {
                    std::fprintf(
                        stderr, "[evt] acq wait-timeout avail=%llu\n",
                        static_cast<unsigned long long>(now_available));
                }
                return 0.0F;
            }
            if (now_available >= acquisition_samples) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }

    std::vector<std::complex<float>> window;
    ReceiverParameters acquisition_parameters;
    std::uint64_t base = 0;
    std::uint64_t acquisition_generation = 0;
    {
        const std::scoped_lock lock(mutex);
        const std::uint64_t available = ring_write_pos - ring_read_pos;
        const std::uint64_t window_size =
            std::min<std::uint64_t>(available, acquisition_samples);
        if (window_size < 2 * 10240) {
            return 0.0F;
        }
        base = ring_read_pos;
        acquisition_generation = latest_generation;
        window.reserve(static_cast<std::size_t>(window_size));
        for (std::uint64_t p = base; p < base + window_size; ++p) {
            window.push_back(ring[p % ring.size()]);
        }
        acquisition_parameters = parameters;
    }

    const auto acquisition_started_at = std::chrono::steady_clock::now();
    const OfdmAcquisition acquisition =
        acquire_ofdm(std::span(window), acquisition_parameters);
    const float acquisition_elapsed_ms = duration_ms(acquisition_started_at);
    if (acquisition.score < 0.20F) {
        const std::scoped_lock lock(mutex);
        if (acquisition_generation == latest_generation && !reset_requested) {
            state.acquisition_time_ms = acquisition_elapsed_ms;
        }
        return 0.0F;
    }

    {
        const std::scoped_lock lock(mutex);
        // Acquisition runs without the coordinator mutex. Never publish a
        // copied window after reset or generation advance.
        if (stopping || reset_requested || cancel_requested ||
            acquisition_generation != latest_generation) {
            return 0.0F;
        }
        const std::uint32_t bandwidth = current_bandwidth;
        const float resampled_rate =
            static_cast<float>(bandwidth) * (8.0F / 7.0F);
        state.acquisition_time_ms = acquisition_elapsed_ms;
        stable_mode = acquisition.mode;
        stable_guard = acquisition.guard;
        sync.valid = true;
        sync.start_pos = base + acquisition.start;
        sync.phase = acquisition.phase;
        sync.score = acquisition.score;
        sync.mode = acquisition.mode;
        sync.guard = acquisition.guard;
        sync.fft_size = acquisition.fft_size;
        sync.guard_size = acquisition.guard_size;
        sync.bandwidth = bandwidth;
        sync.resampled_rate = resampled_rate;
        latest.acquisition_score = acquisition.score;
        if (airspy_tv::is_debug_enabled()) {
            std::fprintf(stderr,
                         "[evt] acq ok score=%.3f start=%llu mode=%d g=%d\n",
                         acquisition.score,
                         static_cast<unsigned long long>(acquisition.start),
                         static_cast<int>(acquisition.mode),
                         static_cast<int>(acquisition.guard));
        }
        ++sync.version;
        static_cast<void>(demod_handle_sync_change(state));
    }
    return acquisition.score;
}

DemodFlow StreamDecoder::Impl::demod_prepare_stream(DemodRuntimeState &state) {
    auto &have_grid = state.have_grid;
    auto &seen_sync_version = state.seen_sync_version;
    auto &decoder_parameters = state.decoder_parameters;
    auto &postprocessor = state.postprocessor;

    // --- wait for first-anchor data, a reset (the front-end
    //     only invalidates the sync on retune/reset), or the
    //     resume of a flushed stream ---
    {
        std::unique_lock lock(mutex);
        // While working toward a first anchor (including the wait
        // for acquisition data), mark the demod as acquisition-
        // pending: the front-end's push-abandon must not fire in
        // the window between this wait waking and the acquisition
        // marking itself busy — a ring full of fresh data with a
        // flush arriving mid-push used to abandon the push there,
        // dropping the first block's remainder and starving small
        // files before the TPS could lock. It may fire only once
        // the demod parks in the retry back-off (never-lockable
        // streams) or the end-of-stream wait.
        if (!have_grid) {
            acquisition_pending = true;
        }
        demod_state.store(static_cast<int>(WorkerState::waiting_sync));
        ring_data.wait(lock, [this, &seen_sync_version, &have_grid] {
            return stopping || sync.version != seen_sync_version ||
                   // A finite stream may close while the first
                   // acquisition is still running. Once the grid
                   // is published, drain the already-written
                   // samples even though the ring is closed.
                   (have_grid && ring_write_pos > ring_read_pos) ||
                   (!have_grid && ring_closed) ||
                   (!have_grid &&
                    ring_write_pos - ring_read_pos >= acquisition_samples);
        });
        if (stopping) {
            demod_state.store(static_cast<int>(WorkerState::exited));
            return DemodFlow::stop;
        }
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
        {
            const std::scoped_lock lock(mutex);
            demod_busy = true;
        }
        demod_state.store(static_cast<int>(WorkerState::processing));
        const float score = demod_run_acquisition(state);
        if (!have_grid) {
            {
                const std::scoped_lock lock(mutex);
                demod_busy = false;
                acquisition_pending = false;
            }
            idle.notify_all();
        }
        if (!have_grid && ring_closed) {
            {
                const std::scoped_lock lock(mutex);
                demod_busy = false;
                // No acquisition means these samples cannot be
                // consumed by the symbol loop. Drop the closed
                // stream's dead data before parking for reset or a
                // new source, so wait_until_idle can observe a
                // genuinely drained decoder.
                ring_read_pos = ring_write_pos;
                idle.notify_all();
            }
            // The stream ended without a signal. Stay alive for
            // the next stream: a reset bumps the sync version and
            // a reopened ring refills the data.
            std::unique_lock lock(mutex);
            demod_state.store(static_cast<int>(WorkerState::waiting_sync));
            ring_data.wait(lock, [this, &seen_sync_version] {
                return stopping || sync.version != seen_sync_version ||
                       (!ring_closed && ring_write_pos > ring_read_pos);
            });
            if (stopping) {
                demod_state.store(static_cast<int>(WorkerState::exited));
                return DemodFlow::stop;
            }
        } else if (score == 0.0F) {
            {
                const std::scoped_lock lock(mutex);
                demod_busy = false;
                acquisition_pending = false;
            }
            // No signal yet (or the acquisition can never succeed
            // for this stream): back off so the retry cannot
            // busy-loop and stall the ring in front of the
            // front-end. The flush/reset notifications still wake
            // this wait. Parked here with demod_busy false, a
            // flush can abandon the front-end push (ring full)
            // and close the ring so the stream ends cleanly
            // instead of deadlocking.
            std::unique_lock lock(mutex);
            demod_state.store(
                static_cast<int>(WorkerState::waiting_acquisition));
            ring_data.wait_for(lock, std::chrono::milliseconds(100),
                               [this, &seen_sync_version] {
                                   return stopping ||
                                          sync.version != seen_sync_version;
                               });
            if (stopping) {
                demod_state.store(static_cast<int>(WorkerState::exited));
                return DemodFlow::stop;
            }
        }
        return DemodFlow::restart;
    }
    return DemodFlow::proceed;
}

DemodInputFlow
StreamDecoder::Impl::demod_read_symbol(DemodRuntimeState &state) {
    auto &next_symbol_start = state.next_symbol_start;
    auto &fft_size = state.fft_size;
    auto &seen_sync_version = state.seen_sync_version;
    auto &fft_in = state.fft_in;
    auto &symbol_count = state.symbol_count;
    auto &guard_size = state.guard_size;
    auto &latest_cp_snr_db = state.latest_cp_snr_db;
    auto &demod_busy_started_at = state.demod_busy_started_at;

    const std::uint64_t needed =
        next_symbol_start + static_cast<std::uint64_t>(fft_size);
    {
        std::unique_lock lock(mutex);
        demod_state.store(static_cast<int>(WorkerState::waiting_ring_data));
        ring_data.wait(lock, [this, needed, &seen_sync_version] {
            return stopping || sync.version != seen_sync_version ||
                   (ring_closed && ring_write_pos < needed) ||
                   ring_write_pos >= needed;
        });
        if (stopping) {
            return DemodInputFlow::stop;
        }
        if (sync.version != seen_sync_version) {
            // A reset or retune landed while this thread was
            // parked on a stale stream position: the reset
            // path rewinds the ring to zero, so the old
            // `needed` may never be reachable again. Re-run
            // the loop head, which sees the new sync version
            // and drops the grid (handle_sync_change).
            return DemodInputFlow::end;
        }
        if (ring_closed && ring_write_pos < needed) {
            return DemodInputFlow::end; // end of stream
        }
    }
    {
        const std::scoped_lock lock(mutex);
        if (sync.version != seen_sync_version) {
            return DemodInputFlow::retry;
        }
        for (std::size_t i = 0; i < fft_size; ++i) {
            const std::uint64_t position = next_symbol_start + i;
            fft_in[i] = ring[position % ring.size()];
        }
        if ((symbol_count % analysis_interval_symbols) == 0U &&
            next_symbol_start >= guard_size) {
            const std::uint64_t prefix_start = next_symbol_start - guard_size;
            if (prefix_start >= ring_read_pos) {
                std::complex<float> correlation{};
                double prefix_power = 0.0;
                double suffix_power = 0.0;
                for (std::size_t i = 0; i < guard_size; ++i) {
                    const auto prefix = ring[(prefix_start + i) % ring.size()];
                    const auto suffix =
                        ring[(prefix_start + fft_size + i) % ring.size()];
                    correlation += std::conj(prefix) * suffix;
                    prefix_power += std::norm(prefix);
                    suffix_power += std::norm(suffix);
                }
                const float rho =
                    prefix_power > 0.0 && suffix_power > 0.0
                        ? static_cast<float>(
                              std::abs(correlation) /
                              std::sqrt(prefix_power * suffix_power))
                        : 0.0F;
                latest_cp_snr_db =
                    10.0F *
                    std::log10(std::max(rho / std::max(1.0F - rho, 1.0e-4F),
                                        minimum_power));
            }
        }
    }
    demod_state.store(static_cast<int>(WorkerState::processing));
    demod_busy_started_at = std::chrono::steady_clock::now();
    {
        const std::scoped_lock lock(mutex);
        ring_read_pos = needed;
        ring_space.notify_all();
    }
    return DemodInputFlow::ready;
}
