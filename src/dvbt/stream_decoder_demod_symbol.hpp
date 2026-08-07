// Per-symbol carrier, pilot, channel, timing, and TPS helpers.

bool StreamDecoder::Impl::demod_maybe_reacquire(DemodRuntimeState &state) {
    if (events_enabled()) {
        emit_event(
            "reanchor_triggered", DecoderEventSeverity::warning,
            state.demod_generation, state.next_symbol_start, state.symbol_count,
            {{"fade_indicator", static_cast<double>(state.fade_indicator)},
             {"carrier_bin_offset",
              static_cast<std::int64_t>(frontend.carrier_offset)}});
    }
    const float reanchor_score = demod_run_acquisition(state, true);
    if (events_enabled()) {
        emit_event(
            "reanchor_result",
            reanchor_score < 0.20F ? DecoderEventSeverity::warning
                                   : DecoderEventSeverity::info,
            state.demod_generation, state.next_symbol_start, state.symbol_count,
            {{"score", static_cast<double>(reanchor_score)},
             {"stable_carrier_bin_offset",
              static_cast<std::int64_t>(frontend.stable_carrier_offset)}});
    }
    if (reanchor_score < 0.20F) {
        return false;
    }
    if (frontend.stable_carrier_offset != std::numeric_limits<int>::max()) {
        // Restore the pre-fade grid and derive phase from absolute position.
        frontend.carrier_offset = frontend.stable_carrier_offset;
        frontend.previous_phase =
            static_cast<int>((frontend.stable_phase + state.symbol_count) % 4);
        state.lock_hold = 1;
    }
    // Acquisition moved the boundary, so the in-flight symbol is stale.
    state.hopeless_window_count = 0;
    return true;
}

void StreamDecoder::Impl::demod_execute_fft_and_track_cfo(
    DemodRuntimeState &state) {
    auto &fft_size = state.fft_size;
    auto &next_symbol_start = state.next_symbol_start;
    auto &nco_phase = state.nco_phase;
    auto &fft_in = state.fft_in;
    auto &plan = state.plan;
    auto &continual_indices = state.continual_indices;
    auto &fft_out = state.fft_out;
    auto &maximum = state.maximum;
    auto &period = state.period;
    auto &fade_indicator = state.fade_indicator;

    auto fft_stage_started_at = std::chrono::steady_clock::now();
    const std::uint64_t start = next_symbol_start;
    std::complex<float> nco = std::polar(1.0F, -nco_phase);
    const std::complex<float> nco_step =
        std::polar(1.0F, -frontend.tracked_cfo_phase);
    for (std::size_t i = 0; i < fft_size; ++i) {
        fft_in[i] *= nco;
        nco *= nco_step;
        if ((i & 511U) == 511U) {
            nco *= 1.0F / std::sqrt(std::norm(nco));
        }
    }
    state.nco_rotate_time_sum_ms += duration_ms(fft_stage_started_at);

    fft_stage_started_at = std::chrono::steady_clock::now();
    plan.execute();
    state.fft_execute_time_sum_ms += duration_ms(fft_stage_started_at);

    fft_stage_started_at = std::chrono::steady_clock::now();
    std::vector<std::complex<float>> current_continual;
    current_continual.reserve(continual_indices.size());
    for (const std::size_t k : continual_indices) {
        current_continual.push_back(
            carrier(fft_out, k, maximum, frontend.carrier_offset));
    }
    float residual_phase = 0.0F;
    fade_indicator = 1.0F; // Only update the CFO loop from a
                           // contiguous symbol pair.
    // The first symbol after a re-anchor follows the previous
    // symbol by a non-period step, so its temporal correlation
    // would measure a spurious residual and overshoot; the
    // carried frequency is already converged, so skip the
    // update.
    if (frontend.previous_continual.size() == current_continual.size() &&
        start ==
            frontend.last_symbol_start + static_cast<std::uint64_t>(period)) {
        std::complex<float> temporal_correlation{};
        double power_current = 0.0;
        double power_previous = 0.0;
        for (std::size_t i = 0; i < current_continual.size(); ++i) {
            temporal_correlation += current_continual[i] *
                                    std::conj(frontend.previous_continual[i]);
            power_current += std::norm(current_continual[i]);
            power_previous += std::norm(frontend.previous_continual[i]);
        }
        residual_phase = std::arg(temporal_correlation);
        // Fade gate: the normalized temporal correlation of the
        // continual carriers collapses when the channel fades
        // (the carriers vanish into the noise). Updating the
        // loop on that uncorrelated garbage would drive the
        // converged tracked CFO away from the true offset, and
        // a deep fade long enough to integrate the error would
        // leave the demodulator rotated when the signal returns
        // (the old chunked pipeline was immune because a failed
        // chunk acquisition froze the tracking). Freeze the
        // loop until the correlation returns. The denominator
        // is the geometric mean of the two powers
        // (sqrt(P_a * P_b)), not their sum: dividing by the
        // sum halves a perfect correlation to 0.5, which made
        // a healthy ~18 dB 64-QAM signal read as marginal
        // (fi ~= 0.5) and let the pilot lock drift onto a
        // multipath alias through the "healthy" path.
        const float normalized_correlation =
            power_current > 0.0 && power_previous > 0.0
                ? static_cast<float>(std::abs(temporal_correlation)) /
                      static_cast<float>(
                          std::sqrt(power_current * power_previous))
                : 0.0F;
        fade_indicator = normalized_correlation;
        if (fade_indicator > 0.25F) {
            constexpr float loop_gain = 0.20F;
            frontend.tracked_cfo_phase +=
                loop_gain * residual_phase / static_cast<float>(period);
            frontend.residual_phase_ema =
                (0.1F * residual_phase) + (0.9F * frontend.residual_phase_ema);
        }
    }
    frontend.last_symbol_start = start;
    frontend.previous_continual = std::move(current_continual);
    if (frontend.just_seeded) {
        frontend.just_seeded = false;
    }
    // Pilot phase/carrier lock, refreshed only when the channel
    // is alive. During a fade the lock latches a noise-driven
    // phase/offset, and its narrow +/-2-bin search cannot
    // escape a bad latch when the signal returns, permanently
    // scrambling the channel estimate and payload (this was the
    // permanent lock loss the old chunked pipeline avoided by
    // skipping failed-acquisition chunks entirely). Freeze the
    // carried values instead; the pilot phase rotates mod 4 and
    // fades span whole 4-symbol cycles, so the carried phase
    // stays valid across the fade. A long freeze (a fade longer
    // than one TPS frame) periodically re-runs the WIDE lock so
    // the demodulator can re-grab the true grid the moment the
    // signal returns, instead of staying latched on stale or
    // noise-latched values.
    state.cfo_track_time_sum_ms += duration_ms(fft_stage_started_at);
}

std::optional<PilotLock>
StreamDecoder::Impl::demod_lock_pilots(DemodRuntimeState &state) {
    auto &lock_hold = state.lock_hold;
    auto &fade_indicator = state.fade_indicator;
    auto &hopeless_window_count = state.hopeless_window_count;
    auto &frozen_symbol_count = state.frozen_symbol_count;
    auto &timing_count = state.timing_count;
    auto &timing_acc = state.timing_acc;
    auto &fft_size = state.fft_size;
    auto &symbol_count = state.symbol_count;
    auto &fft_out = state.fft_out;
    auto &maximum = state.maximum;
    auto &timing_tracker = state.timing_tracker;
    auto &stable_pending_offset = state.stable_pending_offset;
    auto &stable_pending_count = state.stable_pending_count;

    PilotLock lock;
    const bool has_previous_lock =
        frontend.previous_phase >= 0 && frontend.previous_phase < 4 &&
        frontend.carrier_offset != std::numeric_limits<int>::max();
    if (lock_hold > 0 && has_previous_lock) {
        --lock_hold;
        lock = PilotLock{static_cast<int>(frontend.previous_phase),
                         frontend.carrier_offset};
    } else if (has_previous_lock &&
               (fade_indicator <= 0.25F || hopeless_window_count >= 4)) {
        if (frozen_symbol_count == 0) {
            // Distinguish a real fade (fi collapsed) from a
            // decode-stuck state (fi healthy but MER below the
            // decode floor — the carrier grid drifted off,
            // which the continual-carrier correlation cannot
            // see).
            if (fade_indicator > 0.25F) {
                const double timing_samples =
                    timing_count == 0
                        ? 0.0
                        : (timing_acc / static_cast<double>(timing_count)) *
                              static_cast<double>(fft_size) /
                              (2.0 * std::numbers::pi_v<double>);
                if (events_enabled()) {
                    emit_event(
                        "bad_lock_enter", DecoderEventSeverity::warning,
                        state.demod_generation, state.next_symbol_start,
                        symbol_count,
                        {{"fade_indicator",
                          static_cast<double>(fade_indicator)},
                         {"carrier_bin_offset",
                          static_cast<std::int64_t>(frontend.carrier_offset)},
                         {"phase_discontinuities",
                          frontend.phase_discontinuities},
                         {"timing_samples", timing_samples},
                         {"hopeless_window_count", hopeless_window_count}});
                }
            } else {
                if (events_enabled()) {
                    emit_event(
                        "fade_enter", DecoderEventSeverity::warning,
                        state.demod_generation, state.next_symbol_start,
                        symbol_count,
                        {{"fade_indicator",
                          static_cast<double>(fade_indicator)},
                         {"carrier_bin_offset",
                          static_cast<std::int64_t>(frontend.carrier_offset)}});
                }
            }
        }
        // Fade / decode-stuck: hold the frozen phase/offset
        // and re-acquire directly. A fade (or a grid drift)
        // can shift the true carrier offset (observed off=2 at
        // the 581 fade) and disturb the mod-4 scattered-pilot
        // phase, so a narrow re-lock on the frozen grid can
        // only latch a multipath alias or stay stuck on a
        // stale phase — the permanent lock loss seen on the
        // 581 tail (MER stuck at ~7.8 dB with
        // phase-discontinuities climbing). The wideband CP
        // correlation, unlike the continual-carrier
        // correlation, survives the multipath that keeps
        // fade_indicator collapsed, so re-acquisition
        // confirms the signal's return and rebuilds the whole
        // grid (offset, phase, boundary, guard) at once.
        // Retry every 68 symbols (~100 ms) until the signal
        // returns; while faded there is no valid TS to lose,
        // and a re-anchor that lands while the signal is
        // still gone just scores low and retries.
        lock = PilotLock{static_cast<int>(frontend.previous_phase),
                         frontend.carrier_offset};
        if (++frozen_symbol_count >= 68 && frozen_symbol_count % 68 == 0 &&
            demod_maybe_reacquire(state)) {
            return std::nullopt;
        }
    } else {
        // A cold seed has no prior grid to freeze. Discard a stale hold
        // request and establish / verify a real pilot lock instead.
        lock_hold = 0;
        if (frontend.carrier_offset == std::numeric_limits<int>::max()) {
            // First healthy lock after (re-)acquisition: the
            // grid is unknown, so run the full offset search
            // once to establish it.
            lock = lock_pilots(fft_out, maximum, frontend.carrier_offset);
        } else {
            // Grid already established: freeze the carrier
            // offset and re-verify only the rotating phase.
            // The offset is a fixed physical property (the LO
            // never moves; residual drift is sub-bin and owned
            // by the CFO loop), so re-searching it every
            // symbol only chases multipath-induced offset
            // ambiguity — the 545 capture shows lock_pilots
            // returning 0/±1/±2 at random from symbol to
            // symbol even at fi=0.9996, and a 4-symbol
            // confirmation window is short enough to accept
            // one of those spurious offsets, permanently
            // snapping the grid off the true carrier (the
            // observed car 0->3 collapse). The grid is only
            // re-established by a re-anchor after a real
            // fade, which restores the pre-fade stable
            // offset.
            lock = PilotLock{lock_phase_at_offset(fft_out, maximum,
                                                  frontend.carrier_offset,
                                                  timing_tracker.filtered()),
                             frontend.carrier_offset};
        }
        if (frontend.previous_phase >= 0 &&
            lock.phase != (frontend.previous_phase + 1) % 4) {
            ++frontend.phase_discontinuities;
            if (events_enabled()) {
                emit_event(
                    "pilot_phase_jump", DecoderEventSeverity::warning,
                    state.demod_generation, state.next_symbol_start,
                    symbol_count,
                    {{"previous_phase",
                      static_cast<std::int64_t>(frontend.previous_phase)},
                     {"current_phase", static_cast<std::int64_t>(lock.phase)},
                     {"fade_indicator", static_cast<double>(fade_indicator)},
                     {"carrier_bin_offset",
                      static_cast<std::int64_t>(frontend.carrier_offset)}});
            }
        }
        frontend.previous_phase = lock.phase;
        frontend.carrier_offset = lock.offset;
        // Capture the stable grid only once: the carrier grid
        // is fixed for the life of the stream (the LO never
        // moves), so the pre-fade reference the fade re-anchor
        // restores must be a healthy lock, never a
        // multipath-influenced one. A single first lock is not
        // enough — the stream can start through a marginal
        // channel, and a noise-latched alias captured here
        // would be restored by every later re-anchor (the
        // 545 capture's car 0->3: the stable reference was
        // pinned to the alias, so each fade re-anchor snapped
        // the grid to 3 and decoding never recovered). Require
        // the same offset for a few consecutive healthy
        // symbols before trusting it.
        if (frontend.stable_carrier_offset == std::numeric_limits<int>::max()) {
            if (lock.offset == stable_pending_offset) {
                if (++stable_pending_count >= 4) {
                    frontend.stable_phase = lock.phase;
                    frontend.stable_carrier_offset = lock.offset;
                }
            } else {
                stable_pending_offset = lock.offset;
                stable_pending_count = 1;
            }
        }
        const auto was_frozen = frozen_symbol_count;
        frozen_symbol_count = 0;
        if (was_frozen > 0) {
            if (events_enabled()) {
                emit_event(
                    "fade_exit", DecoderEventSeverity::info,
                    state.demod_generation, state.next_symbol_start,
                    symbol_count,
                    {{"fade_indicator", static_cast<double>(fade_indicator)},
                     {"carrier_bin_offset",
                      static_cast<std::int64_t>(frontend.carrier_offset)},
                     {"frozen_symbols", was_frozen}});
            }
        }
    }
    return lock;
}

std::vector<std::complex<float>>
StreamDecoder::Impl::demod_estimate_channel(DemodRuntimeState &state,
                                            const PilotLock &lock) {
    auto &maximum = state.maximum;
    auto &pilot_indices = state.pilot_indices;
    auto &fft_out = state.fft_out;
    auto &symbol_count = state.symbol_count;
    auto &latest_deepest_notch_db = state.latest_deepest_notch_db;
    auto &timing_raw_count = state.timing_raw_count;
    auto &latest_raw_timing = state.latest_raw_timing;
    auto &timing_tracker = state.timing_tracker;
    auto &timing_acc = state.timing_acc;
    auto &timing_count = state.timing_count;
    auto &timing_rejected_count = state.timing_rejected_count;
    auto &fft_size = state.fft_size;
    auto &window_cir_offset_sum = state.window_cir_offset_sum;
    auto &applied_cir_offset = state.applied_cir_offset;
    auto &fade_indicator = state.fade_indicator;
    auto &cir_confidence = state.cir_confidence;
    auto &guard_size = state.guard_size;
    auto &next_symbol_start = state.next_symbol_start;
    auto &nco_phase = state.nco_phase;
    auto &tps_indices = state.tps_indices;
    auto &tps_values = state.tps_values;

    auto channel_stage_started_at = std::chrono::steady_clock::now();
    std::vector<std::complex<float>> channel(maximum + 1);
    const auto &pilots = pilot_indices[static_cast<std::size_t>(lock.phase)];
    for (const std::size_t k : pilots) {
        const float sent = prbs[k] == 0U ? 4.0F / 3.0F : -4.0F / 3.0F;
        const auto received =
            carrier(fft_out, k, maximum, frontend.carrier_offset);
        channel[k] = std::norm(received) > minimum_power
                         ? std::complex<float>{sent, 0.0F} / received
                         : std::complex<float>{};
    }
    state.channel_pilot_time_sum_ms += duration_ms(channel_stage_started_at);

    channel_stage_started_at = std::chrono::steady_clock::now();
    if ((symbol_count % analysis_interval_symbols) == 0U) {
        latest_deepest_notch_db = estimate_channel_notch_db(
            channel, static_cast<std::size_t>(lock.phase), maximum);
    }
    state.channel_notch_time_sum_ms += duration_ms(channel_stage_started_at);
    // Fractional timing estimate: an FFT-window shift of tau
    // samples ramps arg(channel) linearly across carriers
    // (2*pi*k*tau/N). Estimate it from the fixed-spacing
    // scattered-pilot grid, unwrap it against the previous
    // filtered value, and reject isolated group-delay clicks
    // before feeding either the long-term timing loop or the
    // phase verifier.
    channel_stage_started_at = std::chrono::steady_clock::now();
    if (const auto measured_tau = estimate_scattered_timing_tau(
            channel, static_cast<std::size_t>(lock.phase), maximum, fft_size);
        measured_tau.has_value()) {
        latest_raw_timing = *measured_tau;
        ++timing_raw_count;
        const auto previous_filtered_tau = timing_tracker.filtered();
        const auto accepted_tau = timing_tracker.observe(*measured_tau);
        if (accepted_tau.has_value()) {
            if (previous_filtered_tau.has_value() &&
                std::abs(*accepted_tau - *previous_filtered_tau) >
                    timing_outlier_limit_samples) {
                if (events_enabled()) {
                    emit_event(
                        "timing_branch_change", DecoderEventSeverity::warning,
                        state.demod_generation, state.next_symbol_start,
                        symbol_count,
                        {{"previous_filtered_samples", *previous_filtered_tau},
                         {"accepted_samples", *accepted_tau},
                         {"raw_samples", *measured_tau}});
                }
            }
            timing_acc += *accepted_tau * (2.0 * std::numbers::pi_v<double>) /
                          static_cast<double>(fft_size);
            ++timing_count;
        } else {
            ++timing_rejected_count;
            const auto filtered_tau = timing_tracker.filtered();
            if (events_enabled()) {
                emit_event("timing_measurement_rejected",
                           DecoderEventSeverity::warning,
                           state.demod_generation, state.next_symbol_start,
                           symbol_count,
                           {{"raw_samples", *measured_tau},
                            {"filtered_samples",
                             filtered_tau.value_or(*measured_tau)}});
            }
        }
    }
    state.channel_timing_time_sum_ms += duration_ms(channel_stage_started_at);
    // This symbol was demodulated with the current CIR anchor.
    // Accumulate the exact applied position before the CIR
    // update below can move the anchor for the next symbol.
    channel_stage_started_at = std::chrono::steady_clock::now();
    window_cir_offset_sum += static_cast<double>(applied_cir_offset);
    // CIR / delay-spread estimate (scattered pilots -> IFFT ->
    // impulse response) for adaptive FFT-window placement.
    // Once per TPS frame: negligible cost. When the measured
    // delay spread is significant (>= guard/4) the window
    // slides toward the middle of the ISI-free range
    // [spread, guard], balancing the pre/post-ISI margins;
    // the offset steps by at most +/-4 samples per frame so
    // the timing loop is never perturbed by a jump.
    if (++frontend.cir_symbol_count >= 68) {
        frontend.cir_symbol_count = 0;
        if (fade_indicator > 0.25F && frontend.cir_plan) {
            std::fill(frontend.cir_grid.begin(), frontend.cir_grid.end(),
                      std::complex<float>{});
            const std::size_t phase = static_cast<std::size_t>(lock.phase);
            std::size_t slot = 0;
            for (std::size_t k = phase * 3; k <= maximum; k += 12) {
                if (slot >= frontend.cir_grid.size()) {
                    break;
                }
                frontend.cir_grid[slot] = channel[k];
                ++slot;
            }
            frontend.cir_plan.execute();
            const std::size_t n = frontend.cir_response.size();
            std::vector<double> energy(n, 0.0);
            double total = 0.0;
            std::size_t peak = 0;
            double peak_energy = -1.0;
            for (std::size_t i = 0; i < n; ++i) {
                energy[i] =
                    static_cast<double>(std::norm(frontend.cir_response[i]));
                total += energy[i];
                if (energy[i] > peak_energy) {
                    peak_energy = energy[i];
                    peak = i;
                }
            }
            cir_confidence = total > 0.0 ? peak_energy / total : 0.0;
            // Only trust a structured response: the peak tap
            // must hold a meaningful share of the energy, so
            // a noise-driven CIR cannot drag the window.
            if (total > 0.0 && peak_energy / total > 0.05) {
                // Contiguous main lobe: taps above 1% of the
                // peak, wrapping the IFFT window (taps that
                // arrive before the FFT window fold to its
                // tail). Underestimating the spread is safe:
                // the offset only moves the window inside
                // the ISI-free range [d_max, G + d_min].
                const double lobe_threshold = peak_energy * 0.01;
                std::size_t lo = peak;
                std::size_t hi = peak;
                while (energy[(lo + n - 1) % n] >= lobe_threshold &&
                       (lo + n - 1) % n != hi) {
                    lo = (lo + n - 1) % n;
                }
                while (energy[(hi + 1) % n] >= lobe_threshold &&
                       (hi + 1) % n != lo) {
                    hi = (hi + 1) % n;
                }
                const std::size_t lobe_width = (hi + n - lo) % n + 1;
                const double scale = static_cast<double>(fft_size) /
                                     static_cast<double>(12 * frontend.cir_n);
                const double spread_samples =
                    static_cast<double>(lobe_width) * scale;
                // Slide the window to the middle of the
                // ISI-free range [spread, guard] (in offset
                // coordinates relative to the effective
                // symbol start), balancing the pre- and
                // post-ISI margins. Only act when the delay
                // spread is significant: for a single-path /
                // short-delay channel the current anchor is
                // already inside the (wide) ISI-free range,
                // and sliding the window there buys nothing
                // while perturbing the timing loop.
                const double spread_threshold =
                    0.25 * static_cast<double>(guard_size);
                const double target =
                    spread_samples > spread_threshold
                        ? std::clamp((spread_samples -
                                      static_cast<double>(guard_size)) /
                                         2.0,
                                     -static_cast<double>(guard_size), 0.0)
                        : 0.0;
                frontend.cir_offset = static_cast<float>(
                    0.9 * static_cast<double>(frontend.cir_offset) +
                    0.1 * target);
                const int applied =
                    static_cast<int>(std::lround(frontend.cir_offset));
                // Step the anchor by at most +/-4 samples per
                // frame. The timing loop is now rebased onto
                // the window's average position (so slides do
                // not read as sample-clock steps), but the
                // step bound still limits how often the CFO
                // update is skipped by the boundary crossing
                // and keeps a mis-estimated target from
                // sliding far into the ISI region in one
                // frame.
                const int delta =
                    std::clamp(applied - applied_cir_offset, -4, 4);
                if (delta != 0) {
                    if (delta > 0) {
                        next_symbol_start += static_cast<std::uint64_t>(delta);
                    } else {
                        next_symbol_start -= static_cast<std::uint64_t>(-delta);
                    }
                    applied_cir_offset += delta;
                    // The window moved mid-stream: advance
                    // the mixer phase by the skipped samples
                    // and let the contiguous-pair gate skip
                    // the CFO update for this boundary.
                    nco_phase =
                        std::remainder(nco_phase + (frontend.tracked_cfo_phase *
                                                    static_cast<float>(delta)),
                                       2.0F * std::numbers::pi_v<float>);
                }
            }
        } else {
            cir_confidence = 0.0;
        }
    }
    state.channel_cir_time_sum_ms += duration_ms(channel_stage_started_at);

    channel_stage_started_at = std::chrono::steady_clock::now();
    for (std::size_t i = 1; i < pilots.size(); ++i) {
        const std::size_t left = pilots[i - 1];
        const std::size_t right = pilots[i];
        for (std::size_t k = left; k <= right; ++k) {
            const float f =
                static_cast<float>(k - left) / static_cast<float>(right - left);
            channel[k] = channel[left] + (channel[right] - channel[left]) * f;
        }
    }
    std::fill(channel.begin(),
              channel.begin() + static_cast<std::ptrdiff_t>(pilots.front()),
              channel[pilots.front()]);
    std::fill(channel.begin() + static_cast<std::ptrdiff_t>(pilots.back()),
              channel.end(), channel[pilots.back()]);
    state.channel_interpolate_time_sum_ms +=
        duration_ms(channel_stage_started_at);

    channel_stage_started_at = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < tps_indices.size(); ++i) {
        const std::size_t k = tps_indices[i];
        tps_values[i] =
            carrier(fft_out, k, maximum, frontend.carrier_offset) * channel[k];
    }
    state.channel_tps_extract_time_sum_ms +=
        duration_ms(channel_stage_started_at);
    return channel;
}

DemodFlow StreamDecoder::Impl::demod_process_tps(DemodRuntimeState &state) {
    auto &tps_values = state.tps_values;
    auto &tps_mismatch_symbols = state.tps_mismatch_symbols;
    auto &decoder_parameters = state.decoder_parameters;
    auto &selected_parameters = state.selected_parameters;
    auto &workers = state.workers;
    auto &symbol_count = state.symbol_count;

    // TPS differential bits carry continuously across the
    // stream. Before synchronization the decoder checks each
    // sliding 68-bit window; afterward it validates only the
    // expected frame boundary and returns to the sliding search
    // after a bad frame. The FEC configuration remains fixed
    // after its first matching TPS frame; deinterleave parity
    // comes from the measured pilot phase below, not the TPS
    // frame index.
    frontend.tps_snapshot = frontend.tps_decoder.process(tps_values);
    const bool matching_tps =
        frontend.tps_snapshot.ever_locked &&
        frontend.tps_snapshot.parameters.mode == frontend.mode &&
        frontend.tps_snapshot.parameters.guard_interval == frontend.guard;
    if (frontend.tps_snapshot.ever_locked && !matching_tps) {
        if (++tps_mismatch_symbols >= 1400) {
            tps_mismatch_symbols = 0;
            if (demod_run_acquisition(state, true) >= 0.20F) {
                // The grid was republished (the TPS decoded a
                // mode/guard that disagrees with the current
                // grid): the in-flight symbol predates the new
                // grid. Discard it and restart from the
                // published boundary at the loop head.
                return DemodFlow::restart;
            }
        }
    } else {
        tps_mismatch_symbols = 0;
    }
    if (!decoder_parameters && matching_tps &&
        frontend.tps_snapshot.parameters.hierarchy == 0U) {
        if (events_enabled()) {
            emit_event("tps_lock", DecoderEventSeverity::info,
                       state.demod_generation, state.next_symbol_start,
                       symbol_count,
                       {{"transmission_mode",
                         std::string(event_mode_name(frontend.mode))},
                        {"guard_interval",
                         std::string(event_guard_name(frontend.guard))},
                        {"constellation",
                         std::string(event_constellation_name(
                             frontend.tps_snapshot.parameters.constellation))},
                        {"code_rate", std::string(event_code_rate_name(
                                          frontend.tps_snapshot.parameters
                                              .high_priority_code_rate))}});
        }
        decoder_parameters = DecoderParameters{
            frontend.mode,
            selected_parameters.constellation.value_or(
                frontend.tps_snapshot.parameters.constellation),
            selected_parameters.code_rate.value_or(
                frontend.tps_snapshot.parameters.high_priority_code_rate),
            workers.viterbi};
        if (!demod_start_decoder(state)) {
            return DemodFlow::stop;
        }
    }
    return DemodFlow::proceed;
}
