// Demod output, FEC gate, and telemetry helpers.

float StreamDecoder::Impl::demod_fec_floor(
    const Constellation constellation) noexcept {
    switch (constellation) {
    case Constellation::qpsk:
        return 5.0F;
    case Constellation::qam16:
        return 10.0F;
    case Constellation::qam64:
        return 14.0F;
    }
    return 14.0F;
}

void StreamDecoder::Impl::demod_process_batch(
    DemodRuntimeState &state, std::vector<PostprocessedSymbol> batch) {
    for (auto &symbol : batch) {
        state.mer_sum += symbol.mer_db;
        state.preprocess_time_sum += symbol.preprocess_time_ms;
        state.demap_time_sum += symbol.demap_time_ms;
        state.deinterleave_time_sum += symbol.deinterleave_time_ms;
        state.depuncture_time_sum += symbol.depuncture_time_ms;
        EqualizedCallback equalized_sink;
        {
            const std::scoped_lock guard(mutex);
            equalized_sink = equalized_callback;
        }
        if (equalized_sink) {
            equalized_sink(symbol.carriers, symbol.reliabilities,
                           symbol.symbol_index);
        }
        if (state.decoder_parameters &&
            (state.analysis_symbol_count++ % analysis_interval_symbols) == 0U) {
            const float carrier_offset_hz =
                frontend.tracked_cfo_phase * sync.resampled_rate /
                (2.0F * std::numbers::pi_v<float>)+static_cast<float>(
                    frontend.carrier_offset) *
                sync.resampled_rate / static_cast<float>(frontend.fft_size);
            analysis_publisher.publish(
                symbol.carriers, symbol.mer_db, state.latest_cp_snr_db,
                state.latest_deepest_notch_db, carrier_offset_hz, frontend.mode,
                frontend.guard, state.decoder_parameters->constellation);
        }
        state.gate_buffer.push_back(std::move(symbol));
    }
    if (!state.decoder_parameters) {
        // Symbols still arrive before TPS lock. Bound retained analysis data.
        while (state.gate_buffer.size() > gate_window_symbols * 4) {
            state.gate_buffer.pop_front();
        }
        return;
    }

    const float floor =
        demod_fec_floor(state.decoder_parameters->constellation) - 2.0F;
    while (state.gate_buffer.size() >= gate_window_symbols) {
        double window_mer = 0.0;
        for (std::size_t i = 0; i < gate_window_symbols; ++i) {
            window_mer += state.gate_buffer[i].mer_db;
        }
        window_mer /= static_cast<double>(gate_window_symbols);
        const bool hopeless = static_cast<float>(window_mer) < floor;
        state.hopeless_window_count =
            hopeless ? state.hopeless_window_count + 1 : 0;
        if (airspy_tv::is_debug_enabled() && hopeless) {
            std::fprintf(
                stderr,
                "[evt] hopeless mer=%.2f floor=%.2f "
                "count=%llu sym=%llu\n",
                static_cast<float>(window_mer), floor,
                static_cast<unsigned long long>(state.hopeless_window_count),
                static_cast<unsigned long long>(state.symbol_count));
        }
        if (hopeless && !state.in_hopeless_region) {
            state.in_hopeless_region = true;
            static_cast<void>(enqueue_fec({.kind = FecItem::Kind::end,
                                           .generation = state.demod_generation,
                                           .parameters = {},
                                           .mother_metrics = {},
                                           .symbol_index = 0}));
        } else if (!hopeless && state.in_hopeless_region) {
            state.in_hopeless_region = false;
            static_cast<void>(
                enqueue_fec({.kind = FecItem::Kind::begin,
                             .generation = state.demod_generation,
                             .parameters = *state.decoder_parameters,
                             .mother_metrics = {},
                             .symbol_index = 0}));
        }
        if (!hopeless) {
            for (std::size_t i = 0; i < gate_window_symbols; ++i) {
                static_cast<void>(enqueue_fec(
                    {.kind = FecItem::Kind::symbol,
                     .generation = state.demod_generation,
                     .parameters = {},
                     .mother_metrics =
                         std::move(state.gate_buffer[i].mother_metrics),
                     .symbol_index = state.gate_buffer[i].symbol_index}));
            }
        }
        state.gate_buffer.erase(
            state.gate_buffer.begin(),
            state.gate_buffer.begin() +
                static_cast<std::ptrdiff_t>(gate_window_symbols));
    }
}

bool StreamDecoder::Impl::demod_start_decoder(DemodRuntimeState &state) {
    if (!state.decoder_parameters || state.postprocessor != nullptr) {
        return true;
    }
    if (!enqueue_fec({.kind = FecItem::Kind::begin,
                      .generation = state.demod_generation,
                      .parameters = *state.decoder_parameters,
                      .mother_metrics = {},
                      .symbol_index = 0})) {
        return false;
    }
    if (!symbol_postprocessor ||
        !symbol_postprocessor->compatible(
            state.workers.symbol, frontend.mode,
            state.decoder_parameters->constellation,
            state.decoder_parameters->code_rate, state.symbol_queue_capacity)) {
        symbol_postprocessor = std::make_unique<SymbolPostprocessorPool>(
            state.workers.symbol, frontend.mode,
            state.decoder_parameters->constellation,
            state.decoder_parameters->code_rate, state.symbol_queue_capacity);
    } else {
        // A cancelled stream can leave completed symbols behind.
        static_cast<void>(symbol_postprocessor->flush());
    }
    state.postprocessor_pending_symbols = 0;
    state.postprocessor = symbol_postprocessor.get();
    return true;
}

DemodWindowMetrics
StreamDecoder::Impl::demod_update_timing_window(DemodRuntimeState &state) {
    auto &period = state.period;
    auto &window_started_at = state.window_started_at;
    auto &window_symbol_count = state.window_symbol_count;
    auto &timing_count = state.timing_count;
    auto &timing_acc = state.timing_acc;
    auto &fft_size = state.fft_size;
    auto &applied_cir_offset = state.applied_cir_offset;
    auto &window_cir_offset_sum = state.window_cir_offset_sum;
    auto &smoothed_sample_clock_ppm = state.smoothed_sample_clock_ppm;
    auto &last_windowed_timing = state.last_windowed_timing;
    auto &last_windowed_cir_avg = state.last_windowed_cir_avg;
    constexpr std::size_t tau_history_n = DemodRuntimeState::tau_history_n;
    constexpr std::size_t tau_history_min = DemodRuntimeState::tau_history_min;
    auto &tau_history = state.tau_history;
    auto &tau_sample_history = state.tau_sample_history;
    auto &tau_interval_correction_history =
        state.tau_interval_correction_history;
    auto &tau_history_head = state.tau_history_head;
    auto &tau_history_count = state.tau_history_count;
    auto &latest_raw_timing = state.latest_raw_timing;
    auto &cir_confidence = state.cir_confidence;

    const float window_wall = duration_ms(window_started_at);
    const std::uint64_t window_symbols = window_symbol_count;
    const double window_sample_count =
        static_cast<double>(window_symbols) * static_cast<double>(period);
    const float input_seconds = sync.resampled_rate > 0.0F
                                    ? static_cast<float>(window_symbols) *
                                          static_cast<float>(period) /
                                          sync.resampled_rate
                                    : 0.0F;
    const float timing_offset =
        timing_count == 0
            ? 0.0F
            : static_cast<float>(timing_acc /
                                 static_cast<double>(timing_count)) *
                  static_cast<float>(fft_size) /
                  (2.0F * std::numbers::pi_v<float>);
    const double window_cir_avg =
        window_symbols == 0
            ? static_cast<double>(applied_cir_offset)
            : window_cir_offset_sum / static_cast<double>(window_symbols);
    double observed_drift = 0.0;
    const std::uint64_t output_begin_sample =
        state.window_output_begin_sample;
    const std::uint64_t output_end_sample = state.next_symbol_start;
    const std::uint64_t timing_sample_position =
        output_begin_sample + (output_end_sample - output_begin_sample) / 2;
    double smoothed_timing_drift =
        smoothed_sample_clock_ppm * window_sample_count / 1.0e6;
    if (timing_count != 0) {
        // Closed-loop sample-clock tracking: the windowed mean
        // tau is dominated by the channel's mean group delay
        // (multipath), so a P-loop on the absolute value would
        // chase the channel (a constant ~+35 samples at the 581
        // capture). Track only the slow drift between consecutive
        // windows — the sample-clock offset — with a long time
        // constant, and apply it through the frontend resampler. The CIR
        // window slides shift tau by exactly the slide, so both
        // references are expressed relative to the window's
        // average position, keeping the drift estimate blind to
        // the adaptive placement. (A window slide of d samples
        // moves the measured timing offset by -d, so the offset
        // is rebased onto the window's average position by
        // adding the slide back.)
        observed_drift = (static_cast<double>(timing_offset) + window_cir_avg) -
                         (last_windowed_timing + last_windowed_cir_avg);
        last_windowed_timing = static_cast<double>(timing_offset);
        last_windowed_cir_avg = window_cir_avg;
        // Store timing in the CIR-rebased physical coordinate. The median
        // first-difference estimator below sees residual sample-clock drift;
        // the applied resampler correction is added back per interval to
        // recover the source SRO without an integer-window feedback path.
        tau_history[tau_history_head] =
            static_cast<double>(timing_offset) + window_cir_avg;
        tau_sample_history[tau_history_head] = timing_sample_position;
        double interval_correction = 0.0;
        {
            const std::scoped_lock lock(mutex);
            if (state.last_timing_sample_position.has_value()) {
                interval_correction =
                    resampler_timeline
                        .average_correction(*state.last_timing_sample_position,
                                            timing_sample_position)
                        .value_or(0.0);
            }
            resampler_timeline.discard_before(timing_sample_position);
        }
        tau_interval_correction_history[tau_history_head] =
            interval_correction;
        state.last_timing_sample_position = timing_sample_position;
        tau_history_head = (tau_history_head + 1) % tau_history_n;
        if (tau_history_count < tau_history_n) {
            ++tau_history_count;
        }
        if (tau_history_count >= tau_history_min) {
            // Robust drift estimate: the least-squares slope is
            // wrecked by a single outlier window. A multipath
            // group-delay jump can flip the per-pair pilot phase
            // difference past +/-pi and shift the measured tau by
            // a hundred samples in one window (observed on 545:
            // tau=171.11 -> -163.89 in one stats window, after
            // which the slope estimate dropped to ~0 and the loop
            // walked the window off grid). Take the median of the
            // consecutive first differences instead: the true
            // sample-clock drift is slow (0.36 samples/window on
            // 545) and the per-window noise is symmetric, so a
            // handful of outlier windows cannot move the median
            // while the median still tracks the drift.
            const std::size_t diffs_count = tau_history_count - 1;
            std::array<double, tau_history_n - 1> diffs{};
            for (std::size_t i = 0; i < diffs_count; ++i) {
                const std::size_t idx0 =
                    (tau_history_head + tau_history_n - tau_history_count + i) %
                    tau_history_n;
                const std::size_t idx1 = (idx0 + 1) % tau_history_n;
                const double sample_span = static_cast<double>(
                    tau_sample_history[idx1] - tau_sample_history[idx0]);
                const double residual_sro =
                    sample_span > 0.0
                        ? (tau_history[idx1] - tau_history[idx0]) * 1.0e6 /
                              sample_span
                        : 0.0;
                // Add back the correction that actually produced the output
                // samples between these timing measurements. This remains
                // correct when queue occupancy changes the wall-clock delay.
                diffs[i] = residual_sro +
                           tau_interval_correction_history[idx1];
            }
            std::sort(diffs.begin(),
                      diffs.begin() + static_cast<std::ptrdiff_t>(diffs_count));
            const double source_sro_estimate_ppm =
                diffs_count % 2 != 0 ? diffs[diffs_count / 2]
                                     : 0.5 * (diffs[diffs_count / 2 - 1] +
                                              diffs[diffs_count / 2]);
            smoothed_sample_clock_ppm =
                0.1 * source_sro_estimate_ppm + 0.9 * smoothed_sample_clock_ppm;
        }
        const double drift_limit_ppm =
            4.0 * 1.0e6 /
            (static_cast<double>(stats_window_symbols) *
             static_cast<double>(period));
        smoothed_sample_clock_ppm = std::clamp(
            smoothed_sample_clock_ppm, -drift_limit_ppm, drift_limit_ppm);
        smoothed_timing_drift =
            smoothed_sample_clock_ppm * window_sample_count / 1.0e6;
        const double confidence =
            window_symbols == 0 ? 0.0
                                : static_cast<double>(timing_count) /
                                      static_cast<double>(window_symbols);
        if (tau_history_count >= tau_history_min && confidence >= 0.75) {
            const std::scoped_lock lock(mutex);
            const std::uint64_t command_output_sample = ring_read_pos;
            const auto mapped =
                resampler_timeline.input_at_output(command_output_sample);
            if (mapped.has_value()) {
                const std::uint64_t fixed_delay = sro_fixed_delay_samples(
                    mapped->input_rate_hz, current_bandwidth);
                scheduled_sro_commands.push_back({
                    .generation = state.demod_generation,
                    .source_epoch = mapped->stream_epoch,
                    .command_output_sample = command_output_sample,
                    .command_input_sample = mapped->input_sample,
                    .effective_input_sample = mapped->input_sample + fixed_delay,
                    .fixed_delay_samples = fixed_delay,
                    .target_ppm = smoothed_sample_clock_ppm,
                });
                latest.sro_command_output_sample = command_output_sample;
                latest.sro_command_input_sample = mapped->input_sample;
                latest.sro_effective_input_sample =
                    mapped->input_sample + fixed_delay;
                latest.sro_fixed_delay_samples = fixed_delay;
                latest.sro_input_sample_rate_hz = mapped->input_rate_hz;
                latest.sro_pending_commands = scheduled_sro_commands.size();
                sro_resampler_command_ppm.store(smoothed_sample_clock_ppm,
                                                std::memory_order_relaxed);
                sro_resampler_ready.store(true, std::memory_order_relaxed);
            }
        }
        if (airspy_tv::is_debug_enabled() && timing_count != 0 &&
            window_symbols >= stats_window_symbols) {
            std::fprintf(
                stderr,
                "[evt] tloop raw=%.2f tau=%.2f "
                "phys=%.2f drift=%.3f smooth=%.3f sro=%+.4fppm "
                "resamp=%+.4f/%+.4fppm conf=%.3f "
                "cir=%.2f/%.3f ready=%d\n",
                latest_raw_timing, static_cast<double>(timing_offset),
                static_cast<double>(timing_offset) + window_cir_avg,
                observed_drift, smoothed_timing_drift,
                smoothed_sample_clock_ppm,
                sro_resampler_command_ppm.load(std::memory_order_relaxed),
                sro_resampler_applied_ppm.load(std::memory_order_relaxed),
                static_cast<double>(timing_count) /
                    static_cast<double>(window_symbols),
                window_cir_avg, cir_confidence,
                tau_history_count >= tau_history_min);
        }
    }
    return {.wall_time_ms = window_wall,
            .symbol_count = window_symbols,
            .sample_count = window_sample_count,
            .input_seconds = input_seconds,
            .timing_offset = timing_offset,
            .cir_offset = window_cir_avg,
            .observed_drift = observed_drift,
            .smoothed_timing_drift = smoothed_timing_drift,
            .output_begin_sample = output_begin_sample,
            .output_midpoint_sample = timing_sample_position,
            .output_end_sample = output_end_sample};
}

void StreamDecoder::Impl::demod_publish_stats_window(DemodRuntimeState &state) {
    auto &period = state.period;
    auto &timing_count = state.timing_count;
    auto &fft_size = state.fft_size;
    constexpr std::size_t tau_history_min = DemodRuntimeState::tau_history_min;
    auto &tau_history_count = state.tau_history_count;
    auto &smoothed_sample_clock_ppm = state.smoothed_sample_clock_ppm;
    auto &latest_raw_timing = state.latest_raw_timing;
    auto &cir_confidence = state.cir_confidence;
    auto &guard_size = state.guard_size;
    auto &timing_raw_count = state.timing_raw_count;
    auto &timing_rejected_count = state.timing_rejected_count;
    auto &fade_indicator = state.fade_indicator;
    auto &mer_sum = state.mer_sum;
    auto &symbol_count = state.symbol_count;
    auto &preprocess_time_sum = state.preprocess_time_sum;
    auto &demap_time_sum = state.demap_time_sum;
    auto &deinterleave_time_sum = state.deinterleave_time_sum;
    auto &depuncture_time_sum = state.depuncture_time_sum;
    auto &acquisition_time_ms = state.acquisition_time_ms;
    auto &workers = state.workers;
    auto &last_reanchor_carried = state.last_reanchor_carried;
    auto &in_hopeless_region = state.in_hopeless_region;

    const DemodWindowMetrics metrics = demod_update_timing_window(state);
    const float window_wall = metrics.wall_time_ms;
    const std::uint64_t window_symbols = metrics.symbol_count;
    const float input_seconds = metrics.input_seconds;
    const float timing_offset = metrics.timing_offset;
    const double window_cir_avg = metrics.cir_offset;
    const double observed_drift = metrics.observed_drift;
    const double smoothed_timing_drift = metrics.smoothed_timing_drift;
    const std::scoped_lock lock(mutex);
    latest.ofdm_locked = true;
    latest.fft_size = static_cast<std::uint32_t>(fft_size);
    latest.guard_size = static_cast<std::uint32_t>(guard_size);
    latest.tps_locked = frontend.tps_snapshot.currently_valid;
    latest.tps_ever_locked = frontend.tps_snapshot.ever_locked;
    latest.tps_constellation = frontend.tps_snapshot.parameters.constellation;
    latest.tps_code_rate =
        frontend.tps_snapshot.parameters.high_priority_code_rate;
    latest.tps_guard_interval = frontend.tps_snapshot.parameters.guard_interval;
    latest.tps_mode = frontend.tps_snapshot.parameters.mode;
    latest.tps_hierarchy = frontend.tps_snapshot.parameters.hierarchy;
    latest.carrier_bin_offset = frontend.carrier_offset;
    latest.tracked_carrier_offset_hz = frontend.tracked_cfo_phase *
                                       sync.resampled_rate /
                                       (2.0F * std::numbers::pi_v<float>);
    latest.acquisition_start =
        static_cast<std::size_t>(sync.start_pos % period);
    latest.raw_timing_offset_samples = static_cast<float>(latest_raw_timing);
    latest.timing_offset_samples = timing_offset;
    latest.physical_timing_offset_samples =
        timing_count == 0
            ? 0.0F
            : static_cast<float>(static_cast<double>(timing_offset) +
                                 window_cir_avg);
    latest.observed_timing_drift_samples = static_cast<float>(observed_drift);
    latest.smoothed_timing_drift_samples =
        static_cast<float>(smoothed_timing_drift);
    latest.sample_clock_offset_ppm =
        static_cast<float>(smoothed_sample_clock_ppm);
    latest.sro_resampler_ready =
        sro_resampler_ready.load(std::memory_order_relaxed);
    latest.sro_resampler_command_ppm = static_cast<float>(
        sro_resampler_command_ppm.load(std::memory_order_relaxed));
    latest.sro_resampler_applied_ppm = static_cast<float>(
        sro_resampler_applied_ppm.load(std::memory_order_relaxed));
    latest.cir_offset_samples = static_cast<float>(window_cir_avg);
    latest.timing_confidence =
        window_symbols == 0 ? 0.0F
                            : std::clamp(static_cast<float>(timing_count) /
                                             static_cast<float>(window_symbols),
                                         0.0F, 1.0F);
    latest.cir_confidence = static_cast<float>(cir_confidence);
    latest.timing_measurements = timing_raw_count;
    latest.timing_accepted_measurements = timing_count;
    latest.timing_rejected_measurements = timing_rejected_count;
    latest.timing_drift_ready = tau_history_count >= tau_history_min;
    latest.fade_indicator = fade_indicator;
    latest.mer_db =
        mer_sum == 0.0 && window_symbols == 0
            ? 0.0F
            : static_cast<float>(
                  mer_sum / static_cast<double>(
                                std::max<std::uint64_t>(window_symbols, 1)));
    latest.residual_carrier_offset_hz =
        frontend.residual_phase_ema * sync.resampled_rate /
        (2.0F * std::numbers::pi_v<float> * static_cast<float>(period));
    latest.pilot_phase_discontinuities = frontend.phase_discontinuities;
    latest.ofdm_symbols = symbol_count;
    latest.processing_realtime_ratio =
        input_seconds > 0.0F ? (window_wall / 1000.0F) / input_seconds : 0.0F;
    latest.demod_busy_fraction =
        window_wall > 0.0F
            ? static_cast<float>(demod_busy_time_sum_ms /
                                 static_cast<double>(window_wall))
            : 0.0F;
    latest.demod_window_wall_time_ms = window_wall;
    latest.demod_busy_time_ms = static_cast<float>(demod_busy_time_sum_ms);
    latest.demod_ring_wait_time_ms =
        static_cast<float>(state.ring_wait_time_sum_ms);
    latest.demod_ring_copy_time_ms =
        static_cast<float>(state.ring_copy_time_sum_ms);
    latest.demod_fft_cfo_time_ms =
        static_cast<float>(state.fft_cfo_time_sum_ms);
    latest.demod_nco_rotate_time_ms =
        static_cast<float>(state.nco_rotate_time_sum_ms);
    latest.demod_fft_execute_time_ms =
        static_cast<float>(state.fft_execute_time_sum_ms);
    latest.demod_cfo_track_time_ms =
        static_cast<float>(state.cfo_track_time_sum_ms);
    latest.demod_fft_cfo_other_time_ms = static_cast<float>(std::max(
        0.0, state.fft_cfo_time_sum_ms - state.nco_rotate_time_sum_ms -
                 state.fft_execute_time_sum_ms - state.cfo_track_time_sum_ms));
    latest.demod_pilot_lock_time_ms =
        static_cast<float>(state.pilot_lock_time_sum_ms);
    latest.demod_reacquisition_time_ms =
        static_cast<float>(state.reacquisition_time_sum_ms);
    latest.demod_channel_estimate_time_ms =
        static_cast<float>(state.channel_estimate_time_sum_ms);
    latest.demod_channel_pilot_time_ms =
        static_cast<float>(state.channel_pilot_time_sum_ms);
    latest.demod_channel_notch_time_ms =
        static_cast<float>(state.channel_notch_time_sum_ms);
    latest.demod_channel_timing_time_ms =
        static_cast<float>(state.channel_timing_time_sum_ms);
    latest.demod_channel_cir_time_ms =
        static_cast<float>(state.channel_cir_time_sum_ms);
    latest.demod_channel_interpolate_time_ms =
        static_cast<float>(state.channel_interpolate_time_sum_ms);
    latest.demod_channel_tps_extract_time_ms =
        static_cast<float>(state.channel_tps_extract_time_sum_ms);
    const double accounted_channel_time_ms =
        state.channel_pilot_time_sum_ms + state.channel_notch_time_sum_ms +
        state.channel_timing_time_sum_ms + state.channel_cir_time_sum_ms +
        state.channel_interpolate_time_sum_ms +
        state.channel_tps_extract_time_sum_ms;
    latest.demod_channel_other_time_ms = static_cast<float>(std::max(
        0.0, state.channel_estimate_time_sum_ms - accounted_channel_time_ms));
    latest.demod_tps_time_ms = static_cast<float>(state.tps_time_sum_ms);
    latest.demod_payload_extract_time_ms =
        static_cast<float>(state.payload_extract_time_sum_ms);
    latest.demod_symbol_submit_time_ms =
        static_cast<float>(state.symbol_submit_time_sum_ms);
    latest.demod_postprocess_wait_time_ms =
        static_cast<float>(state.postprocess_wait_time_sum_ms);
    latest.demod_output_time_ms = static_cast<float>(state.output_time_sum_ms);
    const double accounted_demod_time_ms =
        state.ring_copy_time_sum_ms + state.fft_cfo_time_sum_ms +
        state.pilot_lock_time_sum_ms + state.reacquisition_time_sum_ms +
        state.channel_estimate_time_sum_ms + state.tps_time_sum_ms +
        state.payload_extract_time_sum_ms + state.symbol_submit_time_sum_ms +
        state.postprocess_wait_time_sum_ms + state.output_time_sum_ms;
    latest.demod_other_time_ms = static_cast<float>(
        std::max(0.0, demod_busy_time_sum_ms - accounted_demod_time_ms));
    demod_busy_time_sum_ms = 0.0;
    latest.symbol_preprocess_work_time_ms = preprocess_time_sum;
    latest.symbol_demap_work_time_ms = demap_time_sum;
    latest.symbol_deinterleave_work_time_ms = deinterleave_time_sum;
    latest.symbol_depuncture_work_time_ms = depuncture_time_sum;
    latest.last_acquisition_time_ms = acquisition_time_ms;
    latest.symbol_workers = workers.symbol;
    latest.resample_workers = workers.resample;
    latest.state_carried = last_reanchor_carried;
    latest.fec_skipped = in_hopeless_region;
    ++latest.processed_chunks;
    acquisition_time_ms = 0.0F;
}

void StreamDecoder::Impl::demod_reset_stats_window(DemodRuntimeState &state) {
    state.window_started_at = std::chrono::steady_clock::now();
    state.window_symbol_count = 0;
    state.window_output_begin_sample = 0;
    state.window_cir_offset_sum = 0.0;
    state.mer_sum = 0.0;
    state.preprocess_time_sum = 0.0F;
    state.demap_time_sum = 0.0F;
    state.deinterleave_time_sum = 0.0F;
    state.depuncture_time_sum = 0.0F;
    state.ring_wait_time_sum_ms = 0.0;
    state.ring_copy_time_sum_ms = 0.0;
    state.fft_cfo_time_sum_ms = 0.0;
    state.nco_rotate_time_sum_ms = 0.0;
    state.fft_execute_time_sum_ms = 0.0;
    state.cfo_track_time_sum_ms = 0.0;
    state.pilot_lock_time_sum_ms = 0.0;
    state.reacquisition_time_sum_ms = 0.0;
    state.channel_estimate_time_sum_ms = 0.0;
    state.channel_pilot_time_sum_ms = 0.0;
    state.channel_notch_time_sum_ms = 0.0;
    state.channel_timing_time_sum_ms = 0.0;
    state.channel_cir_time_sum_ms = 0.0;
    state.channel_interpolate_time_sum_ms = 0.0;
    state.channel_tps_extract_time_sum_ms = 0.0;
    state.tps_time_sum_ms = 0.0;
    state.payload_extract_time_sum_ms = 0.0;
    state.symbol_submit_time_sum_ms = 0.0;
    state.postprocess_wait_time_sum_ms = 0.0;
    state.output_time_sum_ms = 0.0;
    state.timing_acc = 0.0;
    state.timing_count = 0;
    state.latest_raw_timing = 0.0;
    state.timing_raw_count = 0;
    state.timing_rejected_count = 0;
}

void StreamDecoder::Impl::demod_advance_symbol(DemodRuntimeState &state) {
    state.next_symbol_start += state.period;
    state.nco_phase =
        std::remainder(state.nco_phase + frontend.tracked_cfo_phase *
                                             static_cast<float>(state.period),
                       2.0F * std::numbers::pi_v<float>);
}

bool StreamDecoder::Impl::demod_dispatch_payload(
    DemodRuntimeState &state, const PilotLock &lock,
    const std::vector<std::complex<float>> &channel) {
    auto &payload_indices = state.payload_indices;
    auto &fft_out = state.fft_out;
    auto &maximum = state.maximum;
    auto &postprocessor = state.postprocessor;
    auto &postprocessor_pending_symbols =
        state.postprocessor_pending_symbols;
    auto &pending_symbols = state.pending_symbols;

    const auto submit_postprocessor =
        [this, &state, &postprocessor_pending_symbols](
            std::vector<std::complex<float>> submitted_payload,
            std::vector<float> submitted_equalizer_power,
            const std::size_t submitted_symbol_index) {
            auto stage_started_at = std::chrono::steady_clock::now();
            state.postprocessor->submit(std::move(submitted_payload),
                                        std::move(submitted_equalizer_power),
                                        submitted_symbol_index);
            state.symbol_submit_time_sum_ms += duration_ms(stage_started_at);
            ++postprocessor_pending_symbols;
            if (postprocessor_pending_symbols < gate_window_symbols) {
                return;
            }
            stage_started_at = std::chrono::steady_clock::now();
            auto batch = state.postprocessor->take_ordered(gate_window_symbols);
            state.postprocess_wait_time_sum_ms += duration_ms(stage_started_at);
            if (batch.size() != gate_window_symbols) {
                throw std::logic_error(
                    "symbol postprocessor returned an incomplete batch");
            }
            postprocessor_pending_symbols -= batch.size();
            stage_started_at = std::chrono::steady_clock::now();
            demod_process_batch(state, std::move(batch));
            state.output_time_sum_ms += duration_ms(stage_started_at);
        };

    const auto payload_extract_started_at = std::chrono::steady_clock::now();
    std::vector<std::complex<float>> payload;
    payload.reserve(payload_carrier_count(frontend.mode));
    std::vector<float> equalizer_power;
    equalizer_power.reserve(payload_carrier_count(frontend.mode));
    for (const std::size_t k :
         payload_indices[static_cast<std::size_t>(lock.phase)]) {
        payload.push_back(
            carrier(fft_out, k, maximum, frontend.carrier_offset) * channel[k]);
        equalizer_power.push_back(std::norm(channel[k]));
    }
    state.payload_extract_time_sum_ms +=
        duration_ms(payload_extract_started_at);
    if (payload.size() != payload_carrier_count(frontend.mode)) {
        return false;
    }
    if (postprocessor == nullptr) {
        pending_symbols.push_back(
            {.payload = std::move(payload),
             .equalizer_power = std::move(equalizer_power)});
        if (pending_symbols.size() > 272) {
            // The first TPS lock can take ~68-101 symbols on
            // marginal signal; the back-computed parity
            // indices can still decode the buffered symbols,
            // so keep a generous margin before dropping.
            pending_symbols.pop_front();
        }
    } else {
        if (!pending_symbols.empty()) {
            const std::size_t count = pending_symbols.size();
            for (std::size_t index = 0; index < count; ++index) {
                auto pending = std::move(pending_symbols.front());
                pending_symbols.pop_front();
                const std::size_t distance = count - index;
                // Back-computed from the measured pilot phase
                // (the only thing the deinterleave needs is
                // the parity), never from the TPS frame index.
                const std::size_t symbol_index =
                    (static_cast<std::size_t>(lock.phase) + 68 -
                     (distance % 68)) %
                    68;
                submit_postprocessor(std::move(pending.payload),
                                     std::move(pending.equalizer_power),
                                     symbol_index);
            }
        }
        const std::size_t symbol_index = static_cast<std::size_t>(lock.phase);
        submit_postprocessor(std::move(payload), std::move(equalizer_power),
                             symbol_index);
    }
    return true;
}

void StreamDecoder::Impl::demod_finish_stream(DemodRuntimeState &state) {
    auto &postprocessor = state.postprocessor;
    auto &gate_buffer = state.gate_buffer;
    auto &decoder_parameters = state.decoder_parameters;
    auto &in_hopeless_region = state.in_hopeless_region;
    auto &demod_generation = state.demod_generation;
    auto &window_symbol_count = state.window_symbol_count;
    auto &mer_sum = state.mer_sum;

    // --- end of stream: drain the postprocessor, the gate, and the
    //     FEC decoder, then wait for the next stream ---
    if (postprocessor != nullptr) {
        auto tail = postprocessor->flush();
        if (tail.size() != state.postprocessor_pending_symbols) {
            throw std::logic_error(
                "symbol postprocessor flush lost ordered results");
        }
        state.postprocessor_pending_symbols = 0;
        demod_process_batch(state, std::move(tail));
    }
    if (!gate_buffer.empty() && decoder_parameters) {
        double window_mer = 0.0;
        for (const auto &symbol : gate_buffer) {
            window_mer += symbol.mer_db;
        }
        window_mer /= static_cast<double>(gate_buffer.size());
        const float floor =
            demod_fec_floor(decoder_parameters->constellation) - 2.0F;
        const bool hopeless = static_cast<float>(window_mer) < floor;
        if (hopeless && !in_hopeless_region) {
            in_hopeless_region = true;
        } else if (!hopeless && in_hopeless_region) {
            in_hopeless_region = false;
            static_cast<void>(enqueue_fec({.kind = FecItem::Kind::begin,
                                           .generation = demod_generation,
                                           .parameters = *decoder_parameters,
                                           .mother_metrics = {},
                                           .symbol_index = 0}));
        }
        if (!hopeless) {
            for (auto &symbol : gate_buffer) {
                static_cast<void>(enqueue_fec(
                    {.kind = FecItem::Kind::symbol,
                     .generation = demod_generation,
                     .parameters = {},
                     .mother_metrics = std::move(symbol.mother_metrics),
                     .symbol_index = symbol.symbol_index}));
            }
        }
        gate_buffer.clear();
    }
    if (postprocessor != nullptr) {
        static_cast<void>(enqueue_fec({.kind = FecItem::Kind::stream_end,
                                       .generation = demod_generation,
                                       .parameters = {},
                                       .mother_metrics = {},
                                       .symbol_index = 0}));
        // A flushed stream that is then resumed starts a fresh
        // region so the replay's first symbols do not continue a
        // flushed trellis.
        if (decoder_parameters) {
            static_cast<void>(enqueue_fec({.kind = FecItem::Kind::begin,
                                           .generation = demod_generation,
                                           .parameters = *decoder_parameters,
                                           .mother_metrics = {},
                                           .symbol_index = 0}));
        }
    }
    // Publish a final partial-window stats snapshot.
    if (window_symbol_count != 0 || mer_sum != 0.0) {
        demod_publish_stats_window(state);
        demod_reset_stats_window(state);
    }
    {
        const std::scoped_lock lock(mutex);
        // A finite stream can end with a partial next symbol (the
        // guard prefix or a truncated FFT window). It is not
        // decodable input and must not keep wait_until_idle from
        // observing a drained stream after the valid tail flush.
        ring_read_pos = ring_write_pos;
        demod_busy = false;
        acquisition_pending = false;
    }
    idle.notify_all();
}
