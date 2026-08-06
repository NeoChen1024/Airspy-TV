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
    auto &accumulated_window_shift = state.accumulated_window_shift;
    auto &last_telemetry_window_shift = state.last_telemetry_window_shift;
    constexpr std::size_t shift_rate_history_n =
        DemodRuntimeState::shift_rate_history_n;
    auto &shift_rate_history_count = state.shift_rate_history_count;
    auto &shift_rate_steps = state.shift_rate_steps;
    auto &shift_rate_samples = state.shift_rate_samples;
    auto &shift_rate_history_head = state.shift_rate_history_head;
    auto &rolling_shift_steps = state.rolling_shift_steps;
    auto &rolling_shift_samples = state.rolling_shift_samples;
    auto &timing_elapsed_samples = state.timing_elapsed_samples;
    auto &smoothed_sample_clock_ppm = state.smoothed_sample_clock_ppm;
    auto &last_windowed_timing = state.last_windowed_timing;
    auto &last_windowed_cir_avg = state.last_windowed_cir_avg;
    auto &last_timing_window_shift = state.last_timing_window_shift;
    constexpr std::size_t tau_history_n = DemodRuntimeState::tau_history_n;
    constexpr std::size_t tau_history_min = DemodRuntimeState::tau_history_min;
    auto &tau_history = state.tau_history;
    auto &tau_sample_history = state.tau_sample_history;
    auto &tau_history_head = state.tau_history_head;
    auto &tau_history_count = state.tau_history_count;
    auto &fractional_timing = state.fractional_timing;
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
    double corrected_drift = 0.0;
    double window_shift = 0.0;
    const double telemetry_window_shift =
        accumulated_window_shift - last_telemetry_window_shift;
    last_telemetry_window_shift = accumulated_window_shift;
    if (shift_rate_history_count == shift_rate_history_n) {
        rolling_shift_steps -= shift_rate_steps[shift_rate_history_head];
        rolling_shift_samples -= shift_rate_samples[shift_rate_history_head];
    } else {
        ++shift_rate_history_count;
    }
    shift_rate_steps[shift_rate_history_head] = telemetry_window_shift;
    shift_rate_samples[shift_rate_history_head] = window_sample_count;
    rolling_shift_steps += telemetry_window_shift;
    rolling_shift_samples += window_sample_count;
    shift_rate_history_head =
        (shift_rate_history_head + 1) % shift_rate_history_n;
    const double rolling_shift_rate_ppm =
        rolling_shift_samples > 0.0
            ? rolling_shift_steps * 1.0e6 / rolling_shift_samples
            : 0.0;
    const double timing_sample_position =
        timing_elapsed_samples + 0.5 * window_sample_count;
    timing_elapsed_samples += window_sample_count;
    double smoothed_timing_drift =
        smoothed_sample_clock_ppm * window_sample_count / 1.0e6;
    if (timing_count != 0) {
        // Closed-loop sample-clock tracking: the windowed mean
        // tau is dominated by the channel's mean group delay
        // (multipath), so a P-loop on the absolute value would
        // chase the channel (a constant ~+35 samples at the 581
        // capture). Track only the slow drift between consecutive
        // windows — the sample-clock offset — with a long time
        // constant, and accumulate it into the fractional timing
        // the symbol advance consumes. Bounded so a pathological
        // estimate cannot walk the window far off grid. The CIR
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
        window_shift = accumulated_window_shift - last_timing_window_shift;
        last_timing_window_shift = accumulated_window_shift;
        // A positive window step makes the measured pilot slope
        // move negative by only the channel-dependent response
        // fraction. Add that known response back so the loop
        // estimates physical sample-clock drift instead of
        // cancelling its own correction in the measurement.
        corrected_drift =
            observed_drift + timing_window_shift_response * window_shift;
        // The per-window drift is corrected for the loop's own
        // period steps above: a step moves the measured tau by
        // approximately one sample, so treating that step as
        // physical drift would bias the compensation —
        // the tau sawtooth (ramping 0 -> 62 samples until the
        // pilot verify collapses, one badlock per ~38000
        // symbols). Keep a physical-coordinate history and use a
        // median first difference instead: a handful of outliers
        // cannot move the estimate away from true clock drift.
        // Store the timing coordinate after undoing the known
        // response of all integer window corrections. The median
        // first-difference estimator below must see physical
        // sample-clock drift, not the loop's sawtooth response.
        tau_history[tau_history_head] =
            static_cast<double>(timing_offset) + window_cir_avg +
            timing_window_shift_response * accumulated_window_shift;
        tau_sample_history[tau_history_head] = timing_sample_position;
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
                const double sample_span =
                    tau_sample_history[idx1] - tau_sample_history[idx0];
                diffs[i] = sample_span > 0.0
                               ? (tau_history[idx1] - tau_history[idx0]) *
                                     1.0e6 / sample_span
                               : 0.0;
            }
            std::sort(diffs.begin(),
                      diffs.begin() + static_cast<std::ptrdiff_t>(diffs_count));
            const double drift_estimate_ppm =
                diffs_count % 2 != 0 ? diffs[diffs_count / 2]
                                     : 0.5 * (diffs[diffs_count / 2 - 1] +
                                              diffs[diffs_count / 2]);
            smoothed_sample_clock_ppm =
                0.1 * drift_estimate_ppm + 0.9 * smoothed_sample_clock_ppm;
        }
        const double drift_limit_ppm =
            4.0 * 1.0e6 /
            (static_cast<double>(stats_window_symbols) *
             static_cast<double>(period));
        smoothed_sample_clock_ppm = std::clamp(
            smoothed_sample_clock_ppm, -drift_limit_ppm, drift_limit_ppm);
        smoothed_timing_drift =
            smoothed_sample_clock_ppm * window_sample_count / 1.0e6;
        fractional_timing +=
            smoothed_timing_drift / timing_window_shift_response;
        fractional_timing = std::clamp(fractional_timing, -4.0, 4.0);
        if (airspy_tv::is_debug_enabled() && timing_count != 0 &&
            window_symbols >= stats_window_symbols) {
            std::fprintf(
                stderr,
                "[evt] tloop raw=%.2f tau=%.2f "
                "phys=%.2f shift=%.1f drift=%.3f "
                "smooth=%.3f sro=%+.4fppm frac=%.2f "
                "act=%+.4f/%+.4fppm conf=%.3f "
                "cir=%.2f/%.3f ready=%d\n",
                latest_raw_timing, static_cast<double>(timing_offset),
                static_cast<double>(timing_offset) + window_cir_avg +
                    timing_window_shift_response * accumulated_window_shift,
                accumulated_window_shift, corrected_drift,
                smoothed_timing_drift, smoothed_sample_clock_ppm,
                fractional_timing,
                window_sample_count > 0.0
                    ? telemetry_window_shift * 1.0e6 / window_sample_count
                    : 0.0,
                rolling_shift_rate_ppm,
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
            .corrected_drift = corrected_drift,
            .timing_shift = telemetry_window_shift,
            .rolling_shift_rate_ppm = rolling_shift_rate_ppm,
            .smoothed_timing_drift = smoothed_timing_drift};
}

void StreamDecoder::Impl::demod_publish_stats_window(DemodRuntimeState &state) {
    auto &period = state.period;
    auto &timing_count = state.timing_count;
    auto &fft_size = state.fft_size;
    auto &accumulated_window_shift = state.accumulated_window_shift;
    constexpr std::size_t tau_history_min = DemodRuntimeState::tau_history_min;
    auto &tau_history_count = state.tau_history_count;
    auto &smoothed_sample_clock_ppm = state.smoothed_sample_clock_ppm;
    auto &fractional_timing = state.fractional_timing;
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
    const double window_sample_count = metrics.sample_count;
    const float input_seconds = metrics.input_seconds;
    const float timing_offset = metrics.timing_offset;
    const double window_cir_avg = metrics.cir_offset;
    const double observed_drift = metrics.observed_drift;
    const double corrected_drift = metrics.corrected_drift;
    const double telemetry_window_shift = metrics.timing_shift;
    const double rolling_shift_rate_ppm = metrics.rolling_shift_rate_ppm;
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
            : static_cast<float>(
                  static_cast<double>(timing_offset) + window_cir_avg +
                  timing_window_shift_response * accumulated_window_shift);
    latest.observed_timing_drift_samples = static_cast<float>(observed_drift);
    latest.corrected_timing_drift_samples = static_cast<float>(corrected_drift);
    latest.smoothed_timing_drift_samples =
        static_cast<float>(smoothed_timing_drift);
    latest.sample_clock_offset_ppm =
        static_cast<float>(smoothed_sample_clock_ppm);
    latest.cumulative_timing_shift_samples =
        static_cast<float>(accumulated_window_shift);
    latest.timing_shift_rate_ppm =
        window_sample_count > 0.0
            ? static_cast<float>(telemetry_window_shift * 1.0e6 /
                                 window_sample_count)
            : 0.0F;
    latest.rolling_timing_shift_rate_ppm =
        static_cast<float>(rolling_shift_rate_ppm);
    latest.fractional_timing_samples = static_cast<float>(fractional_timing);
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
    state.window_cir_offset_sum = 0.0;
    state.mer_sum = 0.0;
    state.preprocess_time_sum = 0.0F;
    state.demap_time_sum = 0.0F;
    state.deinterleave_time_sum = 0.0F;
    state.depuncture_time_sum = 0.0F;
    state.timing_acc = 0.0;
    state.timing_count = 0;
    state.latest_raw_timing = 0.0;
    state.timing_raw_count = 0;
    state.timing_rejected_count = 0;
}

void StreamDecoder::Impl::demod_advance_symbol(DemodRuntimeState &state) {
    // Consume the closed-loop fractional timing correction as integer steps.
    state.next_symbol_start += state.period;
    if (state.fractional_timing >= 0.5) {
        ++state.next_symbol_start;
        state.accumulated_window_shift += 1.0;
        state.fractional_timing -= 1.0;
    } else if (state.fractional_timing <= -0.5) {
        --state.next_symbol_start;
        state.accumulated_window_shift -= 1.0;
        state.fractional_timing += 1.0;
    }
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
    auto &symbol_count = state.symbol_count;
    auto &window_symbol_count = state.window_symbol_count;
    auto &demod_busy_started_at = state.demod_busy_started_at;
    auto &postprocessor = state.postprocessor;
    auto &pending_symbols = state.pending_symbols;

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
    if (payload.size() != payload_carrier_count(frontend.mode)) {
        demod_advance_symbol(state);
        ++symbol_count;
        ++window_symbol_count;
        if (window_symbol_count >= stats_window_symbols) {
            demod_publish_stats_window(state);
            demod_reset_stats_window(state);
        }
        demod_busy_time_sum_ms += duration_ms(demod_busy_started_at);
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
                postprocessor->submit(std::move(pending.payload),
                                      std::move(pending.equalizer_power),
                                      symbol_index);
            }
        }
        const std::size_t symbol_index = static_cast<std::size_t>(lock.phase);
        postprocessor->submit(std::move(payload), std::move(equalizer_power),
                              symbol_index);
        demod_process_batch(state, postprocessor->take_ready());
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
        demod_process_batch(state, postprocessor->flush());
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
