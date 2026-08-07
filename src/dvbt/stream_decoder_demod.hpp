// Internal StreamDecoder::Impl worker definition.

// ------------------------------------------------------------------ //
// Demod thread: contiguous symbol extraction, CFO/channel/TPS tracking,
// symbol postprocessing, and the windowed MER gate. One continuous symbol
// stream per sync; re-anchors and resets are handled at the loop heads.
// ------------------------------------------------------------------ //
void StreamDecoder::Impl::run_demod() {
    try {
        DemodRuntimeState runtime{latest_generation.load()};
        auto &decoder_parameters = runtime.decoder_parameters;
        auto &postprocessor = runtime.postprocessor;
        auto &demod_generation = runtime.demod_generation;
        auto &seen_sync_version = runtime.seen_sync_version;
        auto &have_grid = runtime.have_grid;
        auto &symbol_count = runtime.symbol_count;
        auto &window_symbol_count = runtime.window_symbol_count;
        auto &demod_busy_started_at = runtime.demod_busy_started_at;
        const auto finish_symbol_attempt = [&] {
            demod_busy_time_sum_ms += duration_ms(demod_busy_started_at);
            runtime.demod_busy_active = false;
        };
        while (true) {
            switch (demod_prepare_stream(runtime)) {
            case DemodFlow::restart:
                continue;
            case DemodFlow::stop:
                return;
            case DemodFlow::proceed:
                break;
            }
            // --- contiguous symbol stream ---
            while (true) {
                {
                    const std::scoped_lock lock(mutex);
                    if (sync.version != seen_sync_version) {
                        static_cast<void>(demod_handle_sync_change(runtime));
                    }
                }
                fire_pending_discontinuity();
                if (have_grid && decoder_parameters &&
                    postprocessor == nullptr && !demod_start_decoder(runtime)) {
                    return;
                }
                if (!have_grid) {
                    break; // reset mid-stream: drain nothing, wait for a
                           // sync
                }
                const DemodInputFlow input_flow = demod_read_symbol(runtime);
                if (input_flow == DemodInputFlow::retry) {
                    continue;
                }
                if (input_flow == DemodInputFlow::end) {
                    break;
                }
                if (input_flow == DemodInputFlow::stop) {
                    return;
                }
                auto stage_started_at = std::chrono::steady_clock::now();
                demod_execute_fft_and_track_cfo(runtime);
                runtime.fft_cfo_time_sum_ms += duration_ms(stage_started_at);

                stage_started_at = std::chrono::steady_clock::now();
                const double reacquisition_before =
                    runtime.reacquisition_time_sum_ms;
                const auto pilot_lock = demod_lock_pilots(runtime);
                runtime.pilot_lock_time_sum_ms += std::max(
                    0.0, static_cast<double>(duration_ms(stage_started_at)) -
                             (runtime.reacquisition_time_sum_ms -
                              reacquisition_before));
                if (!pilot_lock) {
                    finish_symbol_attempt();
                    continue;
                }
                const PilotLock lock = *pilot_lock;
                stage_started_at = std::chrono::steady_clock::now();
                auto channel = demod_estimate_channel(runtime, lock);
                runtime.channel_estimate_time_sum_ms +=
                    duration_ms(stage_started_at);

                stage_started_at = std::chrono::steady_clock::now();
                const double tps_reacquisition_before =
                    runtime.reacquisition_time_sum_ms;
                const DemodFlow tps_flow = demod_process_tps(runtime);
                runtime.tps_time_sum_ms += std::max(
                    0.0, static_cast<double>(duration_ms(stage_started_at)) -
                             (runtime.reacquisition_time_sum_ms -
                              tps_reacquisition_before));
                switch (tps_flow) {
                case DemodFlow::restart:
                    finish_symbol_attempt();
                    continue;
                case DemodFlow::stop:
                    finish_symbol_attempt();
                    return;
                case DemodFlow::proceed:
                    break;
                }
                const bool payload_dispatched =
                    demod_dispatch_payload(runtime, lock, channel);
                if (window_symbol_count == 0) {
                    runtime.window_output_begin_sample =
                        runtime.next_symbol_start;
                }
                ++symbol_count;
                ++window_symbol_count;
                demod_advance_symbol(runtime);
                finish_symbol_attempt();
                if (window_symbol_count >= stats_window_symbols) {
                    demod_publish_stats_window(runtime);
                    static_cast<void>(enqueue_fec(
                        {.kind = FecItem::Kind::stats,
                         .generation = demod_generation,
                         .parameters = {},
                         .mother_metrics = {},
                         .symbol_index = 0,
                         .demod_window_sequence = runtime.window_sequence,
                         .source_epoch = runtime.window_source_epoch}));
                    demod_reset_stats_window(runtime);
                }
                if (!payload_dispatched) {
                    continue;
                }
            }
            demod_finish_stream(runtime);
        }
    } catch (const std::exception &exception) {
        // An unexpected exception here would terminate the process (the
        // thread is joined in ~Impl). Dump the pipeline state before the
        // crash so a mid-stream stall/exit is diagnosable.
        std::fprintf(stderr, "[decoder] demod thread exception: %s\n",
                     exception.what());
        throw;
    } catch (...) {
        std::fprintf(stderr, "[decoder] demod thread exception: unknown\n");
        throw;
    }
}
