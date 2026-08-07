// Internal StreamDecoder::Impl worker definition.

// ------------------------------------------------------------------ //
// Front-end thread: cs16 -> resampled cfloat -> ring, plus the rolling
// acquisition window. Runs independently of the demod and dispatches large
// input blocks to a persistent output-range resampler pool. Q32.32 phase and
// FIR history carry the continuous stream state across submissions.
// ------------------------------------------------------------------ //
void StreamDecoder::Impl::run_frontend() {
    std::unique_ptr<StreamingResampler> resampler;
    std::vector<std::complex<float>> convert_buffer;
    std::optional<Block> bootstrap_block;
    bool production_ready = false;
    std::uint64_t bootstrap_attempts = 0;
    std::uint64_t bootstrap_retained_peak_samples = 0;
    const auto perform_cfo_rebootstrap_locked =
        [&](Block *active_block, const std::size_t input_offset) {
            if (!cfo_rebootstrap_requested.exchange(
                    false, std::memory_order_acq_rel)) {
                return false;
            }

            const std::uint64_t old_generation =
                latest_generation.load(std::memory_order_relaxed);
            const std::uint64_t new_generation =
                latest_generation.fetch_add(1, std::memory_order_acq_rel) + 1;

            for (auto &queued : queue) {
                queued.generation = new_generation;
            }
            if (active_block != nullptr &&
                active_block->generation == old_generation) {
                const std::size_t complex_count =
                    active_block->samples.size() / 2U;
                latest.processed_input_samples += input_offset;
                if (input_offset < complex_count) {
                    Block suffix;
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
                    queued_complex_samples += suffix.stamp.sample_count;
                    queue.push_front(std::move(suffix));
                }
            } else if (!queue.empty()) {
                queue.front().stamp.discontinuity_before = true;
            }

            ring.reset();
            resampler_timeline.clear();
            scheduled_sro_commands.clear();
            scheduled_cfo_commands.clear();
            fec_queue.clear();
            bootstrap_block.reset();
            production_ready = false;
            bootstrap_attempts = 0;
            bootstrap_retained_peak_samples = 0;
            if (resampler != nullptr) {
                resampler->set_sro_correction_ppm(0.0);
                resampler->set_cfo_correction_hz(0.0);
                resampler->reset();
            }
            sro_resampler_command_ppm.store(0.0, std::memory_order_relaxed);
            sro_resampler_applied_ppm.store(0.0, std::memory_order_relaxed);
            sro_resampler_ready.store(false, std::memory_order_release);
            cfo_resampler_command_hz.store(0.0, std::memory_order_relaxed);
            cfo_resampler_applied_hz.store(0.0, std::memory_order_relaxed);
            cfo_resampler_ready.store(false, std::memory_order_release);

            sync.valid = false;
            ++sync.version;
            latest.decoder_generation = new_generation;
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

            input_not_full.notify_all();
            fec_not_full.notify_all();
            fec_ready.notify_all();
            ring_data.notify_all();
            ring_space.notify_all();
            idle.notify_all();
            return true;
        };
    while (true) {
        Block block;
        bool close_ring = false;
        std::size_t block_resample_workers = 1;
        {
            std::unique_lock lock(mutex);
            frontend_state.store(static_cast<int>(WorkerState::waiting_input));
            input_ready.wait(lock, [this] {
                return stopping || reset_requested || flush_requested ||
                       cfo_rebootstrap_requested.load(
                           std::memory_order_acquire) ||
                       !queue.empty();
            });
            if (stopping) {
                frontend_state.store(static_cast<int>(WorkerState::exited));
                return;
            }
            if (reset_requested) {
                frontend_state.store(static_cast<int>(WorkerState::processing));
                // Tracking and the ring are still owned by the demod until
                // it abandons this generation at a symbol boundary.
                reset_acknowledged.wait(lock, [this] {
                    return stopping ||
                           demod_reset_generation >= reset_request_generation;
                });
                if (stopping) {
                    frontend_state.store(static_cast<int>(WorkerState::exited));
                    return;
                }
                const std::uint64_t generation = reset_request_generation;
                ring.reset();
                resampler_timeline.clear();
                scheduled_sro_commands.clear();
                scheduled_cfo_commands.clear();
                cfo_rebootstrap_requested.store(false,
                                                std::memory_order_release);
                bootstrap_block.reset();
                production_ready = false;
                bootstrap_attempts = 0;
                bootstrap_retained_peak_samples = 0;
                if (resampler != nullptr) {
                    resampler->set_cfo_correction_hz(0.0);
                    resampler->reset();
                }
                reset_requested = false;
                cancel_requested = false;
                completed_reset_generation = generation;
                input_not_full.notify_all();
                fec_not_full.notify_all();
                ring_data.notify_all();
                idle.notify_all();
                continue;
            }
            if (perform_cfo_rebootstrap_locked(nullptr, 0)) {
                continue;
            }
            if (queue.empty()) {
                if (flush_requested) {
                    flush_requested = false;
                    ring_closed = true;
                    close_ring = true;
                } else {
                    idle.notify_all();
                    continue;
                }
            } else {
                block = std::move(queue.front());
                queue.pop_front();
                queued_complex_samples -= block.samples.size() / 2;
                input_not_full.notify_one();
                if (block.generation == latest_generation) {
                    ++latest.input_blocks;
                }
                frontend_busy = true;
                block_resample_workers =
                    allocate_workers(parameters.worker_threads).resample;
                frontend_state.store(static_cast<int>(WorkerState::processing));
            }
        }
        if (close_ring) {
            ring_data.notify_all();
            {
                const std::scoped_lock lock(mutex);
                frontend_busy = false;
            }
            idle.notify_all();
            continue;
        }
        if (!block.samples.empty()) {
            const auto frontend_block_started_at =
                std::chrono::steady_clock::now();
            if (resampler == nullptr ||
                resampler->worker_count() != block_resample_workers) {
                resampler = std::make_unique<StreamingResampler>(
                    block_resample_workers);
            }
            // A flush followed by new submits resumes the same stream:
            // reopen the ring so the demod continues past the seam.
            {
                const std::scoped_lock lock(mutex);
                if (block.generation != latest_generation) {
                    frontend_busy = false;
                    idle.notify_all();
                    continue;
                }
                if (ring_closed) {
                    ring_closed = false;
                    ring_data.notify_all();
                }
            }
            {
                const std::scoped_lock lock(mutex);
                // Size the ring for ~0.2 s of this block's rate. Only
                // resize while the ring is empty: the read/write
                // positions are absolute counters wrapped by the size,
                // so a mid-stream resize would corrupt the wrap.
                if (ring_read_pos == ring_write_pos &&
                    ring.size() != ring_capacity_for(block.rate)) {
                    ring.resize(ring_capacity_for(block.rate));
                }
                // Acquisition may start as soon as the first bounded
                // resampler quanta fill its window, before the caller's full
                // input block completes. Publish stream metadata before any
                // corresponding output samples become visible.
                current_bandwidth = block.bandwidth;
            }
            if (resampler->configured() &&
                (resampler->rate() != block.rate ||
                 resampler->bandwidth() != block.bandwidth)) {
                // Retune: drop the filter state and invalidate the sync;
                // the demod re-acquires itself on the new rate.
                resampler->reset();
                sro_resampler_command_ppm.store(0.0, std::memory_order_relaxed);
                sro_resampler_applied_ppm.store(0.0, std::memory_order_relaxed);
                sro_resampler_ready.store(false, std::memory_order_relaxed);
                cfo_resampler_command_hz.store(0.0, std::memory_order_relaxed);
                cfo_resampler_applied_hz.store(0.0, std::memory_order_relaxed);
                cfo_resampler_ready.store(false, std::memory_order_relaxed);
                production_ready = false;
                bootstrap_block.reset();
                bootstrap_attempts = 0;
                bootstrap_retained_peak_samples = 0;
                {
                    const std::scoped_lock lock(mutex);
                    resampler_timeline.clear();
                    scheduled_sro_commands.clear();
                    scheduled_cfo_commands.clear();
                    current_bandwidth = block.bandwidth;
                    sync.valid = false;
                    ++sync.version;
                    // The demod owns all tracking state and clears it when
                    // it observes this invalid sync publication.
                }
                ring_data.notify_all();
            }
            if (!production_ready) {
                const auto append_bootstrap = [&] {
                    if (!bootstrap_block.has_value() ||
                        bootstrap_block->generation != block.generation ||
                        bootstrap_block->rate != block.rate ||
                        bootstrap_block->bandwidth != block.bandwidth ||
                        bootstrap_block->stamp.stream_epoch !=
                            block.stamp.stream_epoch ||
                        bootstrap_block->stamp.end_sample() !=
                            block.stamp.begin_sample) {
                        bootstrap_block = std::move(block);
                        return;
                    }
                    bootstrap_block->samples.insert(
                        bootstrap_block->samples.end(),
                        std::make_move_iterator(block.samples.begin()),
                        std::make_move_iterator(block.samples.end()));
                    bootstrap_block->stamp.sample_count +=
                        block.stamp.sample_count;
                };
                append_bootstrap();

                const std::size_t required = bootstrap_input_samples(
                    bootstrap_block->rate, bootstrap_block->bandwidth);
                const std::size_t buffered =
                    bootstrap_block->samples.size() / 2U;
                bootstrap_retained_peak_samples = std::max<std::uint64_t>(
                    bootstrap_retained_peak_samples, buffered);
                {
                    const std::scoped_lock lock(mutex);
                    latest.bootstrap_attempts = bootstrap_attempts;
                    latest.bootstrap_retained_peak_samples =
                        bootstrap_retained_peak_samples;
                }
                if (required == 0 || buffered < required) {
                    const std::scoped_lock lock(mutex);
                    frontend_busy = false;
                    idle.notify_all();
                    continue;
                }

                ReceiverParameters bootstrap_parameters;
                {
                    const std::scoped_lock lock(mutex);
                    bootstrap_parameters = parameters;
                }
                // Preview the whole retained prefix, not only the minimum
                // acquisition length. A normal source block can be larger
                // than the minimum window and the first usable signal may
                // begin later inside it. Acquisition.start must remain
                // relative to the complete raw prefix that production will
                // replay.
                const auto preview = resample_cs16(
                    bootstrap_block->samples, bootstrap_block->rate,
                    bootstrap_block->bandwidth, block_resample_workers);
                ++bootstrap_attempts;
                const OfdmAcquisition acquisition =
                    acquire_ofdm(preview, bootstrap_parameters, true);
                if (acquisition.score < 0.20F) {
                    // Keep a rolling acquisition-sized raw window. This bounds
                    // memory on noise-only/live inputs while allowing the next
                    // source block to move the preview forward.
                    if (buffered > required) {
                        const std::size_t discard = buffered - required;
                        bootstrap_block->samples.erase(
                            bootstrap_block->samples.begin(),
                            bootstrap_block->samples.begin() +
                                static_cast<std::ptrdiff_t>(discard * 2U));
                        bootstrap_block->stamp.begin_sample += discard;
                        bootstrap_block->stamp.sample_count -= discard;
                        bootstrap_block->stamp.discontinuity_before = false;
                    }
                    const std::scoped_lock lock(mutex);
                    latest.last_acquisition_time_ms =
                        duration_ms(frontend_block_started_at);
                    latest.bootstrap_attempts = bootstrap_attempts;
                    latest.bootstrap_retained_peak_samples =
                        bootstrap_retained_peak_samples;
                    frontend_busy = false;
                    idle.notify_all();
                    continue;
                }

                const double resampled_rate =
                    static_cast<double>(bootstrap_block->bandwidth) * 8.0 / 7.0;
                const double initial_cfo_hz =
                    static_cast<double>(
                        acquisition.total_cfo_phase_per_sample) *
                    resampled_rate / (2.0 * std::numbers::pi_v<double>);
                const double initial_fractional_cfo_hz =
                    static_cast<double>(
                        acquisition.fractional_cfo_phase_per_sample) *
                    resampled_rate / (2.0 * std::numbers::pi_v<double>);
                resampler->configure(bootstrap_block->rate,
                                     bootstrap_block->bandwidth);
                resampler->set_cfo_correction_hz(initial_cfo_hz);
                cfo_resampler_command_hz.store(initial_cfo_hz,
                                               std::memory_order_relaxed);
                cfo_resampler_ready.store(true, std::memory_order_release);

                std::uint64_t output_base = 0;
                {
                    const std::scoped_lock lock(mutex);
                    output_base = ring_write_pos;
                    stable_mode = acquisition.mode;
                    stable_guard = acquisition.guard;
                    sync.valid = true;
                    sync.start_pos = output_base + acquisition.start;
                    sync.acquisition_pilot_phase = acquisition.pilot_phase;
                    sync.score = acquisition.score;
                    sync.mode = acquisition.mode;
                    sync.guard = acquisition.guard;
                    sync.fft_size = acquisition.fft_size;
                    sync.guard_size = acquisition.guard_size;
                    sync.bandwidth = bootstrap_block->bandwidth;
                    sync.resampled_rate = static_cast<float>(resampled_rate);
                    latest.acquisition_score = acquisition.score;
                    latest.acquisition_cfo_hz =
                        static_cast<float>(initial_cfo_hz);
                    latest.acquisition_fractional_cfo_hz =
                        static_cast<float>(initial_fractional_cfo_hz);
                    latest.acquisition_carrier_bin_offset =
                        acquisition.carrier_offset;
                    latest.bootstrap_attempts = bootstrap_attempts;
                    latest.bootstrap_replayed_input_samples = buffered;
                    latest.bootstrap_retained_peak_samples =
                        bootstrap_retained_peak_samples;
                    latest.carrier_bin_offset = 0;
                    latest.tracked_carrier_offset_hz =
                        static_cast<float>(initial_cfo_hz);
                    latest.cfo_resampler_ready = true;
                    latest.cfo_resampler_command_hz =
                        static_cast<float>(initial_cfo_hz);
                    ++sync.version;
                }
                ring_data.notify_all();
                block = std::move(*bootstrap_block);
                bootstrap_block.reset();
                production_ready = true;
            }
            resampler->configure(block.rate, block.bandwidth);
            const std::size_t complex_count = block.samples.size() / 2;
            const auto convert_started_at = std::chrono::steady_clock::now();
            convert_buffer.resize(complex_count);
            dsp::convert_cs16_to_cf32(block.samples, convert_buffer);
            const float convert_time_ms = duration_ms(convert_started_at);
            float resample_time_ms = 0.0F;
            float ring_wait_time_ms = 0.0F;
            float ring_copy_time_ms = 0.0F;
            double applied_sro = resampler->applied_sro_correction_ppm();
            double applied_cfo = resampler->applied_cfo_correction_hz();
            std::size_t input_offset = 0;
            bool abandoned = false;
            bool have_resampled_span = false;
            std::uint64_t resampled_begin_sample = 0;
            std::uint64_t resampled_end_sample = 0;
            const std::size_t maximum_quantum =
                resampler_quantum_samples(block.rate);
            while (input_offset < complex_count && !abandoned) {
                {
                    const std::scoped_lock lock(mutex);
                    if (perform_cfo_rebootstrap_locked(&block, input_offset)) {
                        abandoned = true;
                    }
                }
                if (abandoned) {
                    break;
                }
                const std::uint64_t input_begin =
                    block.stamp.begin_sample + input_offset;
                std::size_t segment =
                    std::min(maximum_quantum, complex_count - input_offset);
                std::optional<ScheduledSroCommand> command_to_apply;
                std::optional<ScheduledCfoCommand> cfo_command_to_apply;
                {
                    const std::scoped_lock lock(mutex);
                    while (!scheduled_sro_commands.empty()) {
                        const auto &next = scheduled_sro_commands.front();
                        if (next.generation != block.generation ||
                            next.source_epoch != block.stamp.stream_epoch) {
                            scheduled_sro_commands.pop_front();
                            continue;
                        }
                        if (next.effective_input_sample <= input_begin) {
                            command_to_apply = next;
                            scheduled_sro_commands.pop_front();
                            continue;
                        }
                        const std::uint64_t distance =
                            next.effective_input_sample - input_begin;
                        if (distance < segment) {
                            segment = static_cast<std::size_t>(distance);
                        }
                        break;
                    }
                    latest.sro_pending_commands = scheduled_sro_commands.size();
                    while (!scheduled_cfo_commands.empty()) {
                        const auto &next = scheduled_cfo_commands.front();
                        if (next.generation != block.generation ||
                            next.source_epoch != block.stamp.stream_epoch) {
                            scheduled_cfo_commands.pop_front();
                            continue;
                        }
                        if (next.effective_input_sample <= input_begin) {
                            cfo_command_to_apply = next;
                            scheduled_cfo_commands.pop_front();
                            continue;
                        }
                        const std::uint64_t distance =
                            next.effective_input_sample - input_begin;
                        if (distance < segment) {
                            segment = static_cast<std::size_t>(distance);
                        }
                        break;
                    }
                    latest.cfo_pending_commands = scheduled_cfo_commands.size();
                }
                if (command_to_apply.has_value()) {
                    resampler->set_sro_correction_ppm(
                        command_to_apply->target_ppm);
                    const std::uint64_t late =
                        input_begin - command_to_apply->effective_input_sample;
                    const std::scoped_lock lock(mutex);
                    latest.sro_applied_input_sample = input_begin;
                    latest.sro_schedule_late_samples = late;
                }
                if (cfo_command_to_apply.has_value()) {
                    resampler->set_cfo_correction_hz(
                        cfo_command_to_apply->target_hz);
                    const std::uint64_t late =
                        input_begin -
                        cfo_command_to_apply->effective_input_sample;
                    const std::scoped_lock lock(mutex);
                    latest.cfo_applied_effective_input_sample =
                        cfo_command_to_apply->effective_input_sample;
                    latest.cfo_applied_input_sample = input_begin;
                    latest.cfo_schedule_late_samples = late;
                    latest.cfo_applied_output_sample = ring_write_pos;
                }
                if (segment == 0) {
                    continue;
                }

                const auto resample_started_at =
                    std::chrono::steady_clock::now();
                const auto resampled = resampler->process(
                    std::span{convert_buffer}.subspan(input_offset, segment));
                resample_time_ms += duration_ms(resample_started_at);
                applied_sro = resampler->applied_sro_correction_ppm();
                applied_cfo = resampler->applied_cfo_correction_hz();
                sro_resampler_applied_ppm.store(applied_sro,
                                                std::memory_order_relaxed);
                cfo_resampler_applied_hz.store(applied_cfo,
                                               std::memory_order_relaxed);
                if (cfo_command_to_apply.has_value()) {
                    cfo_resampler_ready.store(true, std::memory_order_release);
                }
                std::uint64_t output_begin = 0;
                {
                    const std::scoped_lock lock(mutex);
                    output_begin = ring_write_pos;
                    if (!have_resampled_span) {
                        resampled_begin_sample = output_begin;
                        have_resampled_span = true;
                    }
                    resampled_end_sample = output_begin + resampled.size();
                    resampler_timeline.append({
                        .stream_epoch = block.stamp.stream_epoch,
                        .input_begin = input_begin,
                        .input_end = input_begin + segment,
                        .output_begin = output_begin,
                        .output_end = output_begin + resampled.size(),
                        .input_rate_hz = block.rate,
                        .applied_correction_ppm = applied_sro,
                        .applied_cfo_correction_hz = applied_cfo,
                    });
                }

                // Push incrementally. At most one bounded resampler quantum
                // exists beyond the ring, which makes the scheduled-control
                // lead finite and independent of caller block size.
                std::size_t pushed = 0;
                while (pushed < resampled.size()) {
                    std::unique_lock lock(mutex);
                    frontend_state.store(
                        static_cast<int>(WorkerState::waiting_ring_space));
                    const auto ring_wait_started_at =
                        std::chrono::steady_clock::now();
                    ring_space.wait(lock, [this] {
                        return stopping || reset_requested || flush_requested ||
                               cfo_rebootstrap_requested.load(
                                   std::memory_order_acquire) ||
                               ring_write_pos - ring_read_pos < ring.size();
                    });
                    ring_wait_time_ms += duration_ms(ring_wait_started_at);
                    if (stopping) {
                        frontend_state.store(
                            static_cast<int>(WorkerState::exited));
                        return;
                    }
                    frontend_state.store(
                        static_cast<int>(WorkerState::processing));
                    if (perform_cfo_rebootstrap_locked(&block, input_offset)) {
                        abandoned = true;
                        break;
                    }
                    if (reset_requested ||
                        block.generation != latest_generation ||
                        (flush_requested && !demod_busy &&
                         !acquisition_pending &&
                         ring_write_pos - ring_read_pos >= ring.size())) {
                        resampler_timeline.truncate_after(ring_write_pos);
                        abandoned = true;
                        break;
                    }
                    const std::size_t used = ring_write_pos - ring_read_pos;
                    const std::size_t chunk =
                        std::min(ring.size() - used, resampled.size() - pushed);
                    const auto ring_copy_started_at =
                        std::chrono::steady_clock::now();
                    const std::size_t write_index =
                        ring_write_pos % ring.size();
                    const std::size_t first =
                        std::min(chunk, ring.size() - write_index);
                    std::copy_n(resampled.data() + pushed, first,
                                ring.data() + write_index);
                    if (first < chunk) {
                        std::copy_n(resampled.data() + pushed + first,
                                    chunk - first, ring.data());
                    }
                    ring_copy_time_ms += duration_ms(ring_copy_started_at);
                    ring_write_pos += chunk;
                    pushed += chunk;
                    ring_data.notify_all();
                }
                if (!abandoned) {
                    input_offset += segment;
                }
            }
            {
                const std::scoped_lock lock(mutex);
                if (block.generation == latest_generation) {
                    const float total_time_ms =
                        duration_ms(frontend_block_started_at);
                    latest.processed_input_samples += complex_count;
                    latest.decoder_generation = block.generation;
                    latest.source_epoch = block.stamp.stream_epoch;
                    latest.last_frontend_block_wall_time_ms = total_time_ms;
                    latest.last_frontend_convert_time_ms = convert_time_ms;
                    latest.last_frontend_resample_time_ms = resample_time_ms;
                    latest.last_frontend_ring_copy_time_ms = ring_copy_time_ms;
                    latest.last_frontend_ring_wait_time_ms = ring_wait_time_ms;
                    latest.sro_resampler_applied_ppm =
                        static_cast<float>(applied_sro);
                    latest.cfo_resampler_applied_hz =
                        static_cast<float>(applied_cfo);
                    latest.resampler_requested_ratio =
                        resampler->requested_ratio();
                    latest.resampler_effective_ratio =
                        resampler->effective_ratio();
                    current_bandwidth = block.bandwidth;
                    if (telemetry_enabled) {
                        const double accounted =
                            static_cast<double>(convert_time_ms) +
                            static_cast<double>(resample_time_ms) +
                            static_cast<double>(ring_copy_time_ms) +
                            static_cast<double>(ring_wait_time_ms);
                        FrontendBlockTelemetry record;
                        record.envelope = {
                            .sequence = ++frontend_telemetry_sequence,
                            .decoder_generation = block.generation,
                            .source_epoch = block.stamp.stream_epoch,
                            .wall_elapsed_ms = telemetry_elapsed_ms(),
                        };
                        record.input_sample_rate_hz = block.rate;
                        record.channel_bandwidth_hz = block.bandwidth;
                        record.source_begin_sample = block.stamp.begin_sample;
                        record.source_end_sample = block.stamp.end_sample();
                        record.resampled_begin_sample =
                            have_resampled_span ? resampled_begin_sample
                                                : ring_write_pos;
                        record.resampled_end_sample = have_resampled_span
                                                          ? resampled_end_sample
                                                          : ring_write_pos;
                        record.input_complex_samples = complex_count;
                        record.resampled_complex_samples =
                            record.resampled_end_sample -
                            record.resampled_begin_sample;
                        record.discontinuity_before =
                            block.stamp.discontinuity_before;
                        record.abandoned = abandoned;
                        record.bootstrap_attempts = bootstrap_attempts;
                        record.bootstrap_replayed_input_samples =
                            latest.bootstrap_replayed_input_samples;
                        record.bootstrap_retained_peak_samples =
                            bootstrap_retained_peak_samples;
                        record.requested_ratio = resampler->requested_ratio();
                        record.effective_ratio = resampler->effective_ratio();
                        record.commanded_sro_ppm =
                            sro_resampler_command_ppm.load(
                                std::memory_order_relaxed);
                        record.applied_sro_ppm = applied_sro;
                        record.commanded_cfo_hz = cfo_resampler_command_hz.load(
                            std::memory_order_relaxed);
                        record.applied_cfo_hz = applied_cfo;
                        record.command_output_sample =
                            latest.sro_command_output_sample;
                        record.command_input_sample =
                            latest.sro_command_input_sample;
                        record.effective_input_sample =
                            latest.sro_effective_input_sample;
                        record.applied_input_sample =
                            latest.sro_applied_input_sample;
                        record.fixed_delay_samples =
                            latest.sro_fixed_delay_samples;
                        record.late_samples = latest.sro_schedule_late_samples;
                        record.pending_commands = scheduled_sro_commands.size();
                        record.cfo_command_output_sample =
                            latest.cfo_command_output_sample;
                        record.cfo_command_input_sample =
                            latest.cfo_command_input_sample;
                        record.cfo_effective_input_sample =
                            latest.cfo_effective_input_sample;
                        record.cfo_applied_effective_input_sample =
                            latest.cfo_applied_effective_input_sample;
                        record.cfo_applied_input_sample =
                            latest.cfo_applied_input_sample;
                        record.cfo_applied_output_sample =
                            latest.cfo_applied_output_sample;
                        record.cfo_fixed_delay_samples =
                            latest.cfo_fixed_delay_samples;
                        record.cfo_late_samples =
                            latest.cfo_schedule_late_samples;
                        record.cfo_input_sample_rate_hz =
                            latest.cfo_input_sample_rate_hz;
                        record.cfo_pending_commands =
                            scheduled_cfo_commands.size();
                        record.cfo_rebootstrap_requests =
                            latest.cfo_rebootstrap_requests;
                        record.cfo_rebootstrap_count =
                            latest.cfo_rebootstrap_count;
                        record.cfo_rebootstrap_last_residual_hz =
                            latest.cfo_rebootstrap_last_residual_hz;
                        record.cfo_rebootstrap_output_sample =
                            latest.cfo_rebootstrap_output_sample;
                        record.cfo_rebootstrap_source_sample =
                            latest.cfo_rebootstrap_source_sample;
                        record.serial_wall_ms = {
                            {"frontend::total", total_time_ms},
                            {"frontend::convert", convert_time_ms},
                            {"frontend::resample", resample_time_ms},
                            {"frontend::ring_copy", ring_copy_time_ms},
                            {"frontend::ring_wait", ring_wait_time_ms},
                            {"frontend::other",
                             std::max(0.0, static_cast<double>(total_time_ms) -
                                               accounted)},
                        };
                        telemetry_queue.emplace_back(std::move(record));
                    }
                }
            }
        }
        {
            const std::scoped_lock lock(mutex);
            frontend_busy = false;
        }
        idle.notify_all();
    }
}
