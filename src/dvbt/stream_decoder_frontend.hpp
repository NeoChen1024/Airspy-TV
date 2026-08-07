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
    while (true) {
        Block block;
        bool close_ring = false;
        std::size_t block_resample_workers = 1;
        {
            std::unique_lock lock(mutex);
            frontend_state.store(static_cast<int>(WorkerState::waiting_input));
            input_ready.wait(lock, [this] {
                return stopping || reset_requested || flush_requested ||
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
                if (resampler != nullptr) {
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
                {
                    const std::scoped_lock lock(mutex);
                    resampler_timeline.clear();
                    scheduled_sro_commands.clear();
                    current_bandwidth = block.bandwidth;
                    sync.valid = false;
                    ++sync.version;
                    // The demod owns all tracking state and clears it when
                    // it observes this invalid sync publication.
                }
                ring_data.notify_all();
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
            std::size_t input_offset = 0;
            bool abandoned = false;
            const std::size_t maximum_quantum =
                resampler_quantum_samples(block.rate);
            while (input_offset < complex_count && !abandoned) {
                const std::uint64_t input_begin =
                    block.stamp.begin_sample + input_offset;
                std::size_t segment =
                    std::min(maximum_quantum, complex_count - input_offset);
                std::optional<ScheduledSroCommand> command_to_apply;
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
                    latest.sro_pending_commands =
                        scheduled_sro_commands.size();
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
                if (segment == 0) {
                    continue;
                }

                const auto resample_started_at =
                    std::chrono::steady_clock::now();
                const auto resampled = resampler->process(
                    std::span{convert_buffer}.subspan(input_offset, segment));
                resample_time_ms += duration_ms(resample_started_at);
                applied_sro = resampler->applied_sro_correction_ppm();
                sro_resampler_applied_ppm.store(applied_sro,
                                                std::memory_order_relaxed);
                std::uint64_t output_begin = 0;
                {
                    const std::scoped_lock lock(mutex);
                    output_begin = ring_write_pos;
                    resampler_timeline.append({
                        .stream_epoch = block.stamp.stream_epoch,
                        .input_begin = input_begin,
                        .input_end = input_begin + segment,
                        .output_begin = output_begin,
                        .output_end = output_begin + resampled.size(),
                        .input_rate_hz = block.rate,
                        .applied_correction_ppm = applied_sro,
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
                    const std::size_t chunk = std::min(
                        ring.size() - used, resampled.size() - pushed);
                    const auto ring_copy_started_at =
                        std::chrono::steady_clock::now();
                    const std::size_t write_index = ring_write_pos % ring.size();
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
                    latest.processed_input_samples += complex_count;
                    latest.last_frontend_block_wall_time_ms =
                        duration_ms(frontend_block_started_at);
                    latest.last_frontend_convert_time_ms = convert_time_ms;
                    latest.last_frontend_resample_time_ms = resample_time_ms;
                    latest.last_frontend_ring_copy_time_ms = ring_copy_time_ms;
                    latest.last_frontend_ring_wait_time_ms = ring_wait_time_ms;
                    latest.sro_resampler_applied_ppm =
                        static_cast<float>(applied_sro);
                    latest.resampler_requested_ratio =
                        resampler->requested_ratio();
                    latest.resampler_effective_ratio =
                        resampler->effective_ratio();
                    current_bandwidth = block.bandwidth;
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
