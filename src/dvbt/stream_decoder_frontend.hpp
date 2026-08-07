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
            }
            if (resampler->configured() &&
                (resampler->rate() != block.rate ||
                 resampler->bandwidth() != block.bandwidth)) {
                // Retune: drop the filter state and invalidate the sync;
                // the demod re-acquires itself on the new rate.
                resampler->reset();
                {
                    const std::scoped_lock lock(mutex);
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
            const auto resample_started_at = std::chrono::steady_clock::now();
            const auto resampled = resampler->process(convert_buffer);
            const float resample_time_ms = duration_ms(resample_started_at);
            // Push to the ring incrementally: the ring (sized to ~0.2 s
            // of the input rate) holds no more than a block, so each
            // iteration pushes only what fits and waits for the demod to
            // free space. A
            // flush or reset arriving mid-push abandons the push instead
            // of blocking forever behind a demod that is not consuming
            // (e.g. a stream whose acquisition can never succeed): the
            // flush then closes the ring and the demod drains what is
            // there.
            std::size_t pushed = 0;
            float ring_wait_time_ms = 0.0F;
            float ring_copy_time_ms = 0.0F;
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
                    frontend_state.store(static_cast<int>(WorkerState::exited));
                    return;
                }
                frontend_state.store(static_cast<int>(WorkerState::processing));
                // Abandon the push only when the demod is genuinely not
                // consuming (the ring is full AND it is not busy — the
                // acquisition-retry stall): then the flush/reset would
                // otherwise wait forever behind the push. If the demod is
                // keeping up (it frees ring space as it decodes), finish
                // the push so no submitted data is dropped at the end of
                // a stream.
                if (reset_requested || block.generation != latest_generation ||
                    (flush_requested && !demod_busy && !acquisition_pending &&
                     ring_write_pos - ring_read_pos >= ring.size())) {
                    break;
                }
                const std::size_t used = ring_write_pos - ring_read_pos;
                const std::size_t chunk =
                    std::min(ring.size() - used, resampled.size() - pushed);
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
