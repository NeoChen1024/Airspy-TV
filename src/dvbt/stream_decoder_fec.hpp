// Internal StreamDecoder::Impl worker definition.

// ------------------------------------------------------------------ //
// FEC worker: consumes the continuous symbol stream through the stateful
// TransportDecoder (Viterbi pool, RS, TS output). No per-chunk seams:
// symbols flow straight through, and end/begin items only bracket gated
// (hopeless) regions and stream boundaries.
// ------------------------------------------------------------------ //
void StreamDecoder::Impl::run_fec() {
    std::unique_ptr<Decoder> decoder;
    std::uint64_t decoder_generation = 0;
    float fec_work_ms = 0.0F;
    std::uint64_t window_transport_bytes = 0;
    float decoder_transport_time_ms = 0.0F;
    while (true) {
        FecItem item;
        {
            std::unique_lock lock(mutex);
            fec_state.store(static_cast<int>(WorkerState::waiting_fec_item));
            fec_ready.wait(lock,
                           [this] { return stopping || !fec_queue.empty(); });
            if (stopping) {
                fec_state.store(static_cast<int>(WorkerState::exited));
                return;
            }
            fec_state.store(static_cast<int>(WorkerState::processing));
            item = std::move(fec_queue.front());
            fec_queue.pop_front();
            fec_worker_busy = true;
        }
        fec_not_full.notify_one();

        if (item.generation == latest_generation) {
            if (item.kind == FecItem::Kind::begin) {
                const bool parameters_match =
                    decoder != nullptr &&
                    decoder->parameters() == item.parameters;
                if (!parameters_match) {
                    // A mode/constellation/code-rate change requires a
                    // new Decoder configuration. A generation change by
                    // itself does not: the Decoder owns a long-lived
                    // Viterbi worker pool, and reset() is sufficient to
                    // discard the old stream state.
                    if (decoder != nullptr &&
                        decoder_generation == item.generation) {
                        fire_discontinuity(
                            TransportDiscontinuity::fec_region_reset);
                    }
                    decoder = std::make_unique<Decoder>(item.parameters);
                } else {
                    // A new generation (retune/source reset) or a gated
                    // region ended. Keep the Viterbi threads and reset
                    // only the decoder state; generation filtering above
                    // already prevents stale FEC items from crossing the
                    // seam.
                    if (decoder_generation == item.generation) {
                        fire_discontinuity(
                            TransportDiscontinuity::fec_region_reset);
                    }
                    decoder->reset();
                }
                decoder_transport_time_ms = 0.0F;
                decoder_generation = item.generation;
            } else if (item.kind == FecItem::Kind::symbol && decoder &&
                       decoder_generation == item.generation) {
                const auto fec_started_at = std::chrono::steady_clock::now();
                const auto ts =
                    decoder->process_soft_metrics(item.mother_metrics);
                fec_work_ms += duration_ms(fec_started_at);
                if (!ts.empty()) {
                    TransportCallback sink;
                    {
                        const std::scoped_lock guard(mutex);
                        if (item.generation == latest_generation) {
                            sink = callback;
                            window_transport_bytes += ts.size();
                        }
                    }
                    if (sink) {
                        sink(ts);
                    }
                }
            } else if ((item.kind == FecItem::Kind::end ||
                        item.kind == FecItem::Kind::stream_end) &&
                       decoder && decoder_generation == item.generation) {
                const auto fec_started_at = std::chrono::steady_clock::now();
                const auto ts = decoder->flush();
                fec_work_ms += duration_ms(fec_started_at);
                if (!ts.empty()) {
                    TransportCallback sink;
                    {
                        const std::scoped_lock guard(mutex);
                        if (item.generation == latest_generation) {
                            sink = callback;
                            window_transport_bytes += ts.size();
                        }
                    }
                    if (sink) {
                        sink(ts);
                    }
                }
                const float total_transport_time_ms =
                    decoder->timing().transport_time_ms;
                const float window_transport_time_ms = std::max(
                    0.0F, total_transport_time_ms - decoder_transport_time_ms);
                decoder_transport_time_ms = total_transport_time_ms;
                bool fire_stream_end = false;
                {
                    const std::scoped_lock guard(mutex);
                    if (item.generation == latest_generation) {
                        latest.fec_work_time_ms = fec_work_ms;
                        latest.transport_bytes += window_transport_bytes;
                        latest.transport = decoder->stats();
                        latest.transport_work_time_ms =
                            window_transport_time_ms;
                        fire_stream_end =
                            item.kind == FecItem::Kind::stream_end;
                    }
                }
                fec_work_ms = 0.0F;
                window_transport_bytes = 0;
                if (fire_stream_end) {
                    // This event is part of the serialized FEC output
                    // stream: all final TS bytes have been delivered.
                    fire_discontinuity(TransportDiscontinuity::stream_end);
                }
            } else if (item.kind == FecItem::Kind::stats && decoder &&
                       decoder_generation == item.generation) {
                const float total_transport_time_ms =
                    decoder->timing().transport_time_ms;
                const float window_transport_time_ms = std::max(
                    0.0F, total_transport_time_ms - decoder_transport_time_ms);
                decoder_transport_time_ms = total_transport_time_ms;
                const std::scoped_lock guard(mutex);
                if (item.generation == latest_generation) {
                    latest.fec_work_time_ms = fec_work_ms;
                    latest.transport_bytes += window_transport_bytes;
                    latest.transport = decoder->stats();
                    latest.transport_work_time_ms = window_transport_time_ms;
                }
                fec_work_ms = 0.0F;
                window_transport_bytes = 0;
            }
        }
        {
            const std::scoped_lock guard(mutex);
            fec_worker_busy = false;
        }
        idle.notify_all();
    }
}
