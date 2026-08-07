// Internal StreamDecoder::Impl worker definition.

// ------------------------------------------------------------------ //
// FEC worker: consumes the continuous symbol stream through the stateful
// TransportDecoder (Viterbi pool, RS, TS output). No per-chunk seams:
// symbols flow straight through, and end/begin items only bracket gated
// (hopeless) regions and stream boundaries.
// ------------------------------------------------------------------ //
void StreamDecoder::Impl::run_fec() {
    struct DiagnosticContext {
        std::uint64_t generation{};
        std::uint64_t source_epoch{};
        std::uint64_t demod_window_sequence{};
        std::optional<std::uint64_t> tps_symbol_index;
    } diagnostic_context;
    std::uint64_t fec_session = 0;
    std::unique_ptr<Decoder> decoder;
    std::uint64_t decoder_generation = 0;
    float fec_work_ms = 0.0F;
    std::uint64_t window_transport_bytes = 0;
    float decoder_transport_time_ms = 0.0F;
    TransportDecoderStats completed_sessions;
    TransportDecoderStats previous_marker;
    const auto publish_fec_window =
        [this, &decoder, &fec_work_ms, &window_transport_bytes,
         &decoder_transport_time_ms, &fec_session, &completed_sessions,
         &previous_marker](const FecItem &item) {
            if (!decoder) {
                return;
            }
            const float total_transport_time_ms =
                decoder->timing().transport_time_ms;
            const float window_transport_time_ms = std::max(
                0.0F, total_transport_time_ms - decoder_transport_time_ms);
            decoder_transport_time_ms = total_transport_time_ms;
            const TransportDecoderStats current = decoder->stats();
            const TransportDecoderStats delta =
                transport_counter_delta(current, previous_marker);
            previous_marker = current;
            TransportDecoderStats cumulative = completed_sessions;
            add_transport_counters(cumulative, current);
            const std::scoped_lock guard(mutex);
            if (item.generation != latest_generation) {
                return;
            }
            latest.fec_work_time_ms = fec_work_ms;
            latest.transport_bytes += window_transport_bytes;
            latest.transport = current;
            latest.cumulative_transport = cumulative;
            latest.fec_sessions = fec_session;
            latest.transport_work_time_ms = window_transport_time_ms;
            if (telemetry_enabled && item.demod_window_sequence != 0) {
                FecWindowTelemetry record;
                record.envelope = {
                    .sequence = ++fec_telemetry_sequence,
                    .decoder_generation = item.generation,
                    .source_epoch = item.source_epoch,
                    .wall_elapsed_ms = telemetry_elapsed_ms(),
                };
                record.demod_window_sequence =
                    item.demod_window_sequence;
                record.fec_session = fec_session;
                record.output_bytes_delta = window_transport_bytes;
                record.output_bytes_cumulative = latest.transport_bytes;
                record.session = current;
                record.delta = delta;
                record.cumulative = cumulative;
                record.fec_total_ms = fec_work_ms;
                record.transport_nested_ms = window_transport_time_ms;
                telemetry_queue.emplace_back(std::move(record));
            }
            fec_work_ms = 0.0F;
            window_transport_bytes = 0;
        };
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

        diagnostic_context = {
            .generation = item.generation,
            .source_epoch = item.source_epoch,
            .demod_window_sequence = item.demod_window_sequence,
            .tps_symbol_index =
                item.kind == FecItem::Kind::symbol
                    ? std::optional<std::uint64_t>{item.symbol_index}
                    : std::nullopt,
        };

        if (item.generation == latest_generation) {
            if (item.kind == FecItem::Kind::begin) {
                const bool parameters_match =
                    decoder != nullptr &&
                    decoder->parameters() == item.parameters;
                if (decoder != nullptr) {
                    add_transport_counters(completed_sessions,
                                           decoder->stats());
                }
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
                    decoder->set_diagnostic_handler(
                        {.enabled = [this] { return events_enabled(); },
                         .emit = [this, &diagnostic_context,
                                  &fec_session](DiagnosticEvent event) {
                             emit_diagnostic_event(
                                 std::move(event),
                                 diagnostic_context.generation,
                                 diagnostic_context.source_epoch,
                                 diagnostic_context.demod_window_sequence,
                                 fec_session,
                                 diagnostic_context.tps_symbol_index);
                         }});
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
                previous_marker = {};
                ++fec_session;
                {
                    const std::scoped_lock guard(mutex);
                    if (item.generation == latest_generation) {
                        latest.fec_sessions = fec_session;
                        latest.cumulative_transport = completed_sessions;
                    }
                }
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
                bool fire_stream_end = false;
                {
                    const std::scoped_lock guard(mutex);
                    fire_stream_end = item.generation == latest_generation &&
                                      item.kind == FecItem::Kind::stream_end;
                }
                publish_fec_window(item);
                if (fire_stream_end) {
                    // This event is part of the serialized FEC output
                    // stream: all final TS bytes have been delivered.
                    fire_discontinuity(TransportDiscontinuity::stream_end);
                }
            } else if (item.kind == FecItem::Kind::stats && decoder &&
                       decoder_generation == item.generation) {
                publish_fec_window(item);
            }
        }
        {
            const std::scoped_lock guard(mutex);
            fec_worker_busy = false;
        }
        idle.notify_all();
    }
}
