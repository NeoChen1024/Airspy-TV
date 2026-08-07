// Decoder diagnostics formatting and stall fingerprints.

// ------------------------------------------------------------------------ //
// Decoder diagnostics: where the pipeline threads are parked, queue/ring
// watermarks, and stall warnings. The GUI calls this every few seconds so a
// mid-stream stall ("TS stopped, CPU dropped") is visible in the terminal as
// which thread is parked where instead of as a silent hang.
// ------------------------------------------------------------------------ //

[[nodiscard]] const char *worker_state_name(const WorkerState state) {
    switch (state) {
    case WorkerState::idle:
        return "idle";
    case WorkerState::processing:
        return "busy";
    case WorkerState::waiting_input:
        return "wait-input";
    case WorkerState::waiting_ring_space:
        return "wait-ring-space";
    case WorkerState::waiting_ring_data:
        return "wait-ring-data";
    case WorkerState::waiting_sync:
        return "wait-sync";
    case WorkerState::waiting_acquisition:
        return "wait-acq";
    case WorkerState::waiting_fec_item:
        return "wait-fec";
    case WorkerState::exited:
        return "EXITED";
    }
    return "?";
}

void dump_decoder_diagnostics(const StreamDecoderStats &stats) {
    if (!airspy_tv::is_debug_enabled()) {
        return;
    }
    if (stats.failed) {
        std::cerr << "[diag] ERROR " << stats.error << '\n';
    }
    const float iq_percent =
        stats.input_queue_capacity_samples == 0
            ? 0.0F
            : 100.0F * static_cast<float>(stats.queued_input_samples) /
                  static_cast<float>(stats.input_queue_capacity_samples);
    const float ring_percent =
        stats.ring_capacity_samples == 0
            ? 0.0F
            : 100.0F * static_cast<float>(stats.ring_used_samples) /
                  static_cast<float>(stats.ring_capacity_samples);
    std::cerr << "[diag] fe=" << worker_state_name(stats.frontend_state)
              << " dm=" << worker_state_name(stats.demod_state)
              << " fec=" << worker_state_name(stats.fec_state) << " iq="
              << static_cast<int>(std::clamp(iq_percent, 0.0F, 100.0F))
              << "% ring="
              << static_cast<int>(std::clamp(ring_percent, 0.0F, 100.0F))
              << "% fecq=" << stats.queued_symbols
              << " ofdm=" << stats.ofdm_locked << " tps=" << stats.tps_locked
              << " tps-ever=" << stats.tps_ever_locked
              << " car=" << stats.carrier_bin_offset
              << " acq=" << stats.acquisition_score
              << " fi=" << stats.fade_indicator << " mer=" << stats.mer_db
              << "dB"
              << " tau=" << stats.timing_offset_samples
              << " rawtau=" << stats.raw_timing_offset_samples
              << " phys=" << stats.physical_timing_offset_samples
              << " sro=" << stats.sample_clock_offset_ppm << "ppm"
              << " srocmd=" << stats.sro_resampler_command_ppm << "ppm"
              << " sroapply=" << stats.sro_resampler_applied_ppm << "ppm"
              << " srodelay=" << stats.sro_fixed_delay_samples << "smp"
              << " srolate=" << stats.sro_schedule_late_samples << "smp"
              << " sropending=" << stats.sro_pending_commands
              << " tconf=" << stats.timing_confidence
              << " cir=" << stats.cir_offset_samples
              << " circonf=" << stats.cir_confidence
              << " tmeas=" << stats.timing_measurements
              << " taccept=" << stats.timing_accepted_measurements
              << " trej=" << stats.timing_rejected_measurements
              << " sroready=" << (stats.sro_resampler_ready ? 1 : 0)
              << " fec-skip=" << stats.fec_skipped
              << " drop=" << stats.dropped_blocks << " dm-busy="
              << static_cast<int>(stats.demod_busy_fraction * 100.0F) << "%";
    if (stats.processing_realtime_ratio > 0.0F) {
        std::cerr << " rt=" << (1.0F / stats.processing_realtime_ratio) << "x";
    }
    const auto &transport = stats.transport;
    std::cerr << " TS=" << stats.transport_bytes
              << " RS=" << transport.rs_packets
              << " rsbad=" << transport.rs_uncorrectable_packets
              << " TEI=" << transport.tei_packets
              << " tsp=" << transport.ts_packets
              << " rslock=" << (transport.rs_synchronized ? 1 : 0)
              << " energysync=" << (transport.energy_synchronized ? 1 : 0)
              << " outer=" << transport.outer_deinterleaver_phase
              << " dist=" << transport.outer_sync_distance
              << " evidence=" << transport.outer_rs_evidence;
    std::cerr << '\n';

    // Stall fingerprints: a worker parked on a wait whose predicate can never
    // be satisfied while the buffer in front of it is full.
    const bool ring_full =
        stats.ring_capacity_samples != 0 &&
        stats.ring_used_samples >= stats.ring_capacity_samples * 3 / 4;
    const bool iq_full = stats.input_queue_capacity_samples != 0 &&
                         stats.queued_input_samples * 4 >=
                             stats.input_queue_capacity_samples * 3;
    const bool fec_full =
        stats.symbol_queue_capacity != 0 &&
        stats.queued_symbols * 4 >= stats.symbol_queue_capacity * 3;
    if (stats.frontend_state == WorkerState::waiting_ring_space && ring_full) {
        std::cerr << "[diag] WARNING front-end waiting for ring space while "
                     "ring is full (demod not consuming?)\n";
    }
    if (stats.demod_state == WorkerState::waiting_ring_data && ring_full) {
        std::cerr << "[diag] WARNING demod waiting for symbol data while ring "
                     "is full (stale stream position?)\n";
    }
    if (stats.demod_state == WorkerState::waiting_sync && iq_full) {
        std::cerr << "[diag] WARNING demod waiting for sync while input queue "
                     "is full (acquisition starved?)\n";
    }
    if (stats.fec_state == WorkerState::waiting_fec_item && fec_full) {
        std::cerr << "[diag] WARNING FEC waiting for items while FEC queue is "
                     "full (symbol gate stuck?)\n";
    }
    if (stats.frontend_state == WorkerState::exited ||
        stats.demod_state == WorkerState::exited ||
        stats.fec_state == WorkerState::exited) {
        std::cerr << "[diag] WARNING a decoder worker thread exited (only "
                     "expected on stop)\n";
    }
}
