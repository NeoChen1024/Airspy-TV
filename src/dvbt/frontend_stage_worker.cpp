#include "frontend_stage_internal.hpp"

namespace airspy_tv::dvbt {

// FrontendStage worker definition.

// ------------------------------------------------------------------ //
// Front-end thread: cs16 -> resampled cfloat -> ring, plus the rolling
// acquisition window. Runs independently of the demod and dispatches large
// input blocks to a persistent output-range resampler pool. Q32.32 phase and
// FIR history carry the continuous stream state across submissions.
// ------------------------------------------------------------------ //
void FrontendStage::Impl::run() {
    std::unique_ptr<StreamingResampler> resampler;
    std::vector<std::complex<float>> convert_buffer;
    std::optional<InputBlock> bootstrap_block;
    bool production_ready = false;
    std::uint64_t bootstrap_attempts = 0;
    std::uint64_t bootstrap_replayed_input_samples = 0;
    std::uint64_t bootstrap_retained_peak_samples = 0;
    const auto perform_cfo_rebootstrap = [&](InputBlock *active_block,
                                             const std::size_t input_offset) {
        if (!clock.consume_rebootstrap_request()) {
            return false;
        }

        const auto channel_result =
            samples.rebootstrap(active_block, input_offset);
        callbacks.clear_fec();
        bootstrap_block.reset();
        production_ready = false;
        bootstrap_attempts = 0;
        bootstrap_replayed_input_samples = 0;
        bootstrap_retained_peak_samples = 0;
        if (resampler != nullptr) {
            resampler->set_sro_correction_ppm(0.0);
            resampler->set_cfo_correction_hz(0.0);
            resampler->reset();
        }
        clock.reset();

        const std::uint64_t sync_version =
            callbacks.publish_rebootstrap(channel_result);
        samples.publish_sync_version(sync_version);
        callbacks.notify_fec_cancelled();
        return true;
    };
    while (true) {
        InputBlock block;
        std::size_t block_resample_workers = 1;
        state.store(static_cast<int>(WorkerState::waiting_input));
        auto work = samples.wait_frontend();
        if (work.event == SampleChannel::FrontendEvent::stop) {
            state.store(static_cast<int>(WorkerState::exited));
            return;
        }
        state.store(static_cast<int>(WorkerState::processing));
        if (work.event == SampleChannel::FrontendEvent::reset) {
            const std::uint64_t generation = work.reset_generation;
            samples.begin_frontend_reset(generation);
            clock.reset();
            bootstrap_block.reset();
            production_ready = false;
            bootstrap_attempts = 0;
            bootstrap_replayed_input_samples = 0;
            bootstrap_retained_peak_samples = 0;
            if (resampler != nullptr) {
                resampler->set_cfo_correction_hz(0.0);
                resampler->reset();
            }
            callbacks.notify_fec_cancelled();
            samples.complete_frontend_reset(generation);
            continue;
        }
        if (work.event == SampleChannel::FrontendEvent::rebootstrap) {
            static_cast<void>(perform_cfo_rebootstrap(nullptr, 0));
            samples.finish_frontend_work();
            continue;
        }
        if (work.event == SampleChannel::FrontendEvent::close_ring) {
            samples.notify_ring();
            continue;
        }
        block = std::move(*work.block);
        callbacks.input_block_started(block.generation);
        block_resample_workers =
            allocate_workers(callbacks.parameters().worker_threads).resample;
        if (!block.samples.empty()) {
            const auto frontend_block_started_at =
                std::chrono::steady_clock::now();
            if (resampler == nullptr ||
                resampler->worker_count() != block_resample_workers) {
                resampler = std::make_unique<StreamingResampler>(
                    block_resample_workers);
            }
            // A flush followed by new submits resumes the same stream. The
            // channel also resizes an empty ring before exposing new output.
            const auto prepared = samples.prepare_block(
                block.generation, ring_capacity_for(block.rate));
            if (!prepared.accepted) {
                samples.finish_frontend_work();
                continue;
            }
            // Publish stream metadata before corresponding output samples
            // become visible to the demodulator.
            clock.set_bandwidth(block.bandwidth);
            if (resampler->configured() &&
                (resampler->rate() != block.rate ||
                 resampler->bandwidth() != block.bandwidth)) {
                // Retune: drop the filter state and invalidate the sync;
                // the demod re-acquires itself on the new rate.
                resampler->reset();
                clock.reset();
                clock.set_bandwidth(block.bandwidth);
                production_ready = false;
                bootstrap_block.reset();
                bootstrap_attempts = 0;
                bootstrap_replayed_input_samples = 0;
                bootstrap_retained_peak_samples = 0;
                const std::uint64_t sync_version =
                    callbacks.invalidate_sync(block.generation);
                samples.publish_sync_version(sync_version);
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
                callbacks.publish_bootstrap_progress(
                    {.generation = block.generation,
                     .attempts = bootstrap_attempts,
                     .retained_peak_samples = bootstrap_retained_peak_samples,
                     .acquisition_time_ms = std::nullopt});
                if (required == 0 || buffered < required) {
                    samples.finish_frontend_work();
                    continue;
                }

                const ReceiverParameters bootstrap_parameters =
                    callbacks.parameters();
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
                    callbacks.publish_bootstrap_progress(
                        {.generation = block.generation,
                         .attempts = bootstrap_attempts,
                         .retained_peak_samples =
                             bootstrap_retained_peak_samples,
                         .acquisition_time_ms =
                             frontend_duration_ms(frontend_block_started_at)});
                    samples.finish_frontend_work();
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
                clock.set_initial_cfo(initial_cfo_hz);

                const std::uint64_t output_base = samples.write_position();
                const std::uint64_t sync_version =
                    callbacks.publish_acquisition(
                        {.generation = block.generation,
                         .acquisition = acquisition,
                         .output_base = output_base,
                         .bandwidth_hz = bootstrap_block->bandwidth,
                         .resampled_rate_hz =
                             static_cast<float>(resampled_rate),
                         .initial_cfo_hz = static_cast<float>(initial_cfo_hz),
                         .initial_fractional_cfo_hz =
                             static_cast<float>(initial_fractional_cfo_hz),
                         .bootstrap_attempts = bootstrap_attempts,
                         .bootstrap_replayed_input_samples = buffered,
                         .bootstrap_retained_peak_samples =
                             bootstrap_retained_peak_samples});
                samples.publish_sync_version(sync_version);
                bootstrap_replayed_input_samples = buffered;
                block = std::move(*bootstrap_block);
                bootstrap_block.reset();
                production_ready = true;
            }
            resampler->configure(block.rate, block.bandwidth);
            const std::size_t complex_count = block.samples.size() / 2;
            const auto convert_started_at = std::chrono::steady_clock::now();
            convert_buffer.resize(complex_count);
            dsp::convert_cs16_to_cf32(block.samples, convert_buffer);
            const float convert_time_ms =
                frontend_duration_ms(convert_started_at);
            float resample_time_ms = 0.0F;
            float ring_wait_time_ms = 0.0F;
            float ring_copy_time_ms = 0.0F;
            std::size_t input_offset = 0;
            bool abandoned = false;
            bool have_resampled_span = false;
            std::uint64_t resampled_begin_sample = 0;
            std::uint64_t resampled_end_sample = 0;
            const std::size_t maximum_quantum =
                resampler_quantum_samples(block.rate);
            while (input_offset < complex_count && !abandoned) {
                if (perform_cfo_rebootstrap(&block, input_offset)) {
                    abandoned = true;
                }
                if (abandoned) {
                    break;
                }
                const std::uint64_t input_begin =
                    block.stamp.begin_sample + input_offset;
                std::size_t segment =
                    std::min(maximum_quantum, complex_count - input_offset);
                std::optional<ClockControlTimeline::SroCommand>
                    command_to_apply;
                std::optional<ClockControlTimeline::CfoCommand>
                    cfo_command_to_apply;
                {
                    const auto due = clock.consume_due(block.generation,
                                                       block.stamp.stream_epoch,
                                                       input_begin, segment);
                    segment = due.input_samples;
                    command_to_apply = due.sro;
                    cfo_command_to_apply = due.cfo;
                    if (command_to_apply.has_value()) {
                        resampler->set_sro_correction_ppm(
                            command_to_apply->target_ppm);
                    }
                    if (cfo_command_to_apply.has_value()) {
                        resampler->set_cfo_correction_hz(
                            cfo_command_to_apply->target_hz);
                    }
                    callbacks.publish_command(
                        {.generation = block.generation,
                         .input_sample = input_begin,
                         .output_sample = samples.write_position(),
                         .sro = command_to_apply,
                         .cfo = cfo_command_to_apply,
                         .pending_sro = due.pending_sro,
                         .pending_cfo = due.pending_cfo});
                }
                if (segment == 0) {
                    continue;
                }

                const auto resample_started_at =
                    std::chrono::steady_clock::now();
                const auto resampled = resampler->process(
                    std::span{convert_buffer}.subspan(input_offset, segment));
                resample_time_ms += frontend_duration_ms(resample_started_at);
                const double applied_sro =
                    resampler->applied_sro_correction_ppm();
                const double applied_cfo =
                    resampler->applied_cfo_correction_hz();
                clock.set_applied(applied_sro, applied_cfo,
                                  cfo_command_to_apply.has_value());
                const std::uint64_t output_begin = samples.write_position();
                if (!have_resampled_span) {
                    resampled_begin_sample = output_begin;
                    have_resampled_span = true;
                }
                resampled_end_sample = output_begin + resampled.size();
                clock.append({
                    .stream_epoch = block.stamp.stream_epoch,
                    .input_begin = input_begin,
                    .input_end = input_begin + segment,
                    .output_begin = output_begin,
                    .output_end = output_begin + resampled.size(),
                    .input_rate_hz = block.rate,
                    .applied_correction_ppm = applied_sro,
                    .applied_cfo_correction_hz = applied_cfo,
                });

                // Push incrementally. At most one bounded resampler quantum
                // exists beyond the ring, which makes the scheduled-control
                // lead finite and independent of caller block size.
                std::size_t pushed = 0;
                while (pushed < resampled.size()) {
                    state.store(
                        static_cast<int>(WorkerState::waiting_ring_space));
                    const auto push = samples.push(
                        block.generation, std::span{resampled}.subspan(pushed));
                    ring_wait_time_ms += push.wait_time_ms;
                    ring_copy_time_ms += push.copy_time_ms;
                    if (push.status == SampleChannel::PushStatus::stop) {
                        state.store(static_cast<int>(WorkerState::exited));
                        return;
                    }
                    state.store(static_cast<int>(WorkerState::processing));
                    if (push.status == SampleChannel::PushStatus::rebootstrap) {
                        static_cast<void>(
                            perform_cfo_rebootstrap(&block, input_offset));
                        abandoned = true;
                    } else if (push.status !=
                               SampleChannel::PushStatus::written) {
                        abandoned = true;
                    }
                    if (abandoned) {
                        clock.truncate_after(push.write_begin);
                        break;
                    }
                    pushed += push.written;
                }
                if (!abandoned) {
                    input_offset += segment;
                }
            }
            clock.set_bandwidth(block.bandwidth);
            const auto clock_snapshot = clock.snapshot();
            const std::uint64_t final_write_position = samples.write_position();
            callbacks.publish_block(
                {.generation = block.generation,
                 .input_rate_hz = block.rate,
                 .bandwidth_hz = block.bandwidth,
                 .stamp = block.stamp,
                 .input_samples = complex_count,
                 .resampled_begin_sample = have_resampled_span
                                               ? resampled_begin_sample
                                               : final_write_position,
                 .resampled_end_sample = have_resampled_span
                                             ? resampled_end_sample
                                             : final_write_position,
                 .bootstrap_attempts = bootstrap_attempts,
                 .bootstrap_replayed_input_samples =
                     bootstrap_replayed_input_samples,
                 .bootstrap_retained_peak_samples =
                     bootstrap_retained_peak_samples,
                 .clock = clock_snapshot,
                 .total_time_ms =
                     frontend_duration_ms(frontend_block_started_at),
                 .convert_time_ms = convert_time_ms,
                 .resample_time_ms = resample_time_ms,
                 .ring_copy_time_ms = ring_copy_time_ms,
                 .ring_wait_time_ms = ring_wait_time_ms,
                 .requested_ratio = resampler->requested_ratio(),
                 .effective_ratio = resampler->effective_ratio(),
                 .abandoned = abandoned});
        }
        samples.finish_frontend_work();
    }
}

} // namespace airspy_tv::dvbt
