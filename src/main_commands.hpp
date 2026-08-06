// CLI command handlers for enumeration, capture inspection, and decoding.
int enumerate_cli() {
    const EnumerationResult result = SdrDevice::enumerate(true);
    for (const std::string &warning : result.warnings) {
        std::cerr << "warning: " << warning << '\n';
    }
    for (const DeviceDescriptor &device : result.devices) {
        std::cout << airspy_tv::backend_name(device.backend) << '\t'
                  << device.display_name << '\t' << device.id << '\n';
    }
    std::cout << result.devices.size() << " device(s)\n";
    return 0;
}

int record_first_cli(const std::filesystem::path &path, const int duration_ms,
                     const SourceSettings &settings) {
    EnumerationResult result = SdrDevice::enumerate(false);
    if (result.devices.empty()) {
        std::cerr << "No SDR devices found\n";
        return 1;
    }

    const auto native = std::ranges::find_if(
        result.devices, [](const DeviceDescriptor &device) {
            return device.backend == SdrBackend::AirspyNative;
        });
    const DeviceDescriptor &descriptor =
        native == result.devices.end() ? result.devices.front() : *native;
    SdrDevice receiver;
    std::string error;
    if (!receiver.open(descriptor, error)) {
        std::cerr << error << '\n';
        return 1;
    }

    SourceSettings effective = settings;
    if (!receiver.sample_rates().empty()) {
        effective.sample_rate_hz = *std::ranges::min_element(
            receiver.sample_rates(), {},
            [target = effective.sample_rate_hz](const std::uint32_t rate) {
                return std::llabs(static_cast<long long>(rate) - target);
            });
    }
    if (!receiver.start_recording(path, effective, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    ++effective.airspy_gain;
    if (!receiver.set_gain(effective, error) ||
        !receiver.set_bias_tee(false, error) ||
        !receiver.set_center_frequency(effective.center_frequency_hz + 1'000,
                                       error) ||
        !receiver.set_center_frequency(effective.center_frequency_hz, error)) {
        receiver.stop_recording();
        std::cerr << error << '\n';
        return 1;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(std::max(duration_ms, 1)));
    receiver.stop_recording();
    const auto stats = receiver.recording_stats();
    const SpectrumSnapshot spectrum = receiver.spectrum_snapshot();
    std::cout << "Recorded " << stats.complex_samples << " complex samples ("
              << stats.bytes_written
              << " bytes), queue drops=" << stats.dropped_blocks
              << ", source drops=" << stats.source_dropped_samples;
    if (spectrum.valid) {
        std::cout << ", signal power=" << std::fixed << std::setprecision(1)
                  << spectrum.signal_power_dbfs << " dBFS";
    }
    std::cout << '\n';
    return stats.bytes_written == 0 || !spectrum.valid ? 1 : 0;
}

int inspect_iq_cli(const std::filesystem::path &path,
                   const std::uint32_t raw_sample_rate_hz,
                   const std::uint64_t raw_center_frequency_hz) {
    SdrDevice receiver;
    auto demodulator = std::make_unique<StreamDecoder>();
    StreamDecoder *dvbt_demod = demodulator.get();
    receiver.set_demodulator(std::move(demodulator));
    SourceSettings settings;
    settings.sample_rate_hz = raw_sample_rate_hz;
    settings.center_frequency_hz = raw_center_frequency_hz;
    std::string error;
    if (!receiver.open_iq_file(path, settings, error) ||
        !receiver.start_stream(settings, error)) {
        std::cerr << error << '\n';
        return 1;
    }

    SpectrumSnapshot spectrum;
    SignalAnalysisSnapshot analysis;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (std::chrono::steady_clock::now() < deadline) {
        spectrum = receiver.spectrum_snapshot();
        analysis = dvbt_demod->analysis_snapshot();
        if (spectrum.sequence >= 50 && analysis.locked) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const DeviceDescriptor *descriptor = receiver.descriptor();
    std::cout << (descriptor == nullptr ? path.filename().string()
                                        : descriptor->display_name)
              << ", sample-rate=" << settings.sample_rate_hz
              << ", center-frequency=" << settings.center_frequency_hz;
    if (spectrum.valid) {
        std::cout << ", signal-power=" << std::fixed << std::setprecision(1)
                  << spectrum.signal_power_dbfs << " dBFS";
        if (spectrum.channel_metrics_valid) {
            std::cout << ", rf-snr-estimate=" << spectrum.rf_snr_db
                      << " dB, deepest-notch=" << spectrum.deepest_notch_db
                      << " dB";
        }
    }
    if (analysis.locked) {
        const char *guard = "1/4";
        switch (analysis.guard_interval) {
        case airspy_tv::dvbt::GuardInterval::gi_1_32:
            guard = "1/32";
            break;
        case airspy_tv::dvbt::GuardInterval::gi_1_16:
            guard = "1/16";
            break;
        case airspy_tv::dvbt::GuardInterval::gi_1_8:
            guard = "1/8";
            break;
        case airspy_tv::dvbt::GuardInterval::gi_1_4:
            break;
        }
        std::cout << ", ofdm-lock="
                  << (analysis.mode == airspy_tv::dvbt::TransmissionMode::k8
                          ? "8K"
                          : "2K")
                  << ", guard=" << guard << ", cp-snr=" << analysis.cp_snr_db
                  << " dB, mer=" << analysis.mer_db
                  << " dB, channel-notch=" << analysis.deepest_notch_db
                  << " dB, carrier-offset=" << analysis.carrier_offset_hz
                  << " Hz, constellation=";
        switch (analysis.constellation) {
        case airspy_tv::dvbt::Constellation::qpsk:
            std::cout << "QPSK";
            break;
        case airspy_tv::dvbt::Constellation::qam16:
            std::cout << "16-QAM";
            break;
        case airspy_tv::dvbt::Constellation::qam64:
            std::cout << "64-QAM";
            break;
        }
        std::cout << "\n";
    } else {
        std::cout << ", ofdm-lock=no";
    }
    std::cout << '\n';
    receiver.close();
    return spectrum.valid ? 0 : 1;
}

int decode_iq_cli(const std::filesystem::path &source,
                  const std::filesystem::path &destination,
                  const std::uint32_t raw_sample_rate_hz,
                  const ReceiverParameters &parameters, const bool debug) {
    IqFileInfo info;
    std::string error;
    if (!airspy_tv::resolve_iq_file(source, raw_sample_rate_hz, 0, info,
                                    error)) {
        std::cerr << error << '\n';
        return 1;
    }

    try {
        const auto output_path =
            std::filesystem::absolute(destination).lexically_normal();
        if (std::filesystem::absolute(source).lexically_normal() ==
                output_path ||
            std::filesystem::absolute(info.data_path).lexically_normal() ==
                output_path) {
            std::cerr << "Output MPEG-TS path must differ from the I/Q source "
                         "and data paths\n";
            return 1;
        }
    } catch (const std::filesystem::filesystem_error &exception) {
        std::cerr << "Unable to resolve input/output paths: "
                  << exception.what() << '\n';
        return 1;
    }

    std::ifstream input(info.data_path, std::ios::binary);
    std::ofstream output(destination, std::ios::binary | std::ios::trunc);
    if (!input || !output) {
        std::cerr << "Unable to open I/Q input or MPEG-TS output\n";
        return 1;
    }

    StreamDecoder decoder;
    ReceiverParameters decoder_parameters = parameters;
    decoder.set_parameters(decoder_parameters);
    if (debug) {
        std::cerr << "Decoder worker budget="
                  << (decoder_parameters.worker_threads == 0
                          ? airspy_tv::dvbt::default_viterbi_worker_count()
                          : decoder_parameters.worker_threads)
                  << (decoder_parameters.worker_threads == 0 ? " (auto)\n"
                                                             : "\n");
        std::cerr << "DVB-T parameters: bandwidth="
                  << decoder_parameters.channel_bandwidth_hz;
        if (decoder_parameters.mode.has_value()) {
            std::cerr << " mode="
                      << (*decoder_parameters.mode ==
                                  airspy_tv::dvbt::TransmissionMode::k2
                              ? "2k"
                              : "8k");
        } else {
            std::cerr << " mode=auto";
        }
        if (decoder_parameters.guard_interval.has_value()) {
            using airspy_tv::dvbt::GuardInterval;
            std::cerr << " guard="
                      << (*decoder_parameters.guard_interval ==
                                  GuardInterval::gi_1_32
                              ? "1/32"
                          : *decoder_parameters.guard_interval ==
                                  GuardInterval::gi_1_16
                              ? "1/16"
                          : *decoder_parameters.guard_interval ==
                                  GuardInterval::gi_1_8
                              ? "1/8"
                              : "1/4");
        } else {
            std::cerr << " guard=auto";
        }
        if (decoder_parameters.constellation.has_value()) {
            using airspy_tv::dvbt::Constellation;
            std::cerr << " modulation="
                      << (*decoder_parameters.constellation ==
                                  Constellation::qpsk
                              ? "qpsk"
                          : *decoder_parameters.constellation ==
                                  Constellation::qam16
                              ? "qam16"
                              : "qam64");
        } else {
            std::cerr << " modulation=auto";
        }
        if (decoder_parameters.code_rate.has_value()) {
            using airspy_tv::dvbt::CodeRate;
            const char *rate =
                *decoder_parameters.code_rate == CodeRate::rate_1_2   ? "1/2"
                : *decoder_parameters.code_rate == CodeRate::rate_2_3 ? "2/3"
                : *decoder_parameters.code_rate == CodeRate::rate_3_4 ? "3/4"
                : *decoder_parameters.code_rate == CodeRate::rate_5_6 ? "5/6"
                                                                      : "7/8";
            std::cerr << " code-rate=" << rate;
        } else {
            std::cerr << " code-rate=auto";
        }
        std::cerr << '\n';
    }
    std::atomic_bool output_failed{};
    decoder.set_transport_callback(
        [&output, &output_failed](const std::span<const std::uint8_t> ts) {
            output.write(reinterpret_cast<const char *>(ts.data()),
                         static_cast<std::streamsize>(ts.size()));
            if (!output) {
                output_failed.store(true, std::memory_order_relaxed);
            }
        });

    const auto started_at = std::chrono::steady_clock::now();
    // Submit in ~0.2 s spans (the decoder's ingestion budget) rather than a
    // fixed 0.7 s block: the queue and ring are sized from the same budget,
    // so chunking must not exceed it or it silently becomes the latency.
    const std::size_t scalar_samples =
        StreamDecoder::chunk_samples_for(info.sample_rate_hz) * 2;
    std::vector<std::int16_t> block(scalar_samples);
    std::uint64_t input_complex_samples = 0;
    std::uint64_t reported_processed_chunks = 0;
    std::uint64_t reported_processed_samples = 0;
    std::uint64_t reported_transport_bytes = 0;
    std::uint64_t reported_phase_discontinuities = 0;
    const auto report_chunk = [&](const StreamDecoderStats &stats) {
        reported_processed_chunks = stats.processed_chunks;
        reported_processed_samples = stats.processed_input_samples;
        reported_transport_bytes = stats.transport_bytes;
        const float realtime_speed =
            stats.processing_realtime_ratio > 0.0F
                ? 1.0F / stats.processing_realtime_ratio
                : 0.0F;
        std::cerr << "chunk=" << stats.processed_chunks
                  << " input=" << stats.processed_input_samples
                  << " samples TS=" << stats.transport_bytes
                  << " bytes diag=[fe="
                  << worker_state_name(stats.frontend_state)
                  << " dm=" << worker_state_name(stats.demod_state)
                  << " fec=" << worker_state_name(stats.fec_state)
                  << " dm-busy="
                  << static_cast<int>(stats.demod_busy_fraction * 100.0F)
                  << "% ring=" << stats.ring_used_samples << "/"
                  << stats.ring_capacity_samples << "]"
                  << " realtime-speed=" << realtime_speed << "x";
        if (stats.mer_db != 0.0F) {
            std::cerr << " MER=" << stats.mer_db << " dB";
        }
        std::cerr << " carrier=" << stats.carrier_bin_offset
                  << " residual=" << stats.residual_carrier_offset_hz << " Hz"
                  << " tracked=" << stats.tracked_carrier_offset_hz << " Hz"
                  << " start=" << stats.acquisition_start
                  << " timing=" << stats.timing_offset_samples << " smp"
                  << " sro=" << stats.sample_clock_offset_ppm << " ppm"
                  << " shift=" << stats.cumulative_timing_shift_samples
                  << " smp"
                  << " act=" << stats.rolling_timing_shift_rate_ppm << " ppm"
                  << " tconf=" << stats.timing_confidence
                  << " sro-ready=" << (stats.timing_drift_ready ? 1 : 0)
                  << " carried=" << (stats.state_carried ? 1 : 0)
                  << " fec-skip=" << (stats.fec_skipped ? 1 : 0) << '\n';
        if (debug) {
            std::cerr << "  timing: raw=" << stats.raw_timing_offset_samples
                      << " filtered=" << stats.timing_offset_samples
                      << " physical=" << stats.physical_timing_offset_samples
                      << " smp observed-drift="
                      << stats.observed_timing_drift_samples
                      << " corrected-drift="
                      << stats.corrected_timing_drift_samples
                      << " smooth=" << stats.smoothed_timing_drift_samples
                      << " smp/current-window shift-rate="
                      << stats.timing_shift_rate_ppm
                      << " ppm rolling-shift-rate="
                      << stats.rolling_timing_shift_rate_ppm
                      << " ppm frac=" << stats.fractional_timing_samples
                      << " cir=" << stats.cir_offset_samples
                      << " smp cir-conf=" << stats.cir_confidence
                      << " measurements=" << stats.timing_measurements
                      << " accepted=" << stats.timing_accepted_measurements
                      << " rejected=" << stats.timing_rejected_measurements
                      << '\n';
            std::cerr << "  pipeline: frontend-wall="
                      << stats.last_frontend_block_wall_time_ms
                      << " ms convert=" << stats.last_frontend_convert_time_ms
                      << " ms resample=" << stats.last_frontend_resample_time_ms
                      << " ms ring-copy="
                      << stats.last_frontend_ring_copy_time_ms
                      << " ms ring-wait="
                      << stats.last_frontend_ring_wait_time_ms << " ms\n"
                      << "            demod-wall="
                      << stats.demod_window_wall_time_ms
                      << " ms demod-busy=" << stats.demod_busy_time_ms
                      << " ms (" << stats.demod_busy_fraction * 100.0F
                      << "%) last-acquisition="
                      << stats.last_acquisition_time_ms << " ms; workers "
                      << "resample=" << stats.resample_workers
                      << " symbol=" << stats.symbol_workers
                      << " Viterbi=" << stats.transport.viterbi_workers << '\n';
            std::cerr << "  worker work (aggregate): symbol-preprocess="
                      << stats.symbol_preprocess_work_time_ms
                      << " ms symbol-demap=" << stats.symbol_demap_work_time_ms
                      << " ms symbol-deinterleave="
                      << stats.symbol_deinterleave_work_time_ms
                      << " ms symbol-depuncture/quantize="
                      << stats.symbol_depuncture_work_time_ms
                      << " ms FEC=" << stats.fec_work_time_ms
                      << " ms (transport subset="
                      << stats.transport_work_time_ms << " ms)\n";
            std::cerr << "  TPS: " << (stats.tps_locked ? "locked" : "unlocked")
                      << '\n';
            if (stats.transport.pre_viterbi_compared_bits != 0) {
                const double pre_viterbi_ber =
                    static_cast<double>(
                        stats.transport.pre_viterbi_error_bits) /
                    static_cast<double>(
                        stats.transport.pre_viterbi_compared_bits);
                std::cerr << std::format("  BER: pre-Viterbi={:.3e}",
                                         pre_viterbi_ber);
                if (stats.transport.post_viterbi_compared_bits != 0) {
                    const double post_viterbi_ber =
                        static_cast<double>(
                            stats.transport.post_viterbi_error_bits) /
                        static_cast<double>(
                            stats.transport.post_viterbi_compared_bits);
                    std::cerr << std::format(" post-Viterbi={:.3e}",
                                             post_viterbi_ber);
                } else {
                    std::cerr << " post-Viterbi=--";
                }
                std::cerr << '\n';
            }
        }
        const std::uint64_t phase_delta =
            stats.pilot_phase_discontinuities - reported_phase_discontinuities;
        reported_phase_discontinuities = stats.pilot_phase_discontinuities;
        if (phase_delta != 0) {
            std::cerr << "  phase-discontinuities=" << phase_delta << '\n';
        }
    };
    while (input && !output_failed.load(std::memory_order_relaxed)) {
        input.read(
            reinterpret_cast<char *>(block.data()),
            static_cast<std::streamsize>(block.size() * sizeof(block.front())));
        const std::streamsize bytes_read = input.gcount();
        if (bytes_read <= 0) {
            break;
        }
        const std::size_t scalar_count =
            static_cast<std::size_t>(bytes_read) / sizeof(block.front());
        input_complex_samples += scalar_count / 2;
        decoder.submit_blocking(std::span(block).first(scalar_count),
                                info.sample_rate_hz);
        if (scalar_count == block.size()) {
            const auto progress = decoder.stats();
            if (progress.processed_chunks != reported_processed_chunks) {
                report_chunk(progress);
            }
        }
    }
    decoder.flush();
    output.flush();

    const auto stats = decoder.stats();
    if (reported_processed_samples != stats.processed_input_samples ||
        reported_transport_bytes != stats.transport_bytes) {
        report_chunk(stats);
    }
    const double input_seconds = static_cast<double>(input_complex_samples) /
                                 static_cast<double>(info.sample_rate_hz);
    const double wall_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      started_at)
            .count();
    if (debug) {
        std::cerr << "decoded " << input_complex_samples << " complex samples ("
                  << input_seconds << " s) in " << wall_seconds
                  << " s, TS=" << stats.transport_bytes
                  << " bytes, symbols=" << stats.ofdm_symbols
                  << ", RS=" << stats.transport.rs_packets << ", RS failures="
                  << stats.transport.rs_uncorrectable_packets
                  << ", TEI=" << stats.transport.tei_packets
                  << ", packets=" << stats.transport.ts_packets << '\n';
    }
    if (output_failed.load(std::memory_order_relaxed) || !output) {
        std::cerr << "Failed while writing MPEG-TS output\n";
        return 1;
    }
    if (stats.dropped_blocks != 0) {
        std::cerr << "Internal error: decoder-paced I/Q input dropped "
                  << stats.dropped_blocks << " block(s)\n";
        return 1;
    }
    return stats.transport_bytes == 0 ? 2 : 0;
}
