#include "main_commands.hpp"

#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "receiver_session.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <ranges>
#include <string>
#include <thread>

namespace airspy_tv::cli {

int enumerate_cli() {
    const EnumerationResult result = SdrDevice::enumerate(true);
    for (const std::string &warning : result.warnings) {
        std::cerr << "warning: " << warning << '\n';
    }
    for (const DeviceDescriptor &device : result.devices) {
        std::cout << backend_name(device.backend) << '\t' << device.display_name
                  << '\t' << device.id << '\n';
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
    ReceiverSession receiver(TransportPipelineConfig::headless());
    std::string error;
    SourceSettings effective = settings;
    if (!receiver.open_device_and_start(descriptor, effective, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (!receiver.start_recording(path, effective, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    ++effective.airspy_gain;
    if (!receiver.set_gain(effective, error) ||
        !receiver.set_bias_tee(false, error) ||
        !receiver.retune(effective.center_frequency_hz + 1'000, error) ||
        !receiver.retune(effective.center_frequency_hz, error)) {
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
    ReceiverSession receiver(TransportPipelineConfig::headless());
    SourceSettings settings;
    settings.sample_rate_hz = raw_sample_rate_hz;
    settings.center_frequency_hz = raw_center_frequency_hz;
    std::string error;
    if (!receiver.open_iq_file_and_start(path, settings, error)) {
        std::cerr << error << '\n';
        return 1;
    }

    SpectrumSnapshot spectrum;
    dvbt::SignalAnalysisSnapshot analysis;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (std::chrono::steady_clock::now() < deadline) {
        spectrum = receiver.spectrum_snapshot();
        analysis = receiver.dvbt_snapshot().signal;
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
        case dvbt::GuardInterval::gi_1_32:
            guard = "1/32";
            break;
        case dvbt::GuardInterval::gi_1_16:
            guard = "1/16";
            break;
        case dvbt::GuardInterval::gi_1_8:
            guard = "1/8";
            break;
        case dvbt::GuardInterval::gi_1_4:
            break;
        }
        std::cout << ", ofdm-lock="
                  << (analysis.mode == dvbt::TransmissionMode::k8 ? "8K" : "2K")
                  << ", guard=" << guard << ", cp-snr=" << analysis.cp_snr_db
                  << " dB, mer=" << analysis.mer_db
                  << " dB, channel-notch=" << analysis.deepest_notch_db
                  << " dB, carrier-offset=" << analysis.carrier_offset_hz
                  << " Hz, constellation=";
        switch (analysis.constellation) {
        case dvbt::Constellation::qpsk:
            std::cout << "QPSK";
            break;
        case dvbt::Constellation::qam16:
            std::cout << "16-QAM";
            break;
        case dvbt::Constellation::qam64:
            std::cout << "64-QAM";
            break;
        }
        std::cout << '\n';
    } else {
        std::cout << ", ofdm-lock=no";
    }
    std::cout << '\n';
    receiver.close();
    return spectrum.valid ? 0 : 1;
}

} // namespace airspy_tv::cli
