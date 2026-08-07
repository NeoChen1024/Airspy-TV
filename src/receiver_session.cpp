#include "receiver_session.hpp"

#include <algorithm>
#include <cstdlib>
#include <ranges>
#include <utility>

namespace airspy_tv {

ReceiverSession::ReceiverSession() {
    std::string error;
    install_demodulator(make_demodulator(ReceiveStandard::DvbT, error),
                        ReceiveStandard::DvbT);
}

ReceiveStandard ReceiverSession::standard() const noexcept { return standard_; }

std::uint32_t ReceiverSession::channel_bandwidth_hz() const noexcept {
    if (standard_ == ReceiveStandard::DvbT) {
        return dvbt_parameters_.channel_bandwidth_hz;
    }
    return 0;
}

bool ReceiverSession::is_open() const { return device_.is_open(); }

bool ReceiverSession::is_streaming() const { return device_.is_streaming(); }

bool ReceiverSession::is_recording() const { return device_.is_recording(); }

const DeviceDescriptor *ReceiverSession::descriptor() const {
    return device_.descriptor();
}

const std::vector<std::uint32_t> &ReceiverSession::sample_rates() const {
    return device_.sample_rates();
}

std::optional<std::pair<double, double>> ReceiverSession::gain_range() const {
    return device_.gain_range();
}

RecordingStats ReceiverSession::recording_stats() const {
    return device_.recording_stats();
}

TransportRecordingStats ReceiverSession::ts_recording_stats() const {
    return device_.ts_recording_stats();
}

SpectrumSnapshot ReceiverSession::spectrum_snapshot() const {
    return device_.spectrum_snapshot();
}

std::vector<TransportService> ReceiverSession::transport_services() const {
    return device_.transport_services();
}

std::string ReceiverSession::runtime_error() const {
    return device_.runtime_error();
}

std::unique_ptr<Demodulator>
ReceiverSession::make_demodulator(const ReceiveStandard standard,
                                  std::string &error) {
    if (standard == ReceiveStandard::DvbT) {
        return std::make_unique<dvbt::StreamDecoder>();
    }
    error = "The selected television standard is not implemented yet";
    return {};
}

void ReceiverSession::install_demodulator(
    std::unique_ptr<Demodulator> demodulator, const ReceiveStandard standard) {
    dvbt_ = dynamic_cast<dvbt::StreamDecoder *>(demodulator.get());
    if (dvbt_ != nullptr) {
        dvbt_->set_telemetry_enabled(dvbt_telemetry_enabled_,
                                     dvbt_telemetry_started_at_);
    }
    if (demodulator) {
        demodulator->set_discontinuity_callback(discontinuity_callback_);
    }
    device_.set_demodulator(std::move(demodulator));
    standard_ = standard;
    configure_active_demodulator();
}

void ReceiverSession::configure_active_demodulator() {
    if (dvbt_ != nullptr) {
        dvbt_->set_parameters(dvbt_parameters_);
        device_.set_channel_bandwidth(dvbt_parameters_.channel_bandwidth_hz);
    }
}

bool ReceiverSession::select_standard(const ReceiveStandard standard,
                                      const SourceSettings &settings,
                                      const bool restart, std::string &error) {
    if (standard == standard_) {
        return true;
    }

    auto replacement = make_demodulator(standard, error);
    if (!replacement) {
        return false;
    }

    const bool was_streaming = device_.is_streaming();
    device_.stop_stream();
    dvbt_ = nullptr;
    device_.set_demodulator(nullptr);
    install_demodulator(std::move(replacement), standard);

    if (restart && was_streaming && !device_.start_stream(settings, error)) {
        // The source remains open but stopped, with the replacement fully
        // configured and all callbacks rebound. This is a coherent state from
        // which the GUI can retry or close the source.
        return false;
    }
    return true;
}

void ReceiverSession::set_dvbt_parameters(
    const dvbt::ReceiverParameters &parameters) {
    dvbt_parameters_ = parameters;
    configure_active_demodulator();
}

void ReceiverSession::set_display_smoothing(const bool fft_enabled,
                                            const int fft_speed,
                                            const bool signal_enabled,
                                            const int signal_speed) {
    device_.set_display_smoothing(fft_enabled, fft_speed, signal_enabled,
                                  signal_speed);
    device_.set_demodulator_signal_smoothing(signal_enabled, signal_speed);
}

void ReceiverSession::set_dvbt_telemetry_enabled(
    const bool enabled, const dvbt::TelemetryClock::time_point started_at) {
    dvbt_telemetry_enabled_ = enabled;
    dvbt_telemetry_started_at_ = started_at;
    if (dvbt_ != nullptr) {
        dvbt_->set_telemetry_enabled(enabled, started_at);
    }
}

std::vector<dvbt::TelemetryRecord> ReceiverSession::drain_dvbt_telemetry() {
    return dvbt_ != nullptr ? dvbt_->drain_telemetry()
                            : std::vector<dvbt::TelemetryRecord>{};
}

DvbTSessionSnapshot ReceiverSession::dvbt_snapshot() const {
    if (dvbt_ == nullptr) {
        return {};
    }
    return {.signal = dvbt_->analysis_snapshot(), .decoder = dvbt_->stats()};
}

SignalSnapshot ReceiverSession::signal_snapshot() const {
    return device_.signal_snapshot();
}

PipelineSnapshot ReceiverSession::pipeline_snapshot() const {
    return device_.pipeline_snapshot();
}

InputTimelineSnapshot ReceiverSession::input_timeline_snapshot() const {
    return device_.input_timeline_snapshot();
}

bool ReceiverSession::open_device_and_start(const DeviceDescriptor &descriptor,
                                            SourceSettings &settings,
                                            std::string &error) {
    configure_active_demodulator();
    if (!device_.open(descriptor, error)) {
        return false;
    }

    if (!device_.sample_rates().empty()) {
        const auto nearest = std::ranges::min_element(
            device_.sample_rates(), {},
            [target = settings.sample_rate_hz](const std::uint32_t rate) {
                return std::llabs(static_cast<long long>(rate) - target);
            });
        settings.sample_rate_hz = *nearest;
    }
    if (const auto range = device_.gain_range(); range.has_value()) {
        settings.soapy_gain = range->first;
    }
    if (device_.start_stream(settings, error)) {
        return true;
    }
    device_.close();
    return false;
}

bool ReceiverSession::open_iq_file_and_start(const std::filesystem::path &path,
                                             SourceSettings &settings,
                                             std::string &error) {
    configure_active_demodulator();
    if (device_.open_iq_file(path, settings, error) &&
        device_.start_stream(settings, error)) {
        return true;
    }
    device_.close();
    return false;
}

bool ReceiverSession::start_stream(const SourceSettings &settings,
                                   std::string &error) {
    configure_active_demodulator();
    return device_.start_stream(settings, error);
}

void ReceiverSession::stop_stream() { device_.stop_stream(); }

void ReceiverSession::close() { device_.close(); }

bool ReceiverSession::retune(const std::uint64_t frequency_hz,
                             std::string &error) {
    // SdrDevice resets the analyzer, the existing demodulator and transport
    // model after the hardware tune. The decoder object itself is preserved,
    // so its discontinuity callback remains bound and emits retune to mpv.
    return device_.set_center_frequency(frequency_hz, error);
}

bool ReceiverSession::set_frequency_correction_ppm(const double ppm,
                                                   std::string &error) {
    // Frequency correction uses the same in-place retune/reset path.
    return device_.set_frequency_correction_ppm(ppm, error);
}

bool ReceiverSession::set_gain(const SourceSettings &settings,
                               std::string &error) {
    return device_.set_gain(settings, error);
}

bool ReceiverSession::set_bias_tee(const bool enabled, std::string &error) {
    return device_.set_bias_tee(enabled, error);
}

bool ReceiverSession::start_recording(const std::filesystem::path &path,
                                      const SourceSettings &settings,
                                      std::string &error) {
    configure_active_demodulator();
    return device_.start_recording(path, settings, error);
}

void ReceiverSession::stop_recording() { device_.stop_recording(); }

bool ReceiverSession::start_ts_recording(const std::filesystem::path &path,
                                         std::string &error) {
    return device_.start_ts_recording(path, error);
}

void ReceiverSession::stop_ts_recording() { device_.stop_ts_recording(); }

void ReceiverSession::set_transport_sink(TransportSink sink) {
    device_.set_transport_sink(std::move(sink));
}

void ReceiverSession::set_discontinuity_callback(
    DiscontinuityCallback callback) {
    discontinuity_callback_ = std::move(callback);
    if (dvbt_ != nullptr) {
        dvbt_->set_discontinuity_callback(discontinuity_callback_);
    }
}

} // namespace airspy_tv
