#include "receiver_session.hpp"

#include <algorithm>
#include <cstdlib>
#include <ranges>
#include <utility>

namespace airspy_tv {

ReceiverSession::ReceiverSession() : receiver_(transport_) {
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

bool ReceiverSession::is_open() const { return receiver_.is_open(); }

bool ReceiverSession::is_streaming() const { return receiver_.is_streaming(); }

bool ReceiverSession::input_exhausted() const {
    return receiver_.input_exhausted();
}

bool ReceiverSession::is_recording() const { return receiver_.is_recording(); }

const DeviceDescriptor *ReceiverSession::descriptor() const {
    return receiver_.descriptor();
}

const std::vector<std::uint32_t> &ReceiverSession::sample_rates() const {
    return receiver_.sample_rates();
}

std::optional<std::pair<double, double>> ReceiverSession::gain_range() const {
    return receiver_.gain_range();
}

RecordingStats ReceiverSession::recording_stats() const {
    return receiver_.recording_stats();
}

TransportRecordingStats ReceiverSession::ts_recording_stats() const {
    return transport_.snapshot().recorder;
}

RtpUdpStats ReceiverSession::rtp_streaming_stats() const {
    return transport_.snapshot().rtp;
}

SpectrumSnapshot ReceiverSession::spectrum_snapshot() const {
    return receiver_.spectrum_snapshot();
}

std::vector<TransportService> ReceiverSession::transport_services() const {
    return transport_.services();
}

EpgSnapshot
ReceiverSession::epg_snapshot(const std::uint16_t service_id) const {
    return transport_.epg_snapshot(service_id);
}

TransportPipelineSnapshot ReceiverSession::transport_snapshot() const {
    return transport_.snapshot();
}

std::vector<TransportOutputTelemetry>
ReceiverSession::transport_output_telemetry() const {
    return transport_.output_telemetry();
}

std::string ReceiverSession::runtime_error() const {
    return receiver_.runtime_error();
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
    receiver_.set_demodulator(std::move(demodulator));
    standard_ = standard;
    configure_active_demodulator();
}

void ReceiverSession::configure_active_demodulator() {
    if (dvbt_ != nullptr) {
        dvbt_->set_parameters(dvbt_parameters_);
        receiver_.set_channel_bandwidth(dvbt_parameters_.channel_bandwidth_hz);
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

    const bool was_streaming = receiver_.is_streaming();
    receiver_.stop_stream();
    dvbt_ = nullptr;
    receiver_.set_demodulator(nullptr);
    install_demodulator(std::move(replacement), standard);

    if (restart && was_streaming && !receiver_.start_stream(settings, error)) {
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
    receiver_.set_display_smoothing(fft_enabled, fft_speed, signal_enabled,
                                    signal_speed);
    receiver_.set_demodulator_signal_smoothing(signal_enabled, signal_speed);
}

void ReceiverSession::set_display_analysis_enabled(
    const bool enabled) noexcept {
    receiver_.set_display_analysis_enabled(enabled);
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
    return receiver_.signal_snapshot();
}

PipelineSnapshot ReceiverSession::pipeline_snapshot() const {
    return receiver_.pipeline_snapshot();
}

InputTimelineSnapshot ReceiverSession::input_timeline_snapshot() const {
    return receiver_.input_timeline_snapshot();
}

bool ReceiverSession::open_device_and_start(const DeviceDescriptor &descriptor,
                                            SourceSettings &settings,
                                            std::string &error) {
    configure_active_demodulator();
    if (!receiver_.open(descriptor, error)) {
        return false;
    }

    if (!receiver_.sample_rates().empty()) {
        const auto nearest = std::ranges::min_element(
            receiver_.sample_rates(), {},
            [target = settings.sample_rate_hz](const std::uint32_t rate) {
                return std::llabs(static_cast<long long>(rate) - target);
            });
        settings.sample_rate_hz = *nearest;
    }
    if (const auto range = receiver_.gain_range(); range.has_value()) {
        settings.soapy_gain = range->first;
    }
    if (receiver_.start_stream(settings, error)) {
        return true;
    }
    receiver_.close();
    return false;
}

bool ReceiverSession::open_iq_file_and_start(const std::filesystem::path &path,
                                             SourceSettings &settings,
                                             std::string &error) {
    return open_iq_file_and_start(path, settings, {}, error);
}

bool ReceiverSession::open_iq_file_and_start(const std::filesystem::path &path,
                                             SourceSettings &settings,
                                             const IqPlaybackPolicy policy,
                                             std::string &error) {
    configure_active_demodulator();
    if (receiver_.open_iq_file(path, settings, policy, error) &&
        receiver_.start_stream(settings, error)) {
        return true;
    }
    receiver_.close();
    return false;
}

bool ReceiverSession::start_stream(const SourceSettings &settings,
                                   std::string &error) {
    configure_active_demodulator();
    return receiver_.start_stream(settings, error);
}

void ReceiverSession::stop_stream() { receiver_.stop_stream(); }

void ReceiverSession::finish_stream() { receiver_.finish_stream(); }

void ReceiverSession::close() {
    receiver_.close();
    transport_.stop_recording();
    transport_.stop_rtp();
}

bool ReceiverSession::retune(const std::uint64_t frequency_hz,
                             std::string &error) {
    // ReceiverPipeline resets analysis and the existing demodulator after the
    // hardware tune. TransportPipeline receives the decoder's typed retune.
    return receiver_.retune(frequency_hz, error);
}

bool ReceiverSession::set_frequency_correction_ppm(const double ppm,
                                                   std::string &error) {
    // Frequency correction uses the same in-place retune/reset path.
    return receiver_.set_frequency_correction_ppm(ppm, error);
}

bool ReceiverSession::set_gain(const SourceSettings &settings,
                               std::string &error) {
    return receiver_.set_gain(settings, error);
}

bool ReceiverSession::set_bias_tee(const bool enabled, std::string &error) {
    return receiver_.set_bias_tee(enabled, error);
}

bool ReceiverSession::start_recording(const std::filesystem::path &path,
                                      const SourceSettings &settings,
                                      std::string &error) {
    configure_active_demodulator();
    return receiver_.start_recording(path, settings, error);
}

void ReceiverSession::stop_recording() { receiver_.stop_recording(); }

bool ReceiverSession::start_ts_recording(const std::filesystem::path &path,
                                         std::string &error) {
    if (!is_streaming()) {
        error = "Start an SDR or I/Q file source before recording MPEG-TS";
        return false;
    }
    return transport_.start_recording(path, error);
}

void ReceiverSession::stop_ts_recording() { transport_.stop_recording(); }

bool ReceiverSession::start_rtp_streaming(const RtpUdpEndpoint &endpoint,
                                          std::string &error) {
    if (!is_streaming()) {
        error = "Start an SDR or I/Q file source before RTP/UDP streaming";
        return false;
    }
    return transport_.start_rtp(endpoint, error);
}

void ReceiverSession::stop_rtp_streaming() { transport_.stop_rtp(); }

void ReceiverSession::set_transport_sink(TransportSink sink) {
    transport_.set_sink(std::move(sink));
}

void ReceiverSession::set_discontinuity_callback(
    DiscontinuityCallback callback) {
    discontinuity_callback_ = std::move(callback);
    transport_.set_discontinuity_sink(discontinuity_callback_);
}

} // namespace airspy_tv
