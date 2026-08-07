#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/sdr.hpp"

#include <filesystem>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace airspy_tv {

struct DvbTSessionSnapshot {
    dvbt::SignalAnalysisSnapshot signal;
    dvbt::StreamDecoderStats decoder;
};

// Owns the receiver/demodulator relationship and is the only application
// layer object allowed to replace an active demodulator. Same-standard retunes
// preserve the decoder instance; standard replacement first stops the source,
// rebinds callbacks, then optionally restarts it.
class ReceiverSession {
  public:
    using TransportSink = SdrDevice::TransportSink;
    using DiscontinuityCallback = Demodulator::DiscontinuityCallback;

    ReceiverSession();
    ~ReceiverSession() noexcept = default;

    ReceiverSession(const ReceiverSession &) = delete;
    ReceiverSession &operator=(const ReceiverSession &) = delete;
    ReceiverSession(ReceiverSession &&) = delete;
    ReceiverSession &operator=(ReceiverSession &&) = delete;

    [[nodiscard]] ReceiveStandard standard() const noexcept;
    [[nodiscard]] std::uint32_t channel_bandwidth_hz() const noexcept;
    [[nodiscard]] bool is_open() const;
    [[nodiscard]] bool is_streaming() const;
    [[nodiscard]] bool is_recording() const;
    [[nodiscard]] const DeviceDescriptor *descriptor() const;
    [[nodiscard]] const std::vector<std::uint32_t> &sample_rates() const;
    [[nodiscard]] std::optional<std::pair<double, double>> gain_range() const;
    [[nodiscard]] RecordingStats recording_stats() const;
    [[nodiscard]] TransportRecordingStats ts_recording_stats() const;
    [[nodiscard]] SpectrumSnapshot spectrum_snapshot() const;
    [[nodiscard]] std::vector<TransportService> transport_services() const;
    [[nodiscard]] std::string runtime_error() const;

    bool select_standard(ReceiveStandard standard,
                         const SourceSettings &settings, bool restart,
                         std::string &error);
    void set_dvbt_parameters(const dvbt::ReceiverParameters &parameters);
    void set_display_smoothing(bool fft_enabled, int fft_speed,
                               bool signal_enabled, int signal_speed);
    void set_dvbt_telemetry_enabled(
        bool enabled,
        dvbt::TelemetryClock::time_point started_at =
            dvbt::TelemetryClock::now());
    [[nodiscard]] std::vector<dvbt::TelemetryRecord>
    drain_dvbt_telemetry();
    [[nodiscard]] DvbTSessionSnapshot dvbt_snapshot() const;
    [[nodiscard]] SignalSnapshot signal_snapshot() const;
    [[nodiscard]] PipelineSnapshot pipeline_snapshot() const;
    [[nodiscard]] InputTimelineSnapshot input_timeline_snapshot() const;

    bool open_device_and_start(const DeviceDescriptor &descriptor,
                               SourceSettings &settings, std::string &error);
    bool open_iq_file_and_start(const std::filesystem::path &path,
                                SourceSettings &settings, std::string &error);
    bool start_stream(const SourceSettings &settings, std::string &error);
    void stop_stream();
    void close();
    bool retune(std::uint64_t frequency_hz, std::string &error);
    bool set_frequency_correction_ppm(double ppm, std::string &error);
    bool set_gain(const SourceSettings &settings, std::string &error);
    bool set_bias_tee(bool enabled, std::string &error);
    bool start_recording(const std::filesystem::path &path,
                         const SourceSettings &settings, std::string &error);
    void stop_recording();
    bool start_ts_recording(const std::filesystem::path &path,
                            std::string &error);
    void stop_ts_recording();

    void set_transport_sink(TransportSink sink);
    void set_discontinuity_callback(DiscontinuityCallback callback);

  private:
    [[nodiscard]] std::unique_ptr<Demodulator>
    make_demodulator(ReceiveStandard standard, std::string &error);
    void install_demodulator(std::unique_ptr<Demodulator> demodulator,
                             ReceiveStandard standard);
    void configure_active_demodulator();

    SdrDevice device_;
    ReceiveStandard standard_{ReceiveStandard::DvbT};
    dvbt::ReceiverParameters dvbt_parameters_;
    dvbt::StreamDecoder *dvbt_{};
    bool dvbt_telemetry_enabled_{};
    dvbt::TelemetryClock::time_point dvbt_telemetry_started_at_{
        dvbt::TelemetryClock::now()};
    DiscontinuityCallback discontinuity_callback_;
};

} // namespace airspy_tv
