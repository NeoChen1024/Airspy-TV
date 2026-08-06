#pragma once

#include "airspy_tv/demodulator.hpp"
#include "airspy_tv/recorder.hpp"
#include "airspy_tv/spectrum.hpp"
#include "airspy_tv/transport_stream.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace airspy_tv {

enum class SdrBackend { AirspyNative, Soapy, File };
enum class AirspyGainMode { Sensitivity, Linearity };

struct DeviceDescriptor {
    SdrBackend backend{SdrBackend::AirspyNative};
    std::string id;
    std::string display_name;
    std::string driver;
    std::string serial;
    std::map<std::string, std::string> arguments;
};

struct EnumerationResult {
    std::vector<DeviceDescriptor> devices;
    std::vector<std::string> warnings;
};

inline constexpr double max_frequency_correction_ppm = 1000.0;

struct SourceSettings {
    std::uint64_t center_frequency_hz{545'000'000};
    // Software LO correction applied as
    // hardware_frequency = nominal_frequency * (1 + ppm / 1e6).
    double frequency_correction_ppm{};
    std::uint32_t sample_rate_hz{10'000'000};
    AirspyGainMode airspy_gain_mode{AirspyGainMode::Sensitivity};
    int airspy_gain{10};
    double soapy_gain{};
    bool bias_tee{};
};

class SdrDevice {
  public:
    using TransportSink = std::function<void(std::span<const std::uint8_t>)>;

    SdrDevice();
    ~SdrDevice() noexcept;

    SdrDevice(const SdrDevice &) = delete;
    SdrDevice &operator=(const SdrDevice &) = delete;
    SdrDevice(SdrDevice &&) = delete;
    SdrDevice &operator=(SdrDevice &&) = delete;

    static EnumerationResult enumerate(bool include_soapy_airspy);

    bool open(const DeviceDescriptor &descriptor, std::string &error);
    bool open_iq_file(const std::filesystem::path &path,
                      SourceSettings &settings, std::string &error);
    void close();
    bool configure(const SourceSettings &settings, std::string &error);
    bool start_stream(const SourceSettings &settings, std::string &error);
    void stop_stream();
    bool set_center_frequency(std::uint64_t frequency_hz, std::string &error);
    bool set_frequency_correction_ppm(double ppm, std::string &error);
    bool set_gain(const SourceSettings &settings, std::string &error);
    bool set_bias_tee(bool enabled, std::string &error);
    void set_display_smoothing(bool fft_enabled, int fft_speed,
                               bool snr_enabled, int snr_speed);
    void set_demodulator_signal_smoothing(bool enabled, int speed);
    // Install the standard demodulator. The demodulator takes over the
    // MPEG-TS pipeline (service model, TS recorder, transport sink) via its
    // transport callback. Standard-specific parameters are configured on the
    // concrete type before injection. A null demodulator disconnects the
    // pipeline.
    void set_demodulator(std::unique_ptr<Demodulator> demodulator);
    void set_channel_bandwidth(std::uint32_t bandwidth_hz);

    bool start_recording(const std::filesystem::path &path,
                         const SourceSettings &settings, std::string &error);
    void stop_recording();
    bool start_ts_recording(const std::filesystem::path &path,
                            std::string &error);
    void stop_ts_recording();
    void set_transport_sink(TransportSink sink);

    [[nodiscard]] bool is_open() const;
    [[nodiscard]] bool is_streaming() const;
    [[nodiscard]] bool is_recording() const;
    [[nodiscard]] const DeviceDescriptor *descriptor() const;
    [[nodiscard]] const std::vector<std::uint32_t> &sample_rates() const;
    [[nodiscard]] std::optional<std::pair<double, double>> gain_range() const;
    [[nodiscard]] RecordingStats recording_stats() const;
    [[nodiscard]] TransportRecordingStats ts_recording_stats() const;
    [[nodiscard]] SpectrumSnapshot spectrum_snapshot() const;
    [[nodiscard]] SignalSnapshot signal_snapshot() const;
    [[nodiscard]] PipelineSnapshot pipeline_snapshot() const;
    [[nodiscard]] std::vector<TransportService> transport_services() const;
    [[nodiscard]] std::string runtime_error() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::string backend_name(SdrBackend backend);

} // namespace airspy_tv
