#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace airspy_tv {

enum class SdrBackend { AirspyNative, Soapy, File };
enum class AirspyGainMode { Sensitivity, Linearity };
enum class IqPlaybackPacing { realtime, unpaced };
enum class DecoderBackpressurePolicy { drop_when_busy, block };

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
    double frequency_correction_ppm{};
    std::uint32_t sample_rate_hz{10'000'000};
    AirspyGainMode airspy_gain_mode{AirspyGainMode::Sensitivity};
    int airspy_gain{10};
    double soapy_gain{};
    bool bias_tee{};
};

struct IqPlaybackPolicy {
    IqPlaybackPacing pacing{IqPlaybackPacing::realtime};
    DecoderBackpressurePolicy decoder_backpressure{
        DecoderBackpressurePolicy::drop_when_busy};
};

struct SdrSourceCallbacks {
    std::function<void(std::span<const std::int16_t>)> samples;
    std::function<void(std::uint64_t)> discontinuity;
    std::function<void()> finite_input_complete;
    std::function<void()> unexpected_stop;
};

// Source-only owner for Airspy, SoapySDR, and file/stdin I/Q backends. Signal
// analysis, timeline stamping, demodulation, and recording belong to the
// receiver pipeline above this boundary.
class SdrDevice {
  public:
    SdrDevice();
    ~SdrDevice() noexcept;

    SdrDevice(const SdrDevice &) = delete;
    SdrDevice &operator=(const SdrDevice &) = delete;
    SdrDevice(SdrDevice &&) = delete;
    SdrDevice &operator=(SdrDevice &&) = delete;

    static EnumerationResult enumerate(bool include_soapy_airspy);

    bool open(const DeviceDescriptor &descriptor, std::string &error);
    bool open_iq_file(const std::filesystem::path &path,
                      SourceSettings &settings, IqPlaybackPacing pacing,
                      std::string &error);
    void close() noexcept;
    bool configure(const SourceSettings &settings, std::string &error);
    bool start_stream(const SourceSettings &settings,
                      SdrSourceCallbacks callbacks, std::string &error);
    void stop_stream() noexcept;
    bool set_center_frequency(std::uint64_t frequency_hz, std::string &error);
    bool set_frequency_correction_ppm(double ppm, std::string &error);
    bool set_gain(const SourceSettings &settings, std::string &error);
    bool set_bias_tee(bool enabled, std::string &error);

    [[nodiscard]] bool is_open() const;
    [[nodiscard]] bool is_streaming() const;
    [[nodiscard]] bool input_exhausted() const;
    [[nodiscard]] const DeviceDescriptor *descriptor() const;
    [[nodiscard]] const std::vector<std::uint32_t> &sample_rates() const;
    [[nodiscard]] std::optional<std::pair<double, double>> gain_range() const;
    [[nodiscard]] std::string runtime_error() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::string backend_name(SdrBackend backend);

} // namespace airspy_tv
