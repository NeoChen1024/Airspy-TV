#pragma once

#include "airspy_tv/recorder.hpp"
#include "airspy_tv/spectrum.hpp"

#include <cstdint>
#include <filesystem>
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

struct SourceSettings {
    std::uint64_t center_frequency_hz{545'000'000};
    std::uint32_t sample_rate_hz{10'000'000};
    AirspyGainMode airspy_gain_mode{AirspyGainMode::Sensitivity};
    int airspy_gain{10};
    double soapy_gain{};
    bool bias_tee{};
};

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
                      SourceSettings &settings, std::string &error);
    void close();
    bool configure(const SourceSettings &settings, std::string &error);
    bool start_stream(const SourceSettings &settings, std::string &error);
    void stop_stream();
    bool set_center_frequency(std::uint64_t frequency_hz, std::string &error);
    bool set_gain(const SourceSettings &settings, std::string &error);
    bool set_bias_tee(bool enabled, std::string &error);

    bool start_recording(const std::filesystem::path &path,
                         const SourceSettings &settings, std::string &error);
    void stop_recording();

    [[nodiscard]] bool is_open() const;
    [[nodiscard]] bool is_streaming() const;
    [[nodiscard]] bool is_recording() const;
    [[nodiscard]] const DeviceDescriptor *descriptor() const;
    [[nodiscard]] const std::vector<std::uint32_t> &sample_rates() const;
    [[nodiscard]] std::optional<std::pair<double, double>> gain_range() const;
    [[nodiscard]] RecordingStats recording_stats() const;
    [[nodiscard]] SpectrumSnapshot spectrum_snapshot() const;
    [[nodiscard]] std::string runtime_error() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::string backend_name(SdrBackend backend);

} // namespace airspy_tv
