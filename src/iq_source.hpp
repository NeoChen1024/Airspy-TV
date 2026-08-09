#pragma once

#include "airspy_tv/sdr.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace airspy_tv {

struct IqSourceCallbacks {
    std::function<void(std::span<const std::int16_t>)> samples;
    std::function<void(std::uint64_t)> discontinuity;
    std::function<void()> finite_input_complete;
    std::function<void()> unexpected_stop;
};

// Private source-backend contract. Sources own hardware/file handles, pacing,
// and source workers; the receiver pipeline owns timeline stamping, display
// analysis, demodulation, recording, and transport routing.
class IqSource {
  public:
    virtual ~IqSource() = default;

    IqSource(const IqSource &) = delete;
    IqSource &operator=(const IqSource &) = delete;
    IqSource(IqSource &&) = delete;
    IqSource &operator=(IqSource &&) = delete;

    [[nodiscard]] virtual const DeviceDescriptor &
    descriptor() const noexcept = 0;
    [[nodiscard]] virtual const std::vector<std::uint32_t> &
    sample_rates() const noexcept = 0;
    [[nodiscard]] virtual std::optional<std::pair<double, double>>
    gain_range() const noexcept = 0;
    [[nodiscard]] virtual bool is_streaming() const noexcept = 0;
    [[nodiscard]] virtual bool has_active_worker() const noexcept = 0;
    [[nodiscard]] virtual bool input_exhausted() const noexcept = 0;
    [[nodiscard]] virtual std::string runtime_error() const = 0;

    virtual bool configure(const SourceSettings &settings,
                           std::string &error) = 0;
    virtual bool start(IqSourceCallbacks callbacks, std::string &error) = 0;
    virtual void stop() noexcept = 0;
    virtual bool retune(std::uint64_t frequency_hz, double correction_ppm,
                        std::string &error) = 0;
    virtual bool set_gain(const SourceSettings &settings,
                          std::string &error) = 0;
    virtual bool set_bias_tee(bool enabled, std::string &error) = 0;

  protected:
    IqSource() = default;
};

[[nodiscard]] EnumerationResult enumerate_iq_sources(bool include_soapy_airspy);
[[nodiscard]] std::unique_ptr<IqSource>
open_iq_source(const DeviceDescriptor &descriptor, std::string &error);
[[nodiscard]] std::unique_ptr<IqSource>
open_file_iq_source(const std::filesystem::path &path, SourceSettings &settings,
                    IqPlaybackPacing pacing, std::string &error);

} // namespace airspy_tv
