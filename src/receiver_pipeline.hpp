#pragma once

#include "airspy_tv/demodulator.hpp"
#include "airspy_tv/recorder.hpp"
#include "airspy_tv/sdr.hpp"
#include "airspy_tv/spectrum.hpp"
#include "airspy_tv/transport_pipeline.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace airspy_tv {

class ReceiverPipeline {
  public:
    explicit ReceiverPipeline(TransportPipeline &transport);
    ~ReceiverPipeline() noexcept;

    ReceiverPipeline(const ReceiverPipeline &) = delete;
    ReceiverPipeline &operator=(const ReceiverPipeline &) = delete;

    bool open(const DeviceDescriptor &descriptor, std::string &error);
    bool open_iq_file(const std::filesystem::path &path,
                      SourceSettings &settings, IqPlaybackPolicy policy,
                      std::string &error);
    void close() noexcept;
    bool start_stream(const SourceSettings &settings, std::string &error);
    void finish_stream();
    void stop_stream();
    bool retune(std::uint64_t frequency_hz, std::string &error);
    bool set_frequency_correction_ppm(double ppm, std::string &error);
    bool set_gain(const SourceSettings &settings, std::string &error);
    bool set_bias_tee(bool enabled, std::string &error);
    void set_display_smoothing(bool fft_enabled, int fft_speed,
                               bool signal_enabled, int signal_speed);
    void set_display_analysis_enabled(bool enabled) noexcept;
    void set_demodulator_signal_smoothing(bool enabled, int speed);
    void set_demodulator(std::unique_ptr<Demodulator> demodulator);
    void set_channel_bandwidth(std::uint32_t bandwidth_hz);

    bool start_recording(const std::filesystem::path &path,
                         const SourceSettings &settings, std::string &error);
    void stop_recording();

    [[nodiscard]] bool is_open() const;
    [[nodiscard]] bool is_streaming() const;
    [[nodiscard]] bool input_exhausted() const;
    [[nodiscard]] bool is_recording() const;
    [[nodiscard]] const DeviceDescriptor *descriptor() const;
    [[nodiscard]] const std::vector<std::uint32_t> &sample_rates() const;
    [[nodiscard]] std::optional<std::pair<double, double>> gain_range() const;
    [[nodiscard]] RecordingStats recording_stats() const;
    [[nodiscard]] SpectrumSnapshot spectrum_snapshot() const;
    [[nodiscard]] SignalSnapshot signal_snapshot() const;
    [[nodiscard]] PipelineSnapshot pipeline_snapshot() const;
    [[nodiscard]] InputTimelineSnapshot input_timeline_snapshot() const;
    [[nodiscard]] std::string runtime_error() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
