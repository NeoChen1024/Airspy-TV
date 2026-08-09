#pragma once

#include "../byte_rate_tracker.hpp"
#include "../decode_run_reporter.hpp"
#include "../pipeline_load_monitor.hpp"
#include "../receiver_controller.hpp"
#include "../receiver_session.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/mpv_player.hpp"
#include "airspy_tv/sdr.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

struct SDL_Window;

namespace airspy_tv::gui {

inline constexpr float signal_meter_floor_dbfs = -140.0F;
inline constexpr float signal_meter_ceiling_dbfs = 0.0F;
inline constexpr float default_display_floor_dbfs = -100.0F;
inline constexpr float default_display_ceiling_dbfs = -20.0F;
inline constexpr int waterfall_texture_width = 512;
inline constexpr int waterfall_texture_height = 192;
inline constexpr std::size_t waterfall_cell_count =
    static_cast<std::size_t>(waterfall_texture_width) *
    static_cast<std::size_t>(waterfall_texture_height);

struct FileDialogState {
    std::mutex mutex;
    std::optional<std::string> selected_path;
    std::optional<std::string> error;
    std::string default_location;
    bool open{};
};

struct WaterfallDisplay {
    unsigned int texture{};
    std::vector<float> history =
        std::vector<float>(waterfall_cell_count, signal_meter_floor_dbfs);
    std::vector<std::uint8_t> rgba =
        std::vector<std::uint8_t>(waterfall_cell_count * 4);
    std::uint64_t sequence{};
    std::size_t colormap_index{};
    float floor_dbfs{default_display_floor_dbfs};
    float ceiling_dbfs{default_display_ceiling_dbfs};

    void destroy();
};

struct SourceUiState {
    EnumerationResult enumeration;
    SourceSettings settings;
    std::size_t selected_device{};
    bool show_soapy_airspy{};
    bool observed_input_exhausted{};
    std::shared_ptr<FileDialogState> iq_source_dialog{
        std::make_shared<FileDialogState>()};
};

struct OutputUiState {
    MpvPlayer player;
    std::string recording_path{"capture.cs16"};
    std::string ts_recording_path{"capture.ts"};
    std::string rtp_host{"127.0.0.1"};
    std::uint16_t rtp_port{5004};
    ByteRateTracker iq_write_rate;
    ByteRateTracker ts_write_rate;
    ByteRateTracker rtp_write_rate;
    std::optional<std::uint16_t> selected_service_id;
    std::shared_ptr<FileDialogState> file_dialog{
        std::make_shared<FileDialogState>()};
    std::shared_ptr<FileDialogState> ts_file_dialog{
        std::make_shared<FileDialogState>()};
};

struct DisplayUiState {
    std::size_t selected_colormap{};
    float display_floor_dbfs{default_display_floor_dbfs};
    float display_ceiling_dbfs{default_display_ceiling_dbfs};
    bool fft_smoothing{true};
    int fft_smoothing_speed{100};
    bool snr_smoothing{true};
    int snr_smoothing_speed{20};
    PipelineLoadMonitor pipeline_load_monitor;
    WaterfallDisplay waterfall;
};

struct StandardUiState {
    struct DvbTUiState {
        dvbt::ReceiverParameters parameters;
    } dvbt;
};

struct AppFrameSnapshot {
    ReceiveStandard standard{ReceiveStandard::DvbT};
    std::uint32_t channel_bandwidth_hz{};
    bool source_open{};
    bool source_streaming{};
    bool input_exhausted{};
    bool iq_recording{};
    std::optional<DeviceDescriptor> descriptor;
    std::vector<std::uint32_t> sample_rates;
    std::optional<std::pair<double, double>> gain_range;
    SpectrumSnapshot spectrum;
    SignalSnapshot signal;
    PipelineSnapshot pipeline;
    PipelineLoadState pipeline_load{PipelineLoadState::measuring};
    DvbTSessionSnapshot dvbt;
    std::vector<TransportService> services;
    EpgSnapshot epg;
    RecordingStats iq_recording_stats;
    TransportRecordingStats ts_recording_stats;
    RtpUdpStats rtp_stats;
    PlaybackTelemetry playback;
};

struct UiShellState {
    std::string status{"Ready"};
    SDL_Window *window{};
};

struct ReportingUiState {
    ReportingUiState(ReceiverSession &session, SourceSettings &settings,
                     dvbt::ReceiverParameters &parameters,
                     std::optional<std::filesystem::path> directory)
        : decode_report(std::move(directory), "gui"),
          controller(session, decode_report, settings, parameters) {}

    DecodeRunReporter decode_report;
    ReceiverController controller;
};

struct AppState {
    explicit AppState(
        std::optional<std::filesystem::path> report_directory = std::nullopt)
        : reporting(session, source.settings, standard.dvbt.parameters,
                    std::move(report_directory)) {}

    ReceiverSession session;
    SourceUiState source;
    DisplayUiState display;
    StandardUiState standard;
    OutputUiState output;
    UiShellState ui;
    ReportingUiState reporting;
    AppFrameSnapshot frame;
};

} // namespace airspy_tv::gui
