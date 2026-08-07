#pragma once

#include "../byte_rate_tracker.hpp"
#include "../pipeline_load_monitor.hpp"
#include "../receiver_session.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/epg.hpp"
#include "airspy_tv/mpv_player.hpp"
#include "airspy_tv/sdr.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
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

struct AppState {
    MpvPlayer player;
    ReceiverSession session;
    EnumerationResult enumeration;
    SourceSettings settings;
    std::size_t selected_device{};
    bool show_soapy_airspy{};
    std::string status{"Ready"};
    std::string recording_path{"capture.cs16"};
    std::string ts_recording_path{"capture.ts"};
    ByteRateTracker iq_write_rate;
    ByteRateTracker ts_write_rate;
    std::size_t selected_colormap{};
    float display_floor_dbfs{default_display_floor_dbfs};
    float display_ceiling_dbfs{default_display_ceiling_dbfs};
    bool fft_smoothing{true};
    int fft_smoothing_speed{100};
    bool snr_smoothing{true};
    int snr_smoothing_speed{20};
    SpectrumSnapshot spectrum;
    SignalSnapshot signal;
    PipelineSnapshot pipeline;
    PipelineLoadMonitor pipeline_load_monitor;
    PipelineLoadState pipeline_load{PipelineLoadState::measuring};
    std::vector<TransportService> services;
    std::optional<std::uint16_t> selected_service_id;
    struct DvbTUiState {
        dvbt::ReceiverParameters parameters;
        dvbt::SignalAnalysisSnapshot signal;
        dvbt::StreamDecoderStats decoder;
    } dvbt;
    WaterfallDisplay waterfall;
    SDL_Window *window{};
    std::shared_ptr<FileDialogState> file_dialog{
        std::make_shared<FileDialogState>()};
    std::shared_ptr<FileDialogState> ts_file_dialog{
        std::make_shared<FileDialogState>()};
    std::shared_ptr<FileDialogState> iq_source_dialog{
        std::make_shared<FileDialogState>()};
    EpgModel epg;
    const DeviceDescriptor *last_source_descriptor{};
};

} // namespace airspy_tv::gui
