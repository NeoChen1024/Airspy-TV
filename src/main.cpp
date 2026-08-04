#include "airspy_tv/sdr.hpp"

#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/epg.hpp"
#include "airspy_tv/iq_file.hpp"
#include "airspy_tv/mpv_player.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_dialog.h>
#include <SDL3/SDL_opengl.h>
#include <fontconfig/fontconfig.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>
#include <imgui_stdlib.h>
#include <tinycolormap.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

using airspy_tv::AirspyGainMode;
using airspy_tv::DeviceDescriptor;
using airspy_tv::EnumerationResult;
using airspy_tv::EpgEvent;
using airspy_tv::EpgModel;
using airspy_tv::EpgSnapshot;
using airspy_tv::IqFileInfo;
using airspy_tv::MpvPlayer;
using airspy_tv::ReceiveStandard;
using airspy_tv::SdrBackend;
using airspy_tv::SdrDevice;
using airspy_tv::SourceSettings;
using airspy_tv::SpectrumSnapshot;
using airspy_tv::TransportService;
using airspy_tv::dvbt::CodeRate;
using airspy_tv::dvbt::Constellation;
using airspy_tv::dvbt::GuardInterval;
using airspy_tv::dvbt::ReceiverParameters;
using airspy_tv::dvbt::SignalAnalysisSnapshot;
using airspy_tv::dvbt::StreamDecoder;
using airspy_tv::dvbt::StreamDecoderStats;
using airspy_tv::dvbt::TransmissionMode;

constexpr ImVec4 accent{0.12F, 0.58F, 0.92F, 1.0F};
constexpr float panel_width = 410.0F;
constexpr float signal_meter_floor_dbfs = -140.0F;
constexpr float signal_meter_ceiling_dbfs = 0.0F;
constexpr float default_display_floor_dbfs = -100.0F;
constexpr float default_display_ceiling_dbfs = -20.0F;
constexpr float minimum_display_range_db = 1.0F;
constexpr float spectrum_label_margin = 36.0F;
constexpr int waterfall_texture_width = 512;
constexpr int waterfall_texture_height = 192;
constexpr std::size_t waterfall_cell_count =
    static_cast<std::size_t>(waterfall_texture_width) *
    static_cast<std::size_t>(waterfall_texture_height);
constexpr std::uint64_t max_display_frequency = 999'999'999'999ULL;
constexpr std::array<std::uint64_t, 12> digit_steps{
    100'000'000'000ULL,
    10'000'000'000ULL,
    1'000'000'000ULL,
    100'000'000ULL,
    10'000'000ULL,
    1'000'000ULL,
    100'000ULL,
    10'000ULL,
    1'000ULL,
    100ULL,
    10ULL,
    1ULL,
};
constexpr std::array<SDL_DialogFileFilter, 2> recording_filters{{
    {"Raw interleaved I/Q", "cs16;iq"},
    {"All files", "*"},
}};
constexpr std::array<SDL_DialogFileFilter, 2> transport_stream_filters{{
    {"MPEG transport stream", "ts;m2ts"},
    {"All files", "*"},
}};
constexpr std::array<SDL_DialogFileFilter, 3> iq_source_filters{{
    {"I/Q metadata or raw INT16_IQ", "json;cs16;iq"},
    {"I/Q metadata", "json"},
    {"All files", "*"},
}};

struct ColormapOption {
    const char *name;
    tinycolormap::ColormapType type;
};

constexpr std::array colormap_options{
    ColormapOption{"Cubehelix", tinycolormap::ColormapType::Cubehelix},
    ColormapOption{"Viridis", tinycolormap::ColormapType::Viridis},
    ColormapOption{"Cividis", tinycolormap::ColormapType::Cividis},
    ColormapOption{"Magma", tinycolormap::ColormapType::Magma},
    ColormapOption{"Inferno", tinycolormap::ColormapType::Inferno},
    ColormapOption{"Plasma", tinycolormap::ColormapType::Plasma},
    ColormapOption{"Turbo", tinycolormap::ColormapType::Turbo},
    ColormapOption{"Parula", tinycolormap::ColormapType::Parula},
    ColormapOption{"Heat", tinycolormap::ColormapType::Heat},
    ColormapOption{"Hot", tinycolormap::ColormapType::Hot},
    ColormapOption{"Gray", tinycolormap::ColormapType::Gray},
    ColormapOption{"Jet", tinycolormap::ColormapType::Jet},
    ColormapOption{"HSV", tinycolormap::ColormapType::HSV},
    ColormapOption{"GitHub", tinycolormap::ColormapType::Github},
};

struct FileDialogState {
    std::mutex mutex;
    std::optional<std::string> selected_path;
    std::optional<std::string> error;
    std::string default_location;
    bool open{};
};

struct WaterfallDisplay {
    GLuint texture{};
    std::vector<float> history =
        std::vector<float>(waterfall_cell_count, signal_meter_floor_dbfs);
    std::vector<std::uint8_t> rgba =
        std::vector<std::uint8_t>(waterfall_cell_count * 4);
    std::uint64_t sequence{};
    std::size_t colormap_index{};
    float floor_dbfs{default_display_floor_dbfs};
    float ceiling_dbfs{default_display_ceiling_dbfs};

    void destroy() {
        if (texture != 0) {
            glDeleteTextures(1, &texture);
            texture = 0;
        }
    }
};

struct AppState {
    MpvPlayer player;
    SdrDevice receiver;
    EnumerationResult enumeration;
    SourceSettings settings;
    std::size_t selected_device{};
    bool show_soapy_airspy{};
    std::string status{"Ready"};
    std::string recording_path{"capture.cs16"};
    std::string ts_recording_path{"capture.ts"};
    std::size_t selected_colormap{};
    float display_floor_dbfs{default_display_floor_dbfs};
    float display_ceiling_dbfs{default_display_ceiling_dbfs};
    bool fft_smoothing{true};
    int fft_smoothing_speed{100};
    bool snr_smoothing{true};
    int snr_smoothing_speed{20};
    SpectrumSnapshot spectrum;
    SignalAnalysisSnapshot signal_analysis;
    StreamDecoderStats decoder;
    std::vector<TransportService> services;
    std::optional<std::uint16_t> selected_service_id;
    ReceiverParameters dvbt_parameters;
    ReceiveStandard standard{ReceiveStandard::DvbT};
    // The concrete DVB-T demodulator is injected into the receiver; the raw
    // pointer stays valid for the DVB-T-specific GUI panels.
    std::unique_ptr<StreamDecoder> demodulator;
    StreamDecoder *dvbt_demod{};
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

void SDLCALL save_file_callback(void *userdata, const char *const *filelist,
                                const int selected_filter) {
    static_cast<void>(selected_filter);
    const std::unique_ptr<std::shared_ptr<FileDialogState>> callback_state(
        static_cast<std::shared_ptr<FileDialogState> *>(userdata));
    const std::shared_ptr<FileDialogState> &state = *callback_state;
    const std::scoped_lock lock(state->mutex);
    state->open = false;
    if (filelist == nullptr) {
        state->error = SDL_GetError();
    } else if (filelist[0] != nullptr) {
        state->selected_path = filelist[0];
    }
}

void apply_dark_theme() {
    ImGui::StyleColorsDark();
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 0.0F;
    style.ChildRounding = 5.0F;
    style.FrameRounding = 4.0F;
    style.PopupRounding = 4.0F;
    style.ScrollbarRounding = 4.0F;
    style.WindowPadding = ImVec2(10.0F, 10.0F);
    style.FramePadding = ImVec2(8.0F, 5.0F);
    style.ItemSpacing = ImVec2(8.0F, 7.0F);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.035F, 0.043F, 0.055F, 1.0F);
    style.Colors[ImGuiCol_ChildBg] = ImVec4(0.055F, 0.065F, 0.080F, 1.0F);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.09F, 0.11F, 0.14F, 1.0F);
    style.Colors[ImGuiCol_Header] = ImVec4(0.10F, 0.28F, 0.43F, 1.0F);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.12F, 0.40F, 0.62F, 1.0F);
    style.Colors[ImGuiCol_Button] = ImVec4(0.10F, 0.32F, 0.50F, 1.0F);
    style.Colors[ImGuiCol_ButtonHovered] = accent;
    style.Colors[ImGuiCol_CheckMark] = accent;
    style.Colors[ImGuiCol_SliderGrab] = accent;
}

[[nodiscard]] bool load_system_monospace_font(ImGuiIO &io, std::string &error) {
    if (FcInit() == FcFalse) {
        error = "Fontconfig initialization failed";
        return false;
    }

    FcPattern *pattern = FcPatternCreate();
    if (pattern == nullptr) {
        error = "Fontconfig could not create a font query";
        return false;
    }

    const auto *family = reinterpret_cast<const FcChar8 *>("monospace");
    if (FcPatternAddString(pattern, FC_FAMILY, family) == FcFalse ||
        FcConfigSubstitute(nullptr, pattern, FcMatchPattern) == FcFalse) {
        FcPatternDestroy(pattern);
        error = "Fontconfig could not prepare the system monospace query";
        return false;
    }
    FcDefaultSubstitute(pattern);

    FcResult result = FcResultNoMatch;
    FcFontSet *matches = FcFontSort(nullptr, pattern, FcTrue, nullptr, &result);
    FcPatternDestroy(pattern);
    if (matches == nullptr || result != FcResultMatch) {
        if (matches != nullptr) {
            FcFontSetDestroy(matches);
        }
        error = "Fontconfig could not resolve the system monospace font";
        return false;
    }

    // Dear ImGui does not consult the platform font fallback mechanism. Merge
    // the first few fonts from Fontconfig's coverage-trimmed system fallback
    // order to provide the behavior desktop toolkits normally supply. The
    // query intentionally carries no language or character-set requirement.
    constexpr int maximum_font_sources = 6;
    int loaded_sources = 0;
    for (int match_index = 0;
         match_index < matches->nfont && loaded_sources < maximum_font_sources;
         ++match_index) {
        FcChar8 *font_file = nullptr;
        FcPattern *match = matches->fonts[match_index];
        if (FcPatternGetString(match, FC_FILE, 0, &font_file) !=
                FcResultMatch ||
            font_file == nullptr) {
            continue;
        }

        int font_index = 0;
        static_cast<void>(FcPatternGetInteger(match, FC_INDEX, 0, &font_index));
        const std::string font_path{reinterpret_cast<const char *>(font_file)};

        ImFontConfig font_config;
        font_config.FontNo = font_index;
        font_config.MergeMode = loaded_sources != 0;
        if (io.Fonts->AddFontFromFileTTF(font_path.c_str(), 13.0F,
                                         &font_config) != nullptr) {
            ++loaded_sources;
        }
    }
    FcFontSetDestroy(matches);
    if (loaded_sources == 0) {
        error = "Dear ImGui could not load any system monospace fonts";
        return false;
    }
    return true;
}

void refresh_devices(AppState &state) {
    state.enumeration = SdrDevice::enumerate(state.show_soapy_airspy);
    if (state.enumeration.devices.empty()) {
        state.selected_device = 0;
    } else {
        state.selected_device = std::min(state.selected_device,
                                         state.enumeration.devices.size() - 1);
    }
    if (state.enumeration.devices.empty()) {
        state.status = "No SDR devices found";
    } else {
        state.status = std::format("Found {} SDR device(s)",
                                   state.enumeration.devices.size());
    }
}

std::string format_frequency_step(const std::uint64_t step_hz) {
    if (step_hz >= 1'000'000'000ULL) {
        return std::format("{} GHz", step_hz / 1'000'000'000ULL);
    }
    if (step_hz >= 1'000'000ULL) {
        return std::format("{} MHz", step_hz / 1'000'000ULL);
    }
    if (step_hz >= 1'000ULL) {
        return std::format("{} kHz", step_hz / 1'000ULL);
    }
    return std::format("{} Hz", step_hz);
}

std::optional<std::uint64_t>
draw_frequency_control(const char *id, const std::uint64_t frequency_hz) {
    const std::string digits =
        std::format("{:012}", std::min(frequency_hz, max_display_frequency));
    const float font_size = ImGui::GetFontSize() * 2.0F;
    const float digit_width = (ImGui::CalcTextSize("0").x * 2.0F) + 6.0F;
    const float separator_width = (ImGui::CalcTextSize(".").x * 2.0F) + 2.0F;
    const float height = font_size + 10.0F;
    std::optional<std::uint64_t> requested;
    bool significant_digit_seen = false;

    ImGui::PushID(id);
    ImGui::BeginGroup();
    for (std::size_t index = 0; index < digits.size(); ++index) {
        if (index != 0) {
            ImGui::SameLine(0.0F, 0.0F);
        }
        if (index != 0 && index % 3 == 0) {
            ImGui::PushID(static_cast<int>(index));
            ImGui::InvisibleButton("separator",
                                   ImVec2(separator_width, height));
            const ImVec2 separator_min = ImGui::GetItemRectMin();
            ImGui::GetWindowDrawList()->AddText(
                ImGui::GetFont(), font_size,
                ImVec2(separator_min.x + 1.0F, separator_min.y + 4.0F),
                ImGui::GetColorU32(ImGuiCol_TextDisabled), ".");
            ImGui::PopID();
            ImGui::SameLine(0.0F, 0.0F);
        }

        ImGui::PushID(static_cast<int>(index));
        ImGui::InvisibleButton("digit", ImVec2(digit_width, height));
        const ImVec2 item_min = ImGui::GetItemRectMin();
        const ImVec2 item_max = ImGui::GetItemRectMax();
        const bool hovered = ImGui::IsItemHovered();
        if (hovered) {
            ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
            ImGui::GetWindowDrawList()->AddRectFilled(
                item_min, item_max, ImGui::GetColorU32(ImGuiCol_HeaderHovered),
                3.0F);
        }

        int direction = 0;
        int repeat = 1;
        if (hovered && ImGui::GetIO().MouseWheel != 0.0F) {
            const float wheel = ImGui::GetIO().MouseWheel;
            direction = wheel > 0.0F ? 1 : -1;
            repeat =
                std::max(1, static_cast<int>(std::lround(std::abs(wheel))));
        }
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            const float middle = (item_min.y + item_max.y) * 0.5F;
            direction = ImGui::GetMousePos().y < middle ? 1 : -1;
        }

        if (direction != 0) {
            const std::uint64_t step = digit_steps[index];
            const std::uint64_t delta =
                step * static_cast<std::uint64_t>(repeat);
            if (direction > 0) {
                requested = frequency_hz > max_display_frequency - delta
                                ? max_display_frequency
                                : frequency_hz + delta;
            } else {
                requested = frequency_hz > delta ? frequency_hz - delta : 0;
            }
        }

        significant_digit_seen |= digits[index] != '0';
        const ImU32 colour = ImGui::GetColorU32(
            significant_digit_seen ? ImGuiCol_Text : ImGuiCol_TextDisabled);
        const std::array<char, 2> digit_text{digits[index], '\0'};
        ImGui::GetWindowDrawList()->AddText(
            ImGui::GetFont(), font_size,
            ImVec2(item_min.x + 3.0F, item_min.y + 4.0F), colour,
            digit_text.data());
        if (hovered) {
            ImGui::SetTooltip(
                "Wheel/click: +/- %s",
                format_frequency_step(digit_steps[index]).c_str());
        }
        ImGui::PopID();
    }
    ImGui::EndGroup();
    ImGui::PopID();
    return requested;
}

void request_center_frequency(AppState &state,
                              const std::uint64_t frequency_hz) {
    if (!state.receiver.is_open()) {
        state.settings.center_frequency_hz = frequency_hz;
        state.status = "Center frequency selected";
        return;
    }

    std::string error;
    if (state.receiver.set_center_frequency(frequency_hz, error)) {
        state.settings.center_frequency_hz = frequency_hz;
        state.status = "Center frequency applied";
    } else {
        state.status = error;
    }
}

float spectrum_column_peak(const std::span<const float> bins,
                           const std::size_t column,
                           const std::size_t column_count) {
    const std::size_t begin = (column * bins.size()) / column_count;
    const std::size_t end =
        std::max(begin + 1, ((column + 1) * bins.size()) / column_count);
    return *std::ranges::max_element(bins.subspan(begin, end - begin));
}

void draw_spectrum(const ImVec2 size, const SpectrumSnapshot &spectrum,
                   const float floor_dbfs, const float ceiling_dbfs) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    ImGui::InvisibleButton("spectrum-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(4, 9, 15, 255), 4.0F);

    const ImVec2 plot_origin{origin.x + spectrum_label_margin, origin.y};
    const float plot_width = std::max(2.0F, extent.x - plot_origin.x);
    constexpr int grid_divisions = 4;
    for (int grid = 0; grid <= grid_divisions; ++grid) {
        const float fraction =
            static_cast<float>(grid) / static_cast<float>(grid_divisions);
        const float level =
            ceiling_dbfs - (fraction * (ceiling_dbfs - floor_dbfs));
        const float y = origin.y + (canvas_size.y * fraction);
        draw->AddLine(ImVec2(origin.x, y), ImVec2(extent.x, y),
                      IM_COL32(38, 51, 65, 160));
        draw->AddText(ImVec2(origin.x + 3.0F,
                             y + (grid == grid_divisions ? -14.0F : 2.0F)),
                      IM_COL32(118, 133, 148, 220),
                      std::format("{:.0f}", level).c_str());
    }
    for (int index = 0; index <= 4; ++index) {
        const float x =
            plot_origin.x + (plot_width * static_cast<float>(index) / 4.0F);
        draw->AddLine(ImVec2(x, origin.y), ImVec2(x, extent.y),
                      IM_COL32(38, 51, 65, 160));
    }

    if (spectrum.valid) {
        const auto column_count =
            static_cast<std::size_t>(std::clamp(plot_width, 2.0F, 768.0F));
        std::vector<ImVec2> points;
        points.reserve(column_count);
        for (std::size_t column = 0; column < column_count; ++column) {
            const float level =
                spectrum_column_peak(spectrum.bins_dbfs, column, column_count);
            const float normalized = std::clamp(
                (level - floor_dbfs) / (ceiling_dbfs - floor_dbfs), 0.0F, 1.0F);
            points.emplace_back(plot_origin.x +
                                    (plot_width * static_cast<float>(column) /
                                     static_cast<float>(column_count - 1)),
                                extent.y - (normalized * canvas_size.y));
        }
        draw->AddPolyline(points.data(), static_cast<int>(points.size()),
                          IM_COL32(45, 205, 235, 255), 0, 1.4F);
    } else {
        draw->AddText(ImVec2(plot_origin.x + 8.0F, origin.y + 28.0F),
                      IM_COL32(150, 167, 184, 255), "Waiting for I/Q data...");
    }
    draw->AddText(ImVec2(plot_origin.x + 8.0F, origin.y + 6.0F),
                  IM_COL32(150, 167, 184, 255), "Spectrum dBFS/bin");
}

void ensure_waterfall_texture(WaterfallDisplay &waterfall) {
    if (waterfall.texture != 0) {
        return;
    }
    glGenTextures(1, &waterfall.texture);
    glBindTexture(GL_TEXTURE_2D, waterfall.texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, waterfall_texture_width,
                 waterfall_texture_height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 waterfall.rgba.data());
}

void recolour_waterfall(WaterfallDisplay &waterfall,
                        const tinycolormap::ColormapType colormap,
                        const float floor_dbfs, const float ceiling_dbfs) {
    constexpr std::size_t lut_size = 256;
    std::array<std::array<std::uint8_t, 4>, lut_size> colour_lut{};
    for (std::size_t index = 0; index < colour_lut.size(); ++index) {
        const tinycolormap::Color colour = tinycolormap::GetColor(
            static_cast<double>(index) /
                static_cast<double>(colour_lut.size() - 1),
            colormap);
        colour_lut[index] = {
            static_cast<std::uint8_t>(
                std::lround(std::clamp(colour.r(), 0.0, 1.0) * 255.0)),
            static_cast<std::uint8_t>(
                std::lround(std::clamp(colour.g(), 0.0, 1.0) * 255.0)),
            static_cast<std::uint8_t>(
                std::lround(std::clamp(colour.b(), 0.0, 1.0) * 255.0)),
            255,
        };
    }

    for (std::size_t index = 0; index < waterfall.history.size(); ++index) {
        const float normalized =
            std::clamp((waterfall.history[index] - floor_dbfs) /
                           (ceiling_dbfs - floor_dbfs),
                       0.0F, 1.0F);
        const auto lut_index = static_cast<std::size_t>(std::lround(
            normalized * static_cast<float>(colour_lut.size() - 1)));
        const std::size_t pixel = index * 4;
        std::ranges::copy(colour_lut[lut_index],
                          waterfall.rgba.begin() +
                              static_cast<std::ptrdiff_t>(pixel));
    }

    ensure_waterfall_texture(waterfall);
    glBindTexture(GL_TEXTURE_2D, waterfall.texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, waterfall_texture_width,
                    waterfall_texture_height, GL_RGBA, GL_UNSIGNED_BYTE,
                    waterfall.rgba.data());
}

void update_waterfall(WaterfallDisplay &waterfall,
                      const SpectrumSnapshot &spectrum,
                      const std::size_t colormap_index, const float floor_dbfs,
                      const float ceiling_dbfs) {
    IM_ASSERT(waterfall.history.size() == waterfall_cell_count);
    IM_ASSERT(waterfall.rgba.size() == waterfall_cell_count * 4);
    bool changed = false;
    if (spectrum.valid && spectrum.sequence != waterfall.sequence) {
        constexpr std::size_t row_size = waterfall_texture_width;
        std::memmove(waterfall.history.data() + row_size,
                     waterfall.history.data(),
                     (waterfall.history.size() - row_size) * sizeof(float));
        for (std::size_t column = 0; column < row_size; ++column) {
            waterfall.history[column] = spectrum_column_peak(
                spectrum.waterfall_bins_dbfs, column, row_size);
        }
        waterfall.sequence = spectrum.sequence;
        changed = true;
    }
    if (waterfall.colormap_index != colormap_index) {
        waterfall.colormap_index = colormap_index;
        changed = true;
    }
    if (waterfall.floor_dbfs != floor_dbfs ||
        waterfall.ceiling_dbfs != ceiling_dbfs) {
        waterfall.floor_dbfs = floor_dbfs;
        waterfall.ceiling_dbfs = ceiling_dbfs;
        changed = true;
    }
    ensure_waterfall_texture(waterfall);
    if (changed) {
        recolour_waterfall(waterfall, colormap_options[colormap_index].type,
                           floor_dbfs, ceiling_dbfs);
    }
}

void draw_waterfall(const ImVec2 size, WaterfallDisplay &waterfall,
                    const SpectrumSnapshot &spectrum,
                    const std::size_t colormap_index, const float floor_dbfs,
                    const float ceiling_dbfs) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    update_waterfall(waterfall, spectrum, colormap_index, floor_dbfs,
                     ceiling_dbfs);
    ImGui::InvisibleButton("waterfall-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    const ImVec2 plot_origin{origin.x + spectrum_label_margin, origin.y};
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(4, 9, 15, 255));
    draw->AddImage(ImTextureRef(static_cast<ImTextureID>(waterfall.texture)),
                   plot_origin, extent);
    draw->AddText(ImVec2(plot_origin.x + 8.0F, origin.y + 6.0F), IM_COL32_WHITE,
                  "Waterfall");
}

void draw_constellation(const ImVec2 size,
                        const SignalAnalysisSnapshot &analysis) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    ImGui::InvisibleButton("constellation-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    const ImVec2 center{(origin.x + extent.x) * 0.5F,
                        (origin.y + extent.y) * 0.5F};
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(4, 9, 15, 255), 4.0F);
    draw->AddLine(ImVec2(origin.x, center.y), ImVec2(extent.x, center.y),
                  IM_COL32(50, 65, 80, 255));
    draw->AddLine(ImVec2(center.x, origin.y), ImVec2(center.x, extent.y),
                  IM_COL32(50, 65, 80, 255));
    if (analysis.locked) {
        constexpr float constellation_extent = 1.55F;
        const float scale = std::min(canvas_size.x, canvas_size.y) * 0.46F /
                            constellation_extent;
        for (std::size_t index = 0; index < analysis.point_count; ++index) {
            const auto point = analysis.points[index];
            const ImVec2 position{center.x + (point.real() * scale),
                                  center.y - (point.imag() * scale)};
            if (position.x >= origin.x && position.x <= extent.x &&
                position.y >= origin.y && position.y <= extent.y) {
                draw->AddCircleFilled(position, 1.5F,
                                      IM_COL32(70, 220, 255, 150));
            }
        }
    }
    draw->AddText(ImVec2(origin.x + 8.0F, origin.y + 6.0F),
                  IM_COL32(150, 167, 184, 255),
                  analysis.locked ? "Equalized DVB-T carriers"
                                  : "Waiting for DVB-T OFDM lock");
}

void draw_metric(const char *label, const char *value, const float fraction,
                 const ImVec4 colour) {
    ImGui::TextUnformatted(label);
    ImGui::SameLine(115.0F);
    ImGui::TextColored(colour, "%s", value);
    ImGui::ProgressBar(fraction, ImVec2(-1.0F, 5.0F), "");
}

void draw_disabled_wrapped(const std::string_view text) {
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%.*s", static_cast<int>(text.size()), text.data());
    ImGui::PopStyleColor();
}

void draw_status_indicator(const char *label, const ImVec4 colour) {
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    const ImVec2 indicator_position{
        cursor.x + 5.0F, cursor.y + (ImGui::GetTextLineHeight() * 0.5F)};
    ImGui::GetWindowDrawList()->AddCircleFilled(
        indicator_position, 4.0F, ImGui::ColorConvertFloat4ToU32(colour));
    ImGui::Dummy(ImVec2(12.0F, ImGui::GetTextLineHeight()));
    ImGui::SameLine();
    ImGui::TextColored(colour, "%s", label);
}

void draw_bipolar_metric(const char *label, const char *value,
                         const float position, const ImVec4 colour) {
    ImGui::TextUnformatted(label);
    ImGui::SameLine(115.0F);
    ImGui::TextColored(colour, "%s", value);

    ImGui::PushID(label);
    ImGui::InvisibleButton("meter", ImVec2(-1.0F, 5.0F));
    ImGui::PopID();

    const ImVec2 bar_min = ImGui::GetItemRectMin();
    const ImVec2 bar_max = ImGui::GetItemRectMax();
    const float center_x = (bar_min.x + bar_max.x) * 0.5F;
    const float marker_x = bar_min.x + ((bar_max.x - bar_min.x) *
                                        std::clamp(position, 0.0F, 1.0F));
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(bar_min, bar_max, ImGui::GetColorU32(ImGuiCol_FrameBg),
                        ImGui::GetStyle().FrameRounding);
    if (marker_x != center_x) {
        draw->AddRectFilled(ImVec2(std::min(center_x, marker_x), bar_min.y),
                            ImVec2(std::max(center_x, marker_x), bar_max.y),
                            ImGui::ColorConvertFloat4ToU32(colour));
    }
    draw->AddLine(ImVec2(center_x, bar_min.y - 1.0F),
                  ImVec2(center_x, bar_max.y + 1.0F),
                  IM_COL32(225, 235, 245, 220));
    draw->AddLine(ImVec2(marker_x, bar_min.y), ImVec2(marker_x, bar_max.y),
                  ImGui::ColorConvertFloat4ToU32(colour), 2.0F);
}

std::string format_recording_duration(const std::uint64_t milliseconds) {
    const std::uint64_t total_seconds = milliseconds / 1000;
    const std::uint64_t hours = total_seconds / 3600;
    const std::uint64_t minutes = (total_seconds / 60) % 60;
    const std::uint64_t seconds = total_seconds % 60;
    const std::uint64_t tenths = (milliseconds % 1000) / 100;
    return std::format("{:02}:{:02}:{:02}.{}", hours, minutes, seconds, tenths);
}

bool file_dialog_is_open(const std::shared_ptr<FileDialogState> &dialog);

void draw_source_gain_controls(AppState &state) {
    const DeviceDescriptor *descriptor = state.receiver.descriptor();
    if (descriptor == nullptr || descriptor->backend == SdrBackend::File) {
        return;
    }

    ImGui::SeparatorText("SDR gain");
    if (descriptor->backend == SdrBackend::AirspyNative) {
        bool gain_changed = false;
        const int mode =
            state.settings.airspy_gain_mode == AirspyGainMode::Sensitivity ? 0
                                                                           : 1;
        if (ImGui::RadioButton("Sensitivity", mode == 0)) {
            state.settings.airspy_gain_mode = AirspyGainMode::Sensitivity;
            gain_changed = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Linearity", mode == 1)) {
            state.settings.airspy_gain_mode = AirspyGainMode::Linearity;
            gain_changed = true;
        }
        gain_changed |= ImGui::SliderInt("Profile gain",
                                         &state.settings.airspy_gain, 0, 21);
        if (gain_changed) {
            std::string error;
            state.status = state.receiver.set_gain(state.settings, error)
                               ? "Airspy gain applied"
                               : error;
        }
        if (ImGui::Checkbox("Bias-T", &state.settings.bias_tee)) {
            std::string error;
            if (state.receiver.set_bias_tee(state.settings.bias_tee, error)) {
                state.status = state.settings.bias_tee ? "Bias-T enabled"
                                                       : "Bias-T disabled";
            } else {
                state.status = error;
            }
        }
    } else if (const auto range = state.receiver.gain_range();
               range.has_value()) {
        if (ImGui::SliderScalar("Generic gain", ImGuiDataType_Double,
                                &state.settings.soapy_gain, &range->first,
                                &range->second, "%.1f")) {
            std::string error;
            state.status = state.receiver.set_gain(state.settings, error)
                               ? "Soapy gain applied"
                               : error;
        }
        draw_disabled_wrapped(
            "Soapy driver-defined gain; not comparable across devices.");
    }
}

void draw_source_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Source", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("source-panel");

    const bool source_open = state.receiver.is_open();
    std::uint32_t decoder_threads =
        static_cast<std::uint32_t>(state.dvbt_parameters.worker_threads);
    ImGui::BeginDisabled(source_open);
    ImGui::TextUnformatted("Decoder worker budget");
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::InputScalar("##decoder-worker-budget", ImGuiDataType_U32,
                           &decoder_threads)) {
        decoder_threads = std::min(decoder_threads, std::uint32_t{256});
        state.dvbt_parameters.worker_threads = decoder_threads;
        state.dvbt_demod->set_parameters(state.dvbt_parameters);
        state.receiver.set_channel_bandwidth(
            state.dvbt_parameters.channel_bandwidth_hz);
    }
    ImGui::EndDisabled();
    ImGui::TextDisabled("0 = Auto (%zu logical CPUs)",
                        airspy_tv::dvbt::default_viterbi_worker_count());

    std::optional<std::string> selected_iq_source;
    {
        const std::scoped_lock lock(state.iq_source_dialog->mutex);
        if (state.iq_source_dialog->selected_path.has_value()) {
            selected_iq_source =
                std::move(*state.iq_source_dialog->selected_path);
            state.iq_source_dialog->selected_path.reset();
        }
        if (state.iq_source_dialog->error.has_value()) {
            state.status = "File dialog: " + *state.iq_source_dialog->error;
            state.iq_source_dialog->error.reset();
        }
    }
    if (selected_iq_source.has_value()) {
        std::string error;
        state.dvbt_demod->set_parameters(state.dvbt_parameters);
        state.receiver.set_channel_bandwidth(
            state.dvbt_parameters.channel_bandwidth_hz);
        if (state.receiver.open_iq_file(*selected_iq_source, state.settings,
                                        error) &&
            state.receiver.start_stream(state.settings, error)) {
            state.status =
                "Playing I/Q from " +
                std::filesystem::path(*selected_iq_source).filename().string();
        } else {
            state.receiver.close();
            state.status = error;
        }
    }

    const bool source_dialog_open = file_dialog_is_open(state.iq_source_dialog);
    ImGui::BeginDisabled(state.receiver.is_open() || source_dialog_open);
    const char *preview = state.enumeration.devices.empty()
                              ? "No devices"
                              : state.enumeration.devices[state.selected_device]
                                    .display_name.c_str();
    ImGui::TextUnformatted("Device");
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::BeginCombo("##source-device", preview)) {
        for (std::size_t index = 0; index < state.enumeration.devices.size();
             ++index) {
            const bool selected = index == state.selected_device;
            if (ImGui::Selectable(
                    state.enumeration.devices[index].display_name.c_str(),
                    selected)) {
                state.selected_device = index;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s", preview);
    }
    ImGui::Checkbox("Show Airspy through Soapy", &state.show_soapy_airspy);
    ImGui::SameLine();
    if (ImGui::SmallButton("Refresh")) {
        refresh_devices(state);
    }
    ImGui::EndDisabled();

    if (!state.receiver.is_open()) {
        ImGui::BeginDisabled(state.enumeration.devices.empty());
        if (ImGui::Button("Open device", ImVec2(-1.0F, 0.0F))) {
            std::string error;
            state.dvbt_demod->set_parameters(state.dvbt_parameters);
            state.receiver.set_channel_bandwidth(
                state.dvbt_parameters.channel_bandwidth_hz);
            const DeviceDescriptor &descriptor =
                state.enumeration.devices[state.selected_device];
            if (state.receiver.open(descriptor, error)) {
                if (!state.receiver.sample_rates().empty()) {
                    const auto nearest = std::ranges::min_element(
                        state.receiver.sample_rates(), {},
                        [target = state.settings.sample_rate_hz](
                            const std::uint32_t rate) {
                            return std::llabs(static_cast<long long>(rate) -
                                              target);
                        });
                    state.settings.sample_rate_hz = *nearest;
                }
                if (const auto range = state.receiver.gain_range();
                    range.has_value()) {
                    state.settings.soapy_gain = range->first;
                }
                if (state.receiver.start_stream(state.settings, error)) {
                    state.status = "Receiving from " + descriptor.display_name;
                } else {
                    state.receiver.close();
                    state.status = error;
                }
            } else {
                state.status = error;
            }
        }
        ImGui::EndDisabled();
        ImGui::BeginDisabled(source_dialog_open);
        if (ImGui::Button("Open I/Q file...", ImVec2(-1.0F, 0.0F))) {
            {
                const std::scoped_lock lock(state.iq_source_dialog->mutex);
                state.iq_source_dialog->open = true;
                state.iq_source_dialog->default_location = ".";
            }
            auto callback_state =
                std::make_unique<std::shared_ptr<FileDialogState>>(
                    state.iq_source_dialog);
            SDL_ShowOpenFileDialog(
                &save_file_callback, callback_state.release(), state.window,
                iq_source_filters.data(),
                static_cast<int>(iq_source_filters.size()), ".", false);
        }
        ImGui::EndDisabled();
        ImGui::TextUnformatted("Raw sample rate (Hz)");
        ImGui::SetNextItemWidth(-1.0F);
        ImGui::InputScalar("##raw-sample-rate", ImGuiDataType_U32,
                           &state.settings.sample_rate_hz);
        draw_disabled_wrapped(std::format(
            "JSON supplies metadata; raw INT16_IQ uses {:.3f} MSPS / {:.3f} "
            "MHz.",
            static_cast<double>(state.settings.sample_rate_hz) / 1e6,
            static_cast<double>(state.settings.center_frequency_hz) / 1e6));
    } else {
        const DeviceDescriptor *descriptor = state.receiver.descriptor();
        ImGui::TextColored(ImVec4(0.35F, 0.88F, 0.55F, 1.0F), "OPEN");
        ImGui::SameLine();
        ImGui::TextWrapped("%s", descriptor->display_name.c_str());
        if (!descriptor->serial.empty()) {
            draw_disabled_wrapped("Serial: " + descriptor->serial);
        }
        ImGui::BeginDisabled(state.receiver.is_recording());
        if (ImGui::Button(descriptor->backend == SdrBackend::File
                              ? "Close I/Q file"
                              : "Close device",
                          ImVec2(-1.0F, 0.0F))) {
            state.receiver.close();
            state.status = "Source closed";
        }
        ImGui::EndDisabled();

        if (descriptor->backend == SdrBackend::File) {
            ImGui::Text("Format       CS16 / INT16_IQ");
            ImGui::Text("Sample rate  %.3f MSPS",
                        static_cast<double>(state.settings.sample_rate_hz) /
                            1e6);
            ImGui::Text(
                "Center       %.6f MHz",
                static_cast<double>(state.settings.center_frequency_hz) / 1e6);
            if (!state.receiver.is_streaming() &&
                ImGui::Button("Replay from beginning", ImVec2(-1.0F, 0.0F))) {
                std::string error;
                state.status =
                    state.receiver.start_stream(state.settings, error)
                        ? "Replaying I/Q file"
                        : error;
            }
        } else {
            ImGui::BeginDisabled(state.receiver.is_recording());
            if (!state.receiver.sample_rates().empty()) {
                const std::string rate_preview = std::format(
                    "{:.3f} MSPS",
                    static_cast<double>(state.settings.sample_rate_hz) / 1e6);
                ImGui::TextUnformatted("Sample rate");
                ImGui::SetNextItemWidth(-1.0F);
                if (ImGui::BeginCombo("##sample-rate", rate_preview.c_str())) {
                    for (const std::uint32_t rate :
                         state.receiver.sample_rates()) {
                        const std::string label = std::format(
                            "{:.3f} MSPS", static_cast<double>(rate) / 1e6);
                        if (ImGui::Selectable(
                                label.c_str(),
                                rate == state.settings.sample_rate_hz)) {
                            state.settings.sample_rate_hz = rate;
                        }
                    }
                    ImGui::EndCombo();
                }
            }
            if (ImGui::Button("Apply sample rate", ImVec2(-1.0F, 0.0F))) {
                std::string error;
                state.status =
                    state.receiver.start_stream(state.settings, error)
                        ? "Sample rate applied"
                        : error;
            }
            ImGui::EndDisabled();
        }
    }

    draw_source_gain_controls(state);

    for (const std::string &warning : state.enumeration.warnings) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.64F, 0.25F, 1.0F));
        ImGui::TextWrapped("%s", warning.c_str());
        ImGui::PopStyleColor();
    }
    ImGui::PopID();
}

void draw_receiver_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Demodulator",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("receiver-panel");

    ImGui::TextUnformatted("Decode standard");
    ImGui::PushID("standard");
    ImGui::SetNextItemWidth(-1.0F);
    constexpr std::array standard_names{"DVB-T", "DVB-C", "DVB-T2", "DTMB",
                                        "ATSC"};
    if (ImGui::BeginCombo(
            "##value",
            standard_names[static_cast<std::size_t>(state.standard)])) {
        for (int index = 0; index < static_cast<int>(standard_names.size());
             ++index) {
            const bool implemented =
                static_cast<ReceiveStandard>(index) == ReceiveStandard::DvbT;
            if (!implemented) {
                ImGui::BeginDisabled(true);
            }
            if (ImGui::Selectable(
                    standard_names[static_cast<std::size_t>(index)],
                    state.standard == static_cast<ReceiveStandard>(index))) {
                state.standard = static_cast<ReceiveStandard>(index);
                // Future: rebuild the demodulator for the selected standard
                // (dvbc/dvbt2/dtmb/atsc modules) and re-inject it through
                // SdrDevice::set_demodulator; only DVB-T is implemented
                // today.
            }
            if (!implemented) {
                ImGui::EndDisabled();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::PopID();
    ImGui::Separator();

    bool parameters_changed = false;
    const auto draw_optional_combo =
        [&parameters_changed](const char *label,
                              const std::span<const char *const> names,
                              int &selected) {
            ImGui::TextUnformatted(label);
            ImGui::PushID(label);
            ImGui::SetNextItemWidth(-1.0F);
            if (ImGui::BeginCombo("##value",
                                  names[static_cast<std::size_t>(selected)])) {
                for (int index = 0; index < static_cast<int>(names.size());
                     ++index) {
                    if (ImGui::Selectable(
                            names[static_cast<std::size_t>(index)],
                            selected == index)) {
                        selected = index;
                        parameters_changed = true;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::PopID();
        };

    int bandwidth = 1;
    switch (state.dvbt_parameters.channel_bandwidth_hz) {
    case 5'000'000:
        bandwidth = 0;
        break;
    case 7'000'000:
        bandwidth = 2;
        break;
    case 8'000'000:
        bandwidth = 3;
        break;
    default:
        bandwidth = 1;
        break;
    }
    constexpr std::array bandwidth_names{"5 MHz", "6 MHz", "7 MHz", "8 MHz"};
    constexpr std::array<std::uint32_t, 4> bandwidth_values{
        5'000'000, 6'000'000, 7'000'000, 8'000'000};
    draw_optional_combo("Channel bandwidth", bandwidth_names, bandwidth);
    state.dvbt_parameters.channel_bandwidth_hz =
        bandwidth_values[static_cast<std::size_t>(bandwidth)];

    int selected_mode =
        !state.dvbt_parameters.mode.has_value()
            ? 0
            : (*state.dvbt_parameters.mode == TransmissionMode::k2 ? 1 : 2);
    constexpr std::array mode_names{"Auto", "2K", "8K"};
    draw_optional_combo("Transmission mode", mode_names, selected_mode);
    state.dvbt_parameters.mode =
        selected_mode == 0
            ? std::nullopt
            : std::optional{selected_mode == 1 ? TransmissionMode::k2
                                               : TransmissionMode::k8};

    int guard = 0;
    if (state.dvbt_parameters.guard_interval.has_value()) {
        switch (*state.dvbt_parameters.guard_interval) {
        case GuardInterval::gi_1_32:
            guard = 1;
            break;
        case GuardInterval::gi_1_16:
            guard = 2;
            break;
        case GuardInterval::gi_1_8:
            guard = 3;
            break;
        case GuardInterval::gi_1_4:
            guard = 4;
            break;
        }
    }
    constexpr std::array guard_names{"Auto", "1/32", "1/16", "1/8", "1/4"};
    draw_optional_combo("Guard interval", guard_names, guard);
    constexpr std::array guard_values{
        GuardInterval::gi_1_32, GuardInterval::gi_1_16, GuardInterval::gi_1_8,
        GuardInterval::gi_1_4};
    state.dvbt_parameters.guard_interval =
        guard == 0
            ? std::nullopt
            : std::optional{guard_values[static_cast<std::size_t>(guard - 1)]};

    int modulation = 0;
    if (state.dvbt_parameters.constellation.has_value()) {
        switch (*state.dvbt_parameters.constellation) {
        case Constellation::qpsk:
            modulation = 1;
            break;
        case Constellation::qam16:
            modulation = 2;
            break;
        case Constellation::qam64:
            modulation = 3;
            break;
        }
    }
    constexpr std::array modulation_names{"Auto (TPS)", "QPSK", "16-QAM",
                                          "64-QAM"};
    draw_optional_combo("Modulation", modulation_names, modulation);
    constexpr std::array modulation_values{
        Constellation::qpsk, Constellation::qam16, Constellation::qam64};
    state.dvbt_parameters.constellation =
        modulation == 0
            ? std::nullopt
            : std::optional{
                  modulation_values[static_cast<std::size_t>(modulation - 1)]};

    int code_rate = 0;
    if (state.dvbt_parameters.code_rate.has_value()) {
        switch (*state.dvbt_parameters.code_rate) {
        case CodeRate::rate_1_2:
            code_rate = 1;
            break;
        case CodeRate::rate_2_3:
            code_rate = 2;
            break;
        case CodeRate::rate_3_4:
            code_rate = 3;
            break;
        case CodeRate::rate_5_6:
            code_rate = 4;
            break;
        case CodeRate::rate_7_8:
            code_rate = 5;
            break;
        }
    }
    constexpr std::array code_rate_names{"Auto (TPS)", "1/2", "2/3",
                                         "3/4",        "5/6", "7/8"};
    draw_optional_combo("Code rate", code_rate_names, code_rate);
    constexpr std::array code_rate_values{
        CodeRate::rate_1_2, CodeRate::rate_2_3, CodeRate::rate_3_4,
        CodeRate::rate_5_6, CodeRate::rate_7_8};
    state.dvbt_parameters.code_rate =
        code_rate == 0
            ? std::nullopt
            : std::optional{
                  code_rate_values[static_cast<std::size_t>(code_rate - 1)]};

    if (parameters_changed) {
        state.dvbt_demod->set_parameters(state.dvbt_parameters);
        state.receiver.set_channel_bandwidth(
            state.dvbt_parameters.channel_bandwidth_hz);
        state.status = "DVB-T parameters updated; receiver reacquiring";
    }
    ImGui::PopID();
}

void consume_file_dialog_result(AppState &state,
                                const std::shared_ptr<FileDialogState> &dialog,
                                std::string &path,
                                const std::string_view description) {
    const std::scoped_lock lock(dialog->mutex);
    if (dialog->selected_path.has_value()) {
        path = std::move(*dialog->selected_path);
        dialog->selected_path.reset();
        state.status = std::string(description) + " path selected";
    }
    if (dialog->error.has_value()) {
        state.status = "File dialog: " + *dialog->error;
        dialog->error.reset();
    }
}

bool file_dialog_is_open(const std::shared_ptr<FileDialogState> &dialog) {
    const std::scoped_lock lock(dialog->mutex);
    return dialog->open;
}

void show_recording_file_dialog(
    AppState &state, const std::shared_ptr<FileDialogState> &dialog,
    const std::string &path,
    const std::span<const SDL_DialogFileFilter> filters) {
    {
        const std::scoped_lock lock(dialog->mutex);
        if (dialog->open) {
            return;
        }
        dialog->open = true;
        dialog->default_location = path;
    }

    auto callback_state =
        std::make_unique<std::shared_ptr<FileDialogState>>(dialog);
    SDL_ShowSaveFileDialog(&save_file_callback, callback_state.release(),
                           state.window, filters.data(),
                           static_cast<int>(filters.size()),
                           dialog->default_location.c_str());
}

void draw_recorder_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Raw I/Q Recorder",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("iq-recorder");
    consume_file_dialog_result(state, state.file_dialog, state.recording_path,
                               "I/Q recording");
    const bool dialog_open = file_dialog_is_open(state.file_dialog);
    draw_disabled_wrapped("Interleaved signed 16-bit little-endian I/Q (CS16)");
    ImGui::TextUnformatted("Output file");
    ImGui::BeginDisabled(state.receiver.is_recording() || dialog_open);
    ImGui::SetNextItemWidth(-92.0F);
    ImGui::InputText("##recording-output", &state.recording_path);
    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        show_recording_file_dialog(state, state.file_dialog,
                                   state.recording_path, recording_filters);
    }
    ImGui::EndDisabled();
    if (dialog_open) {
        ImGui::TextDisabled("Waiting for file selection...");
    }

    if (!state.receiver.is_recording()) {
        const bool file_source =
            state.receiver.descriptor() != nullptr &&
            state.receiver.descriptor()->backend == SdrBackend::File;
        ImGui::BeginDisabled(!state.receiver.is_open() || file_source ||
                             dialog_open || state.recording_path.empty());
        if (ImGui::Button("Start recording", ImVec2(-1.0F, 0.0F))) {
            std::string error;
            if (state.receiver.start_recording(
                    std::filesystem::path(state.recording_path), state.settings,
                    error)) {
                state.status = "Recording raw I/Q";
            } else {
                state.status = error;
            }
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop recording", ImVec2(-1.0F, 0.0F))) {
        state.receiver.stop_recording();
        state.status = "Recording stopped; JSON sidecar written";
    }

    const auto stats = state.receiver.recording_stats();
    ImGui::Text("Duration: %s",
                format_recording_duration(stats.elapsed_milliseconds).c_str());
    ImGui::Text("Written: %.2f MiB",
                static_cast<double>(stats.bytes_written) / (1024.0 * 1024.0));
    ImGui::Text("Queue drops: %llu",
                static_cast<unsigned long long>(stats.dropped_blocks));
    ImGui::Text("Source drops: %llu",
                static_cast<unsigned long long>(stats.source_dropped_samples));
    ImGui::PopID();
}

void draw_epg_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("EPG", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("epg-panel");
    if (state.services.empty()) {
        draw_disabled_wrapped("No DVB-T services detected");
        ImGui::PopID();
        return;
    }
    // The EPG follows the service selected in the DVB-T panel; there is no
    // second selector here.
    if (!state.selected_service_id.has_value()) {
        draw_disabled_wrapped("Select a service in the DVB-T panel");
        ImGui::PopID();
        return;
    }
    const auto selected_service = std::ranges::find_if(
        state.services, [&state](const TransportService &service) {
            return state.selected_service_id == service.service_id;
        });
    if (selected_service != state.services.end()) {
        ImGui::TextDisabled(
            "%s", selected_service->name.empty()
                      ? std::format("Service {}", selected_service->service_id)
                            .c_str()
                      : std::format("{}  ({})", selected_service->name,
                                    selected_service->service_id)
                            .c_str());
    }
    const EpgSnapshot snapshot = state.epg.snapshot(*state.selected_service_id);
    if (snapshot.events.empty()) {
        draw_disabled_wrapped("Waiting for EIT p/f data...");
        ImGui::PopID();
        return;
    }
    const std::uint64_t now_utc = snapshot.utc_now.value_or(
        static_cast<std::uint64_t>(std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now())));
    const auto now_event =
        std::ranges::find_if(snapshot.events, [now_utc](const EpgEvent &event) {
            return now_utc >= event.start_time_utc &&
                   now_utc < event.start_time_utc + event.duration_seconds;
        });
    const auto next_event =
        std::ranges::find_if(snapshot.events, [now_utc](const EpgEvent &event) {
            return event.start_time_utc > now_utc;
        });

    const auto format_time = [](const std::uint64_t unix_seconds) {
        const std::time_t when = static_cast<std::time_t>(unix_seconds);
        const std::tm *local = std::localtime(&when);
        char buffer[32] = "---- --:--";
        if (local != nullptr) {
            std::strftime(buffer, sizeof(buffer), "%m-%d %H:%M", local);
        }
        return std::string(buffer);
    };
    // Broadcasters pad DVB text with regular and full-width spaces; trim them
    // for display so the panel matches what a TV would show.
    const auto trimmed = [](const std::string &text) {
        constexpr std::string_view full_width_space = "\xE3\x80\x80";
        std::size_t begin = 0;
        std::size_t end = text.size();
        const auto is_space = [&full_width_space](const std::string &value,
                                                  const std::size_t at) {
            return value[at] == ' ' || value[at] == '\t' ||
                   (at + full_width_space.size() <= value.size() &&
                    value.compare(at, full_width_space.size(),
                                  full_width_space) == 0);
        };
        while (begin < end && is_space(text, begin)) {
            begin += text[begin] == ' ' || text[begin] == '\t'
                         ? 1
                         : full_width_space.size();
        }
        while (end > begin) {
            const std::size_t previous =
                end - (text[end - 1] == ' ' || text[end - 1] == '\t'
                           ? 1
                           : full_width_space.size());
            if (!is_space(text, previous)) {
                break;
            }
            end = previous;
        }
        return text.substr(begin, end - begin);
    };
    const auto draw_event = [&format_time, &trimmed](const char *tag,
                                                     const EpgEvent &event,
                                                     const bool is_now) {
        ImGui::TextDisabled("%s", tag);
        const std::string start = format_time(event.start_time_utc);
        const std::string end =
            format_time(event.start_time_utc + event.duration_seconds);
        const std::string name = trimmed(event.name);
        const std::string description = trimmed(event.description);
        ImGui::Text("  %s - %s", start.c_str(), end.c_str());
        ImGui::SameLine();
        if (is_now) {
            ImGui::TextColored(ImVec4(0.35F, 0.88F, 0.55F, 1.0F), ">> %s",
                               name.c_str());
        } else {
            ImGui::Text("%s", name.c_str());
        }
        if (!event.genre.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("[%s]", event.genre.c_str());
        }
        if (!description.empty() && description != name) {
            ImGui::TextWrapped("%s", description.c_str());
        }
    };
    if (now_event != snapshot.events.end()) {
        draw_event("NOW", *now_event, true);
    } else {
        ImGui::TextDisabled("NOW");
        ImGui::TextDisabled("  (no event currently running)");
    }
    ImGui::Separator();
    if (next_event != snapshot.events.end()) {
        draw_event("NEXT", *next_event, false);
    } else {
        ImGui::TextDisabled("NEXT");
        ImGui::TextDisabled("  (no upcoming event)");
    }
    ImGui::PopID();
}

void draw_ts_recorder_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("MPEG-TS Stream Recorder",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("ts-recorder");
    consume_file_dialog_result(state, state.ts_file_dialog,
                               state.ts_recording_path, "MPEG-TS recording");
    const bool dialog_open = file_dialog_is_open(state.ts_file_dialog);
    const bool ts_source_available = state.receiver.is_streaming();
    const auto ts_stats = state.receiver.ts_recording_stats();

    draw_disabled_wrapped("Decoded DVB-T transport stream (MPEG-TS)");
    ImGui::TextUnformatted("Output file");
    ImGui::BeginDisabled(dialog_open);
    ImGui::SetNextItemWidth(-92.0F);
    ImGui::InputText("##ts-recording-output", &state.ts_recording_path);
    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        show_recording_file_dialog(state, state.ts_file_dialog,
                                   state.ts_recording_path,
                                   transport_stream_filters);
    }
    ImGui::EndDisabled();
    if (dialog_open) {
        ImGui::TextDisabled("Waiting for file selection...");
    }

    if (!ts_stats.active) {
        ImGui::BeginDisabled(!ts_source_available || dialog_open ||
                             state.ts_recording_path.empty());
        if (ImGui::Button("Start recording", ImVec2(-1.0F, 0.0F))) {
            std::string error;
            state.status = state.receiver.start_ts_recording(
                               state.ts_recording_path, error)
                               ? "Recording decoded MPEG-TS"
                               : error;
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop recording", ImVec2(-1.0F, 0.0F))) {
        state.receiver.stop_ts_recording();
        state.status = "MPEG-TS recording stopped";
    }

    ImGui::Text(
        "Duration: %s",
        format_recording_duration(ts_stats.elapsed_milliseconds).c_str());
    ImGui::Text("Written: %.2f MiB",
                static_cast<double>(ts_stats.bytes_written) /
                    (1024.0 * 1024.0));
    ImGui::Text("Queue drops: %llu",
                static_cast<unsigned long long>(ts_stats.dropped_blocks));
    ImGui::PopID();
}

void draw_sidebar(AppState &state) {
    draw_source_panel(state);

    if (ImGui::CollapsingHeader("Spectrum & Waterfall",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("spectrum-panel");
        const ColormapOption &selected =
            colormap_options[state.selected_colormap];
        ImGui::SetNextItemWidth(-1.0F);
        if (ImGui::BeginCombo("##waterfall-colormap", selected.name)) {
            for (std::size_t index = 0; index < colormap_options.size();
                 ++index) {
                const bool is_selected = index == state.selected_colormap;
                if (ImGui::Selectable(colormap_options[index].name,
                                      is_selected)) {
                    state.selected_colormap = index;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Waterfall colormap");
        }
        draw_spectrum(ImVec2(-1.0F, 150.0F), state.spectrum,
                      state.display_floor_dbfs, state.display_ceiling_dbfs);
        draw_waterfall(ImVec2(-1.0F, 150.0F), state.waterfall, state.spectrum,
                       state.selected_colormap, state.display_floor_dbfs,
                       state.display_ceiling_dbfs);

        ImGui::TextDisabled("Display range");
        const float ceiling_min =
            state.display_floor_dbfs + minimum_display_range_db;
        ImGui::SliderFloat("Ceiling##spectrum-range",
                           &state.display_ceiling_dbfs, ceiling_min,
                           signal_meter_ceiling_dbfs, "%.0f dBFS");
        state.display_ceiling_dbfs = std::clamp(
            state.display_ceiling_dbfs, ceiling_min, signal_meter_ceiling_dbfs);
        const float floor_max =
            state.display_ceiling_dbfs - minimum_display_range_db;
        ImGui::SliderFloat("Floor##spectrum-range", &state.display_floor_dbfs,
                           signal_meter_floor_dbfs, floor_max, "%.0f dBFS");
        state.display_floor_dbfs = std::clamp(
            state.display_floor_dbfs, signal_meter_floor_dbfs, floor_max);

        bool smoothing_changed = false;
        smoothing_changed |=
            ImGui::Checkbox("FFT smoothing", &state.fft_smoothing);
        ImGui::BeginDisabled(!state.fft_smoothing);
        ImGui::TextUnformatted("FFT smoothing speed");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-1.0F);
        smoothing_changed |= ImGui::InputInt("##fft-smoothing-speed",
                                             &state.fft_smoothing_speed);
        state.fft_smoothing_speed = std::max(state.fft_smoothing_speed, 1);
        ImGui::EndDisabled();
        smoothing_changed |=
            ImGui::Checkbox("SNR smoothing", &state.snr_smoothing);
        ImGui::BeginDisabled(!state.snr_smoothing);
        ImGui::TextUnformatted("SNR smoothing speed");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-1.0F);
        smoothing_changed |= ImGui::InputInt("##snr-smoothing-speed",
                                             &state.snr_smoothing_speed);
        state.snr_smoothing_speed = std::max(state.snr_smoothing_speed, 1);
        ImGui::EndDisabled();
        if (smoothing_changed) {
            state.receiver.set_display_smoothing(
                state.fft_smoothing, state.fft_smoothing_speed,
                state.snr_smoothing, state.snr_smoothing_speed);
            state.dvbt_demod->set_snr_smoothing(state.snr_smoothing,
                                                state.snr_smoothing_speed);
        }
        ImGui::PopID();
    }

    draw_receiver_panel(state);

    if (ImGui::CollapsingHeader("Constellation",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("constellation-panel");
        draw_constellation(ImVec2(-1.0F, 300.0F), state.signal_analysis);
        ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Signal & FEC",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("signal-quality-panel");
        const bool locked = state.signal_analysis.locked;
        const ImVec4 lock_colour = locked ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                                          : ImVec4(1.0F, 0.38F, 0.25F, 1.0F);
        draw_status_indicator(locked ? "OFDM MONITOR LOCKED"
                                     : "OFDM MONITOR UNLOCKED",
                              lock_colour);

        const bool transport_locked = state.decoder.transport.rs_synchronized &&
                                      state.decoder.transport.ts_packets != 0;
        const auto &transport = state.decoder.transport;
        const bool pre_viterbi_available =
            transport.pre_viterbi_compared_bits != 0;
        const double pre_viterbi_ber =
            pre_viterbi_available
                ? static_cast<double>(transport.pre_viterbi_error_bits) /
                      static_cast<double>(transport.pre_viterbi_compared_bits)
                : 0.0;
        const bool decoder_active = state.decoder.processing ||
                                    state.decoder.ofdm_locked ||
                                    state.decoder.input_blocks != 0;
        const ImVec4 decoder_colour =
            transport_locked
                ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                : (decoder_active ? ImVec4(1.0F, 0.72F, 0.22F, 1.0F)
                                  : ImVec4(0.55F, 0.62F, 0.70F, 1.0F));
        draw_status_indicator(
            transport_locked
                ? "TS DECODER LOCKED"
                : (decoder_active ? "TS DECODER ACQUIRING" : "TS DECODER IDLE"),
            decoder_colour);

        const bool ratio_available =
            state.decoder.processing_realtime_ratio > 0.0F;
        const bool input_queue_near_full =
            state.decoder.input_queue_capacity_samples != 0 &&
            state.decoder.queued_input_samples * 4 >=
                state.decoder.input_queue_capacity_samples * 3;
        const bool fec_queue_near_full =
            state.decoder.symbol_queue_capacity != 0 &&
            state.decoder.queued_symbols * 4 >=
                state.decoder.symbol_queue_capacity * 3;
        const bool cpu_slow =
            ratio_available && state.decoder.processing_realtime_ratio > 1.0F;
        const bool fec_signal_limited =
            !transport_locked && fec_queue_near_full && pre_viterbi_available &&
            pre_viterbi_ber >= 0.10;
        const bool cpu_overload =
            !fec_signal_limited && cpu_slow && input_queue_near_full;
        const char *pipeline_status = "CPU LOAD";
        ImVec4 pipeline_colour{0.55F, 0.62F, 0.70F, 1.0F};
        if (fec_signal_limited) {
            pipeline_status = "FEC SIGNAL LIMITED";
            pipeline_colour = ImVec4(1.0F, 0.72F, 0.22F, 1.0F);
        } else if (cpu_overload) {
            pipeline_status = "CPU OVERLOAD";
            pipeline_colour = ImVec4(1.0F, 0.38F, 0.25F, 1.0F);
        } else if (cpu_slow) {
            pipeline_status = "CPU SLOW";
            pipeline_colour = ImVec4(1.0F, 0.72F, 0.22F, 1.0F);
        } else if (ratio_available) {
            pipeline_status = "CPU REALTIME";
            pipeline_colour = ImVec4(0.35F, 0.88F, 0.55F, 1.0F);
        }
        const float input_queue_percent =
            state.decoder.input_queue_capacity_samples == 0
                ? 0.0F
                : 100.0F *
                      static_cast<float>(state.decoder.queued_input_samples) /
                      static_cast<float>(
                          state.decoder.input_queue_capacity_samples);
        draw_status_indicator(pipeline_status, pipeline_colour);
        if (ImGui::BeginTable("cpu-diagnostics", 4,
                              ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_PadOuterX)) {
            ImGui::TableSetupColumn("label-left",
                                    ImGuiTableColumnFlags_WidthFixed, 72.0F);
            ImGui::TableSetupColumn("value-left",
                                    ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("label-right",
                                    ImGuiTableColumnFlags_WidthFixed, 72.0F);
            ImGui::TableSetupColumn("value-right",
                                    ImGuiTableColumnFlags_WidthStretch);
            const auto cell = [](const char *label, const auto &value) {
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", label);
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(value.c_str());
            };
            cell("Load",
                 ratio_available
                     ? std::format("{:.2f}x input",
                                   state.decoder.processing_realtime_ratio)
                     : std::string{"Measuring"});
            cell("IQ queue",
                 std::format("{:3.0f}%",
                             std::clamp(input_queue_percent, 0.0F, 100.0F)));
            cell("FEC queue", std::to_string(state.decoder.queued_symbols));
            cell("Drops", std::to_string(state.decoder.dropped_blocks));
            cell("Resample", std::to_string(state.decoder.resample_workers));
            cell("Symbol", std::to_string(state.decoder.symbol_workers));
            cell("Inner FEC",
                 std::to_string(state.decoder.transport.viterbi_workers));
            cell("", std::string{});
            ImGui::EndTable();
        }
        ImGui::Separator();

        const std::string power =
            state.spectrum.valid
                ? std::format("{:.1f} dBFS", state.spectrum.signal_power_dbfs)
                : "-- dBFS";
        const float power_fraction =
            state.spectrum.valid
                ? std::clamp(
                      (state.spectrum.signal_power_dbfs -
                       signal_meter_floor_dbfs) /
                          (signal_meter_ceiling_dbfs - signal_meter_floor_dbfs),
                      0.0F, 1.0F)
                : 0.0F;
        draw_metric("Signal power", power.c_str(), power_fraction,
                    ImVec4(0.35F, 0.78F, 0.95F, 1.0F));
        const bool has_snr = state.signal_analysis.locked ||
                             state.spectrum.channel_metrics_valid;
        const float snr_value = state.signal_analysis.locked
                                    ? state.signal_analysis.cp_snr_db
                                    : state.spectrum.rf_snr_db;
        const std::string snr =
            has_snr ? std::format("{:.1f} dB", snr_value) : "-- dB";
        const float snr_fraction =
            has_snr ? std::clamp((snr_value + 5.0F) / 40.0F, 0.0F, 1.0F) : 0.0F;
        draw_metric(
            state.signal_analysis.locked ? "CP SNR est." : "RF SNR est.",
            snr.c_str(), snr_fraction, ImVec4(0.35F, 0.88F, 0.55F, 1.0F));
        const bool has_notch = state.signal_analysis.locked ||
                               state.spectrum.channel_metrics_valid;
        const float notch_value = state.signal_analysis.locked
                                      ? state.signal_analysis.deepest_notch_db
                                      : state.spectrum.deepest_notch_db;
        const std::string notch =
            has_notch ? std::format("{:.1f} dB", notch_value) : "-- dB";
        const float notch_fraction =
            has_notch ? std::clamp(1.0F + (notch_value / 40.0F), 0.0F, 1.0F)
                      : 0.0F;
        draw_metric("Deepest Notch", notch.c_str(), notch_fraction,
                    ImVec4(0.35F, 0.78F, 0.95F, 1.0F));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip(
                "Robust channel dip: lower 1%% versus median; outer filter "
                "skirts are excluded when OFDM is locked.");
        }
        const std::string carrier_offset =
            state.signal_analysis.locked
                ? std::format("{:+.2f} kHz",
                              state.signal_analysis.carrier_offset_hz / 1000.0F)
                : "-- kHz";
        const float fft_size =
            state.signal_analysis.mode == airspy_tv::dvbt::TransmissionMode::k8
                ? 8192.0F
                : 2048.0F;
        const float dvbt_sample_rate_hz =
            static_cast<float>(state.dvbt_parameters.channel_bandwidth_hz) *
            (8.0F / 7.0F);
        const float maximum_fractional_offset_hz =
            dvbt_sample_rate_hz / (2.0F * fft_size);
        const float carrier_offset_position =
            state.signal_analysis.locked
                ? 0.5F + (state.signal_analysis.carrier_offset_hz /
                          (2.0F * maximum_fractional_offset_hz))
                : 0.5F;
        draw_bipolar_metric("Carrier offset", carrier_offset.c_str(),
                            carrier_offset_position,
                            ImVec4(0.52F, 0.82F, 1.0F, 1.0F));
        const std::string mer =
            state.signal_analysis.locked
                ? std::format("{:.1f} dB", state.signal_analysis.mer_db)
                : "-- dB";
        draw_metric(
            "MER", mer.c_str(),
            state.signal_analysis.locked
                ? std::clamp(state.signal_analysis.mer_db / 40.0F, 0.0F, 1.0F)
                : 0.0F,
            ImVec4(0.35F, 0.78F, 0.95F, 1.0F));
        const auto ber_quality = [](const double ber) {
            return ber == 0.0
                       ? 1.0F
                       : std::clamp(static_cast<float>(-std::log10(ber) / 7.0),
                                    0.0F, 1.0F);
        };
        const std::string pre_viterbi_text =
            pre_viterbi_available ? std::format("{:.2e}", pre_viterbi_ber)
                                  : "--";
        draw_metric("Inner BER", pre_viterbi_text.c_str(),
                    pre_viterbi_available ? ber_quality(pre_viterbi_ber) : 0.0F,
                    ImVec4(0.75F, 0.72F, 0.30F, 1.0F));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip(
                "Bit errors before the outer FEC, measured against the "
                "received mother-code metrics. For DVB-T this is the "
                "pre-Viterbi BER (re-encoded survivor path).\n%llu errors / "
                "%llu compared bits",
                static_cast<unsigned long long>(
                    transport.pre_viterbi_error_bits),
                static_cast<unsigned long long>(
                    transport.pre_viterbi_compared_bits));
        }

        const bool post_viterbi_available =
            transport.post_viterbi_compared_bits != 0;
        const double post_viterbi_ber =
            post_viterbi_available
                ? static_cast<double>(transport.post_viterbi_error_bits) /
                      static_cast<double>(transport.post_viterbi_compared_bits)
                : 0.0;
        const std::string post_viterbi_text =
            post_viterbi_available ? std::format("{:.2e}", post_viterbi_ber)
                                   : "--";
        draw_metric("Outer BER", post_viterbi_text.c_str(),
                    post_viterbi_available ? ber_quality(post_viterbi_ber)
                                           : 0.0F,
                    ImVec4(0.35F, 0.88F, 0.55F, 1.0F));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip(
                "Residual bit errors after the outer FEC: payload-bit "
                "corrections made by successful RS(204,188) codewords; "
                "uncorrectable packets are reported separately.\n%llu "
                "corrected bits / %llu checked bits; %llu RS failures",
                static_cast<unsigned long long>(
                    transport.post_viterbi_error_bits),
                static_cast<unsigned long long>(
                    transport.post_viterbi_compared_bits),
                static_cast<unsigned long long>(
                    transport.rs_uncorrectable_packets));
        }
        ImGui::TextColored(
            transport.rs_uncorrectable_packets == 0
                ? ImVec4(0.45F, 0.82F, 0.62F, 1.0F)
                : ImVec4(0.95F, 0.55F, 0.28F, 1.0F),
            "RS failures %llu    TEI output %llu",
            static_cast<unsigned long long>(transport.rs_uncorrectable_packets),
            static_cast<unsigned long long>(transport.tei_packets));
        ImGui::Separator();
        const char *mode = "--";
        const char *guard = "--";
        const char *modulation = "--";
        const char *code_rate = "--";
        if (state.dvbt_parameters.code_rate.has_value()) {
            switch (*state.dvbt_parameters.code_rate) {
            case airspy_tv::dvbt::CodeRate::rate_1_2:
                code_rate = "1/2 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_2_3:
                code_rate = "2/3 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_3_4:
                code_rate = "3/4 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_5_6:
                code_rate = "5/6 (manual)";
                break;
            case airspy_tv::dvbt::CodeRate::rate_7_8:
                code_rate = "7/8 (manual)";
                break;
            }
        }
        if (state.signal_analysis.locked) {
            mode = state.signal_analysis.mode ==
                           airspy_tv::dvbt::TransmissionMode::k8
                       ? "8K"
                       : "2K";
            switch (state.signal_analysis.guard_interval) {
            case airspy_tv::dvbt::GuardInterval::gi_1_32:
                guard = "1/32";
                break;
            case airspy_tv::dvbt::GuardInterval::gi_1_16:
                guard = "1/16";
                break;
            case airspy_tv::dvbt::GuardInterval::gi_1_8:
                guard = "1/8";
                break;
            case airspy_tv::dvbt::GuardInterval::gi_1_4:
                guard = "1/4";
                break;
            }
            switch (state.signal_analysis.constellation) {
            case airspy_tv::dvbt::Constellation::qpsk:
                modulation = "QPSK";
                break;
            case airspy_tv::dvbt::Constellation::qam16:
                modulation = "16-QAM";
                break;
            case airspy_tv::dvbt::Constellation::qam64:
                modulation = "64-QAM";
                break;
            }
        }
        if (state.decoder.tps_locked) {
            mode = state.decoder.tps_mode == TransmissionMode::k8 ? "8K (TPS)"
                                                                  : "2K (TPS)";
            switch (state.decoder.tps_guard_interval) {
            case GuardInterval::gi_1_32:
                guard = "1/32 (TPS)";
                break;
            case GuardInterval::gi_1_16:
                guard = "1/16 (TPS)";
                break;
            case GuardInterval::gi_1_8:
                guard = "1/8 (TPS)";
                break;
            case GuardInterval::gi_1_4:
                guard = "1/4 (TPS)";
                break;
            }
            switch (state.decoder.tps_constellation) {
            case Constellation::qpsk:
                modulation = "QPSK (TPS)";
                break;
            case Constellation::qam16:
                modulation = "16-QAM (TPS)";
                break;
            case Constellation::qam64:
                modulation = "64-QAM (TPS)";
                break;
            }
            if (!state.dvbt_parameters.code_rate.has_value()) {
                switch (state.decoder.tps_code_rate) {
                case CodeRate::rate_1_2:
                    code_rate = "1/2 (TPS)";
                    break;
                case CodeRate::rate_2_3:
                    code_rate = "2/3 (TPS)";
                    break;
                case CodeRate::rate_3_4:
                    code_rate = "3/4 (TPS)";
                    break;
                case CodeRate::rate_5_6:
                    code_rate = "5/6 (TPS)";
                    break;
                case CodeRate::rate_7_8:
                    code_rate = "7/8 (TPS)";
                    break;
                }
            }
        }
        ImGui::Text("Bandwidth      %u MHz",
                    state.dvbt_parameters.channel_bandwidth_hz / 1'000'000U);
        ImGui::Text("Mode           %s", mode);
        ImGui::Text("Guard          %s", guard);
        ImGui::Text("Modulation     %s", modulation);
        ImGui::Text("Code rate      %s", code_rate);
        draw_disabled_wrapped(
            state.decoder.tps_locked
                ? "TPS synchronization and BCH are valid; auto decoder "
                  "parameters come from this TPS frame."
            : state.signal_analysis.locked
                ? "OFDM monitor is locked; waiting for a valid TPS frame."
                : "RF estimates use the selected channel bandwidth and "
                  "out-of-channel noise; MER and constellation require OFDM "
                  "lock.");
        ImGui::PopID();
    }

    draw_epg_panel(state);
    draw_ts_recorder_panel(state);
    draw_recorder_panel(state);
}

void draw_video_panel(AppState &state) {
    const ImVec2 available = ImGui::GetContentRegionAvail();
    ImGui::Dummy(available);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(5, 8, 13, 255), 5.0F);

    const float header_height = 46.0F;
    const float footer_height = 46.0F;
    const ImVec2 footer_origin{origin.x, extent.y - footer_height};
    const ImVec2 video_origin{origin.x, origin.y + header_height};
    const ImVec2 video_extent{extent.x, footer_origin.y};
    const float pixel_density = state.window == nullptr
                                    ? 1.0F
                                    : SDL_GetWindowPixelDensity(state.window);
    const int video_width = static_cast<int>(
        std::max(1.0F, (video_extent.x - video_origin.x) * pixel_density));
    const int video_height = static_cast<int>(
        std::max(1.0F, (video_extent.y - video_origin.y) * pixel_density));
    const std::uint32_t video_texture =
        state.player.render(video_width, video_height);
    if (video_texture != 0 && state.player.ready()) {
        draw->AddImage(static_cast<ImTextureID>(video_texture), video_origin,
                       video_extent);
    }

    draw->AddRectFilled(origin, ImVec2(extent.x, origin.y + header_height),
                        IM_COL32(15, 24, 35, 255), 5.0F);
    const auto selected_service = std::ranges::find_if(
        state.services, [&state](const TransportService &service) {
            return state.selected_service_id == service.service_id;
        });
    const std::string program_title =
        selected_service == state.services.end()
            ? "DVB-T SERVICE"
            : std::format(
                  "DVB-T  {}",
                  selected_service->name.empty()
                      ? std::format("Service {}", selected_service->service_id)
                      : selected_service->name);
    draw->AddText(ImVec2(origin.x + 16.0F, origin.y + 14.0F),
                  IM_COL32(220, 230, 240, 255), program_title.c_str());
    draw->AddText(ImVec2(extent.x - 165.0F, origin.y + 14.0F),
                  IM_COL32(90, 205, 255, 255), "VIDEO PREVIEW");

    const ImVec2 center{(origin.x + extent.x) * 0.5F,
                        (origin.y + footer_origin.y) * 0.5F};
    const std::string player_status = state.player.status();
    if (!state.player.ready()) {
        draw->AddCircle(center, 54.0F, IM_COL32(55, 78, 102, 255), 0, 2.0F);
        draw->AddTriangleFilled(ImVec2(center.x - 14.0F, center.y - 24.0F),
                                ImVec2(center.x - 14.0F, center.y + 24.0F),
                                ImVec2(center.x + 28.0F, center.y),
                                IM_COL32(65, 175, 235, 230));
        const ImVec2 text_size = ImGui::CalcTextSize(player_status.c_str());
        draw->AddText(ImVec2(center.x - (text_size.x * 0.5F), center.y + 74.0F),
                      IM_COL32(130, 150, 170, 255), player_status.c_str());
    }

    draw->AddRectFilled(footer_origin, extent, IM_COL32(13, 20, 30, 245), 5.0F);

    constexpr float horizontal_padding = 18.0F;
    constexpr float mute_width = 72.0F;
    constexpr float control_spacing = 8.0F;
    ImGui::SetCursorScreenPos(
        ImVec2(footer_origin.x + horizontal_padding, footer_origin.y + 8.0F));
    const float service_width = std::max(180.0F, available.x * 0.48F);
    ImGui::SetNextItemWidth(service_width);
    const std::string service_preview =
        selected_service == state.services.end()
            ? "No DVB-T services"
            : (selected_service->name.empty()
                   ? std::format("Service {}", selected_service->service_id)
                   : std::format("{}  ({})", selected_service->name,
                                 selected_service->service_id));
    ImGui::BeginDisabled(state.services.empty());
    if (ImGui::BeginCombo("##service-selection", service_preview.c_str())) {
        for (const auto &service : state.services) {
            const std::string label =
                service.name.empty()
                    ? std::format("Service {}", service.service_id)
                    : std::format("{}  ({})", service.name, service.service_id);
            const bool selected =
                state.selected_service_id == service.service_id;
            if (ImGui::Selectable(label.c_str(), selected)) {
                state.selected_service_id = service.service_id;
                state.player.select_service(service);
                state.status = "Selected " + label;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    ImGui::SameLine(0.0F, control_spacing);
    if (ImGui::Button(state.player.muted() ? "Unmute" : "Mute",
                      ImVec2(mute_width, 0.0F))) {
        state.player.set_muted(!state.player.muted());
    }
    ImGui::SameLine(0.0F, control_spacing);
    float volume = state.player.volume();
    ImGui::SetNextItemWidth(std::max(
        100.0F, available.x - (horizontal_padding * 2.0F) - service_width -
                    mute_width - (control_spacing * 2.0F)));
    if (ImGui::SliderFloat("##playback-volume", &volume, 0.0F, 100.0F,
                           "Volume %.0f%%")) {
        state.player.set_volume(volume);
    }
}

void draw_application(AppState &state) {
    state.player.set_source_active(state.receiver.is_streaming());
    const DeviceDescriptor *source_descriptor = state.receiver.descriptor();
    if (source_descriptor != state.last_source_descriptor) {
        state.epg.reset();
        state.last_source_descriptor = source_descriptor;
    }
    state.player.poll_events();
    state.spectrum = state.receiver.spectrum_snapshot();
    state.signal_analysis = state.dvbt_demod->analysis_snapshot();
    state.decoder = state.dvbt_demod->stats();
    state.services = state.receiver.transport_services();
    if (!state.services.empty() &&
        std::ranges::none_of(state.services, [&state](const auto &service) {
            return state.selected_service_id == service.service_id;
        })) {
        state.selected_service_id = state.services.front().service_id;
    }
    if (state.selected_service_id.has_value()) {
        const auto service = std::ranges::find_if(
            state.services, [&state](const TransportService &candidate) {
                return candidate.service_id == *state.selected_service_id;
            });
        if (service != state.services.end()) {
            state.player.select_service(*service);
        }
    } else {
        state.player.clear_service();
    }
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                                       ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings |
                                       ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("Airspy TV", nullptr, flags);

    ImGui::BeginChild("top-bar", ImVec2(0.0F, 58.0F), ImGuiChildFlags_Borders);
    ImGui::TextColored(accent, "AIRSPY TV");
    ImGui::SameLine(128.0F);
    const bool file_source =
        state.receiver.descriptor() != nullptr &&
        state.receiver.descriptor()->backend == SdrBackend::File;
    ImGui::BeginDisabled(file_source);
    if (const auto frequency = draw_frequency_control(
            "center-frequency", state.settings.center_frequency_hz);
        frequency.has_value()) {
        request_center_frequency(state, *frequency);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    const float status_width = ImGui::CalcTextSize(state.status.c_str()).x;
    ImGui::SetCursorPosX(
        std::max(ImGui::GetCursorPosX(),
                 ImGui::GetWindowWidth() - status_width - 20.0F));
    ImGui::TextColored(state.receiver.is_recording()
                           ? ImVec4(1.0F, 0.30F, 0.28F, 1.0F)
                           : ImVec4(0.58F, 0.66F, 0.74F, 1.0F),
                       "%s", state.status.c_str());
    ImGui::EndChild();

    const float sidebar =
        std::min(panel_width, ImGui::GetContentRegionAvail().x * 0.42F);
    ImGui::BeginChild("side-panel", ImVec2(sidebar, 0.0F),
                      ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar);
    draw_sidebar(state);
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("video-panel", ImVec2(0.0F, 0.0F),
                      ImGuiChildFlags_Borders);
    draw_video_panel(state);
    ImGui::EndChild();

    ImGui::End();
}

int enumerate_cli() {
    const EnumerationResult result = SdrDevice::enumerate(true);
    for (const std::string &warning : result.warnings) {
        std::cerr << "warning: " << warning << '\n';
    }
    for (const DeviceDescriptor &device : result.devices) {
        std::cout << airspy_tv::backend_name(device.backend) << '\t'
                  << device.display_name << '\t' << device.id << '\n';
    }
    std::cout << result.devices.size() << " device(s)\n";
    return 0;
}

int record_first_cli(const std::filesystem::path &path, const int duration_ms) {
    EnumerationResult result = SdrDevice::enumerate(false);
    if (result.devices.empty()) {
        std::cerr << "No SDR devices found\n";
        return 1;
    }

    const auto native = std::ranges::find_if(
        result.devices, [](const DeviceDescriptor &device) {
            return device.backend == SdrBackend::AirspyNative;
        });
    const DeviceDescriptor &descriptor =
        native == result.devices.end() ? result.devices.front() : *native;
    SdrDevice receiver;
    std::string error;
    if (!receiver.open(descriptor, error)) {
        std::cerr << error << '\n';
        return 1;
    }

    SourceSettings settings;
    if (!receiver.sample_rates().empty()) {
        settings.sample_rate_hz = *std::ranges::min_element(
            receiver.sample_rates(), {},
            [target = settings.sample_rate_hz](const std::uint32_t rate) {
                return std::llabs(static_cast<long long>(rate) - target);
            });
    }
    if (!receiver.start_recording(path, settings, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    ++settings.airspy_gain;
    if (!receiver.set_gain(settings, error) ||
        !receiver.set_bias_tee(false, error) ||
        !receiver.set_center_frequency(settings.center_frequency_hz + 1'000,
                                       error) ||
        !receiver.set_center_frequency(settings.center_frequency_hz, error)) {
        receiver.stop_recording();
        std::cerr << error << '\n';
        return 1;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(std::max(duration_ms, 1)));
    receiver.stop_recording();
    const auto stats = receiver.recording_stats();
    const SpectrumSnapshot spectrum = receiver.spectrum_snapshot();
    std::cout << "Recorded " << stats.complex_samples << " complex samples ("
              << stats.bytes_written
              << " bytes), queue drops=" << stats.dropped_blocks
              << ", source drops=" << stats.source_dropped_samples;
    if (spectrum.valid) {
        std::cout << ", signal power=" << std::fixed << std::setprecision(1)
                  << spectrum.signal_power_dbfs << " dBFS";
    }
    std::cout << '\n';
    return stats.bytes_written == 0 || !spectrum.valid ? 1 : 0;
}

int inspect_iq_cli(const std::filesystem::path &path,
                   const std::uint32_t raw_sample_rate_hz,
                   const std::uint64_t raw_center_frequency_hz) {
    SdrDevice receiver;
    auto demodulator = std::make_unique<StreamDecoder>();
    StreamDecoder *dvbt_demod = demodulator.get();
    receiver.set_demodulator(std::move(demodulator));
    SourceSettings settings;
    settings.sample_rate_hz = raw_sample_rate_hz;
    settings.center_frequency_hz = raw_center_frequency_hz;
    std::string error;
    if (!receiver.open_iq_file(path, settings, error) ||
        !receiver.start_stream(settings, error)) {
        std::cerr << error << '\n';
        return 1;
    }

    SpectrumSnapshot spectrum;
    SignalAnalysisSnapshot analysis;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (std::chrono::steady_clock::now() < deadline) {
        spectrum = receiver.spectrum_snapshot();
        analysis = dvbt_demod->analysis_snapshot();
        if (spectrum.sequence >= 50 && analysis.locked) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const DeviceDescriptor *descriptor = receiver.descriptor();
    std::cout << (descriptor == nullptr ? path.filename().string()
                                        : descriptor->display_name)
              << ", sample-rate=" << settings.sample_rate_hz
              << ", center-frequency=" << settings.center_frequency_hz;
    if (spectrum.valid) {
        std::cout << ", signal-power=" << std::fixed << std::setprecision(1)
                  << spectrum.signal_power_dbfs << " dBFS";
        if (spectrum.channel_metrics_valid) {
            std::cout << ", rf-snr-estimate=" << spectrum.rf_snr_db
                      << " dB, deepest-notch=" << spectrum.deepest_notch_db
                      << " dB";
        }
    }
    if (analysis.locked) {
        const char *guard = "1/4";
        switch (analysis.guard_interval) {
        case airspy_tv::dvbt::GuardInterval::gi_1_32:
            guard = "1/32";
            break;
        case airspy_tv::dvbt::GuardInterval::gi_1_16:
            guard = "1/16";
            break;
        case airspy_tv::dvbt::GuardInterval::gi_1_8:
            guard = "1/8";
            break;
        case airspy_tv::dvbt::GuardInterval::gi_1_4:
            break;
        }
        std::cout << ", ofdm-lock="
                  << (analysis.mode == airspy_tv::dvbt::TransmissionMode::k8
                          ? "8K"
                          : "2K")
                  << ", guard=" << guard << ", cp-snr=" << analysis.cp_snr_db
                  << " dB, mer=" << analysis.mer_db
                  << " dB, channel-notch=" << analysis.deepest_notch_db
                  << " dB, carrier-offset=" << analysis.carrier_offset_hz
                  << " Hz, constellation=";
        switch (analysis.constellation) {
        case airspy_tv::dvbt::Constellation::qpsk:
            std::cout << "QPSK";
            break;
        case airspy_tv::dvbt::Constellation::qam16:
            std::cout << "16-QAM";
            break;
        case airspy_tv::dvbt::Constellation::qam64:
            std::cout << "64-QAM";
            break;
        }
        std::cout << "\n";
    } else {
        std::cout << ", ofdm-lock=no";
    }
    std::cout << '\n';
    receiver.close();
    return spectrum.valid ? 0 : 1;
}

int decode_iq_cli(const std::filesystem::path &source,
                  const std::filesystem::path &destination,
                  const std::uint32_t raw_sample_rate_hz,
                  const std::size_t decoder_threads, const bool debug) {
    IqFileInfo info;
    std::string error;
    if (!airspy_tv::resolve_iq_file(source, raw_sample_rate_hz, 0, info,
                                    error)) {
        std::cerr << error << '\n';
        return 1;
    }

    try {
        const auto output_path =
            std::filesystem::absolute(destination).lexically_normal();
        if (std::filesystem::absolute(source).lexically_normal() ==
                output_path ||
            std::filesystem::absolute(info.data_path).lexically_normal() ==
                output_path) {
            std::cerr << "Output MPEG-TS path must differ from the I/Q source "
                         "and data paths\n";
            return 1;
        }
    } catch (const std::filesystem::filesystem_error &exception) {
        std::cerr << "Unable to resolve input/output paths: "
                  << exception.what() << '\n';
        return 1;
    }

    std::ifstream input(info.data_path, std::ios::binary);
    std::ofstream output(destination, std::ios::binary | std::ios::trunc);
    if (!input || !output) {
        std::cerr << "Unable to open I/Q input or MPEG-TS output\n";
        return 1;
    }

    StreamDecoder decoder;
    ReceiverParameters decoder_parameters;
    decoder_parameters.worker_threads = decoder_threads;
    decoder.set_parameters(decoder_parameters);
    if (debug) {
        std::cerr << "Decoder worker budget="
                  << (decoder_threads == 0
                          ? airspy_tv::dvbt::default_viterbi_worker_count()
                          : decoder_threads)
                  << (decoder_threads == 0 ? " (auto)\n" : "\n");
    }
    std::atomic_bool output_failed{};
    decoder.set_transport_callback(
        [&output, &output_failed](const std::span<const std::uint8_t> ts) {
            output.write(reinterpret_cast<const char *>(ts.data()),
                         static_cast<std::streamsize>(ts.size()));
            if (!output) {
                output_failed.store(true, std::memory_order_relaxed);
            }
        });

    const auto started_at = std::chrono::steady_clock::now();
    constexpr std::size_t scalar_samples =
        StreamDecoder::processing_chunk_samples * 2;
    std::vector<std::int16_t> block(scalar_samples);
    std::uint64_t input_complex_samples = 0;
    std::uint64_t reported_processed_chunks = 0;
    std::uint64_t reported_processed_samples = 0;
    std::uint64_t reported_transport_bytes = 0;
    std::uint64_t reported_phase_discontinuities = 0;
    const auto report_chunk = [&](const StreamDecoderStats &stats) {
        reported_processed_chunks = stats.processed_chunks;
        reported_processed_samples = stats.processed_input_samples;
        reported_transport_bytes = stats.transport_bytes;
        const float realtime_speed =
            stats.processing_realtime_ratio > 0.0F
                ? 1.0F / stats.processing_realtime_ratio
                : 0.0F;
        std::cerr << "chunk=" << stats.processed_chunks
                  << " input=" << stats.processed_input_samples
                  << " samples TS=" << stats.transport_bytes
                  << " bytes realtime-speed=" << realtime_speed << "x";
        if (stats.mer_db != 0.0F) {
            std::cerr << " MER=" << stats.mer_db << " dB";
        }
        std::cerr << " carrier=" << stats.carrier_bin_offset
                  << " residual=" << stats.residual_carrier_offset_hz << " Hz"
                  << " tracked=" << stats.tracked_carrier_offset_hz << " Hz"
                  << " start=" << stats.acquisition_start
                  << " timing=" << stats.timing_offset_samples << " smp"
                  << " carried=" << (stats.state_carried ? 1 : 0)
                  << " fec-skip=" << (stats.fec_skipped ? 1 : 0) << '\n';
        if (debug) {
            std::cerr << "  stages: resample=" << stats.resample_time_ms
                      << " ms acquisition=" << stats.acquisition_time_ms
                      << " ms equalization=" << stats.equalization_time_ms
                      << " ms FEC=" << stats.fec_time_ms << " ms; workers "
                      << "resample=" << stats.resample_workers
                      << " symbol=" << stats.symbol_workers
                      << " Viterbi=" << stats.transport.viterbi_workers << '\n';
            std::cerr << "  FEC detail: demap=" << stats.demap_time_ms
                      << " ms deinterleave=" << stats.deinterleave_time_ms
                      << " ms depuncture/quantize=" << stats.depuncture_time_ms
                      << " ms transport=" << stats.transport_time_ms << " ms\n";
            std::cerr << "  TPS: " << (stats.tps_locked ? "locked" : "unlocked")
                      << '\n';
            if (stats.transport.pre_viterbi_compared_bits != 0) {
                const double pre_viterbi_ber =
                    static_cast<double>(
                        stats.transport.pre_viterbi_error_bits) /
                    static_cast<double>(
                        stats.transport.pre_viterbi_compared_bits);
                std::cerr << std::format("  BER: pre-Viterbi={:.3e}",
                                         pre_viterbi_ber);
                if (stats.transport.post_viterbi_compared_bits != 0) {
                    const double post_viterbi_ber =
                        static_cast<double>(
                            stats.transport.post_viterbi_error_bits) /
                        static_cast<double>(
                            stats.transport.post_viterbi_compared_bits);
                    std::cerr << std::format(" post-Viterbi={:.3e}",
                                             post_viterbi_ber);
                } else {
                    std::cerr << " post-Viterbi=--";
                }
                std::cerr << '\n';
            }
        }
        const std::uint64_t phase_delta =
            stats.pilot_phase_discontinuities - reported_phase_discontinuities;
        reported_phase_discontinuities = stats.pilot_phase_discontinuities;
        if (phase_delta != 0) {
            std::cerr << "  phase-discontinuities=" << phase_delta << '\n';
        }
    };
    while (input && !output_failed.load(std::memory_order_relaxed)) {
        input.read(
            reinterpret_cast<char *>(block.data()),
            static_cast<std::streamsize>(block.size() * sizeof(block.front())));
        const std::streamsize bytes_read = input.gcount();
        if (bytes_read <= 0) {
            break;
        }
        const std::size_t scalar_count =
            static_cast<std::size_t>(bytes_read) / sizeof(block.front());
        input_complex_samples += scalar_count / 2;
        decoder.submit_blocking(std::span(block).first(scalar_count),
                                info.sample_rate_hz);
        if (scalar_count == block.size()) {
            const auto progress = decoder.stats();
            if (progress.processed_chunks != reported_processed_chunks) {
                report_chunk(progress);
            }
        }
    }
    decoder.flush();
    output.flush();

    const auto stats = decoder.stats();
    if (reported_processed_samples != stats.processed_input_samples ||
        reported_transport_bytes != stats.transport_bytes) {
        report_chunk(stats);
    }
    const double input_seconds = static_cast<double>(input_complex_samples) /
                                 static_cast<double>(info.sample_rate_hz);
    const double wall_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      started_at)
            .count();
    if (debug) {
        std::cerr << "decoded " << input_complex_samples << " complex samples ("
                  << input_seconds << " s) in " << wall_seconds
                  << " s, TS=" << stats.transport_bytes
                  << " bytes, symbols=" << stats.ofdm_symbols
                  << ", RS=" << stats.transport.rs_packets << ", RS failures="
                  << stats.transport.rs_uncorrectable_packets
                  << ", TEI=" << stats.transport.tei_packets
                  << ", packets=" << stats.transport.ts_packets << '\n';
    }
    if (output_failed.load(std::memory_order_relaxed) || !output) {
        std::cerr << "Failed while writing MPEG-TS output\n";
        return 1;
    }
    if (stats.dropped_blocks != 0) {
        std::cerr << "Internal error: decoder-paced I/Q input dropped "
                  << stats.dropped_blocks << " block(s)\n";
        return 1;
    }
    return stats.transport_bytes == 0 ? 2 : 0;
}

} // namespace

int main(const int argc, char **argv) {
    if (argc > 1 && std::string_view(argv[1]) == "--enumerate") {
        return enumerate_cli();
    }
    if (argc > 1 && std::string_view(argv[1]) == "--help") {
        std::cout << "Usage: airspy-tv [--enumerate|--record-first PATH "
                     "[MILLISECONDS]|--inspect-iq PATH [SAMPLE_RATE_HZ] "
                     "[CENTER_FREQUENCY_HZ]|--decode-iq INPUT OUTPUT.ts "
                     "[SAMPLE_RATE_HZ] [--decoder-threads N] [-d|--debug]|"
                     "--help]\n";
        return 0;
    }
    if (argc > 2 && std::string_view(argv[1]) == "--record-first") {
        int duration_ms = 1000;
        if (argc > 3) {
            const std::string_view duration_text = argv[3];
            const auto parsed = std::from_chars(
                duration_text.begin(), duration_text.end(), duration_ms);
            if (parsed.ec != std::errc{} || parsed.ptr != duration_text.end() ||
                duration_ms <= 0) {
                std::cerr << "Invalid recording duration: " << duration_text
                          << '\n';
                return 2;
            }
        }
        return record_first_cli(argv[2], duration_ms);
    }
    if (argc > 2 && std::string_view(argv[1]) == "--inspect-iq") {
        std::uint32_t sample_rate_hz = 10'000'000;
        std::uint64_t center_frequency_hz = 545'000'000;
        if (argc > 3) {
            const std::string_view text = argv[3];
            const auto parsed =
                std::from_chars(text.begin(), text.end(), sample_rate_hz);
            if (parsed.ec != std::errc{} || parsed.ptr != text.end() ||
                sample_rate_hz == 0) {
                std::cerr << "Invalid raw I/Q sample rate: " << text << '\n';
                return 2;
            }
        }
        if (argc > 4) {
            const std::string_view text = argv[4];
            const auto parsed =
                std::from_chars(text.begin(), text.end(), center_frequency_hz);
            if (parsed.ec != std::errc{} || parsed.ptr != text.end()) {
                std::cerr << "Invalid raw I/Q center frequency: " << text
                          << '\n';
                return 2;
            }
        }
        return inspect_iq_cli(argv[2], sample_rate_hz, center_frequency_hz);
    }
    if (argc > 1 && std::string_view(argv[1]) == "--decode-iq") {
        std::uint32_t sample_rate_hz = 10'000'000;
        std::size_t decoder_threads = 0;
        bool debug = false;
        bool sample_rate_supplied = false;
        if (argc < 4) {
            std::cerr << "Usage: airspy-tv --decode-iq INPUT OUTPUT.ts "
                         "[SAMPLE_RATE_HZ] [--decoder-threads N] "
                         "[-d|--debug]\n";
            return 2;
        }
        for (int index = 4; index < argc; ++index) {
            const std::string_view text = argv[index];
            if (text == "-d" || text == "--debug") {
                debug = true;
                continue;
            }
            if (text == "--decoder-threads" || text == "--viterbi-threads") {
                if (++index >= argc) {
                    std::cerr << "Missing decoder thread count\n";
                    return 2;
                }
                const std::string_view count_text = argv[index];
                const auto parsed = std::from_chars(
                    count_text.begin(), count_text.end(), decoder_threads);
                if (parsed.ec != std::errc{} ||
                    parsed.ptr != count_text.end() || decoder_threads > 256) {
                    std::cerr << "Invalid decoder worker budget: " << count_text
                              << " (expected 0..256)\n";
                    return 2;
                }
                continue;
            }
            if (sample_rate_supplied) {
                std::cerr << "Unexpected --decode-iq argument: " << text
                          << '\n';
                return 2;
            }
            const auto parsed =
                std::from_chars(text.begin(), text.end(), sample_rate_hz);
            if (parsed.ec != std::errc{} || parsed.ptr != text.end() ||
                sample_rate_hz == 0) {
                std::cerr << "Invalid raw I/Q sample rate: " << text << '\n';
                return 2;
            }
            sample_rate_supplied = true;
        }
        return decode_iq_cli(argv[2], argv[3], sample_rate_hz, decoder_threads,
                             debug);
    }

    SDL_SetAppMetadata("Airspy TV", "0.1.0", "io.github.airspy-tv");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS,
                        SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    SDL_Window *window =
        SDL_CreateWindow("Airspy TV — DVB-T Receiver", 1500, 900,
                         SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE |
                             SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (window == nullptr) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == nullptr) {
        std::cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    std::string font_error;
    if (!load_system_monospace_font(io, font_error)) {
        std::cerr << "warning: " << font_error << '\n';
        io.Fonts->AddFontDefault();
    }
    apply_dark_theme();

    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    AppState state;
    state.demodulator = std::make_unique<StreamDecoder>();
    state.dvbt_demod = state.demodulator.get();
    state.dvbt_demod->set_parameters(state.dvbt_parameters);
    state.receiver.set_demodulator(std::move(state.demodulator));
    state.window = window;
    std::string player_error;
    if (!state.player.initialize(player_error)) {
        std::cerr << player_error << '\n';
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
        SDL_GL_DestroyContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    state.receiver.set_transport_sink(
        [&state](const std::span<const std::uint8_t> ts) {
            state.epg.consume(ts);
            state.player.submit(ts);
        });
    refresh_devices(state);
    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT ||
                event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
                running = false;
            }
        }

        const std::string runtime_error = state.receiver.runtime_error();
        if (!runtime_error.empty()) {
            state.status = runtime_error;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        draw_application(state);
        ImGui::Render();

        int pixel_width = 0;
        int pixel_height = 0;
        SDL_GetWindowSizeInPixels(window, &pixel_width, &pixel_height);
        glViewport(0, 0, pixel_width, pixel_height);
        glClearColor(0.025F, 0.03F, 0.04F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    state.receiver.set_transport_sink({});
    state.receiver.close();
    state.player.shutdown();
    state.waterfall.destroy();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
