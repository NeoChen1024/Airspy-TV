#include "airspy_tv/sdr.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <iostream>
#include <ranges>
#include <string>
#include <string_view>
#include <thread>

namespace {

using airspy_tv::AirspyGainMode;
using airspy_tv::DeviceDescriptor;
using airspy_tv::EnumerationResult;
using airspy_tv::SdrBackend;
using airspy_tv::SdrDevice;
using airspy_tv::SourceSettings;

constexpr ImVec4 accent{0.12F, 0.58F, 0.92F, 1.0F};
constexpr float panel_width = 410.0F;

struct AppState {
    SdrDevice receiver;
    EnumerationResult enumeration;
    SourceSettings settings;
    std::size_t selected_device{};
    bool show_soapy_airspy{};
    std::string status{"Ready"};
    std::array<char, 512> recording_path{};

    AppState() {
        constexpr std::string_view default_path = "capture.cs16";
        std::copy(default_path.begin(), default_path.end(),
                  recording_path.begin());
    }
};

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

void draw_spectrum(const ImVec2 size) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    ImGui::InvisibleButton("spectrum-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(4, 9, 15, 255), 4.0F);

    for (int index = 1; index < 6; ++index) {
        const float y =
            origin.y + (canvas_size.y * static_cast<float>(index) / 6.0F);
        draw->AddLine(ImVec2(origin.x, y), ImVec2(extent.x, y),
                      IM_COL32(38, 51, 65, 160));
    }
    for (int index = 1; index < 8; ++index) {
        const float x =
            origin.x + (canvas_size.x * static_cast<float>(index) / 8.0F);
        draw->AddLine(ImVec2(x, origin.y), ImVec2(x, extent.y),
                      IM_COL32(38, 51, 65, 160));
    }

    ImVec2 previous{origin.x, extent.y - 20.0F};
    for (int index = 0; index <= 128; ++index) {
        const float phase = static_cast<float>(index) / 128.0F;
        const float carrier =
            std::exp(-900.0F * (phase - 0.58F) * (phase - 0.58F));
        const float ripple = (0.06F * std::sin(phase * 81.0F)) +
                             (0.035F * std::sin(phase * 211.0F));
        const float level =
            std::clamp(0.22F + ripple + (carrier * 0.62F), 0.05F, 0.95F);
        const ImVec2 point{origin.x + (phase * canvas_size.x),
                           extent.y - (level * canvas_size.y)};
        if (index != 0) {
            draw->AddLine(previous, point, IM_COL32(45, 205, 235, 255), 1.4F);
        }
        previous = point;
    }
    draw->AddText(ImVec2(origin.x + 8.0F, origin.y + 6.0F),
                  IM_COL32(150, 167, 184, 255), "RF spectrum — mock data");
}

ImU32 waterfall_colour(const float value) {
    const float clamped = std::clamp(value, 0.0F, 1.0F);
    const int red = static_cast<int>(25.0F + (190.0F * clamped * clamped));
    const int green = static_cast<int>(8.0F + (210.0F * clamped));
    const int blue =
        static_cast<int>(55.0F + (180.0F * (1.0F - std::abs(clamped - 0.55F))));
    return IM_COL32(red, green, blue, 255);
}

void draw_waterfall(const ImVec2 size) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    ImGui::InvisibleButton("waterfall-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    ImDrawList *draw = ImGui::GetWindowDrawList();
    constexpr int columns = 48;
    constexpr int rows = 24;
    const float cell_width = canvas_size.x / static_cast<float>(columns);
    const float cell_height = canvas_size.y / static_cast<float>(rows);
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            const float x =
                static_cast<float>(column) / static_cast<float>(columns);
            const float y = static_cast<float>(row) / static_cast<float>(rows);
            const float carrier = std::exp(-900.0F * (x - 0.58F) * (x - 0.58F));
            const float texture =
                0.16F + (0.11F * std::sin((x * 93.0F) + (y * 17.0F))) +
                (0.05F * std::sin((x * 271.0F) - (y * 31.0F)));
            const float value = std::clamp(
                texture + (carrier * (0.62F + (0.18F * std::sin(y * 28.0F)))),
                0.0F, 1.0F);
            const ImVec2 low{
                origin.x + (static_cast<float>(column) * cell_width),
                origin.y + (static_cast<float>(row) * cell_height)};
            draw->AddRectFilled(
                low,
                ImVec2(low.x + cell_width + 0.5F, low.y + cell_height + 0.5F),
                waterfall_colour(value));
        }
    }
    draw->AddText(ImVec2(origin.x + 8.0F, origin.y + 6.0F), IM_COL32_WHITE,
                  "Waterfall — mock data");
}

void draw_constellation(const ImVec2 size) {
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
    constexpr std::array<float, 4> levels{-0.72F, -0.24F, 0.24F, 0.72F};
    int point_index = 0;
    for (const float i_level : levels) {
        for (const float q_level : levels) {
            for (int sample = 0; sample < 5; ++sample, ++point_index) {
                const float jitter_x =
                    std::sin(static_cast<float>(point_index) * 12.9898F) * 2.4F;
                const float jitter_y =
                    std::sin(static_cast<float>(point_index) * 78.233F) * 2.4F;
                const ImVec2 point{
                    center.x + (i_level * canvas_size.x * 0.43F) + jitter_x,
                    center.y - (q_level * canvas_size.y * 0.43F) + jitter_y};
                draw->AddCircleFilled(point, 2.0F, IM_COL32(70, 220, 255, 210));
            }
        }
    }
    draw->AddText(ImVec2(origin.x + 8.0F, origin.y + 6.0F),
                  IM_COL32(150, 167, 184, 255), "16-QAM — mock data");
}

void draw_metric(const char *label, const char *value, const float fraction,
                 const ImVec4 colour) {
    ImGui::TextUnformatted(label);
    ImGui::SameLine(115.0F);
    ImGui::TextColored(colour, "%s", value);
    ImGui::ProgressBar(fraction, ImVec2(-1.0F, 5.0F), "");
}

void draw_source_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Source", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    ImGui::BeginDisabled(state.receiver.is_open());
    const char *preview = state.enumeration.devices.empty()
                              ? "No devices"
                              : state.enumeration.devices[state.selected_device]
                                    .display_name.c_str();
    if (ImGui::BeginCombo("Device", preview)) {
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
                state.status = "Opened " + descriptor.display_name;
            } else {
                state.status = error;
            }
        }
        ImGui::EndDisabled();
    } else {
        const DeviceDescriptor *descriptor = state.receiver.descriptor();
        ImGui::TextColored(ImVec4(0.35F, 0.88F, 0.55F, 1.0F), "OPEN");
        ImGui::SameLine();
        ImGui::TextWrapped("%s", descriptor->display_name.c_str());
        if (!descriptor->serial.empty()) {
            ImGui::TextDisabled("Serial: %s", descriptor->serial.c_str());
        }
        ImGui::BeginDisabled(state.receiver.is_recording());
        if (ImGui::Button("Close device", ImVec2(-1.0F, 0.0F))) {
            state.receiver.close();
            state.status = "Device closed";
        }
        ImGui::EndDisabled();
    }

    for (const std::string &warning : state.enumeration.warnings) {
        ImGui::TextColored(ImVec4(1.0F, 0.64F, 0.25F, 1.0F), "%s",
                           warning.c_str());
    }
}

void draw_receiver_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Receiver", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    ImGui::BeginDisabled(!state.receiver.is_open() ||
                         state.receiver.is_recording());
    ImGui::InputScalar("Center frequency (Hz)", ImGuiDataType_U64,
                       &state.settings.center_frequency_hz);

    if (!state.receiver.sample_rates().empty()) {
        const std::string preview = std::format(
            "{:.3f} MSPS",
            static_cast<double>(state.settings.sample_rate_hz) / 1e6);
        if (ImGui::BeginCombo("Sample rate", preview.c_str())) {
            for (const std::uint32_t rate : state.receiver.sample_rates()) {
                const std::string label =
                    std::format("{:.3f} MSPS", static_cast<double>(rate) / 1e6);
                if (ImGui::Selectable(label.c_str(),
                                      rate == state.settings.sample_rate_hz)) {
                    state.settings.sample_rate_hz = rate;
                }
            }
            ImGui::EndCombo();
        }
    }

    const DeviceDescriptor *descriptor = state.receiver.descriptor();
    if (descriptor != nullptr &&
        descriptor->backend == SdrBackend::AirspyNative) {
        const int mode =
            state.settings.airspy_gain_mode == AirspyGainMode::Sensitivity ? 0
                                                                           : 1;
        if (ImGui::RadioButton("Sensitivity", mode == 0)) {
            state.settings.airspy_gain_mode = AirspyGainMode::Sensitivity;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Linearity", mode == 1)) {
            state.settings.airspy_gain_mode = AirspyGainMode::Linearity;
        }
        ImGui::SliderInt("Profile gain", &state.settings.airspy_gain, 0, 21);
        ImGui::Checkbox("Bias-T", &state.settings.bias_tee);
    } else if (const auto range = state.receiver.gain_range();
               range.has_value()) {
        ImGui::SliderScalar("Generic gain", ImGuiDataType_Double,
                            &state.settings.soapy_gain, &range->first,
                            &range->second, "%.1f");
        ImGui::TextDisabled(
            "Soapy driver-defined gain; not comparable across devices.");
    }

    if (ImGui::Button("Apply receiver settings", ImVec2(-1.0F, 0.0F))) {
        std::string error;
        state.status = state.receiver.configure(state.settings, error)
                           ? "Receiver settings applied"
                           : error;
    }
    ImGui::EndDisabled();
}

void draw_recorder_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Raw I/Q Recorder",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::TextDisabled("Interleaved signed 16-bit little-endian I/Q (CS16)");
    ImGui::BeginDisabled(state.receiver.is_recording());
    ImGui::InputText("Output", state.recording_path.data(),
                     state.recording_path.size());
    ImGui::EndDisabled();

    if (!state.receiver.is_recording()) {
        ImGui::BeginDisabled(!state.receiver.is_open());
        if (ImGui::Button("Start recording", ImVec2(-1.0F, 0.0F))) {
            std::string error;
            if (state.receiver.start_recording(
                    std::filesystem::path(state.recording_path.data()),
                    state.settings, error)) {
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
    ImGui::Text("Samples: %s",
                std::format("{}", stats.complex_samples).c_str());
    ImGui::Text("Written: %.2f MiB",
                static_cast<double>(stats.bytes_written) / (1024.0 * 1024.0));
    ImGui::Text("Queue drops: %llu",
                static_cast<unsigned long long>(stats.dropped_blocks));
    ImGui::Text("Source drops: %llu",
                static_cast<unsigned long long>(stats.source_dropped_samples));
}

void draw_sidebar(AppState &state) {
    draw_source_panel(state);
    draw_receiver_panel(state);

    if (ImGui::CollapsingHeader("Spectrum & Waterfall",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        draw_spectrum(ImVec2(-1.0F, 150.0F));
        draw_waterfall(ImVec2(-1.0F, 150.0F));
    }

    draw_recorder_panel(state);

    if (ImGui::CollapsingHeader("DVB-T Constellation",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        draw_constellation(ImVec2(-1.0F, 300.0F));
    }

    if (ImGui::CollapsingHeader("Signal Quality",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        draw_metric("SNR", "27.4 dB", 0.76F, ImVec4(0.35F, 0.88F, 0.55F, 1.0F));
        draw_metric("MER", "25.8 dB", 0.70F, ImVec4(0.35F, 0.78F, 0.95F, 1.0F));
        draw_metric("Viterbi BER", "1.2e-5", 0.91F,
                    ImVec4(0.75F, 0.72F, 0.30F, 1.0F));
        draw_metric("Post-RS BER", "0.0", 1.0F,
                    ImVec4(0.35F, 0.88F, 0.55F, 1.0F));
        ImGui::Separator();
        ImGui::Text("Mode           8K");
        ImGui::Text("Guard          1/4");
        ImGui::Text("Modulation     16-QAM");
        ImGui::Text("Code rate      3/4");
        ImGui::TextDisabled("Mock decoder telemetry");
    }
}

void draw_video_panel() {
    const ImVec2 available = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("video-surface", available);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(5, 8, 13, 255), 5.0F);

    const float header_height = 46.0F;
    draw->AddRectFilled(origin, ImVec2(extent.x, origin.y + header_height),
                        IM_COL32(15, 24, 35, 255), 5.0F);
    draw->AddText(ImVec2(origin.x + 16.0F, origin.y + 14.0F),
                  IM_COL32(220, 230, 240, 255), "DVB-T PROGRAM 01");
    draw->AddText(ImVec2(extent.x - 165.0F, origin.y + 14.0F),
                  IM_COL32(90, 205, 255, 255), "VIDEO PREVIEW");

    const ImVec2 center{(origin.x + extent.x) * 0.5F,
                        (origin.y + extent.y) * 0.5F};
    draw->AddCircle(center, 54.0F, IM_COL32(55, 78, 102, 255), 0, 2.0F);
    draw->AddTriangleFilled(ImVec2(center.x - 14.0F, center.y - 24.0F),
                            ImVec2(center.x - 14.0F, center.y + 24.0F),
                            ImVec2(center.x + 28.0F, center.y),
                            IM_COL32(65, 175, 235, 230));
    const char *message = "libmpv video surface placeholder";
    const ImVec2 text_size = ImGui::CalcTextSize(message);
    draw->AddText(ImVec2(center.x - (text_size.x * 0.5F), center.y + 74.0F),
                  IM_COL32(130, 150, 170, 255), message);

    const float footer_height = 72.0F;
    const ImVec2 footer_origin{origin.x, extent.y - footer_height};
    draw->AddRectFilled(footer_origin, extent, IM_COL32(13, 20, 30, 245), 5.0F);
    draw->AddText(ImVec2(origin.x + 18.0F, footer_origin.y + 13.0F),
                  IM_COL32_WHITE, "Program 01   1920x1080i   H.264   AAC");
    draw->AddText(ImVec2(origin.x + 18.0F, footer_origin.y + 40.0F),
                  IM_COL32(145, 160, 176, 255),
                  "PAT / PMT / SDT service selection will appear here");
}

void draw_application(AppState &state) {
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
    ImGui::SetNextItemWidth(235.0F);
    ImGui::InputScalar("##frequency", ImGuiDataType_U64,
                       &state.settings.center_frequency_hz);
    ImGui::SameLine();
    ImGui::TextDisabled("6 MHz DVB-T");
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
    draw_video_panel();
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
    std::this_thread::sleep_for(
        std::chrono::milliseconds(std::max(duration_ms, 1)));
    receiver.stop_recording();
    const auto stats = receiver.recording_stats();
    std::cout << "Recorded " << stats.complex_samples << " complex samples ("
              << stats.bytes_written
              << " bytes), queue drops=" << stats.dropped_blocks
              << ", source drops=" << stats.source_dropped_samples << '\n';
    return stats.bytes_written == 0 ? 1 : 0;
}

} // namespace

int main(const int argc, char **argv) {
    if (argc > 1 && std::string_view(argv[1]) == "--enumerate") {
        return enumerate_cli();
    }
    if (argc > 1 && std::string_view(argv[1]) == "--help") {
        std::cout << "Usage: airspy-tv [--enumerate|--record-first PATH "
                     "[MILLISECONDS]|--help]\n";
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
    apply_dark_theme();

    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    AppState state;
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

    state.receiver.close();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
