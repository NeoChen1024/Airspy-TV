#include "app_state.hpp"
#include "panels.hpp"
#include "widgets.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_dialog.h>
#include <imgui.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace airspy_tv::gui {
namespace {

constexpr std::array<SDL_DialogFileFilter, 3> iq_source_filters{{
    {"I/Q metadata or raw INT16_IQ", "json;cs16;iq"},
    {"I/Q metadata", "json"},
    {"All files", "*"},
}};

void draw_source_gain_controls(AppState &state) {
    const DeviceDescriptor *descriptor =
        state.frame.descriptor ? &*state.frame.descriptor : nullptr;
    if (descriptor == nullptr || descriptor->backend == SdrBackend::File) {
        return;
    }

    ImGui::SeparatorText("SDR gain");
    if (descriptor->backend == SdrBackend::AirspyNative) {
        bool gain_changed = false;
        const int mode = state.source.settings.airspy_gain_mode ==
                                 AirspyGainMode::Sensitivity
                             ? 0
                             : 1;
        if (ImGui::RadioButton("Sensitivity", mode == 0)) {
            state.source.settings.airspy_gain_mode =
                AirspyGainMode::Sensitivity;
            gain_changed = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Linearity", mode == 1)) {
            state.source.settings.airspy_gain_mode = AirspyGainMode::Linearity;
            gain_changed = true;
        }
        gain_changed |= ImGui::SliderInt(
            "Profile gain", &state.source.settings.airspy_gain, 0, 21);
        if (gain_changed) {
            std::string error;
            state.ui.status =
                state.session.set_gain(state.source.settings, error)
                    ? "Airspy gain applied"
                    : error;
        }
        if (ImGui::Checkbox("Bias-T", &state.source.settings.bias_tee)) {
            std::string error;
            if (state.session.set_bias_tee(state.source.settings.bias_tee,
                                           error)) {
                state.ui.status = state.source.settings.bias_tee
                                      ? "Bias-T enabled"
                                      : "Bias-T disabled";
            } else {
                state.ui.status = error;
            }
        }
    } else if (const auto range = state.frame.gain_range; range.has_value()) {
        if (ImGui::SliderScalar("Generic gain", ImGuiDataType_Double,
                                &state.source.settings.soapy_gain,
                                &range->first, &range->second, "%.1f")) {
            std::string error;
            state.ui.status =
                state.session.set_gain(state.source.settings, error)
                    ? "Soapy gain applied"
                    : error;
        }
        draw_disabled_wrapped(
            "Soapy driver-defined gain; not comparable across devices.");
    }
}

} // namespace

void draw_source_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Source", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("source-panel");

    const bool source_open = state.frame.source_open;
    const DeviceDescriptor *source_descriptor =
        state.frame.descriptor ? &*state.frame.descriptor : nullptr;
    const bool file_source = source_descriptor != nullptr &&
                             source_descriptor->backend == SdrBackend::File;
    ImGui::SeparatorText("Frequency correction");
    if (file_source) {
        ImGui::BeginDisabled();
    }
    ImGui::TextDisabled("Hardware LO = nominal × (1 + ppm / 1e6)");
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::InputDouble("SDR correction (ppm)",
                           &state.source.settings.frequency_correction_ppm, 0.1,
                           1.0, "%.3f")) {
        if (!std::isfinite(state.source.settings.frequency_correction_ppm)) {
            state.source.settings.frequency_correction_ppm = 0.0;
        }
        state.source.settings.frequency_correction_ppm =
            std::clamp(state.source.settings.frequency_correction_ppm,
                       -airspy_tv::max_frequency_correction_ppm,
                       airspy_tv::max_frequency_correction_ppm);
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip(
            "Positive values tune the hardware LO higher than the nominal "
            "frequency. Applying a new value while streaming retunes the SDR "
            "and resets decoder tracking.");
    }
    if (ImGui::Button("Apply frequency correction", ImVec2(-1.0F, 0.0F))) {
        if (!source_open) {
            state.ui.status =
                "Frequency correction will apply when source starts";
        } else {
            std::string error;
            state.ui.status =
                state.session.set_frequency_correction_ppm(
                    state.source.settings.frequency_correction_ppm, error)
                    ? "Frequency correction applied"
                    : error;
        }
    }
    if (file_source) {
        ImGui::EndDisabled();
        draw_disabled_wrapped(
            "I/Q file frequency is fixed by its metadata; correction applies "
            "to live SDR sources only.");
    }

    std::optional<std::string> selected_iq_source;
    {
        const std::scoped_lock lock(state.source.iq_source_dialog->mutex);
        if (state.source.iq_source_dialog->selected_path.has_value()) {
            selected_iq_source =
                std::move(*state.source.iq_source_dialog->selected_path);
            state.source.iq_source_dialog->selected_path.reset();
        }
        if (state.source.iq_source_dialog->error.has_value()) {
            state.ui.status =
                "File dialog: " + *state.source.iq_source_dialog->error;
            state.source.iq_source_dialog->error.reset();
        }
    }
    if (selected_iq_source.has_value()) {
        state.ui.status =
            state.reporting.controller.open_iq_file(*selected_iq_source)
                .message;
    }

    const bool source_dialog_open =
        file_dialog_is_open(state.source.iq_source_dialog);
    ImGui::BeginDisabled(state.frame.source_open || source_dialog_open);
    const char *preview =
        state.source.enumeration.devices.empty()
            ? "No devices"
            : state.source.enumeration.devices[state.source.selected_device]
                  .display_name.c_str();
    ImGui::TextUnformatted("Device");
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::BeginCombo("##source-device", preview)) {
        for (std::size_t index = 0;
             index < state.source.enumeration.devices.size(); ++index) {
            const bool selected = index == state.source.selected_device;
            if (ImGui::Selectable(state.source.enumeration.devices[index]
                                      .display_name.c_str(),
                                  selected)) {
                state.source.selected_device = index;
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
    ImGui::Checkbox("Show Airspy through Soapy",
                    &state.source.show_soapy_airspy);
    ImGui::SameLine();
    if (ImGui::SmallButton("Refresh")) {
        refresh_devices(state);
    }
    ImGui::EndDisabled();

    if (!state.frame.source_open) {
        ImGui::BeginDisabled(state.source.enumeration.devices.empty());
        if (ImGui::Button("Open device", ImVec2(-1.0F, 0.0F))) {
            const DeviceDescriptor &descriptor =
                state.source.enumeration.devices[state.source.selected_device];
            state.ui.status =
                state.reporting.controller.open_device(descriptor).message;
        }
        ImGui::EndDisabled();
        ImGui::BeginDisabled(source_dialog_open);
        if (ImGui::Button("Open I/Q file...", ImVec2(-1.0F, 0.0F))) {
            {
                const std::scoped_lock lock(
                    state.source.iq_source_dialog->mutex);
                state.source.iq_source_dialog->open = true;
                state.source.iq_source_dialog->default_location = ".";
            }
            auto callback_state =
                std::make_unique<std::shared_ptr<FileDialogState>>(
                    state.source.iq_source_dialog);
            SDL_ShowOpenFileDialog(
                &file_dialog_callback, callback_state.release(),
                state.ui.window, iq_source_filters.data(),
                static_cast<int>(iq_source_filters.size()), ".", false);
        }
        ImGui::EndDisabled();
        ImGui::TextUnformatted("Raw sample rate (Hz)");
        ImGui::SetNextItemWidth(-1.0F);
        ImGui::InputScalar("##raw-sample-rate", ImGuiDataType_U32,
                           &state.source.settings.sample_rate_hz);
        draw_disabled_wrapped(std::format(
            "JSON supplies metadata; raw INT16_IQ uses {:.3f} MSPS / {:.3f} "
            "MHz.",
            static_cast<double>(state.source.settings.sample_rate_hz) / 1e6,
            static_cast<double>(state.source.settings.center_frequency_hz) /
                1e6));
    } else {
        const DeviceDescriptor *descriptor = &*state.frame.descriptor;
        ImGui::TextColored(ImVec4(0.35F, 0.88F, 0.55F, 1.0F), "OPEN");
        ImGui::SameLine();
        ImGui::TextWrapped("%s", descriptor->display_name.c_str());
        if (!descriptor->serial.empty()) {
            draw_disabled_wrapped("Serial: " + descriptor->serial);
        }
        ImGui::BeginDisabled(state.frame.iq_recording);
        if (ImGui::Button(descriptor->backend == SdrBackend::File
                              ? "Close I/Q file"
                              : "Close device",
                          ImVec2(-1.0F, 0.0F))) {
            state.ui.status = state.reporting.controller.close().message;
        }
        ImGui::EndDisabled();

        if (descriptor->backend == SdrBackend::File) {
            ImGui::Text("Format       CS16 / INT16_IQ");
            ImGui::Text(
                "Sample rate  %.3f MSPS",
                static_cast<double>(state.source.settings.sample_rate_hz) /
                    1e6);
            ImGui::Text(
                "Center       %.6f MHz",
                static_cast<double>(state.source.settings.center_frequency_hz) /
                    1e6);
            if (!state.frame.source_streaming &&
                ImGui::Button("Replay from beginning", ImVec2(-1.0F, 0.0F))) {
                state.ui.status = state.reporting.controller
                                      .restart_stream("Replaying I/Q file")
                                      .message;
            }
        } else {
            ImGui::BeginDisabled(state.frame.iq_recording);
            if (!state.frame.sample_rates.empty()) {
                const std::string rate_preview = std::format(
                    "{:.3f} MSPS",
                    static_cast<double>(state.source.settings.sample_rate_hz) /
                        1e6);
                ImGui::TextUnformatted("Sample rate");
                ImGui::SetNextItemWidth(-1.0F);
                if (ImGui::BeginCombo("##sample-rate", rate_preview.c_str())) {
                    for (const std::uint32_t rate : state.frame.sample_rates) {
                        const std::string label = std::format(
                            "{:.3f} MSPS", static_cast<double>(rate) / 1e6);
                        if (ImGui::Selectable(
                                label.c_str(),
                                rate == state.source.settings.sample_rate_hz)) {
                            state.source.settings.sample_rate_hz = rate;
                        }
                    }
                    ImGui::EndCombo();
                }
            }
            if (ImGui::Button("Apply sample rate", ImVec2(-1.0F, 0.0F))) {
                state.ui.status = state.reporting.controller
                                      .restart_stream("Sample rate applied")
                                      .message;
            }
            ImGui::EndDisabled();
        }
    }

    draw_source_gain_controls(state);

    for (const std::string &warning : state.source.enumeration.warnings) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.64F, 0.25F, 1.0F));
        ImGui::TextWrapped("%s", warning.c_str());
        ImGui::PopStyleColor();
    }
    ImGui::PopID();
}

} // namespace airspy_tv::gui
