#include "app_state.hpp"
#include "decode_reporting.hpp"
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
    const DeviceDescriptor *descriptor = state.session.descriptor();
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
            state.status = state.session.set_gain(state.settings, error)
                               ? "Airspy gain applied"
                               : error;
        }
        if (ImGui::Checkbox("Bias-T", &state.settings.bias_tee)) {
            std::string error;
            if (state.session.set_bias_tee(state.settings.bias_tee, error)) {
                state.status = state.settings.bias_tee ? "Bias-T enabled"
                                                       : "Bias-T disabled";
            } else {
                state.status = error;
            }
        }
    } else if (const auto range = state.session.gain_range();
               range.has_value()) {
        if (ImGui::SliderScalar("Generic gain", ImGuiDataType_Double,
                                &state.settings.soapy_gain, &range->first,
                                &range->second, "%.1f")) {
            std::string error;
            state.status = state.session.set_gain(state.settings, error)
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

    const bool source_open = state.session.is_open();
    const DeviceDescriptor *source_descriptor = state.session.descriptor();
    const bool file_source = source_descriptor != nullptr &&
                             source_descriptor->backend == SdrBackend::File;
    ImGui::SeparatorText("Frequency correction");
    if (file_source) {
        ImGui::BeginDisabled();
    }
    ImGui::TextDisabled("Hardware LO = nominal × (1 + ppm / 1e6)");
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::InputDouble("SDR correction (ppm)",
                           &state.settings.frequency_correction_ppm, 0.1, 1.0,
                           "%.3f")) {
        if (!std::isfinite(state.settings.frequency_correction_ppm)) {
            state.settings.frequency_correction_ppm = 0.0;
        }
        state.settings.frequency_correction_ppm =
            std::clamp(state.settings.frequency_correction_ppm,
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
            state.status = "Frequency correction will apply when source starts";
        } else {
            std::string error;
            state.status = state.session.set_frequency_correction_ppm(
                               state.settings.frequency_correction_ppm, error)
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
        prepare_decode_report(state);
        if (state.session.open_iq_file_and_start(*selected_iq_source,
                                                 state.settings, error)) {
            state.status =
                "Playing I/Q from " +
                std::filesystem::path(*selected_iq_source).filename().string();
            std::string report_error;
            if (!start_decode_report(state, report_error)) {
                state.status += "; report unavailable: " + report_error;
            }
        } else {
            cancel_decode_report_start(state);
            state.status = error;
        }
    }

    const bool source_dialog_open = file_dialog_is_open(state.iq_source_dialog);
    ImGui::BeginDisabled(state.session.is_open() || source_dialog_open);
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

    if (!state.session.is_open()) {
        ImGui::BeginDisabled(state.enumeration.devices.empty());
        if (ImGui::Button("Open device", ImVec2(-1.0F, 0.0F))) {
            std::string error;
            const DeviceDescriptor &descriptor =
                state.enumeration.devices[state.selected_device];
            prepare_decode_report(state);
            if (state.session.open_device_and_start(descriptor, state.settings,
                                                    error)) {
                state.status = "Receiving from " + descriptor.display_name;
                std::string report_error;
                if (!start_decode_report(state, report_error)) {
                    state.status += "; report unavailable: " + report_error;
                }
            } else {
                cancel_decode_report_start(state);
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
                &file_dialog_callback, callback_state.release(), state.window,
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
        const DeviceDescriptor *descriptor = state.session.descriptor();
        ImGui::TextColored(ImVec4(0.35F, 0.88F, 0.55F, 1.0F), "OPEN");
        ImGui::SameLine();
        ImGui::TextWrapped("%s", descriptor->display_name.c_str());
        if (!descriptor->serial.empty()) {
            draw_disabled_wrapped("Serial: " + descriptor->serial);
        }
        ImGui::BeginDisabled(state.session.is_recording());
        if (ImGui::Button(descriptor->backend == SdrBackend::File
                              ? "Close I/Q file"
                              : "Close device",
                          ImVec2(-1.0F, 0.0F))) {
            finish_decode_report_source(state);
            state.session.close();
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
            if (!state.session.is_streaming() &&
                ImGui::Button("Replay from beginning", ImVec2(-1.0F, 0.0F))) {
                std::string error;
                prepare_decode_report(state);
                if (state.session.start_stream(state.settings, error)) {
                    state.status = "Replaying I/Q file";
                    std::string report_error;
                    if (!start_decode_report(state, report_error)) {
                        state.status += "; report unavailable: " + report_error;
                    }
                } else {
                    cancel_decode_report_start(state);
                    state.status = error;
                }
            }
        } else {
            ImGui::BeginDisabled(state.session.is_recording());
            if (!state.session.sample_rates().empty()) {
                const std::string rate_preview = std::format(
                    "{:.3f} MSPS",
                    static_cast<double>(state.settings.sample_rate_hz) / 1e6);
                ImGui::TextUnformatted("Sample rate");
                ImGui::SetNextItemWidth(-1.0F);
                if (ImGui::BeginCombo("##sample-rate", rate_preview.c_str())) {
                    for (const std::uint32_t rate :
                         state.session.sample_rates()) {
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
                finish_decode_report_source(state);
                prepare_decode_report(state);
                if (state.session.start_stream(state.settings, error)) {
                    state.status = "Sample rate applied";
                    std::string report_error;
                    if (!start_decode_report(state, report_error)) {
                        state.status += "; report unavailable: " + report_error;
                    }
                } else {
                    cancel_decode_report_start(state);
                    state.status = error;
                }
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

} // namespace airspy_tv::gui
