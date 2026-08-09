#include "app_state.hpp"
#include "panels.hpp"
#include "widgets.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <string>

namespace airspy_tv::gui {

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
            standard_names[static_cast<std::size_t>(state.frame.standard)])) {
        for (std::size_t index = 0; index < standard_names.size(); ++index) {
            const bool implemented =
                static_cast<ReceiveStandard>(index) == ReceiveStandard::DvbT;
            if (!implemented) {
                ImGui::BeginDisabled(true);
            }
            if (ImGui::Selectable(standard_names[index],
                                  state.frame.standard ==
                                      static_cast<ReceiveStandard>(index))) {
                std::string error;
                if (!state.session.select_standard(
                        static_cast<ReceiveStandard>(index),
                        state.source.settings, true, error)) {
                    state.ui.status = error;
                }
            }
            if (!implemented) {
                ImGui::EndDisabled();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::PopID();
    ImGui::Separator();

    draw_standard_settings_panel(state);
    ImGui::PopID();
}

void draw_common_signal_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Signal quality",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("common-signal-panel");

    const ImVec4 lock_colour = state.frame.signal.signal_locked
                                   ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                                   : ImVec4(1.0F, 0.38F, 0.25F, 1.0F);
    draw_status_indicator(state.frame.signal.signal_locked ? "SIGNAL LOCKED"
                                                           : "SIGNAL UNLOCKED",
                          lock_colour);
    const ImVec4 transport_colour = state.frame.signal.transport_locked
                                        ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                                        : ImVec4(0.95F, 0.72F, 0.30F, 1.0F);
    draw_status_indicator(state.frame.signal.transport_locked
                              ? "TRANSPORT LOCKED"
                              : "TRANSPORT UNLOCKED",
                          transport_colour);

    const char *pipeline_status = "PIPELINE LOAD";
    ImVec4 pipeline_colour{0.55F, 0.62F, 0.70F, 1.0F};
    if (state.frame.pipeline.failed) {
        pipeline_status = "PIPELINE FAILED";
        pipeline_colour = ImVec4(1.0F, 0.38F, 0.25F, 1.0F);
    } else if (state.frame.pipeline_load == PipelineLoadState::overload) {
        pipeline_status = "PIPELINE OVERLOAD";
        pipeline_colour = ImVec4(1.0F, 0.38F, 0.25F, 1.0F);
    } else if (state.frame.pipeline_load == PipelineLoadState::slow) {
        pipeline_status = "PIPELINE SLOW";
        pipeline_colour = ImVec4(1.0F, 0.72F, 0.22F, 1.0F);
    } else if (state.frame.pipeline_load == PipelineLoadState::realtime) {
        pipeline_status = "PIPELINE REALTIME";
        pipeline_colour = ImVec4(0.35F, 0.88F, 0.55F, 1.0F);
    }
    draw_status_indicator(pipeline_status, pipeline_colour);
    if (state.frame.pipeline.failed && !state.frame.pipeline.error.empty()) {
        ImGui::TextWrapped("%s", state.frame.pipeline.error.c_str());
    }

    const auto format_percent = [](const float fraction) {
        return std::format("{:3.0f}%",
                           std::clamp(fraction * 100.0F, 0.0F, 100.0F));
    };
    if (ImGui::BeginTable("common-pipeline-stages", 2,
                          ImGuiTableFlags_SizingStretchProp |
                              ImGuiTableFlags_PadOuterX)) {
        ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                                72.0F);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
        for (std::size_t index = 0; index < state.frame.pipeline.stage_count;
             ++index) {
            const auto &stage = state.frame.pipeline.stages[index];
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%.*s", static_cast<int>(stage.name.size()),
                                stage.name.data());
            ImGui::TableNextColumn();
            if (stage.queue_valid) {
                ImGui::TextUnformatted(
                    format_percent(stage.queue_fraction).c_str());
            } else if (stage.busy_valid) {
                ImGui::TextUnformatted(
                    format_percent(stage.busy_fraction).c_str());
            } else {
                ImGui::TextUnformatted(" --%");
            }
        }
        ImGui::TableNextColumn();
        ImGui::TextDisabled("Drops");
        ImGui::TableNextColumn();
        ImGui::Text("%llu", static_cast<unsigned long long>(
                                state.frame.pipeline.dropped_blocks));
        ImGui::EndTable();
    }
    ImGui::Separator();

    const std::string power =
        state.frame.spectrum.valid
            ? std::format("{:.1f} dBFS", state.frame.spectrum.signal_power_dbfs)
            : "-- dBFS";
    const float power_fraction =
        state.frame.spectrum.valid
            ? std::clamp(
                  (state.frame.spectrum.signal_power_dbfs -
                   signal_meter_floor_dbfs) /
                      (signal_meter_ceiling_dbfs - signal_meter_floor_dbfs),
                  0.0F, 1.0F)
            : 0.0F;
    draw_metric("Signal power", power.c_str(), power_fraction,
                ImVec4(0.35F, 0.78F, 0.95F, 1.0F));

    const bool has_snr = state.frame.signal.signal_locked ||
                         state.frame.spectrum.channel_metrics_valid;
    const float snr_value = state.frame.signal.signal_locked
                                ? state.frame.signal.snr_db
                                : state.frame.spectrum.rf_snr_db;
    const std::string snr =
        has_snr ? std::format("{:.1f} dB", snr_value) : "-- dB";
    draw_metric(
        state.frame.signal.signal_locked ? "Demod SNR est." : "RF SNR est.",
        snr.c_str(),
        has_snr ? std::clamp((snr_value + 5.0F) / 40.0F, 0.0F, 1.0F) : 0.0F,
        ImVec4(0.35F, 0.88F, 0.55F, 1.0F));

    const bool has_notch = state.frame.signal.signal_locked ||
                           state.frame.spectrum.channel_metrics_valid;
    const float notch_value = state.frame.signal.signal_locked
                                  ? state.frame.signal.deepest_notch_db
                                  : state.frame.spectrum.deepest_notch_db;
    const std::string notch =
        has_notch ? std::format("{:.1f} dB", notch_value) : "-- dB";
    draw_metric("Deepest notch", notch.c_str(),
                has_notch ? std::clamp(1.0F + (notch_value / 40.0F), 0.0F, 1.0F)
                          : 0.0F,
                ImVec4(0.35F, 0.78F, 0.95F, 1.0F));

    const std::string carrier_offset =
        state.frame.signal.signal_locked
            ? std::format("{:+.2f} kHz",
                          state.frame.signal.carrier_offset_hz / 1000.0F)
            : "-- kHz";
    const float carrier_position =
        state.frame.signal.signal_locked &&
                state.frame.signal.carrier_offset_limit_hz > 0.0F
            ? std::clamp(
                  0.5F +
                      state.frame.signal.carrier_offset_hz /
                          (2.0F * state.frame.signal.carrier_offset_limit_hz),
                  0.0F, 1.0F)
            : 0.5F;
    draw_bipolar_metric("Carrier offset", carrier_offset.c_str(),
                        carrier_position, ImVec4(0.52F, 0.82F, 1.0F, 1.0F));

    const std::string mer =
        state.frame.signal.signal_locked
            ? std::format("{:.1f} dB", state.frame.signal.mer_db)
            : "-- dB";
    draw_metric("MER", mer.c_str(),
                state.frame.signal.signal_locked
                    ? std::clamp(state.frame.signal.mer_db / 40.0F, 0.0F, 1.0F)
                    : 0.0F,
                ImVec4(0.35F, 0.78F, 0.95F, 1.0F));
    ImGui::PopID();
}

} // namespace airspy_tv::gui
