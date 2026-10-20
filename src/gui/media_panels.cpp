#include "app_state.hpp"
#include "panels.hpp"
#include "widgets.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_dialog.h>
#include <imgui.h>
#include <imgui_stdlib.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <format>
#include <ranges>
#include <span>
#include <string>
#include <string_view>

namespace airspy_tv::gui {
namespace {

constexpr std::array<SDL_DialogFileFilter, 2> recording_filters{{
    {"Raw interleaved I/Q", "cs16;iq"},
    {"All files", "*"},
}};
constexpr std::array<SDL_DialogFileFilter, 2> transport_stream_filters{{
    {"MPEG transport stream", "ts;m2ts"},
    {"All files", "*"},
}};

std::string format_recording_duration(const std::uint64_t milliseconds) {
    const std::uint64_t total_seconds = milliseconds / 1000;
    const std::uint64_t hours = total_seconds / 3600;
    const std::uint64_t minutes = (total_seconds / 60) % 60;
    const std::uint64_t seconds = total_seconds % 60;
    const std::uint64_t tenths = (milliseconds % 1000) / 100;
    return std::format("{:02}:{:02}:{:02}.{}", hours, minutes, seconds, tenths);
}

template <typename Stats>
void draw_recorder_write_stats(ByteRateTracker &rate_tracker,
                               const Stats &stats) {
    const double write_mib_per_second =
        rate_tracker.update(stats.active, stats.bytes_written);
    ImGui::Text("Duration: %s",
                format_recording_duration(stats.elapsed_milliseconds).c_str());
    ImGui::Text("Written: %.2f MiB",
                static_cast<double>(stats.bytes_written) / (1024.0 * 1024.0));
    ImGui::Text("Write rate: %.2f MiB/s", write_mib_per_second);
    ImGui::Text("Queue drops: %llu",
                static_cast<unsigned long long>(stats.dropped_blocks));
    ImGui::Text("Write errors: %llu",
                static_cast<unsigned long long>(stats.write_errors));
    if (!stats.error.empty()) {
        ImGui::TextWrapped("Last write error: %s", stats.error.c_str());
    }
}

} // namespace

void draw_recorder_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Raw I/Q Recorder")) {
        return;
    }
    ImGui::PushID("iq-recorder");
    consume_file_dialog_result(state, state.output.file_dialog,
                               state.output.recording_path, "I/Q recording");
    const bool dialog_open = file_dialog_is_open(state.output.file_dialog);
    draw_disabled_wrapped("Interleaved signed 16-bit little-endian I/Q (CS16)");
    ImGui::TextUnformatted("Output file");
    ImGui::BeginDisabled(state.frame.iq_recording || dialog_open);
    ImGui::SetNextItemWidth(-92.0F);
    ImGui::InputText("##recording-output", &state.output.recording_path);
    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        show_recording_file_dialog(state, state.output.file_dialog,
                                   state.output.recording_path,
                                   recording_filters);
    }
    ImGui::EndDisabled();
    if (dialog_open) {
        ImGui::TextDisabled("Waiting for file selection...");
    }

    if (!state.frame.iq_recording) {
        const bool file_source =
            state.frame.descriptor.has_value() &&
            state.frame.descriptor->backend == SdrBackend::File;
        ImGui::BeginDisabled(!state.frame.source_open || file_source ||
                             dialog_open ||
                             state.output.recording_path.empty());
        if (ImGui::Button("Start recording", ImVec2(-1.0F, 0.0F))) {
            state.ui.status =
                state.reporting.controller
                    .start_iq_recording(state.output.recording_path)
                    .message;
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop recording", ImVec2(-1.0F, 0.0F))) {
        state.ui.status =
            state.reporting.controller.stop_iq_recording().message;
    }

    const auto stats = state.frame.iq_recording_stats;
    draw_recorder_write_stats(state.output.iq_write_rate, stats);
    ImGui::Text("Source drops: %llu",
                static_cast<unsigned long long>(stats.source_dropped_samples));
    ImGui::PopID();
}

void draw_epg_panel(AppState &state) {
    ImGui::PushID("epg-panel");
    ImGui::TextColored(ImVec4(0.59F, 0.65F, 0.72F, 1.0F), "EPG");
    ImGui::SameLine();
    if (state.frame.services.empty()) {
        draw_disabled_wrapped("No services detected");
        ImGui::PopID();
        return;
    }
    // The EPG follows the service selected for playback; there is no
    // second selector here.
    if (!state.output.selected_service_id.has_value()) {
        draw_disabled_wrapped("Select a service in the player controls");
        ImGui::PopID();
        return;
    }
    const auto selected_service = std::ranges::find_if(
        state.frame.services, [&state](const TransportService &service) {
            return state.output.selected_service_id == service.service_id;
        });
    if (selected_service != state.frame.services.end()) {
        ImGui::TextDisabled(
            "%s", selected_service->name.empty()
                      ? std::format("Service {}", selected_service->service_id)
                            .c_str()
                      : std::format("{}  ({})", selected_service->name,
                                    selected_service->service_id)
                            .c_str());
    }
    const EpgSnapshot snapshot = state.frame.epg;
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
    // Time range on one line, then the title and the description wrapped to
    // the strip width: a SameLine title would be clipped at the right edge.
    const auto draw_event = [&format_time, &trimmed](const char *tag,
                                                     const EpgEvent &event,
                                                     const bool is_now) {
        const std::string start = format_time(event.start_time_utc);
        const std::string end =
            format_time(event.start_time_utc + event.duration_seconds);
        ImGui::TextDisabled("%s  %s - %s", tag, start.c_str(), end.c_str());
        std::string title = trimmed(event.name);
        if (!event.genre.empty()) {
            title += "  [" + event.genre + "]";
        }
        if (is_now) {
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(0.35F, 0.88F, 0.55F, 1.0F));
        }
        ImGui::TextWrapped("%s", title.c_str());
        if (is_now) {
            ImGui::PopStyleColor();
        }
        const std::string description = trimmed(event.description);
        if (!description.empty() && description != event.name) {
            ImGui::TextWrapped("%s", description.c_str());
        }
    };
    if (now_event != snapshot.events.end()) {
        draw_event("NOW", *now_event, true);
    } else {
        ImGui::TextDisabled("NOW  (no event currently running)");
    }
    ImGui::Separator();
    if (next_event != snapshot.events.end()) {
        draw_event("NEXT", *next_event, false);
    } else {
        ImGui::TextDisabled("NEXT  (no upcoming event)");
    }
    ImGui::PopID();
}

void draw_ts_recorder_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("MPEG-TS Stream Recorder")) {
        return;
    }
    ImGui::PushID("ts-recorder");
    consume_file_dialog_result(state, state.output.ts_file_dialog,
                               state.output.ts_recording_path,
                               "MPEG-TS recording");
    const bool dialog_open = file_dialog_is_open(state.output.ts_file_dialog);
    const bool ts_source_available = state.frame.source_streaming;
    const auto ts_stats = state.frame.ts_recording_stats;

    draw_disabled_wrapped("Decoded transport stream (MPEG-TS)");
    ImGui::TextUnformatted("Output file");
    ImGui::BeginDisabled(dialog_open);
    ImGui::SetNextItemWidth(-92.0F);
    ImGui::InputText("##ts-recording-output", &state.output.ts_recording_path);
    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        show_recording_file_dialog(state, state.output.ts_file_dialog,
                                   state.output.ts_recording_path,
                                   transport_stream_filters);
    }
    ImGui::EndDisabled();
    if (dialog_open) {
        ImGui::TextDisabled("Waiting for file selection...");
    }

    if (!ts_stats.active) {
        ImGui::BeginDisabled(!ts_source_available || dialog_open ||
                             state.output.ts_recording_path.empty());
        if (ImGui::Button("Start recording", ImVec2(-1.0F, 0.0F))) {
            state.ui.status =
                state.reporting.controller
                    .start_ts_recording(state.output.ts_recording_path)
                    .message;
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop recording", ImVec2(-1.0F, 0.0F))) {
        state.ui.status =
            state.reporting.controller.stop_ts_recording().message;
    }

    draw_recorder_write_stats(state.output.ts_write_rate, ts_stats);
    ImGui::PopID();
}

void draw_rtp_streaming_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("RTP/UDP MPEG-TS Streaming")) {
        return;
    }
    ImGui::PushID("rtp-streaming");
    const auto stats = state.frame.rtp_stats;
    ImGui::BeginDisabled(stats.active);
    ImGui::TextUnformatted("Destination host");
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputText("##rtp-host", &state.output.rtp_host);
    ImGui::TextUnformatted("UDP port");
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputScalar("##rtp-port", ImGuiDataType_U16, &state.output.rtp_port);
    ImGui::EndDisabled();

    if (!stats.active) {
        ImGui::BeginDisabled(!state.frame.source_streaming ||
                             state.output.rtp_host.empty() ||
                             state.output.rtp_port == 0);
        if (ImGui::Button("Start streaming", ImVec2(-1.0F, 0.0F))) {
            const RtpUdpEndpoint endpoint{.host = state.output.rtp_host,
                                          .port = state.output.rtp_port};
            state.ui.status =
                state.reporting.controller.start_rtp(endpoint).message;
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop streaming", ImVec2(-1.0F, 0.0F))) {
        state.ui.status = state.reporting.controller.stop_rtp().message;
    }

    const double mib_per_second =
        state.output.rtp_write_rate.update(stats.active, stats.wire_bytes_sent);
    ImGui::Text("Duration: %s",
                format_recording_duration(stats.elapsed_milliseconds).c_str());
    ImGui::Text("Sent: %.2f MiB",
                static_cast<double>(stats.wire_bytes_sent) / (1024.0 * 1024.0));
    ImGui::Text("Send rate: %.2f MiB/s", mib_per_second);
    ImGui::Text("Datagrams: %llu",
                static_cast<unsigned long long>(stats.datagrams_sent));
    ImGui::Text("Queued: %.1f KiB",
                static_cast<double>(stats.queued_bytes) / 1024.0);
    ImGui::Text("Dropped: %llu   Send errors: %llu",
                static_cast<unsigned long long>(stats.dropped_datagrams),
                static_cast<unsigned long long>(stats.write_errors));
    if (!stats.last_error.empty()) {
        ImGui::TextWrapped("Last send error: %s", stats.last_error.c_str());
    }
    ImGui::PopID();
}

void draw_playback_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("Playback", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    const auto telemetry = state.frame.playback;
    const bool playing = state.output.player.ready() && !telemetry.paused &&
                         !telemetry.buffering;
    const char *playback_status =
        telemetry.buffering ? "BUFFERING"
                            : (state.output.player.ready()
                                   ? (telemetry.paused ? "PAUSED" : "PLAYING")
                                   : "PLAYER IDLE");
    draw_status_indicator(
        playback_status,
        playing ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                : (telemetry.buffering || state.output.player.ready()
                       ? ImVec4(0.95F, 0.72F, 0.30F, 1.0F)
                       : ImVec4(0.55F, 0.62F, 0.70F, 1.0F)));
    std::string position = "--:--:--";
    if (telemetry.playback_time_s > 0.0) {
        const auto total = static_cast<std::int64_t>(telemetry.playback_time_s);
        position = std::format("{:02}:{:02}:{:02}", total / 3600,
                               (total % 3600) / 60, total % 60);
    }
    ImGui::TextUnformatted("Position");
    ImGui::SameLine(115.0F);
    ImGui::TextColored(ImVec4(0.52F, 0.82F, 1.0F, 1.0F), "%s",
                       position.c_str());
    if (begin_metric_pair("playback-metrics")) {
        next_metric_cell();
        const std::string av_sync =
            std::format("{:+7.1f} ms", telemetry.avsync_ms);
        draw_metric("A/V sync", av_sync.c_str(),
                    std::clamp(1.0F - static_cast<float>(
                                          std::abs(telemetry.avsync_ms)) /
                                          50.0F,
                               0.0F, 1.0F),
                    std::abs(telemetry.avsync_ms) < 20.0
                        ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                        : (std::abs(telemetry.avsync_ms) < 80.0
                               ? ImVec4(0.95F, 0.72F, 0.30F, 1.0F)
                               : ImVec4(0.95F, 0.38F, 0.28F, 1.0F)));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            draw_tooltip(
                "Audio-vs-video presentation offset reported by libmpv. "
                "Positive "
                "means audio is ahead of video. Sustained drift here is an A/V "
                "clock problem; RF loss instead shows up as discontinuity "
                "growth "
                "and a queued-buffer dip.");
        }
        next_metric_cell();
        const std::string buffer =
            std::format("{:4.1f} / {:4.1f} MiB",
                        static_cast<double>(telemetry.queued_bytes) /
                            static_cast<double>(1U << 20U),
                        static_cast<double>(telemetry.queue_capacity) /
                            static_cast<double>(1U << 20U));
        draw_metric("TS buffer", buffer.c_str(),
                    telemetry.queue_capacity == 0
                        ? 0.0F
                        : static_cast<float>(
                              static_cast<double>(telemetry.queued_bytes) /
                              static_cast<double>(telemetry.queue_capacity)),
                    ImVec4(0.52F, 0.82F, 1.0F, 1.0F));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            draw_tooltip(
                "Decoded TS bytes of the selected service queued for libmpv vs "
                "the queue capacity. Playback buffers to 2 MiB at startup and "
                "after a stall, re-enters buffering below 1 MiB, and the "
                "playback rate is steered by a fraction of a percent to hold "
                "the "
                "queue near 2.5 MiB so clock mismatch cannot drain or overflow "
                "it over a long session.");
        }
        end_metric_pair();
    }
    const std::string buffered =
        telemetry.queued_seconds.has_value()
            ? std::format("{:6.2f} s", *telemetry.queued_seconds)
            : std::string("    -- s");
    ImGui::Text("Buffered        %s   speed %.4fx   stalls %llu",
                buffered.c_str(), telemetry.playback_speed,
                static_cast<unsigned long long>(telemetry.buffering_stalls));
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        draw_tooltip(
            "Queue depth in stream seconds at the service's PCR-measured bit "
            "rate, the playback rate currently requested from libmpv, and "
            "how many times playback drained the queue and had to wait for "
            "it to refill (each one is a gap).");
    }
    const auto &clock = state.frame.transport_clock;
    const std::string mux_rate =
        clock.bitrate_bps.has_value()
            ? std::format("{:.3f} Mbit/s", *clock.bitrate_bps / 1e6)
            : std::string("--");
    const std::string clock_offset =
        clock.clock_offset_ppm.has_value()
            ? std::format("{:+.1f} ppm over {:.0f} s", *clock.clock_offset_ppm,
                          clock.clock_span_s)
            : std::string("--");
    ImGui::Text("Mux rate        %s", mux_rate.c_str());
    ImGui::Text("PCR clock       %s", clock_offset.c_str());
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        draw_tooltip(
            "Broadcast clock (PCR) against this computer's clock at the "
            "decoder output: positive means the broadcast runs fast relative "
            "to the system clock. Only meaningful for live reception; for a "
            "recording it reflects the processing speed.");
    }
    ImGui::Text("PCR/PTS/DTS     %llu / %llu / %llu",
                static_cast<unsigned long long>(clock.pcr_count),
                static_cast<unsigned long long>(clock.pts_count),
                static_cast<unsigned long long>(clock.dts_count));
    ImGui::Text(
        "PCR seams       %llu+%llu   max gap %.0f ms",
        static_cast<unsigned long long>(clock.pcr_flagged_discontinuities),
        static_cast<unsigned long long>(clock.pcr_unflagged_discontinuities),
        clock.pcr_interval_max_ms);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        draw_tooltip(
            "Timestamps seen in the full multiplex, PCR seams announced by "
            "the discontinuity indicator + unannounced jumps, and the longest "
            "gap between consecutive PCRs of one PID (DVB bounds it at "
            "100 ms).");
    }
    ImGui::Text("Dropped frames  %lld   (VO %lld)",
                static_cast<long long>(telemetry.dropped_frames),
                static_cast<long long>(telemetry.vo_dropped_frames));
    ImGui::Text("Discontinuities  %llu",
                static_cast<unsigned long long>(telemetry.discontinuities));
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        draw_tooltip(
            "Stream-level seams: FEC-region resets (fade gaps), stream ends, "
            "and retunes. These are out-of-band events reported by the "
            "decoder, distinct from per-packet TEI corruption; retunes "
            "restart the demuxer.");
    }
    ImGui::Separator();
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
    const float pixel_density =
        state.ui.window == nullptr ? 1.0F
                                   : SDL_GetWindowPixelDensity(state.ui.window);
    const int video_width = static_cast<int>(
        std::max(1.0F, (video_extent.x - video_origin.x) * pixel_density));
    const int video_height = static_cast<int>(
        std::max(1.0F, (video_extent.y - video_origin.y) * pixel_density));
    const std::uint32_t video_texture =
        state.output.player.render(video_width, video_height);
    if (video_texture != 0 && state.output.player.ready()) {
        draw->AddImage(static_cast<ImTextureID>(video_texture), video_origin,
                       video_extent);
    }

    draw->AddRectFilled(origin, ImVec2(extent.x, origin.y + header_height),
                        IM_COL32(15, 24, 35, 255), 5.0F);
    const auto selected_service = std::ranges::find_if(
        state.frame.services, [&state](const TransportService &service) {
            return state.output.selected_service_id == service.service_id;
        });
    const std::string program_title =
        selected_service == state.frame.services.end()
            ? "DIGITAL TV SERVICE"
            : std::format(
                  "DIGITAL TV  {}",
                  selected_service->name.empty()
                      ? std::format("Service {}", selected_service->service_id)
                      : selected_service->name);
    draw->AddText(ImVec2(origin.x + 16.0F, origin.y + 14.0F),
                  IM_COL32(220, 230, 240, 255), program_title.c_str());
    constexpr float expand_button_width = 92.0F;
    ImGui::SetCursorScreenPos(
        ImVec2(extent.x - expand_button_width - 12.0F, origin.y + 9.0F));
    if (ImGui::Button(state.ui.video_expanded ? "Restore" : "Expand",
                      ImVec2(expand_button_width, 0.0F))) {
        state.ui.video_expanded = !state.ui.video_expanded;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        draw_tooltip(state.ui.video_expanded
                         ? "Show the panels again (Esc or F)"
                         : "Hide the panels and fill the window (F)");
    }

    const ImVec2 center{(origin.x + extent.x) * 0.5F,
                        (origin.y + footer_origin.y) * 0.5F};
    const std::string player_status = state.output.player.status();
    if (!state.output.player.ready()) {
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
        selected_service == state.frame.services.end()
            ? "No services"
            : (selected_service->name.empty()
                   ? std::format("Service {}", selected_service->service_id)
                   : std::format("{}  ({})", selected_service->name,
                                 selected_service->service_id));
    ImGui::BeginDisabled(state.frame.services.empty());
    if (ImGui::BeginCombo("##service-selection", service_preview.c_str())) {
        for (const auto &service : state.frame.services) {
            const std::string label =
                service.name.empty()
                    ? std::format("Service {}", service.service_id)
                    : std::format("{}  ({})", service.name, service.service_id);
            const bool selected =
                state.output.selected_service_id == service.service_id;
            if (ImGui::Selectable(label.c_str(), selected)) {
                state.output.selected_service_id = service.service_id;
                state.output.player.select_service(service);
                state.ui.status = "Selected " + label;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    ImGui::SameLine(0.0F, control_spacing);
    if (ImGui::Button(state.output.player.muted() ? "Unmute" : "Mute",
                      ImVec2(mute_width, 0.0F))) {
        state.output.player.set_muted(!state.output.player.muted());
    }
    ImGui::SameLine(0.0F, control_spacing);
    float volume = state.output.player.volume();
    ImGui::SetNextItemWidth(std::max(
        100.0F, available.x - (horizontal_padding * 2.0F) - service_width -
                    mute_width - (control_spacing * 2.0F)));
    if (ImGui::SliderFloat("##playback-volume", &volume, 0.0F, 100.0F,
                           "Volume %.0f%%")) {
        state.output.player.set_volume(volume);
    }
}

} // namespace airspy_tv::gui
