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
    ImGui::BeginDisabled(state.session.is_recording() || dialog_open);
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

    if (!state.session.is_recording()) {
        const bool file_source =
            state.session.descriptor() != nullptr &&
            state.session.descriptor()->backend == SdrBackend::File;
        ImGui::BeginDisabled(!state.session.is_open() || file_source ||
                             dialog_open || state.recording_path.empty());
        if (ImGui::Button("Start recording", ImVec2(-1.0F, 0.0F))) {
            std::string error;
            if (state.session.start_recording(
                    std::filesystem::path(state.recording_path), state.settings,
                    error)) {
                state.status = "Recording raw I/Q";
            } else {
                state.status = error;
            }
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop recording", ImVec2(-1.0F, 0.0F))) {
        state.session.stop_recording();
        state.status = "Recording stopped; JSON sidecar written";
    }

    const auto stats = state.session.recording_stats();
    draw_recorder_write_stats(state.iq_write_rate, stats);
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
        draw_disabled_wrapped("No services detected");
        ImGui::PopID();
        return;
    }
    // The EPG follows the service selected for playback; there is no
    // second selector here.
    if (!state.selected_service_id.has_value()) {
        draw_disabled_wrapped("Select a service in the player controls");
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
    const bool ts_source_available = state.session.is_streaming();
    const auto ts_stats = state.session.ts_recording_stats();

    draw_disabled_wrapped("Decoded transport stream (MPEG-TS)");
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
            state.status =
                state.session.start_ts_recording(state.ts_recording_path, error)
                    ? "Recording decoded MPEG-TS"
                    : error;
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop recording", ImVec2(-1.0F, 0.0F))) {
        state.session.stop_ts_recording();
        state.status = "MPEG-TS recording stopped";
    }

    draw_recorder_write_stats(state.ts_write_rate, ts_stats);
    ImGui::PopID();
}

void draw_rtp_streaming_panel(AppState &state) {
    if (!ImGui::CollapsingHeader("RTP/UDP MPEG-TS Streaming",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::PushID("rtp-streaming");
    const auto stats = state.session.rtp_streaming_stats();
    ImGui::BeginDisabled(stats.active);
    ImGui::TextUnformatted("Destination host");
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputText("##rtp-host", &state.rtp_host);
    ImGui::TextUnformatted("UDP port");
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputScalar("##rtp-port", ImGuiDataType_U16, &state.rtp_port);
    ImGui::EndDisabled();

    if (!stats.active) {
        ImGui::BeginDisabled(!state.session.is_streaming() ||
                             state.rtp_host.empty() || state.rtp_port == 0);
        if (ImGui::Button("Start streaming", ImVec2(-1.0F, 0.0F))) {
            std::string error;
            const RtpUdpEndpoint endpoint{.host = state.rtp_host,
                                          .port = state.rtp_port};
            state.status = state.session.start_rtp_streaming(endpoint, error)
                               ? "Streaming MPEG-TS over RTP/UDP to " +
                                     format_rtp_udp_endpoint(endpoint)
                               : error;
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop streaming", ImVec2(-1.0F, 0.0F))) {
        state.session.stop_rtp_streaming();
        state.status = "RTP/UDP streaming stopped";
    }

    const double mib_per_second =
        state.rtp_write_rate.update(stats.active, stats.wire_bytes_sent);
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
    const auto telemetry = state.player.telemetry();
    const bool playing =
        state.player.ready() && !telemetry.paused && !telemetry.buffering;
    const char *playback_status =
        telemetry.buffering
            ? "BUFFERING"
            : (state.player.ready() ? (telemetry.paused ? "PAUSED" : "PLAYING")
                                    : "PLAYER IDLE");
    draw_status_indicator(playback_status,
                          playing ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
                                  : (telemetry.buffering || state.player.ready()
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
    const std::string av_sync = std::format("{:+.1f} ms", telemetry.avsync_ms);
    draw_metric(
        "A/V sync", av_sync.c_str(),
        std::clamp(1.0F - static_cast<float>(std::abs(telemetry.avsync_ms)) /
                              50.0F,
                   0.0F, 1.0F),
        std::abs(telemetry.avsync_ms) < 20.0
            ? ImVec4(0.35F, 0.88F, 0.55F, 1.0F)
            : (std::abs(telemetry.avsync_ms) < 80.0
                   ? ImVec4(0.95F, 0.72F, 0.30F, 1.0F)
                   : ImVec4(0.95F, 0.38F, 0.28F, 1.0F)));
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip(
            "Audio-vs-video presentation offset reported by libmpv. Positive "
            "means audio is ahead of video. Sustained drift here is an A/V "
            "clock problem; RF loss instead shows up as discontinuity growth "
            "and a queued-buffer dip.");
    }
    const std::string buffer =
        std::format("{:.1f} / {:.1f} MiB",
                    static_cast<double>(telemetry.queued_bytes) /
                        static_cast<double>(1U << 20U),
                    static_cast<double>(telemetry.queue_capacity) /
                        static_cast<double>(1U << 20U));
    draw_metric(
        "TS buffer", buffer.c_str(),
        telemetry.queue_capacity == 0
            ? 0.0F
            : static_cast<float>(static_cast<double>(telemetry.queued_bytes) /
                                 static_cast<double>(telemetry.queue_capacity)),
        ImVec4(0.52F, 0.82F, 1.0F, 1.0F));
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip(
            "Decoded TS bytes queued for libmpv vs the queue capacity; growth "
            "here means playback is not keeping up (stalled audio device or a "
            "paused player). Playback buffers to 2 MiB at startup/recovery, "
            "and re-enters buffering below 1 MiB.");
    }
    ImGui::Text("Dropped frames  %lld   (VO %lld)",
                static_cast<long long>(telemetry.dropped_frames),
                static_cast<long long>(telemetry.vo_dropped_frames));
    ImGui::Text("Discontinuities  %llu",
                static_cast<unsigned long long>(telemetry.discontinuities));
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip(
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
            ? "DIGITAL TV SERVICE"
            : std::format(
                  "DIGITAL TV  {}",
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
            ? "No services"
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

} // namespace airspy_tv::gui
