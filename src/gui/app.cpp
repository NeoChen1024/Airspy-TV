#include "app.hpp"

#include "app_state.hpp"
#include "decode_reporting.hpp"
#include "panels.hpp"
#include "widgets.hpp"
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <ranges>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace airspy_tv::gui {
namespace {

constexpr float panel_width = 410.0F;

void update_app_state(AppState &state) {
    state.frame.standard = state.session.standard();
    state.frame.channel_bandwidth_hz = state.session.channel_bandwidth_hz();
    state.frame.source_open = state.session.is_open();
    state.frame.source_streaming = state.session.is_streaming();
    state.frame.input_exhausted = state.session.input_exhausted();
    state.frame.iq_recording = state.session.is_recording();
    if (const auto *descriptor = state.session.descriptor();
        descriptor != nullptr) {
        state.frame.descriptor = *descriptor;
    } else {
        state.frame.descriptor.reset();
    }
    state.frame.sample_rates = state.session.sample_rates();
    state.frame.gain_range = state.session.gain_range();
    state.output.player.set_source_active(state.frame.source_streaming);
    state.output.player.poll_events();
    state.frame.playback = state.output.player.telemetry();
    state.frame.spectrum = state.session.spectrum_snapshot();
    state.frame.signal = state.session.signal_snapshot();
    state.frame.pipeline = state.session.pipeline_snapshot();
    state.frame.iq_recording_stats = state.session.recording_stats();
    state.frame.ts_recording_stats = state.session.ts_recording_stats();
    state.frame.rtp_stats = state.session.rtp_streaming_stats();
    update_standard_state(state);
    float queue_pressure = 0.0F;
    for (std::size_t index = 0; index < state.frame.pipeline.stage_count;
         ++index) {
        const auto &stage = state.frame.pipeline.stages[index];
        if (stage.queue_valid) {
            queue_pressure = std::max(queue_pressure, stage.queue_fraction);
        }
    }
    state.frame.pipeline_load =
        state.display.pipeline_load_monitor.update(PipelineLoadSample{
            .active =
                state.frame.source_streaming || state.frame.pipeline.processing,
            .realtime_ratio = state.frame.pipeline.processing_realtime_ratio,
            .queue_pressure_fraction = queue_pressure,
            .dropped_blocks = state.frame.pipeline.dropped_blocks,
            .sequence = state.frame.pipeline.sequence});
    state.frame.services = state.session.transport_services();
    if (!state.frame.services.empty() &&
        std::ranges::none_of(
            state.frame.services, [&state](const auto &service) {
                return state.output.selected_service_id == service.service_id;
            })) {
        state.output.selected_service_id =
            state.frame.services.front().service_id;
    }
    if (state.output.selected_service_id.has_value()) {
        const auto service = std::ranges::find_if(
            state.frame.services, [&state](const TransportService &candidate) {
                return candidate.service_id ==
                       *state.output.selected_service_id;
            });
        if (service != state.frame.services.end()) {
            state.output.player.select_service(*service);
        }
        state.frame.epg =
            state.session.epg_snapshot(*state.output.selected_service_id);
    } else {
        state.output.player.clear_service();
        state.frame.epg = {};
    }
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
    const bool file_source =
        state.frame.descriptor.has_value() &&
        state.frame.descriptor->backend == SdrBackend::File;
    ImGui::BeginDisabled(file_source);
    if (const auto frequency = draw_frequency_control(
            "center-frequency", state.source.settings.center_frequency_hz);
        frequency.has_value()) {
        request_center_frequency(state, *frequency);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    const float status_width = ImGui::CalcTextSize(state.ui.status.c_str()).x;
    ImGui::SetCursorPosX(
        std::max(ImGui::GetCursorPosX(),
                 ImGui::GetWindowWidth() - status_width - 20.0F));
    ImGui::TextColored(state.frame.iq_recording
                           ? ImVec4(1.0F, 0.30F, 0.28F, 1.0F)
                           : ImVec4(0.58F, 0.66F, 0.74F, 1.0F),
                       "%s", state.ui.status.c_str());
    ImGui::EndChild();

    const float sidebar =
        std::min(panel_width, ImGui::GetContentRegionAvail().x * 0.42F);
    ImGui::BeginChild("side-panel", ImVec2(sidebar, 0.0F),
                      ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar);
    draw_source_panel(state);
    draw_spectrum_panel(state);
    draw_receiver_panel(state);
    draw_constellation_panel(state);
    draw_common_signal_panel(state);
    draw_standard_diagnostics_panel(state);
    draw_playback_panel(state);
    draw_epg_panel(state);
    draw_ts_recorder_panel(state);
    draw_rtp_streaming_panel(state);
    draw_recorder_panel(state);
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("video-panel", ImVec2(0.0F, 0.0F),
                      ImGuiChildFlags_Borders);
    draw_video_panel(state);
    ImGui::EndChild();

    ImGui::End();
}

} // namespace

int run_gui(std::optional<std::filesystem::path> report_directory) {
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
        SDL_CreateWindow("Airspy TV Receiver", 1500, 900,
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

    AppState state(std::move(report_directory));
    state.reporting.decode_report.set_transport_output_provider([&state] {
        const auto playback = state.output.player.telemetry();
        return std::vector<TransportOutputTelemetry>{
            {.name = "mpv-playback",
             .type = "playback",
             .active = playback.active,
             .blocks_accepted = playback.blocks_accepted,
             .bytes_accepted = playback.bytes_accepted,
             .blocks_processed = playback.blocks_processed,
             .bytes_processed = playback.bytes_processed,
             .dropped_blocks = playback.dropped_blocks,
             .dropped_bytes = playback.dropped_bytes,
             .queued_bytes = playback.queued_bytes,
             .queue_capacity_bytes = playback.queue_capacity,
             .error = {}},
        };
    });
    initialize_standard_state(state);
    state.ui.window = window;
    std::string player_error;
    if (!state.output.player.initialize(player_error)) {
        std::cerr << player_error << '\n';
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
        SDL_GL_DestroyContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    state.session.set_transport_sink(
        [&state](const std::span<const std::uint8_t> ts) {
            state.output.player.submit(ts);
        });
    state.session.set_discontinuity_callback(
        [&state](const TransportDiscontinuity discontinuity) {
            state.output.player.on_discontinuity(discontinuity);
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

        const std::string runtime_error = state.session.runtime_error();
        if (!runtime_error.empty()) {
            state.ui.status = runtime_error;
        }
        const bool input_exhausted = state.frame.input_exhausted;
        if (input_exhausted && !state.source.observed_input_exhausted) {
            state.ui.status = "I/Q file playback finished";
        }
        state.source.observed_input_exhausted = input_exhausted;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        update_app_state(state);
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

    finalize_decode_report(state);
    state.session.set_transport_sink({});
    state.session.close();
    state.output.player.shutdown();
    state.display.waterfall.destroy();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

} // namespace airspy_tv::gui
