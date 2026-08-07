#include "app.hpp"

#include "app_state.hpp"
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

namespace airspy_tv::gui {
namespace {

constexpr float panel_width = 410.0F;

void update_app_state(AppState &state) {
    state.player.set_source_active(state.session.is_streaming());
    const DeviceDescriptor *source_descriptor = state.session.descriptor();
    if (source_descriptor != state.last_source_descriptor) {
        state.epg.reset();
        state.last_source_descriptor = source_descriptor;
    }
    state.player.poll_events();
    state.spectrum = state.session.spectrum_snapshot();
    state.signal = state.session.signal_snapshot();
    state.pipeline = state.session.pipeline_snapshot();
    update_standard_state(state);
    float queue_pressure = 0.0F;
    for (std::size_t index = 0; index < state.pipeline.stage_count; ++index) {
        const auto &stage = state.pipeline.stages[index];
        if (stage.queue_valid) {
            queue_pressure = std::max(queue_pressure, stage.queue_fraction);
        }
    }
    state.pipeline_load = state.pipeline_load_monitor.update(PipelineLoadSample{
        .active = state.session.is_streaming() || state.pipeline.processing,
        .realtime_ratio = state.pipeline.processing_realtime_ratio,
        .queue_pressure_fraction = queue_pressure,
        .dropped_blocks = state.pipeline.dropped_blocks,
        .sequence = state.pipeline.sequence});
    state.services = state.session.transport_services();
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
        state.session.descriptor() != nullptr &&
        state.session.descriptor()->backend == SdrBackend::File;
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
    ImGui::TextColored(state.session.is_recording()
                           ? ImVec4(1.0F, 0.30F, 0.28F, 1.0F)
                           : ImVec4(0.58F, 0.66F, 0.74F, 1.0F),
                       "%s", state.status.c_str());
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

int run_gui() {
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

    AppState state;
    initialize_standard_state(state);
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
    state.session.set_transport_sink(
        [&state](const std::span<const std::uint8_t> ts) {
            state.epg.consume(ts);
            state.player.submit(ts);
        });
    state.session.set_discontinuity_callback(
        [&state](const TransportDiscontinuity discontinuity) {
            state.player.on_discontinuity(discontinuity);
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
            state.status = runtime_error;
        }

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

    state.session.set_transport_sink({});
    state.session.close();
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

} // namespace airspy_tv::gui
