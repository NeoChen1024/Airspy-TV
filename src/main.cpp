#include "airspy_tv/sdr.hpp"

#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/epg.hpp"
#include "airspy_tv/iq_file.hpp"
#include "airspy_tv/mpv_player.hpp"
#include "byte_rate_tracker.hpp"
#include "pipeline_load_monitor.hpp"
#include "receiver_session.hpp"

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
#include <cctype>
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
#include <getopt.h>
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
using airspy_tv::ByteRateTracker;
using airspy_tv::DeviceDescriptor;
using airspy_tv::EnumerationResult;
using airspy_tv::EpgEvent;
using airspy_tv::EpgModel;
using airspy_tv::EpgSnapshot;
using airspy_tv::IqFileInfo;
using airspy_tv::MpvPlayer;
using airspy_tv::PipelineLoadMonitor;
using airspy_tv::PipelineLoadSample;
using airspy_tv::PipelineLoadState;
using airspy_tv::PipelineSnapshot;
using airspy_tv::ReceiverSession;
using airspy_tv::ReceiveStandard;
using airspy_tv::SdrBackend;
using airspy_tv::SdrDevice;
using airspy_tv::SignalSnapshot;
using airspy_tv::SourceSettings;
using airspy_tv::SpectrumSnapshot;
using airspy_tv::TransportDiscontinuity;
using airspy_tv::TransportService;
using airspy_tv::dvbt::CodeRate;
using airspy_tv::dvbt::Constellation;
using airspy_tv::dvbt::GuardInterval;
using airspy_tv::dvbt::ReceiverParameters;
using airspy_tv::dvbt::SignalAnalysisSnapshot;
using airspy_tv::dvbt::StreamDecoder;
using airspy_tv::dvbt::StreamDecoderStats;
using airspy_tv::dvbt::TransmissionMode;
using airspy_tv::dvbt::WorkerState;

// Decoder diagnostics helpers (defined in the anonymous namespace below):
// forward declarations so the GUI panels and the periodic dump can use them
// before the definitions.
[[nodiscard]] const char *worker_state_name(const WorkerState state);
void dump_decoder_diagnostics(const StreamDecoderStats &stats);

#include "main_gui.hpp"

#include "decoder_diagnostics.hpp"
#include "main_commands.hpp"

#include "main_cli.hpp"

int main(const int argc, char **argv) {
    bool debug = false;
    std::optional<std::filesystem::path> inspect_iq_path;
    std::optional<std::filesystem::path> decode_iq_path;
    std::optional<std::filesystem::path> ts_output_path;
    std::optional<std::filesystem::path> record_path;
    std::uint32_t sample_rate_hz = 10'000'000;
    std::uint64_t center_frequency_hz = 545'000'000;
    double frequency_correction_ppm = 0.0;
    int duration_ms = 1000;
    int airspy_gain = 10;
    AirspyGainMode airspy_gain_mode = AirspyGainMode::Sensitivity;
    bool bias_tee = false;
    ReceiverParameters dvbt_parameters;

    int opt;
    while ((opt = getopt_long(argc, argv, "hd", cli_options, nullptr)) != -1) {
        switch (opt) {
        case 'h':
            print_cli_usage();
            return 0;
        case 'd':
            debug = true;
            break;
        case opt_enumerate:
            return enumerate_cli();
        case opt_inspect_iq:
            inspect_iq_path = optarg;
            break;
        case opt_decode_iq:
            decode_iq_path = optarg;
            break;
        case opt_ts_output:
            ts_output_path = optarg;
            break;
        case opt_sample_rate:
            sample_rate_hz =
                static_cast<std::uint32_t>(parse_u64(optarg, "sample rate"));
            if (sample_rate_hz == 0) {
                cli_usage_error("sample rate must be non-zero");
            }
            break;
        case opt_mode:
            // Validated here; DVB-T is the only implemented standard today,
            // so the parsed value is otherwise unused (a future standard
            // switch would select the demodulator type).
            static_cast<void>(parse_standard(optarg));
            break;
        case opt_dvbt_mode:
            dvbt_parameters.mode = parse_dvbt_mode(optarg);
            break;
        case opt_dvbt_bandwidth:
            dvbt_parameters.channel_bandwidth_hz = parse_bandwidth(optarg);
            break;
        case opt_dvbt_guard:
            dvbt_parameters.guard_interval = parse_guard_interval(optarg);
            break;
        case opt_dvbt_modulation:
            dvbt_parameters.constellation = parse_modulation(optarg);
            break;
        case opt_dvbt_code_rate:
            dvbt_parameters.code_rate = parse_code_rate(optarg);
            break;
        case opt_decoder_threads:
            dvbt_parameters.worker_threads = static_cast<std::size_t>(
                parse_u64(optarg, "decoder thread count"));
            if (dvbt_parameters.worker_threads > 256) {
                cli_usage_error("decoder thread count must be 0..256");
            }
            break;
        case opt_record_first:
            record_path = optarg;
            break;
        case opt_duration:
            duration_ms =
                static_cast<int>(parse_u64(optarg, "recording duration"));
            if (duration_ms <= 0) {
                cli_usage_error("recording duration must be positive");
            }
            break;
        case opt_frequency:
            center_frequency_hz = parse_u64(optarg, "center frequency");
            break;
        case opt_ppm:
            frequency_correction_ppm =
                parse_double(optarg, "frequency correction");
            break;
        case opt_gain:
            airspy_gain = static_cast<int>(parse_u64(optarg, "gain"));
            break;
        case opt_gain_mode:
            airspy_gain_mode = parse_gain_mode(optarg);
            break;
        case opt_bias_tee:
            bias_tee = true;
            break;
        default:
            cli_usage_error("unrecognized option");
        }
    }
    if (optind != argc) {
        cli_usage_error(
            std::format("unexpected positional argument: '{}'", argv[optind]));
    }

    const int command_count = (decode_iq_path.has_value() ? 1 : 0) +
                              (record_path.has_value() ? 1 : 0) +
                              (inspect_iq_path.has_value() ? 1 : 0);
    if (command_count > 1) {
        cli_usage_error("--decode-iq, --record-first and --inspect-iq are "
                        "mutually exclusive");
    }

    if (inspect_iq_path.has_value()) {
        return inspect_iq_cli(*inspect_iq_path, sample_rate_hz,
                              center_frequency_hz);
    }
    if (decode_iq_path.has_value()) {
        if (!ts_output_path.has_value()) {
            cli_usage_error("--decode-iq requires --ts-output");
        }
        return decode_iq_cli(*decode_iq_path, *ts_output_path, sample_rate_hz,
                             dvbt_parameters, debug);
    }
    if (record_path.has_value()) {
        SourceSettings settings;
        settings.center_frequency_hz = center_frequency_hz;
        settings.frequency_correction_ppm = frequency_correction_ppm;
        settings.sample_rate_hz = sample_rate_hz;
        settings.airspy_gain_mode = airspy_gain_mode;
        settings.airspy_gain = airspy_gain;
        settings.bias_tee = bias_tee;
        return record_first_cli(*record_path, duration_ms, settings);
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
    state.session.set_dvbt_parameters(state.dvbt.parameters);
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
