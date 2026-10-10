cmake_minimum_required(VERSION 3.25)

project(AirspyTV VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AIRSPY_TV_NATIVE_ARCH
       "Optimize generated code for the build host CPU" ON)
if(AIRSPY_TV_NATIVE_ARCH AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-march=native)
endif()

# Keep full Debug symbols while optimizing DSP hot paths and disabling runtime
# assertions, matching Release throughput as closely as an optimized debugger
# build can. Set this option OFF when assertion-heavy diagnostics are needed.
option(AIRSPY_TV_OPTIMIZED_DEBUG
       "Enable -O3/-DNDEBUG while retaining Debug symbols" ON)
if(AIRSPY_TV_OPTIMIZED_DEBUG AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options("$<$<CONFIG:Debug>:-O3;-DNDEBUG>")
endif()

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(SDL3 CONFIG REQUIRED)
find_package(SoapySDR CONFIG REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(AIRSPY REQUIRED IMPORTED_TARGET libairspy)
pkg_check_modules(FFTW3F REQUIRED IMPORTED_TARGET fftw3f)
pkg_check_modules(FONTCONFIG REQUIRED IMPORTED_TARGET fontconfig)
pkg_check_modules(MPV REQUIRED IMPORTED_TARGET mpv)

set(JSON_BuildTests OFF CACHE INTERNAL "Disable nlohmann/json tests")
add_subdirectory(contrib/json EXCLUDE_FROM_ALL)

add_subdirectory(contrib/solid-resampler EXCLUDE_FROM_ALL)

add_subdirectory(contrib/libfec EXCLUDE_FROM_ALL)

# ViterbiDecoderCpp (LGPL-2.1, header-only) provides AVX2, SSE4.1, AArch64
# NEON, and scalar soft-Viterbi backends selected at compile time.
add_library(viterbi-decoder INTERFACE)
target_include_directories(viterbi-decoder SYSTEM INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/contrib/viterbi-decoder-cpp/include")
target_compile_features(viterbi-decoder INTERFACE cxx_std_17)

option(AIRSPY_TV_USE_SIMD_VITERBI
       "Use available ViterbiDecoderCpp SIMD for soft Viterbi" ON)

set(TINYCOLORMAP_BUILD_TOOLS OFF CACHE INTERNAL
    "Disable tinycolormap optional tools")
add_subdirectory(contrib/tinycolormap EXCLUDE_FROM_ALL)

set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/contrib/imgui")
if(NOT EXISTS "${IMGUI_DIR}/imgui.cpp")
    message(FATAL_ERROR "Dear ImGui submodule is missing. Run: git submodule update --init --recursive")
endif()

add_library(imgui STATIC
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)
target_include_directories(imgui SYSTEM PUBLIC
    "${IMGUI_DIR}"
    "${IMGUI_DIR}/backends"
    "${IMGUI_DIR}/misc/cpp"
)
target_link_libraries(imgui PUBLIC SDL3::SDL3 OpenGL::GL)

add_library(airspy-tv-common STATIC
    src/debug.cpp
    src/fec/outer_fec.cpp
    src/fec/reed_solomon.cpp
    src/fec/soft_viterbi.cpp
    src/jsonl.cpp
    src/rtp_udp_output.cpp
    src/transport_output.cpp
    src/transport_write_all.cpp
)
target_include_directories(airspy-tv-common PUBLIC include)
target_link_libraries(airspy-tv-common PUBLIC Threads::Threads
    nlohmann_json::nlohmann_json PRIVATE libfec::rs viterbi-decoder)
if(AIRSPY_TV_USE_SIMD_VITERBI)
    target_compile_definitions(airspy-tv-common PRIVATE
        AIRSPY_TV_USE_SIMD_VITERBI=1)
endif()

add_library(airspy-tv-dvbt STATIC
    src/dvbt/analysis_publisher.cpp
    src/dvbt/clock_control_timeline.cpp
    src/dvbt/decoder.cpp
    src/dvbt/demod_dsp.cpp
    src/dvbt/demod_stage.cpp
    src/dvbt/demod_stage_output.cpp
    src/dvbt/demod_stage_session.cpp
    src/dvbt/demod_stage_symbol.cpp
    src/dvbt/demod_stage_worker.cpp
    src/dvbt/fec_stage.cpp
    src/dvbt/frontend_stage.cpp
    src/dvbt/frontend_stage_worker.cpp
    src/dvbt/inner_decoder.cpp
    src/dvbt/ofdm_acquisition.cpp
    src/dvbt/sample_channel.cpp
    src/dvbt/stream_decoder.cpp
    src/dvbt/streaming_resampler.cpp
    src/dvbt/signal_analyzer.cpp
    src/dvbt/soft_demapper.cpp
    src/dvbt/transport_decoder.cpp
    src/dvbt/tps_decoder.cpp
)
target_include_directories(airspy-tv-dvbt PUBLIC include)
target_link_libraries(airspy-tv-dvbt PUBLIC PkgConfig::FFTW3F airspy-tv-common
    Threads::Threads PRIVATE solid-resampler::solid-resampler)

add_library(airspy-tv-runtime STATIC
    src/decode_progress.cpp
    src/dvbt/dvbt_decode_report.cpp
    src/dvbt/dvbt_report_codec.cpp
    src/dvbt/dvbt_report_writer.cpp
    src/decode_run_reporter.cpp
    src/epg.cpp
    src/iq_file.cpp
    src/iq_source.cpp
    src/receiver_session.cpp
    src/receiver_controller.cpp
    src/receiver_pipeline.cpp
    src/recorder.cpp
    src/sdr.cpp
    src/si_common.cpp
    src/spectrum.cpp
    src/telemetry_stream_router.cpp
    src/transport_observer.cpp
    src/transport_pipeline.cpp
    src/transport_stream.cpp
)
target_include_directories(airspy-tv-runtime PUBLIC include PRIVATE src)
target_link_libraries(airspy-tv-runtime PUBLIC
    airspy-tv-dvbt
    nlohmann_json::nlohmann_json
    PkgConfig::AIRSPY
    PkgConfig::FFTW3F
    SoapySDR
    Threads::Threads
)

add_executable(airspy-tv
    src/gui/app.cpp
    src/gui/decode_reporting.cpp
    src/gui/dvbt_panels.cpp
    src/gui/media_panels.cpp
    src/gui/receiver_panels.cpp
    src/gui/source_panels.cpp
    src/gui/spectrum_panels.cpp
    src/gui/widgets.cpp
    src/live_decode.cpp
    src/main.cpp
    src/main_cli.cpp
    src/main_commands.cpp
    src/mpv_player.cpp
    src/playback_stream_buffer.cpp
    src/offline_decode.cpp
)
target_include_directories(airspy-tv PRIVATE include)
target_link_libraries(airspy-tv PRIVATE
    airspy-tv-runtime
    imgui
    nlohmann_json::nlohmann_json
    tinycolormap
    PkgConfig::AIRSPY
    PkgConfig::FFTW3F
    PkgConfig::FONTCONFIG
    PkgConfig::MPV
    SoapySDR
    Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(airspy-tv-common PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    target_compile_options(airspy-tv-dvbt PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    target_compile_options(airspy-tv-runtime PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    target_compile_options(airspy-tv PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

include(CTest)
if(BUILD_TESTING)
    function(airspy_tv_test name library)
        string(REPLACE "-" "_" source "${name}")
        add_executable(${name}-test tests/${source}_test.cpp ${ARGN})
        target_include_directories(${name}-test PRIVATE include src)
        if(library)
            target_link_libraries(${name}-test PRIVATE ${library})
        endif()
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
            target_compile_options(${name}-test PRIVATE
                -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
        endif()
        add_test(NAME ${name} COMMAND ${name}-test)
    endfunction()

    foreach(name jsonl sample-timeline transport-output rtp-udp-output)
        airspy_tv_test(${name} airspy-tv-common)
    endforeach()
    airspy_tv_test(reed-solomon "airspy-tv-common;libfec::rs")
    airspy_tv_test(frequency-translating-resampler solid-resampler::solid-resampler)

    foreach(name dvbt-inner-decoder dvbt-transport-decoder dvbt-fec-stage
                 dvbt-sample-channel dvbt-demod-dsp dvbt-signal-analyzer)
        airspy_tv_test(${name} airspy-tv-dvbt)
    endforeach()
    foreach(name dvbt-fec-stage dvbt-sample-channel dvbt-demod-dsp)
        target_include_directories(${name}-test PRIVATE src/dvbt)
    endforeach()

    add_executable(dvbt-stream-decoder-test tests/dvbt_stream_decoder_test.cpp)
    target_link_libraries(dvbt-stream-decoder-test PRIVATE airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-stream-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    foreach(STREAM_CASE clean cfo-centering cfo-drift cfo-rebootstrap
                        worker-parity queue-capacity reset-new reset-hopeless
                        mode-transition lifecycle-stress reset-live reset-wait
                        no-acq)
        add_test(NAME dvbt-stream-decoder-${STREAM_CASE}
                 COMMAND dvbt-stream-decoder-test)
        set_tests_properties(dvbt-stream-decoder-${STREAM_CASE} PROPERTIES
            ENVIRONMENT "AIRSPYTV_STREAM_TEST=${STREAM_CASE}"
            TIMEOUT 90)
    endforeach()

    foreach(name decode-report receiver-controller telemetry-stream-router epg-model recorder
                 sdr-file-source transport-observer transport-pipeline
                 transport-stream-model)
        airspy_tv_test(${name} airspy-tv-runtime)
    endforeach()
    airspy_tv_test(byte-rate-tracker "")
    airspy_tv_test(vector-ops "")
    airspy_tv_test(playback-stream-buffer Threads::Threads src/playback_stream_buffer.cpp)
    set_tests_properties(dvbt-signal-analyzer sdr-file-source transport-output
                         playback-stream-buffer transport-stream-model
                         PROPERTIES TIMEOUT 30)
endif()

install(TARGETS airspy-tv RUNTIME DESTINATION bin)
