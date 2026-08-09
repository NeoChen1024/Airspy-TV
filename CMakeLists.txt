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

# Migrate the old AVX2-specific option when reusing an existing build tree.
set(_airspy_tv_simd_viterbi_default ON)
if(DEFINED AIRSPY_TV_USE_AVX2_VITERBI AND
   NOT DEFINED AIRSPY_TV_USE_SIMD_VITERBI)
    set(_airspy_tv_simd_viterbi_default
        "${AIRSPY_TV_USE_AVX2_VITERBI}")
    message(DEPRECATION "AIRSPY_TV_USE_AVX2_VITERBI is deprecated; use "
                        "AIRSPY_TV_USE_SIMD_VITERBI")
endif()
option(AIRSPY_TV_USE_SIMD_VITERBI
       "Use available ViterbiDecoderCpp SIMD for soft Viterbi"
       "${_airspy_tv_simd_viterbi_default}")
unset(_airspy_tv_simd_viterbi_default)

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
    src/decode_report.cpp
    src/decode_run_reporter.cpp
    src/epg.cpp
    src/iq_file.cpp
    src/iq_source.cpp
    src/receiver_session.cpp
    src/recorder.cpp
    src/sdr.cpp
    src/si_common.cpp
    src/spectrum.cpp
    src/transport_router.cpp
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
    add_executable(jsonl-test
        tests/jsonl_test.cpp
    )
    target_link_libraries(jsonl-test PRIVATE airspy-tv-common)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(jsonl-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME jsonl COMMAND jsonl-test)

    add_executable(decode-report-test
        tests/decode_report_test.cpp
    )
    target_include_directories(decode-report-test PRIVATE src)
    target_link_libraries(decode-report-test PRIVATE airspy-tv-runtime)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(decode-report-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME decode-report COMMAND decode-report-test)

    add_executable(sample-timeline-test
        tests/sample_timeline_test.cpp
    )
    target_link_libraries(sample-timeline-test PRIVATE airspy-tv-common)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(sample-timeline-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME sample-timeline COMMAND sample-timeline-test)

    add_executable(frequency-translating-resampler-test
        tests/frequency_translating_resampler_test.cpp
    )
    target_link_libraries(frequency-translating-resampler-test PRIVATE
        solid-resampler::solid-resampler)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(frequency-translating-resampler-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME frequency-translating-resampler COMMAND frequency-translating-resampler-test)

    add_executable(dvbt-inner-decoder-test
        tests/dvbt_inner_decoder_test.cpp
    )
    target_link_libraries(dvbt-inner-decoder-test PRIVATE airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-inner-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME dvbt-inner-decoder COMMAND dvbt-inner-decoder-test)

    add_executable(reed-solomon-test
        tests/reed_solomon_test.cpp
    )
    target_link_libraries(reed-solomon-test PRIVATE airspy-tv-common)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(reed-solomon-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME reed-solomon COMMAND reed-solomon-test)

    add_executable(dvbt-transport-decoder-test
        tests/dvbt_transport_decoder_test.cpp
    )
    target_link_libraries(dvbt-transport-decoder-test PRIVATE
        airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-transport-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME dvbt-transport-decoder COMMAND dvbt-transport-decoder-test)

    add_executable(dvbt-fec-stage-test
        tests/dvbt_fec_stage_test.cpp
    )
    target_include_directories(dvbt-fec-stage-test PRIVATE src/dvbt)
    target_link_libraries(dvbt-fec-stage-test PRIVATE airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-fec-stage-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME dvbt-fec-stage COMMAND dvbt-fec-stage-test)

    add_executable(dvbt-sample-channel-test
        tests/dvbt_sample_channel_test.cpp
    )
    target_include_directories(dvbt-sample-channel-test PRIVATE src/dvbt)
    target_link_libraries(dvbt-sample-channel-test PRIVATE airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-sample-channel-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME dvbt-sample-channel COMMAND dvbt-sample-channel-test)

    add_executable(dvbt-stream-decoder-test
        tests/dvbt_stream_decoder_test.cpp
    )
    target_link_libraries(dvbt-stream-decoder-test PRIVATE
        airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-stream-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    foreach(STREAM_CASE clean cfo-centering cfo-drift cfo-rebootstrap
                        worker-parity reset-new reset-hopeless reset-live
                        reset-wait no-acq)
        add_test(NAME dvbt-stream-decoder-${STREAM_CASE}
                 COMMAND dvbt-stream-decoder-test)
        set_tests_properties(dvbt-stream-decoder-${STREAM_CASE} PROPERTIES
            ENVIRONMENT "AIRSPYTV_STREAM_TEST=${STREAM_CASE}"
            TIMEOUT 90)
    endforeach()

    add_executable(epg-model-test
        tests/epg_model_test.cpp
    )
    target_include_directories(epg-model-test PRIVATE include src)
    target_link_libraries(epg-model-test PRIVATE airspy-tv-runtime)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(epg-model-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME epg-model COMMAND epg-model-test)

    add_executable(pipeline-load-monitor-test
        tests/pipeline_load_monitor_test.cpp
    )
    target_include_directories(pipeline-load-monitor-test PRIVATE src)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(pipeline-load-monitor-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME pipeline-load-monitor COMMAND pipeline-load-monitor-test)

    add_executable(byte-rate-tracker-test
        tests/byte_rate_tracker_test.cpp
    )
    target_include_directories(byte-rate-tracker-test PRIVATE src)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(byte-rate-tracker-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME byte-rate-tracker COMMAND byte-rate-tracker-test)

    add_executable(recorder-test
        tests/recorder_test.cpp
    )
    target_link_libraries(recorder-test PRIVATE airspy-tv-runtime)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(recorder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME recorder COMMAND recorder-test)

    add_executable(sdr-file-source-test
        tests/sdr_file_source_test.cpp
    )
    target_link_libraries(sdr-file-source-test PRIVATE airspy-tv-runtime)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(sdr-file-source-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME sdr-file-source COMMAND sdr-file-source-test)

    add_executable(transport-output-test
        tests/transport_output_test.cpp
    )
    target_link_libraries(transport-output-test PRIVATE airspy-tv-common)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(transport-output-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME transport-output COMMAND transport-output-test)

    add_executable(rtp-udp-output-test
        tests/rtp_udp_output_test.cpp
    )
    target_link_libraries(rtp-udp-output-test PRIVATE airspy-tv-common)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(rtp-udp-output-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME rtp-udp-output COMMAND rtp-udp-output-test)

    add_executable(vector-ops-test
        tests/vector_ops_test.cpp
    )
    target_include_directories(vector-ops-test PRIVATE include)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(vector-ops-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME vector-ops COMMAND vector-ops-test)
endif()

install(TARGETS airspy-tv RUNTIME DESTINATION bin)
