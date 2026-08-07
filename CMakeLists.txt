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

add_subdirectory(contrib/liquid-resampler EXCLUDE_FROM_ALL)

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
    src/dvbt/decoder.cpp
    src/dvbt/inner_decoder.cpp
    src/dvbt/ofdm_acquisition.cpp
    src/dvbt/stream_decoder.cpp
    src/dvbt/signal_analyzer.cpp
    src/dvbt/soft_demapper.cpp
    src/dvbt/transport_decoder.cpp
    src/dvbt/tps_decoder.cpp
)
target_include_directories(airspy-tv-dvbt PUBLIC include)
target_link_libraries(airspy-tv-dvbt PUBLIC PkgConfig::FFTW3F airspy-tv-common
    Threads::Threads PRIVATE liquid-resampler::liquid-resampler)

add_executable(airspy-tv
    src/epg.cpp
    src/iq_file.cpp
    src/main.cpp
    src/mpv_player.cpp
    src/receiver_session.cpp
    src/recorder.cpp
    src/sdr.cpp
    src/si_common.cpp
    src/spectrum.cpp
    src/transport_router.cpp
    src/transport_stream.cpp
)
target_include_directories(airspy-tv PRIVATE include)
target_link_libraries(airspy-tv PRIVATE
    airspy-tv-dvbt
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

    add_executable(sample-timeline-test
        tests/sample_timeline_test.cpp
    )
    target_link_libraries(sample-timeline-test PRIVATE airspy-tv-common)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(sample-timeline-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME sample-timeline COMMAND sample-timeline-test)

    add_executable(arbitrary-resampler-test
        tests/arbitrary_resampler_test.cpp
    )
    target_link_libraries(arbitrary-resampler-test PRIVATE
        liquid-resampler::liquid-resampler)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(arbitrary-resampler-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME arbitrary-resampler COMMAND arbitrary-resampler-test)

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

    add_executable(dvbt-stream-decoder-test
        tests/dvbt_stream_decoder_test.cpp
    )
    target_link_libraries(dvbt-stream-decoder-test PRIVATE
        airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-stream-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    foreach(STREAM_CASE clean worker-parity reset-new reset-hopeless reset-live
                        reset-wait no-acq)
        add_test(NAME dvbt-stream-decoder-${STREAM_CASE}
                 COMMAND dvbt-stream-decoder-test)
        set_tests_properties(dvbt-stream-decoder-${STREAM_CASE} PROPERTIES
            ENVIRONMENT "AIRSPYTV_STREAM_TEST=${STREAM_CASE}"
            TIMEOUT 90)
    endforeach()

    add_executable(epg-model-test
        tests/epg_model_test.cpp
        src/epg.cpp
        src/si_common.cpp
    )
    target_include_directories(epg-model-test PRIVATE include src)
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
