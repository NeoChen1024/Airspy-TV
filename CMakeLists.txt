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
pkg_check_modules(VOLK REQUIRED IMPORTED_TARGET volk)

set(JSON_BuildTests OFF CACHE INTERNAL "Disable nlohmann/json tests")
add_subdirectory(contrib/json EXCLUDE_FROM_ALL)

set(BUILD_EXAMPLES OFF CACHE BOOL "Disable liquid-dsp examples" FORCE)
set(BUILD_AUTOTESTS OFF CACHE BOOL "Disable liquid-dsp tests" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Disable liquid-dsp benchmarks" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build liquid-dsp shared library" FORCE)
set(BUILD_STATIC_LIBS OFF CACHE BOOL "Disable liquid-dsp static library" FORCE)
set(ENABLE_LOGGING OFF CACHE BOOL "Disable liquid-dsp logging" FORCE)
# liquid-dsp controls call-site compilation separately from ENABLE_LOGGING.
# Keep only exceptional fatal call sites so trace calls do not remain in DSP
# inner loops as no-op logger invocations.
set(LOGGING_LEVEL "fatal" CACHE STRING
    "Compile only fatal liquid-dsp logging call sites" FORCE)
add_subdirectory(contrib/liquid-dsp EXCLUDE_FROM_ALL)

# libcorrect's legacy CMakeLists injects a private ASan flag whenever the
# enclosing build type is Debug, without propagating the ASan runtime to its
# consumers. Isolate that subproject policy while keeping Airspy TV's own
# selected build type unchanged.
set(AIRSPY_TV_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
set(CMAKE_BUILD_TYPE Release)
# libcorrect caches a failed SIMD probe across build-policy changes. Re-run it
# so an existing Debug tree does not remain permanently pinned to scalar
# Viterbi after switching to optimized Debug flags.
unset(HAVE_SSE)
unset(HAVE_SSE CACHE)
add_subdirectory(contrib/libcorrect EXCLUDE_FROM_ALL)
set(CMAKE_BUILD_TYPE "${AIRSPY_TV_BUILD_TYPE}")
unset(AIRSPY_TV_BUILD_TYPE)

# ViterbiDecoderCpp (LGPL-2.1, header-only) AVX2 soft Viterbi backend. The
# decoder type is a compile-time template choice; its AVX2 intrinsics require
# -mavx2 (implied by the default AIRSPY_TV_NATIVE_ARCH=-march=native). When
# AVX2 is unavailable at compile time, soft_viterbi.cpp falls back to
# libcorrect's convolutional decoder automatically.
option(AIRSPY_TV_USE_AVX2_VITERBI
       "Use ViterbiDecoderCpp AVX2 for the soft Viterbi decoder" ON)
if(AIRSPY_TV_USE_AVX2_VITERBI)
    add_library(viterbi-decoder INTERFACE)
    target_include_directories(viterbi-decoder SYSTEM INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/contrib/viterbi-decoder-cpp/include")
    target_compile_features(viterbi-decoder INTERFACE cxx_std_17)
    # The AVX2 intrinsics only compile when the translation unit is built with
    # -mavx2 (implied by the default -march=native). Warn when the current
    # target cannot provide it, so the silent libcorrect fallback in
    # soft_viterbi.cpp is not mistaken for an AVX2 build. try_compile does not
    # inherit add_compile_options, so mirror the TU's actual flags here.
    include(CheckCXXSourceCompiles)
    unset(AIRSPY_TV_HAVE_AVX2)
    unset(AIRSPY_TV_HAVE_AVX2 CACHE)
    set(_airspy_tv_check_flags "")
    if(AIRSPY_TV_NATIVE_ARCH AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        # Mirror the TU's actual flags: with -march=native the probe result
        # matches what soft_viterbi.cpp sees via __AVX2__, including on hosts
        # whose CPU lacks AVX2 (where the TU silently falls back).
        set(_airspy_tv_check_flags "-march=native")
    endif()
    set(CMAKE_REQUIRED_FLAGS "${_airspy_tv_check_flags}")
    check_cxx_source_compiles("
        #include <immintrin.h>
        int main() { __m256i x = _mm256_setzero_si256(); (void)x; return 0; }
    " AIRSPY_TV_HAVE_AVX2)
    unset(CMAKE_REQUIRED_FLAGS)
    if(NOT AIRSPY_TV_HAVE_AVX2)
        message(WARNING "AIRSPY_TV_USE_AVX2_VITERBI is ON but the compiler "
                        "target lacks AVX2 (e.g. AIRSPY_TV_NATIVE_ARCH=OFF); "
                        "soft_viterbi.cpp will fall back to libcorrect.")
    endif()
endif()

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
    src/fec/outer_fec.cpp
    src/fec/soft_viterbi.cpp
)
target_include_directories(airspy-tv-common PUBLIC include)
target_include_directories(airspy-tv-common SYSTEM PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/contrib/libcorrect/include")
target_link_libraries(airspy-tv-common PUBLIC Threads::Threads
    PRIVATE correct_static)
if(AIRSPY_TV_USE_AVX2_VITERBI)
    target_link_libraries(airspy-tv-common PRIVATE viterbi-decoder)
    target_compile_definitions(airspy-tv-common PRIVATE
        AIRSPY_TV_USE_AVX2_VITERBI=1)
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
    Threads::Threads PRIVATE liquid::liquid PkgConfig::VOLK)

add_executable(airspy-tv
    src/epg.cpp
    src/iq_file.cpp
    src/main.cpp
    src/mpv_player.cpp
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
    PkgConfig::VOLK
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
    add_executable(dvbt-inner-decoder-test
        tests/dvbt_inner_decoder_test.cpp
    )
    target_link_libraries(dvbt-inner-decoder-test PRIVATE airspy-tv-dvbt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-inner-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME dvbt-inner-decoder COMMAND dvbt-inner-decoder-test)

    add_executable(dvbt-transport-decoder-test
        tests/dvbt_transport_decoder_test.cpp
    )
    target_include_directories(dvbt-transport-decoder-test SYSTEM PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/contrib/libcorrect/include")
    target_link_libraries(dvbt-transport-decoder-test PRIVATE
        airspy-tv-dvbt correct_static)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-transport-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME dvbt-transport-decoder COMMAND dvbt-transport-decoder-test)

    add_executable(dvbt-stream-decoder-test
        tests/dvbt_stream_decoder_test.cpp
    )
    target_include_directories(dvbt-stream-decoder-test SYSTEM PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/contrib/libcorrect/include")
    target_link_libraries(dvbt-stream-decoder-test PRIVATE
        airspy-tv-dvbt correct_static)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(dvbt-stream-decoder-test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    foreach(STREAM_CASE clean reset-new reset-live reset-wait no-acq)
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
endif()

install(TARGETS airspy-tv RUNTIME DESTINATION bin)
