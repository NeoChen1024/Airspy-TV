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
pkg_check_modules(VOLK REQUIRED IMPORTED_TARGET volk)

set(JSON_BuildTests OFF CACHE INTERNAL "Disable nlohmann/json tests")
add_subdirectory(contrib/json EXCLUDE_FROM_ALL)

set(BUILD_EXAMPLES OFF CACHE BOOL "Disable liquid-dsp examples" FORCE)
set(BUILD_AUTOTESTS OFF CACHE BOOL "Disable liquid-dsp tests" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Disable liquid-dsp benchmarks" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build liquid-dsp shared library" FORCE)
set(BUILD_STATIC_LIBS OFF CACHE BOOL "Disable liquid-dsp static library" FORCE)
set(ENABLE_LOGGING OFF CACHE BOOL "Disable liquid-dsp logging" FORCE)
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

add_library(airspy-tv-dvbt STATIC
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
target_include_directories(airspy-tv-dvbt SYSTEM PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/contrib/libcorrect/include")
target_link_libraries(airspy-tv-dvbt PUBLIC PkgConfig::FFTW3F Threads::Threads
    PRIVATE correct_static liquid::liquid PkgConfig::VOLK)

add_executable(airspy-tv
    src/iq_file.cpp
    src/main.cpp
    src/recorder.cpp
    src/sdr.cpp
    src/spectrum.cpp
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
    PkgConfig::VOLK
    SoapySDR
    Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
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
endif()

install(TARGETS airspy-tv RUNTIME DESTINATION bin)
