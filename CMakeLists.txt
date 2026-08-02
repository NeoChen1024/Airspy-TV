cmake_minimum_required(VERSION 3.25)

project(AirspyTV VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

add_executable(airspy-tv
    src/main.cpp
    src/recorder.cpp
    src/sdr.cpp
    src/spectrum.cpp
)
target_include_directories(airspy-tv PRIVATE include)
target_link_libraries(airspy-tv PRIVATE
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
    target_compile_options(airspy-tv PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

install(TARGETS airspy-tv RUNTIME DESTINATION bin)
