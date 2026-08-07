add_library(solid-resampler STATIC
    src/frequency_translating_resampler.cpp
)
add_library(solid-resampler::solid-resampler ALIAS solid-resampler)

target_include_directories(solid-resampler
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_features(solid-resampler PUBLIC cxx_std_20)
target_link_libraries(solid-resampler PUBLIC Threads::Threads)
set_target_properties(solid-resampler PROPERTIES
    OUTPUT_NAME solid-resampler
    POSITION_INDEPENDENT_CODE ON
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(solid-resampler PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()
