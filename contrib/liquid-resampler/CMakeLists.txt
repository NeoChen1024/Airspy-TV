add_library(liquid-resampler STATIC
    src/arbitrary_resampler.cpp
)
add_library(liquid-resampler::liquid-resampler ALIAS liquid-resampler)

target_include_directories(liquid-resampler
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_features(liquid-resampler PUBLIC cxx_std_20)
target_link_libraries(liquid-resampler PUBLIC Threads::Threads)
set_target_properties(liquid-resampler PROPERTIES
    OUTPUT_NAME liquid-resampler
    POSITION_INDEPENDENT_CODE ON
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(liquid-resampler PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()
