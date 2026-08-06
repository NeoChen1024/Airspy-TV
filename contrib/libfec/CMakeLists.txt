add_library(libfec-rs STATIC
    decode_rs_char.c
    encode_rs_char.c
    include/libfec_rs.h
    init_rs_char.c
)
add_library(libfec::rs ALIAS libfec-rs)

target_include_directories(libfec-rs
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_features(libfec-rs PRIVATE c_std_99)
set_target_properties(libfec-rs PROPERTIES
    OUTPUT_NAME fec-rs
    POSITION_INDEPENDENT_CODE ON
)
