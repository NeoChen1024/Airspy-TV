#pragma once

#include "airspy_tv/dvbt/receiver_parameters.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>

int offline_decode_cli(
    const std::filesystem::path &source,
    const std::filesystem::path &destination, std::uint32_t raw_sample_rate_hz,
    const airspy_tv::dvbt::ReceiverParameters &parameters,
    const std::optional<std::filesystem::path> &report_directory);
