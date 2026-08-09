#pragma once

#include "airspy_tv/sdr.hpp"

#include <cstdint>
#include <filesystem>

namespace airspy_tv::cli {

int enumerate_cli();
int record_first_cli(const std::filesystem::path &path, int duration_ms,
                     const SourceSettings &settings);
int inspect_iq_cli(const std::filesystem::path &path,
                   std::uint32_t raw_sample_rate_hz,
                   std::uint64_t raw_center_frequency_hz);

} // namespace airspy_tv::cli
