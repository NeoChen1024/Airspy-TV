#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace airspy_tv {

struct IqFileInfo {
    std::filesystem::path data_path;
    std::string source;
    std::uint32_t sample_rate_hz{};
    std::uint64_t center_frequency_hz{};
    std::uintmax_t file_size_bytes{};
};

// Resolve either an Airspy TV JSON sidecar or a raw airspy_rx INT16_IQ file.
// JSON data_file values are deliberately restricted to a basename in the
// sidecar's directory.
[[nodiscard]] bool resolve_iq_file(const std::filesystem::path &path,
                                   std::uint32_t raw_sample_rate_hz,
                                   std::uint64_t raw_center_frequency_hz,
                                   IqFileInfo &info, std::string &error);

} // namespace airspy_tv
