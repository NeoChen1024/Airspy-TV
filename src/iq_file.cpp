#include "airspy_tv/iq_file.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <utility>

namespace airspy_tv {
namespace {

[[nodiscard]] bool is_json_path(const std::filesystem::path &path) {
    std::string extension = path.extension().string();
    std::ranges::transform(extension, extension.begin(), [](const char value) {
        return static_cast<char>(
            std::tolower(static_cast<unsigned char>(value)));
    });
    return extension == ".json";
}

} // namespace

bool resolve_iq_file(const std::filesystem::path &path,
                     const std::uint32_t raw_sample_rate_hz,
                     const std::uint64_t raw_center_frequency_hz,
                     IqFileInfo &info, std::string &error) {
    if (path.empty()) {
        error = "I/Q source path is empty";
        return false;
    }

    IqFileInfo resolved{
        .data_path = path,
        .source = "airspy_rx INT16_IQ",
        .sample_rate_hz = raw_sample_rate_hz,
        .center_frequency_hz = raw_center_frequency_hz,
    };
    try {
        if (is_json_path(path)) {
            std::ifstream sidecar(path);
            if (!sidecar) {
                error = "Unable to open I/Q metadata: " + path.string();
                return false;
            }
            const nlohmann::json metadata = nlohmann::json::parse(sidecar);
            if (metadata.value("datatype", std::string{}) != "ci16_le" ||
                metadata.value("iq_order", std::string{}) != "IQ") {
                error = "Only ci16_le metadata with IQ ordering is supported";
                return false;
            }

            const std::string data_file = metadata.value("data_file", "");
            if (data_file.empty()) {
                resolved.data_path = path;
                resolved.data_path.replace_extension();
            } else {
                const std::filesystem::path relative(data_file);
                if (relative.is_absolute() || relative.has_parent_path() ||
                    relative.filename() != relative) {
                    error = "I/Q metadata data_file must be a filename in the "
                            "same directory";
                    return false;
                }
                resolved.data_path = path.parent_path() / relative;
            }

            const auto sample_rate =
                metadata.at("sample_rate").get<std::uint64_t>();
            if (sample_rate == 0 ||
                sample_rate > std::numeric_limits<std::uint32_t>::max()) {
                error = "I/Q metadata sample_rate is out of range";
                return false;
            }
            resolved.sample_rate_hz = static_cast<std::uint32_t>(sample_rate);
            resolved.center_frequency_hz =
                metadata.value("center_frequency", std::uint64_t{});
            resolved.source =
                metadata.value("source", std::string{"Recorded I/Q"});
        } else if (resolved.sample_rate_hz == 0) {
            error = "A positive sample rate is required for raw INT16_IQ";
            return false;
        }

        if (!std::filesystem::is_regular_file(resolved.data_path)) {
            error =
                "I/Q data file was not found: " + resolved.data_path.string();
            return false;
        }
        resolved.file_size_bytes =
            std::filesystem::file_size(resolved.data_path);
        if (resolved.file_size_bytes == 0 ||
            resolved.file_size_bytes % (sizeof(std::int16_t) * 2) != 0) {
            error = "I/Q data file must contain complete interleaved INT16 I/Q "
                    "samples";
            return false;
        }
    } catch (const nlohmann::json::exception &exception) {
        error = std::string("Invalid I/Q metadata: ") + exception.what();
        return false;
    } catch (const std::filesystem::filesystem_error &exception) {
        error =
            std::string("Unable to inspect I/Q source: ") + exception.what();
        return false;
    }

    info = std::move(resolved);
    return true;
}

} // namespace airspy_tv
