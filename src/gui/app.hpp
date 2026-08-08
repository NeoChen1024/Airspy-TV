#pragma once

#include <filesystem>
#include <optional>

namespace airspy_tv::gui {

int run_gui(
    std::optional<std::filesystem::path> report_directory = std::nullopt);

} // namespace airspy_tv::gui
