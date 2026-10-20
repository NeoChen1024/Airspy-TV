#pragma once

#include <filesystem>
#include <optional>

namespace airspy_tv::gui {

struct GuiOptions {
    std::optional<std::filesystem::path> report_directory;
    // Opened at startup as if chosen from the Source panel.
    std::optional<std::filesystem::path> iq_input;
};

int run_gui(GuiOptions options = {});

} // namespace airspy_tv::gui
