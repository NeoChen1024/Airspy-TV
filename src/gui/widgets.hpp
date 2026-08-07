#pragma once

#include "app_state.hpp"

#include <SDL3/SDL_dialog.h>
#include <imgui.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace airspy_tv::gui {

inline constexpr ImVec4 accent{0.12F, 0.58F, 0.92F, 1.0F};

void SDLCALL file_dialog_callback(void *userdata, const char *const *filelist,
                                  int selected_filter);
void apply_dark_theme();
[[nodiscard]] bool load_system_monospace_font(ImGuiIO &io, std::string &error);

[[nodiscard]] std::optional<std::uint64_t>
draw_frequency_control(const char *id, std::uint64_t frequency_hz);
void draw_metric(const char *label, const char *value, float fraction,
                 ImVec4 colour);
void draw_disabled_wrapped(std::string_view text);
void draw_status_indicator(const char *label, ImVec4 colour);
void draw_bipolar_metric(const char *label, const char *value, float position,
                         ImVec4 colour);

void consume_file_dialog_result(AppState &state,
                                const std::shared_ptr<FileDialogState> &dialog,
                                std::string &path,
                                std::string_view description);
[[nodiscard]] bool
file_dialog_is_open(const std::shared_ptr<FileDialogState> &dialog);
void show_recording_file_dialog(AppState &state,
                                const std::shared_ptr<FileDialogState> &dialog,
                                const std::string &path,
                                std::span<const SDL_DialogFileFilter> filters);

} // namespace airspy_tv::gui
