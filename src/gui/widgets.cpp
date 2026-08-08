#include "widgets.hpp"

#include "decode_reporting.hpp"
#include "panels.hpp"

#include <fontconfig/fontconfig.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <memory>
#include <mutex>

namespace airspy_tv::gui {
namespace {

constexpr std::uint64_t max_display_frequency = 999'999'999'999ULL;
constexpr std::array<std::uint64_t, 12> digit_steps{
    100'000'000'000ULL,
    10'000'000'000ULL,
    1'000'000'000ULL,
    100'000'000ULL,
    10'000'000ULL,
    1'000'000ULL,
    100'000ULL,
    10'000ULL,
    1'000ULL,
    100ULL,
    10ULL,
    1ULL,
};

} // namespace

void SDLCALL file_dialog_callback(void *userdata, const char *const *filelist,
                                  const int selected_filter) {
    static_cast<void>(selected_filter);
    const std::unique_ptr<std::shared_ptr<FileDialogState>> callback_state(
        static_cast<std::shared_ptr<FileDialogState> *>(userdata));
    const std::shared_ptr<FileDialogState> &state = *callback_state;
    const std::scoped_lock lock(state->mutex);
    state->open = false;
    if (filelist == nullptr) {
        state->error = SDL_GetError();
    } else if (filelist[0] != nullptr) {
        state->selected_path = filelist[0];
    }
}

void apply_dark_theme() {
    ImGui::StyleColorsDark();
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 0.0F;
    style.ChildRounding = 5.0F;
    style.FrameRounding = 4.0F;
    style.PopupRounding = 4.0F;
    style.ScrollbarRounding = 4.0F;
    style.WindowPadding = ImVec2(10.0F, 10.0F);
    style.FramePadding = ImVec2(8.0F, 5.0F);
    style.ItemSpacing = ImVec2(8.0F, 7.0F);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.035F, 0.043F, 0.055F, 1.0F);
    style.Colors[ImGuiCol_ChildBg] = ImVec4(0.055F, 0.065F, 0.080F, 1.0F);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.09F, 0.11F, 0.14F, 1.0F);
    style.Colors[ImGuiCol_Header] = ImVec4(0.10F, 0.28F, 0.43F, 1.0F);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.12F, 0.40F, 0.62F, 1.0F);
    style.Colors[ImGuiCol_Button] = ImVec4(0.10F, 0.32F, 0.50F, 1.0F);
    style.Colors[ImGuiCol_ButtonHovered] = accent;
    style.Colors[ImGuiCol_CheckMark] = accent;
    style.Colors[ImGuiCol_SliderGrab] = accent;
}

[[nodiscard]] bool load_system_monospace_font(ImGuiIO &io, std::string &error) {
    if (FcInit() == FcFalse) {
        error = "Fontconfig initialization failed";
        return false;
    }

    FcPattern *pattern = FcPatternCreate();
    if (pattern == nullptr) {
        error = "Fontconfig could not create a font query";
        return false;
    }

    const auto *family = reinterpret_cast<const FcChar8 *>("monospace");
    if (FcPatternAddString(pattern, FC_FAMILY, family) == FcFalse ||
        FcConfigSubstitute(nullptr, pattern, FcMatchPattern) == FcFalse) {
        FcPatternDestroy(pattern);
        error = "Fontconfig could not prepare the system monospace query";
        return false;
    }
    FcDefaultSubstitute(pattern);

    FcResult result = FcResultNoMatch;
    FcFontSet *matches = FcFontSort(nullptr, pattern, FcTrue, nullptr, &result);
    FcPatternDestroy(pattern);
    if (matches == nullptr || result != FcResultMatch) {
        if (matches != nullptr) {
            FcFontSetDestroy(matches);
        }
        error = "Fontconfig could not resolve the system monospace font";
        return false;
    }

    // Dear ImGui does not consult the platform font fallback mechanism. Merge
    // the first few fonts from Fontconfig's coverage-trimmed system fallback
    // order to provide the behavior desktop toolkits normally supply. The
    // query intentionally carries no language or character-set requirement.
    constexpr int maximum_font_sources = 6;
    int loaded_sources = 0;
    for (int match_index = 0;
         match_index < matches->nfont && loaded_sources < maximum_font_sources;
         ++match_index) {
        FcChar8 *font_file = nullptr;
        const FcPattern *match = matches->fonts[match_index];
        if (FcPatternGetString(match, FC_FILE, 0, &font_file) !=
                FcResultMatch ||
            font_file == nullptr) {
            continue;
        }

        int font_index = 0;
        static_cast<void>(FcPatternGetInteger(match, FC_INDEX, 0, &font_index));
        const std::string font_path{reinterpret_cast<const char *>(font_file)};

        ImFontConfig font_config;
        font_config.FontNo = static_cast<ImU32>(font_index);
        font_config.MergeMode = loaded_sources != 0;
        if (io.Fonts->AddFontFromFileTTF(font_path.c_str(), 13.0F,
                                         &font_config) != nullptr) {
            ++loaded_sources;
        }
    }
    FcFontSetDestroy(matches);
    if (loaded_sources == 0) {
        error = "Dear ImGui could not load any system monospace fonts";
        return false;
    }
    return true;
}

void refresh_devices(AppState &state) {
    state.enumeration = SdrDevice::enumerate(state.show_soapy_airspy);
    if (state.enumeration.devices.empty()) {
        state.selected_device = 0;
    } else {
        state.selected_device = std::min(state.selected_device,
                                         state.enumeration.devices.size() - 1);
    }
    if (state.enumeration.devices.empty()) {
        state.status = "No SDR devices found";
    } else {
        state.status = std::format("Found {} SDR device(s)",
                                   state.enumeration.devices.size());
    }
}

static std::string format_frequency_step(const std::uint64_t step_hz) {
    if (step_hz >= 1'000'000'000ULL) {
        return std::format("{} GHz", step_hz / 1'000'000'000ULL);
    }
    if (step_hz >= 1'000'000ULL) {
        return std::format("{} MHz", step_hz / 1'000'000ULL);
    }
    if (step_hz >= 1'000ULL) {
        return std::format("{} kHz", step_hz / 1'000ULL);
    }
    return std::format("{} Hz", step_hz);
}

std::optional<std::uint64_t>
draw_frequency_control(const char *id, const std::uint64_t frequency_hz) {
    const std::string digits =
        std::format("{:012}", std::min(frequency_hz, max_display_frequency));
    const float font_size = ImGui::GetFontSize() * 2.0F;
    const float digit_width = (ImGui::CalcTextSize("0").x * 2.0F) + 6.0F;
    const float separator_width = (ImGui::CalcTextSize(".").x * 2.0F) + 2.0F;
    const float height = font_size + 10.0F;
    std::optional<std::uint64_t> requested;
    bool significant_digit_seen = false;

    ImGui::PushID(id);
    ImGui::BeginGroup();
    for (std::size_t index = 0; index < digits.size(); ++index) {
        if (index != 0) {
            ImGui::SameLine(0.0F, 0.0F);
        }
        if (index != 0 && index % 3 == 0) {
            ImGui::PushID(static_cast<int>(index));
            ImGui::InvisibleButton("separator",
                                   ImVec2(separator_width, height));
            const ImVec2 separator_min = ImGui::GetItemRectMin();
            ImGui::GetWindowDrawList()->AddText(
                ImGui::GetFont(), font_size,
                ImVec2(separator_min.x + 1.0F, separator_min.y + 4.0F),
                ImGui::GetColorU32(ImGuiCol_TextDisabled), ".");
            ImGui::PopID();
            ImGui::SameLine(0.0F, 0.0F);
        }

        ImGui::PushID(static_cast<int>(index));
        ImGui::InvisibleButton("digit", ImVec2(digit_width, height));
        const ImVec2 item_min = ImGui::GetItemRectMin();
        const ImVec2 item_max = ImGui::GetItemRectMax();
        const bool hovered = ImGui::IsItemHovered();
        if (hovered) {
            ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
            ImGui::GetWindowDrawList()->AddRectFilled(
                item_min, item_max, ImGui::GetColorU32(ImGuiCol_HeaderHovered),
                3.0F);
        }

        int direction = 0;
        int repeat = 1;
        if (hovered && ImGui::GetIO().MouseWheel != 0.0F) {
            const float wheel = ImGui::GetIO().MouseWheel;
            direction = wheel > 0.0F ? 1 : -1;
            repeat =
                std::max(1, static_cast<int>(std::lround(std::abs(wheel))));
        }
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            const float middle = (item_min.y + item_max.y) * 0.5F;
            direction = ImGui::GetMousePos().y < middle ? 1 : -1;
        }

        if (direction != 0) {
            const std::uint64_t step = digit_steps[index];
            const std::uint64_t delta =
                step * static_cast<std::uint64_t>(repeat);
            if (direction > 0) {
                requested = frequency_hz > max_display_frequency - delta
                                ? max_display_frequency
                                : frequency_hz + delta;
            } else {
                requested = frequency_hz > delta ? frequency_hz - delta : 0;
            }
        }

        significant_digit_seen |= digits[index] != '0';
        const ImU32 colour = ImGui::GetColorU32(
            significant_digit_seen ? ImGuiCol_Text : ImGuiCol_TextDisabled);
        const std::array<char, 2> digit_text{digits[index], '\0'};
        ImGui::GetWindowDrawList()->AddText(
            ImGui::GetFont(), font_size,
            ImVec2(item_min.x + 3.0F, item_min.y + 4.0F), colour,
            digit_text.data());
        if (hovered) {
            ImGui::SetTooltip(
                "Wheel/click: +/- %s",
                format_frequency_step(digit_steps[index]).c_str());
        }
        ImGui::PopID();
    }
    ImGui::EndGroup();
    ImGui::PopID();
    return requested;
}

void request_center_frequency(AppState &state,
                              const std::uint64_t frequency_hz) {
    if (!state.session.is_open()) {
        state.settings.center_frequency_hz = frequency_hz;
        state.status = "Center frequency selected";
        return;
    }

    finish_decode_report_source(state);
    prepare_decode_report(state);
    std::string error;
    if (state.session.retune(frequency_hz, error)) {
        state.settings.center_frequency_hz = frequency_hz;
        state.status = "Center frequency applied";
        std::string report_error;
        if (!start_decode_report(state, report_error)) {
            state.status += "; report unavailable: " + report_error;
        }
    } else {
        state.status = error;
        if (state.session.is_streaming()) {
            std::string report_error;
            if (!start_decode_report(state, report_error)) {
                state.status += "; report unavailable: " + report_error;
            }
        } else {
            cancel_decode_report_start(state);
        }
    }
}

void draw_metric(const char *label, const char *value, const float fraction,
                 const ImVec4 colour) {
    ImGui::TextUnformatted(label);
    ImGui::SameLine(115.0F);
    ImGui::TextColored(colour, "%s", value);
    ImGui::ProgressBar(fraction, ImVec2(-1.0F, 5.0F), "");
}

void draw_disabled_wrapped(const std::string_view text) {
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%.*s", static_cast<int>(text.size()), text.data());
    ImGui::PopStyleColor();
}

void draw_status_indicator(const char *label, const ImVec4 colour) {
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    const ImVec2 indicator_position{
        cursor.x + 5.0F, cursor.y + (ImGui::GetTextLineHeight() * 0.5F)};
    ImGui::GetWindowDrawList()->AddCircleFilled(
        indicator_position, 4.0F, ImGui::ColorConvertFloat4ToU32(colour));
    ImGui::Dummy(ImVec2(12.0F, ImGui::GetTextLineHeight()));
    ImGui::SameLine();
    ImGui::TextColored(colour, "%s", label);
}

void draw_bipolar_metric(const char *label, const char *value,
                         const float position, const ImVec4 colour) {
    ImGui::TextUnformatted(label);
    ImGui::SameLine(115.0F);
    ImGui::TextColored(colour, "%s", value);

    ImGui::PushID(label);
    ImGui::InvisibleButton("meter", ImVec2(-1.0F, 5.0F));
    ImGui::PopID();

    const ImVec2 bar_min = ImGui::GetItemRectMin();
    const ImVec2 bar_max = ImGui::GetItemRectMax();
    const float center_x = (bar_min.x + bar_max.x) * 0.5F;
    const float marker_x = bar_min.x + ((bar_max.x - bar_min.x) *
                                        std::clamp(position, 0.0F, 1.0F));
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(bar_min, bar_max, ImGui::GetColorU32(ImGuiCol_FrameBg),
                        ImGui::GetStyle().FrameRounding);
    if (marker_x != center_x) {
        draw->AddRectFilled(ImVec2(std::min(center_x, marker_x), bar_min.y),
                            ImVec2(std::max(center_x, marker_x), bar_max.y),
                            ImGui::ColorConvertFloat4ToU32(colour));
    }
    draw->AddLine(ImVec2(center_x, bar_min.y - 1.0F),
                  ImVec2(center_x, bar_max.y + 1.0F),
                  IM_COL32(225, 235, 245, 220));
    draw->AddLine(ImVec2(marker_x, bar_min.y), ImVec2(marker_x, bar_max.y),
                  ImGui::ColorConvertFloat4ToU32(colour), 2.0F);
}

void consume_file_dialog_result(AppState &state,
                                const std::shared_ptr<FileDialogState> &dialog,
                                std::string &path,
                                const std::string_view description) {
    const std::scoped_lock lock(dialog->mutex);
    if (dialog->selected_path.has_value()) {
        path = std::move(*dialog->selected_path);
        dialog->selected_path.reset();
        state.status = std::string(description) + " path selected";
    }
    if (dialog->error.has_value()) {
        state.status = "File dialog: " + *dialog->error;
        dialog->error.reset();
    }
}

bool file_dialog_is_open(const std::shared_ptr<FileDialogState> &dialog) {
    const std::scoped_lock lock(dialog->mutex);
    return dialog->open;
}

void show_recording_file_dialog(
    AppState &state, const std::shared_ptr<FileDialogState> &dialog,
    const std::string &path,
    const std::span<const SDL_DialogFileFilter> filters) {
    {
        const std::scoped_lock lock(dialog->mutex);
        if (dialog->open) {
            return;
        }
        dialog->open = true;
        dialog->default_location = path;
    }

    auto callback_state =
        std::make_unique<std::shared_ptr<FileDialogState>>(dialog);
    SDL_ShowSaveFileDialog(&file_dialog_callback, callback_state.release(),
                           state.window, filters.data(),
                           static_cast<int>(filters.size()),
                           dialog->default_location.c_str());
}

} // namespace airspy_tv::gui
