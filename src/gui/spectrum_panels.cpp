#include "app_state.hpp"
#include "panels.hpp"

#include <SDL3/SDL_opengl.h>
#include <imgui.h>
#include <tinycolormap.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ranges>
#include <span>

namespace airspy_tv::gui {
namespace {

constexpr float minimum_display_range_db = 1.0F;
constexpr float spectrum_label_margin = 36.0F;

struct ColormapOption {
    const char *name;
    tinycolormap::ColormapType type;
};

constexpr std::array colormap_options{
    ColormapOption{"Cubehelix", tinycolormap::ColormapType::Cubehelix},
    ColormapOption{"Viridis", tinycolormap::ColormapType::Viridis},
    ColormapOption{"Cividis", tinycolormap::ColormapType::Cividis},
    ColormapOption{"Magma", tinycolormap::ColormapType::Magma},
    ColormapOption{"Inferno", tinycolormap::ColormapType::Inferno},
    ColormapOption{"Plasma", tinycolormap::ColormapType::Plasma},
    ColormapOption{"Turbo", tinycolormap::ColormapType::Turbo},
    ColormapOption{"Parula", tinycolormap::ColormapType::Parula},
    ColormapOption{"Heat", tinycolormap::ColormapType::Heat},
    ColormapOption{"Hot", tinycolormap::ColormapType::Hot},
    ColormapOption{"Gray", tinycolormap::ColormapType::Gray},
    ColormapOption{"Jet", tinycolormap::ColormapType::Jet},
    ColormapOption{"HSV", tinycolormap::ColormapType::HSV},
    ColormapOption{"GitHub", tinycolormap::ColormapType::Github},
};

float spectrum_column_peak(const std::span<const float> bins,
                           const std::size_t column,
                           const std::size_t column_count) {
    const std::size_t begin = (column * bins.size()) / column_count;
    const std::size_t end =
        std::max(begin + 1, ((column + 1) * bins.size()) / column_count);
    return *std::ranges::max_element(bins.subspan(begin, end - begin));
}

void draw_bandwidth_overlay(ImDrawList &draw, const ImVec2 plot_origin,
                            const ImVec2 extent,
                            const std::uint32_t sample_rate_hz,
                            const std::uint32_t channel_bandwidth_hz) {
    if (sample_rate_hz == 0 || channel_bandwidth_hz == 0) {
        return;
    }
    const float bandwidth_fraction =
        std::min(static_cast<float>(channel_bandwidth_hz) /
                     static_cast<float>(sample_rate_hz),
                 1.0F);
    const float overlay_width = (extent.x - plot_origin.x) * bandwidth_fraction;
    const float center = (plot_origin.x + extent.x) * 0.5F;
    draw.AddRectFilled(ImVec2(center - (overlay_width * 0.5F), plot_origin.y),
                       ImVec2(center + (overlay_width * 0.5F), extent.y),
                       IM_COL32(255, 255, 255, 87));
}

void draw_spectrum(const ImVec2 size, const SpectrumSnapshot &spectrum,
                   const std::uint32_t channel_bandwidth_hz,
                   const float floor_dbfs, const float ceiling_dbfs) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    ImGui::InvisibleButton("spectrum-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(4, 9, 15, 255), 4.0F);

    const ImVec2 plot_origin{origin.x + spectrum_label_margin, origin.y};
    const float plot_width = std::max(2.0F, extent.x - plot_origin.x);
    constexpr int grid_divisions = 4;
    for (int grid = 0; grid <= grid_divisions; ++grid) {
        const float fraction =
            static_cast<float>(grid) / static_cast<float>(grid_divisions);
        const float level =
            ceiling_dbfs - (fraction * (ceiling_dbfs - floor_dbfs));
        const float y = origin.y + (canvas_size.y * fraction);
        draw->AddLine(ImVec2(origin.x, y), ImVec2(extent.x, y),
                      IM_COL32(38, 51, 65, 160));
        draw->AddText(ImVec2(origin.x + 3.0F,
                             y + (grid == grid_divisions ? -14.0F : 2.0F)),
                      IM_COL32(118, 133, 148, 220),
                      std::format("{:.0f}", level).c_str());
    }
    for (int index = 0; index <= 4; ++index) {
        const float x =
            plot_origin.x + (plot_width * static_cast<float>(index) / 4.0F);
        draw->AddLine(ImVec2(x, origin.y), ImVec2(x, extent.y),
                      IM_COL32(38, 51, 65, 160));
    }
    draw_bandwidth_overlay(*draw, plot_origin, extent, spectrum.sample_rate_hz,
                           channel_bandwidth_hz);

    if (spectrum.valid) {
        const auto column_count =
            static_cast<std::size_t>(std::clamp(plot_width, 2.0F, 768.0F));
        std::vector<ImVec2> points;
        points.reserve(column_count);
        for (std::size_t column = 0; column < column_count; ++column) {
            const float level =
                spectrum_column_peak(spectrum.bins_dbfs, column, column_count);
            const float normalized = std::clamp(
                (level - floor_dbfs) / (ceiling_dbfs - floor_dbfs), 0.0F, 1.0F);
            points.emplace_back(plot_origin.x +
                                    (plot_width * static_cast<float>(column) /
                                     static_cast<float>(column_count - 1)),
                                extent.y - (normalized * canvas_size.y));
        }
        draw->AddPolyline(points.data(), static_cast<int>(points.size()),
                          IM_COL32(45, 205, 235, 255), 0, 1.4F);
    } else {
        draw->AddText(ImVec2(plot_origin.x + 8.0F, origin.y + 28.0F),
                      IM_COL32(150, 167, 184, 255), "Waiting for I/Q data...");
    }
    draw->AddText(ImVec2(plot_origin.x + 8.0F, origin.y + 6.0F),
                  IM_COL32(150, 167, 184, 255), "Spectrum dBFS/bin");
}

void ensure_waterfall_texture(WaterfallDisplay &waterfall) {
    if (waterfall.texture != 0) {
        return;
    }
    glGenTextures(1, &waterfall.texture);
    glBindTexture(GL_TEXTURE_2D, waterfall.texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, waterfall_texture_width,
                 waterfall_texture_height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 waterfall.rgba.data());
}

void recolour_waterfall(WaterfallDisplay &waterfall,
                        const tinycolormap::ColormapType colormap,
                        const float floor_dbfs, const float ceiling_dbfs) {
    constexpr std::size_t lut_size = 256;
    std::array<std::array<std::uint8_t, 4>, lut_size> colour_lut{};
    for (std::size_t index = 0; index < colour_lut.size(); ++index) {
        const tinycolormap::Color colour = tinycolormap::GetColor(
            static_cast<double>(index) /
                static_cast<double>(colour_lut.size() - 1),
            colormap);
        colour_lut[index] = {
            static_cast<std::uint8_t>(
                std::lround(std::clamp(colour.r(), 0.0, 1.0) * 255.0)),
            static_cast<std::uint8_t>(
                std::lround(std::clamp(colour.g(), 0.0, 1.0) * 255.0)),
            static_cast<std::uint8_t>(
                std::lround(std::clamp(colour.b(), 0.0, 1.0) * 255.0)),
            255,
        };
    }

    for (std::size_t index = 0; index < waterfall.history.size(); ++index) {
        const float normalized =
            std::clamp((waterfall.history[index] - floor_dbfs) /
                           (ceiling_dbfs - floor_dbfs),
                       0.0F, 1.0F);
        const auto lut_index = static_cast<std::size_t>(std::lround(
            normalized * static_cast<float>(colour_lut.size() - 1)));
        const std::size_t pixel = index * 4;
        std::ranges::copy(colour_lut[lut_index],
                          waterfall.rgba.begin() +
                              static_cast<std::ptrdiff_t>(pixel));
    }

    ensure_waterfall_texture(waterfall);
    glBindTexture(GL_TEXTURE_2D, waterfall.texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, waterfall_texture_width,
                    waterfall_texture_height, GL_RGBA, GL_UNSIGNED_BYTE,
                    waterfall.rgba.data());
}

void update_waterfall(WaterfallDisplay &waterfall,
                      const SpectrumSnapshot &spectrum,
                      const std::size_t colormap_index, const float floor_dbfs,
                      const float ceiling_dbfs) {
    IM_ASSERT(waterfall.history.size() == waterfall_cell_count);
    IM_ASSERT(waterfall.rgba.size() == waterfall_cell_count * 4);
    bool changed = false;
    if (spectrum.valid && spectrum.sequence != waterfall.sequence) {
        constexpr std::size_t row_size = waterfall_texture_width;
        std::memmove(waterfall.history.data() + row_size,
                     waterfall.history.data(),
                     (waterfall.history.size() - row_size) * sizeof(float));
        for (std::size_t column = 0; column < row_size; ++column) {
            waterfall.history[column] = spectrum_column_peak(
                spectrum.waterfall_bins_dbfs, column, row_size);
        }
        waterfall.sequence = spectrum.sequence;
        changed = true;
    }
    if (waterfall.colormap_index != colormap_index) {
        waterfall.colormap_index = colormap_index;
        changed = true;
    }
    if (waterfall.floor_dbfs != floor_dbfs ||
        waterfall.ceiling_dbfs != ceiling_dbfs) {
        waterfall.floor_dbfs = floor_dbfs;
        waterfall.ceiling_dbfs = ceiling_dbfs;
        changed = true;
    }
    ensure_waterfall_texture(waterfall);
    if (changed) {
        recolour_waterfall(waterfall, colormap_options[colormap_index].type,
                           floor_dbfs, ceiling_dbfs);
    }
}

void draw_waterfall(const ImVec2 size, WaterfallDisplay &waterfall,
                    const SpectrumSnapshot &spectrum,
                    const std::size_t colormap_index,
                    const std::uint32_t channel_bandwidth_hz,
                    const float floor_dbfs, const float ceiling_dbfs) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    update_waterfall(waterfall, spectrum, colormap_index, floor_dbfs,
                     ceiling_dbfs);
    ImGui::InvisibleButton("waterfall-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    const ImVec2 plot_origin{origin.x + spectrum_label_margin, origin.y};
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(4, 9, 15, 255));
    draw->AddImage(ImTextureRef(static_cast<ImTextureID>(waterfall.texture)),
                   plot_origin, extent);
    draw_bandwidth_overlay(*draw, plot_origin, extent, spectrum.sample_rate_hz,
                           channel_bandwidth_hz);
    draw->AddText(ImVec2(plot_origin.x + 8.0F, origin.y + 6.0F), IM_COL32_WHITE,
                  "Waterfall");
}

void draw_constellation(const ImVec2 size, const SignalSnapshot &signal) {
    ImVec2 canvas_size = size;
    if (canvas_size.x <= 0.0F) {
        canvas_size.x = ImGui::GetContentRegionAvail().x;
    }
    ImGui::InvisibleButton("constellation-canvas", canvas_size);
    const ImVec2 origin = ImGui::GetItemRectMin();
    const ImVec2 extent = ImGui::GetItemRectMax();
    const ImVec2 center{(origin.x + extent.x) * 0.5F,
                        (origin.y + extent.y) * 0.5F};
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, extent, IM_COL32(4, 9, 15, 255), 4.0F);
    draw->AddLine(ImVec2(origin.x, center.y), ImVec2(extent.x, center.y),
                  IM_COL32(50, 65, 80, 255));
    draw->AddLine(ImVec2(center.x, origin.y), ImVec2(center.x, extent.y),
                  IM_COL32(50, 65, 80, 255));
    if (signal.signal_locked) {
        constexpr float constellation_extent = 1.55F;
        const float scale = std::min(canvas_size.x, canvas_size.y) * 0.46F /
                            constellation_extent;
        for (std::size_t index = 0; index < signal.constellation_count;
             ++index) {
            const auto point = signal.constellation[index];
            const ImVec2 position{center.x + (point.real() * scale),
                                  center.y - (point.imag() * scale)};
            if (position.x >= origin.x && position.x <= extent.x &&
                position.y >= origin.y && position.y <= extent.y) {
                draw->AddCircleFilled(position, 1.5F,
                                      IM_COL32(70, 220, 255, 150));
            }
        }
    }
    draw->AddText(ImVec2(origin.x + 8.0F, origin.y + 6.0F),
                  IM_COL32(150, 167, 184, 255),
                  signal.signal_locked ? "Equalized constellation"
                                       : "Waiting for signal lock");
}

} // namespace

void WaterfallDisplay::destroy() {
    if (texture != 0) {
        glDeleteTextures(1, &texture);
        texture = 0;
    }
}

void draw_spectrum_panel(AppState &state) {
    if (ImGui::CollapsingHeader("Spectrum & Waterfall",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("spectrum-panel");
        const ColormapOption &selected =
            colormap_options[state.display.selected_colormap];
        ImGui::SetNextItemWidth(-1.0F);
        if (ImGui::BeginCombo("##waterfall-colormap", selected.name)) {
            for (std::size_t index = 0; index < colormap_options.size();
                 ++index) {
                const bool is_selected =
                    index == state.display.selected_colormap;
                if (ImGui::Selectable(colormap_options[index].name,
                                      is_selected)) {
                    state.display.selected_colormap = index;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Waterfall colormap");
        }
        draw_spectrum(ImVec2(-1.0F, 150.0F), state.frame.spectrum,
                      state.frame.channel_bandwidth_hz,
                      state.display.display_floor_dbfs,
                      state.display.display_ceiling_dbfs);
        draw_waterfall(ImVec2(-1.0F, 150.0F), state.display.waterfall,
                       state.frame.spectrum, state.display.selected_colormap,
                       state.frame.channel_bandwidth_hz,
                       state.display.display_floor_dbfs,
                       state.display.display_ceiling_dbfs);

        ImGui::TextDisabled("Display range");
        const float ceiling_min =
            state.display.display_floor_dbfs + minimum_display_range_db;
        ImGui::SliderFloat("Ceiling##spectrum-range",
                           &state.display.display_ceiling_dbfs, ceiling_min,
                           signal_meter_ceiling_dbfs, "%.0f dBFS");
        state.display.display_ceiling_dbfs =
            std::clamp(state.display.display_ceiling_dbfs, ceiling_min,
                       signal_meter_ceiling_dbfs);
        const float floor_max =
            state.display.display_ceiling_dbfs - minimum_display_range_db;
        ImGui::SliderFloat("Floor##spectrum-range",
                           &state.display.display_floor_dbfs,
                           signal_meter_floor_dbfs, floor_max, "%.0f dBFS");
        state.display.display_floor_dbfs =
            std::clamp(state.display.display_floor_dbfs,
                       signal_meter_floor_dbfs, floor_max);

        bool smoothing_changed = false;
        smoothing_changed |=
            ImGui::Checkbox("FFT smoothing", &state.display.fft_smoothing);
        ImGui::BeginDisabled(!state.display.fft_smoothing);
        ImGui::TextUnformatted("FFT smoothing speed");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-1.0F);
        smoothing_changed |= ImGui::InputInt(
            "##fft-smoothing-speed", &state.display.fft_smoothing_speed);
        state.display.fft_smoothing_speed =
            std::max(state.display.fft_smoothing_speed, 1);
        ImGui::EndDisabled();
        smoothing_changed |=
            ImGui::Checkbox("SNR smoothing", &state.display.snr_smoothing);
        ImGui::BeginDisabled(!state.display.snr_smoothing);
        ImGui::TextUnformatted("SNR smoothing speed");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-1.0F);
        smoothing_changed |= ImGui::InputInt(
            "##snr-smoothing-speed", &state.display.snr_smoothing_speed);
        state.display.snr_smoothing_speed =
            std::max(state.display.snr_smoothing_speed, 1);
        ImGui::EndDisabled();
        if (smoothing_changed) {
            state.session.set_display_smoothing(
                state.display.fft_smoothing, state.display.fft_smoothing_speed,
                state.display.snr_smoothing, state.display.snr_smoothing_speed);
        }
        ImGui::PopID();
    }
}

void draw_constellation_panel(AppState &state) {
    if (ImGui::CollapsingHeader("Constellation",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("constellation-panel");
        draw_constellation(ImVec2(-1.0F, 300.0F), state.frame.signal);
        ImGui::PopID();
    }
}

} // namespace airspy_tv::gui
