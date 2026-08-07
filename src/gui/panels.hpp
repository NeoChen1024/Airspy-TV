#pragma once

#include <cstdint>

namespace airspy_tv::gui {

struct AppState;

void initialize_standard_state(AppState &state);
void update_standard_state(AppState &state);
void refresh_devices(AppState &state);
void request_center_frequency(AppState &state, std::uint64_t frequency_hz);
void draw_source_panel(AppState &state);
void draw_receiver_panel(AppState &state);
void draw_standard_settings_panel(AppState &state);
void draw_spectrum_panel(AppState &state);
void draw_constellation_panel(AppState &state);
void draw_common_signal_panel(AppState &state);
void draw_standard_diagnostics_panel(AppState &state);
void draw_playback_panel(AppState &state);
void draw_epg_panel(AppState &state);
void draw_ts_recorder_panel(AppState &state);
void draw_recorder_panel(AppState &state);
void draw_video_panel(AppState &state);

} // namespace airspy_tv::gui
