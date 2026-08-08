#pragma once

#include "app_state.hpp"

#include <string>

namespace airspy_tv::gui {

void prepare_decode_report(AppState &state);
void cancel_decode_report_start(AppState &state);
bool start_decode_report(AppState &state, std::string &error);
void update_decode_report(AppState &state);
void finish_decode_report_source(AppState &state);
void finalize_decode_report(AppState &state);

} // namespace airspy_tv::gui
