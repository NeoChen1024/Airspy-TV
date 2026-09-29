#include "decode_reporting.hpp"

#include <string>

namespace airspy_tv::gui {
void update_decode_report(AppState &state) {
    std::string error;
    if (!state.reporting.decode_report.update(state.session, error)) {
        state.ui.status = "Decode report failed: " + error;
    }
}

void finalize_decode_report(AppState &state) {
    std::string error;
    if (!state.reporting.decode_report.finalize(state.session, error)) {
        state.ui.status = "Decode report failed: " + error;
    }
}

} // namespace airspy_tv::gui
