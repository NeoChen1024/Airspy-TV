#include "decode_reporting.hpp"

#include <string>

namespace airspy_tv::gui {
void prepare_decode_report(AppState &state) {
    state.decode_report.prepare(state.session);
}

void cancel_decode_report_start(AppState &state) {
    state.decode_report.cancel_start(state.session);
}

bool start_decode_report(AppState &state, std::string &error) {
    return state.decode_report.start_source(state.session, state.settings,
                                            state.dvbt.parameters,
                                            "GUI transport pipeline", error);
}

void update_decode_report(AppState &state) {
    std::string error;
    if (!state.decode_report.update(state.session, error)) {
        state.status = "Decode report failed: " + error;
    }
}

void finish_decode_report_source(AppState &state) {
    std::string error;
    if (!state.decode_report.finish_source(state.session, error)) {
        state.status = "Decode report failed: " + error;
    }
}

void finalize_decode_report(AppState &state) {
    std::string error;
    if (!state.decode_report.finalize(state.session, error)) {
        state.status = "Decode report failed: " + error;
    }
}

} // namespace airspy_tv::gui
