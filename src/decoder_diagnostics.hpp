#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"

namespace airspy_tv::gui {

[[nodiscard]] const char *worker_state_name(dvbt::WorkerState state);
void dump_decoder_diagnostics(const dvbt::StreamDecoderStats &stats);

} // namespace airspy_tv::gui
