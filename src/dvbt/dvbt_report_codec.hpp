#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"

#include <nlohmann/json.hpp>

#include <ostream>

namespace airspy_tv {

enum class DvbTReportStream { frontend, demod, fec, event };

struct DvbTJsonRecord {
    DvbTReportStream stream;
    nlohmann::json value;
};

[[nodiscard]] DvbTJsonRecord
encode_dvbt_telemetry(const dvbt::TelemetryRecord &record);
[[nodiscard]] nlohmann::json
encode_dvbt_decoder_config(const dvbt::ReceiverParameters &parameters);
[[nodiscard]] const char *dvbt_worker_state_name(dvbt::WorkerState state);
void format_debug_telemetry(std::ostream &stream,
                            const dvbt::TelemetryRecord &record);

} // namespace airspy_tv
