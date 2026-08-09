#include "receiver_controller.hpp"

#include <utility>

namespace airspy_tv {
namespace {

void append_report_error(std::string &message, const bool report_ok,
                         const std::string &report_error) {
    if (!report_ok) {
        message += "; report unavailable: " + report_error;
    }
}

} // namespace

ReceiverController::ReceiverController(
    ReceiverSession &session, DecodeRunReporter &reporter,
    SourceSettings &settings, dvbt::ReceiverParameters &dvbt_parameters)
    : session_(session), reporter_(reporter), settings_(settings),
      dvbt_parameters_(dvbt_parameters) {}

ReceiverCommandResult ReceiverController::start_report(std::string message) {
    std::string report_error;
    if (!reporter_.start_source(session_, settings_, dvbt_parameters_, "",
                                report_error)) {
        message += "; report unavailable: " + report_error;
    }
    return {.success = true, .message = std::move(message)};
}

ReceiverCommandResult
ReceiverController::open_device(const DeviceDescriptor &descriptor) {
    reporter_.prepare(session_);
    std::string error;
    if (!session_.open_device_and_start(descriptor, settings_, error)) {
        reporter_.cancel_start(session_);
        return {.message = std::move(error)};
    }
    return start_report("Receiving from " + descriptor.display_name);
}

ReceiverCommandResult
ReceiverController::open_iq_file(const std::filesystem::path &path) {
    reporter_.prepare(session_);
    std::string error;
    if (!session_.open_iq_file_and_start(path, settings_, error)) {
        reporter_.cancel_start(session_);
        return {.message = std::move(error)};
    }
    return start_report("Playing I/Q from " + path.filename().string());
}

ReceiverCommandResult ReceiverController::restart_stream(std::string message) {
    std::string report_error;
    const bool report_ok = reporter_.finish_source(session_, report_error);
    reporter_.prepare(session_);
    std::string error;
    if (!session_.start_stream(settings_, error)) {
        reporter_.cancel_start(session_);
        append_report_error(error, report_ok, report_error);
        return {.message = std::move(error)};
    }
    auto result = start_report(std::move(message));
    append_report_error(result.message, report_ok, report_error);
    return result;
}

ReceiverCommandResult
ReceiverController::retune(const std::uint64_t frequency_hz) {
    if (!session_.is_open()) {
        settings_.center_frequency_hz = frequency_hz;
        return {.success = true, .message = "Center frequency selected"};
    }

    std::string report_error;
    const bool report_ok = reporter_.finish_source(session_, report_error);
    reporter_.prepare(session_);
    std::string error;
    if (session_.retune(frequency_hz, error)) {
        settings_.center_frequency_hz = frequency_hz;
        auto result = start_report("Center frequency applied");
        append_report_error(result.message, report_ok, report_error);
        return result;
    }
    if (session_.is_streaming()) {
        auto result = start_report(std::move(error));
        result.success = false;
        append_report_error(result.message, report_ok, report_error);
        return result;
    }
    reporter_.cancel_start(session_);
    append_report_error(error, report_ok, report_error);
    return {.message = std::move(error)};
}

ReceiverCommandResult ReceiverController::close() {
    std::string report_error;
    const bool report_ok = reporter_.finish_source(session_, report_error);
    session_.close();
    return {.success = true,
            .message = report_ok ? "Source closed"
                                 : "Source closed; report unavailable: " +
                                       report_error};
}

ReceiverCommandResult
ReceiverController::start_iq_recording(const std::filesystem::path &path) {
    std::string error;
    return session_.start_recording(path, settings_, error)
               ? ReceiverCommandResult{.success = true,
                                       .message = "Recording raw I/Q"}
               : ReceiverCommandResult{.message = std::move(error)};
}

ReceiverCommandResult ReceiverController::stop_iq_recording() {
    session_.stop_recording();
    return {.success = true,
            .message = "Recording stopped; JSON sidecar written"};
}

ReceiverCommandResult
ReceiverController::start_ts_recording(const std::filesystem::path &path) {
    std::string error;
    return session_.start_ts_recording(path, error)
               ? ReceiverCommandResult{.success = true,
                                       .message = "Recording decoded MPEG-TS"}
               : ReceiverCommandResult{.message = std::move(error)};
}

ReceiverCommandResult ReceiverController::stop_ts_recording() {
    session_.stop_ts_recording();
    return {.success = true, .message = "MPEG-TS recording stopped"};
}

ReceiverCommandResult
ReceiverController::start_rtp(const RtpUdpEndpoint &endpoint) {
    std::string error;
    return session_.start_rtp_streaming(endpoint, error)
               ? ReceiverCommandResult{.success = true,
                                       .message =
                                           "Streaming MPEG-TS over RTP/UDP "
                                           "to " +
                                           format_rtp_udp_endpoint(endpoint)}
               : ReceiverCommandResult{.message = std::move(error)};
}

ReceiverCommandResult ReceiverController::stop_rtp() {
    session_.stop_rtp_streaming();
    return {.success = true, .message = "RTP/UDP streaming stopped"};
}

} // namespace airspy_tv
