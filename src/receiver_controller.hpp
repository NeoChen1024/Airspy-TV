#pragma once

#include "decode_run_reporter.hpp"
#include "receiver_session.hpp"

#include <filesystem>
#include <string>

namespace airspy_tv {

struct ReceiverCommandResult {
    bool success{};
    std::string message;
};

// Application-agnostic orchestration for source/report lifecycle transitions.
// UI and CLI adapters remain responsible for presentation and event policy.
class ReceiverController {
  public:
    ReceiverController(ReceiverSession &session, DecodeRunReporter &reporter,
                       SourceSettings &settings,
                       dvbt::ReceiverParameters &dvbt_parameters);

    [[nodiscard]] ReceiverCommandResult
    open_device(const DeviceDescriptor &descriptor);
    [[nodiscard]] ReceiverCommandResult
    open_iq_file(const std::filesystem::path &path);
    [[nodiscard]] ReceiverCommandResult restart_stream(std::string message);
    [[nodiscard]] ReceiverCommandResult retune(std::uint64_t frequency_hz);
    [[nodiscard]] ReceiverCommandResult close();
    [[nodiscard]] ReceiverCommandResult
    start_iq_recording(const std::filesystem::path &path);
    [[nodiscard]] ReceiverCommandResult stop_iq_recording();
    [[nodiscard]] ReceiverCommandResult
    start_ts_recording(const std::filesystem::path &path);
    [[nodiscard]] ReceiverCommandResult stop_ts_recording();
    [[nodiscard]] ReceiverCommandResult
    start_rtp(const RtpUdpEndpoint &endpoint);
    [[nodiscard]] ReceiverCommandResult stop_rtp();

  private:
    [[nodiscard]] ReceiverCommandResult start_report(std::string message);

    ReceiverSession &session_;
    DecodeRunReporter &reporter_;
    SourceSettings &settings_;
    dvbt::ReceiverParameters &dvbt_parameters_;
};

} // namespace airspy_tv
