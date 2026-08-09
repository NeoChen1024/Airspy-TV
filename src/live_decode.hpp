#pragma once

#include "airspy_tv/demodulator.hpp"
#include "airspy_tv/dvbt/receiver_parameters.hpp"
#include "airspy_tv/rtp_udp_output.hpp"
#include "airspy_tv/sdr.hpp"

#include <filesystem>
#include <optional>
#include <string>

struct LiveDecodeConfig {
    airspy_tv::ReceiveStandard standard{airspy_tv::ReceiveStandard::DvbT};
    std::optional<std::string> device_id;
    std::optional<std::filesystem::path> iq_input;
    std::optional<std::filesystem::path> destination;
    std::optional<airspy_tv::RtpUdpEndpoint> rtp_output;
    std::optional<std::filesystem::path> report_directory;
    airspy_tv::SourceSettings source;
    airspy_tv::dvbt::ReceiverParameters dvbt;
};

int live_decode_cli(LiveDecodeConfig config);
