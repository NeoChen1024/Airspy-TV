#include "main_cli.hpp"

#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <format>
#include <iostream>
#include <ranges>

namespace airspy_tv::cli {

namespace {

constexpr std::array<option, 28> cli_option_table{{
    {"help", no_argument, nullptr, 'h'},
    {"enumerate", no_argument, nullptr, opt_enumerate},
    {"inspect-iq", required_argument, nullptr, opt_inspect_iq},
    {"decode-iq", required_argument, nullptr, opt_decode_iq},
    {"decode-live", no_argument, nullptr, opt_decode_live},
    {"device", required_argument, nullptr, opt_device},
    {"iq-input", required_argument, nullptr, opt_iq_input},
    {"ts-output", required_argument, nullptr, opt_ts_output},
    {"rtp-output", required_argument, nullptr, opt_rtp_output},
    {"report-dir", required_argument, nullptr, opt_report_dir},
    {"sample-rate", required_argument, nullptr, opt_sample_rate},
    {"mode", required_argument, nullptr, opt_mode},
    {"dvbt-mode", required_argument, nullptr, opt_dvbt_mode},
    {"dvbt-channel-bandwidth", required_argument, nullptr, opt_dvbt_bandwidth},
    {"dvbt-guard", required_argument, nullptr, opt_dvbt_guard},
    {"dvbt-modulation", required_argument, nullptr, opt_dvbt_modulation},
    {"dvbt-code-rate", required_argument, nullptr, opt_dvbt_code_rate},
    {"decoder-threads", required_argument, nullptr, opt_decoder_threads},
    {"offline-queue-multiplier", required_argument, nullptr,
     opt_offline_queue_multiplier},
    {"record-first", required_argument, nullptr, opt_record_first},
    {"duration", required_argument, nullptr, opt_duration},
    {"frequency", required_argument, nullptr, opt_frequency},
    {"ppm", required_argument, nullptr, opt_ppm},
    {"gain", required_argument, nullptr, opt_gain},
    {"gain-mode", required_argument, nullptr, opt_gain_mode},
    {"bias-tee", no_argument, nullptr, opt_bias_tee},
    {"debug", no_argument, nullptr, 'd'},
    {nullptr, 0, nullptr, 0},
}};

} // namespace

const option *cli_options() noexcept { return cli_option_table.data(); }

[[noreturn]] void cli_usage_error(const std::string_view message) {
    std::cerr << "airspy-tv: " << message << "\n"
              << "Try 'airspy-tv --help' for usage.\n";
    std::exit(2);
}

void print_cli_usage() {
    std::cout
        << "Usage: airspy-tv [options]\n"
        << "\n"
        << "General:\n"
        << "  -h, --help                     Show this help\n"
        << "  -d, --debug                    Verbose decoder diagnostics\n"
        << "      --report-dir DIR           Write a machine-readable decode "
           "report\n"
        << "\n"
        << "Device information:\n"
        << "      --enumerate                List SDR devices and exit\n"
        << "      --inspect-iq PATH          Inspect a raw I/Q capture "
           "(needs --sample-rate)\n"
        << "\n"
        << "I/Q to MPEG-TS decoding (offline):\n"
        << "      --decode-iq PATH|-        Raw I/Q input file (.cs16 / "
           "raw INT16_IQ; a .json\n"
        << "                                 sidecar is honoured when "
           "present)\n"
        << "      --ts-output PATH|-        MPEG-TS output file (required "
           "with --decode-iq)\n"
        << "      --sample-rate HZ          Raw I/Q sample rate (default "
           "10000000)\n"
        << "      --decoder-threads N       Worker budget 0..256 "
           "(default 0 = auto)\n"
        << "      --offline-queue-multiplier N\n"
           "                                 Scale demod/FEC queues 1..16 "
           "(default 4)\n"
        << "\n"
        << "Live MPEG-TS decoding (SDR or real-time I/Q replay):\n"
        << "      --decode-live             Decode continuously until "
           "SIGINT/SIGTERM\n"
        << "      --device ID               Exact device ID from --enumerate "
           "(default: native Airspy or first)\n"
        << "      --iq-input PATH|-         Replay an I/Q file or stdin in "
           "real time "
           "instead of using an SDR\n"
        << "      --ts-output PATH|-        MPEG-TS file or stdout\n"
        << "      --rtp-output HOST:PORT    RTP/UDP output; IPv6 uses "
           "[ADDRESS]:PORT\n"
        << "                                 (at least one live output is "
           "required)\n"
        << "\n"
        << "Broadcast standard (default dvbt):\n"
        << "      --mode STANDARD           dvbt (DVB-C/DVB-T2/DTMB/ATSC "
           "reserved)\n"
        << "\n"
        << "DVB-T transmission parameters (default: auto from TPS):\n"
        << "      --dvbt-mode 2k|8k                Force transmission mode\n"
        << "      --dvbt-channel-bandwidth 5M|6M|7M|8M\n"
        << "      --dvbt-guard 1/32|1/16|1/8|1/4   Force guard interval\n"
        << "      --dvbt-modulation qpsk|qam16|qam64\n"
        << "      --dvbt-code-rate 1/2|2/3|3/4|5/6|7/8\n"
        << "\n"
        << "I/Q recording (needs an SDR device):\n"
        << "      --record-first PATH       Record I/Q to PATH and exit\n"
        << "      --duration MS             Recording duration (default "
           "1000)\n"
        << "      --frequency HZ            Center frequency (default "
           "545000000)\n"
        << "      --sample-rate HZ          Sample rate (default 10000000)\n"
        << "      --ppm DOUBLE              Frequency-correction ppm\n"
        << "      --gain N                  Airspy profile gain (default 10)\n"
        << "      --gain-mode sensitivity|linearity\n"
        << "      --bias-tee                Enable the Bias-T supply\n";
}

std::uint64_t parse_u64(const std::string_view text,
                        const std::string_view what) {
    std::uint64_t value = 0;
    const auto parsed = std::from_chars(text.begin(), text.end(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.end()) {
        cli_usage_error(std::format("invalid {}: '{}'", what, text));
    }
    return value;
}

double parse_double(const std::string_view text, const std::string_view what) {
    double value = 0.0;
    const auto parsed = std::from_chars(text.begin(), text.end(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.end() ||
        !std::isfinite(value)) {
        cli_usage_error(std::format("invalid {}: '{}'", what, text));
    }
    return value;
}

std::uint32_t parse_bandwidth(const std::string_view text) {
    constexpr std::array<std::pair<std::string_view, std::uint32_t>, 4> named{
        {{"5M", 5'000'000U},
         {"6M", 6'000'000U},
         {"7M", 7'000'000U},
         {"8M", 8'000'000U}}};
    for (const auto &[name, value] : named) {
        if (text.size() == name.size() &&
            std::ranges::equal(text, name, [](const char a, const char b) {
                return std::tolower(static_cast<unsigned char>(a)) ==
                       std::tolower(static_cast<unsigned char>(b));
            })) {
            return value;
        }
    }
    const std::uint64_t hz = parse_u64(text, "channel bandwidth");
    if (hz != 5'000'000U && hz != 6'000'000U && hz != 7'000'000U &&
        hz != 8'000'000U) {
        cli_usage_error(
            std::format("invalid channel bandwidth: '{}' (expected 5M/6M/7M/"
                        "8M or an explicit Hz value)",
                        text));
    }
    return static_cast<std::uint32_t>(hz);
}

dvbt::TransmissionMode parse_dvbt_mode(const std::string_view text) {
    if (text == "2k" || text == "2K") {
        return dvbt::TransmissionMode::k2;
    }
    if (text == "8k" || text == "8K") {
        return dvbt::TransmissionMode::k8;
    }
    cli_usage_error(
        std::format("invalid --dvbt-mode: '{}' (expected 2k|8k)", text));
}

dvbt::GuardInterval parse_guard_interval(const std::string_view text) {
    if (text == "1/32") {
        return dvbt::GuardInterval::gi_1_32;
    }
    if (text == "1/16") {
        return dvbt::GuardInterval::gi_1_16;
    }
    if (text == "1/8") {
        return dvbt::GuardInterval::gi_1_8;
    }
    if (text == "1/4") {
        return dvbt::GuardInterval::gi_1_4;
    }
    cli_usage_error(std::format(
        "invalid --dvbt-guard: '{}' (expected 1/32|1/16|1/8|1/4)", text));
}

dvbt::Constellation parse_modulation(const std::string_view text) {
    if (text == "qpsk") {
        return dvbt::Constellation::qpsk;
    }
    if (text == "qam16") {
        return dvbt::Constellation::qam16;
    }
    if (text == "qam64") {
        return dvbt::Constellation::qam64;
    }
    cli_usage_error(std::format(
        "invalid --dvbt-modulation: '{}' (expected qpsk|qam16|qam64)", text));
}

dvbt::CodeRate parse_code_rate(const std::string_view text) {
    if (text == "1/2") {
        return dvbt::CodeRate::rate_1_2;
    }
    if (text == "2/3") {
        return dvbt::CodeRate::rate_2_3;
    }
    if (text == "3/4") {
        return dvbt::CodeRate::rate_3_4;
    }
    if (text == "5/6") {
        return dvbt::CodeRate::rate_5_6;
    }
    if (text == "7/8") {
        return dvbt::CodeRate::rate_7_8;
    }
    cli_usage_error(std::format(
        "invalid --dvbt-code-rate: '{}' (expected 1/2|2/3|3/4|5/6|7/8)", text));
}

ReceiveStandard parse_standard(const std::string_view text) {
    if (text == "dvbt") {
        return ReceiveStandard::DvbT;
    }
    cli_usage_error(std::format(
        "invalid --mode: '{}' (only dvbt is implemented today)", text));
}

AirspyGainMode parse_gain_mode(const std::string_view text) {
    if (text == "sensitivity") {
        return AirspyGainMode::Sensitivity;
    }
    if (text == "linearity") {
        return AirspyGainMode::Linearity;
    }
    cli_usage_error(std::format("invalid --gain-mode: '{}' (expected "
                                "sensitivity|linearity)",
                                text));
}

} // namespace airspy_tv::cli
