#include "airspy_tv/sdr.hpp"

#include "airspy_tv/debug.hpp"
#include "airspy_tv/dvbt/signal_analyzer.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "gui/app.hpp"
#include "live_decode.hpp"
#include "main_cli.hpp"
#include "main_commands.hpp"
#include "offline_decode.hpp"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <format>
#include <getopt.h>
#include <optional>
#include <string>

namespace {

using airspy_tv::AirspyGainMode;
using airspy_tv::SourceSettings;
using airspy_tv::dvbt::ReceiverParameters;
using namespace airspy_tv::cli;

} // namespace

int main(const int argc, char **argv) {
    std::optional<std::filesystem::path> inspect_iq_path;
    std::optional<std::filesystem::path> decode_iq_path;
    bool decode_live = false;
    std::optional<std::string> device_id;
    std::optional<std::filesystem::path> live_iq_input;
    std::optional<std::filesystem::path> ts_output_path;
    std::optional<std::string> rtp_output_text;
    std::optional<std::filesystem::path> report_directory;
    std::optional<std::filesystem::path> record_path;
    std::uint32_t sample_rate_hz = 10'000'000;
    bool sample_rate_supplied = false;
    std::uint64_t center_frequency_hz = 545'000'000;
    double frequency_correction_ppm = 0.0;
    int duration_ms = 1000;
    int airspy_gain = 10;
    AirspyGainMode airspy_gain_mode = AirspyGainMode::Sensitivity;
    bool bias_tee = false;
    ReceiverParameters dvbt_parameters;
    airspy_tv::ReceiveStandard standard = airspy_tv::ReceiveStandard::DvbT;

    int opt = 0;
    while ((opt = getopt_long(argc, argv, "hd", cli_options(), nullptr)) !=
           -1) {
        switch (opt) {
        case 'h':
            print_cli_usage();
            return 0;
        case 'd':
            airspy_tv::debug_enabled.store(true, std::memory_order_relaxed);
            break;
        case opt_enumerate:
            return enumerate_cli();
        case opt_inspect_iq:
            inspect_iq_path = optarg;
            break;
        case opt_decode_iq:
            decode_iq_path = optarg;
            break;
        case opt_decode_live:
            decode_live = true;
            break;
        case opt_device:
            device_id = optarg;
            break;
        case opt_iq_input:
            live_iq_input = optarg;
            break;
        case opt_ts_output:
            ts_output_path = optarg;
            break;
        case opt_rtp_output:
            rtp_output_text = optarg;
            break;
        case opt_report_dir:
            report_directory = optarg;
            break;
        case opt_sample_rate:
            sample_rate_hz =
                static_cast<std::uint32_t>(parse_u64(optarg, "sample rate"));
            if (sample_rate_hz == 0) {
                cli_usage_error("sample rate must be non-zero");
            }
            sample_rate_supplied = true;
            break;
        case opt_mode:
            standard = parse_standard(optarg);
            break;
        case opt_dvbt_mode:
            dvbt_parameters.mode = parse_dvbt_mode(optarg);
            break;
        case opt_dvbt_bandwidth:
            dvbt_parameters.channel_bandwidth_hz = parse_bandwidth(optarg);
            break;
        case opt_dvbt_guard:
            dvbt_parameters.guard_interval = parse_guard_interval(optarg);
            break;
        case opt_dvbt_modulation:
            dvbt_parameters.constellation = parse_modulation(optarg);
            break;
        case opt_dvbt_code_rate:
            dvbt_parameters.code_rate = parse_code_rate(optarg);
            break;
        case opt_decoder_threads:
            dvbt_parameters.worker_threads = static_cast<std::size_t>(
                parse_u64(optarg, "decoder thread count"));
            if (dvbt_parameters.worker_threads > 256) {
                cli_usage_error("decoder thread count must be 0..256");
            }
            break;
        case opt_record_first:
            record_path = optarg;
            break;
        case opt_duration:
            duration_ms =
                static_cast<int>(parse_u64(optarg, "recording duration"));
            if (duration_ms <= 0) {
                cli_usage_error("recording duration must be positive");
            }
            break;
        case opt_frequency:
            center_frequency_hz = parse_u64(optarg, "center frequency");
            break;
        case opt_ppm:
            frequency_correction_ppm =
                parse_double(optarg, "frequency correction");
            break;
        case opt_gain:
            airspy_gain = static_cast<int>(parse_u64(optarg, "gain"));
            break;
        case opt_gain_mode:
            airspy_gain_mode = parse_gain_mode(optarg);
            break;
        case opt_bias_tee:
            bias_tee = true;
            break;
        default:
            cli_usage_error("unrecognized option");
        }
    }
    if (optind != argc) {
        cli_usage_error(
            std::format("unexpected positional argument: '{}'", argv[optind]));
    }

    const int command_count = (decode_iq_path.has_value() ? 1 : 0) +
                              (decode_live ? 1 : 0) +
                              (record_path.has_value() ? 1 : 0) +
                              (inspect_iq_path.has_value() ? 1 : 0);
    if (command_count > 1) {
        cli_usage_error("--decode-iq, --decode-live, --record-first and "
                        "--inspect-iq are mutually exclusive");
    }
    if (report_directory.has_value() &&
        (inspect_iq_path.has_value() || record_path.has_value())) {
        cli_usage_error(
            "--report-dir is valid with --decode-iq, --decode-live or the GUI "
            "receiver");
    }
    if (device_id.has_value() && !decode_live) {
        cli_usage_error("--device is valid only with --decode-live");
    }
    if (live_iq_input.has_value() && !decode_live) {
        cli_usage_error("--iq-input is valid only with --decode-live");
    }
    if (live_iq_input.has_value() && device_id.has_value()) {
        cli_usage_error("--device and --iq-input are mutually exclusive");
    }
    if (ts_output_path.has_value() && !decode_live &&
        !decode_iq_path.has_value()) {
        cli_usage_error("--ts-output requires --decode-iq or --decode-live");
    }
    if (rtp_output_text.has_value() && !decode_live) {
        cli_usage_error("--rtp-output is valid only with --decode-live");
    }

    if (inspect_iq_path.has_value()) {
        return inspect_iq_cli(*inspect_iq_path, sample_rate_hz,
                              center_frequency_hz);
    }
    if (decode_iq_path.has_value()) {
        if (!ts_output_path.has_value()) {
            cli_usage_error("--decode-iq requires --ts-output");
        }
        if (*decode_iq_path == std::filesystem::path("-") &&
            !sample_rate_supplied) {
            cli_usage_error("--decode-iq - requires an explicit --sample-rate");
        }
        return offline_decode_cli(*decode_iq_path, *ts_output_path,
                                  sample_rate_hz, dvbt_parameters,
                                  report_directory);
    }
    if (decode_live) {
        if (!ts_output_path.has_value() && !rtp_output_text.has_value()) {
            cli_usage_error(
                "--decode-live requires --ts-output or --rtp-output");
        }
        if (live_iq_input == std::filesystem::path("-") &&
            !sample_rate_supplied) {
            cli_usage_error("--decode-live --iq-input - requires an explicit "
                            "--sample-rate");
        }
        std::optional<airspy_tv::RtpUdpEndpoint> rtp_output;
        if (rtp_output_text.has_value()) {
            airspy_tv::RtpUdpEndpoint endpoint;
            std::string endpoint_error;
            if (!airspy_tv::parse_rtp_udp_endpoint(*rtp_output_text, endpoint,
                                                   endpoint_error)) {
                cli_usage_error(endpoint_error);
            }
            rtp_output = std::move(endpoint);
        }
        SourceSettings settings;
        settings.center_frequency_hz = center_frequency_hz;
        settings.frequency_correction_ppm = frequency_correction_ppm;
        settings.sample_rate_hz = sample_rate_hz;
        settings.airspy_gain_mode = airspy_gain_mode;
        settings.airspy_gain = airspy_gain;
        settings.bias_tee = bias_tee;
        return live_decode_cli({.standard = standard,
                                .device_id = device_id,
                                .iq_input = live_iq_input,
                                .destination = ts_output_path,
                                .rtp_output = std::move(rtp_output),
                                .report_directory = report_directory,
                                .source = settings,
                                .dvbt = dvbt_parameters});
    }
    if (record_path.has_value()) {
        SourceSettings settings;
        settings.center_frequency_hz = center_frequency_hz;
        settings.frequency_correction_ppm = frequency_correction_ppm;
        settings.sample_rate_hz = sample_rate_hz;
        settings.airspy_gain_mode = airspy_gain_mode;
        settings.airspy_gain = airspy_gain;
        settings.bias_tee = bias_tee;
        return record_first_cli(*record_path, duration_ms, settings);
    }

    return airspy_tv::gui::run_gui(report_directory);
}
