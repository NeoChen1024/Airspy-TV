#pragma once

#include "airspy_tv/demodulator.hpp"
#include "airspy_tv/dvbt/receiver_parameters.hpp"
#include "airspy_tv/sdr.hpp"

#include <cstdint>
#include <getopt.h>
#include <string_view>

namespace airspy_tv::cli {

enum CliOption : int {
    opt_enumerate = 1000,
    opt_inspect_iq,
    opt_decode_iq,
    opt_decode_live,
    opt_device,
    opt_iq_input,
    opt_ts_output,
    opt_rtp_output,
    opt_report_dir,
    opt_sample_rate,
    opt_mode,
    opt_dvbt_mode,
    opt_dvbt_bandwidth,
    opt_dvbt_guard,
    opt_dvbt_modulation,
    opt_dvbt_code_rate,
    opt_decoder_threads,
    opt_offline_queue_multiplier,
    opt_record_first,
    opt_duration,
    opt_frequency,
    opt_ppm,
    opt_gain,
    opt_gain_mode,
    opt_bias_tee,
};

[[nodiscard]] const option *cli_options() noexcept;

[[noreturn]] void cli_usage_error(std::string_view message);
void print_cli_usage();

[[nodiscard]] std::uint64_t parse_u64(std::string_view text,
                                      std::string_view what);
[[nodiscard]] double parse_double(std::string_view text, std::string_view what);
[[nodiscard]] std::uint32_t parse_bandwidth(std::string_view text);
[[nodiscard]] dvbt::TransmissionMode parse_dvbt_mode(std::string_view text);
[[nodiscard]] dvbt::GuardInterval parse_guard_interval(std::string_view text);
[[nodiscard]] dvbt::Constellation parse_modulation(std::string_view text);
[[nodiscard]] dvbt::CodeRate parse_code_rate(std::string_view text);
[[nodiscard]] ReceiveStandard parse_standard(std::string_view text);
[[nodiscard]] AirspyGainMode parse_gain_mode(std::string_view text);

} // namespace airspy_tv::cli
