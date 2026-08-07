#include "offline_decode.hpp"

#include "airspy_tv/debug.hpp"
#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/iq_file.hpp"
#include "airspy_tv/sample_timeline.hpp"
#include "decode_report.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <format>
#include <iomanip>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace {

using airspy_tv::DecodeReport;
using airspy_tv::DecodeReportConfig;
using airspy_tv::dvbt::StreamDecoder;
using airspy_tv::dvbt::StreamDecoderStats;

[[nodiscard]] int percent(const std::uint64_t used,
                          const std::uint64_t capacity) {
    if (capacity == 0) {
        return 0;
    }
    return static_cast<int>(std::clamp(
        100.0 * static_cast<double>(used) / static_cast<double>(capacity),
        0.0, 100.0));
}

void print_progress(const StreamDecoderStats &stats,
                    const std::uint64_t submitted_samples,
                    const std::uint32_t sample_rate_hz,
                    const double wall_seconds) {
    const double input_seconds =
        sample_rate_hz == 0
            ? 0.0
            : static_cast<double>(submitted_samples) /
                  static_cast<double>(sample_rate_hz);
    const double speed = wall_seconds > 0.0 ? input_seconds / wall_seconds : 0.0;
    const int iq = percent(stats.queued_input_samples,
                           stats.input_queue_capacity_samples);
    const int demod = static_cast<int>(
        std::clamp(100.0F * stats.demod_busy_fraction, 0.0F, 100.0F));
    const int fec = percent(stats.queued_symbols, stats.symbol_queue_capacity);
    std::cerr << std::fixed << std::setprecision(1)
              << "wall=" << wall_seconds << "s input=" << input_seconds
              << "s speed=" << speed << "x MER=";
    if (stats.ofdm_locked) {
        std::cerr << stats.mer_db << "dB";
    } else {
        std::cerr << "--";
    }
    std::cerr << " OFDM=" << (stats.ofdm_locked ? "lock" : "search")
              << " TPS=" << (stats.tps_locked ? "lock" : "search")
              << " TS=" << std::setprecision(1)
              << static_cast<double>(stats.transport_bytes) /
                     (1024.0 * 1024.0)
              << "MiB TEI=" << stats.cumulative_transport.tei_packets
              << std::format(" IQ={:3d}% Demod={:3d}% FEC={:3d}%", iq,
                             demod, fec)
              << '\n';
}

} // namespace

int offline_decode_cli(
    const std::filesystem::path &source,
    const std::filesystem::path &destination,
    const std::uint32_t raw_sample_rate_hz,
    const airspy_tv::dvbt::ReceiverParameters &parameters,
    const std::optional<std::filesystem::path> &report_directory) {
    const bool stdin_source = source == std::filesystem::path("-");
    const bool stdout_destination = destination == std::filesystem::path("-");
    airspy_tv::IqFileInfo info;
    std::string error;
    if (stdin_source && raw_sample_rate_hz == 0) {
        std::cerr << "stdin I/Q input requires a positive sample rate\n";
        return 1;
    }
    if (stdin_source) {
        info = {.data_path = {},
                .source = "stdin raw little-endian interleaved CS16",
                .sample_rate_hz = raw_sample_rate_hz,
                .center_frequency_hz = 0,
                .file_size_bytes = 0};
    } else if (!airspy_tv::resolve_iq_file(source, raw_sample_rate_hz, 0, info,
                                           error)) {
        std::cerr << error << '\n';
        return 1;
    }

    if (!stdin_source && !stdout_destination) {
        try {
            const auto output_path =
                std::filesystem::absolute(destination).lexically_normal();
            if (std::filesystem::absolute(source).lexically_normal() ==
                    output_path ||
                std::filesystem::absolute(info.data_path).lexically_normal() ==
                    output_path) {
                std::cerr << "Output MPEG-TS path must differ from the I/Q "
                             "source and data paths\n";
                return 1;
            }
        } catch (const std::filesystem::filesystem_error &exception) {
            std::cerr << "Unable to resolve input/output paths: "
                      << exception.what() << '\n';
            return 1;
        }
    }

    const auto started_at = std::chrono::steady_clock::now();
    std::unique_ptr<DecodeReport> report;
    if (report_directory.has_value()) {
        try {
            report = std::make_unique<DecodeReport>(
                DecodeReportConfig{
                    .directory = *report_directory,
                    .source = stdin_source ? "stdin" : source.string(),
                    .destination = stdout_destination ? "stdout"
                                                      : destination.string(),
                    .sample_rate_hz = info.sample_rate_hz,
                    .decoder = parameters,
                });
        } catch (const std::exception &exception) {
            std::cerr << "Unable to initialize performance report: "
                      << exception.what() << '\n';
            return 1;
        }
    }

    std::unique_ptr<std::ifstream> input_file;
    std::istream *input = &std::cin;
    if (!stdin_source) {
        input_file = std::make_unique<std::ifstream>(info.data_path,
                                                     std::ios::binary);
        input = input_file.get();
    }
    std::unique_ptr<std::ofstream> output_file;
    std::ostream *output = &std::cout;
    if (!stdout_destination) {
        output_file = std::make_unique<std::ofstream>(
            destination, std::ios::binary | std::ios::trunc);
        output = output_file.get();
    }

    if (!*input || !*output) {
        error = "Unable to open I/Q input or MPEG-TS output";
        std::cerr << error << '\n';
        if (report) {
            try {
                report->finalize("failed", 1, error, {}, 0, 0.0);
            } catch (const std::exception &exception) {
                std::cerr << "Unable to finalize performance report: "
                          << exception.what() << '\n';
            }
        }
        return 1;
    }

    StreamDecoder decoder;
    decoder.set_parameters(parameters);
    const bool detailed = report != nullptr || airspy_tv::is_debug_enabled();
    decoder.set_telemetry_enabled(detailed, started_at);
    std::atomic_bool output_failed{};
    decoder.set_transport_callback(
        [output, &output_failed](const std::span<const std::uint8_t> ts) {
            output->write(reinterpret_cast<const char *>(ts.data()),
                          static_cast<std::streamsize>(ts.size()));
            if (!*output) {
                output_failed.store(true, std::memory_order_relaxed);
            }
        });

    airspy_tv::InputSampleTimeline input_timeline;
    input_timeline.begin_stream(info.sample_rate_hz);
    const std::size_t scalar_samples =
        StreamDecoder::chunk_samples_for(info.sample_rate_hz) * 2;
    std::vector<std::int16_t> block(scalar_samples);
    std::uint64_t submitted_samples = 0;
    bool input_failed = false;
    bool report_failed = false;
    std::string report_error;
    auto last_periodic = started_at;

    const auto drain_records = [&] {
        if (!detailed || report_failed) {
            return;
        }
        auto records = decoder.drain_telemetry();
        try {
            if (report) {
                report->consume(records);
            }
            if (airspy_tv::is_debug_enabled()) {
                for (const auto &record : records) {
                    airspy_tv::format_debug_telemetry(std::cerr, record);
                }
            }
        } catch (const std::exception &exception) {
            report_failed = true;
            report_error = exception.what();
        }
    };

    const auto periodic = [&](const bool final) {
        const auto now = std::chrono::steady_clock::now();
        if (!final && now - last_periodic < std::chrono::seconds(1)) {
            return;
        }
        last_periodic = now;
        const double elapsed =
            std::chrono::duration<double>(now - started_at).count();
        const auto stats = decoder.stats();
        print_progress(stats, submitted_samples, info.sample_rate_hz, elapsed);
        if (report && !report_failed) {
            try {
                report->write_pipeline(stats, submitted_samples, elapsed);
                report->flush();
            } catch (const std::exception &exception) {
                report_failed = true;
                report_error = exception.what();
            }
        }
    };

    while (*input && !output_failed.load(std::memory_order_relaxed) &&
           !report_failed) {
        input->read(reinterpret_cast<char *>(block.data()),
                    static_cast<std::streamsize>(block.size() *
                                                 sizeof(block.front())));
        const std::streamsize bytes_read = input->gcount();
        if (bytes_read <= 0) {
            break;
        }
        if ((bytes_read % static_cast<std::streamsize>(
                              sizeof(std::int16_t) * 2)) != 0) {
            input_failed = true;
            error = "I/Q input ended with an incomplete CS16 sample";
            break;
        }
        const std::size_t scalar_count =
            static_cast<std::size_t>(bytes_read) / sizeof(block.front());
        const std::size_t complex_count = scalar_count / 2;
        submitted_samples += complex_count;
        decoder.submit_blocking(
            std::span(block).first(scalar_count), info.sample_rate_hz,
            parameters.channel_bandwidth_hz,
            input_timeline.stamp(complex_count, info.sample_rate_hz));
        drain_records();
        if (decoder.stats().failed) {
            break;
        }
        periodic(false);
    }
    if (input->bad()) {
        input_failed = true;
        error = "Failed while reading I/Q input";
    }

    decoder.flush();
    drain_records();
    output->flush();
    periodic(true);

    const auto stats = decoder.stats();
    const double wall_seconds = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() -
                                    started_at)
                                    .count();
    int exit_code = 0;
    std::string status = "completed";
    if (report_failed) {
        error = "Performance report failed: " + report_error;
        exit_code = 1;
        status = "failed";
    } else if (input_failed) {
        exit_code = 1;
        status = "failed";
    } else if (output_failed.load(std::memory_order_relaxed) || !*output) {
        error = "Failed while writing MPEG-TS output";
        exit_code = 1;
        status = "failed";
    } else if (stats.failed) {
        error = stats.error;
        exit_code = 1;
        status = "failed";
    } else if (stats.dropped_blocks != 0) {
        error = std::format(
            "Internal error: decoder-paced input dropped {} block(s)",
            stats.dropped_blocks);
        exit_code = 1;
        status = "failed";
    } else if (stats.transport_bytes == 0) {
        exit_code = 2;
        status = "no_transport";
    }

    if (!error.empty()) {
        std::cerr << error << '\n';
    }
    if (report) {
        try {
            report->finalize(status, exit_code, error, stats,
                             submitted_samples, wall_seconds);
        } catch (const std::exception &exception) {
            std::cerr << "Unable to finalize performance report: "
                      << exception.what() << '\n';
            return 1;
        }
    }
    return exit_code;
}
