#include "offline_decode.hpp"

#include "airspy_tv/iq_file.hpp"
#include "airspy_tv/transport_output.hpp"
#include "decode_progress.hpp"
#include "decode_run_reporter.hpp"
#include "receiver_session.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <span>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

[[nodiscard]] bool output_path_is_safe(const std::filesystem::path &source,
                                       const airspy_tv::IqFileInfo &source_info,
                                       const std::filesystem::path &destination,
                                       std::string &error) {
    try {
        const auto output_path =
            std::filesystem::absolute(destination).lexically_normal();
        if (std::filesystem::absolute(source).lexically_normal() ==
                output_path ||
            std::filesystem::absolute(source_info.data_path)
                    .lexically_normal() == output_path) {
            error = "Output MPEG-TS path must differ from the I/Q source and "
                    "data paths";
            return false;
        }
    } catch (const std::filesystem::filesystem_error &exception) {
        error = "Unable to resolve input/output paths: " +
                std::string(exception.what());
        return false;
    }
    return true;
}

[[nodiscard]] std::uint64_t
submitted_samples(const airspy_tv::InputTimelineSnapshot &initial,
                  const airspy_tv::InputTimelineSnapshot &current) {
    return current.delivered_samples >= initial.delivered_samples
               ? current.delivered_samples - initial.delivered_samples
               : current.delivered_samples;
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
    airspy_tv::IqFileInfo source_info;
    std::string error;
    if (stdin_source && raw_sample_rate_hz == 0) {
        std::cerr << "stdin I/Q input requires a positive sample rate\n";
        return 1;
    }
    if (stdin_source) {
        source_info = {
            .data_path = {},
            .source = "stdin raw little-endian interleaved CS16",
            .sample_rate_hz = raw_sample_rate_hz,
            .center_frequency_hz = 0,
            .file_size_bytes = 0,
        };
    } else if (!airspy_tv::resolve_iq_file(source, raw_sample_rate_hz, 0,
                                           source_info, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (!stdin_source && !stdout_destination &&
        !output_path_is_safe(source, source_info, destination, error)) {
        std::cerr << error << '\n';
        return 1;
    }

    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 24U << 20U,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::block_producer,
        .criticality = airspy_tv::TransportSinkCriticality::required,
        .thread_name = "ts-offline-output",
    });
    const bool output_started =
        stdout_destination
            ? output.start_fd(STDOUT_FILENO, false, "stdout", error)
            : output.start_file(destination, error);
    if (!output_started) {
        std::cerr << error << '\n';
        return 1;
    }

    airspy_tv::SourceSettings settings;
    settings.sample_rate_hz = source_info.sample_rate_hz;
    settings.center_frequency_hz = source_info.center_frequency_hz;
    constexpr airspy_tv::IqPlaybackPolicy playback_policy{
        .pacing = airspy_tv::IqPlaybackPacing::unpaced,
        .decoder_backpressure = airspy_tv::DecoderBackpressurePolicy::block,
    };

    airspy_tv::ReceiverSession session;
    session.set_display_analysis_enabled(false);
    session.set_dvbt_parameters(parameters);
    session.set_transport_sink(
        [&output](const std::span<const std::uint8_t> ts) {
            static_cast<void>(output.submit(ts));
        });

    airspy_tv::DecodeRunReporter reporter(report_directory, "offline");
    reporter.set_transport_output_provider([&output] {
        return std::vector<airspy_tv::TransportOutputTelemetry>{
            airspy_tv::transport_output_telemetry(
                "offline-ts-output", "file-or-stream", output.stats())};
    });
    reporter.prepare(session);
    const auto initial_timeline = session.input_timeline_snapshot();
    if (!session.open_iq_file_and_start(source, settings, playback_policy,
                                        error)) {
        reporter.cancel_start(session);
        session.set_transport_sink({});
        session.close();
        std::cerr << error << '\n';
        return 1;
    }
    const std::string destination_name =
        stdout_destination ? "stdout" : destination.string();
    if (!reporter.start_source(session, settings, parameters, destination_name,
                               error)) {
        session.set_transport_sink({});
        session.close();
        std::cerr << "Decode report failed: " << error << '\n';
        return 1;
    }

    const auto started_at = std::chrono::steady_clock::now();
    auto last_progress = started_at;
    bool report_failed = false;
    while (session.is_streaming()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const std::string runtime_error = session.runtime_error();
        if (!runtime_error.empty()) {
            error = runtime_error;
            break;
        }
        const auto output_stats = output.stats();
        if (output_stats.failed) {
            error = output_stats.error;
            break;
        }
        const auto stats = session.dvbt_snapshot().decoder;
        if (stats.failed) {
            error = stats.error;
            break;
        }
        std::string report_error;
        if (!reporter.update(session, report_error, false)) {
            error = "Decode report failed: " + report_error;
            report_failed = true;
            break;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_progress >= std::chrono::seconds(1)) {
            last_progress = now;
            airspy_tv::format_decode_progress(
                std::cerr, stats,
                submitted_samples(initial_timeline,
                                  session.input_timeline_snapshot()),
                settings.sample_rate_hz,
                std::chrono::duration<double>(now - started_at).count());
        }
    }

    session.finish_stream();
    session.set_transport_sink({});
    output.stop(error.empty());

    if (error.empty()) {
        const std::string runtime_error = session.runtime_error();
        if (!runtime_error.empty()) {
            error = runtime_error;
        } else if (output.stats().failed) {
            error = output.stats().error;
        } else if (!session.input_exhausted()) {
            error = "I/Q input stream stopped";
        }
    }

    const auto stats = session.dvbt_snapshot().decoder;
    const auto final_timeline = session.input_timeline_snapshot();
    const double wall_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      started_at)
            .count();
    airspy_tv::format_decode_progress(
        std::cerr, stats, submitted_samples(initial_timeline, final_timeline),
        settings.sample_rate_hz, wall_seconds);

    int exit_code = 0;
    if (error.empty() && stats.failed) {
        error = stats.error;
    }
    if (error.empty() && stats.dropped_blocks != 0) {
        error = "Internal error: blocking decoder input dropped " +
                std::to_string(stats.dropped_blocks) + " block(s)";
    }

    const auto output_stats = output.stats();
    if (error.empty() && output_stats.dropped_blocks != 0) {
        error = "Internal error: exact MPEG-TS output dropped " +
                std::to_string(output_stats.dropped_blocks) + " block(s)";
    }
    if (report_failed || !error.empty() || output_stats.failed) {
        if (error.empty()) {
            error = "Failed while writing MPEG-TS output";
        }
        exit_code = 1;
    } else if (stats.transport_bytes == 0) {
        exit_code = 2;
    }

    std::string report_error;
    if (!reporter.finalize(session, report_error, error, exit_code)) {
        if (error.empty()) {
            error = "Decode report failed: " + report_error;
        } else {
            error += "; decode report failed: " + report_error;
        }
        exit_code = 1;
    }
    session.close();
    if (!error.empty()) {
        std::cerr << error << '\n';
    }
    return exit_code;
}
