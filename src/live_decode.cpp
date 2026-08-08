#include "live_decode.hpp"

#include "airspy_tv/transport_output.hpp"
#include "decode_progress.hpp"
#include "decode_run_reporter.hpp"
#include "receiver_session.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <ranges>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <utility>

namespace {

volatile std::sig_atomic_t stop_live_decode{};

extern "C" void request_live_decode_stop(int /*signal*/) {
    stop_live_decode = 1;
}

class SignalHandlerScope {
  public:
    SignalHandlerScope()
        : old_interrupt_(std::signal(SIGINT, request_live_decode_stop)),
          old_terminate_(std::signal(SIGTERM, request_live_decode_stop)),
          old_pipe_(std::signal(SIGPIPE, SIG_IGN)) {
        stop_live_decode = 0;
    }

    SignalHandlerScope(const SignalHandlerScope &) = delete;
    SignalHandlerScope &operator=(const SignalHandlerScope &) = delete;
    SignalHandlerScope(SignalHandlerScope &&) = delete;
    SignalHandlerScope &operator=(SignalHandlerScope &&) = delete;

    ~SignalHandlerScope() {
        std::signal(SIGINT, old_interrupt_);
        std::signal(SIGTERM, old_terminate_);
        std::signal(SIGPIPE, old_pipe_);
    }

  private:
    using Handler = void (*)(int);
    Handler old_interrupt_;
    Handler old_terminate_;
    Handler old_pipe_;
};

[[nodiscard]] const airspy_tv::DeviceDescriptor *
select_device(const airspy_tv::EnumerationResult &enumeration,
              const std::optional<std::string> &device_id) {
    if (device_id.has_value()) {
        const auto match = std::ranges::find(enumeration.devices, *device_id,
                                             &airspy_tv::DeviceDescriptor::id);
        return match == enumeration.devices.end() ? nullptr : &*match;
    }
    const auto native = std::ranges::find_if(
        enumeration.devices, [](const airspy_tv::DeviceDescriptor &device) {
            return device.backend == airspy_tv::SdrBackend::AirspyNative;
        });
    if (native != enumeration.devices.end()) {
        return &*native;
    }
    return enumeration.devices.empty() ? nullptr : &enumeration.devices.front();
}

[[nodiscard]] std::uint64_t
submitted_samples(const airspy_tv::InputTimelineSnapshot &initial,
                  const airspy_tv::InputTimelineSnapshot &current) {
    return current.delivered_samples >= initial.delivered_samples
               ? current.delivered_samples - initial.delivered_samples
               : current.delivered_samples;
}

} // namespace

int live_decode_cli(LiveDecodeConfig config) {
    const bool stdout_destination =
        config.destination == std::filesystem::path("-");
    airspy_tv::EnumerationResult enumeration;
    const airspy_tv::DeviceDescriptor *descriptor = nullptr;
    if (!config.iq_input.has_value()) {
        enumeration = airspy_tv::SdrDevice::enumerate(false);
        for (const auto &warning : enumeration.warnings) {
            std::cerr << "warning: " << warning << '\n';
        }
        descriptor = select_device(enumeration, config.device_id);
        if (descriptor == nullptr) {
            if (config.device_id.has_value()) {
                std::cerr << "SDR device ID not found: " << *config.device_id
                          << '\n';
            } else {
                std::cerr << "No SDR devices found\n";
            }
            return 1;
        }
    }

    std::unique_ptr<airspy_tv::AsyncTransportOutput> output;
    std::string error;
    if (config.destination.has_value()) {
        output = std::make_unique<airspy_tv::AsyncTransportOutput>(
            airspy_tv::TransportOutputConfig{
                .queue_capacity_bytes = 8U << 20U,
                .overflow_policy =
                    airspy_tv::TransportOverflowPolicy::drop_oldest,
                .criticality = airspy_tv::TransportSinkCriticality::required,
                .thread_name = "ts-cli-output",
            });
        const bool output_started =
            stdout_destination
                ? output->start_fd(STDOUT_FILENO, false, "stdout", error)
                : output->start_file(*config.destination, error);
        if (!output_started) {
            std::cerr << error << '\n';
            return 1;
        }
    }

    airspy_tv::ReceiverSession session;
    session.set_dvbt_parameters(config.dvbt);
    if (!session.select_standard(config.standard, config.source, false,
                                 error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (output != nullptr) {
        session.set_transport_sink(
            [&output](const std::span<const std::uint8_t> transport_stream) {
                static_cast<void>(output->submit(transport_stream));
            });
    }

    airspy_tv::DecodeRunReporter reporter(config.report_directory, "live-cli");
    reporter.prepare(session);
    const bool source_started =
        config.iq_input.has_value()
            ? session.open_iq_file_and_start(*config.iq_input, config.source,
                                             error)
            : session.open_device_and_start(*descriptor, config.source, error);
    if (!source_started) {
        reporter.cancel_start(session);
        std::cerr << error << '\n';
        return 1;
    }
    if (config.rtp_output.has_value() &&
        !session.start_rtp_streaming(*config.rtp_output, error)) {
        reporter.cancel_start(session);
        std::cerr << error << '\n';
        session.close();
        return 1;
    }
    std::string destination;
    if (config.destination.has_value()) {
        destination =
            stdout_destination ? "stdout" : config.destination->string();
    }
    if (config.rtp_output.has_value()) {
        if (!destination.empty()) {
            destination += "; ";
        }
        destination +=
            "rtp://" + airspy_tv::format_rtp_udp_endpoint(*config.rtp_output);
    }
    if (!reporter.start_source(session, config.source, config.dvbt, destination,
                               error)) {
        std::cerr << "Decode report failed: " << error << '\n';
        session.close();
        return 1;
    }

    const auto *source_descriptor = session.descriptor();
    std::cerr << (config.iq_input.has_value() ? "Replaying " : "Receiving ")
              << (source_descriptor == nullptr
                      ? std::string("unknown source")
                      : source_descriptor->display_name)
              << " at " << config.source.center_frequency_hz << " Hz, "
              << config.source.sample_rate_hz << " sample/s\n";
    const SignalHandlerScope signal_handlers;
    const auto started_at = std::chrono::steady_clock::now();
    const auto initial_timeline = session.input_timeline_snapshot();
    auto last_progress = started_at;
    auto next_output_warning = started_at;
    std::uint64_t reported_dropped_blocks = 0;
    auto next_rtp_warning = started_at;
    std::uint64_t reported_rtp_drops = 0;
    std::uint64_t reported_rtp_write_errors = 0;
    int exit_code = 0;

    while (stop_live_decode == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const auto now = std::chrono::steady_clock::now();
        if (output != nullptr) {
            const auto output_stats = output->stats();
            if (output_stats.failed && output_stats.required) {
                error = output_stats.error;
                exit_code = 1;
                break;
            }
            if (output_stats.dropped_blocks > reported_dropped_blocks &&
                now >= next_output_warning) {
                const std::uint64_t newly_dropped =
                    output_stats.dropped_blocks - reported_dropped_blocks;
                reported_dropped_blocks = output_stats.dropped_blocks;
                next_output_warning = now + std::chrono::seconds(1);
                std::cerr
                    << "warning: MPEG-TS output is not keeping up; dropped "
                    << newly_dropped << " oldest queued block(s), "
                    << output_stats.dropped_blocks << " total ("
                    << static_cast<double>(output_stats.dropped_bytes) /
                           (1024.0 * 1024.0)
                    << " MiB); reception continues\n";
            }
        }
        if (config.rtp_output.has_value()) {
            const auto rtp_stats = session.rtp_streaming_stats();
            if ((rtp_stats.dropped_datagrams > reported_rtp_drops ||
                 rtp_stats.write_errors > reported_rtp_write_errors) &&
                now >= next_rtp_warning) {
                const std::uint64_t new_drops =
                    rtp_stats.dropped_datagrams - reported_rtp_drops;
                const std::uint64_t new_errors =
                    rtp_stats.write_errors - reported_rtp_write_errors;
                reported_rtp_drops = rtp_stats.dropped_datagrams;
                reported_rtp_write_errors = rtp_stats.write_errors;
                next_rtp_warning = now + std::chrono::seconds(1);
                std::cerr << "warning: RTP/UDP output dropped " << new_drops
                          << " datagram(s), send errors=" << new_errors
                          << "; reception continues\n";
            }
        }
        const std::string runtime_error = session.runtime_error();
        if (!runtime_error.empty()) {
            error = runtime_error;
            exit_code = 1;
            break;
        }
        if (!session.is_streaming()) {
            if (session.input_exhausted()) {
                std::string report_error;
                if (!reporter.update(session, report_error)) {
                    error = "Decode report failed: " + report_error;
                    exit_code = 1;
                }
                break;
            }
            error = config.iq_input.has_value() ? "I/Q input stream stopped"
                                                : "SDR input stream stopped";
            exit_code = 1;
            break;
        }
        const auto decoder_stats = session.dvbt_snapshot().decoder;
        if (decoder_stats.failed) {
            error = decoder_stats.error;
            exit_code = 1;
            break;
        }
        std::string report_error;
        if (!reporter.update(session, report_error)) {
            error = "Decode report failed: " + report_error;
            exit_code = 1;
            break;
        }

        if (now - last_progress >= std::chrono::seconds(1)) {
            last_progress = now;
            const double elapsed =
                std::chrono::duration<double>(now - started_at).count();
            airspy_tv::format_decode_progress(
                std::cerr, decoder_stats,
                submitted_samples(initial_timeline,
                                  session.input_timeline_snapshot()),
                config.source.sample_rate_hz, elapsed);
        }
    }

    session.finish_stream();
    session.set_transport_sink({});
    if (output != nullptr) {
        const auto drain_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
        while (output->stats().queued_bytes != 0 &&
               std::chrono::steady_clock::now() < drain_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        output->stop(false);
        const auto output_stats = output->stats();
        if (output_stats.failed && exit_code == 0) {
            error = output_stats.error;
            exit_code = 1;
        }
    }
    session.stop_rtp_streaming();
    if (exit_code == 0 &&
        session.dvbt_snapshot().decoder.transport_bytes == 0) {
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
