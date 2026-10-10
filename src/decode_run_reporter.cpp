#include "decode_run_reporter.hpp"

#include "airspy_tv/debug.hpp"
#include "decode_report.hpp"
#include "dvbt/dvbt_decode_report.hpp"
#include "receiver_session.hpp"

#include <algorithm>
#include <exception>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace airspy_tv {

DecodeRunReporter::DecodeRunReporter(
    std::optional<std::filesystem::path> directory, std::string context)
    : directory_(std::move(directory)), context_(std::move(context)) {}

DecodeRunReporter::~DecodeRunReporter() = default;

DecodeReport &DecodeRunReporter::report(const ReceiverSession &session) {
    if (report_ == nullptr) {
        if (session.standard() != ReceiveStandard::DvbT) {
            throw std::logic_error(
                "The active television standard has no decode report");
        }
        report_ = make_dvbt_decode_report();
    }
    return *report_;
}

void DecodeRunReporter::set_transport_output_provider(
    TransportOutputProvider provider) {
    transport_output_provider_ = std::move(provider);
}

std::vector<TransportOutputTelemetry>
DecodeRunReporter::transport_outputs(const ReceiverSession &session) const {
    auto outputs = session.transport_output_telemetry();
    if (transport_output_provider_) {
        auto external = transport_output_provider_();
        outputs.insert(outputs.end(), std::make_move_iterator(external.begin()),
                       std::make_move_iterator(external.end()));
    }
    return outputs;
}

std::uint64_t
DecodeRunReporter::submitted_samples(const ReceiverSession &session) const {
    const std::uint64_t delivered =
        session.input_timeline_snapshot().delivered_samples;
    const std::uint64_t baseline = source_timeline_baseline_.delivered_samples;
    return delivered >= baseline ? delivered - baseline : delivered;
}

double DecodeRunReporter::elapsed_seconds() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         started_at_)
        .count();
}

void DecodeRunReporter::restore_telemetry_selection(ReceiverSession &session) {
    session.set_telemetry_enabled(is_debug_enabled(),
                                  std::chrono::steady_clock::now());
}

bool DecodeRunReporter::abandon(ReceiverSession &session,
                                const std::string_view message,
                                std::string &error) {
    error = std::string(message);
    report(session).close();
    prepared_ = false;
    source_active_ = false;
    completed_ = true;
    restore_telemetry_selection(session);
    return false;
}

bool DecodeRunReporter::drain_telemetry(ReceiverSession &session,
                                        std::string &error) {
    auto &writer = report(session);
    if (!writer.is_open() && !is_debug_enabled()) {
        return true;
    }
    try {
        writer.drain_telemetry(session, is_debug_enabled());
    } catch (const std::exception &exception) {
        return abandon(session, exception.what(), error);
    }
    return true;
}

void DecodeRunReporter::prepare(ReceiverSession &session) {
    if (completed_ || prepared_ || source_active_) {
        return;
    }
    if (!directory_.has_value()) {
        session.set_telemetry_enabled(is_debug_enabled(),
                                      std::chrono::steady_clock::now());
        return;
    }
    auto &writer = report(session);
    if (!writer.is_open()) {
        started_at_ = std::chrono::steady_clock::now();
        session.set_telemetry_enabled(true, started_at_);
    }
    last_periodic_ = std::chrono::steady_clock::now();
    source_timeline_baseline_ = session.input_timeline_snapshot();
    source_transport_bytes_baseline_ =
        writer.capture_baseline(session).transport_bytes;
    prepared_ = true;
    saw_streaming_ = false;
}

void DecodeRunReporter::cancel_start(ReceiverSession &session) {
    if (!prepared_) {
        return;
    }
    prepared_ = false;
    if (!report(session).is_open()) {
        restore_telemetry_selection(session);
    }
}

bool DecodeRunReporter::start_source(ReceiverSession &session,
                                     const SourceSettings &settings,
                                     const std::string_view destination,
                                     std::string &error) {
    if (!prepared_) {
        return true;
    }
    const auto report_directory = directory_;
    if (!report_directory.has_value()) {
        prepared_ = false;
        error = "Decode report directory is unavailable";
        return false;
    }
    const DeviceDescriptor *descriptor = session.descriptor();
    if (descriptor == nullptr) {
        cancel_start(session);
        error = "Source descriptor is unavailable";
        return false;
    }

    auto &writer = report(session);
    try {
        if (!writer.is_open()) {
            writer.open(DecodeReportConfig{
                .directory = report_directory.value(),
                .context = context_,
            });
        }
        auto timeline = session.input_timeline_snapshot();
        timeline.source_head_sample =
            source_timeline_baseline_.source_head_sample;
        timeline.delivered_samples =
            source_timeline_baseline_.delivered_samples;
        const auto outputs = transport_outputs(session);
        writer.begin_source(
            session,
            DecodeSourceSessionConfig{
                .source = descriptor->id.empty() ? descriptor->display_name
                                                 : descriptor->id,
                .destination = std::string(destination),
                .sample_rate_hz = settings.sample_rate_hz,
                .center_frequency_hz = settings.center_frequency_hz,
            },
            timeline, elapsed_seconds(), outputs);
    } catch (const std::exception &exception) {
        return abandon(session, exception.what(), error);
    }

    prepared_ = false;
    source_active_ = true;
    saw_streaming_ = true;
    return true;
}

bool DecodeRunReporter::update(ReceiverSession &session, std::string &error,
                               const bool finish_stopped_source) {
    auto &writer = report(session);
    if (!drain_telemetry(session, error) || !writer.is_open() ||
        !source_active_) {
        return error.empty();
    }

    const auto now = std::chrono::steady_clock::now();
    saw_streaming_ |= session.is_streaming();
    if (now - last_periodic_ >= std::chrono::seconds(1)) {
        last_periodic_ = now;
        try {
            static_cast<void>(writer.sample(session));
            writer.write_sample(submitted_samples(session), elapsed_seconds(),
                                transport_outputs(session));
            writer.flush();
        } catch (const std::exception &exception) {
            return abandon(session, exception.what(), error);
        }
    }

    if (finish_stopped_source && saw_streaming_ && !session.is_streaming()) {
        return finish_source(session, error);
    }
    return true;
}

bool DecodeRunReporter::finish_source(ReceiverSession &session,
                                      std::string &error,
                                      const std::string_view source_failure) {
    auto &writer = report(session);
    if (!writer.is_open()) {
        cancel_start(session);
        return true;
    }
    if (!drain_telemetry(session, error) || !source_active_) {
        return error.empty();
    }

    const auto stats = writer.sample(session);
    const std::uint64_t samples = submitted_samples(session);
    const double elapsed = elapsed_seconds();
    const std::uint64_t transport_bytes =
        stats.transport_bytes >= source_transport_bytes_baseline_
            ? stats.transport_bytes - source_transport_bytes_baseline_
            : stats.transport_bytes;
    const bool source_failed = stats.failed || !source_failure.empty();
    std::string status = "completed";
    int exit_code = 0;
    if (source_failed) {
        status = "failed";
        exit_code = 1;
    } else if (transport_bytes == 0) {
        status = "no_transport";
        exit_code = 2;
    }
    std::string source_error;
    if (!source_failure.empty()) {
        source_error = source_failure;
    } else if (stats.failed) {
        source_error = stats.error;
    }
    try {
        writer.write_sample(samples, elapsed, transport_outputs(session));
        writer.end_source(status, source_error,
                          session.input_timeline_snapshot(), samples, elapsed);
        writer.flush();
        source_active_ = false;
        saw_streaming_ = false;
        any_transport_ |= exit_code == 0;
        any_failed_ |= exit_code == 1;
        ++source_sessions_;
    } catch (const std::exception &exception) {
        return abandon(session, exception.what(), error);
    }
    return true;
}

bool DecodeRunReporter::finalize(ReceiverSession &session, std::string &error,
                                 const std::string_view run_failure,
                                 const int exit_code) {
    cancel_start(session);
    auto &writer = report(session);
    if (!writer.is_open()) {
        return drain_telemetry(session, error);
    }
    if (!finish_source(session, error, run_failure) || !writer.is_open()) {
        return error.empty();
    }

    const bool run_failed = any_failed_ || !run_failure.empty();
    std::string_view status = "completed";
    if (run_failed) {
        status = "failed";
    } else if (source_sessions_ != 0 && !any_transport_) {
        status = "no_transport";
    }
    std::string run_error;
    if (!run_failure.empty()) {
        run_error = run_failure;
    } else if (any_failed_) {
        run_error = "one or more source sessions failed";
    }
    try {
        writer.finalize(status, run_failed ? std::max(exit_code, 1) : exit_code,
                        run_error, elapsed_seconds());
        writer.close();
        completed_ = true;
        restore_telemetry_selection(session);
    } catch (const std::exception &exception) {
        return abandon(session, exception.what(), error);
    }
    return true;
}

} // namespace airspy_tv
