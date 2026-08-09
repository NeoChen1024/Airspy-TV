#include "decode_run_reporter.hpp"

#include "airspy_tv/debug.hpp"
#include "decode_report.hpp"
#include "receiver_session.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <iterator>
#include <utility>

namespace airspy_tv {

DecodeRunReporter::DecodeRunReporter(
    std::optional<std::filesystem::path> directory, std::string context)
    : directory_(std::move(directory)), context_(std::move(context)) {}

DecodeRunReporter::~DecodeRunReporter() = default;

bool DecodeRunReporter::enabled() const noexcept {
    return directory_.has_value();
}

bool DecodeRunReporter::source_active() const noexcept {
    return source_active_;
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
    session.set_dvbt_telemetry_enabled(is_debug_enabled(),
                                       std::chrono::steady_clock::now());
}

bool DecodeRunReporter::abandon(ReceiverSession &session,
                                const std::string_view message,
                                std::string &error) {
    error = std::string(message);
    writer_.reset();
    prepared_ = false;
    source_active_ = false;
    completed_ = true;
    restore_telemetry_selection(session);
    return false;
}

bool DecodeRunReporter::drain_telemetry(ReceiverSession &session,
                                        std::string &error) {
    if (writer_ == nullptr && !is_debug_enabled()) {
        return true;
    }

    auto records = session.drain_dvbt_telemetry();
    if (writer_ != nullptr) {
        try {
            writer_->consume(records);
        } catch (const std::exception &exception) {
            return abandon(session, exception.what(), error);
        }
    }
    if (is_debug_enabled()) {
        for (const auto &record : records) {
            format_debug_telemetry(std::cerr, record);
        }
    }
    return true;
}

void DecodeRunReporter::prepare(ReceiverSession &session) {
    if (completed_ || prepared_ || source_active_) {
        return;
    }
    if (!directory_.has_value()) {
        session.set_dvbt_telemetry_enabled(is_debug_enabled(),
                                           std::chrono::steady_clock::now());
        return;
    }
    if (writer_ == nullptr) {
        started_at_ = std::chrono::steady_clock::now();
        session.set_dvbt_telemetry_enabled(true, started_at_);
    }
    last_periodic_ = std::chrono::steady_clock::now();
    source_timeline_baseline_ = session.input_timeline_snapshot();
    source_stats_baseline_ = session.dvbt_snapshot().decoder;
    prepared_ = true;
    saw_streaming_ = false;
}

void DecodeRunReporter::cancel_start(ReceiverSession &session) {
    if (!prepared_) {
        return;
    }
    prepared_ = false;
    if (writer_ == nullptr) {
        restore_telemetry_selection(session);
    }
}

bool DecodeRunReporter::start_source(ReceiverSession &session,
                                     const SourceSettings &settings,
                                     const dvbt::ReceiverParameters &parameters,
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

    try {
        if (writer_ == nullptr) {
            writer_ = std::make_unique<DecodeReport>(DecodeReportConfig{
                .directory = report_directory.value(),
                .context = context_,
            });
        }
        auto timeline = session.input_timeline_snapshot();
        timeline.source_head_sample =
            source_timeline_baseline_.source_head_sample;
        timeline.delivered_samples =
            source_timeline_baseline_.delivered_samples;
        const auto current_stats = session.dvbt_snapshot().decoder;
        auto initial_stats = source_stats_baseline_;
        initial_stats.decoder_generation = current_stats.decoder_generation;
        initial_stats.source_epoch = current_stats.source_epoch;
        const auto outputs = transport_outputs(session);
        writer_->begin_source(
            DecodeSourceSessionConfig{
                .source = descriptor->id.empty() ? descriptor->display_name
                                                 : descriptor->id,
                .destination = std::string(destination),
                .sample_rate_hz = settings.sample_rate_hz,
                .center_frequency_hz = settings.center_frequency_hz,
                .decoder = parameters,
            },
            timeline, initial_stats, elapsed_seconds(), outputs);
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
    if (!drain_telemetry(session, error) || writer_ == nullptr ||
        !source_active_) {
        return error.empty();
    }

    const auto now = std::chrono::steady_clock::now();
    saw_streaming_ |= session.is_streaming();
    if (now - last_periodic_ >= std::chrono::seconds(1)) {
        last_periodic_ = now;
        try {
            const auto stats = session.dvbt_snapshot().decoder;
            const double elapsed = elapsed_seconds();
            writer_->write_pipeline(stats, submitted_samples(session), elapsed);
            writer_->write_transport_outputs(transport_outputs(session),
                                             stats.decoder_generation,
                                             stats.source_epoch, elapsed);
            writer_->flush();
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
    if (writer_ == nullptr) {
        cancel_start(session);
        return true;
    }
    if (!drain_telemetry(session, error) || !source_active_) {
        return error.empty();
    }

    const auto stats = session.dvbt_snapshot().decoder;
    const std::uint64_t samples = submitted_samples(session);
    const double elapsed = elapsed_seconds();
    const std::uint64_t transport_bytes =
        stats.transport_bytes >= source_stats_baseline_.transport_bytes
            ? stats.transport_bytes - source_stats_baseline_.transport_bytes
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
        writer_->write_pipeline(stats, samples, elapsed);
        writer_->write_transport_outputs(transport_outputs(session),
                                         stats.decoder_generation,
                                         stats.source_epoch, elapsed);
        writer_->end_source(status, source_error, stats,
                            session.input_timeline_snapshot(), samples,
                            elapsed);
        writer_->flush();
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
    if (writer_ == nullptr) {
        return drain_telemetry(session, error);
    }
    if (!finish_source(session, error, run_failure) || writer_ == nullptr) {
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
        writer_->finalize(status,
                          run_failed ? std::max(exit_code, 1) : exit_code,
                          run_error, elapsed_seconds());
        writer_.reset();
        completed_ = true;
        restore_telemetry_selection(session);
    } catch (const std::exception &exception) {
        return abandon(session, exception.what(), error);
    }
    return true;
}

} // namespace airspy_tv
