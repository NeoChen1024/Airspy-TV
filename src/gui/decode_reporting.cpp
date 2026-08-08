#include "decode_reporting.hpp"

#include "../decode_report.hpp"
#include "airspy_tv/debug.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace airspy_tv::gui {
namespace {

[[nodiscard]] std::uint64_t submitted_samples(const AppState &state) {
    const std::uint64_t delivered =
        state.session.input_timeline_snapshot().delivered_samples;
    const std::uint64_t baseline =
        state.decode_report.source_timeline_baseline.delivered_samples;
    return delivered >= baseline ? delivered - baseline : delivered;
}

[[nodiscard]] double elapsed_seconds(const AppState &state) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         state.decode_report.started_at)
        .count();
}

void restore_telemetry_selection(AppState &state) {
    state.session.set_dvbt_telemetry_enabled(is_debug_enabled(),
                                             std::chrono::steady_clock::now());
}

void abandon_report(AppState &state, const std::string &error) {
    state.status = "Decode report failed: " + error;
    state.decode_report.writer.reset();
    state.decode_report.prepared = false;
    state.decode_report.completed = true;
    restore_telemetry_selection(state);
}

bool drain_telemetry(AppState &state) {
    if (state.decode_report.writer == nullptr && !is_debug_enabled()) {
        return true;
    }

    auto records = state.session.drain_dvbt_telemetry();
    if (state.decode_report.writer != nullptr) {
        try {
            state.decode_report.writer->consume(records);
        } catch (const std::exception &exception) {
            abandon_report(state, exception.what());
            return false;
        }
    }
    if (is_debug_enabled()) {
        for (const auto &record : records) {
            format_debug_telemetry(std::cerr, record);
        }
    }
    return true;
}

void finish_active_source(AppState &state) {
    if (state.decode_report.writer == nullptr ||
        !state.decode_report.source_active) {
        return;
    }

    const auto &stats = state.dvbt.decoder;
    const std::uint64_t samples = submitted_samples(state);
    const double elapsed = elapsed_seconds(state);
    const std::string status = stats.failed                 ? "failed"
                               : stats.transport_bytes == 0 ? "no_transport"
                                                            : "completed";
    const int exit_code = stats.failed ? 1 : stats.transport_bytes == 0 ? 2 : 0;
    const std::string error = stats.failed ? stats.error : std::string{};
    try {
        state.decode_report.writer->write_pipeline(stats, samples, elapsed);
        state.decode_report.writer->end_source(
            status, error, stats, state.session.input_timeline_snapshot(),
            samples, elapsed);
        state.decode_report.writer->flush();
        state.decode_report.source_active = false;
        state.decode_report.saw_streaming = false;
        state.decode_report.any_transport |= exit_code == 0;
        state.decode_report.any_failed |= exit_code == 1;
        ++state.decode_report.source_sessions;
    } catch (const std::exception &exception) {
        abandon_report(state, exception.what());
    }
}

} // namespace

GuiDecodeReportState::GuiDecodeReportState(
    std::optional<std::filesystem::path> report_directory)
    : directory(std::move(report_directory)) {}

GuiDecodeReportState::~GuiDecodeReportState() = default;

void prepare_decode_report(AppState &state) {
    if (!state.decode_report.directory.has_value() ||
        state.decode_report.completed || state.decode_report.prepared ||
        state.decode_report.source_active) {
        return;
    }
    if (state.decode_report.writer == nullptr) {
        state.decode_report.started_at = std::chrono::steady_clock::now();
        state.session.set_dvbt_telemetry_enabled(
            true, state.decode_report.started_at);
    }
    state.decode_report.last_periodic = std::chrono::steady_clock::now();
    state.decode_report.source_timeline_baseline =
        state.session.input_timeline_snapshot();
    state.decode_report.source_stats_baseline =
        state.session.dvbt_snapshot().decoder;
    state.decode_report.prepared = true;
    state.decode_report.saw_streaming = false;
}

void cancel_decode_report_start(AppState &state) {
    if (!state.decode_report.prepared) {
        return;
    }
    state.decode_report.prepared = false;
    if (state.decode_report.writer == nullptr) {
        restore_telemetry_selection(state);
    }
}

bool start_decode_report(AppState &state, std::string &error) {
    if (!state.decode_report.prepared) {
        return true;
    }
    const DeviceDescriptor *descriptor = state.session.descriptor();
    if (descriptor == nullptr) {
        error = "Source descriptor is unavailable";
        cancel_decode_report_start(state);
        return false;
    }

    try {
        if (state.decode_report.writer == nullptr) {
            state.decode_report.writer =
                std::make_unique<DecodeReport>(DecodeReportConfig{
                    .directory = *state.decode_report.directory,
                    .context = "gui",
                });
        }
        auto timeline = state.session.input_timeline_snapshot();
        timeline.source_head_sample =
            state.decode_report.source_timeline_baseline.source_head_sample;
        timeline.delivered_samples =
            state.decode_report.source_timeline_baseline.delivered_samples;
        const auto current_stats = state.session.dvbt_snapshot().decoder;
        auto initial_stats = state.decode_report.source_stats_baseline;
        initial_stats.decoder_generation = current_stats.decoder_generation;
        initial_stats.source_epoch = current_stats.source_epoch;
        state.decode_report.writer->begin_source(
            DecodeSourceSessionConfig{
                .source = descriptor->id.empty() ? descriptor->display_name
                                                 : descriptor->id,
                .destination = "GUI transport pipeline",
                .sample_rate_hz = state.settings.sample_rate_hz,
                .center_frequency_hz = state.settings.center_frequency_hz,
                .decoder = state.dvbt.parameters,
            },
            timeline, initial_stats, elapsed_seconds(state));
    } catch (const std::exception &exception) {
        error = exception.what();
        state.decode_report.writer.reset();
        state.decode_report.prepared = false;
        state.decode_report.source_active = false;
        state.decode_report.completed = true;
        restore_telemetry_selection(state);
        return false;
    }

    state.decode_report.prepared = false;
    state.decode_report.source_active = true;
    state.decode_report.saw_streaming = true;
    return true;
}

void update_decode_report(AppState &state) {
    if (!drain_telemetry(state) || state.decode_report.writer == nullptr ||
        !state.decode_report.source_active) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    state.decode_report.saw_streaming |= state.session.is_streaming();
    if (now - state.decode_report.last_periodic >= std::chrono::seconds(1)) {
        state.decode_report.last_periodic = now;
        try {
            state.decode_report.writer->write_pipeline(state.dvbt.decoder,
                                                       submitted_samples(state),
                                                       elapsed_seconds(state));
            state.decode_report.writer->flush();
        } catch (const std::exception &exception) {
            abandon_report(state, exception.what());
            return;
        }
    }

    if (state.decode_report.saw_streaming && !state.session.is_streaming()) {
        finish_active_source(state);
    }
}

void finish_decode_report_source(AppState &state) {
    if (state.decode_report.writer == nullptr) {
        cancel_decode_report_start(state);
        return;
    }
    if (!drain_telemetry(state)) {
        return;
    }
    state.dvbt.decoder = state.session.dvbt_snapshot().decoder;
    finish_active_source(state);
}

void finalize_decode_report(AppState &state) {
    cancel_decode_report_start(state);
    if (state.decode_report.writer == nullptr) {
        return;
    }
    if (!drain_telemetry(state)) {
        return;
    }
    state.dvbt.decoder = state.session.dvbt_snapshot().decoder;
    finish_active_source(state);
    if (state.decode_report.writer == nullptr) {
        return;
    }
    const std::string_view status =
        state.decode_report.any_failed ? "failed"
        : state.decode_report.source_sessions != 0 &&
                !state.decode_report.any_transport
            ? "no_transport"
            : "completed";
    const std::string_view error = state.decode_report.any_failed
                                       ? "one or more source sessions failed"
                                       : "";
    try {
        state.decode_report.writer->finalize(status, 0, error,
                                             elapsed_seconds(state));
        state.decode_report.writer.reset();
        state.decode_report.completed = true;
        restore_telemetry_selection(state);
    } catch (const std::exception &exception) {
        abandon_report(state, exception.what());
    }
}

} // namespace airspy_tv::gui
