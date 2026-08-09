#pragma once

#include "airspy_tv/dvbt/stream_decoder.hpp"
#include "airspy_tv/sdr.hpp"

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace airspy_tv {

class DecodeReport;
class ReceiverSession;

// Owns one run-scoped DecodeReport and its per-source boundaries. This is
// shared by live GUI and CLI adapters; it intentionally contains no UI,
// argument parsing, signal handling, or source-start policy.
class DecodeRunReporter {
  public:
    DecodeRunReporter(
        std::optional<std::filesystem::path> directory = std::nullopt,
        std::string context = {});
    ~DecodeRunReporter();

    DecodeRunReporter(const DecodeRunReporter &) = delete;
    DecodeRunReporter &operator=(const DecodeRunReporter &) = delete;

    void prepare(ReceiverSession &session);
    void cancel_start(ReceiverSession &session);
    bool start_source(ReceiverSession &session, const SourceSettings &settings,
                      const dvbt::ReceiverParameters &parameters,
                      std::string_view destination, std::string &error);
    bool update(ReceiverSession &session, std::string &error,
                bool finish_stopped_source = true);
    bool finish_source(ReceiverSession &session, std::string &error,
                       std::string_view source_failure = {});
    bool finalize(ReceiverSession &session, std::string &error,
                  std::string_view run_failure = {}, int exit_code = 0);

    [[nodiscard]] bool enabled() const noexcept;
    [[nodiscard]] bool source_active() const noexcept;

  private:
    [[nodiscard]] std::uint64_t
    submitted_samples(const ReceiverSession &session) const;
    [[nodiscard]] double elapsed_seconds() const;
    static void restore_telemetry_selection(ReceiverSession &session);
    bool abandon(ReceiverSession &session, std::string_view message,
                 std::string &error);
    bool drain_telemetry(ReceiverSession &session, std::string &error);

    std::optional<std::filesystem::path> directory_;
    std::string context_;
    std::unique_ptr<DecodeReport> writer_;
    std::chrono::steady_clock::time_point started_at_{};
    std::chrono::steady_clock::time_point last_periodic_{};
    InputTimelineSnapshot source_timeline_baseline_;
    dvbt::StreamDecoderStats source_stats_baseline_;
    bool prepared_{};
    bool source_active_{};
    bool completed_{};
    bool saw_streaming_{};
    bool any_transport_{};
    bool any_failed_{};
    std::uint64_t source_sessions_{};
};

} // namespace airspy_tv
