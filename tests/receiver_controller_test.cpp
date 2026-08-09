#include "receiver_controller.hpp"

#include "airspy_tv/jsonl.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <unistd.h>

namespace {

class TemporaryDirectory {
  public:
    TemporaryDirectory() {
        std::string pattern = "/tmp/airspy-tv-controller-XXXXXX";
        if (::mkdtemp(pattern.data()) != nullptr) {
            path = pattern;
        }
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }

    TemporaryDirectory(const TemporaryDirectory &) = delete;
    TemporaryDirectory &operator=(const TemporaryDirectory &) = delete;
    TemporaryDirectory(TemporaryDirectory &&) = delete;
    TemporaryDirectory &operator=(TemporaryDirectory &&) = delete;

    std::filesystem::path path;
};

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool test_unopened_retune_is_a_pure_configuration_change() {
    airspy_tv::ReceiverSession session;
    airspy_tv::DecodeRunReporter reporter;
    airspy_tv::SourceSettings settings;
    airspy_tv::dvbt::ReceiverParameters parameters;
    airspy_tv::ReceiverController controller(session, reporter, settings,
                                             parameters);
    const auto result = controller.retune(557'000'000);
    return require(result.success, "unopened retune succeeds") &&
           require(settings.center_frequency_hz == 557'000'000,
                   "controller updates source configuration") &&
           require(!session.is_open(),
                   "configuration-only retune does not open a source") &&
           require(controller.close().success,
                   "closing an idle controller is idempotent");
}

bool test_source_report_lifecycle_across_replay_and_failed_retune() {
    const TemporaryDirectory directory;
    if (!require(!directory.path.empty(), "temporary directory is available")) {
        return false;
    }
    const auto source_path = directory.path / "source.cs16";
    {
        std::ofstream source(source_path, std::ios::binary);
    }
    std::error_code resize_error;
    std::filesystem::resize_file(source_path, 40'000'000, resize_error);
    if (!require(!resize_error, "sparse realtime I/Q source is created")) {
        return false;
    }

    airspy_tv::ReceiverSession session;
    airspy_tv::DecodeRunReporter reporter(directory.path / "report", "test");
    airspy_tv::SourceSettings settings;
    settings.sample_rate_hz = 10'000'000;
    airspy_tv::dvbt::ReceiverParameters parameters;
    airspy_tv::ReceiverController controller(session, reporter, settings,
                                             parameters);

    const auto opened = controller.open_iq_file(source_path);
    if (!require(opened.success, "controller opens an I/Q source") ||
        !require(session.is_streaming(), "opened source is streaming")) {
        return false;
    }
    const auto replayed = controller.restart_stream("Replaying I/Q file");
    if (!require(replayed.success, "controller restarts an active source") ||
        !require(session.is_streaming(), "replayed source is streaming")) {
        return false;
    }

    const auto frequency_before = settings.center_frequency_hz;
    const auto retuned = controller.retune(557'000'000);
    if (!require(!retuned.success, "file-source retune fails") ||
        !require(retuned.message.find("fixed by its metadata") !=
                     std::string::npos,
                 "retune failure is preserved for the UI") ||
        !require(settings.center_frequency_hz == frequency_before,
                 "failed retune does not alter selected frequency") ||
        !require(session.is_streaming(),
                 "failed in-place retune leaves the source streaming")) {
        return false;
    }

    if (!require(controller.close().success, "controller closes the source") ||
        !require(!session.is_open(), "closed source releases its backend")) {
        return false;
    }
    std::string report_error;
    if (!require(reporter.finalize(session, report_error),
                 "controller report finalizes: " + report_error)) {
        return false;
    }

    std::ifstream sessions_stream(directory.path /
                                  "report/source-sessions.jsonl");
    airspy_tv::JsonlReader sessions(sessions_stream);
    std::size_t session_count = 0;
    while (sessions.read().has_value()) {
        ++session_count;
    }
    return require(
        session_count == 3,
        "open, replay, and failed retune produce three source sessions");
}

} // namespace

int main() {
    return test_unopened_retune_is_a_pure_configuration_change() &&
                   test_source_report_lifecycle_across_replay_and_failed_retune()
               ? 0
               : 1;
}
