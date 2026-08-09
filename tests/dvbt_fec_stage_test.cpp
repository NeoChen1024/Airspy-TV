#include "fec_stage.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::TransportDiscontinuity;
using airspy_tv::dvbt::CodeRate;
using airspy_tv::dvbt::Constellation;
using airspy_tv::dvbt::DecoderParameters;
using airspy_tv::dvbt::FecItem;
using airspy_tv::dvbt::FecStage;
using airspy_tv::dvbt::FecStageDiagnosticContext;
using airspy_tv::dvbt::FecStageSession;
using airspy_tv::dvbt::FecStageWindow;
using airspy_tv::dvbt::TransmissionMode;

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool test_ordered_sessions_and_generation_filtering() {
    std::atomic<std::uint64_t> current_generation{1};
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<std::uint64_t> sessions;
    std::vector<std::uint64_t> windows;
    std::vector<TransportDiscontinuity> discontinuities;
    std::string failure;

    FecStage stage(
        {.generation_current =
             [&current_generation](const std::uint64_t generation) {
                 return generation == current_generation.load();
             },
         .publish_session =
             [&](const FecStageSession &session) {
                 const std::scoped_lock lock(mutex);
                 sessions.push_back(session.fec_session);
                 changed.notify_all();
             },
         .publish_window =
             [&](const FecStageWindow &window) {
                 const std::scoped_lock lock(mutex);
                 windows.push_back(window.demod_window_sequence);
                 changed.notify_all();
             },
         .emit_transport = [](std::uint64_t,
                              std::span<const std::uint8_t>) { return true; },
         .emit_discontinuity =
             [&](std::uint64_t, const TransportDiscontinuity discontinuity) {
                 const std::scoped_lock lock(mutex);
                 discontinuities.push_back(discontinuity);
                 changed.notify_all();
             },
         .diagnostics_enabled = [] { return false; },
         .emit_diagnostic = [](const airspy_tv::DiagnosticEvent &,
                               const FecStageDiagnosticContext &) {},
         .notify_idle = [&] { changed.notify_all(); },
         .worker_failed =
             [&](const std::exception_ptr &, std::string message) {
                 const std::scoped_lock lock(mutex);
                 failure = std::move(message);
                 changed.notify_all();
             }},
        2);

    const DecoderParameters parameters{
        TransmissionMode::k2, Constellation::qpsk, CodeRate::rate_1_2, 1};
    if (!require(stage.enqueue({.kind = FecItem::Kind::begin,
                                .generation = 1,
                                .parameters = parameters,
                                .mother_metrics = {},
                                .symbol_index = 0,
                                .demod_window_sequence = 0,
                                .source_epoch = 0}),
                 "first FEC session is accepted") ||
        !require(stage.enqueue({.kind = FecItem::Kind::begin,
                                .generation = 1,
                                .parameters = parameters,
                                .mother_metrics = {},
                                .symbol_index = 0,
                                .demod_window_sequence = 0,
                                .source_epoch = 0}),
                 "same-generation reset is accepted") ||
        !require(stage.enqueue({.kind = FecItem::Kind::stats,
                                .generation = 1,
                                .parameters = {},
                                .mother_metrics = {},
                                .symbol_index = 0,
                                .demod_window_sequence = 11,
                                .source_epoch = 7}),
                 "FEC stats marker is accepted") ||
        !require(stage.enqueue({.kind = FecItem::Kind::stream_end,
                                .generation = 1,
                                .parameters = {},
                                .mother_metrics = {},
                                .symbol_index = 0,
                                .demod_window_sequence = 12,
                                .source_epoch = 7}),
                 "FEC stream end is accepted")) {
        return false;
    }

    {
        std::unique_lock lock(mutex);
        changed.wait_for(lock, std::chrono::seconds(2), [&] {
            if (!failure.empty()) {
                return true;
            }
            if (sessions.size() != 2 || windows.size() != 2 ||
                discontinuities.size() != 2) {
                return false;
            }
            const auto snapshot = stage.snapshot();
            return snapshot.queued_items == 0 && !snapshot.processing;
        });
    }
    const auto snapshot = stage.snapshot();
    const bool ordered =
        require(failure.empty(), "FEC worker stays healthy") &&
        require(sessions == std::vector<std::uint64_t>({1, 2}),
                "FEC sessions remain ordered") &&
        require(windows == std::vector<std::uint64_t>({11, 12}),
                "FEC windows remain ordered") &&
        require(discontinuities.size() == 2 &&
                    discontinuities[0] ==
                        TransportDiscontinuity::fec_region_reset &&
                    discontinuities[1] == TransportDiscontinuity::stream_end,
                "FEC reset precedes serialized stream end") &&
        require(snapshot.queued_items == 0 && !snapshot.processing,
                "FEC stage reaches an idle snapshot") &&
        require(snapshot.capacity == 2, "FEC queue capacity is observable");

    current_generation.store(2);
    stage.notify_cancelled();
    const bool stale_rejected =
        require(!stage.enqueue({.kind = FecItem::Kind::stats,
                                .generation = 1,
                                .parameters = {},
                                .mother_metrics = {},
                                .symbol_index = 0,
                                .demod_window_sequence = 13,
                                .source_epoch = 0}),
                "stale generation is rejected");
    stage.set_capacity(4);
    const bool capacity_updated =
        require(stage.snapshot().capacity == 4,
                "FEC queue capacity updates atomically");
    stage.stop();
    return ordered && stale_rejected && capacity_updated;
}

} // namespace

int main() {
    try {
        return test_ordered_sessions_and_generation_filtering() ? 0 : 1;
    } catch (const std::exception &exception) {
        std::cerr << "FEC stage test failed: " << exception.what() << '\n';
        return 1;
    }
}
