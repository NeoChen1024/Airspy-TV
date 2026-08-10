#include "airspy_tv/transport_output.hpp"
#include "transport_write_all.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <span>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

class TemporaryFile {
  public:
    TemporaryFile() {
        std::string pattern = "/tmp/airspy-tv-output-XXXXXX";
        const int fd = ::mkstemp(pattern.data());
        if (fd >= 0) {
            static_cast<void>(::close(fd));
            path = pattern;
        }
    }

    ~TemporaryFile() {
        std::error_code error;
        std::filesystem::remove(path, error);
    }

    TemporaryFile(const TemporaryFile &) = delete;
    TemporaryFile &operator=(const TemporaryFile &) = delete;
    TemporaryFile(TemporaryFile &&) = delete;
    TemporaryFile &operator=(TemporaryFile &&) = delete;

    std::filesystem::path path;
};

bool require(const bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

std::vector<std::uint8_t> read_bytes(const std::filesystem::path &path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()};
}

bool test_ordered_file_round_trip_and_drain() {
    const TemporaryFile file;
    if (!require(!file.path.empty(), "temporary output file is available")) {
        return false;
    }
    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 1U << 20U,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::fail_sink,
        .criticality = airspy_tv::TransportSinkCriticality::required,
        .write_batch_bytes = 188 * 28,
        .write_batch_delay = std::chrono::milliseconds(2),
        .thread_name = "ts-out-test",
    });
    std::string error;
    if (!require(output.start_file(file.path, error),
                 "file output starts: " + error)) {
        return false;
    }

    std::vector<std::uint8_t> expected;
    for (std::uint8_t block_index = 0; block_index < 32; ++block_index) {
        std::vector<std::uint8_t> block(std::size_t{188} * 7, block_index);
        expected.insert(expected.end(), block.begin(), block.end());
        if (!require(output.submit(block), "ordered block is accepted")) {
            return false;
        }
    }
    output.stop(true);

    const auto stats = output.stats();
    return require(!stats.failed, "lossless file output did not fail") &&
           require(stats.blocks_accepted == 32 && stats.blocks_written == 32,
                   "batching preserves logical block statistics") &&
           require(stats.bytes_written == expected.size(),
                   "all queued bytes were drained") &&
           require(read_bytes(file.path) == expected,
                   "output preserves block and byte ordering");
}

bool test_required_overflow_fails_sink() {
    const TemporaryFile file;
    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 16,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::fail_sink,
        .criticality = airspy_tv::TransportSinkCriticality::required,
        .thread_name = "ts-out-test",
    });
    std::string error;
    if (!require(output.start_file(file.path, error),
                 "required output starts")) {
        return false;
    }
    const std::vector<std::uint8_t> oversized(17, 0x47);
    const bool accepted = output.submit(oversized);
    output.stop(true);
    const auto stats = output.stats();
    return require(!accepted, "oversized required block is rejected") &&
           require(stats.failed, "required overflow fails its sink") &&
           require(stats.required, "required criticality is reported") &&
           require(stats.dropped_blocks == 1,
                   "required overflow increments drop count");
}

bool test_optional_overflow_is_sink_local() {
    const TemporaryFile file;
    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 16,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::drop_newest,
        .criticality = airspy_tv::TransportSinkCriticality::optional,
        .thread_name = "ts-out-test",
    });
    std::string error;
    if (!require(output.start_file(file.path, error),
                 "optional output starts")) {
        return false;
    }
    const std::vector<std::uint8_t> oversized(17, 0x47);
    const bool accepted = output.submit(oversized);
    const auto active_stats = output.stats();
    const std::vector<std::uint8_t> valid(16, 0x48);
    const bool valid_accepted = output.submit(valid);
    output.stop(true);
    const auto stats = output.stats();
    return require(!accepted, "optional oversized block is dropped") &&
           require(active_stats.active && !active_stats.failed,
                   "optional overflow leaves sink active") &&
           require(valid_accepted, "optional sink accepts following data") &&
           require(!stats.required && !stats.failed,
                   "optional failure policy stays sink-local") &&
           require(read_bytes(file.path) == valid,
                   "optional sink writes later accepted data");
}

bool test_write_failure_is_reported() {
    if (!std::filesystem::exists("/dev/full")) {
        return true;
    }
    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 4096,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::fail_sink,
        .criticality = airspy_tv::TransportSinkCriticality::required,
        .thread_name = "ts-out-test",
    });
    std::string error;
    if (!require(output.start_file("/dev/full", error),
                 "/dev/full output starts")) {
        return false;
    }
    const std::vector<std::uint8_t> block(188, 0x47);
    if (!require(output.submit(block), "write-failure block is queued")) {
        return false;
    }
    for (int attempt = 0; attempt < 100 && !output.stats().failed; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    output.stop(true);
    const auto stats = output.stats();
    return require(stats.failed, "worker write failure is reported") &&
           require(!stats.error.empty(), "write failure includes an error") &&
           require(stats.blocks_accepted == 1 && stats.blocks_written == 0 &&
                       stats.dropped_blocks == 1 &&
                       stats.dropped_bytes == block.size(),
                   "failed in-flight block is accounted exactly once") &&
           require(stats.queued_bytes == 0,
                   "failed in-flight accounting is cleared");
}

bool test_deterministic_partial_write_sequence() {
    enum class ScriptStep { partial_three, eintr, partial_two, eagain, finish };
    const std::vector<std::uint8_t> input{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<std::uint8_t> written;
    std::size_t call = 0;
    constexpr std::array script{ScriptStep::partial_three, ScriptStep::eintr,
                                ScriptStep::partial_two, ScriptStep::eagain,
                                ScriptStep::finish};
    std::atomic<bool> abort_requested{false};
    std::string error;
    const bool ok = airspy_tv::detail::transport_write_all(
        input, abort_requested,
        [&](const std::span<const std::uint8_t> remaining) {
            const auto step = script.at(call++);
            if (step == ScriptStep::eintr || step == ScriptStep::eagain) {
                return airspy_tv::detail::TransportWriteStepResult{
                    .kind =
                        airspy_tv::detail::TransportWriteStepKind::retry,
                    .bytes = 0,
                    .error = {}};
            }
            const std::size_t count =
                step == ScriptStep::partial_three
                    ? 3U
                    : step == ScriptStep::partial_two ? 2U : remaining.size();
            written.insert(written.end(), remaining.begin(),
                           remaining.begin() +
                               static_cast<std::ptrdiff_t>(count));
            return airspy_tv::detail::TransportWriteStepResult{
                .kind = airspy_tv::detail::TransportWriteStepKind::progress,
                .bytes = count,
                .error = {}};
        },
        error);
    return require(ok && error.empty(),
                   "partial writes and retries complete successfully") &&
           require(written == input,
                   "partial-write loop neither duplicates nor loses bytes") &&
           require(call == script.size(),
                   "partial/EINTR/EAGAIN sequence is consumed exactly once");
}

bool test_deterministic_write_failure_and_cancel() {
    const std::vector<std::uint8_t> input{0, 1, 2, 3};
    std::atomic<bool> abort_requested{false};
    std::string error;
    std::size_t call = 0;
    const bool failed = airspy_tv::detail::transport_write_all(
        input, abort_requested,
        [&](const std::span<const std::uint8_t>) {
            if (call++ == 0) {
                return airspy_tv::detail::TransportWriteStepResult{
                    .kind =
                        airspy_tv::detail::TransportWriteStepKind::progress,
                    .bytes = 2,
                    .error = {}};
            }
            return airspy_tv::detail::TransportWriteStepResult{
                .kind = airspy_tv::detail::TransportWriteStepKind::failure,
                .bytes = 0,
                .error = "injected ENOSPC"};
        },
        error);
    if (!require(!failed && error == "injected ENOSPC",
                 "deterministic terminal error is preserved")) {
        return false;
    }

    abort_requested = true;
    error.clear();
    bool called = false;
    const bool canceled = airspy_tv::detail::transport_write_all(
        input, abort_requested,
        [&](const std::span<const std::uint8_t>) {
            called = true;
            return airspy_tv::detail::TransportWriteStepResult{};
        },
        error);
    return require(!canceled && error.empty(),
                   "cancellation is distinct from writer failure") &&
           require(!called, "preexisting cancellation performs no write");
}

bool test_scripted_writer_core_drain_failure_and_abort() {
    const std::vector<std::uint8_t> first{0, 1, 2, 3};
    const std::vector<std::uint8_t> second{4, 5, 6};
    std::vector<std::uint8_t> written;
    std::size_t calls = 0;
    airspy_tv::detail::TransportWriterCore draining(
        {.queue_capacity_bytes = 64, .thread_name = "ts-core-test"},
        [&](const std::span<const std::uint8_t> remaining) {
            ++calls;
            if (calls == 2 || calls == 5) {
                return airspy_tv::detail::TransportWriteStepResult{
                    .kind = airspy_tv::detail::TransportWriteStepKind::retry,
                    .bytes = 0,
                    .error = {}};
            }
            const std::size_t count = std::min<std::size_t>(2, remaining.size());
            written.insert(written.end(), remaining.begin(),
                           remaining.begin() +
                               static_cast<std::ptrdiff_t>(count));
            return airspy_tv::detail::TransportWriteStepResult{
                .kind = airspy_tv::detail::TransportWriteStepKind::progress,
                .bytes = count,
                .error = {}};
        });
    std::string error;
    if (!require(draining.start(error), "scripted drain core starts: " + error) ||
        !require(draining.submit(first) && draining.submit(second),
                 "scripted drain accepts both blocks")) {
        return false;
    }
    draining.stop(true);
    const auto drained = draining.stats();
    const std::vector<std::uint8_t> expected{0, 1, 2, 3, 4, 5, 6};
    if (!require(written == expected,
                 "drain stop preserves scripted partial-write ordering") ||
        !require(drained.blocks_accepted == 2 && drained.blocks_written == 2 &&
                     drained.bytes_written == expected.size() &&
                     drained.dropped_blocks == 0,
                 "drain counters count each logical block once")) {
        return false;
    }

    std::promise<void> failure_seen;
    auto failure_ready = failure_seen.get_future();
    bool failure_signaled = false;
    airspy_tv::detail::TransportWriterCore failing(
        {.queue_capacity_bytes = 64, .thread_name = "ts-core-test"},
        [&](const std::span<const std::uint8_t>) {
            if (!failure_signaled) {
                failure_signaled = true;
                failure_seen.set_value();
            }
            return airspy_tv::detail::TransportWriteStepResult{
                .kind = airspy_tv::detail::TransportWriteStepKind::failure,
                .error = "injected ENOSPC"};
        });
    error.clear();
    if (!require(failing.start(error), "scripted failure core starts: " + error) ||
        !require(failing.submit(first), "scripted failure block is accepted") ||
        !require(failure_ready.wait_for(std::chrono::seconds(1)) ==
                     std::future_status::ready,
                 "scripted failure reaches the writer")) {
        return false;
    }
    failing.stop(true);
    const auto failed = failing.stats();
    if (!require(failed.failed && failed.error == "injected ENOSPC" &&
                     failed.blocks_accepted == 1 && failed.blocks_written == 0 &&
                     failed.dropped_blocks == 1 &&
                     failed.dropped_bytes == first.size(),
                 "terminal scripted failure has exact accounting")) {
        return false;
    }

    std::promise<void> retry_seen;
    auto retry_ready = retry_seen.get_future();
    std::atomic_bool retry_signaled{};
    airspy_tv::detail::TransportWriterCore aborting(
        {.queue_capacity_bytes = 64, .thread_name = "ts-core-test"},
        [&](const std::span<const std::uint8_t>) {
            if (!retry_signaled.exchange(true)) {
                retry_seen.set_value();
            }
            return airspy_tv::detail::TransportWriteStepResult{
                .kind = airspy_tv::detail::TransportWriteStepKind::retry,
                .bytes = 0,
                .error = {}};
        });
    error.clear();
    if (!require(aborting.start(error), "scripted abort core starts: " + error) ||
        !require(aborting.submit(first) && aborting.submit(second),
                 "scripted abort accepts queued blocks") ||
        !require(retry_ready.wait_for(std::chrono::seconds(1)) ==
                     std::future_status::ready,
                 "scripted abort reaches an in-flight retry")) {
        return false;
    }
    aborting.stop(false);
    const auto aborted = aborting.stats();
    return require(!aborted.failed && aborted.blocks_written == 0,
                   "abort is not reported as a writer failure") &&
           require(aborted.blocks_accepted == 2 && aborted.dropped_blocks == 2 &&
                       aborted.dropped_bytes == first.size() + second.size() &&
                       aborted.queued_bytes == 0,
                   "abort accounts in-flight and queued blocks exactly once");
}

bool test_blocked_output_drops_and_remains_cancellable() {
    std::array<int, 2> pipe_fds{};
    if (!require(::pipe(pipe_fds.data()) == 0,
                 "blocked-output pipe is created")) {
        return false;
    }
    const int original_flags = ::fcntl(pipe_fds[1], F_GETFL);
    if (!require(
            original_flags >= 0 &&
                ::fcntl(pipe_fds[1], F_SETFL, original_flags | O_NONBLOCK) == 0,
            "pipe can be filled without blocking")) {
        static_cast<void>(::close(pipe_fds[0]));
        static_cast<void>(::close(pipe_fds[1]));
        return false;
    }
    const std::vector<std::uint8_t> filler(4096, 0x00);
    while (::write(pipe_fds[1], filler.data(), filler.size()) > 0) {
    }

    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 1024,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::drop_oldest,
        .criticality = airspy_tv::TransportSinkCriticality::required,
        .thread_name = "ts-out-test",
    });
    std::string error;
    if (!require(output.start_fd(pipe_fds[1], true, "blocked pipe", error),
                 "blocked pipe output starts")) {
        static_cast<void>(::close(pipe_fds[0]));
        return false;
    }

    const std::vector<std::uint8_t> block(256, 0x47);
    for (int index = 0; index < 64; ++index) {
        static_cast<void>(output.submit(block));
    }
    for (int attempt = 0; attempt < 100 && output.stats().dropped_blocks == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto blocked_stats = output.stats();
    const auto stop_started = std::chrono::steady_clock::now();
    output.stop(false);
    const auto stop_elapsed = std::chrono::steady_clock::now() - stop_started;
    static_cast<void>(::close(pipe_fds[0]));

    return require(blocked_stats.active && !blocked_stats.failed,
                   "blocked drop-oldest output remains active") &&
           require(blocked_stats.dropped_blocks != 0,
                   "blocked output drops queued blocks") &&
           require(stop_elapsed < std::chrono::seconds(1),
                   "blocked output cancellation does not hang") &&
           require(output.stats().blocks_accepted ==
                       output.stats().blocks_written +
                           output.stats().dropped_blocks,
                   "abort stop accounts every accepted logical block");
}

bool test_blocking_output_applies_backpressure() {
    std::array<int, 2> pipe_fds{};
    if (!require(::pipe(pipe_fds.data()) == 0,
                 "blocking-output pipe is created")) {
        return false;
    }

    const int original_flags = ::fcntl(pipe_fds[1], F_GETFL);
    if (original_flags < 0 ||
        ::fcntl(pipe_fds[1], F_SETFL, original_flags | O_NONBLOCK) < 0) {
        static_cast<void>(::close(pipe_fds[0]));
        static_cast<void>(::close(pipe_fds[1]));
        return false;
    }
    const std::vector<std::uint8_t> filler(4096, 0x00);
    while (::write(pipe_fds[1], filler.data(), filler.size()) > 0) {
    }

    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 188,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::block_producer,
        .criticality = airspy_tv::TransportSinkCriticality::required,
        .thread_name = "ts-out-test",
    });
    std::string error;
    if (!require(output.start_fd(pipe_fds[1], true, "blocking pipe", error),
                 "blocking pipe output starts")) {
        static_cast<void>(::close(pipe_fds[0]));
        return false;
    }

    const std::vector<std::uint8_t> packet(188, 0x47);
    if (!require(output.submit(packet), "first blocking packet is accepted")) {
        static_cast<void>(::close(pipe_fds[0]));
        return false;
    }
    if (!require(output.submit(packet), "second blocking packet is accepted")) {
        static_cast<void>(::close(pipe_fds[0]));
        return false;
    }

    std::atomic_bool producer_finished{};
    std::atomic_bool third_accepted{};
    std::thread producer([&] {
        third_accepted = output.submit(packet);
        producer_finished = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const bool producer_blocked = !producer_finished.load();
    std::array<std::uint8_t, 8192> drained{};
    static_cast<void>(::read(pipe_fds[0], drained.data(), drained.size()));
    for (int attempt = 0; attempt < 100 && !producer_finished.load();
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    output.stop(false);
    producer.join();
    static_cast<void>(::close(pipe_fds[0]));
    const auto stats = output.stats();
    return require(producer_blocked,
                   "full blocking queue applies producer backpressure") &&
           require(third_accepted.load(),
                   "blocked producer resumes when queue space is available") &&
           require(stats.blocks_accepted == 3,
                   "blocking output accepts every producer block") &&
           require(stats.queue_capacity_bytes == packet.size(),
                   "blocking output reports its queue capacity");
}

bool test_discard_queued_is_sink_local() {
    std::array<int, 2> pipe_fds{};
    if (!require(::pipe(pipe_fds.data()) == 0,
                 "discard-output pipe is created")) {
        return false;
    }
    const int original_flags = ::fcntl(pipe_fds[1], F_GETFL);
    if (original_flags < 0 ||
        ::fcntl(pipe_fds[1], F_SETFL, original_flags | O_NONBLOCK) < 0) {
        static_cast<void>(::close(pipe_fds[0]));
        static_cast<void>(::close(pipe_fds[1]));
        return false;
    }
    const std::vector<std::uint8_t> filler(4096, 0x00);
    while (::write(pipe_fds[1], filler.data(), filler.size()) > 0) {
    }

    airspy_tv::AsyncTransportOutput output({
        .queue_capacity_bytes = 1024,
        .overflow_policy = airspy_tv::TransportOverflowPolicy::drop_oldest,
        .criticality = airspy_tv::TransportSinkCriticality::optional,
        .thread_name = "ts-out-test",
    });
    std::string error;
    if (!require(output.start_fd(pipe_fds[1], true, "discard pipe", error),
                 "discard pipe output starts")) {
        static_cast<void>(::close(pipe_fds[0]));
        return false;
    }
    const std::vector<std::uint8_t> block(256, 0x47);
    for (int index = 0; index < 4; ++index) {
        static_cast<void>(output.submit(block));
    }
    output.discard_queued();
    const auto discarded = output.stats();
    output.stop(false);
    static_cast<void>(::close(pipe_fds[0]));
    return require(discarded.dropped_blocks != 0,
                   "discard records queued blocks as dropped") &&
           require(discarded.queued_bytes <= block.size(),
                   "discard leaves at most the in-flight block");
}

} // namespace

int main() {
    const bool ok = test_ordered_file_round_trip_and_drain() &&
                    test_required_overflow_fails_sink() &&
                    test_optional_overflow_is_sink_local() &&
                    test_write_failure_is_reported() &&
                    test_deterministic_partial_write_sequence() &&
                    test_deterministic_write_failure_and_cancel() &&
                    test_scripted_writer_core_drain_failure_and_abort() &&
                    test_blocked_output_drops_and_remains_cancellable() &&
                    test_blocking_output_applies_backpressure() &&
                    test_discard_queued_is_sink_local();
    return ok ? 0 : 1;
}
