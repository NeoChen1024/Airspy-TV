#include "airspy_tv/transport_output.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
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
           require(!stats.error.empty(), "write failure includes an error");
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
                   "blocked output cancellation does not hang");
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
                    test_blocked_output_drops_and_remains_cancellable() &&
                    test_blocking_output_applies_backpressure() &&
                    test_discard_queued_is_sink_local();
    return ok ? 0 : 1;
}
