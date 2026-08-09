#include "airspy_tv/recorder.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

class TemporaryDirectory {
  public:
    TemporaryDirectory() {
        std::string pattern = "/tmp/airspy-tv-recorder-XXXXXX";
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

std::vector<std::uint8_t> read_bytes(const std::filesystem::path &path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()};
}

airspy_tv::RecordingMetadata metadata() {
    return {.source = "test",
            .center_frequency_hz = 545'000'000,
            .sample_rate_hz = 10'000'000};
}

bool test_raw_round_trip_and_pending_drain() {
    const TemporaryDirectory directory;
    const auto path = directory.path / "capture.cs16";
    airspy_tv::RawIqRecorder recorder({.queue_capacity_samples = 32});
    std::string error;
    if (!require(recorder.start(path, metadata(), error),
                 "raw recorder starts: " + error)) {
        return false;
    }

    const std::vector<std::int16_t> first{1, -1, 2, -2, 3, -3, 4, -4};
    const std::vector<std::int16_t> second{5, -5, 6, -6};
    recorder.submit(first);
    recorder.submit(second);
    recorder.add_source_dropped_samples(7);
    recorder.stop();

    std::vector<std::uint8_t> expected((first.size() * sizeof(std::int16_t)) +
                                       (second.size() * sizeof(std::int16_t)));
    std::size_t offset = 0;
    for (const auto block : {std::span(first), std::span(second)}) {
        const auto *bytes =
            reinterpret_cast<const std::uint8_t *>(block.data());
        std::memcpy(expected.data() + offset, bytes, block.size_bytes());
        offset += block.size_bytes();
    }

    const auto stats = recorder.stats();
    std::ifstream sidecar(path.string() + ".json");
    nlohmann::json sidecar_json;
    sidecar >> sidecar_json;
    return require(!stats.active && !stats.failed,
                   "drained raw recorder stops cleanly") &&
           require(stats.bytes_written == expected.size() &&
                       stats.complex_samples == 6,
                   "raw recorder reports drained bytes and samples") &&
           require(stats.source_dropped_samples == 7,
                   "raw recorder preserves source drop count") &&
           require(read_bytes(path) == expected,
                   "raw recorder preserves submitted byte order") &&
           require(sidecar_json.at("complex_samples") == 6 &&
                       !sidecar_json.at("failed").get<bool>(),
                   "raw sidecar records final writer state");
}

bool test_raw_overflow_and_open_failure() {
    const TemporaryDirectory directory;
    airspy_tv::RawIqRecorder recorder({.queue_capacity_samples = 2});
    std::string error;
    if (!require(!recorder.start(directory.path / "missing" / "capture.cs16",
                                 metadata(), error) &&
                     !error.empty(),
                 "raw recorder reports filesystem open failure")) {
        return false;
    }

    error.clear();
    const auto path = directory.path / "bounded.cs16";
    if (!require(recorder.start(path, metadata(), error),
                 "bounded raw recorder starts")) {
        return false;
    }
    const std::vector<std::int16_t> oversized{1, -1, 2, -2, 3, -3};
    recorder.submit(oversized);
    recorder.stop();
    const auto stats = recorder.stats();
    return require(!stats.failed && stats.dropped_blocks == 1,
                   "raw recorder drops a block larger than its bound") &&
           require(stats.bytes_written == 0 && read_bytes(path).empty(),
                   "dropped raw block is not partially written");
}

bool test_raw_write_failure() {
    if (!std::filesystem::exists("/dev/full")) {
        return true;
    }
    airspy_tv::RawIqRecorder recorder({.queue_capacity_samples = 16});
    std::string error;
    if (!require(recorder.start("/dev/full", metadata(), error),
                 "/dev/full raw recorder starts")) {
        return false;
    }
    const std::vector<std::int16_t> block{1, -1, 2, -2};
    recorder.submit(block);
    for (int attempt = 0; attempt < 100 && !recorder.stats().failed;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    recorder.stop();
    const auto stats = recorder.stats();
    return require(stats.failed && !stats.active,
                   "raw writer failure stops the recorder") &&
           require(stats.write_errors == 1 && !stats.error.empty(),
                   "raw writer failure is observable");
}

bool test_transport_recorder_adapter() {
    const TemporaryDirectory directory;
    const auto path = directory.path / "capture.ts";
    airspy_tv::TransportStreamRecorder recorder;
    std::string error;
    if (!require(recorder.start(path, error), "TS recorder starts: " + error)) {
        return false;
    }
    const std::vector<std::uint8_t> packet(188, 0x47);
    recorder.submit(packet);
    recorder.stop();
    const auto stats = recorder.stats();
    if (!require(!stats.failed && stats.bytes_written == packet.size() &&
                     read_bytes(path) == packet,
                 "TS recorder adapter drains ordered output")) {
        return false;
    }

    if (!std::filesystem::exists("/dev/full")) {
        return true;
    }
    error.clear();
    if (!require(recorder.start("/dev/full", error),
                 "/dev/full TS recorder starts")) {
        return false;
    }
    recorder.submit(packet);
    for (int attempt = 0; attempt < 100 && !recorder.stats().failed;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    recorder.stop();
    const auto failed = recorder.stats();
    return require(failed.failed && failed.write_errors == 1 &&
                       !failed.error.empty(),
                   "TS recorder exposes the transport writer failure");
}

} // namespace

int main() {
    try {
        const bool ok = test_raw_round_trip_and_pending_drain() &&
                        test_raw_overflow_and_open_failure() &&
                        test_raw_write_failure() &&
                        test_transport_recorder_adapter();
        return ok ? 0 : 1;
    } catch (const std::exception &exception) {
        std::cerr << "Recorder test failed: " << exception.what() << '\n';
        return 1;
    }
}
