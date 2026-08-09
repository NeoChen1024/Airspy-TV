#include "airspy_tv/demodulator.hpp"
#include "airspy_tv/sdr.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

class TemporaryDirectory {
  public:
    TemporaryDirectory() {
        std::string pattern = "/tmp/airspy-tv-source-XXXXXX";
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

class StdinPipe {
  public:
    StdinPipe() {
        std::array<int, 2> descriptors{};
        if (::pipe(descriptors.data()) != 0) {
            return;
        }
        saved_stdin_ = ::dup(STDIN_FILENO);
        if (saved_stdin_ < 0 || ::dup2(descriptors[0], STDIN_FILENO) < 0) {
            ::close(descriptors[0]);
            ::close(descriptors[1]);
            if (saved_stdin_ >= 0) {
                ::close(saved_stdin_);
                saved_stdin_ = -1;
            }
            return;
        }
        ::close(descriptors[0]);
        writer_ = descriptors[1];
    }

    ~StdinPipe() {
        close_writer();
        if (saved_stdin_ >= 0) {
            static_cast<void>(::dup2(saved_stdin_, STDIN_FILENO));
            ::close(saved_stdin_);
        }
    }

    StdinPipe(const StdinPipe &) = delete;
    StdinPipe &operator=(const StdinPipe &) = delete;
    StdinPipe(StdinPipe &&) = delete;
    StdinPipe &operator=(StdinPipe &&) = delete;

    [[nodiscard]] bool valid() const noexcept {
        return saved_stdin_ >= 0 && writer_ >= 0;
    }

    [[nodiscard]] bool
    write_all(const std::span<const std::int16_t> samples) const {
        const auto *data = reinterpret_cast<const char *>(samples.data());
        std::size_t remaining = samples.size_bytes();
        while (remaining != 0) {
            const ssize_t written = ::write(writer_, data, remaining);
            if (written < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return false;
            }
            data += written;
            remaining -= static_cast<std::size_t>(written);
        }
        return true;
    }

    void close_writer() noexcept {
        if (writer_ >= 0) {
            ::close(writer_);
            writer_ = -1;
        }
    }

  private:
    int saved_stdin_{-1};
    int writer_{-1};
};

class ProbeDemodulator final : public airspy_tv::Demodulator {
  public:
    void request_reset() override { ++requested_resets; }
    void reset() override { ++resets; }

    void submit(const std::span<const std::int16_t> interleaved_iq,
                const std::uint32_t sample_rate_hz,
                const std::uint32_t channel_bandwidth_hz,
                const airspy_tv::InputSampleStamp stamp) override {
        ++nonblocking_submits;
        record_submit(interleaved_iq, sample_rate_hz, channel_bandwidth_hz,
                      stamp);
    }

    void submit_blocking(const std::span<const std::int16_t> interleaved_iq,
                         const std::uint32_t sample_rate_hz,
                         const std::uint32_t channel_bandwidth_hz,
                         const airspy_tv::InputSampleStamp stamp) override {
        ++blocking_submits;
        record_submit(interleaved_iq, sample_rate_hz, channel_bandwidth_hz,
                      stamp);
    }

    void record_submit(const std::span<const std::int16_t> interleaved_iq,
                       const std::uint32_t sample_rate_hz,
                       const std::uint32_t channel_bandwidth_hz,
                       const airspy_tv::InputSampleStamp stamp) {
        static_cast<void>(sample_rate_hz);
        static_cast<void>(channel_bandwidth_hz);
        const std::scoped_lock lock(mutex);
        samples += interleaved_iq.size() / 2;
        stamps.push_back(stamp);
    }

    void flush() override { ++flushes; }
    void wait_until_idle() override {}
    void set_transport_callback(TransportCallback callback) override {
        transport_callback = std::move(callback);
    }
    void set_discontinuity_callback(DiscontinuityCallback callback) override {
        discontinuity_callback = std::move(callback);
    }
    void set_signal_smoothing(const bool enabled, const int speed) override {
        static_cast<void>(enabled);
        static_cast<void>(speed);
    }
    [[nodiscard]] airspy_tv::SignalSnapshot signal_snapshot() const override {
        return {};
    }
    [[nodiscard]] airspy_tv::PipelineSnapshot
    pipeline_snapshot() const override {
        return {};
    }

    [[nodiscard]] std::vector<airspy_tv::InputSampleStamp>
    stamp_snapshot() const {
        const std::scoped_lock lock(mutex);
        return stamps;
    }

    [[nodiscard]] std::size_t sample_count() const {
        const std::scoped_lock lock(mutex);
        return samples;
    }

    mutable std::mutex mutex;
    std::size_t samples{};
    std::vector<airspy_tv::InputSampleStamp> stamps;
    std::atomic<std::uint64_t> requested_resets;
    std::atomic<std::uint64_t> resets;
    std::atomic<std::uint64_t> flushes;
    std::atomic<std::uint64_t> nonblocking_submits;
    std::atomic<std::uint64_t> blocking_submits;
    TransportCallback transport_callback;
    DiscontinuityCallback discontinuity_callback;
};

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool wait_for_exhaustion(const airspy_tv::SdrDevice &device) {
    for (int attempt = 0; attempt < 200 && !device.input_exhausted();
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return device.input_exhausted();
}

template <typename Predicate> bool wait_until(Predicate predicate) {
    for (int attempt = 0; attempt < 200 && !predicate(); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return predicate();
}

bool test_file_source_pipeline() {
    const TemporaryDirectory directory;
    const auto path = directory.path / "source.cs16";
    const std::vector<std::int16_t> iq{1, -1, 2, -2, 3, -3, 4, -4,
                                       5, -5, 6, -6, 7, -7, 8, -8};
    {
        std::ofstream output(path, std::ios::binary);
        output.write(
            reinterpret_cast<const char *>(iq.data()),
            static_cast<std::streamsize>(iq.size() * sizeof(std::int16_t)));
    }

    airspy_tv::SdrDevice device;
    auto demodulator = std::make_unique<ProbeDemodulator>();
    const ProbeDemodulator *probe = demodulator.get();
    device.set_demodulator(std::move(demodulator));
    device.set_display_analysis_enabled(false);

    airspy_tv::SourceSettings settings;
    settings.sample_rate_hz = 1'000'000;
    std::string error;
    if (!require(device.open_iq_file(path, settings, error),
                 "file source opens: " + error) ||
        !require(device.start_stream(settings, error),
                 "file source starts: " + error) ||
        !require(wait_for_exhaustion(device), "file source reaches EOF")) {
        return false;
    }

    auto stamps = probe->stamp_snapshot();
    const auto first_timeline = device.input_timeline_snapshot();
    if (!require(stamps.size() == 1 && stamps.front().stream_epoch == 1 &&
                     stamps.front().begin_sample == 0 &&
                     stamps.front().sample_count == 8 &&
                     stamps.front().discontinuity_before,
                 "first replay has one epoch-stamped source block") ||
        !require(first_timeline.stream_epoch == 1 &&
                     first_timeline.source_head_sample == 8 &&
                     first_timeline.delivered_samples == 8,
                 "first replay advances the shared source timeline") ||
        !require(probe->flushes.load() == 1,
                 "finite file source flushes the demodulator")) {
        return false;
    }

    if (!require(device.start_stream(settings, error),
                 "same file source restarts: " + error) ||
        !require(wait_for_exhaustion(device), "second replay reaches EOF")) {
        return false;
    }
    stamps = probe->stamp_snapshot();
    const auto second_timeline = device.input_timeline_snapshot();
    const auto spectrum = device.spectrum_snapshot();
    device.stop_stream();
    device.close();

    return require(stamps.size() == 2 && stamps.back().stream_epoch == 2 &&
                       stamps.back().begin_sample == 8 &&
                       stamps.back().discontinuity_before,
                   "replay generation changes without rewinding source time") &&
           require(second_timeline.stream_epoch == 2 &&
                       second_timeline.source_head_sample == 16 &&
                       second_timeline.delivered_samples == 16,
                   "second replay preserves monotonic source coordinates") &&
           require(probe->flushes.load() == 2,
                   "each finite replay flushes exactly once") &&
           require(probe->nonblocking_submits.load() == 2 &&
                       probe->blocking_submits.load() == 0,
                   "default playback uses nonblocking decoder submission") &&
           require(
               !spectrum.valid,
               "disabled display analysis does not produce spectrum work") &&
           require(device.descriptor() == nullptr,
                   "closing the pipeline releases source ownership");
}

bool test_stdin_source_pipeline() {
    StdinPipe input;
    const std::vector<std::int16_t> iq{11, -11, 12, -12, 13, -13, 14, -14};
    if (!require(input.valid(), "stdin test pipe is available") ||
        !require(input.write_all(iq), "stdin fixture is written")) {
        return false;
    }
    input.close_writer();

    airspy_tv::SdrDevice device;
    auto demodulator = std::make_unique<ProbeDemodulator>();
    const ProbeDemodulator *probe = demodulator.get();
    device.set_demodulator(std::move(demodulator));
    device.set_display_analysis_enabled(false);

    airspy_tv::SourceSettings settings;
    settings.sample_rate_hz = 1'000'000;
    std::string error;
    if (!require(device.open_iq_file("-", settings, error),
                 "stdin source opens: " + error) ||
        !require(device.start_stream(settings, error),
                 "stdin source starts: " + error) ||
        !require(wait_for_exhaustion(device), "stdin source reaches EOF")) {
        return false;
    }

    const auto stamps = probe->stamp_snapshot();
    const bool result =
        require(stamps.size() == 1 && stamps.front().sample_count == 4 &&
                    stamps.front().stream_epoch == 1,
                "stdin samples use the common timeline") &&
        require(probe->sample_count() == 4,
                "stdin samples reach the demodulator") &&
        require(probe->flushes.load() == 1,
                "stdin EOF flushes the demodulator") &&
        require(device.runtime_error().empty(),
                "stdin EOF is not reported as an error");
    device.close();
    return result;
}

bool test_file_runtime_failure_is_restartable() {
    const TemporaryDirectory directory;
    const auto path = directory.path / "restart.cs16";
    const std::vector<std::int16_t> iq{1, -1, 2, -2};
    {
        std::ofstream output(path, std::ios::binary);
        output.write(
            reinterpret_cast<const char *>(iq.data()),
            static_cast<std::streamsize>(iq.size() * sizeof(iq.front())));
    }

    airspy_tv::SdrDevice device;
    auto demodulator = std::make_unique<ProbeDemodulator>();
    const ProbeDemodulator *probe = demodulator.get();
    device.set_demodulator(std::move(demodulator));

    airspy_tv::SourceSettings settings;
    settings.sample_rate_hz = 1'000'000;
    std::string error;
    if (!require(device.open_iq_file(path, settings, error),
                 "file source opens before removal: " + error)) {
        return false;
    }
    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    if (!require(!remove_error, "opened fixture can be removed") ||
        !require(device.start_stream(settings, error),
                 "asynchronous file source starts: " + error) ||
        !require(wait_until([&device] { return !device.is_streaming(); }),
                 "missing file stops its worker") ||
        !require(device.runtime_error().find("Unable to read I/Q file") !=
                     std::string::npos,
                 "missing file reports a runtime error")) {
        return false;
    }

    {
        std::ofstream output(path, std::ios::binary);
        output.write(
            reinterpret_cast<const char *>(iq.data()),
            static_cast<std::streamsize>(iq.size() * sizeof(iq.front())));
    }
    if (!require(device.start_stream(settings, error),
                 "file source restarts after runtime failure: " + error) ||
        !require(wait_for_exhaustion(device),
                 "restarted source reaches finite EOF")) {
        return false;
    }

    const bool result = require(device.runtime_error().empty(),
                                "restart clears the previous runtime error") &&
                        require(probe->sample_count() == 2,
                                "restarted source delivers its samples") &&
                        require(probe->flushes.load() == 1,
                                "only the successful finite input flushes");
    device.close();
    return result;
}

bool test_stop_interrupts_file_pacing() {
    const TemporaryDirectory directory;
    const auto path = directory.path / "slow.cs16";
    const std::vector<std::int16_t> iq(64, 1);
    {
        std::ofstream output(path, std::ios::binary);
        output.write(
            reinterpret_cast<const char *>(iq.data()),
            static_cast<std::streamsize>(iq.size() * sizeof(iq.front())));
    }

    airspy_tv::SdrDevice device;
    auto demodulator = std::make_unique<ProbeDemodulator>();
    const ProbeDemodulator *probe = demodulator.get();
    device.set_demodulator(std::move(demodulator));

    airspy_tv::SourceSettings settings;
    settings.sample_rate_hz = 1;
    std::string error;
    if (!require(device.open_iq_file(path, settings, error),
                 "slow file source opens: " + error) ||
        !require(device.start_stream(settings, error),
                 "slow file source starts: " + error) ||
        !require(wait_until([probe] { return probe->sample_count() != 0; }),
                 "slow source enters pacing")) {
        return false;
    }

    const auto started_at = std::chrono::steady_clock::now();
    device.stop_stream();
    const auto stop_time = std::chrono::steady_clock::now() - started_at;
    return require(stop_time < std::chrono::milliseconds(250),
                   "stop interrupts file pacing promptly") &&
           require(!device.input_exhausted(),
                   "manually stopped input is not classified as EOF");
}

bool test_unpaced_blocking_playback_policy() {
    const TemporaryDirectory directory;
    const auto path = directory.path / "offline.cs16";
    const std::vector<std::int16_t> iq(64, 2);
    {
        std::ofstream output(path, std::ios::binary);
        output.write(
            reinterpret_cast<const char *>(iq.data()),
            static_cast<std::streamsize>(iq.size() * sizeof(iq.front())));
    }

    airspy_tv::SdrDevice device;
    auto demodulator = std::make_unique<ProbeDemodulator>();
    const ProbeDemodulator *probe = demodulator.get();
    device.set_demodulator(std::move(demodulator));

    airspy_tv::SourceSettings settings;
    settings.sample_rate_hz = 1;
    constexpr airspy_tv::IqPlaybackPolicy policy{
        .pacing = airspy_tv::IqPlaybackPacing::unpaced,
        .decoder_backpressure = airspy_tv::DecoderBackpressurePolicy::block,
    };
    std::string error;
    const auto started_at = std::chrono::steady_clock::now();
    if (!require(device.open_iq_file(path, settings, policy, error),
                 "offline-policy file source opens: " + error) ||
        !require(device.start_stream(settings, error),
                 "offline-policy file source starts: " + error) ||
        !require(wait_for_exhaustion(device),
                 "unpaced source reaches EOF without wall-clock pacing")) {
        return false;
    }
    const auto elapsed = std::chrono::steady_clock::now() - started_at;
    const bool result =
        require(elapsed < std::chrono::milliseconds(250),
                "unpaced policy bypasses the 32-second source duration") &&
        require(probe->blocking_submits.load() == 1 &&
                    probe->nonblocking_submits.load() == 0,
                "offline policy uses blocking decoder submission");
    device.close();
    return result;
}

} // namespace

int main() {
    try {
        return test_file_source_pipeline() && test_stdin_source_pipeline() &&
                       test_file_runtime_failure_is_restartable() &&
                       test_stop_interrupts_file_pacing() &&
                       test_unpaced_blocking_playback_policy()
                   ? 0
                   : 1;
    } catch (const std::exception &exception) {
        std::cerr << "SDR file-source test failed: " << exception.what()
                  << '\n';
        return 1;
    }
}
