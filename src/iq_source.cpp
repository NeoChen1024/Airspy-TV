#include "iq_source.hpp"

#include "airspy_tv/iq_file.hpp"
#include "airspy_tv/thread_name.hpp"

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <format>
#include <fstream>
#include <iomanip>
#include <libairspy/airspy.h>
#include <limits>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <utility>

namespace airspy_tv {
namespace {

constexpr std::size_t max_airspy_devices = 32;
constexpr std::size_t source_block_samples = 32'768;

std::string format_serial(const std::uint64_t serial) {
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setw(16) << std::setfill('0')
           << serial;
    return output.str();
}

std::string format_airspy_error(const std::string_view operation,
                                const int code) {
    return std::string(operation) + ": " +
           airspy_error_name(static_cast<airspy_error>(code)) + " (" +
           std::to_string(code) + ")";
}

std::string argument_value(const std::map<std::string, std::string> &arguments,
                           const std::string_view key) {
    const auto item = arguments.find(std::string(key));
    return item == arguments.end() ? std::string{} : item->second;
}

std::uint64_t corrected_frequency(const std::uint64_t nominal_hz,
                                  const double ppm) noexcept {
    const long double factor =
        1.0L + (static_cast<long double>(ppm) / 1'000'000.0L);
    const long double corrected = static_cast<long double>(nominal_hz) * factor;
    return corrected <= 0.0L
               ? 0
               : static_cast<std::uint64_t>(std::round(corrected));
}

struct AirspyDeviceCloser {
    void operator()(airspy_device *device) const noexcept {
        if (device != nullptr) {
            static_cast<void>(airspy_close(device));
        }
    }
};

struct SoapyDeviceCloser {
    void operator()(SoapySDR::Device *device) const noexcept {
        if (device == nullptr) {
            return;
        }
        try {
            SoapySDR::Device::unmake(device);
        } catch (...) { // NOLINT(bugprone-empty-catch)
        }
    }
};

class SourceBase : public IqSource {
  public:
    [[nodiscard]] const DeviceDescriptor &descriptor() const noexcept override {
        return descriptor_;
    }

    [[nodiscard]] const std::vector<std::uint32_t> &
    sample_rates() const noexcept override {
        return sample_rates_;
    }

    [[nodiscard]] std::optional<std::pair<double, double>>
    gain_range() const noexcept override {
        return gain_range_;
    }

    [[nodiscard]] bool is_streaming() const noexcept override {
        return streaming_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool input_exhausted() const noexcept override {
        return input_exhausted_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::string runtime_error() const override {
        const std::scoped_lock lock(error_mutex_);
        return runtime_error_;
    }

  protected:
    SourceBase(DeviceDescriptor descriptor, std::vector<std::uint32_t> rates,
               std::optional<std::pair<double, double>> gain_range = {})
        : descriptor_(std::move(descriptor)), sample_rates_(std::move(rates)),
          gain_range_(std::move(gain_range)) {}

    void begin(IqSourceCallbacks callbacks) {
        callbacks_ = std::move(callbacks);
        input_exhausted_.store(false, std::memory_order_relaxed);
        streaming_.store(true, std::memory_order_relaxed);
        const std::scoped_lock lock(error_mutex_);
        runtime_error_.clear();
    }

    void set_error(std::string message) {
        const std::scoped_lock lock(error_mutex_);
        runtime_error_ = std::move(message);
    }

    [[nodiscard]] IqSourceCallbacks &callbacks() noexcept { return callbacks_; }

    void mark_input_exhausted() noexcept {
        input_exhausted_.store(true, std::memory_order_relaxed);
    }

    [[nodiscard]] bool stop_streaming() noexcept {
        return streaming_.exchange(false, std::memory_order_relaxed);
    }

  private:
    DeviceDescriptor descriptor_;
    std::vector<std::uint32_t> sample_rates_;
    std::optional<std::pair<double, double>> gain_range_;
    IqSourceCallbacks callbacks_;
    std::atomic<bool> streaming_{false};
    std::atomic<bool> input_exhausted_{false};
    mutable std::mutex error_mutex_;
    std::string runtime_error_;
};

class AirspySource final : public SourceBase {
  public:
    AirspySource(DeviceDescriptor descriptor, airspy_device *device,
                 std::vector<std::uint32_t> rates)
        : SourceBase(std::move(descriptor), std::move(rates)), device_(device) {
    }

    ~AirspySource() override {
        stop();
        if (device_ != nullptr) {
            airspy_close(device_);
        }
    }

    AirspySource(const AirspySource &) = delete;
    AirspySource &operator=(const AirspySource &) = delete;
    AirspySource(AirspySource &&) = delete;
    AirspySource &operator=(AirspySource &&) = delete;

    [[nodiscard]] bool has_active_worker() const noexcept override {
        return is_streaming();
    }

    bool configure(const SourceSettings &settings,
                   std::string &error) override {
        const auto frequency = corrected_frequency(
            settings.center_frequency_hz, settings.frequency_correction_ppm);
        if (frequency > std::numeric_limits<std::uint32_t>::max()) {
            error = "Airspy center frequency is out of range";
            return false;
        }
        const auto run = [&](const std::string_view name, const int result) {
            if (result == AIRSPY_SUCCESS) {
                return true;
            }
            error = format_airspy_error(name, result);
            return false;
        };
        if (!run("airspy_set_sample_type",
                 airspy_set_sample_type(device_, AIRSPY_SAMPLE_INT16_IQ)) ||
            !run("airspy_set_samplerate",
                 airspy_set_samplerate(device_, settings.sample_rate_hz)) ||
            !run("airspy_set_freq",
                 airspy_set_freq(device_,
                                 static_cast<std::uint32_t>(frequency)))) {
            return false;
        }
        return set_bias_tee(settings.bias_tee, error) &&
               set_gain(settings, error);
    }

    bool start(IqSourceCallbacks callbacks, std::string &error) override {
        begin(std::move(callbacks));
        const int result =
            airspy_start_rx(device_, &AirspySource::receive, this);
        if (result == AIRSPY_SUCCESS) {
            return true;
        }
        static_cast<void>(stop_streaming());
        error = format_airspy_error("airspy_start_rx", result);
        return false;
    }

    void stop() noexcept override {
        if (stop_streaming()) {
            static_cast<void>(airspy_stop_rx(device_));
        }
    }

    bool retune(const std::uint64_t frequency_hz, const double correction_ppm,
                std::string &error) override {
        const auto frequency =
            corrected_frequency(frequency_hz, correction_ppm);
        if (frequency > std::numeric_limits<std::uint32_t>::max()) {
            error = "Airspy center frequency is out of range";
            return false;
        }
        const int result =
            airspy_set_freq(device_, static_cast<std::uint32_t>(frequency));
        if (result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_set_freq", result);
            return false;
        }
        return true;
    }

    bool set_gain(const SourceSettings &settings, std::string &error) override {
        const auto gain =
            static_cast<std::uint8_t>(std::clamp(settings.airspy_gain, 0, 21));
        const int result =
            settings.airspy_gain_mode == AirspyGainMode::Sensitivity
                ? airspy_set_sensitivity_gain(device_, gain)
                : airspy_set_linearity_gain(device_, gain);
        if (result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_set_gain_profile", result);
            return false;
        }
        return true;
    }

    bool set_bias_tee(const bool enabled, std::string &error) override {
        const int result =
            airspy_set_rf_bias(device_, static_cast<std::uint8_t>(enabled));
        if (result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_set_rf_bias", result);
            return false;
        }
        return true;
    }

  private:
    static int receive(airspy_transfer *transfer) {
        auto *self = static_cast<AirspySource *>(transfer->ctx);
        if (self == nullptr || !self->is_streaming() ||
            transfer->samples == nullptr || transfer->sample_count <= 0) {
            return 0;
        }
        if (transfer->dropped_samples != 0 && self->callbacks().discontinuity) {
            self->callbacks().discontinuity(
                static_cast<std::uint64_t>(transfer->dropped_samples));
        }
        if (self->callbacks().samples) {
            const auto *samples =
                static_cast<const std::int16_t *>(transfer->samples);
            self->callbacks().samples(std::span(
                samples, static_cast<std::size_t>(transfer->sample_count) * 2));
        }
        return 0;
    }

    airspy_device *device_{};
};

class SoapySource final : public SourceBase {
  public:
    SoapySource(DeviceDescriptor descriptor, SoapySDR::Device *device,
                std::vector<std::uint32_t> rates,
                std::optional<std::pair<double, double>> gain_range)
        : SourceBase(std::move(descriptor), std::move(rates),
                     std::move(gain_range)),
          device_(device) {}

    ~SoapySource() override {
        stop();
        SoapyDeviceCloser{}(device_);
    }

    SoapySource(const SoapySource &) = delete;
    SoapySource &operator=(const SoapySource &) = delete;
    SoapySource(SoapySource &&) = delete;
    SoapySource &operator=(SoapySource &&) = delete;

    [[nodiscard]] bool has_active_worker() const noexcept override {
        return worker_.joinable();
    }

    bool configure(const SourceSettings &settings,
                   std::string &error) override {
        try {
            device_->setSampleRate(SOAPY_SDR_RX, 0, settings.sample_rate_hz);
            device_->setFrequency(SOAPY_SDR_RX, 0,
                                  static_cast<double>(corrected_frequency(
                                      settings.center_frequency_hz,
                                      settings.frequency_correction_ppm)));
            return set_gain(settings, error);
        } catch (const std::exception &exception) {
            error = std::string("SoapySDR configure: ") + exception.what();
            return false;
        }
    }

    bool start(IqSourceCallbacks callbacks, std::string &error) override {
        try {
            stream_ = device_->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
            if (stream_ == nullptr) {
                throw std::runtime_error("setupStream returned null");
            }
            const int result = device_->activateStream(stream_);
            if (result != 0) {
                throw std::runtime_error(std::string("activateStream: ") +
                                         SoapySDR::errToStr(result));
            }
            stream_active_ = true;
            begin(std::move(callbacks));
            worker_ = std::thread([this] { run(); });
            return true;
        } catch (const std::exception &exception) {
            error = std::string("SoapySDR start stream: ") + exception.what();
            stop();
            return false;
        }
    }

    void stop() noexcept override {
        static_cast<void>(stop_streaming());
        if (worker_.joinable()) {
            worker_.join();
        }
        if (stream_ != nullptr) {
            if (stream_active_) {
                try {
                    static_cast<void>(device_->deactivateStream(stream_));
                } catch (...) { // NOLINT(bugprone-empty-catch)
                }
            }
            try {
                device_->closeStream(stream_);
            } catch (...) { // NOLINT(bugprone-empty-catch)
            }
            stream_active_ = false;
            stream_ = nullptr;
        }
    }

    bool retune(const std::uint64_t frequency_hz, const double correction_ppm,
                std::string &error) override {
        try {
            device_->setFrequency(SOAPY_SDR_RX, 0,
                                  static_cast<double>(corrected_frequency(
                                      frequency_hz, correction_ppm)));
            return true;
        } catch (const std::exception &exception) {
            error = std::string("SoapySDR set center frequency: ") +
                    exception.what();
            return false;
        }
    }

    bool set_gain(const SourceSettings &settings, std::string &error) override {
        const auto range = gain_range();
        if (!range.has_value()) {
            return true;
        }
        try {
            const auto [minimum, maximum] = *range;
            device_->setGain(SOAPY_SDR_RX, 0,
                             std::clamp(settings.soapy_gain, minimum, maximum));
            return true;
        } catch (const std::exception &exception) {
            error = std::string("SoapySDR set gain: ") + exception.what();
            return false;
        }
    }

    bool set_bias_tee(const bool enabled, std::string &error) override {
        static_cast<void>(enabled);
        static_cast<void>(error);
        return true;
    }

  private:
    void run() {
        set_current_thread_name("iq-soapy");
        try {
            std::vector<std::int16_t> samples(source_block_samples * 2);
            while (is_streaming()) {
                std::array<void *, 1> buffers{samples.data()};
                int flags = 0;
                long long time_ns = 0;
                const int received = device_->readStream(
                    stream_, buffers.data(), source_block_samples, flags,
                    time_ns, 100'000);
                if (received > 0) {
                    if (callbacks().samples) {
                        callbacks().samples(
                            std::span(samples.data(),
                                      static_cast<std::size_t>(received) * 2));
                    }
                    continue;
                }
                if (received == SOAPY_SDR_TIMEOUT ||
                    received == SOAPY_SDR_OVERFLOW) {
                    if (received == SOAPY_SDR_OVERFLOW &&
                        callbacks().discontinuity) {
                        callbacks().discontinuity(1);
                    }
                    continue;
                }
                fail(std::string("SoapySDR readStream: ") +
                     SoapySDR::errToStr(received));
                return;
            }
        } catch (const std::exception &exception) {
            fail(std::string("SoapySDR receive worker: ") + exception.what());
        } catch (...) {
            fail("SoapySDR receive worker failed with an unknown exception");
        }
    }

    void fail(std::string message) {
        set_error(std::move(message));
        if (stop_streaming() && callbacks().unexpected_stop) {
            callbacks().unexpected_stop();
        }
    }

    SoapySDR::Device *device_{};
    SoapySDR::Stream *stream_{};
    bool stream_active_{};
    std::thread worker_;
};

class FileIqSource final : public SourceBase {
  public:
    FileIqSource(DeviceDescriptor descriptor, std::filesystem::path path,
                 const bool stdin_source, const std::uint32_t sample_rate_hz,
                 const IqPlaybackPacing pacing)
        : SourceBase(std::move(descriptor),
                     std::vector<std::uint32_t>{sample_rate_hz}),
          path_(std::move(path)), stdin_source_(stdin_source),
          sample_rate_hz_(sample_rate_hz), pacing_(pacing) {}

    ~FileIqSource() override { stop(); }

    FileIqSource(const FileIqSource &) = delete;
    FileIqSource &operator=(const FileIqSource &) = delete;
    FileIqSource(FileIqSource &&) = delete;
    FileIqSource &operator=(FileIqSource &&) = delete;

    [[nodiscard]] bool has_active_worker() const noexcept override {
        return worker_.joinable();
    }

    bool configure(const SourceSettings &settings,
                   std::string &error) override {
        if (settings.sample_rate_hz == 0) {
            error = "A positive sample rate is required for I/Q playback";
            return false;
        }
        sample_rate_hz_ = settings.sample_rate_hz;
        return true;
    }

    bool start(IqSourceCallbacks callbacks, std::string &error) override {
        begin(std::move(callbacks));
        try {
            worker_ = std::thread([this] {
                if (stdin_source_) {
                    run_stdin();
                } else {
                    run_file();
                }
            });
            return true;
        } catch (const std::exception &exception) {
            static_cast<void>(stop_streaming());
            error = "Unable to start I/Q source worker: " +
                    std::string(exception.what());
            return false;
        }
    }

    void stop() noexcept override {
        static_cast<void>(stop_streaming());
        pace_condition_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bool retune(const std::uint64_t frequency_hz, const double correction_ppm,
                std::string &error) override {
        static_cast<void>(frequency_hz);
        static_cast<void>(correction_ppm);
        error = "File source center frequency is fixed by its metadata";
        return false;
    }

    bool set_gain(const SourceSettings &settings, std::string &error) override {
        static_cast<void>(settings);
        static_cast<void>(error);
        return true;
    }

    bool set_bias_tee(const bool enabled, std::string &error) override {
        static_cast<void>(enabled);
        static_cast<void>(error);
        return true;
    }

  private:
    void complete(const bool reached_eof) {
        if (!stop_streaming()) {
            return;
        }
        if (reached_eof) {
            if (callbacks().finite_input_complete) {
                callbacks().finite_input_complete();
            }
            mark_input_exhausted();
        } else if (callbacks().unexpected_stop) {
            callbacks().unexpected_stop();
        }
    }

    void pace(const std::chrono::steady_clock::time_point started_at,
              const std::uint64_t emitted_samples) {
        if (pacing_ == IqPlaybackPacing::unpaced) {
            return;
        }
        const auto elapsed =
            std::chrono::duration<double>(static_cast<double>(emitted_samples) /
                                          static_cast<double>(sample_rate_hz_));
        std::unique_lock lock(pace_mutex_);
        static_cast<void>(pace_condition_.wait_until(
            lock, started_at + elapsed, [this] { return !is_streaming(); }));
    }

    void run_file() {
        set_current_thread_name("iq-file");
        std::ifstream input(path_, std::ios::binary);
        if (!input) {
            set_error("Unable to read I/Q file: " + path_.string());
            complete(false);
            return;
        }

        std::vector<std::int16_t> samples(source_block_samples * 2);
        const auto started_at = std::chrono::steady_clock::now();
        std::uint64_t emitted_samples = 0;
        bool reached_eof = false;
        while (is_streaming()) {
            input.read(reinterpret_cast<char *>(samples.data()),
                       static_cast<std::streamsize>(samples.size() *
                                                    sizeof(std::int16_t)));
            const std::streamsize bytes_read = input.gcount();
            if (bytes_read <= 0) {
                if (input.eof()) {
                    reached_eof = true;
                } else {
                    set_error("Unable to read I/Q file: " + path_.string());
                }
                break;
            }
            const std::size_t scalar_count =
                static_cast<std::size_t>(bytes_read) / sizeof(std::int16_t);
            if (scalar_count % 2 != 0) {
                set_error("I/Q file ended with an incomplete complex sample");
                break;
            }
            if (callbacks().samples) {
                callbacks().samples(std::span(samples.data(), scalar_count));
            }
            emitted_samples += scalar_count / 2;
            pace(started_at, emitted_samples);
        }
        complete(reached_eof);
    }

    void run_stdin() {
        set_current_thread_name("iq-stdin");
        std::vector<std::int16_t> samples(source_block_samples * 2);
        const std::size_t capacity_bytes =
            samples.size() * sizeof(samples.front());
        std::size_t buffered_bytes = 0;
        const auto started_at = std::chrono::steady_clock::now();
        std::uint64_t emitted_samples = 0;
        bool reached_eof = false;

        const auto submit_buffer = [&] {
            const std::size_t scalar_count =
                buffered_bytes / sizeof(samples.front());
            if (callbacks().samples) {
                callbacks().samples(std::span(samples.data(), scalar_count));
            }
            emitted_samples += scalar_count / 2;
            buffered_bytes = 0;
            pace(started_at, emitted_samples);
        };

        while (is_streaming()) {
            pollfd input{.fd = STDIN_FILENO, .events = POLLIN, .revents = 0};
            const int ready = ::poll(&input, 1, 50);
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                set_error(std::string("Unable to poll stdin I/Q: ") +
                          std::strerror(errno));
                break;
            }
            if (ready == 0) {
                continue;
            }
            if ((input.revents & (POLLERR | POLLNVAL)) != 0) {
                set_error("Unable to read stdin I/Q stream");
                break;
            }
            if ((input.revents & (POLLIN | POLLHUP)) == 0) {
                continue;
            }

            // The pacing mutex is local to pace() and is released before this
            // iteration; the analyzer does not model that return correctly.
            // NOLINTNEXTLINE(clang-analyzer-unix.BlockInCriticalSection)
            const ssize_t bytes_read = ::read(
                STDIN_FILENO,
                reinterpret_cast<char *>(samples.data()) + buffered_bytes,
                capacity_bytes - buffered_bytes);
            if (bytes_read < 0) {
                if (errno == EINTR || errno == EAGAIN) {
                    continue;
                }
                set_error(std::string("Unable to read stdin I/Q: ") +
                          std::strerror(errno));
                break;
            }
            if (bytes_read == 0) {
                reached_eof = true;
                break;
            }
            buffered_bytes += static_cast<std::size_t>(bytes_read);
            if (buffered_bytes == capacity_bytes) {
                submit_buffer();
            }
        }

        if (reached_eof && buffered_bytes != 0) {
            if (buffered_bytes % (sizeof(std::int16_t) * 2) != 0) {
                set_error("stdin I/Q ended with an incomplete complex sample");
                reached_eof = false;
            } else {
                submit_buffer();
            }
        }
        complete(reached_eof);
    }

    std::filesystem::path path_;
    bool stdin_source_{};
    std::uint32_t sample_rate_hz_{};
    IqPlaybackPacing pacing_{IqPlaybackPacing::realtime};
    std::mutex pace_mutex_;
    std::condition_variable pace_condition_;
    std::thread worker_;
};

} // namespace

EnumerationResult enumerate_iq_sources(const bool include_soapy_airspy) {
    EnumerationResult result;
    std::array<std::uint64_t, max_airspy_devices> serials{};
    const int airspy_count =
        airspy_list_devices(serials.data(), static_cast<int>(serials.size()));
    if (airspy_count < 0) {
        result.warnings.push_back(
            format_airspy_error("Airspy enumeration", airspy_count));
    } else {
        const int bounded_count =
            std::min(airspy_count, static_cast<int>(serials.size()));
        for (int index = 0; index < bounded_count; ++index) {
            const std::string serial =
                format_serial(serials[static_cast<std::size_t>(index)]);
            result.devices.push_back({
                .backend = SdrBackend::AirspyNative,
                .id = "airspy-native:" + serial,
                .display_name = "Airspy R2 / Mini [native] — " + serial,
                .driver = "libairspy",
                .serial = serial,
                .arguments = {},
            });
        }
    }

    try {
        for (const auto &arguments : SoapySDR::Device::enumerate()) {
            const std::string driver = argument_value(arguments, "driver");
            if (!include_soapy_airspy && driver == "airspy") {
                continue;
            }
            const std::string serial = argument_value(arguments, "serial");
            std::string label = argument_value(arguments, "label");
            if (label.empty()) {
                label = argument_value(arguments, "device");
            }
            if (label.empty()) {
                label = driver.empty() ? "SoapySDR device" : driver;
            }
            result.devices.push_back({
                .backend = SdrBackend::Soapy,
                .id = std::format("soapy:{}:{}:{}", driver, serial, label),
                .display_name = std::format("{} [Soapy{}]", label,
                                            driver.empty() ? "" : "/" + driver),
                .driver = driver,
                .serial = serial,
                .arguments = arguments,
            });
        }
    } catch (const std::exception &exception) {
        result.warnings.push_back(std::string("SoapySDR enumeration: ") +
                                  exception.what());
    }
    return result;
}

std::unique_ptr<IqSource> open_iq_source(const DeviceDescriptor &descriptor,
                                         std::string &error) {
    if (descriptor.backend == SdrBackend::AirspyNative) {
        std::uint64_t serial{};
        const char *begin = descriptor.serial.data();
        const char *end = begin + descriptor.serial.size();
        if (const auto parsed = std::from_chars(begin, end, serial, 16);
            parsed.ec != std::errc{} || parsed.ptr != end) {
            error = "Invalid Airspy serial: " + descriptor.serial;
            return {};
        }
        airspy_device *raw_device = nullptr;
        const int open_result = airspy_open_sn(&raw_device, serial);
        if (open_result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_open_sn", open_result);
            return {};
        }
        std::unique_ptr<airspy_device, AirspyDeviceCloser> device(raw_device);
        std::uint32_t count{};
        int result = airspy_get_samplerates(device.get(), &count, 0);
        std::vector<std::uint32_t> rates;
        if (result == AIRSPY_SUCCESS && count > 0 && count < 256) {
            rates.resize(count);
            result = airspy_get_samplerates(device.get(), rates.data(), count);
        }
        if (result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_get_samplerates", result);
            return {};
        }
        auto source = std::make_unique<AirspySource>(descriptor, device.get(),
                                                     std::move(rates));
        device.release(); // NOLINT(bugprone-unused-return-value)
        return source;
    }

    if (descriptor.backend != SdrBackend::Soapy) {
        error = "Unsupported SDR source backend";
        return {};
    }
    try {
        std::unique_ptr<SoapySDR::Device, SoapyDeviceCloser> device(
            SoapySDR::Device::make(descriptor.arguments));
        if (!device) {
            error = "SoapySDR returned a null device";
            return {};
        }
        std::vector<std::uint32_t> rates;
        for (const double rate : device->listSampleRates(SOAPY_SDR_RX, 0)) {
            if (rate > 0.0 &&
                rate <= static_cast<double>(
                            std::numeric_limits<std::uint32_t>::max())) {
                rates.push_back(static_cast<std::uint32_t>(rate));
            }
        }
        std::optional<std::pair<double, double>> gain_range;
        if (device->hasGainMode(SOAPY_SDR_RX, 0) ||
            !device->listGains(SOAPY_SDR_RX, 0).empty()) {
            const SoapySDR::Range range = device->getGainRange(SOAPY_SDR_RX, 0);
            gain_range = std::pair{range.minimum(), range.maximum()};
        }
        auto source = std::make_unique<SoapySource>(
            descriptor, device.get(), std::move(rates), std::move(gain_range));
        device.release(); // NOLINT(bugprone-unused-return-value)
        return source;
    } catch (const std::exception &exception) {
        error = std::string("SoapySDR open: ") + exception.what();
        return {};
    }
}

std::unique_ptr<IqSource> open_file_iq_source(const std::filesystem::path &path,
                                              SourceSettings &settings,
                                              const IqPlaybackPacing pacing,
                                              std::string &error) {
    IqFileInfo info;
    const bool stdin_source = path == std::filesystem::path("-");
    if (stdin_source) {
        if (settings.sample_rate_hz == 0) {
            error = "A positive sample rate is required for stdin I/Q";
            return {};
        }
        info = {.data_path = {},
                .source = "stdin raw little-endian interleaved CS16",
                .sample_rate_hz = settings.sample_rate_hz,
                .center_frequency_hz = settings.center_frequency_hz,
                .file_size_bytes = 0};
    } else if (!resolve_iq_file(path, settings.sample_rate_hz,
                                settings.center_frequency_hz, info, error)) {
        return {};
    }
    settings.sample_rate_hz = info.sample_rate_hz;
    settings.center_frequency_hz = info.center_frequency_hz;
    DeviceDescriptor descriptor{
        .backend = SdrBackend::File,
        .id = stdin_source ? "stream:stdin" : "file:" + info.data_path.string(),
        .display_name =
            stdin_source ? "stdin [I/Q stream]"
                         : info.data_path.filename().string() + " [I/Q file]",
        .driver = stdin_source ? "stdin" : "file",
        .serial = {},
        .arguments = {{"source", info.source},
                      {"path",
                       stdin_source ? "stdin" : info.data_path.string()}},
    };
    return std::make_unique<FileIqSource>(std::move(descriptor), info.data_path,
                                          stdin_source, settings.sample_rate_hz,
                                          pacing);
}

} // namespace airspy_tv
