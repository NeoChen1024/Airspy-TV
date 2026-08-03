#include "airspy_tv/sdr.hpp"

#include "airspy_tv/iq_file.hpp"

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <libairspy/airspy.h>
#include <limits>
#include <mutex>
#include <span>
#include <sstream>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

constexpr std::size_t max_airspy_devices = 32;
constexpr std::size_t soapy_block_samples = 32'768;
constexpr std::size_t file_block_samples = 32'768;

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

} // namespace

struct SdrDevice::Impl {
    DeviceDescriptor current;
    bool opened{};
    airspy_device *airspy{};
    SoapySDR::Device *soapy{};
    SoapySDR::Stream *soapy_stream{};
    std::vector<std::uint32_t> rates;
    std::optional<std::pair<double, double>> soapy_gain_range;
    RawIqRecorder recorder;
    TransportStreamRecorder ts_recorder;
    TransportStreamModel transport_model;
    SpectrumAnalyzer analyzer;
    dvbt::SignalAnalyzer signal_analyzer;
    dvbt::StreamDecoder stream_decoder;
    std::thread soapy_worker;
    std::thread file_worker;
    std::filesystem::path file_path;
    std::atomic<bool> streaming;
    std::atomic<std::uint32_t> active_sample_rate;
    std::atomic<std::uint32_t> active_channel_bandwidth{6'000'000};
    mutable std::mutex error_mutex;
    std::string async_error;

    Impl() {
        stream_decoder.set_transport_callback(
            [this](const std::span<const std::uint8_t> ts) {
                transport_model.consume(ts);
                ts_recorder.submit(ts);
            });
    }

    static int airspy_rx_callback(airspy_transfer *transfer) {
        auto *self = static_cast<Impl *>(transfer->ctx);
        if (self == nullptr || !self->streaming ||
            transfer->samples == nullptr || transfer->sample_count <= 0) {
            return 0;
        }

        const auto *samples =
            static_cast<const std::int16_t *>(transfer->samples);
        const auto scalar_count =
            static_cast<std::size_t>(transfer->sample_count) * 2;
        const std::span sample_block(samples, scalar_count);
        if (transfer->dropped_samples != 0) {
            self->recorder.add_source_dropped_samples(
                transfer->dropped_samples);
            self->analyzer.reset();
            self->signal_analyzer.reset();
            self->stream_decoder.reset();
        }
        self->analyzer.submit(sample_block, self->active_sample_rate,
                              self->active_channel_bandwidth);
        self->signal_analyzer.submit(sample_block, self->active_sample_rate,
                                     self->active_channel_bandwidth);
        self->stream_decoder.submit(sample_block, self->active_sample_rate,
                                    self->active_channel_bandwidth);
        self->recorder.submit(sample_block);
        return 0;
    }

    void set_async_error(std::string message) {
        const std::scoped_lock lock(error_mutex);
        async_error = std::move(message);
    }

    void run_soapy() {
        std::vector<std::int16_t> samples(soapy_block_samples * 2);
        while (streaming) {
            std::array<void *, 1> buffers{samples.data()};
            int flags = 0;
            long long time_ns = 0;
            const int received =
                soapy->readStream(soapy_stream, buffers.data(),
                                  soapy_block_samples, flags, time_ns, 100'000);
            if (received > 0) {
                const std::span sample_block(
                    samples.data(), static_cast<std::size_t>(received) * 2);
                analyzer.submit(sample_block, active_sample_rate,
                                active_channel_bandwidth);
                signal_analyzer.submit(sample_block, active_sample_rate,
                                       active_channel_bandwidth);
                stream_decoder.submit(sample_block, active_sample_rate,
                                      active_channel_bandwidth);
                recorder.submit(sample_block);
                continue;
            }
            if (received == SOAPY_SDR_TIMEOUT ||
                received == SOAPY_SDR_OVERFLOW) {
                if (received == SOAPY_SDR_OVERFLOW) {
                    recorder.add_source_dropped_samples(1);
                    analyzer.reset();
                    signal_analyzer.reset();
                    stream_decoder.reset();
                }
                continue;
            }
            set_async_error(std::string("SoapySDR readStream: ") +
                            SoapySDR::errToStr(received));
            streaming = false;
        }
        analyzer.reset();
        signal_analyzer.reset();
        stream_decoder.reset();
    }

    void run_file() {
        std::ifstream input(file_path, std::ios::binary);
        if (!input) {
            set_async_error("Unable to read I/Q file: " + file_path.string());
            streaming = false;
            return;
        }

        std::vector<std::int16_t> samples(file_block_samples * 2);
        const auto started_at = std::chrono::steady_clock::now();
        std::uint64_t emitted_samples = 0;
        while (streaming) {
            input.read(reinterpret_cast<char *>(samples.data()),
                       static_cast<std::streamsize>(samples.size() *
                                                    sizeof(std::int16_t)));
            const std::streamsize bytes_read = input.gcount();
            if (bytes_read <= 0) {
                break;
            }
            const std::size_t scalar_count =
                static_cast<std::size_t>(bytes_read) / sizeof(std::int16_t);
            if (scalar_count % 2 != 0) {
                set_async_error(
                    "I/Q file ended with an incomplete complex sample");
                break;
            }
            const std::span sample_block(samples.data(), scalar_count);
            analyzer.submit(sample_block, active_sample_rate,
                            active_channel_bandwidth);
            signal_analyzer.submit(sample_block, active_sample_rate,
                                   active_channel_bandwidth);
            stream_decoder.submit(sample_block, active_sample_rate,
                                  active_channel_bandwidth);
            emitted_samples += scalar_count / 2;

            const auto elapsed = std::chrono::duration<double>(
                static_cast<double>(emitted_samples) /
                static_cast<double>(active_sample_rate.load()));
            std::this_thread::sleep_until(started_at + elapsed);
        }
        if (streaming) {
            stream_decoder.flush();
        }
        if (streaming.exchange(false)) {
            set_async_error("I/Q file playback finished");
        }
    }

    void stop_source() {
        const bool was_streaming = streaming.exchange(false);
        // Cancel queued/active DSP immediately. In particular, the native
        // DVB-T worker may otherwise continue decoding a large chunk while
        // the UI is already tearing down after a window-close request.
        analyzer.reset();
        signal_analyzer.reset();
        stream_decoder.reset();
        if (current.backend == SdrBackend::AirspyNative && airspy != nullptr &&
            was_streaming) {
            airspy_stop_rx(airspy);
        }
        if (soapy_worker.joinable()) {
            soapy_worker.join();
        }
        if (file_worker.joinable()) {
            file_worker.join();
        }
        if (soapy != nullptr && soapy_stream != nullptr) {
            soapy->deactivateStream(soapy_stream);
            soapy->closeStream(soapy_stream);
            soapy_stream = nullptr;
        }
    }

    void stop_all() {
        stop_source();
        recorder.stop();
        ts_recorder.stop();
    }
};

SdrDevice::SdrDevice() : impl_(std::make_unique<Impl>()) {}

SdrDevice::~SdrDevice() noexcept {
    try {
        close();
    } catch (...) { // NOLINT(bugprone-empty-catch)
    }
}

EnumerationResult SdrDevice::enumerate(const bool include_soapy_airspy) {
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

bool SdrDevice::open(const DeviceDescriptor &descriptor, std::string &error) {
    close();
    impl_->transport_model.reset();
    impl_->current = descriptor;
    impl_->async_error.clear();

    if (descriptor.backend == SdrBackend::AirspyNative) {
        std::uint64_t serial{};
        const char *begin = descriptor.serial.data();
        const char *end = begin + descriptor.serial.size();
        if (const auto parsed = std::from_chars(begin, end, serial, 16);
            parsed.ec != std::errc{}) {
            error = "Invalid Airspy serial: " + descriptor.serial;
            return false;
        }
        const int open_result = airspy_open_sn(&impl_->airspy, serial);
        if (open_result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_open_sn", open_result);
            impl_->airspy = nullptr;
            return false;
        }

        std::uint32_t count{};
        int result = airspy_get_samplerates(impl_->airspy, &count, 0);
        if (result == AIRSPY_SUCCESS && count > 0 && count < 256) {
            impl_->rates.resize(count);
            result = airspy_get_samplerates(impl_->airspy, impl_->rates.data(),
                                            count);
        }
        if (result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_get_samplerates", result);
            airspy_close(impl_->airspy);
            impl_->airspy = nullptr;
            impl_->rates.clear();
            return false;
        }
    } else if (descriptor.backend == SdrBackend::Soapy) {
        try {
            impl_->soapy = SoapySDR::Device::make(descriptor.arguments);
            if (impl_->soapy == nullptr) {
                error = "SoapySDR returned a null device";
                return false;
            }
            const auto rates = impl_->soapy->listSampleRates(SOAPY_SDR_RX, 0);
            impl_->rates.reserve(rates.size());
            for (const double rate : rates) {
                if (rate > 0.0 &&
                    rate <= static_cast<double>(
                                std::numeric_limits<std::uint32_t>::max())) {
                    impl_->rates.push_back(static_cast<std::uint32_t>(rate));
                }
            }
            if (impl_->soapy->hasGainMode(SOAPY_SDR_RX, 0) ||
                !impl_->soapy->listGains(SOAPY_SDR_RX, 0).empty()) {
                const SoapySDR::Range range =
                    impl_->soapy->getGainRange(SOAPY_SDR_RX, 0);
                impl_->soapy_gain_range =
                    std::pair{range.minimum(), range.maximum()};
            }
        } catch (const std::exception &exception) {
            error = std::string("SoapySDR open: ") + exception.what();
            if (impl_->soapy != nullptr) {
                SoapySDR::Device::unmake(impl_->soapy);
                impl_->soapy = nullptr;
            }
            return false;
        }
    }

    impl_->opened = true;
    return true;
}

bool SdrDevice::open_iq_file(const std::filesystem::path &path,
                             SourceSettings &settings, std::string &error) {
    close();
    impl_->transport_model.reset();
    IqFileInfo info;
    if (!resolve_iq_file(path, settings.sample_rate_hz,
                         settings.center_frequency_hz, info, error)) {
        return false;
    }
    settings.sample_rate_hz = info.sample_rate_hz;
    settings.center_frequency_hz = info.center_frequency_hz;
    impl_->file_path = info.data_path;
    impl_->rates = {settings.sample_rate_hz};
    impl_->current = {
        .backend = SdrBackend::File,
        .id = "file:" + info.data_path.string(),
        .display_name = info.data_path.filename().string() + " [I/Q file]",
        .driver = "file",
        .serial = {},
        .arguments = {{"source", info.source},
                      {"path", info.data_path.string()}},
    };
    impl_->async_error.clear();
    impl_->opened = true;
    return true;
}

void SdrDevice::close() {
    impl_->stop_all();
    if (impl_->airspy != nullptr) {
        airspy_close(impl_->airspy);
        impl_->airspy = nullptr;
    }
    if (impl_->soapy != nullptr) {
        SoapySDR::Device::unmake(impl_->soapy);
        impl_->soapy = nullptr;
    }
    impl_->opened = false;
    impl_->file_path.clear();
    impl_->rates.clear();
    impl_->soapy_gain_range.reset();
    impl_->transport_model.reset();
}

bool SdrDevice::configure(const SourceSettings &settings, std::string &error) {
    if (!impl_->opened) {
        error = "No SDR device is open";
        return false;
    }
    if (impl_->streaming) {
        error = "Stop recording before changing receiver settings";
        return false;
    }

    if (impl_->current.backend == SdrBackend::File) {
        return true;
    }

    if (impl_->current.backend == SdrBackend::AirspyNative) {
        if (settings.center_frequency_hz >
            std::numeric_limits<std::uint32_t>::max()) {
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
                 airspy_set_sample_type(impl_->airspy,
                                        AIRSPY_SAMPLE_INT16_IQ)) ||
            !run("airspy_set_samplerate",
                 airspy_set_samplerate(impl_->airspy,
                                       settings.sample_rate_hz)) ||
            !run("airspy_set_freq",
                 airspy_set_freq(impl_->airspy,
                                 static_cast<std::uint32_t>(
                                     settings.center_frequency_hz)))) {
            return false;
        }
        return set_bias_tee(settings.bias_tee, error) &&
               set_gain(settings, error);
    }

    try {
        impl_->soapy->setSampleRate(SOAPY_SDR_RX, 0, settings.sample_rate_hz);
        impl_->soapy->setFrequency(
            SOAPY_SDR_RX, 0, static_cast<double>(settings.center_frequency_hz));
        return set_gain(settings, error);
    } catch (const std::exception &exception) {
        error = std::string("SoapySDR configure: ") + exception.what();
        return false;
    }
}

bool SdrDevice::start_stream(const SourceSettings &settings,
                             std::string &error) {
    if (!impl_->opened) {
        error = "No SDR device is open";
        return false;
    }
    if (impl_->recorder.stats().active) {
        error = "Stop recording before restarting the receiver";
        return false;
    }

    impl_->stop_source();
    impl_->transport_model.reset();
    if (!configure(settings, error)) {
        return false;
    }

    impl_->async_error.clear();
    impl_->active_sample_rate = settings.sample_rate_hz;
    if (impl_->current.backend == SdrBackend::File) {
        impl_->streaming = true;
        impl_->file_worker = std::thread([this] { impl_->run_file(); });
        return true;
    }
    if (impl_->current.backend == SdrBackend::AirspyNative) {
        impl_->streaming = true;
        const int result = airspy_start_rx(
            impl_->airspy, &Impl::airspy_rx_callback, impl_.get());
        if (result != AIRSPY_SUCCESS) {
            impl_->streaming = false;
            error = format_airspy_error("airspy_start_rx", result);
            return false;
        }
        return true;
    }

    try {
        impl_->soapy_stream =
            impl_->soapy->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
        if (impl_->soapy_stream == nullptr) {
            throw std::runtime_error("setupStream returned null");
        }
        const int result = impl_->soapy->activateStream(impl_->soapy_stream);
        if (result != 0) {
            throw std::runtime_error(std::string("activateStream: ") +
                                     SoapySDR::errToStr(result));
        }
        impl_->streaming = true;
        impl_->soapy_worker = std::thread([this] { impl_->run_soapy(); });
        return true;
    } catch (const std::exception &exception) {
        error = std::string("SoapySDR start stream: ") + exception.what();
        impl_->stop_source();
        return false;
    }
}

void SdrDevice::stop_stream() {
    impl_->stop_source();
    impl_->recorder.stop();
}

bool SdrDevice::set_gain(const SourceSettings &settings, std::string &error) {
    if (!impl_->opened) {
        error = "No SDR device is open";
        return false;
    }

    if (impl_->current.backend == SdrBackend::AirspyNative) {
        const auto gain =
            static_cast<std::uint8_t>(std::clamp(settings.airspy_gain, 0, 21));
        const int result =
            settings.airspy_gain_mode == AirspyGainMode::Sensitivity
                ? airspy_set_sensitivity_gain(impl_->airspy, gain)
                : airspy_set_linearity_gain(impl_->airspy, gain);
        if (result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_set_gain_profile", result);
            return false;
        }
        return true;
    }

    if (impl_->current.backend == SdrBackend::File) {
        return true;
    }

    if (!impl_->soapy_gain_range.has_value()) {
        return true;
    }
    try {
        const auto [minimum, maximum] = *impl_->soapy_gain_range;
        impl_->soapy->setGain(
            SOAPY_SDR_RX, 0, std::clamp(settings.soapy_gain, minimum, maximum));
        return true;
    } catch (const std::exception &exception) {
        error = std::string("SoapySDR set gain: ") + exception.what();
        return false;
    }
}

bool SdrDevice::set_center_frequency(const std::uint64_t frequency_hz,
                                     std::string &error) {
    if (!impl_->opened) {
        error = "No SDR device is open";
        return false;
    }

    if (impl_->current.backend == SdrBackend::AirspyNative) {
        if (frequency_hz > std::numeric_limits<std::uint32_t>::max()) {
            error = "Airspy center frequency is out of range";
            return false;
        }
        const int result = airspy_set_freq(
            impl_->airspy, static_cast<std::uint32_t>(frequency_hz));
        if (result != AIRSPY_SUCCESS) {
            error = format_airspy_error("airspy_set_freq", result);
            return false;
        }
        impl_->analyzer.reset();
        impl_->signal_analyzer.reset();
        impl_->stream_decoder.reset();
        impl_->transport_model.reset();
        return true;
    }

    if (impl_->current.backend == SdrBackend::File) {
        error = "File source center frequency is fixed by its metadata";
        return false;
    }

    try {
        impl_->soapy->setFrequency(SOAPY_SDR_RX, 0,
                                   static_cast<double>(frequency_hz));
        impl_->analyzer.reset();
        impl_->signal_analyzer.reset();
        impl_->stream_decoder.reset();
        impl_->transport_model.reset();
        return true;
    } catch (const std::exception &exception) {
        error =
            std::string("SoapySDR set center frequency: ") + exception.what();
        return false;
    }
}

bool SdrDevice::set_bias_tee(const bool enabled, std::string &error) {
    if (!impl_->opened) {
        error = "No SDR device is open";
        return false;
    }
    if (impl_->current.backend == SdrBackend::File) {
        return true;
    }
    if (impl_->current.backend != SdrBackend::AirspyNative) {
        return true;
    }

    const int result =
        airspy_set_rf_bias(impl_->airspy, static_cast<std::uint8_t>(enabled));
    if (result != AIRSPY_SUCCESS) {
        error = format_airspy_error("airspy_set_rf_bias", result);
        return false;
    }
    return true;
}

void SdrDevice::set_display_smoothing(const bool fft_enabled,
                                      const int fft_speed,
                                      const bool snr_enabled,
                                      const int snr_speed) {
    impl_->analyzer.set_smoothing(fft_enabled, fft_speed, snr_enabled,
                                  snr_speed);
    impl_->signal_analyzer.set_snr_smoothing(snr_enabled, snr_speed);
}

void SdrDevice::set_dvbt_parameters(
    const dvbt::ReceiverParameters &parameters) {
    impl_->active_channel_bandwidth = parameters.channel_bandwidth_hz;
    impl_->signal_analyzer.set_parameters(parameters);
    impl_->stream_decoder.set_parameters(parameters);
}

bool SdrDevice::start_recording(const std::filesystem::path &path,
                                const SourceSettings &settings,
                                std::string &error) {
    if (impl_->opened && impl_->current.backend == SdrBackend::File) {
        error = "Raw I/Q recording is unavailable during file playback";
        return false;
    }
    if ((!impl_->streaming ||
         impl_->active_sample_rate != settings.sample_rate_hz) &&
        !start_stream(settings, error)) {
        return false;
    }

    const RecordingMetadata metadata{
        .source = impl_->current.display_name,
        .center_frequency_hz = settings.center_frequency_hz,
        .sample_rate_hz = settings.sample_rate_hz,
    };
    return impl_->recorder.start(path, metadata, error);
}

void SdrDevice::stop_recording() { impl_->recorder.stop(); }

bool SdrDevice::start_ts_recording(const std::filesystem::path &path,
                                   std::string &error) {
    if (!impl_->streaming) {
        error = "Start an SDR or I/Q file source before recording MPEG-TS";
        return false;
    }
    return impl_->ts_recorder.start(path, error);
}

void SdrDevice::stop_ts_recording() { impl_->ts_recorder.stop(); }

bool SdrDevice::is_open() const { return impl_->opened; }

bool SdrDevice::is_streaming() const { return impl_->streaming; }

bool SdrDevice::is_recording() const { return impl_->recorder.stats().active; }

const DeviceDescriptor *SdrDevice::descriptor() const {
    return impl_->opened ? &impl_->current : nullptr;
}

const std::vector<std::uint32_t> &SdrDevice::sample_rates() const {
    return impl_->rates;
}

std::optional<std::pair<double, double>> SdrDevice::gain_range() const {
    return impl_->soapy_gain_range;
}

RecordingStats SdrDevice::recording_stats() const {
    return impl_->recorder.stats();
}

TransportRecordingStats SdrDevice::ts_recording_stats() const {
    return impl_->ts_recorder.stats();
}

SpectrumSnapshot SdrDevice::spectrum_snapshot() const {
    return impl_->analyzer.snapshot();
}

dvbt::SignalAnalysisSnapshot SdrDevice::signal_analysis_snapshot() const {
    return impl_->signal_analyzer.snapshot();
}

dvbt::StreamDecoderStats SdrDevice::decoder_stats() const {
    return impl_->stream_decoder.stats();
}

std::vector<TransportService> SdrDevice::transport_services() const {
    return impl_->transport_model.services();
}

std::string SdrDevice::runtime_error() const {
    const std::scoped_lock lock(impl_->error_mutex);
    return impl_->async_error;
}

std::string backend_name(const SdrBackend backend) {
    switch (backend) {
    case SdrBackend::AirspyNative:
        return "Airspy native";
    case SdrBackend::Soapy:
        return "SoapySDR";
    case SdrBackend::File:
        return "I/Q file";
    }
    return "Unknown";
}

} // namespace airspy_tv
