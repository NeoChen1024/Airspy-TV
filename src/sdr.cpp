#include "airspy_tv/sdr.hpp"

#include "airspy_tv/transport_router.hpp"
#include "iq_source.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

[[nodiscard]] bool valid_frequency_correction(const double ppm) noexcept {
    return std::isfinite(ppm) && std::abs(ppm) <= max_frequency_correction_ppm;
}

} // namespace

struct SdrDevice::Impl {
    std::unique_ptr<IqSource> source;
    std::vector<std::uint32_t> empty_rates;
    RawIqRecorder recorder;
    TransportStreamRecorder ts_recorder;
    RtpUdpTransportOutput rtp_output;
    TransportStreamModel transport_model;
    TransportStreamRouter transport_router{transport_model, ts_recorder,
                                           rtp_output};
    SpectrumAnalyzer analyzer;
    InputSampleTimeline input_timeline;
    std::unique_ptr<Demodulator> demodulator;
    std::uint64_t center_frequency_hz{};
    double frequency_correction_ppm{};
    std::atomic<std::uint32_t> active_sample_rate;
    std::atomic<std::uint32_t> active_channel_bandwidth{6'000'000};
    std::atomic<bool> display_analysis_enabled{true};
    DecoderBackpressurePolicy decoder_backpressure{
        DecoderBackpressurePolicy::drop_when_busy};

    void submit_source_block(const std::span<const std::int16_t> sample_block) {
        const auto rate = active_sample_rate.load(std::memory_order_relaxed);
        const auto bandwidth =
            active_channel_bandwidth.load(std::memory_order_relaxed);
        const InputSampleStamp stamp =
            input_timeline.stamp(sample_block.size() / 2, rate);
        if (display_analysis_enabled.load(std::memory_order_relaxed)) {
            analyzer.submit(sample_block, rate, bandwidth);
        }
        if (demodulator) {
            if (decoder_backpressure == DecoderBackpressurePolicy::block) {
                demodulator->submit_blocking(sample_block, rate, bandwidth,
                                             stamp);
            } else {
                demodulator->submit(sample_block, rate, bandwidth, stamp);
            }
        }
        recorder.submit(sample_block);
    }

    void handle_discontinuity(const std::uint64_t known_dropped_samples) {
        recorder.add_source_dropped_samples(known_dropped_samples);
        input_timeline.mark_discontinuity(known_dropped_samples);
        analyzer.reset();
        if (demodulator) {
            demodulator->request_reset();
        }
    }

    void handle_unexpected_stop() {
        analyzer.reset();
        if (demodulator) {
            // Reset decoder state; smart-pointer ownership is unchanged.
            // NOLINTNEXTLINE(readability-ambiguous-smartptr-reset-call)
            demodulator->reset();
        }
    }

    [[nodiscard]] IqSourceCallbacks source_callbacks() {
        return {
            .samples =
                [this](const std::span<const std::int16_t> samples) {
                    submit_source_block(samples);
                },
            .discontinuity =
                [this](const std::uint64_t dropped_samples) {
                    handle_discontinuity(dropped_samples);
                },
            .finite_input_complete =
                [this] {
                    if (demodulator) {
                        demodulator->flush();
                    }
                },
            .unexpected_stop = [this] { handle_unexpected_stop(); },
        };
    }

    void stop_source(const bool reset_demodulator = true) {
        const bool was_streaming = source && source->is_streaming();
        if (source) {
            source->stop();
        }
        analyzer.reset();
        if (reset_demodulator && demodulator) {
            // Reset decoder state; smart-pointer ownership is unchanged.
            // NOLINTNEXTLINE(readability-ambiguous-smartptr-reset-call)
            demodulator->reset();
        }
        if (!reset_demodulator && was_streaming && demodulator) {
            demodulator->flush();
        }
    }

    void stop_all() {
        stop_source();
        recorder.stop();
        ts_recorder.stop();
        rtp_output.stop();
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
    return enumerate_iq_sources(include_soapy_airspy);
}

bool SdrDevice::open(const DeviceDescriptor &descriptor, std::string &error) {
    close();
    impl_->transport_model.reset();
    impl_->decoder_backpressure = DecoderBackpressurePolicy::drop_when_busy;
    impl_->source = open_iq_source(descriptor, error);
    return impl_->source != nullptr;
}

bool SdrDevice::open_iq_file(const std::filesystem::path &path,
                             SourceSettings &settings, std::string &error) {
    return open_iq_file(path, settings, {}, error);
}

bool SdrDevice::open_iq_file(const std::filesystem::path &path,
                             SourceSettings &settings,
                             const IqPlaybackPolicy policy,
                             std::string &error) {
    close();
    impl_->transport_model.reset();
    impl_->decoder_backpressure = policy.decoder_backpressure;
    impl_->source = open_file_iq_source(path, settings, policy.pacing, error);
    return impl_->source != nullptr;
}

void SdrDevice::close() {
    impl_->stop_all();
    impl_->source.reset();
    impl_->center_frequency_hz = 0;
    impl_->frequency_correction_ppm = 0.0;
    impl_->decoder_backpressure = DecoderBackpressurePolicy::drop_when_busy;
    impl_->transport_model.reset();
}

bool SdrDevice::configure(const SourceSettings &settings, std::string &error) {
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    if (impl_->source->has_active_worker()) {
        error = "Stop the source before changing receiver settings";
        return false;
    }
    if (!valid_frequency_correction(settings.frequency_correction_ppm)) {
        error = "Frequency correction must be finite and within +/-1000 ppm";
        return false;
    }
    impl_->center_frequency_hz = settings.center_frequency_hz;
    impl_->frequency_correction_ppm = settings.frequency_correction_ppm;
    return impl_->source->configure(settings, error);
}

bool SdrDevice::start_stream(const SourceSettings &settings,
                             std::string &error) {
    if (!impl_->source) {
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

    impl_->active_sample_rate.store(settings.sample_rate_hz,
                                    std::memory_order_relaxed);
    impl_->input_timeline.begin_stream(settings.sample_rate_hz);
    return impl_->source->start(impl_->source_callbacks(), error);
}

void SdrDevice::stop_stream() {
    impl_->stop_source();
    impl_->recorder.stop();
}

void SdrDevice::finish_stream() {
    impl_->stop_source(false);
    impl_->recorder.stop();
}

bool SdrDevice::set_gain(const SourceSettings &settings, std::string &error) {
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    return impl_->source->set_gain(settings, error);
}

bool SdrDevice::set_center_frequency(const std::uint64_t frequency_hz,
                                     std::string &error) {
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    if (!impl_->source->retune(frequency_hz, impl_->frequency_correction_ppm,
                               error)) {
        return false;
    }
    impl_->analyzer.reset();
    impl_->input_timeline.mark_discontinuity();
    if (impl_->demodulator) {
        // Reset decoder state; smart-pointer ownership is unchanged.
        // NOLINTNEXTLINE(readability-ambiguous-smartptr-reset-call)
        impl_->demodulator->reset();
    }
    impl_->transport_model.reset();
    impl_->center_frequency_hz = frequency_hz;
    return true;
}

bool SdrDevice::set_frequency_correction_ppm(const double ppm,
                                             std::string &error) {
    if (!valid_frequency_correction(ppm)) {
        error = "Frequency correction must be finite and within +/-1000 ppm";
        return false;
    }
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    if (impl_->source->descriptor().backend == SdrBackend::File) {
        error = "File source frequency correction is fixed by its metadata";
        return false;
    }

    const double previous = impl_->frequency_correction_ppm;
    impl_->frequency_correction_ppm = ppm;
    if (impl_->center_frequency_hz == 0) {
        return true;
    }
    if (set_center_frequency(impl_->center_frequency_hz, error)) {
        return true;
    }
    impl_->frequency_correction_ppm = previous;
    return false;
}

bool SdrDevice::set_bias_tee(const bool enabled, std::string &error) {
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    return impl_->source->set_bias_tee(enabled, error);
}

void SdrDevice::set_display_smoothing(const bool fft_enabled,
                                      const int fft_speed,
                                      const bool snr_enabled,
                                      const int snr_speed) {
    impl_->analyzer.set_smoothing(fft_enabled, fft_speed, snr_enabled,
                                  snr_speed);
}

void SdrDevice::set_display_analysis_enabled(const bool enabled) noexcept {
    impl_->display_analysis_enabled.store(enabled, std::memory_order_relaxed);
    if (!enabled) {
        impl_->analyzer.reset();
    }
}

void SdrDevice::set_demodulator_signal_smoothing(const bool enabled,
                                                 const int speed) {
    if (impl_->demodulator) {
        impl_->demodulator->set_signal_smoothing(enabled, speed);
    }
}

void SdrDevice::set_demodulator(std::unique_ptr<Demodulator> demodulator) {
    if (impl_->source && impl_->source->has_active_worker()) {
        throw std::logic_error(
            "cannot replace the demodulator before the source is stopped");
    }
    impl_->demodulator = std::move(demodulator);
    if (impl_->demodulator) {
        impl_->demodulator->set_transport_callback(
            [this](const std::span<const std::uint8_t> ts) {
                impl_->transport_router.consume(ts);
            });
    }
}

void SdrDevice::set_channel_bandwidth(const std::uint32_t bandwidth_hz) {
    impl_->active_channel_bandwidth.store(bandwidth_hz,
                                          std::memory_order_relaxed);
}

bool SdrDevice::start_recording(const std::filesystem::path &path,
                                const SourceSettings &settings,
                                std::string &error) {
    if (impl_->source &&
        impl_->source->descriptor().backend == SdrBackend::File) {
        error = "Raw I/Q recording is unavailable during file playback";
        return false;
    }
    if ((!is_streaming() ||
         impl_->active_sample_rate.load(std::memory_order_relaxed) !=
             settings.sample_rate_hz) &&
        !start_stream(settings, error)) {
        return false;
    }

    const DeviceDescriptor *current = descriptor();
    const RecordingMetadata metadata{
        .source = current == nullptr ? std::string{} : current->display_name,
        .center_frequency_hz = settings.center_frequency_hz,
        .sample_rate_hz = settings.sample_rate_hz,
    };
    return impl_->recorder.start(path, metadata, error);
}

void SdrDevice::stop_recording() { impl_->recorder.stop(); }

bool SdrDevice::start_ts_recording(const std::filesystem::path &path,
                                   std::string &error) {
    if (!is_streaming()) {
        error = "Start an SDR or I/Q file source before recording MPEG-TS";
        return false;
    }
    return impl_->ts_recorder.start(path, error);
}

void SdrDevice::stop_ts_recording() { impl_->ts_recorder.stop(); }

bool SdrDevice::start_rtp_streaming(const RtpUdpEndpoint &endpoint,
                                    std::string &error) {
    if (!is_streaming()) {
        error = "Start an SDR or I/Q file source before RTP/UDP streaming";
        return false;
    }
    return impl_->rtp_output.start(endpoint, error);
}

void SdrDevice::stop_rtp_streaming() { impl_->rtp_output.stop(); }

void SdrDevice::set_transport_sink(TransportSink sink) {
    impl_->transport_router.set_sink(std::move(sink));
}

bool SdrDevice::is_open() const { return impl_->source != nullptr; }

bool SdrDevice::is_streaming() const {
    return impl_->source && impl_->source->is_streaming();
}

bool SdrDevice::input_exhausted() const {
    return impl_->source && impl_->source->input_exhausted();
}

bool SdrDevice::is_recording() const { return impl_->recorder.stats().active; }

const DeviceDescriptor *SdrDevice::descriptor() const {
    return impl_->source ? &impl_->source->descriptor() : nullptr;
}

const std::vector<std::uint32_t> &SdrDevice::sample_rates() const {
    return impl_->source ? impl_->source->sample_rates() : impl_->empty_rates;
}

std::optional<std::pair<double, double>> SdrDevice::gain_range() const {
    return impl_->source ? impl_->source->gain_range() : std::nullopt;
}

RecordingStats SdrDevice::recording_stats() const {
    return impl_->recorder.stats();
}

TransportRecordingStats SdrDevice::ts_recording_stats() const {
    return impl_->ts_recorder.stats();
}

RtpUdpStats SdrDevice::rtp_streaming_stats() const {
    return impl_->rtp_output.stats();
}

SpectrumSnapshot SdrDevice::spectrum_snapshot() const {
    return impl_->analyzer.snapshot();
}

SignalSnapshot SdrDevice::signal_snapshot() const {
    return impl_->demodulator ? impl_->demodulator->signal_snapshot()
                              : SignalSnapshot{};
}

PipelineSnapshot SdrDevice::pipeline_snapshot() const {
    return impl_->demodulator ? impl_->demodulator->pipeline_snapshot()
                              : PipelineSnapshot{};
}

InputTimelineSnapshot SdrDevice::input_timeline_snapshot() const {
    return impl_->input_timeline.snapshot();
}

std::vector<TransportService> SdrDevice::transport_services() const {
    return impl_->transport_model.services();
}

std::string SdrDevice::runtime_error() const {
    return impl_->source ? impl_->source->runtime_error() : std::string{};
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
