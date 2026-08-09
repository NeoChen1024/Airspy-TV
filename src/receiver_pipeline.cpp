#include "receiver_pipeline.hpp"

#include <atomic>
#include <span>
#include <stdexcept>
#include <utility>

namespace airspy_tv {

struct ReceiverPipeline::Impl {
    explicit Impl(TransportPipeline &selected_transport)
        : transport(selected_transport) {}

    SdrDevice source;
    RawIqRecorder recorder;
    SpectrumAnalyzer analyzer;
    InputSampleTimeline input_timeline;
    std::unique_ptr<Demodulator> demodulator;
    TransportPipeline &transport;
    std::atomic<std::uint32_t> active_sample_rate;
    std::atomic<std::uint32_t> active_channel_bandwidth{6'000'000};
    std::atomic<bool> display_analysis_enabled{true};
    DecoderBackpressurePolicy decoder_backpressure{
        DecoderBackpressurePolicy::drop_when_busy};

    void submit_source_block(const std::span<const std::int16_t> samples) {
        const auto rate = active_sample_rate.load(std::memory_order_relaxed);
        const auto bandwidth =
            active_channel_bandwidth.load(std::memory_order_relaxed);
        const auto stamp = input_timeline.stamp(samples.size() / 2, rate);
        if (display_analysis_enabled.load(std::memory_order_relaxed)) {
            analyzer.submit(samples, rate, bandwidth);
        }
        if (demodulator) {
            if (decoder_backpressure == DecoderBackpressurePolicy::block) {
                demodulator->submit_blocking(samples, rate, bandwidth, stamp);
            } else {
                demodulator->submit(samples, rate, bandwidth, stamp);
            }
        }
        recorder.submit(samples);
    }

    void handle_discontinuity(const std::uint64_t dropped_samples) {
        recorder.add_source_dropped_samples(dropped_samples);
        input_timeline.mark_discontinuity(dropped_samples);
        analyzer.reset();
        if (demodulator) {
            demodulator->request_reset();
        }
    }

    void handle_unexpected_stop() {
        analyzer.reset();
        if (demodulator) {
            demodulator->reset();
        }
    }

    SdrSourceCallbacks source_callbacks() {
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
        const bool was_streaming = source.is_streaming();
        source.stop_stream();
        analyzer.reset();
        if (reset_demodulator && demodulator) {
            demodulator->reset();
        } else if (!reset_demodulator && was_streaming && demodulator) {
            demodulator->flush();
        }
    }
};

ReceiverPipeline::ReceiverPipeline(TransportPipeline &transport)
    : impl_(std::make_unique<Impl>(transport)) {}

ReceiverPipeline::~ReceiverPipeline() noexcept { close(); }

bool ReceiverPipeline::open(const DeviceDescriptor &descriptor,
                            std::string &error) {
    close();
    impl_->decoder_backpressure = DecoderBackpressurePolicy::drop_when_busy;
    return impl_->source.open(descriptor, error);
}

bool ReceiverPipeline::open_iq_file(const std::filesystem::path &path,
                                    SourceSettings &settings,
                                    const IqPlaybackPolicy policy,
                                    std::string &error) {
    close();
    impl_->decoder_backpressure = policy.decoder_backpressure;
    return impl_->source.open_iq_file(path, settings, policy.pacing, error);
}

void ReceiverPipeline::close() noexcept {
    impl_->stop_source();
    impl_->recorder.stop();
    impl_->source.close();
    impl_->decoder_backpressure = DecoderBackpressurePolicy::drop_when_busy;
}

bool ReceiverPipeline::start_stream(const SourceSettings &settings,
                                    std::string &error) {
    if (impl_->recorder.stats().active) {
        error = "Stop recording before restarting the receiver";
        return false;
    }
    impl_->stop_source();
    impl_->active_sample_rate.store(settings.sample_rate_hz,
                                    std::memory_order_relaxed);
    impl_->input_timeline.begin_stream(settings.sample_rate_hz);
    return impl_->source.start_stream(settings, impl_->source_callbacks(),
                                      error);
}

void ReceiverPipeline::finish_stream() {
    impl_->stop_source(false);
    impl_->recorder.stop();
}

void ReceiverPipeline::stop_stream() {
    impl_->stop_source();
    impl_->recorder.stop();
}

bool ReceiverPipeline::retune(const std::uint64_t frequency_hz,
                              std::string &error) {
    if (!impl_->source.set_center_frequency(frequency_hz, error)) {
        return false;
    }
    impl_->analyzer.reset();
    impl_->input_timeline.mark_discontinuity();
    if (impl_->demodulator) {
        impl_->demodulator->reset();
    }
    return true;
}

bool ReceiverPipeline::set_frequency_correction_ppm(const double ppm,
                                                    std::string &error) {
    if (!impl_->source.set_frequency_correction_ppm(ppm, error)) {
        return false;
    }
    impl_->analyzer.reset();
    impl_->input_timeline.mark_discontinuity();
    if (impl_->demodulator) {
        impl_->demodulator->reset();
    }
    return true;
}

bool ReceiverPipeline::set_gain(const SourceSettings &settings,
                                std::string &error) {
    return impl_->source.set_gain(settings, error);
}

bool ReceiverPipeline::set_bias_tee(const bool enabled, std::string &error) {
    return impl_->source.set_bias_tee(enabled, error);
}

void ReceiverPipeline::set_display_smoothing(const bool fft_enabled,
                                             const int fft_speed,
                                             const bool signal_enabled,
                                             const int signal_speed) {
    impl_->analyzer.set_smoothing(fft_enabled, fft_speed, signal_enabled,
                                  signal_speed);
}

void ReceiverPipeline::set_display_analysis_enabled(
    const bool enabled) noexcept {
    impl_->display_analysis_enabled.store(enabled, std::memory_order_relaxed);
    if (!enabled) {
        impl_->analyzer.reset();
    }
}

void ReceiverPipeline::set_demodulator_signal_smoothing(const bool enabled,
                                                        const int speed) {
    if (impl_->demodulator) {
        impl_->demodulator->set_signal_smoothing(enabled, speed);
    }
}

void ReceiverPipeline::set_demodulator(
    std::unique_ptr<Demodulator> demodulator) {
    if (impl_->source.is_streaming()) {
        throw std::logic_error(
            "cannot replace the demodulator before the source is stopped");
    }
    impl_->demodulator = std::move(demodulator);
    if (impl_->demodulator) {
        impl_->demodulator->set_transport_callback(
            [this](const std::span<const std::uint8_t> ts) {
                impl_->transport.consume(ts);
            });
        impl_->demodulator->set_discontinuity_callback(
            [this](const TransportDiscontinuity discontinuity) {
                impl_->transport.notify_discontinuity(discontinuity);
            });
    }
}

void ReceiverPipeline::set_channel_bandwidth(const std::uint32_t bandwidth_hz) {
    impl_->active_channel_bandwidth.store(bandwidth_hz,
                                          std::memory_order_relaxed);
}

bool ReceiverPipeline::start_recording(const std::filesystem::path &path,
                                       const SourceSettings &settings,
                                       std::string &error) {
    if (impl_->source.descriptor() != nullptr &&
        impl_->source.descriptor()->backend == SdrBackend::File) {
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
    return impl_->recorder.start(
        path,
        {.source = current == nullptr ? std::string{} : current->display_name,
         .center_frequency_hz = settings.center_frequency_hz,
         .sample_rate_hz = settings.sample_rate_hz},
        error);
}

void ReceiverPipeline::stop_recording() { impl_->recorder.stop(); }

bool ReceiverPipeline::is_open() const { return impl_->source.is_open(); }
bool ReceiverPipeline::is_streaming() const {
    return impl_->source.is_streaming();
}
bool ReceiverPipeline::input_exhausted() const {
    return impl_->source.input_exhausted();
}
bool ReceiverPipeline::is_recording() const {
    return impl_->recorder.stats().active;
}
const DeviceDescriptor *ReceiverPipeline::descriptor() const {
    return impl_->source.descriptor();
}
const std::vector<std::uint32_t> &ReceiverPipeline::sample_rates() const {
    return impl_->source.sample_rates();
}
std::optional<std::pair<double, double>> ReceiverPipeline::gain_range() const {
    return impl_->source.gain_range();
}
RecordingStats ReceiverPipeline::recording_stats() const {
    return impl_->recorder.stats();
}
SpectrumSnapshot ReceiverPipeline::spectrum_snapshot() const {
    return impl_->analyzer.snapshot();
}
SignalSnapshot ReceiverPipeline::signal_snapshot() const {
    return impl_->demodulator ? impl_->demodulator->signal_snapshot()
                              : SignalSnapshot{};
}
PipelineSnapshot ReceiverPipeline::pipeline_snapshot() const {
    return impl_->demodulator ? impl_->demodulator->pipeline_snapshot()
                              : PipelineSnapshot{};
}
InputTimelineSnapshot ReceiverPipeline::input_timeline_snapshot() const {
    return impl_->input_timeline.snapshot();
}
std::string ReceiverPipeline::runtime_error() const {
    const auto source_error = impl_->source.runtime_error();
    return source_error.empty() ? impl_->transport.runtime_error()
                                : source_error;
}

} // namespace airspy_tv
