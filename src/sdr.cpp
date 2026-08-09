#include "airspy_tv/sdr.hpp"

#include "iq_source.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>
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
    std::uint64_t center_frequency_hz{};
    double frequency_correction_ppm{};
};

SdrDevice::SdrDevice() : impl_(std::make_unique<Impl>()) {}

SdrDevice::~SdrDevice() noexcept { close(); }

EnumerationResult SdrDevice::enumerate(const bool include_soapy_airspy) {
    return enumerate_iq_sources(include_soapy_airspy);
}

bool SdrDevice::open(const DeviceDescriptor &descriptor, std::string &error) {
    close();
    impl_->source = open_iq_source(descriptor, error);
    return impl_->source != nullptr;
}

bool SdrDevice::open_iq_file(const std::filesystem::path &path,
                             SourceSettings &settings,
                             const IqPlaybackPacing pacing,
                             std::string &error) {
    close();
    impl_->source = open_file_iq_source(path, settings, pacing, error);
    return impl_->source != nullptr;
}

void SdrDevice::close() noexcept {
    stop_stream();
    impl_->source.reset();
    impl_->center_frequency_hz = 0;
    impl_->frequency_correction_ppm = 0.0;
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
                             SdrSourceCallbacks callbacks, std::string &error) {
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    stop_stream();
    if (!configure(settings, error)) {
        return false;
    }
    return impl_->source->start(
        {.samples = std::move(callbacks.samples),
         .discontinuity = std::move(callbacks.discontinuity),
         .finite_input_complete = std::move(callbacks.finite_input_complete),
         .unexpected_stop = std::move(callbacks.unexpected_stop)},
        error);
}

void SdrDevice::stop_stream() noexcept {
    if (impl_->source) {
        impl_->source->stop();
    }
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
    if (impl_->center_frequency_hz == 0 ||
        set_center_frequency(impl_->center_frequency_hz, error)) {
        return true;
    }
    impl_->frequency_correction_ppm = previous;
    return false;
}

bool SdrDevice::set_gain(const SourceSettings &settings, std::string &error) {
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    return impl_->source->set_gain(settings, error);
}

bool SdrDevice::set_bias_tee(const bool enabled, std::string &error) {
    if (!impl_->source) {
        error = "No SDR device is open";
        return false;
    }
    return impl_->source->set_bias_tee(enabled, error);
}

bool SdrDevice::is_open() const { return impl_->source != nullptr; }

bool SdrDevice::is_streaming() const {
    return impl_->source && impl_->source->is_streaming();
}

bool SdrDevice::input_exhausted() const {
    return impl_->source && impl_->source->input_exhausted();
}

const DeviceDescriptor *SdrDevice::descriptor() const {
    return impl_->source ? &impl_->source->descriptor() : nullptr;
}

const std::vector<std::uint32_t> &SdrDevice::sample_rates() const {
    return impl_->source ? impl_->source->sample_rates() : impl_->empty_rates;
}

std::optional<std::pair<double, double>> SdrDevice::gain_range() const {
    return impl_->source ? impl_->source->gain_range() : std::nullopt;
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
