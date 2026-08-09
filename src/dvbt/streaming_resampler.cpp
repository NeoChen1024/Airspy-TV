#include "streaming_resampler.hpp"

#include "resampler_config.hpp"

#include <cmath>
#include <stdexcept>

namespace airspy_tv::dvbt {

StreamingResampler::StreamingResampler(const std::size_t worker_count)
    : resampler_(worker_count, "dvbt-resamp-") {
    resampler_.set_max_slew_rate(sro_slew_rate_ppm_per_second);
}

std::size_t StreamingResampler::worker_count() const noexcept {
    return resampler_.worker_count();
}

void StreamingResampler::reset() {
    if (configured()) {
        resampler_.set_ratio(resampler_.nominal_ratio());
    }
    resampler_.reset();
}

void StreamingResampler::configure(const std::uint32_t rate,
                                    const std::uint32_t bandwidth) {
    if (configured() && rate == rate_ && bandwidth == bandwidth_) {
        return;
    }
    resampler_.configure(make_resampler_config(rate, bandwidth));
    rate_ = rate;
    bandwidth_ = bandwidth;
}

std::span<const std::complex<float>>
StreamingResampler::process(const std::span<const std::complex<float>> input) {
    return resampler_.process(input);
}

void StreamingResampler::set_sro_correction_ppm(const double correction_ppm) {
    const double scale = 1.0 + correction_ppm * 1.0e-6;
    if (!(scale > 0.0) || !std::isfinite(scale)) {
        throw std::invalid_argument("invalid SRO resampler correction");
    }
    resampler_.set_ratio(resampler_.nominal_ratio() / scale);
}

void StreamingResampler::set_cfo_correction_hz(const double correction_hz) {
    if (!std::isfinite(correction_hz)) {
        throw std::invalid_argument("invalid CFO resampler correction");
    }
    resampler_.set_frequency_shift(-correction_hz);
}

double StreamingResampler::applied_sro_correction_ppm() const noexcept {
    const double ratio = resampler_.effective_ratio();
    return ratio > 0.0 ? (resampler_.nominal_ratio() / ratio - 1.0) * 1.0e6
                       : 0.0;
}

double StreamingResampler::applied_cfo_correction_hz() const noexcept {
    return -resampler_.effective_frequency_shift();
}

double StreamingResampler::requested_cfo_correction_hz() const noexcept {
    return -resampler_.requested_frequency_shift();
}

double StreamingResampler::requested_ratio() const noexcept {
    return resampler_.requested_ratio();
}

double StreamingResampler::effective_ratio() const noexcept {
    return resampler_.effective_ratio();
}

bool StreamingResampler::configured() const noexcept {
    return resampler_.configured();
}

std::uint32_t StreamingResampler::rate() const noexcept { return rate_; }

std::uint32_t StreamingResampler::bandwidth() const noexcept {
    return bandwidth_;
}

} // namespace airspy_tv::dvbt
