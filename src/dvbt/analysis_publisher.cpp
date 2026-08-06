#include "airspy_tv/dvbt/analysis_publisher.hpp"

#include <algorithm>
#include <chrono>

namespace airspy_tv::dvbt {

void AnalysisPublisher::publish(
    const std::span<const std::complex<float>> carriers, const float mer_db,
    const float cp_snr_db, const float deepest_notch_db,
    const float carrier_offset_hz, const TransmissionMode mode,
    const GuardInterval guard_interval, const Constellation constellation) {
    SignalAnalysisSnapshot next;
    next.point_count = std::min(carriers.size(), next.points.size());
    if (next.point_count != 0) {
        for (std::size_t index = 0; index < next.point_count; ++index) {
            next.points[index] =
                carriers[index * carriers.size() / next.point_count];
        }
    }
    next.mer_db = mer_db;
    next.cp_snr_db = cp_snr_db;
    next.deepest_notch_db = deepest_notch_db;
    next.carrier_offset_hz = carrier_offset_hz;
    next.mode = mode;
    next.guard_interval = guard_interval;
    next.constellation = constellation;
    next.locked = true;
    next.source = SignalAnalysisSource::demodulator;

    const auto now = std::chrono::steady_clock::now();
    const std::scoped_lock lock(mutex_);
    if (smoothing_enabled_ && latest_.locked &&
        last_publish_ != std::chrono::steady_clock::time_point{}) {
        const float elapsed_seconds =
            std::chrono::duration<float>(now - last_publish_).count();
        const float alpha = std::clamp(static_cast<float>(smoothing_speed_) *
                                           elapsed_seconds / 10.0F,
                                       0.0F, 1.0F);
        next.mer_db = (alpha * next.mer_db) + ((1.0F - alpha) * latest_.mer_db);
        next.cp_snr_db =
            (alpha * next.cp_snr_db) + ((1.0F - alpha) * latest_.cp_snr_db);
        next.deepest_notch_db = (alpha * next.deepest_notch_db) +
                                ((1.0F - alpha) * latest_.deepest_notch_db);
        next.carrier_offset_hz = (alpha * next.carrier_offset_hz) +
                                 ((1.0F - alpha) * latest_.carrier_offset_hz);
    }
    next.sequence = latest_.sequence + 1;
    latest_ = next;
    last_publish_ = now;
}

void AnalysisPublisher::reset() {
    const std::scoped_lock lock(mutex_);
    const std::uint64_t sequence = latest_.sequence;
    latest_ = {};
    latest_.sequence = sequence + 1;
    last_publish_ = {};
}

void AnalysisPublisher::set_smoothing(const bool enabled, const int speed) {
    const std::scoped_lock lock(mutex_);
    smoothing_enabled_ = enabled;
    smoothing_speed_ = std::max(speed, 1);
}

bool AnalysisPublisher::locked() const {
    const std::scoped_lock lock(mutex_);
    return latest_.locked;
}

SignalAnalysisSnapshot AnalysisPublisher::snapshot() const {
    const std::scoped_lock lock(mutex_);
    return latest_;
}

} // namespace airspy_tv::dvbt
