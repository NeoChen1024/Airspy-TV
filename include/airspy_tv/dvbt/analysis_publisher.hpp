#pragma once

#include "airspy_tv/dvbt/signal_analyzer.hpp"

#include <chrono>
#include <complex>
#include <cstddef>
#include <mutex>
#include <span>

namespace airspy_tv::dvbt {

// Immutable GUI snapshots published by the production demod/postprocessor
// path. The independent SignalAnalyzer remains only as a pre-lock monitor.
class AnalysisPublisher {
  public:
    void publish(std::span<const std::complex<float>> carriers, float mer_db,
                 float cp_snr_db, float deepest_notch_db,
                 float carrier_offset_hz, TransmissionMode mode,
                 GuardInterval guard_interval, Constellation constellation);
    void reset();
    void set_smoothing(bool enabled, int speed);

    [[nodiscard]] bool locked() const;
    [[nodiscard]] SignalAnalysisSnapshot snapshot() const;

  private:
    mutable std::mutex mutex_;
    SignalAnalysisSnapshot latest_;
    std::chrono::steady_clock::time_point last_publish_{};
    bool smoothing_enabled_{true};
    int smoothing_speed_{20};
};

} // namespace airspy_tv::dvbt
