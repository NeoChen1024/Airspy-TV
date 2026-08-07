#pragma once

#include <chrono>
#include <cstdint>

namespace airspy_tv {

// GUI-thread-owned rate tracker for monotonically increasing byte counters.
// Sampling over a fixed minimum interval avoids exposing callback-sized bursts
// as a rapidly flickering throughput value.
class ByteRateTracker {
  public:
    using Clock = std::chrono::steady_clock;

    [[nodiscard]] double update(const bool active, const std::uint64_t bytes,
                                const Clock::time_point now = Clock::now()) {
        if (!active) {
            reset();
            return 0.0;
        }
        if (!initialized_ || bytes < sampled_bytes_) {
            initialized_ = true;
            sampled_at_ = now;
            sampled_bytes_ = bytes;
            mib_per_second_ = 0.0;
            return mib_per_second_;
        }

        const double elapsed_seconds =
            std::chrono::duration<double>(now - sampled_at_).count();
        if (elapsed_seconds < minimum_sample_seconds) {
            return mib_per_second_;
        }

        const std::uint64_t written = bytes - sampled_bytes_;
        mib_per_second_ =
            static_cast<double>(written) / (bytes_per_mib * elapsed_seconds);
        sampled_at_ = now;
        sampled_bytes_ = bytes;
        return mib_per_second_;
    }

    void reset() noexcept {
        initialized_ = false;
        sampled_bytes_ = 0;
        mib_per_second_ = 0.0;
        sampled_at_ = {};
    }

  private:
    static constexpr double minimum_sample_seconds = 1.0;
    static constexpr double bytes_per_mib = 1024.0 * 1024.0;

    bool initialized_{};
    std::uint64_t sampled_bytes_{};
    double mib_per_second_{};
    Clock::time_point sampled_at_{};
};

} // namespace airspy_tv
