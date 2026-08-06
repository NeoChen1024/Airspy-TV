#pragma once

#include <algorithm>
#include <cstdint>

namespace airspy_tv {

enum class PipelineLoadState {
    measuring,
    realtime,
    slow,
    overload,
};

struct PipelineLoadSample {
    bool active{};
    float realtime_ratio{};
    // Highest pressure among the active demodulator's published queues. The
    // monitor intentionally does not assume a fixed frontend/demod/FEC shape.
    float queue_pressure_fraction{};
    std::uint64_t dropped_blocks{};
    std::uint64_t sequence{};
};

// Debounces the decoder's windowed throughput estimate. Live input naturally
// makes wall time track signal time even when the CPU is mostly waiting, so a
// ratio just above 1.0 is not overload evidence by itself. Queue pressure must
// accompany a sustained slow ratio; a nearly full queue or a new drop remains
// an immediate overload signal.
class PipelineLoadMonitor {
  public:
    [[nodiscard]] PipelineLoadState update(const PipelineLoadSample &sample) {
        if (!sample.active || sample.realtime_ratio <= 0.0F) {
            reset();
            return state_;
        }

        const bool restarted = !initialized_ || sample.sequence < sequence_ ||
                               sample.dropped_blocks < dropped_blocks_;
        if (restarted) {
            initialized_ = true;
            state_ = PipelineLoadState::realtime;
            slow_windows_ = 0;
            recovery_windows_ = 0;
        }

        const bool new_drop = initialized_ && !restarted &&
                              sample.dropped_blocks > dropped_blocks_;
        const bool new_window = restarted || sample.sequence != sequence_;
        sequence_ = sample.sequence;
        dropped_blocks_ = sample.dropped_blocks;

        if (new_drop) {
            state_ = PipelineLoadState::overload;
            slow_windows_ = 0;
            recovery_windows_ = 0;
            return state_;
        }
        if (!new_window) {
            return state_;
        }

        const float pressure =
            std::clamp(sample.queue_pressure_fraction, 0.0F, 1.0F);
        constexpr float overload_pressure = 0.75F;
        constexpr float overload_release_pressure = 0.60F;
        constexpr float slow_enter_ratio = 1.10F;
        constexpr float slow_enter_pressure = 0.25F;
        constexpr float slow_hold_ratio = 1.03F;
        constexpr float slow_hold_pressure = 0.15F;
        constexpr unsigned int transition_windows = 3;

        const bool slow_evidence = sample.realtime_ratio >= slow_enter_ratio &&
                                   pressure >= slow_enter_pressure;
        const bool slow_hold = sample.realtime_ratio > slow_hold_ratio &&
                               pressure > slow_hold_pressure;

        if (pressure >= overload_pressure) {
            state_ = PipelineLoadState::overload;
            slow_windows_ = 0;
            recovery_windows_ = 0;
            return state_;
        }

        if (state_ == PipelineLoadState::overload) {
            if (pressure >= overload_release_pressure) {
                recovery_windows_ = 0;
            } else if (++recovery_windows_ >= transition_windows) {
                state_ = slow_hold ? PipelineLoadState::slow
                                   : PipelineLoadState::realtime;
                recovery_windows_ = 0;
            }
            return state_;
        }

        if (state_ == PipelineLoadState::slow) {
            if (slow_hold) {
                recovery_windows_ = 0;
            } else if (++recovery_windows_ >= transition_windows) {
                state_ = PipelineLoadState::realtime;
                recovery_windows_ = 0;
            }
            return state_;
        }

        if (slow_evidence) {
            if (++slow_windows_ >= transition_windows) {
                state_ = PipelineLoadState::slow;
                slow_windows_ = 0;
            }
        } else {
            slow_windows_ = 0;
        }
        return state_;
    }

    void reset() noexcept {
        state_ = PipelineLoadState::measuring;
        initialized_ = false;
        slow_windows_ = 0;
        recovery_windows_ = 0;
        sequence_ = 0;
        dropped_blocks_ = 0;
    }

  private:
    PipelineLoadState state_{PipelineLoadState::measuring};
    bool initialized_{};
    unsigned int slow_windows_{};
    unsigned int recovery_windows_{};
    std::uint64_t sequence_{};
    std::uint64_t dropped_blocks_{};
};

} // namespace airspy_tv
