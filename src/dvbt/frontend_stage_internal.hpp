#pragma once

#include "frontend_stage.hpp"

#include "airspy_tv/dsp/vector_ops.hpp"
#include "airspy_tv/thread_name.hpp"

#include "pipeline_config.hpp"
#include "streaming_resampler.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iterator>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {

[[nodiscard]] inline float frontend_duration_ms(
    const std::chrono::steady_clock::time_point started_at) noexcept {
    return std::chrono::duration<float, std::milli>(
               std::chrono::steady_clock::now() - started_at)
        .count();
}

// Source conditioning ahead of the mixer. A mirrored stream (I and Q swapped
// by the radio or the file format) is conjugated so the pilot pattern reads
// the right way round. A slow estimate of the radio's DC offset is subtracted
// only when the channel sits away from the source centre: a centred channel
// puts its own centre carrier at DC (a boosted continual pilot in 8K mode),
// and the demodulator already tolerates a spike on one carrier. Both act at
// the source rate before any frequency translation.
struct InputConditioner {
    std::complex<float> dc{};
    bool remove_dc{};
    bool mirror{};

    void reset_dc() noexcept { dc = {}; }

    void apply(const std::span<std::complex<float>> block,
               const std::uint32_t sample_rate_hz) noexcept {
        if (block.empty()) {
            return;
        }
        if (remove_dc) {
            // The block mean moves the estimate with a 50 ms time constant;
            // a block longer than that replaces it outright.
            std::complex<float> sum{};
            for (const auto &value : block) {
                sum += value;
            }
            const float weight = std::min(
                1.0F, static_cast<float>(block.size()) /
                          (0.05F * static_cast<float>(sample_rate_hz)));
            dc += (sum / static_cast<float>(block.size()) - dc) * weight;
        }
        const std::complex<float> offset = dc;
        if (mirror) {
            for (auto &value : block) {
                value = std::conj(value - offset);
            }
        } else if (remove_dc) {
            for (auto &value : block) {
                value -= offset;
            }
        }
    }
};

struct FrontendStage::Impl {
    Impl(SampleChannel &samples, ClockControlTimeline &clock,
         Callbacks callbacks);
    ~Impl() noexcept;

    void stop() noexcept;
    void run_guarded() noexcept;
    void run();

    SampleChannel &samples;
    ClockControlTimeline &clock;
    Callbacks callbacks;
    std::atomic<int> state{static_cast<int>(WorkerState::idle)};
    std::thread worker;
};

} // namespace airspy_tv::dvbt
