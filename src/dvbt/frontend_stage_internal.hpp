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
