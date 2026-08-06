#include "pipeline_load_monitor.hpp"

#include <cstdlib>
#include <iostream>
#include <string_view>

namespace {

using airspy_tv::PipelineLoadMonitor;
using airspy_tv::PipelineLoadSample;
using airspy_tv::PipelineLoadState;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "Pipeline load monitor test failed: " << message << '\n';
        std::exit(1);
    }
}

PipelineLoadSample sample(const std::uint64_t sequence, const float ratio,
                          const float queue_fraction,
                          const std::uint64_t drops = 0) {
    return {.active = true,
            .realtime_ratio = ratio,
            .input_queue_fraction = queue_fraction,
            .fec_queue_fraction = 0.0F,
            .dropped_blocks = drops,
            .sequence = sequence};
}

void test_isolated_slow_window_is_debounced() {
    PipelineLoadMonitor monitor;
    require(monitor.update(sample(1, 1.0F, 0.0F)) ==
                PipelineLoadState::realtime,
            "first valid window should establish realtime state");
    require(monitor.update(sample(2, 1.30F, 0.40F)) ==
                PipelineLoadState::realtime,
            "one slow window must not flash the indicator");
    require(monitor.update(sample(2, 1.30F, 0.40F)) ==
                PipelineLoadState::realtime,
            "GUI polls of one stats window must not count repeatedly");
    require(monitor.update(sample(3, 1.0F, 0.0F)) ==
                PipelineLoadState::realtime,
            "an isolated slow window must clear without a transition");
}

void test_sustained_pressure_transitions_and_recovers() {
    PipelineLoadMonitor monitor;
    static_cast<void>(monitor.update(sample(1, 1.0F, 0.0F)));
    require(monitor.update(sample(2, 1.20F, 0.30F)) ==
                PipelineLoadState::realtime,
            "slow transition should require three windows");
    require(monitor.update(sample(3, 1.20F, 0.30F)) ==
                PipelineLoadState::realtime,
            "slow transition should remain debounced after two windows");
    require(monitor.update(sample(4, 1.20F, 0.30F)) ==
                PipelineLoadState::slow,
            "three slow pressured windows should report slow");
    require(monitor.update(sample(5, 1.0F, 0.05F)) ==
                PipelineLoadState::slow,
            "slow state should not clear on one healthy window");
    require(monitor.update(sample(6, 1.0F, 0.05F)) ==
                PipelineLoadState::slow,
            "slow state should not clear on two healthy windows");
    require(monitor.update(sample(7, 1.0F, 0.05F)) ==
                PipelineLoadState::realtime,
            "slow state should clear after sustained recovery");
}

void test_slow_wall_ratio_without_backlog_stays_realtime() {
    PipelineLoadMonitor monitor;
    for (std::uint64_t sequence = 1; sequence <= 6; ++sequence) {
        require(monitor.update(sample(sequence, 1.25F, 0.0F)) ==
                    PipelineLoadState::realtime,
                "wall-time jitter without queue pressure is not CPU slowness");
    }
}

void test_overload_evidence_is_immediate() {
    PipelineLoadMonitor monitor;
    static_cast<void>(monitor.update(sample(1, 1.0F, 0.0F)));
    require(monitor.update(sample(2, 1.0F, 0.80F)) ==
                PipelineLoadState::overload,
            "near-full queue should report overload immediately");

    PipelineLoadMonitor drop_monitor;
    static_cast<void>(drop_monitor.update(sample(1, 1.0F, 0.0F, 4)));
    require(drop_monitor.update(sample(1, 1.0F, 0.0F, 5)) ==
                PipelineLoadState::overload,
            "a new input drop should report overload without a new window");
}

void test_inactive_decoder_returns_to_measuring() {
    PipelineLoadMonitor monitor;
    static_cast<void>(monitor.update(sample(1, 1.0F, 0.0F)));
    PipelineLoadSample inactive;
    require(monitor.update(inactive) == PipelineLoadState::measuring,
            "inactive decoder should not retain a realtime claim");
}

} // namespace

int main() {
    test_isolated_slow_window_is_debounced();
    test_sustained_pressure_transitions_and_recovers();
    test_slow_wall_ratio_without_backlog_stays_realtime();
    test_overload_evidence_is_immediate();
    test_inactive_decoder_returns_to_measuring();
    std::cout << "Pipeline load monitor tests passed\n";
    return 0;
}
