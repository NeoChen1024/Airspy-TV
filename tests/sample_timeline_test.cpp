#include "airspy_tv/dsp/resampler_timeline.hpp"
#include "airspy_tv/sample_timeline.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

void test_input_timeline() {
    airspy_tv::InputSampleTimeline timeline;
    const auto first = timeline.stamp(4, 10'000'000);
    require(first.stream_epoch == 1 && first.begin_sample == 0 &&
                first.end_sample() == 4 && first.discontinuity_before,
            "initial stamp");
    const auto second = timeline.stamp(4, 10'000'000);
    require(second.stream_epoch == 1 && second.begin_sample == 4 &&
                !second.discontinuity_before,
            "continuous stamp");

    timeline.mark_discontinuity(7);
    const auto after_drop = timeline.stamp(3, 10'000'000);
    require(after_drop.stream_epoch == 2 && after_drop.begin_sample == 15 &&
                after_drop.discontinuity_before,
            "known dropped samples must advance source time");

    timeline.begin_stream(8'000'000);
    const auto next_stream = timeline.stamp(2, 8'000'000);
    require(next_stream.stream_epoch == 3 && next_stream.begin_sample == 18 &&
                next_stream.discontinuity_before,
            "new stream epoch must preserve monotonic source position");
    const auto snapshot = timeline.snapshot();
    require(snapshot.source_head_sample == 20 &&
                snapshot.delivered_samples == 13 &&
                snapshot.sample_rate_hz == 8'000'000,
            "input timeline snapshot");
}

void test_resampler_timeline() {
    airspy_tv::dsp::ResamplerRateTimeline timeline;
    timeline.append({.stream_epoch = 4,
                     .input_begin = 1'000,
                     .input_end = 1'200,
                     .output_begin = 0,
                     .output_end = 100,
                     .input_rate_hz = 10'000'000,
                     .applied_correction_ppm = 1.0,
                     .applied_cfo_correction_hz = 125.0});
    timeline.append({.stream_epoch = 4,
                     .input_begin = 1'200,
                     .input_end = 1'400,
                     .output_begin = 100,
                     .output_end = 200,
                     .input_rate_hz = 10'000'000,
                     .applied_correction_ppm = 3.0,
                     .applied_cfo_correction_hz = 250.0});

    const auto middle = timeline.input_at_output(50);
    require(middle.has_value() && middle->input_sample == 1'100,
            "output-to-input interpolation");
    const auto boundary = timeline.input_at_output(100);
    require(boundary.has_value() && boundary->input_sample == 1'200,
            "span boundary mapping");
    const auto average = timeline.average_correction(50, 150);
    require(average.has_value() && std::abs(*average - 2.0) < 1.0e-12,
            "weighted correction average");
    const auto cfo = timeline.cfo_correction_at(125);
    require(cfo.has_value() && *cfo == 250.0,
            "frequency correction span lookup");

    timeline.discard_before(100);
    require(timeline.size() == 1, "discard completed spans");
    timeline.truncate_after(150);
    const auto end = timeline.input_at_output(150);
    require(end.has_value() && end->input_sample == 1'300,
            "truncate preserves input mapping");
    require(!timeline.average_correction(150, 151).has_value(),
            "uncovered range must not fabricate correction");
}

} // namespace

int main() {
    try {
        test_input_timeline();
        test_resampler_timeline();
        std::cout << "Sample timeline tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Sample timeline test failed: " << error.what() << '\n';
        return 1;
    }
}
