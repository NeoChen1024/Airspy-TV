#include "solid_resampler/frequency_translating_resampler.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <random>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using solid_resampler::FrequencyTranslatingResampler;
using solid_resampler::ResamplerConfig;

[[nodiscard]] constexpr ResamplerConfig
make_test_config(const double input_rate_hz, const double output_rate_hz) {
    const double nyquist_hz = 0.5 * std::min(input_rate_hz, output_rate_hz);
    return {
        .input_rate_hz = input_rate_hz,
        .output_rate_hz = output_rate_hz,
        .passband_edge_hz = 0.8 * nyquist_hz,
        .stopband_edge_hz = nyquist_hz,
    };
}

[[nodiscard]] constexpr ResamplerConfig
make_dvbt_config(const double channel_bandwidth_hz) {
    constexpr double input_rate_hz = 10'000'000.0;
    constexpr double active_carrier_edge_fraction = 3408.0 / 8192.0;
    const double output_rate_hz = channel_bandwidth_hz * 8.0 / 7.0;
    return {
        .input_rate_hz = input_rate_hz,
        .output_rate_hz = output_rate_hz,
        .passband_edge_hz = output_rate_hz * active_carrier_edge_fraction,
        .stopband_edge_hz = 0.5 * output_rate_hz,
        .stopband_attenuation_db = 80.0F,
    };
}

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

[[nodiscard]] std::vector<std::complex<float>>
make_input(const std::size_t count) {
    // Fixed seeds make numerical regression failures reproducible.
    // NOLINTNEXTLINE(bugprone-random-generator-seed)
    std::mt19937 generator{0x52534d50U};
    std::uniform_real_distribution<float> distribution{-1.0F, 1.0F};
    std::vector<std::complex<float>> input(count);
    for (auto &sample : input) {
        sample = {distribution(generator), distribution(generator)};
    }
    return input;
}

[[nodiscard]] std::vector<std::complex<float>>
copy_output(FrequencyTranslatingResampler &resampler,
            const std::span<const std::complex<float>> input) {
    const auto output = resampler.process(input);
    return {output.begin(), output.end()};
}

void append_output(std::vector<std::complex<float>> &destination,
                   FrequencyTranslatingResampler &resampler,
                   const std::span<const std::complex<float>> input) {
    const auto output = resampler.process(input);
    destination.insert(destination.end(), output.begin(), output.end());
}

void test_streaming_boundaries_and_workers() {
    constexpr ResamplerConfig config =
        make_test_config(10'000'000.0, 64'000'000.0 / 7.0);
    const auto input = make_input(250'003);

    FrequencyTranslatingResampler reference{1};
    reference.configure(config);
    const auto expected = copy_output(reference, input);
    require(!expected.empty(), "single-block resampler output");

    FrequencyTranslatingResampler chunked{1};
    chunked.configure(config);
    std::vector<std::complex<float>> chunked_output;
    // Fixed seeds make chunk-boundary regressions reproducible.
    // NOLINTNEXTLINE(bugprone-random-generator-seed)
    std::mt19937 generator{0x424c4f43U};
    std::uniform_int_distribution<std::size_t> chunk_size{1, 8191};
    for (std::size_t offset = 0; offset < input.size();) {
        const std::size_t count =
            std::min(chunk_size(generator), input.size() - offset);
        append_output(chunked_output, chunked,
                      std::span{input}.subspan(offset, count));
        offset += count;
    }
    require(chunked_output == expected,
            "arbitrary input block boundaries must be bit-identical");

    for (const std::size_t workers : {2U, 4U, 8U}) {
        FrequencyTranslatingResampler parallel{workers};
        parallel.configure(config);
        const auto actual = copy_output(parallel, input);
        require(actual == expected,
                "parallel output ranges must be bit-identical");
    }
}

void test_sparse_output_boundaries() {
    constexpr ResamplerConfig config =
        make_test_config(8'000'000.0, 1'000'000.0);
    const auto input = make_input(4097);
    FrequencyTranslatingResampler reference{1};
    reference.configure(config);
    const auto expected = copy_output(reference, input);

    FrequencyTranslatingResampler sample_by_sample{1};
    sample_by_sample.configure(config);
    std::vector<std::complex<float>> actual;
    for (const auto &sample : input) {
        append_output(actual, sample_by_sample, std::span{&sample, 1U});
    }
    require(actual == expected,
            "zero-output blocks must preserve phase and FIR history");
}

void test_reset_and_ratio_telemetry() {
    constexpr ResamplerConfig config =
        make_test_config(10'000'000.0, 64'000'000.0 / 7.0);
    const auto input = make_input(32'003);
    FrequencyTranslatingResampler resampler{2};
    require(!resampler.configured(), "new resampler must be unconfigured");
    resampler.configure(config);
    require(resampler.configured(), "configured resampler state");
    require(resampler.worker_count() == 2U, "resampler worker count");

    const auto first = copy_output(resampler, input);
    resampler.reset();
    const auto second = copy_output(resampler, input);
    require(second == first, "reset must reproduce initial stream state");

    constexpr double requested = 0.999'987'5;
    resampler.set_ratio(requested);
    (void)resampler.process(std::span{input}.first(4096));
    require(resampler.requested_ratio() == requested,
            "requested ratio telemetry");
    const double q32_tolerance =
        0.6 / static_cast<double>(std::uint64_t{1} << 32U);
    require(std::abs(resampler.effective_ratio() - requested) < q32_tolerance,
            "effective Q32.32 ratio precision");
    require(resampler.phase_step_q32() != 0U, "Q32.32 phase step telemetry");
}

void test_rate_change_continuity_and_counts() {
    constexpr ResamplerConfig config =
        make_test_config(1'000'000.0, 1'000'000.0);
    std::vector<std::complex<float>> constant(20'000, {1.0F, -0.25F});
    FrequencyTranslatingResampler resampler{2};
    resampler.configure(config);

    const auto before =
        copy_output(resampler, std::span{constant}.first(10'000));
    require(before.size() == 10'000U, "unity-ratio output count");
    resampler.set_ratio(0.8);
    const auto after =
        copy_output(resampler, std::span{constant}.subspan(10'000));
    require(after.size() == 8'000U, "reduced-ratio output count");

    const auto close_to_constant = [](const std::complex<float> sample) {
        return std::abs(sample - std::complex<float>{1.0F, -0.25F}) < 2.0e-3F;
    };
    require(
        std::ranges::all_of(std::span{before}.subspan(64), close_to_constant),
        "resampler must preserve steady-state gain");
    require(std::ranges::all_of(after, close_to_constant),
            "ratio update must preserve FIR history and continuity");

    resampler.reset();
    resampler.set_ratio(1.25);
    const auto expanded = resampler.process(constant);
    require(expanded.size() == 25'000U, "increased-ratio output count");

    resampler.configure(make_test_config(2'000'000.0, 1'000'000.0));
    require(resampler.process(constant).size() == 10'000U,
            "reconfiguration must apply a new nominal ratio");
}

void test_bounded_ratio_slew() {
    constexpr ResamplerConfig config =
        make_test_config(1'000'000.0, 1'000'000.0);
    const auto input = make_input(250'000);
    FrequencyTranslatingResampler resampler{1};
    resampler.configure(config);
    resampler.set_max_slew_rate(2.0);
    require(resampler.max_slew_rate() == 2.0, "configured slew telemetry");

    (void)resampler.process(input);
    resampler.set_ratio(1.0 / (1.0 + 20.0e-6));
    (void)resampler.process(input);
    const double first_correction_ppm =
        ((1.0 / resampler.effective_ratio()) - 1.0) * 1.0e6;
    require(first_correction_ppm > 0.45 && first_correction_ppm < 0.55,
            "slew must be limited by elapsed input duration");

    for (int block = 0; block < 39; ++block) {
        (void)resampler.process(input);
    }
    const double final_correction_ppm =
        ((1.0 / resampler.effective_ratio()) - 1.0) * 1.0e6;
    require(std::abs(final_correction_ppm - 20.0) < 0.01,
            "bounded slew must converge to the requested ratio");

    resampler.reset();
    resampler.set_ratio(1.0);
    (void)resampler.process(input);
    require(std::abs(resampler.effective_ratio() - 1.0) < 1.0e-9,
            "reset must clear pending slew state");
}

[[nodiscard]] float
maximum_error(const std::span<const std::complex<float>> left,
              const std::span<const std::complex<float>> right) {
    require(left.size() == right.size(), "frequency-shift output size");
    float result = 0.0F;
    for (std::size_t index = 0; index < left.size(); ++index) {
        result = std::max(result, std::abs(left[index] - right[index]));
    }
    return result;
}

void test_frequency_translation() {
    constexpr ResamplerConfig config = make_test_config(1'000'000.0, 800'000.0);
    constexpr double input_tone_hz = 137'500.0;
    constexpr double shift_hz = -82'250.0;
    constexpr double expected_tone_hz = input_tone_hz + shift_hz;
    std::vector<std::complex<float>> input(50'003);
    for (std::size_t index = 0; index < input.size(); ++index) {
        const double phase = 2.0 * std::numbers::pi * input_tone_hz *
                             static_cast<double>(index) / config.input_rate_hz;
        input[index] = {static_cast<float>(std::cos(phase)),
                        static_cast<float>(std::sin(phase))};
    }

    FrequencyTranslatingResampler reference{1};
    reference.configure(config);
    reference.set_frequency_shift(shift_hz);
    const auto expected = copy_output(reference, input);
    require(reference.requested_frequency_shift() == shift_hz,
            "requested frequency-shift telemetry");
    require(std::abs(reference.effective_frequency_shift() - shift_hz) < 1.0e-6,
            "effective Q0.64 frequency-shift telemetry");

    const std::size_t skip = 2U * reference.filter_taps();
    std::complex<double> phase_step_sum{};
    for (std::size_t index = skip + 1U; index < expected.size(); ++index) {
        phase_step_sum +=
            static_cast<std::complex<double>>(expected[index]) *
            std::conj(static_cast<std::complex<double>>(expected[index - 1U]));
    }
    const double measured_hz = std::arg(phase_step_sum) *
                               config.output_rate_hz / (2.0 * std::numbers::pi);
    require(std::abs(measured_hz - expected_tone_hz) < 0.1,
            "translated output tone frequency");

    FrequencyTranslatingResampler chunked{1};
    chunked.configure(config);
    chunked.set_frequency_shift(shift_hz);
    std::vector<std::complex<float>> chunked_output;
    for (std::size_t offset = 0; offset < input.size();) {
        const std::size_t count =
            std::min<std::size_t>(4093, input.size() - offset);
        append_output(chunked_output, chunked,
                      std::span{input}.subspan(offset, count));
        offset += count;
    }
    const float chunked_error = maximum_error(chunked_output, expected);
    if (chunked_error >= 2.0e-4F) {
        throw std::runtime_error(
            "mixer phase continuity across input blocks: " +
            std::to_string(chunked_error));
    }

    FrequencyTranslatingResampler parallel{4};
    parallel.configure(config);
    parallel.set_frequency_shift(shift_hz);
    const auto parallel_output = copy_output(parallel, input);
    const float parallel_error = maximum_error(parallel_output, expected);
    if (parallel_error >= 2.0e-4F) {
        throw std::runtime_error("parallel mixer phase seeding: " +
                                 std::to_string(parallel_error));
    }

    FrequencyTranslatingResampler zero_shift{4};
    zero_shift.configure(config);
    const auto unshifted = copy_output(zero_shift, input);
    zero_shift.reset();
    zero_shift.set_frequency_shift(0.0);
    const auto explicit_zero = copy_output(zero_shift, input);
    require(explicit_zero == unshifted,
            "zero frequency shift must preserve bit-identical output");

    constexpr ResamplerConfig unity_config =
        make_test_config(1'000'000.0, 1'000'000.0);
    constexpr double first_shift_hz = 73'125.0;
    constexpr double second_shift_hz = -41'875.0;
    constexpr std::size_t command_boundary = 12'000;
    std::vector<std::complex<float>> constant(24'000, {1.0F, 0.0F});
    FrequencyTranslatingResampler changing{4};
    changing.configure(unity_config);
    changing.set_frequency_shift(first_shift_hz);
    const auto first =
        copy_output(changing, std::span{constant}.first(command_boundary));
    changing.set_frequency_shift(second_shift_hz);
    const auto second =
        copy_output(changing, std::span{constant}.subspan(command_boundary));
    require(first.size() == command_boundary &&
                second.size() == constant.size() - command_boundary,
            "unity-ratio frequency-command output count");
    const double boundary_phase = 2.0 * std::numbers::pi * first_shift_hz *
                                  static_cast<double>(command_boundary) /
                                  unity_config.output_rate_hz;
    const std::complex<float> expected_boundary{
        static_cast<float>(std::cos(boundary_phase)),
        static_cast<float>(std::sin(boundary_phase))};
    require(std::abs(second.front() - expected_boundary) < 2.0e-3F,
            "frequency command update reset the mixer phase");
}

[[nodiscard]] double measure_tone_gain(FrequencyTranslatingResampler &resampler,
                                       const ResamplerConfig &config,
                                       const double frequency_hz) {
    constexpr std::size_t input_count = 32'768;
    std::vector<std::complex<float>> input(input_count);
    for (std::size_t index = 0; index < input.size(); ++index) {
        const double phase = 2.0 * std::numbers::pi * frequency_hz *
                             static_cast<double>(index) / config.input_rate_hz;
        input[index] = {static_cast<float>(std::cos(phase)),
                        static_cast<float>(std::sin(phase))};
    }
    resampler.reset();
    const auto output = resampler.process(input);
    const std::size_t skip =
        std::min(2U * resampler.filter_taps(), output.size());
    double power = 0.0;
    for (const auto sample : output.subspan(skip)) {
        power += std::norm(sample);
    }
    return std::sqrt(power / static_cast<double>(output.size() - skip));
}

void test_dvbt_stopband_attenuation() {
    constexpr std::array bandwidths{5'000'000.0, 6'000'000.0, 7'000'000.0,
                                    8'000'000.0};
    constexpr std::array<std::size_t, 4> expected_taps{128, 112, 96, 96};
    constexpr double maximum_stopband_gain = 1.0e-4;

    for (std::size_t index = 0; index < bandwidths.size(); ++index) {
        const auto config = make_dvbt_config(bandwidths[index]);
        FrequencyTranslatingResampler resampler{1};
        resampler.configure(config);
        require(resampler.filter_taps() == expected_taps[index],
                "DVB-T 80 dB filter tap estimate");
        require(resampler.filter_taps() % 16U == 0U,
                "filter taps must be SIMD-friendly");

        const double passband_gain =
            measure_tone_gain(resampler, config, config.passband_edge_hz);
        require(std::abs(passband_gain - 1.0) < 1.0e-3,
                "DVB-T passband-edge gain");

        double worst_stopband_gain = 0.0;
        constexpr std::size_t sweep_intervals = 16;
        for (std::size_t step = 0; step <= sweep_intervals; ++step) {
            const double frequency_hz =
                config.stopband_edge_hz +
                (((0.5 * config.input_rate_hz) - config.stopband_edge_hz) *
                 static_cast<double>(step) /
                 static_cast<double>(sweep_intervals));
            worst_stopband_gain =
                std::max(worst_stopband_gain,
                         measure_tone_gain(resampler, config, frequency_hz));
        }
        if (worst_stopband_gain > maximum_stopband_gain) {
            throw std::runtime_error(
                "DVB-T stopband missed 80 dB target: " +
                std::to_string(20.0 * std::log10(worst_stopband_gain)) + " dB");
        }
    }
}

} // namespace

int main() {
    try {
        test_streaming_boundaries_and_workers();
        test_sparse_output_boundaries();
        test_reset_and_ratio_telemetry();
        test_rate_change_continuity_and_counts();
        test_bounded_ratio_slew();
        test_frequency_translation();
        test_dvbt_stopband_attenuation();
        std::cout << "Frequency-translating resampler tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Frequency-translating resampler test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
