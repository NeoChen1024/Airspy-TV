#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>

namespace airspy_tv::dsp {

struct ResamplerRateSpan {
    std::uint64_t stream_epoch{};
    std::uint64_t input_begin{};
    std::uint64_t input_end{};
    std::uint64_t output_begin{};
    std::uint64_t output_end{};
    std::uint32_t input_rate_hz{};
    double applied_correction_ppm{};
    double applied_cfo_correction_hz{};
};

struct MappedInputPosition {
    std::uint64_t stream_epoch{};
    std::uint64_t input_sample{};
    std::uint32_t input_rate_hz{};
};

// Maps common-resampler output coordinates back to stamped source samples and
// retains the correction which actually produced each output range. External
// synchronization is intentional so callers can update this alongside their
// sample ring under one lock.
class ResamplerRateTimeline {
  public:
    void clear() noexcept { spans_.clear(); }

    void append(const ResamplerRateSpan span) {
        if (span.input_end <= span.input_begin ||
            span.output_end <= span.output_begin) {
            return;
        }
        spans_.push_back(span);
    }

    [[nodiscard]] std::optional<MappedInputPosition>
    input_at_output(const std::uint64_t output_sample) const noexcept {
        const auto item = std::find_if(
            spans_.rbegin(), spans_.rend(), [output_sample](const auto &span) {
                return output_sample >= span.output_begin &&
                       output_sample <= span.output_end;
            });
        if (item == spans_.rend()) {
            return std::nullopt;
        }
        const auto &span = *item;
        if (output_sample == span.output_end) {
            return MappedInputPosition{span.stream_epoch, span.input_end,
                                       span.input_rate_hz};
        }
        const auto output_offset =
            static_cast<long double>(output_sample - span.output_begin);
        const auto input_count =
            static_cast<long double>(span.input_end - span.input_begin);
        const auto output_count =
            static_cast<long double>(span.output_end - span.output_begin);
        const auto input_offset = static_cast<std::uint64_t>(
            std::floor(output_offset * input_count / output_count));
        return MappedInputPosition{span.stream_epoch,
                                   span.input_begin + input_offset,
                                   span.input_rate_hz};
    }

    [[nodiscard]] std::optional<double>
    average_correction(const std::uint64_t output_begin,
                       const std::uint64_t output_end) const noexcept {
        if (output_end <= output_begin) {
            return std::nullopt;
        }
        long double weighted_sum = 0.0L;
        std::uint64_t covered = 0;
        for (const auto &span : spans_) {
            const std::uint64_t begin =
                std::max(output_begin, span.output_begin);
            const std::uint64_t end = std::min(output_end, span.output_end);
            if (end <= begin) {
                continue;
            }
            const std::uint64_t count = end - begin;
            weighted_sum +=
                static_cast<long double>(count) * span.applied_correction_ppm;
            covered += count;
        }
        if (covered != output_end - output_begin) {
            return std::nullopt;
        }
        return static_cast<double>(weighted_sum /
                                   static_cast<long double>(covered));
    }

    [[nodiscard]] std::optional<double>
    cfo_correction_at(const std::uint64_t output_sample) const noexcept {
        const auto item = std::find_if(
            spans_.rbegin(), spans_.rend(), [output_sample](const auto &span) {
                return output_sample >= span.output_begin &&
                       output_sample < span.output_end;
            });
        return item == spans_.rend()
                   ? std::nullopt
                   : std::optional<double>{item->applied_cfo_correction_hz};
    }

    void discard_before(const std::uint64_t output_sample) {
        while (!spans_.empty() && spans_.front().output_end <= output_sample) {
            spans_.pop_front();
        }
    }

    void truncate_after(const std::uint64_t output_sample) {
        while (!spans_.empty() && spans_.back().output_begin >= output_sample) {
            spans_.pop_back();
        }
        if (spans_.empty() || spans_.back().output_end <= output_sample) {
            return;
        }
        auto &span = spans_.back();
        const auto retained_output =
            static_cast<long double>(output_sample - span.output_begin);
        const auto total_output =
            static_cast<long double>(span.output_end - span.output_begin);
        const auto total_input =
            static_cast<long double>(span.input_end - span.input_begin);
        span.input_end = span.input_begin +
                         static_cast<std::uint64_t>(std::ceil(
                             retained_output * total_input / total_output));
        span.output_end = output_sample;
    }

    [[nodiscard]] std::size_t size() const noexcept { return spans_.size(); }

  private:
    std::deque<ResamplerRateSpan> spans_;
};

} // namespace airspy_tv::dsp
