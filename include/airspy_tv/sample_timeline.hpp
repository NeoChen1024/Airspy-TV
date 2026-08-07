#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>

namespace airspy_tv {

struct InputSampleStamp {
    std::uint64_t stream_epoch{};
    std::uint64_t begin_sample{};
    std::uint64_t sample_count{};
    std::uint32_t sample_rate_hz{};
    bool discontinuity_before{};

    [[nodiscard]] std::uint64_t end_sample() const noexcept {
        return begin_sample + sample_count;
    }

    [[nodiscard]] bool valid_for(const std::size_t count,
                                 const std::uint32_t rate) const noexcept {
        return stream_epoch != 0 && sample_count == count &&
               sample_rate_hz == rate;
    }
};

struct InputTimelineSnapshot {
    std::uint64_t stream_epoch{};
    std::uint64_t source_head_sample{};
    std::uint64_t delivered_samples{};
    std::uint32_t sample_rate_hz{};
};

// Per-receiver source timeline. The absolute source position never rewinds;
// epochs mark points across which sample continuity is not guaranteed.
class InputSampleTimeline {
  public:
    void begin_stream(const std::uint32_t sample_rate_hz) {
        const std::scoped_lock lock(mutex_);
        ++stream_epoch_;
        sample_rate_hz_ = sample_rate_hz;
        discontinuity_before_next_ = true;
    }

    void mark_discontinuity(const std::uint64_t known_dropped_samples = 0) {
        const std::scoped_lock lock(mutex_);
        source_head_sample_ += known_dropped_samples;
        ++stream_epoch_;
        discontinuity_before_next_ = true;
    }

    [[nodiscard]] InputSampleStamp stamp(const std::size_t sample_count,
                                         const std::uint32_t sample_rate_hz) {
        const std::scoped_lock lock(mutex_);
        if (stream_epoch_ == 0) {
            stream_epoch_ = 1;
            discontinuity_before_next_ = true;
        }
        if (sample_rate_hz_ != sample_rate_hz) {
            if (sample_rate_hz_ != 0) {
                ++stream_epoch_;
                discontinuity_before_next_ = true;
            }
            sample_rate_hz_ = sample_rate_hz;
        }
        const InputSampleStamp result{
            .stream_epoch = stream_epoch_,
            .begin_sample = source_head_sample_,
            .sample_count = sample_count,
            .sample_rate_hz = sample_rate_hz_,
            .discontinuity_before = discontinuity_before_next_,
        };
        source_head_sample_ += sample_count;
        delivered_samples_ += sample_count;
        discontinuity_before_next_ = false;
        return result;
    }

    [[nodiscard]] InputTimelineSnapshot snapshot() const {
        const std::scoped_lock lock(mutex_);
        return {
            .stream_epoch = stream_epoch_,
            .source_head_sample = source_head_sample_,
            .delivered_samples = delivered_samples_,
            .sample_rate_hz = sample_rate_hz_,
        };
    }

  private:
    mutable std::mutex mutex_;
    std::uint64_t stream_epoch_{};
    std::uint64_t source_head_sample_{};
    std::uint64_t delivered_samples_{};
    std::uint32_t sample_rate_hz_{};
    bool discontinuity_before_next_{};
};

} // namespace airspy_tv
