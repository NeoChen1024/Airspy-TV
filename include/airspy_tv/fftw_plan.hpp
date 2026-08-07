#pragma once

#include <fftw3.h>

#include <mutex>
#include <new>
#include <utility>

namespace airspy_tv {

// FFTW plan execution is safe on independent plans, but planner creation and
// destruction share process-global state. Serialize only those lifecycle
// operations; execute() remains lock-free.
class FftwfPlan {
  public:
    FftwfPlan() noexcept = default;

    static FftwfPlan dft_1d(const int size, fftwf_complex *input,
                            fftwf_complex *output, const int sign,
                            const unsigned flags) {
        std::scoped_lock const lock(planner_mutex());
        fftwf_plan plan = fftwf_plan_dft_1d(size, input, output, sign, flags);
        if (plan == nullptr) {
            throw std::bad_alloc();
        }
        return FftwfPlan(plan);
    }

    ~FftwfPlan() noexcept { reset(); }

    FftwfPlan(const FftwfPlan &) = delete;
    FftwfPlan &operator=(const FftwfPlan &) = delete;

    FftwfPlan(FftwfPlan &&other) noexcept
        : plan_(std::exchange(other.plan_, nullptr)) {}

    FftwfPlan &operator=(FftwfPlan &&other) noexcept {
        if (this != &other) {
            reset();
            plan_ = std::exchange(other.plan_, nullptr);
        }
        return *this;
    }

    void execute() const noexcept { fftwf_execute(plan_); }

    void reset() noexcept {
        if (plan_ == nullptr) {
            return;
        }
        std::scoped_lock const lock(planner_mutex());
        fftwf_destroy_plan(std::exchange(plan_, nullptr));
    }

    [[nodiscard]] explicit operator bool() const noexcept {
        return plan_ != nullptr;
    }

  private:
    explicit FftwfPlan(fftwf_plan plan) noexcept : plan_(plan) {}

    static std::mutex &planner_mutex() noexcept {
        static std::mutex mutex;
        return mutex;
    }

    fftwf_plan plan_{};
};

} // namespace airspy_tv
