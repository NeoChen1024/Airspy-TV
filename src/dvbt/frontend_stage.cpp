#include "frontend_stage_internal.hpp"

#include <exception>
#include <memory>
#include <utility>

namespace airspy_tv::dvbt {

FrontendStage::Impl::Impl(SampleChannel &selected_samples,
                          ClockControlTimeline &selected_clock,
                          Callbacks selected_callbacks)
    : samples(selected_samples), clock(selected_clock),
      callbacks(std::move(selected_callbacks)) {
    if (!callbacks.parameters || !callbacks.input_block_started ||
        !callbacks.invalidate_sync || !callbacks.publish_rebootstrap ||
        !callbacks.publish_bootstrap_progress ||
        !callbacks.publish_acquisition || !callbacks.publish_command ||
        !callbacks.publish_block || !callbacks.clear_fec ||
        !callbacks.notify_fec_cancelled || !callbacks.worker_failure) {
        throw std::invalid_argument("incomplete frontend stage callbacks");
    }
    worker = std::thread([this] {
        set_current_thread_name("dvbt-frontend");
        run_guarded();
    });
}

FrontendStage::Impl::~Impl() noexcept { stop(); }

void FrontendStage::Impl::stop() noexcept {
    samples.stop();
    if (worker.joinable()) {
        worker.join();
    }
}

void FrontendStage::Impl::run_guarded() noexcept {
    try {
        run();
    } catch (...) {
        const std::exception_ptr error = std::current_exception();
        std::string message = "frontend worker failed";
        try {
            std::rethrow_exception(error);
        } catch (const std::exception &exception) {
            message += ": ";
            message += exception.what();
        } catch (...) {
            message += ": unknown exception";
        }
        callbacks.worker_failure(error, std::move(message));
    }
}

FrontendStage::FrontendStage(SampleChannel &samples,
                             ClockControlTimeline &clock, Callbacks callbacks)
    : impl_(std::make_unique<Impl>(samples, clock, std::move(callbacks))) {}

FrontendStage::~FrontendStage() noexcept = default;

void FrontendStage::stop() noexcept { impl_->stop(); }

WorkerState FrontendStage::worker_state() const noexcept {
    return static_cast<WorkerState>(
        impl_->state.load(std::memory_order_relaxed));
}

} // namespace airspy_tv::dvbt
