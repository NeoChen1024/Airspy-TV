#include "clock_control_timeline.hpp"

#include <algorithm>
#include <deque>
#include <mutex>
#include <utility>

namespace airspy_tv::dvbt {

struct ClockControlTimeline::Impl {
    mutable std::mutex mutex;
    dsp::ResamplerRateTimeline timeline;
    std::deque<SroCommand> sro_commands;
    std::deque<CfoCommand> cfo_commands;
    double sro_command_ppm{};
    double sro_applied_ppm{};
    double cfo_command_hz{};
    double cfo_applied_hz{};
    std::uint32_t bandwidth_hz{};
    bool sro_ready{};
    bool cfo_ready{};
    bool rebootstrap{};
};

ClockControlTimeline::ClockControlTimeline()
    : impl_(std::make_unique<Impl>()) {}
ClockControlTimeline::~ClockControlTimeline() noexcept = default;

void ClockControlTimeline::reset() noexcept {
    const std::scoped_lock lock(impl_->mutex);
    impl_->timeline.clear();
    impl_->sro_commands.clear();
    impl_->cfo_commands.clear();
    impl_->sro_command_ppm = 0.0;
    impl_->sro_applied_ppm = 0.0;
    impl_->cfo_command_hz = 0.0;
    impl_->cfo_applied_hz = 0.0;
    impl_->bandwidth_hz = 0;
    impl_->sro_ready = false;
    impl_->cfo_ready = false;
    impl_->rebootstrap = false;
}

void ClockControlTimeline::set_bandwidth(
    const std::uint32_t bandwidth_hz) noexcept {
    const std::scoped_lock lock(impl_->mutex);
    impl_->bandwidth_hz = bandwidth_hz;
}

ClockControlTimeline::Snapshot ClockControlTimeline::snapshot() const {
    const std::scoped_lock lock(impl_->mutex);
    return {.sro_command_ppm = impl_->sro_command_ppm,
            .sro_applied_ppm = impl_->sro_applied_ppm,
            .cfo_command_hz = impl_->cfo_command_hz,
            .cfo_applied_hz = impl_->cfo_applied_hz,
            .pending_sro = impl_->sro_commands.size(),
            .pending_cfo = impl_->cfo_commands.size(),
            .bandwidth_hz = impl_->bandwidth_hz,
            .sro_ready = impl_->sro_ready,
            .cfo_ready = impl_->cfo_ready,
            .rebootstrap_requested = impl_->rebootstrap};
}

bool ClockControlTimeline::request_rebootstrap() noexcept {
    const std::scoped_lock lock(impl_->mutex);
    if (impl_->rebootstrap) {
        return false;
    }
    impl_->rebootstrap = true;
    return true;
}

bool ClockControlTimeline::consume_rebootstrap_request() noexcept {
    const std::scoped_lock lock(impl_->mutex);
    return std::exchange(impl_->rebootstrap, false);
}

bool ClockControlTimeline::rebootstrap_requested() const noexcept {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->rebootstrap;
}

void ClockControlTimeline::set_initial_cfo(
    const double correction_hz) noexcept {
    const std::scoped_lock lock(impl_->mutex);
    impl_->cfo_command_hz = correction_hz;
    impl_->cfo_ready = true;
}

void ClockControlTimeline::set_applied(
    const double sro_ppm, const double cfo_hz,
    const bool cfo_command_applied) noexcept {
    const std::scoped_lock lock(impl_->mutex);
    impl_->sro_applied_ppm = sro_ppm;
    impl_->cfo_applied_hz = cfo_hz;
    if (cfo_command_applied) {
        impl_->cfo_ready = true;
    }
}

void ClockControlTimeline::schedule_sro(SroCommand command) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->sro_command_ppm = command.target_ppm;
    impl_->sro_ready = true;
    impl_->sro_commands.push_back(command);
}

void ClockControlTimeline::replace_cfo(CfoCommand command) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->cfo_command_hz = command.target_hz;
    impl_->cfo_commands.clear();
    impl_->cfo_commands.push_back(command);
}

ClockControlTimeline::DueCommands ClockControlTimeline::consume_due(
    const std::uint64_t generation, const std::uint64_t source_epoch,
    const std::uint64_t input_begin, const std::size_t maximum_samples) {
    const std::scoped_lock lock(impl_->mutex);
    DueCommands result{.input_samples = maximum_samples,
                       .sro = std::nullopt,
                       .cfo = std::nullopt,
                       .pending_sro = 0,
                       .pending_cfo = 0};
    const auto consume = [generation, source_epoch, input_begin,
                          &result](auto &commands, auto &due) {
        while (!commands.empty()) {
            const auto &next = commands.front();
            if (next.generation != generation ||
                next.source_epoch != source_epoch) {
                commands.pop_front();
                continue;
            }
            if (next.effective_input_sample <= input_begin) {
                due = next;
                commands.pop_front();
                continue;
            }
            const std::uint64_t distance =
                next.effective_input_sample - input_begin;
            if (distance < result.input_samples) {
                result.input_samples = static_cast<std::size_t>(distance);
            }
            break;
        }
    };
    consume(impl_->sro_commands, result.sro);
    consume(impl_->cfo_commands, result.cfo);
    result.pending_sro = impl_->sro_commands.size();
    result.pending_cfo = impl_->cfo_commands.size();
    return result;
}

void ClockControlTimeline::append(dsp::ResamplerRateSpan span) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->timeline.append(span);
}

void ClockControlTimeline::truncate_after(const std::uint64_t output_sample) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->timeline.truncate_after(output_sample);
}

void ClockControlTimeline::discard_before(const std::uint64_t output_sample) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->timeline.discard_before(output_sample);
}

std::optional<dsp::MappedInputPosition>
ClockControlTimeline::input_at_output(const std::uint64_t output_sample) const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->timeline.input_at_output(output_sample);
}

std::optional<double>
ClockControlTimeline::average_correction(const std::uint64_t output_begin,
                                         const std::uint64_t output_end) const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->timeline.average_correction(output_begin, output_end);
}

std::optional<double> ClockControlTimeline::cfo_correction_at(
    const std::uint64_t output_sample) const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->timeline.cfo_correction_at(output_sample);
}

} // namespace airspy_tv::dvbt
