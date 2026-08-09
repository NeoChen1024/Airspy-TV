#pragma once

#include "airspy_tv/dsp/resampler_timeline.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace airspy_tv::dvbt {

class ClockControlTimeline {
  public:
    struct SroCommand {
        std::uint64_t generation{};
        std::uint64_t source_epoch{};
        std::uint64_t command_output_sample{};
        std::uint64_t command_input_sample{};
        std::uint64_t effective_input_sample{};
        std::uint64_t fixed_delay_samples{};
        double target_ppm{};
    };

    struct CfoCommand {
        std::uint64_t generation{};
        std::uint64_t source_epoch{};
        std::uint64_t command_output_sample{};
        std::uint64_t command_input_sample{};
        std::uint64_t effective_input_sample{};
        std::uint64_t fixed_delay_samples{};
        double target_hz{};
    };

    struct DueCommands {
        std::size_t input_samples{};
        std::optional<SroCommand> sro;
        std::optional<CfoCommand> cfo;
        std::size_t pending_sro{};
        std::size_t pending_cfo{};
    };

    struct Snapshot {
        double sro_command_ppm{};
        double sro_applied_ppm{};
        double cfo_command_hz{};
        double cfo_applied_hz{};
        std::size_t pending_sro{};
        std::size_t pending_cfo{};
        std::uint32_t bandwidth_hz{};
        bool sro_ready{};
        bool cfo_ready{};
        bool rebootstrap_requested{};
    };

    ClockControlTimeline();
    ~ClockControlTimeline() noexcept;

    ClockControlTimeline(const ClockControlTimeline &) = delete;
    ClockControlTimeline &operator=(const ClockControlTimeline &) = delete;
    ClockControlTimeline(ClockControlTimeline &&) = delete;
    ClockControlTimeline &operator=(ClockControlTimeline &&) = delete;

    void reset() noexcept;
    void set_bandwidth(std::uint32_t bandwidth_hz) noexcept;
    [[nodiscard]] Snapshot snapshot() const;

    [[nodiscard]] bool request_rebootstrap() noexcept;
    [[nodiscard]] bool consume_rebootstrap_request() noexcept;
    [[nodiscard]] bool rebootstrap_requested() const noexcept;

    void set_initial_cfo(double correction_hz) noexcept;
    void set_applied(double sro_ppm, double cfo_hz,
                     bool cfo_command_applied) noexcept;

    void schedule_sro(SroCommand command);
    void replace_cfo(CfoCommand command);
    [[nodiscard]] DueCommands consume_due(std::uint64_t generation,
                                          std::uint64_t source_epoch,
                                          std::uint64_t input_begin,
                                          std::size_t maximum_samples);

    void append(dsp::ResamplerRateSpan span);
    void truncate_after(std::uint64_t output_sample);
    void discard_before(std::uint64_t output_sample);
    [[nodiscard]] std::optional<dsp::MappedInputPosition>
    input_at_output(std::uint64_t output_sample) const;
    [[nodiscard]] std::optional<double>
    average_correction(std::uint64_t output_begin,
                       std::uint64_t output_end) const;
    [[nodiscard]] std::optional<double>
    cfo_correction_at(std::uint64_t output_sample) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
