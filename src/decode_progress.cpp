#include "decode_progress.hpp"

#include <algorithm>
#include <format>
#include <iomanip>

namespace airspy_tv {
void format_decode_progress(std::ostream &stream,
                            const PipelineSnapshot &pipeline,
                            const std::string_view decoder_status,
                            const std::uint64_t submitted_samples,
                            const std::uint32_t sample_rate_hz,
                            const double wall_seconds) {
    const double input_seconds = sample_rate_hz == 0
                                     ? 0.0
                                     : static_cast<double>(submitted_samples) /
                                           static_cast<double>(sample_rate_hz);
    const double speed =
        wall_seconds > 0.0 ? input_seconds / wall_seconds : 0.0;
    stream << std::fixed << std::setprecision(1) << "wall=" << wall_seconds
           << "s input=" << input_seconds << "s speed=" << speed << "x";
    if (!decoder_status.empty()) {
        stream << ' ' << decoder_status;
    }
    stream << " TS="
           << static_cast<double>(pipeline.transport_bytes) / (1024.0 * 1024.0)
           << "MiB TEI=" << pipeline.transport_error_packets;
    for (std::size_t index = 0; index < pipeline.stage_count; ++index) {
        const auto &stage = pipeline.stages[index];
        const float fraction =
            stage.queue_valid ? stage.queue_fraction : stage.busy_fraction;
        stream << std::format(
            " {}={:3d}%", stage.name,
            static_cast<int>(std::clamp(100.0F * fraction, 0.0F, 100.0F)));
    }
    stream << '\n';
}

} // namespace airspy_tv
