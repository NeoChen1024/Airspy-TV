#include "telemetry_stream_router.hpp"

#include "airspy_tv/jsonl.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

namespace airspy_tv {

struct TelemetryStreamRouter::Impl {
    struct Stream {
        std::ofstream output;
        std::unique_ptr<JsonlWriter> writer;
    };

    Impl(const std::filesystem::path &directory,
         std::vector<TelemetryStreamSpec> selected_specs)
        : specs(std::move(selected_specs)) {
        streams.reserve(specs.size());
        for (const auto &spec : specs) {
            streams.emplace_back();
            auto &stream = streams.back();
            stream.output.open(directory / spec.path,
                               std::ios::binary | std::ios::trunc);
            if (!stream.output) {
                throw std::runtime_error("Unable to create report stream: " +
                                         spec.path.string());
            }
            stream.writer = std::make_unique<JsonlWriter>(stream.output);
        }
    }

    [[nodiscard]] std::size_t index(const std::string &key) const {
        for (std::size_t index = 0; index < specs.size(); ++index) {
            if (specs[index].key == key) {
                return index;
            }
        }
        throw std::logic_error("Unknown telemetry stream key: " + key);
    }

    std::vector<TelemetryStreamSpec> specs;
    std::vector<Stream> streams;
};

TelemetryStreamRouter::TelemetryStreamRouter(
    std::filesystem::path directory, std::vector<TelemetryStreamSpec> streams)
    : impl_(std::make_unique<Impl>(directory, std::move(streams))) {}

TelemetryStreamRouter::~TelemetryStreamRouter() noexcept = default;

void TelemetryStreamRouter::write(const std::string &key,
                                  const nlohmann::json &record) {
    impl_->streams[impl_->index(key)].writer->write(record);
}

void TelemetryStreamRouter::write_batch(
    const std::string &key, const std::span<const nlohmann::json> records) {
    impl_->streams[impl_->index(key)].writer->write_batch(records);
}

void TelemetryStreamRouter::flush() {
    for (auto &stream : impl_->streams) {
        stream.writer->flush();
    }
}

const std::vector<TelemetryStreamSpec> &TelemetryStreamRouter::specs() const {
    return impl_->specs;
}

} // namespace airspy_tv
