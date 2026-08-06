#include "airspy_tv/transport_router.hpp"

#include <utility>

namespace airspy_tv {

TransportStreamRouter::TransportStreamRouter(
    TransportStreamModel &model, TransportStreamRecorder &recorder) noexcept
    : model_(model), recorder_(recorder) {}

void TransportStreamRouter::consume(
    const std::span<const std::uint8_t> transport_stream) {
    model_.consume(transport_stream);
    recorder_.submit(transport_stream);

    Sink sink;
    {
        const std::scoped_lock lock(sink_mutex_);
        sink = sink_;
    }
    if (sink) {
        sink(transport_stream);
    }
}

void TransportStreamRouter::set_sink(Sink sink) {
    const std::scoped_lock lock(sink_mutex_);
    sink_ = std::move(sink);
}

} // namespace airspy_tv
