#include "airspy_tv/transport_pipeline.hpp"

#include <mutex>
#include <utility>

namespace airspy_tv {

struct TransportPipeline::Impl {
    TransportStreamModel service_model;
    EpgModel epg_model;
    AsyncTransportObserver service_observer{
        [this](const std::span<const std::uint8_t> ts) {
            service_model.consume(ts);
        },
        [this](const TransportDiscontinuity discontinuity) {
            service_model.on_discontinuity(discontinuity);
        },
        {.queue_capacity_bytes = 256U << 10U,
         .thread_name = "ts-service-model"}};
    AsyncTransportObserver epg_observer{
        [this](const std::span<const std::uint8_t> ts) {
            epg_model.consume(ts);
        },
        [this](const TransportDiscontinuity discontinuity) {
            epg_model.on_discontinuity(discontinuity);
        },
        {.queue_capacity_bytes = 256U << 10U, .thread_name = "ts-epg-model"}};
    TransportStreamRecorder recorder;
    RtpUdpTransportOutput rtp;
    mutable std::mutex sink_mutex;
    Sink sink;
    DiscontinuitySink discontinuity_sink;
};

TransportPipeline::TransportPipeline() : impl_(std::make_unique<Impl>()) {}

TransportPipeline::~TransportPipeline() noexcept { stop(); }

void TransportPipeline::consume(
    const std::span<const std::uint8_t> transport_stream) {
    static_cast<void>(impl_->service_observer.submit(transport_stream));
    static_cast<void>(impl_->epg_observer.submit(transport_stream));
    impl_->recorder.submit(transport_stream);
    impl_->rtp.submit(transport_stream);

    Sink sink;
    {
        const std::scoped_lock lock(impl_->sink_mutex);
        sink = impl_->sink;
    }
    if (sink) {
        sink(transport_stream);
    }
}

void TransportPipeline::notify_discontinuity(
    const TransportDiscontinuity discontinuity) {
    impl_->service_observer.notify_discontinuity(discontinuity);
    impl_->epg_observer.notify_discontinuity(discontinuity);
    if (discontinuity == TransportDiscontinuity::retune) {
        impl_->recorder.discard_queued();
        impl_->rtp.discard_queued();
    }

    DiscontinuitySink sink;
    {
        const std::scoped_lock lock(impl_->sink_mutex);
        sink = impl_->discontinuity_sink;
    }
    if (sink) {
        sink(discontinuity);
    }
}

void TransportPipeline::set_sink(Sink sink) {
    const std::scoped_lock lock(impl_->sink_mutex);
    impl_->sink = std::move(sink);
}

void TransportPipeline::set_discontinuity_sink(
    DiscontinuitySink discontinuity_sink) {
    const std::scoped_lock lock(impl_->sink_mutex);
    impl_->discontinuity_sink = std::move(discontinuity_sink);
}

bool TransportPipeline::start_recording(const std::filesystem::path &path,
                                        std::string &error) {
    return impl_->recorder.start(path, error);
}

void TransportPipeline::stop_recording() noexcept { impl_->recorder.stop(); }

bool TransportPipeline::start_rtp(const RtpUdpEndpoint &endpoint,
                                  std::string &error) {
    return impl_->rtp.start(endpoint, error);
}

void TransportPipeline::stop_rtp() noexcept { impl_->rtp.stop(); }

void TransportPipeline::stop() noexcept {
    set_sink({});
    set_discontinuity_sink({});
    impl_->recorder.stop();
    impl_->rtp.stop();
    impl_->service_observer.stop(false);
    impl_->epg_observer.stop(false);
}

std::vector<TransportService> TransportPipeline::services() const {
    return impl_->service_model.services();
}

EpgSnapshot
TransportPipeline::epg_snapshot(const std::uint16_t service_id) const {
    return impl_->epg_model.snapshot(service_id);
}

TransportPipelineSnapshot TransportPipeline::snapshot() const {
    return {.service_observer = impl_->service_observer.stats(),
            .epg_observer = impl_->epg_observer.stats(),
            .recorder = impl_->recorder.stats(),
            .rtp = impl_->rtp.stats()};
}

std::vector<TransportOutputTelemetry>
TransportPipeline::output_telemetry() const {
    const auto current = snapshot();
    return {
        {.name = "service-model",
         .type = "observer",
         .active = current.service_observer.active,
         .failed = current.service_observer.failed,
         .blocks_accepted = current.service_observer.blocks_accepted,
         .bytes_accepted = current.service_observer.bytes_accepted,
         .blocks_processed = current.service_observer.blocks_processed,
         .bytes_processed = current.service_observer.bytes_processed,
         .dropped_blocks = current.service_observer.dropped_blocks,
         .dropped_bytes = current.service_observer.dropped_bytes,
         .queued_bytes = current.service_observer.queued_bytes,
         .queue_capacity_bytes = current.service_observer.queue_capacity_bytes,
         .error = current.service_observer.error},
        {.name = "epg-model",
         .type = "observer",
         .active = current.epg_observer.active,
         .failed = current.epg_observer.failed,
         .blocks_accepted = current.epg_observer.blocks_accepted,
         .bytes_accepted = current.epg_observer.bytes_accepted,
         .blocks_processed = current.epg_observer.blocks_processed,
         .bytes_processed = current.epg_observer.bytes_processed,
         .dropped_blocks = current.epg_observer.dropped_blocks,
         .dropped_bytes = current.epg_observer.dropped_bytes,
         .queued_bytes = current.epg_observer.queued_bytes,
         .queue_capacity_bytes = current.epg_observer.queue_capacity_bytes,
         .error = current.epg_observer.error},
        {.name = "ts-recorder",
         .type = "file",
         .active = current.recorder.active,
         .failed = current.recorder.failed,
         .blocks_accepted = current.recorder.blocks_accepted,
         .bytes_accepted = current.recorder.bytes_accepted,
         .blocks_processed = current.recorder.blocks_written,
         .bytes_processed = current.recorder.bytes_written,
         .dropped_blocks = current.recorder.dropped_blocks,
         .dropped_bytes = current.recorder.dropped_bytes,
         .errors = current.recorder.write_errors,
         .queued_bytes = current.recorder.queued_bytes,
         .queue_capacity_bytes = current.recorder.queue_capacity_bytes,
         .error = current.recorder.error},
        {.name = "rtp-udp",
         .type = "network",
         .active = current.rtp.active,
         .failed = current.rtp.failed,
         .blocks_accepted = current.rtp.datagrams_accepted,
         .bytes_accepted = current.rtp.wire_bytes_accepted,
         .blocks_processed = current.rtp.datagrams_sent,
         .bytes_processed = current.rtp.wire_bytes_sent,
         .dropped_blocks = current.rtp.dropped_datagrams,
         .dropped_bytes = current.rtp.dropped_wire_bytes,
         .errors = current.rtp.write_errors,
         .queued_bytes = current.rtp.queued_bytes,
         .queue_capacity_bytes = current.rtp.queue_capacity_bytes,
         .error = current.rtp.last_error},
    };
}

std::string TransportPipeline::runtime_error() const {
    const auto service = impl_->service_observer.stats();
    if (service.failed) {
        return "Transport service observer failed: " + service.error;
    }
    const auto epg = impl_->epg_observer.stats();
    return epg.failed ? "EPG observer failed: " + epg.error : std::string{};
}

} // namespace airspy_tv
