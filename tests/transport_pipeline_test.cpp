#include "airspy_tv/transport_pipeline.hpp"

#include <cstdint>
#include <iostream>
#include <span>
#include <string_view>
#include <vector>

namespace {

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool test_capacities_and_external_fanout() {
    airspy_tv::TransportPipeline pipeline;
    std::vector<std::uint8_t> received;
    std::vector<airspy_tv::TransportDiscontinuity> discontinuities;
    pipeline.set_sink([&](const std::span<const std::uint8_t> block) {
        received.insert(received.end(), block.begin(), block.end());
    });
    pipeline.set_discontinuity_sink(
        [&](const airspy_tv::TransportDiscontinuity discontinuity) {
            discontinuities.push_back(discontinuity);
        });

    const std::vector<std::uint8_t> packet(188, 0x47);
    pipeline.consume(packet);
    pipeline.notify_discontinuity(airspy_tv::TransportDiscontinuity::retune);
    const auto snapshot = pipeline.snapshot();

    return require(snapshot.service_observer.queue_capacity_bytes ==
                       (256U << 10U),
                   "service model owns a 256 KiB queue") &&
           require(snapshot.epg_observer.queue_capacity_bytes == (256U << 10U),
                   "EPG model owns a 256 KiB queue") &&
           require(snapshot.recorder.queue_capacity_bytes == (24U << 20U),
                   "TS recorder owns a 24 MiB queue") &&
           require(snapshot.rtp.queue_capacity_bytes == (8U << 20U),
                   "RTP output owns an 8 MiB queue") &&
           require(received == packet,
                   "external sink receives an independent transport copy") &&
           require(discontinuities.size() == 1 &&
                       discontinuities.front() ==
                           airspy_tv::TransportDiscontinuity::retune,
                   "external sink receives typed retune discontinuity");
}

bool test_headless_pipeline_omits_metadata_observers() {
    airspy_tv::TransportPipeline pipeline(
        airspy_tv::TransportPipelineConfig::headless());
    const std::vector<std::uint8_t> packet(188, 0x47);
    pipeline.consume(packet);
    pipeline.notify_discontinuity(
        airspy_tv::TransportDiscontinuity::fec_region_reset);
    const auto snapshot = pipeline.snapshot();
    const auto telemetry = pipeline.output_telemetry();
    return require(!snapshot.service_observer.active &&
                       snapshot.service_observer.queue_capacity_bytes == 0,
                   "headless pipeline does not create the service observer") &&
           require(!snapshot.epg_observer.active &&
                       snapshot.epg_observer.queue_capacity_bytes == 0,
                   "headless pipeline does not create the EPG observer") &&
           require(telemetry.size() == 2,
                   "headless telemetry omits disabled metadata observers");
}

} // namespace

int main() {
    return test_capacities_and_external_fanout() &&
                   test_headless_pipeline_omits_metadata_observers()
               ? 0
               : 1;
}
