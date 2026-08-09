#include "airspy_tv/transport_stream.hpp"

#include "si_common.hpp"

#include <algorithm>
#include <map>
#include <mutex>
#include <span>
#include <utility>
#include <vector>

namespace airspy_tv {
struct TransportStreamModel::Impl {
    mutable std::mutex mutex;
    std::map<std::uint16_t, TransportService> services;
    si::SectionFeed feed{[this](const std::span<const std::uint8_t> section) {
        dispatch(section);
    }};

    void dispatch(const std::span<const std::uint8_t> section) {
        if (section.size() < 8) {
            return;
        }
        if (section[0] == 0x00U) {
            parse_pat(section);
        } else if (section[0] == 0x02U) {
            parse_pmt(section);
        } else if (section[0] == 0x42U) {
            parse_sdt(section);
        }
    }

    void parse_pat(const std::span<const std::uint8_t> section) {
        if (section.size() < 12 || (section[5] & 1U) == 0U) {
            return;
        }
        for (std::size_t offset = 8; offset + 4 <= section.size() - 4;
             offset += 4) {
            const auto service_id = static_cast<std::uint16_t>(
                (static_cast<unsigned int>(section[offset]) << 8U) |
                section[offset + 1]);
            if (service_id == 0) {
                continue;
            }
            const auto pmt_pid = static_cast<std::uint16_t>(
                ((static_cast<unsigned int>(section[offset + 2]) & 0x1FU)
                 << 8U) |
                section[offset + 3]);
            auto &service = services[service_id];
            service.service_id = service_id;
            service.pmt_pid = pmt_pid;
        }
    }

    void parse_pmt(const std::span<const std::uint8_t> section) {
        if (section.size() < 16 || (section[5] & 1U) == 0U) {
            return;
        }
        const auto service_id = static_cast<std::uint16_t>(
            (static_cast<unsigned int>(section[3]) << 8U) | section[4]);
        auto &service = services[service_id];
        service.service_id = service_id;
        service.pcr_pid = static_cast<std::uint16_t>(
            ((static_cast<unsigned int>(section[8]) & 0x1FU) << 8U) |
            section[9]);
        const std::size_t program_info_length =
            ((static_cast<std::size_t>(section[10]) & 0x0FU) << 8U) |
            section[11];
        std::size_t offset = 12 + program_info_length;
        std::vector<TransportStreamComponent> components;
        while (offset + 5 <= section.size() - 4) {
            const auto pid = static_cast<std::uint16_t>(
                ((static_cast<unsigned int>(section[offset + 1]) & 0x1FU)
                 << 8U) |
                section[offset + 2]);
            components.push_back({pid, section[offset]});
            const std::size_t info_length =
                ((static_cast<std::size_t>(section[offset + 3]) & 0x0FU)
                 << 8U) |
                section[offset + 4];
            offset += 5 + info_length;
        }
        service.components = std::move(components);
    }

    void parse_sdt(const std::span<const std::uint8_t> section) {
        if (section.size() < 15 || (section[5] & 1U) == 0U) {
            return;
        }
        std::size_t offset = 11;
        while (offset + 5 <= section.size() - 4) {
            const auto service_id = static_cast<std::uint16_t>(
                (static_cast<unsigned int>(section[offset]) << 8U) |
                section[offset + 1]);
            const std::size_t descriptors_length =
                ((static_cast<std::size_t>(section[offset + 3]) & 0x0FU)
                 << 8U) |
                section[offset + 4];
            const std::size_t descriptors_end =
                std::min(offset + 5 + descriptors_length, section.size() - 4);
            std::size_t descriptor = offset + 5;
            while (descriptor + 2 <= descriptors_end) {
                const std::size_t length = section[descriptor + 1];
                if (descriptor + 2 + length > descriptors_end) {
                    break;
                }
                if (section[descriptor] == 0x48U && length >= 3) {
                    auto payload = section.subspan(descriptor + 2, length);
                    const std::size_t provider_length = payload[1];
                    if (2 + provider_length < payload.size()) {
                        const std::size_t name_length =
                            payload[2 + provider_length];
                        if (3 + provider_length + name_length <=
                            payload.size()) {
                            auto &service = services[service_id];
                            service.service_id = service_id;
                            service.provider = si::dvb_text(
                                payload.subspan(2, provider_length));
                            service.name = si::dvb_text(payload.subspan(
                                3 + provider_length, name_length));
                        }
                    }
                }
                descriptor += 2 + length;
            }
            offset += 5 + descriptors_length;
        }
    }
};

TransportStreamModel::TransportStreamModel()
    : impl_(std::make_unique<Impl>()) {}
TransportStreamModel::~TransportStreamModel() noexcept = default;

void TransportStreamModel::reset() {
    const std::scoped_lock lock(impl_->mutex);
    impl_->services.clear();
    impl_->feed.reset();
}

void TransportStreamModel::consume(
    const std::span<const std::uint8_t> transport_stream) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->feed.consume(transport_stream);
}

std::vector<TransportService> TransportStreamModel::services() const {
    const std::scoped_lock lock(impl_->mutex);
    std::vector<TransportService> snapshot;
    snapshot.reserve(impl_->services.size());
    for (const auto &[id, service] : impl_->services) {
        static_cast<void>(id);
        snapshot.push_back(service);
    }
    return snapshot;
}

} // namespace airspy_tv
