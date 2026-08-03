#include "airspy_tv/transport_stream.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <utility>

namespace airspy_tv {
namespace {

constexpr std::size_t packet_size = 188;

std::uint32_t crc32_mpeg(const std::span<const std::uint8_t> bytes) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint32_t>(byte) << 24U;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80000000U) != 0U ? (crc << 1U) ^ 0x04C11DB7U
                                            : crc << 1U;
        }
    }
    return crc;
}

std::string dvb_text(std::span<const std::uint8_t> bytes) {
    const bool mislabeled_ucs2 =
        bytes.size() >= 3 && bytes.front() == 0x14U && bytes[1] < 0x81U;
    if (!bytes.empty() && (bytes.front() == 0x11U || mislabeled_ucs2)) {
        // ETSI EN 300 468: ISO/IEC 10646 Basic Multilingual Plane, encoded as
        // big-endian 16-bit code units. Some Taiwan SDTs label the same bytes
        // as Big5 (0x14); an impossible Big5 lead byte makes that case safe to
        // recognize without mis-decoding a conforming Big5 string.
        std::string text;
        for (std::size_t offset = 1; offset + 1 < bytes.size(); offset += 2) {
            const std::uint32_t codepoint =
                (static_cast<std::uint32_t>(bytes[offset]) << 8U) |
                bytes[offset + 1];
            if (codepoint < 0x80U) {
                text.push_back(static_cast<char>(codepoint));
            } else if (codepoint < 0x800U) {
                text.push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
                text.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
            } else {
                text.push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
                text.push_back(
                    static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
                text.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
            }
        }
        return text;
    }
    // 0x15 explicitly selects UTF-8. Unmarked Taiwan broadcasts commonly
    // carry UTF-8 as well, so preserve those bytes and only replace controls.
    if (!bytes.empty() && bytes.front() == 0x15U) {
        bytes = bytes.subspan(1);
    }
    std::string text;
    text.reserve(bytes.size());
    for (const std::uint8_t byte : bytes) {
        text.push_back(byte < 0x20U && byte != '\t' ? '?'
                                                    : static_cast<char>(byte));
    }
    return text;
}

struct SectionAssembler {
    std::vector<std::uint8_t> bytes;
    std::size_t expected{};
    std::optional<std::uint8_t> continuity;
};

} // namespace

struct TransportStreamModel::Impl {
    mutable std::mutex mutex;
    std::map<std::uint16_t, TransportService> services;
    std::map<std::uint16_t, SectionAssembler> assemblers;

    void dispatch(const std::span<const std::uint8_t> section) {
        if (section.size() < 8 || crc32_mpeg(section) != 0U) {
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
                            service.provider =
                                dvb_text(payload.subspan(2, provider_length));
                            service.name = dvb_text(payload.subspan(
                                3 + provider_length, name_length));
                        }
                    }
                }
                descriptor += 2 + length;
            }
            offset += 5 + descriptors_length;
        }
    }

    void append(std::uint16_t pid, std::span<const std::uint8_t> bytes,
                bool allow_new) {
        auto &assembler = assemblers[pid];
        while (!bytes.empty()) {
            if (assembler.bytes.empty()) {
                if (!allow_new || bytes.front() == 0xFFU) {
                    return;
                }
                assembler.expected = 0;
            }
            const std::size_t required =
                assembler.expected == 0
                    ? std::min<std::size_t>(3 - assembler.bytes.size(),
                                            bytes.size())
                    : std::min(assembler.expected - assembler.bytes.size(),
                               bytes.size());
            assembler.bytes.insert(assembler.bytes.end(), bytes.begin(),
                                   bytes.begin() +
                                       static_cast<std::ptrdiff_t>(required));
            bytes = bytes.subspan(required);
            if (assembler.expected == 0 && assembler.bytes.size() == 3) {
                assembler.expected =
                    3 + (((static_cast<std::size_t>(assembler.bytes[1]) & 0x0FU)
                          << 8U) |
                         assembler.bytes[2]);
                if (assembler.expected < 8 || assembler.expected > 4096) {
                    assembler.bytes.clear();
                    assembler.expected = 0;
                    return;
                }
            }
            if (assembler.expected != 0 &&
                assembler.bytes.size() == assembler.expected) {
                dispatch(assembler.bytes);
                assembler.bytes.clear();
                assembler.expected = 0;
                allow_new = true;
            }
        }
    }

    void consume_packet(const std::span<const std::uint8_t> packet) {
        if (packet[0] != 0x47U || (packet[1] & 0x80U) != 0U) {
            return;
        }
        const auto pid = static_cast<std::uint16_t>(
            ((static_cast<unsigned int>(packet[1]) & 0x1FU) << 8U) | packet[2]);
        const unsigned int adaptation = (packet[3] >> 4U) & 0x03U;
        if (adaptation == 0U || adaptation == 2U) {
            return;
        }
        std::size_t offset = 4;
        if (adaptation == 3U) {
            offset += 1 + packet[offset];
            if (offset >= packet.size()) {
                return;
            }
        }
        auto &assembler = assemblers[pid];
        const std::uint8_t continuity = packet[3] & 0x0FU;
        if (assembler.continuity.has_value() &&
            continuity != static_cast<std::uint8_t>(
                              (*assembler.continuity + 1U) & 0x0FU)) {
            assembler.bytes.clear();
            assembler.expected = 0;
        }
        assembler.continuity = continuity;
        const bool start = (packet[1] & 0x40U) != 0U;
        if (!start) {
            append(pid, packet.subspan(offset), false);
            return;
        }
        const std::size_t pointer = packet[offset];
        ++offset;
        if (offset + pointer > packet.size()) {
            assembler.bytes.clear();
            return;
        }
        if (pointer != 0) {
            append(pid, packet.subspan(offset, pointer), false);
        }
        assembler.bytes.clear();
        assembler.expected = 0;
        append(pid, packet.subspan(offset + pointer), true);
    }
};

TransportStreamModel::TransportStreamModel()
    : impl_(std::make_unique<Impl>()) {}
TransportStreamModel::~TransportStreamModel() noexcept = default;

void TransportStreamModel::reset() {
    const std::scoped_lock lock(impl_->mutex);
    impl_->services.clear();
    impl_->assemblers.clear();
}

void TransportStreamModel::consume(
    const std::span<const std::uint8_t> transport_stream) {
    const std::scoped_lock lock(impl_->mutex);
    for (std::size_t offset = 0;
         offset + packet_size <= transport_stream.size();
         offset += packet_size) {
        impl_->consume_packet(transport_stream.subspan(offset, packet_size));
    }
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
