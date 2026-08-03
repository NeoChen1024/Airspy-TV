#pragma once

// Internal PSI/SI helpers shared by the service model (transport_stream.cpp)
// and the EPG model (epg.cpp). Not installed; both consumers live in the same
// executable.

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace airspy_tv::si {

constexpr std::size_t ts_packet_size = 188;

[[nodiscard]] std::uint32_t crc32_mpeg(std::span<const std::uint8_t> bytes);

// ETSI EN 300 468 Annex A DVB text. Handles UTF-16 BE (0x11), UTF-8 (0x15),
// the Taiwan "labeled Big5 but actually UCS-2" quirk, and passes other
// encodings through byte-for-byte with control characters replaced.
[[nodiscard]] std::string dvb_text(std::span<const std::uint8_t> bytes);

struct SectionAssembler {
    std::vector<std::uint8_t> bytes;
    std::size_t expected{};
    std::optional<std::uint8_t> continuity;
};

// Consumes full 188-byte transport packets and invokes a handler with each
// complete, CRC-valid section. Continuity mismatches discard the partial
// section so a dropped packet never yields a corrupt payload.
class SectionFeed {
  public:
    using Handler = std::function<void(std::span<const std::uint8_t>)>;

    explicit SectionFeed(Handler handler) : handler_(std::move(handler)) {}

    void consume(std::span<const std::uint8_t> transport_stream) {
        for (std::size_t offset = 0;
             offset + ts_packet_size <= transport_stream.size();
             offset += ts_packet_size) {
            consume_packet(transport_stream.subspan(offset, ts_packet_size));
        }
    }

    void reset() { assemblers_.clear(); }

  private:
    void append(std::uint16_t pid, std::span<const std::uint8_t> bytes,
                bool allow_new) {
        auto &assembler = assemblers_[pid];
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
                const std::span<const std::uint8_t> section(assembler.bytes);
                if (section.size() >= 8 && crc32_mpeg(section) == 0U) {
                    handler_(section);
                }
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
        auto &assembler = assemblers_[pid];
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

    Handler handler_;
    std::map<std::uint16_t, SectionAssembler> assemblers_;
};

} // namespace airspy_tv::si
