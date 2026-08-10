#include "airspy_tv/epg.hpp"

#include "si_common.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

constexpr std::int64_t mjd_unix_epoch = 40587; // MJD of 1970-01-01

[[nodiscard]] std::uint32_t bcd_seconds(const std::uint32_t value) noexcept {
    const auto digit = [](const std::uint32_t nibble) {
        return ((nibble >> 4U) * 10U) + (nibble & 0x0FU);
    };
    const std::uint32_t hours = digit((value >> 16U) & 0xFFU);
    const std::uint32_t minutes = digit((value >> 8U) & 0xFFU);
    const std::uint32_t seconds = digit(value & 0xFFU);
    return (((hours * 60U) + minutes) * 60U) + seconds;
}

[[nodiscard]] std::uint64_t mjd_to_unix(const std::uint32_t mjd) noexcept {
    const std::int64_t days = static_cast<std::int64_t>(mjd) - mjd_unix_epoch;
    if (days < 0) {
        return 0;
    }
    return static_cast<std::uint64_t>(days) * 86400U;
}

// ETSI EN 300 468 content_nibble_level_1 category names (table 27).
[[nodiscard]] std::string content_genre(const std::uint8_t level) noexcept {
    switch (level) {
    case 0x01U:
        return "Movie/Drama";
    case 0x02U:
        return "News";
    case 0x03U:
        return "Show/Game";
    case 0x04U:
        return "Sports";
    case 0x05U:
        return "Children";
    case 0x06U:
        return "Music";
    case 0x07U:
        return "Arts/Culture";
    case 0x08U:
        return "Social/Politics";
    case 0x09U:
        return "Education/Science";
    case 0x0AU:
        return "Leisure";
    case 0x0BU:
        return "Special";
    default:
        return {};
    }
}

} // namespace

struct EpgModel::Impl {
    mutable std::mutex mutex;
    std::map<std::uint16_t, std::vector<EpgEvent>> events;
    std::optional<std::uint64_t> utc_time;
    std::chrono::steady_clock::time_point utc_time_received;
    si::SectionFeed feed{[this](const std::uint16_t,
                               const std::span<const std::uint8_t> section) {
        dispatch(section);
    }};

    void dispatch(const std::span<const std::uint8_t> section) {
        if (section.size() < 8) {
            return;
        }
        if (section[0] == 0x4EU) {
            parse_eit_pf(section);
        } else if (section[0] == 0x70U || section[0] == 0x73U) {
            parse_time_section(section);
        }
    }

    // EIT p/f header: table_id, section_length, service_id (table_id
    // extension), version, section/last_section, transport_stream_id,
    // original_network_id, segment_last_section_number, last_table_id, then
    // the event loop at offset 14.
    void parse_eit_pf(const std::span<const std::uint8_t> section) {
        if (section.size() < 18 || (section[5] & 1U) == 0U) {
            return;
        }
        const auto service_id = static_cast<std::uint16_t>(
            (static_cast<unsigned int>(section[3]) << 8U) | section[4]);
        auto &target = events[service_id];

        std::size_t offset = 14;
        while (offset + 12 <= section.size() - 4) {
            const auto event_id = static_cast<std::uint16_t>(
                (static_cast<unsigned int>(section[offset]) << 8U) |
                section[offset + 1]);
            const auto mjd = static_cast<std::uint32_t>(
                (static_cast<unsigned int>(section[offset + 2]) << 8U) |
                section[offset + 3]);
            const auto start_bcd = static_cast<std::uint32_t>(
                (static_cast<unsigned int>(section[offset + 4]) << 16U) |
                (static_cast<unsigned int>(section[offset + 5]) << 8U) |
                section[offset + 6]);
            const auto duration_bcd = static_cast<std::uint32_t>(
                (static_cast<unsigned int>(section[offset + 7]) << 16U) |
                (static_cast<unsigned int>(section[offset + 8]) << 8U) |
                section[offset + 9]);
            const auto running_status =
                static_cast<std::uint8_t>((section[offset + 10] >> 5U) & 0x07U);
            const std::size_t descriptors_length =
                ((static_cast<std::size_t>(section[offset + 10]) & 0x0FU)
                 << 8U) |
                section[offset + 11];
            const std::size_t descriptors_end =
                std::min(offset + 12 + descriptors_length, section.size() - 4);
            std::string name;
            std::string description;
            std::string genre;
            std::size_t descriptor = offset + 12;
            while (descriptor + 2 <= descriptors_end) {
                const std::size_t length = section[descriptor + 1];
                if (descriptor + 2 + length > descriptors_end) {
                    break;
                }
                const auto payload = section.subspan(descriptor + 2, length);
                if (section[descriptor] == 0x4DU && length >= 5) {
                    // short_event_descriptor: language(3), name_len, name,
                    // text_len, text.
                    const std::size_t name_length = payload[3];
                    if (4 + name_length < payload.size()) {
                        name = si::dvb_text(payload.subspan(4, name_length));
                        const std::size_t text_length =
                            payload[4 + name_length];
                        if (5 + name_length + text_length <= payload.size()) {
                            description = si::dvb_text(
                                payload.subspan(5 + name_length, text_length));
                        }
                    }
                } else if (section[descriptor] == 0x54U && length >= 2) {
                    // content_descriptor: per entry nibble_1(4), nibble_2(4),
                    // user_byte(8).
                    genre = content_genre(
                        static_cast<std::uint8_t>(payload[0] >> 4U));
                }
                descriptor += 2 + length;
            }
            const EpgEvent event{
                .event_id = event_id,
                .start_time_utc = mjd_to_unix(mjd) + bcd_seconds(start_bcd),
                .duration_seconds = bcd_seconds(duration_bcd),
                .running_status = running_status,
                .name = std::move(name),
                .description = std::move(description),
                .genre = std::move(genre),
            };
            auto existing = std::ranges::find_if(
                target, [event_id](const EpgEvent &candidate) {
                    return candidate.event_id == event_id;
                });
            if (existing == target.end()) {
                target.push_back(event);
            } else {
                *existing = event;
            }
            offset += 12 + descriptors_length;
        }
    }

    // TDT (0x70) and TOT (0x73) both carry UTC_time as MJD + BCD after the
    // three-byte section header.
    void parse_time_section(const std::span<const std::uint8_t> section) {
        if (section.size() < 8) {
            return;
        }
        const auto mjd = static_cast<std::uint32_t>(
            (static_cast<unsigned int>(section[3]) << 8U) | section[4]);
        const auto time_bcd = static_cast<std::uint32_t>(
            (static_cast<unsigned int>(section[5]) << 16U) |
            (static_cast<unsigned int>(section[6]) << 8U) | section[7]);
        utc_time = mjd_to_unix(mjd) + bcd_seconds(time_bcd);
        utc_time_received = std::chrono::steady_clock::now();
    }
};

EpgModel::EpgModel() : impl_(std::make_unique<Impl>()) {}
EpgModel::~EpgModel() noexcept = default;

void EpgModel::reset() {
    const std::scoped_lock lock(impl_->mutex);
    impl_->events.clear();
    impl_->utc_time.reset();
    impl_->feed.reset();
}

void EpgModel::on_discontinuity(const TransportDiscontinuity discontinuity) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->feed.reset();
    if (discontinuity == TransportDiscontinuity::retune) {
        impl_->events.clear();
        impl_->utc_time.reset();
    }
}

void EpgModel::consume(const std::span<const std::uint8_t> transport_stream) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->feed.consume(transport_stream);
}

EpgSnapshot EpgModel::snapshot(const std::uint16_t service_id) const {
    const std::scoped_lock lock(impl_->mutex);
    EpgSnapshot snapshot;
    const auto found = impl_->events.find(service_id);
    if (found != impl_->events.end()) {
        snapshot.events = found->second;
        std::ranges::sort(snapshot.events, {}, &EpgEvent::start_time_utc);
    }
    if (impl_->utc_time.has_value()) {
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - impl_->utc_time_received)
                .count();
        snapshot.utc_now =
            *impl_->utc_time +
            static_cast<std::uint64_t>(std::max<std::int64_t>(elapsed, 0));
    }
    return snapshot;
}

} // namespace airspy_tv
