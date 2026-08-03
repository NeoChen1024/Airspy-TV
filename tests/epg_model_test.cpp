// Unit test for the EPG model: EIT p/f event parsing (MJD/BCD time,
// short-event descriptor, content descriptor), TDT/TOT clock capture, and the
// PAT-derived service-order association used because EIT carries no
// service_id.

#include "airspy_tv/epg.hpp"
#include "si_common.hpp"

#include <cstdint>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {

using airspy_tv::EpgModel;
using airspy_tv::EpgSnapshot;

std::uint32_t crc32_mpeg(std::span<const std::uint8_t> bytes) {
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

// Appends the MPEG-2 CRC32 to a section body.
std::vector<std::uint8_t> with_crc(std::vector<std::uint8_t> section) {
    const std::uint32_t crc = crc32_mpeg(section);
    section.push_back(static_cast<std::uint8_t>(crc >> 24U));
    section.push_back(static_cast<std::uint8_t>(crc >> 16U));
    section.push_back(static_cast<std::uint8_t>(crc >> 8U));
    section.push_back(static_cast<std::uint8_t>(crc));
    return section;
}

std::uint8_t bcd(const std::uint32_t value) {
    return static_cast<std::uint8_t>(((value / 10U) << 4U) | (value % 10U));
}

// Wraps one or more sections into 188-byte TS packets on the given PID with
// proper PUSI/pointer_field handling. A single section per packet keeps the
// helper trivial.
std::vector<std::uint8_t> packets(std::uint16_t pid,
                                  const std::vector<std::uint8_t> &section) {
    std::vector<std::uint8_t> output;
    for (std::size_t offset = 0; offset < section.size(); offset += 183) {
        const bool start = offset == 0;
        std::vector<std::uint8_t> packet(188, 0xFF);
        packet[0] = 0x47;
        packet[1] = static_cast<std::uint8_t>(
            0x40U | ((pid >> 8U) & 0x1FU)); // PUSI set
        packet[2] = static_cast<std::uint8_t>(pid & 0xFFU);
        packet[3] = 0x10U; // payload only, continuity 0
        const std::size_t payload_start = start ? 5 : 4;
        if (start) {
            packet[4] = 0x00; // pointer_field
        }
        const std::size_t count =
            std::min<std::size_t>(section.size() - offset, 188 - payload_start);
        for (std::size_t i = 0; i < count; ++i) {
            packet[payload_start + i] = section[offset + i];
        }
        output.insert(output.end(), packet.begin(), packet.end());
    }
    return output;
}

void expect(const bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

// unix time -> MJD (16 bits) + BCD hh:mm:ss (24 bits), UTC.
void encode_utc(const std::uint64_t unix_seconds, std::uint8_t &mjd_hi,
                std::uint8_t &mjd_lo, std::uint8_t &hh, std::uint8_t &mm,
                std::uint8_t &ss) {
    const std::uint64_t days = unix_seconds / 86400U;
    const std::uint64_t mjd = days + 40587U;
    const std::uint64_t remainder = unix_seconds % 86400U;
    mjd_hi = static_cast<std::uint8_t>(mjd >> 8U);
    mjd_lo = static_cast<std::uint8_t>(mjd);
    hh = bcd(static_cast<std::uint32_t>(remainder / 3600U));
    mm = bcd(static_cast<std::uint32_t>((remainder / 60U) % 60U));
    ss = bcd(static_cast<std::uint32_t>(remainder % 60U));
}

std::vector<std::uint8_t> make_tdt(const std::uint64_t unix_seconds) {
    std::vector<std::uint8_t> section(8, 0);
    section[0] = 0x70U; // TDT
    std::uint8_t mjd_hi = 0;
    std::uint8_t mjd_lo = 0;
    std::uint8_t hh = 0;
    std::uint8_t mm = 0;
    std::uint8_t ss = 0;
    encode_utc(unix_seconds, mjd_hi, mjd_lo, hh, mm, ss);
    section[3] = mjd_hi;
    section[4] = mjd_lo;
    section[5] = hh;
    section[6] = mm;
    section[7] = ss;
    const std::size_t section_length = 5 + 4; // UTC_time + CRC
    section[1] = static_cast<std::uint8_t>((section_length >> 8U) & 0x0FU);
    section[2] = static_cast<std::uint8_t>(section_length & 0xFFU);
    return section;
}

// Short-event descriptor bytes: tag 0x4D, length, language, name, text.
std::vector<std::uint8_t> short_event(const std::string &name,
                                      const std::string &text) {
    std::vector<std::uint8_t> descriptor{0x4DU, 0x00U, 'e', 'n', 'g'};
    descriptor.push_back(static_cast<std::uint8_t>(name.size()));
    descriptor.insert(descriptor.end(), name.begin(), name.end());
    descriptor.push_back(static_cast<std::uint8_t>(text.size()));
    descriptor.insert(descriptor.end(), text.begin(), text.end());
    descriptor[1] = static_cast<std::uint8_t>(descriptor.size() - 2);
    return descriptor;
}

std::vector<std::uint8_t> content_descriptor(const std::uint8_t level) {
    // tag, length=2, nibble_1<<4|nibble_2, user_byte.
    return {0x54U, 0x02U, static_cast<std::uint8_t>((level << 4U) | 0x00U),
            0x00U};
}

std::vector<std::uint8_t> make_eit_pf(const std::uint16_t service_id,
                                      const std::uint64_t start_unix,
                                      const std::uint32_t duration_seconds,
                                      const std::string &name,
                                      const std::string &text,
                                      const std::uint8_t genre,
                                      const std::uint8_t running_status) {
    std::vector<std::uint8_t> event;
    event.push_back(0x00U); // event_id hi
    event.push_back(0x01U); // event_id lo
    std::uint8_t mjd_hi = 0;
    std::uint8_t mjd_lo = 0;
    std::uint8_t hh = 0;
    std::uint8_t mm = 0;
    std::uint8_t ss = 0;
    encode_utc(start_unix, mjd_hi, mjd_lo, hh, mm, ss);
    event.push_back(mjd_hi);
    event.push_back(mjd_lo);
    event.push_back(hh);
    event.push_back(mm);
    event.push_back(ss);
    event.push_back(bcd(duration_seconds / 3600U));
    event.push_back(bcd((duration_seconds / 60U) % 60U));
    event.push_back(bcd(duration_seconds % 60U));
    const auto name_desc = short_event(name, text);
    const auto genre_desc = content_descriptor(genre);
    const std::size_t descriptors_length = name_desc.size() + genre_desc.size();
    event.push_back(static_cast<std::uint8_t>(
        (running_status << 5U) | ((descriptors_length >> 8U) & 0x0FU)));
    event.push_back(static_cast<std::uint8_t>(descriptors_length & 0xFFU));
    event.insert(event.end(), name_desc.begin(), name_desc.end());
    event.insert(event.end(), genre_desc.begin(), genre_desc.end());

    // EIT p/f header: table_id, section_length, service_id (table_id
    // extension), version/current, section/last_section, transport_stream_id,
    // original_network_id, segment_last_section_number, last_table_id.
    std::vector<std::uint8_t> section{
        0x4EU, 0x00U, 0x00U, static_cast<std::uint8_t>(service_id >> 8U),
        static_cast<std::uint8_t>(service_id & 0xFFU), 0xC1U, 0x00U, 0x01U,
        0x00U, 0x01U, 0x00U, 0x01U, 0xF0U, 0x4EU};
    section.insert(section.end(), event.begin(), event.end());
    const std::size_t section_length = section.size() - 3 + 4; // incl. CRC
    section[1] = static_cast<std::uint8_t>((section_length >> 8U) & 0x0FU);
    section[2] = static_cast<std::uint8_t>(section_length & 0xFFU);
    return section;
}

void test_encodings() {
    using airspy_tv::si::dvb_text;
    // Big5 中文
    const std::vector<std::uint8_t> big5{0x14U, 0xA4U, 0xA4U, 0xA4U, 0xE5U};
    expect(dvb_text(big5) == "\xE4\xB8\xAD\xE6\x96\x87",
           "Big5 decodes to UTF-8");
    // GB-2312 中文
    const std::vector<std::uint8_t> gb{0x13U, 0xD6U, 0xD0U, 0xCEU, 0xC4U};
    expect(dvb_text(gb) == "\xE4\xB8\xAD\xE6\x96\x87",
           "GB-2312 decodes to UTF-8");
    // EUC-KR 한국
    const std::vector<std::uint8_t> kr{0x12U, 0xC7U, 0xD1U, 0xB1U, 0xB9U};
    expect(dvb_text(kr) == "\xED\x95\x9C\xEA\xB5\xAD",
           "EUC-KR decodes to UTF-8");
    // ISO-8859-15 euro sign
    const std::vector<std::uint8_t> latin{0x0BU, 0xA4U};
    expect(dvb_text(latin) == "\xE2\x82\xAC", "ISO-8859-15 decodes");
    // UTF-8 passthrough (0x15 marker stripped)
    const std::vector<std::uint8_t> utf8{
        0x15U, 0xE4U, 0xB8U, 0xADU, 0xE6U, 0x96U, 0x87U};
    expect(dvb_text(utf8) == "\xE4\xB8\xAD\xE6\x96\x87",
           "UTF-8 marker stripped and bytes preserved");
    // Taiwan quirk: 0x14-labeled UTF-16BE
    const std::vector<std::uint8_t> mislabeled{
        0x14U, 0x00U, 0x4EU, 0x00U, 0x65U, 0x00U, 0x77U, 0x00U, 0x73U};
    expect(dvb_text(mislabeled) == "News",
           "mislabeled Big5/UCS-2 decodes as UTF-16BE");
    // Invalid Big5 bytes fall back to lossless passthrough, not garbage.
    const std::vector<std::uint8_t> invalid_big5{0x14U, 0xA4U, 0x20U};
    expect(dvb_text(invalid_big5) == "\xA4 ",
           "invalid Big5 falls back to passthrough");
}

} // namespace

int main() {
    test_encodings();

    // TDT: a fixed wall-clock time is captured and reported.
    constexpr std::uint64_t tdt_unix = 1'777'000'000U;
    EpgModel clock_model;
    clock_model.consume(packets(0x0014, with_crc(make_tdt(tdt_unix))));
    const EpgSnapshot clock_snapshot = clock_model.snapshot(1);
    expect(clock_snapshot.utc_now.has_value(),
           "TDT should set the broadcast clock");
    expect(*clock_snapshot.utc_now >= tdt_unix &&
               *clock_snapshot.utc_now <= tdt_unix + 60U,
           "TDT clock should match the encoded time");

    // Two services, each with its own EIT p/f section; the service id comes
    // from the table_id extension, not arrival order.
    EpgModel model;
    constexpr std::uint64_t start_unix = 1'777'000'000U;
    const std::vector<std::uint8_t> eit_one = with_crc(make_eit_pf(
        0x0100U, start_unix, 1800U, "News Hour", "Headlines", 0x02U, 0x04U));
    const std::vector<std::uint8_t> eit_two = with_crc(make_eit_pf(
        0x0101U, start_unix + 1800U, 3600U, "Sports Live", "Match coverage",
        0x04U, 0x01U));
    // Deliver in reverse order to prove the service id is read from the
    // section and not inferred from arrival order.
    model.consume(packets(0x0012, eit_two));
    model.consume(packets(0x0012, eit_one));

    const EpgSnapshot first = model.snapshot(0x0100U);
    expect(first.events.size() == 1, "service 0x0100 gets the first EIT p/f");
    const airspy_tv::EpgEvent &event = first.events.front();
    expect(event.event_id == 1, "event_id parsed");
    expect(event.start_time_utc == start_unix, "MJD/BCD start time decoded");
    expect(event.duration_seconds == 1800U, "BCD duration decoded");
    expect(event.name == "News Hour", "short-event name decoded");
    expect(event.description == "Headlines", "short-event text decoded");
    expect(event.genre == "News", "content descriptor mapped to genre");
    expect(event.running_status == 0x04U, "running status decoded");

    const EpgSnapshot second = model.snapshot(0x0101U);
    expect(second.events.size() == 1, "service 0x0101 gets the second EIT");
    expect(second.events.front().name == "Sports Live",
           "second service event name");
    expect(second.events.front().genre == "Sports", "second service genre");

    // A corrupt (bad CRC) section must be ignored.
    EpgModel corrupt_model;
    std::vector<std::uint8_t> bad = with_crc(make_tdt(tdt_unix));
    bad.back() ^= 0xFFU;
    corrupt_model.consume(packets(0x0014, bad));
    expect(!corrupt_model.snapshot(1).utc_now.has_value(),
           "bad CRC section is ignored");

    std::cout << "epg-model: all checks passed\n";
    return 0;
}
