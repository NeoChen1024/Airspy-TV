#include "airspy_tv/transport_stream.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using airspy_tv::TransportService;
using airspy_tv::TransportStreamModel;

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

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

std::vector<std::uint8_t> finish_section(std::vector<std::uint8_t> section) {
    const std::size_t section_length = section.size() - 3U + 4U;
    section[1] = static_cast<std::uint8_t>(
        0xB0U | ((section_length >> 8U) & 0x0FU));
    section[2] = static_cast<std::uint8_t>(section_length & 0xFFU);
    const std::uint32_t crc = crc32_mpeg(section);
    section.push_back(static_cast<std::uint8_t>(crc >> 24U));
    section.push_back(static_cast<std::uint8_t>(crc >> 16U));
    section.push_back(static_cast<std::uint8_t>(crc >> 8U));
    section.push_back(static_cast<std::uint8_t>(crc));
    return section;
}

std::vector<std::uint8_t> long_header(const std::uint8_t table_id,
                                      const std::uint16_t extension,
                                      const std::uint8_t version,
                                      const std::uint8_t section,
                                      const std::uint8_t last) {
    return {table_id,
            0,
            0,
            static_cast<std::uint8_t>(extension >> 8U),
            static_cast<std::uint8_t>(extension),
            static_cast<std::uint8_t>(0xC1U | ((version & 0x1FU) << 1U)),
            section,
            last};
}

std::vector<std::uint8_t>
pat(const std::uint8_t version, const std::uint8_t section,
    const std::uint8_t last,
    const std::vector<std::pair<std::uint16_t, std::uint16_t>> &programs,
    const bool malformed_tail = false) {
    auto bytes = long_header(0x00, 1, version, section, last);
    for (const auto [service, pid] : programs) {
        bytes.push_back(static_cast<std::uint8_t>(service >> 8U));
        bytes.push_back(static_cast<std::uint8_t>(service));
        bytes.push_back(static_cast<std::uint8_t>(0xE0U | (pid >> 8U)));
        bytes.push_back(static_cast<std::uint8_t>(pid));
    }
    if (malformed_tail) {
        bytes.push_back(0xAA);
    }
    return finish_section(std::move(bytes));
}

std::vector<std::uint8_t>
pmt(const std::uint16_t service, const std::uint8_t version,
    const std::uint16_t pcr_pid,
    const std::vector<std::pair<std::uint8_t, std::uint16_t>> &components,
    const bool malformed_info = false) {
    auto bytes = long_header(0x02, service, version, 0, 0);
    bytes.push_back(static_cast<std::uint8_t>(0xE0U | (pcr_pid >> 8U)));
    bytes.push_back(static_cast<std::uint8_t>(pcr_pid));
    bytes.push_back(malformed_info ? 0xF0U : 0xF0U);
    bytes.push_back(malformed_info ? 0x20U : 0x00U);
    for (const auto [type, pid] : components) {
        bytes.push_back(type);
        bytes.push_back(static_cast<std::uint8_t>(0xE0U | (pid >> 8U)));
        bytes.push_back(static_cast<std::uint8_t>(pid));
        bytes.push_back(0xF0U);
        bytes.push_back(0x00U);
    }
    return finish_section(std::move(bytes));
}

std::vector<std::uint8_t> sdt(const std::uint8_t version,
                              const std::uint16_t service,
                              const std::string &provider,
                              const std::string &name,
                              const bool malformed_descriptor = false) {
    auto bytes = long_header(0x42, 1, version, 0, 0);
    bytes.insert(bytes.end(), {0x00, 0x01, 0xFF});
    std::vector<std::uint8_t> descriptor{
        0x48,
        0,
        0x01,
        static_cast<std::uint8_t>(provider.size())};
    descriptor.insert(descriptor.end(), provider.begin(), provider.end());
    descriptor.push_back(static_cast<std::uint8_t>(name.size()));
    descriptor.insert(descriptor.end(), name.begin(), name.end());
    descriptor[1] = static_cast<std::uint8_t>(descriptor.size() - 2U);
    if (malformed_descriptor) {
        descriptor[1] = static_cast<std::uint8_t>(descriptor[1] + 8U);
    }
    bytes.push_back(static_cast<std::uint8_t>(service >> 8U));
    bytes.push_back(static_cast<std::uint8_t>(service));
    bytes.push_back(0xFF);
    bytes.push_back(static_cast<std::uint8_t>(0xF0U |
                                              (descriptor.size() >> 8U)));
    bytes.push_back(static_cast<std::uint8_t>(descriptor.size()));
    bytes.insert(bytes.end(), descriptor.begin(), descriptor.end());
    return finish_section(std::move(bytes));
}

class Packetizer {
  public:
    std::vector<std::uint8_t> packet(const std::uint16_t pid,
                                     const std::vector<std::uint8_t> &section) {
        if (section.size() > 183U) {
            std::abort();
        }
        std::vector<std::uint8_t> result(188, 0xFF);
        result[0] = 0x47;
        result[1] = static_cast<std::uint8_t>(0x40U | (pid >> 8U));
        result[2] = static_cast<std::uint8_t>(pid);
        result[3] = static_cast<std::uint8_t>(0x10U | continuity_[pid]);
        continuity_[pid] = static_cast<std::uint8_t>((continuity_[pid] + 1U) &
                                                     0x0FU);
        result[4] = 0;
        std::copy(section.begin(), section.end(), result.begin() + 5);
        return result;
    }

  private:
    std::map<std::uint16_t, std::uint8_t> continuity_;
};

const TransportService *find_service(const std::vector<TransportService> &all,
                                     const std::uint16_t id) {
    const auto found = std::ranges::find(all, id, &TransportService::service_id);
    return found == all.end() ? nullptr : &*found;
}

bool test_atomic_pat_and_pmt_versions() {
    TransportStreamModel model;
    Packetizer packets;
    model.consume(packets.packet(0, pat(31, 0, 1, {{1, 100}})));
    if (!require(model.services().empty(),
                 "incomplete multi-section PAT is not published")) {
        return false;
    }
    model.consume(packets.packet(0, pat(31, 1, 1, {{2, 200}})));
    if (!require(model.services().size() == 2,
                 "complete PAT publishes all sections atomically")) {
        return false;
    }
    model.consume(packets.packet(100, pmt(1, 31, 300, {{0x02, 256}})));
    auto current = model.services();
    const auto *service = find_service(current, 1);
    if (!require(service != nullptr && service->pcr_pid == 300 &&
                     service->components.size() == 1,
                 "PMT populates PCR and components")) {
        return false;
    }

    model.consume(packets.packet(0, pat(0, 0, 1, {{1, 101}})));
    if (!require(model.services().size() == 2,
                 "partial wrapped PAT version preserves active snapshot")) {
        return false;
    }
    model.consume(packets.packet(0, pat(0, 1, 1, {}, true)));
    current = model.services();
    service = find_service(current, 1);
    if (!require(current.size() == 2 && service != nullptr &&
                     service->pmt_pid == 100,
                 "malformed complete candidate cannot partially mutate PAT")) {
        return false;
    }
    model.consume(packets.packet(0, pat(0, 0, 1, {{1, 101}})));
    model.consume(packets.packet(0, pat(0, 1, 1, {})));
    current = model.services();
    service = find_service(current, 1);
    if (!require(current.size() == 1 && service != nullptr &&
                     service->pmt_pid == 101 &&
                     service->pcr_pid == 0x1FFF &&
                     service->components.empty(),
                 "wrapped PAT removes services and clears changed PMT state")) {
        return false;
    }

    model.consume(packets.packet(0, pat(31, 0, 0, {{9, 900}})));
    if (!require(model.services().size() == 1 &&
                     find_service(model.services(), 1) != nullptr,
                 "stale pre-wrap PAT cannot roll the model back")) {
        return false;
    }
    model.consume(packets.packet(100, pmt(1, 0, 400, {{0x03, 257}})));
    if (!require(find_service(model.services(), 1)->components.empty(),
                 "PMT arriving on a stale PID is ignored")) {
        return false;
    }
    model.consume(packets.packet(101, pmt(1, 31, 301, {{0x02, 260}})));
    model.consume(packets.packet(101, pmt(1, 0, 302, {{0x03, 261}})));
    current = model.services();
    service = find_service(current, 1);
    if (!require(service != nullptr && service->pcr_pid == 302 &&
                     service->components.size() == 1 &&
                     service->components.front().pid == 261,
                 "wrapped PMT version atomically replaces components")) {
        return false;
    }
    model.consume(packets.packet(101, pmt(1, 1, 999, {}, true)));
    current = model.services();
    service = find_service(current, 1);
    if (!require(service != nullptr && service->pcr_pid == 302 &&
                     service->components.front().pid == 261,
                 "malformed PMT preserves the active version")) {
        return false;
    }

    model.consume(packets.packet(0, pat(1, 0, 0, {{1, 100}})));
    model.consume(packets.packet(100, pmt(1, 31, 303, {{0x04, 262}})));
    current = model.services();
    service = find_service(current, 1);
    return require(service != nullptr && service->pmt_pid == 100 &&
                       service->pcr_pid == 303 &&
                       service->components.front().pid == 262,
                   "returning to an earlier PMT PID starts fresh version state");
}

bool test_sdt_versions_and_malformed_packets() {
    TransportStreamModel model;
    Packetizer packets;
    model.consume(packets.packet(0, pat(1, 0, 0, {{1, 100}})));
    model.consume(packets.packet(0x11, sdt(31, 1, "Provider A", "News")));
    auto current = model.services();
    auto service = find_service(current, 1);
    if (!require(service != nullptr && service->name == "News" &&
                     service->provider == "Provider A",
                 "SDT publishes service metadata")) {
        return false;
    }
    model.consume(
        packets.packet(0x11, sdt(0, 1, "Broken", "Broken", true)));
    current = model.services();
    service = find_service(current, 1);
    if (!require(service != nullptr && service->name == "News",
                 "malformed SDT preserves active metadata")) {
        return false;
    }
    model.consume(packets.packet(0x11, sdt(0, 1, "Provider B", "Sports")));
    current = model.services();
    service = find_service(current, 1);
    if (!require(service != nullptr && service->name == "Sports" &&
                     service->provider == "Provider B",
                 "wrapped SDT version replaces metadata")) {
        return false;
    }

    auto bad_crc = packets.packet(0, pat(2, 0, 0, {{2, 200}}));
    bad_crc[20] ^= 0x80U;
    model.consume(bad_crc);
    std::vector<std::uint8_t> bad_pointer(188, 0xFF);
    bad_pointer[0] = 0x47;
    bad_pointer[1] = 0x40;
    bad_pointer[2] = 0;
    bad_pointer[3] = 0x10;
    bad_pointer[4] = 250;
    model.consume(bad_pointer);
    std::vector<std::uint8_t> bad_adaptation(188, 0xFF);
    bad_adaptation[0] = 0x47;
    bad_adaptation[1] = 0x40;
    bad_adaptation[2] = 0;
    bad_adaptation[3] = 0x30;
    bad_adaptation[4] = 184;
    model.consume(bad_adaptation);
    model.consume(packets.packet(0, pat(2, 1, 0, {{2, 200}})));
    current = model.services();
    service = find_service(current, 1);
    return require(current.size() == 1 && service != nullptr &&
                       service->name == "Sports",
                   "bad CRC, packet bounds, and section numbering preserve "
                   "active tables");
}

} // namespace

int main() {
    return test_atomic_pat_and_pmt_versions() &&
                   test_sdt_versions_and_malformed_packets()
               ? 0
               : 1;
}
