#include "airspy_tv/transport_stream.hpp"

#include "si_common.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace airspy_tv {
namespace {

struct CompletedTable {
    std::uint32_t key{};
    std::uint8_t version{};
    std::vector<std::vector<std::uint8_t>> sections;
};

[[nodiscard]] bool newer_version(const std::uint8_t candidate,
                                 const std::uint8_t active) {
    const unsigned int delta =
        (static_cast<unsigned int>(candidate) + 32U - active) & 0x1FU;
    return delta != 0U && delta < 16U;
}

class VersionedSectionCollector {
  public:
    [[nodiscard]] std::optional<CompletedTable>
    add(const std::uint32_t key,
        const std::span<const std::uint8_t> section) {
        if (section.size() < 12 || (section[5] & 1U) == 0U) {
            return std::nullopt;
        }
        const std::uint8_t version = (section[5] >> 1U) & 0x1FU;
        const std::uint8_t number = section[6];
        const std::uint8_t last = section[7];
        if (number > last) {
            return std::nullopt;
        }
        auto &state = states_[key];
        if (state.active_version.has_value()) {
            if (version == *state.active_version ||
                !newer_version(version, *state.active_version)) {
                return std::nullopt;
            }
        }
        if (!state.pending_version.has_value() ||
            *state.pending_version != version) {
            state.pending_version = version;
            state.last_section = last;
            state.sections.clear();
        } else if (state.last_section != last) {
            state.pending_version.reset();
            state.sections.clear();
            return std::nullopt;
        }
        state.sections[number] = {section.begin(), section.end()};
        if (state.sections.size() !=
            static_cast<std::size_t>(state.last_section) + 1U) {
            return std::nullopt;
        }
        CompletedTable completed{
            .key = key, .version = version, .sections = {}};
        completed.sections.reserve(state.sections.size());
        for (std::uint16_t index = 0; index <= state.last_section; ++index) {
            const auto found = state.sections.find(static_cast<std::uint8_t>(index));
            if (found == state.sections.end()) {
                return std::nullopt;
            }
            completed.sections.push_back(found->second);
        }
        return completed;
    }

    void commit(const std::uint32_t key, const std::uint8_t version) {
        auto found = states_.find(key);
        if (found == states_.end() ||
            found->second.pending_version != version) {
            return;
        }
        found->second.active_version = version;
        found->second.pending_version.reset();
        found->second.sections.clear();
    }

    void reject(const std::uint32_t key, const std::uint8_t version) {
        auto found = states_.find(key);
        if (found != states_.end() &&
            found->second.pending_version == version) {
            found->second.pending_version.reset();
            found->second.sections.clear();
        }
    }

    void discard_pending() {
        for (auto &[key, state] : states_) {
            static_cast<void>(key);
            state.pending_version.reset();
            state.sections.clear();
        }
    }

    void erase(const std::uint32_t key) { states_.erase(key); }

    void reset() { states_.clear(); }

  private:
    struct State {
        std::optional<std::uint8_t> active_version;
        std::optional<std::uint8_t> pending_version;
        std::uint8_t last_section{};
        std::map<std::uint8_t, std::vector<std::uint8_t>> sections;
    };
    std::map<std::uint32_t, State> states_;
};

[[nodiscard]] std::uint16_t extension(
    const std::span<const std::uint8_t> section) {
    return static_cast<std::uint16_t>(
        (static_cast<unsigned int>(section[3]) << 8U) | section[4]);
}

[[nodiscard]] std::uint32_t table_key(
    const std::uint16_t pid, const std::span<const std::uint8_t> section) {
    return (static_cast<std::uint32_t>(pid) << 16U) | extension(section);
}

} // namespace

struct TransportStreamModel::Impl {
    mutable std::mutex mutex;
    std::map<std::uint16_t, TransportService> services;
    VersionedSectionCollector pat_tables;
    VersionedSectionCollector pmt_tables;
    VersionedSectionCollector sdt_tables;
    si::SectionFeed feed{
        [this](const std::uint16_t pid,
               const std::span<const std::uint8_t> section) {
            dispatch(pid, section);
        }};

    void dispatch(const std::uint16_t pid,
                  const std::span<const std::uint8_t> section) {
        if (section.size() < 12) {
            return;
        }
        if (pid == 0x0000U && section[0] == 0x00U) {
            collect_pat(pid, section);
        } else if (section[0] == 0x02U) {
            collect_pmt(pid, section);
        } else if (pid == 0x0011U && section[0] == 0x42U) {
            collect_sdt(pid, section);
        }
    }

    void collect_pat(const std::uint16_t pid,
                     const std::span<const std::uint8_t> section) {
        auto completed = pat_tables.add(table_key(pid, section), section);
        if (!completed.has_value()) {
            return;
        }
        std::map<std::uint16_t, std::uint16_t> programs;
        bool valid = true;
        for (const auto &bytes : completed->sections) {
            const std::span<const std::uint8_t> part(bytes);
            if ((part.size() - 12U) % 4U != 0U) {
                valid = false;
                break;
            }
            for (std::size_t offset = 8; offset < part.size() - 4;
                 offset += 4) {
                const auto service_id = static_cast<std::uint16_t>(
                    (static_cast<unsigned int>(part[offset]) << 8U) |
                    part[offset + 1]);
                if (service_id == 0) {
                    continue;
                }
                programs[service_id] = static_cast<std::uint16_t>(
                    ((static_cast<unsigned int>(part[offset + 2]) & 0x1FU)
                     << 8U) |
                    part[offset + 3]);
            }
        }
        if (!valid) {
            pat_tables.reject(completed->key, completed->version);
            return;
        }
        std::map<std::uint16_t, TransportService> updated;
        for (const auto &[service_id, pmt_pid] : programs) {
            TransportService service{.service_id = service_id,
                                     .pmt_pid = pmt_pid,
                                     .pcr_pid = 0x1FFF,
                                     .name = {},
                                     .provider = {},
                                     .components = {}};
            if (const auto old = services.find(service_id);
                old != services.end()) {
                service.name = old->second.name;
                service.provider = old->second.provider;
                if (old->second.pmt_pid == pmt_pid) {
                    service.pcr_pid = old->second.pcr_pid;
                    service.components = old->second.components;
                }
            }
            updated.emplace(service_id, std::move(service));
        }
        for (const auto &[service_id, old] : services) {
            const auto replacement = programs.find(service_id);
            if (replacement == programs.end() ||
                replacement->second != old.pmt_pid) {
                pmt_tables.erase((static_cast<std::uint32_t>(old.pmt_pid)
                                  << 16U) |
                                 service_id);
            }
        }
        services = std::move(updated);
        pat_tables.commit(completed->key, completed->version);
    }

    void collect_pmt(const std::uint16_t pid,
                     const std::span<const std::uint8_t> section) {
        const std::uint16_t service_id = extension(section);
        const auto service = services.find(service_id);
        if (service == services.end() || service->second.pmt_pid != pid) {
            return;
        }
        auto completed = pmt_tables.add(table_key(pid, section), section);
        if (!completed.has_value()) {
            return;
        }
        std::uint16_t pcr_pid = 0x1FFF;
        std::vector<TransportStreamComponent> components;
        bool valid = true;
        for (const auto &bytes : completed->sections) {
            const std::span<const std::uint8_t> part(bytes);
            if (part.size() < 16) {
                valid = false;
                break;
            }
            pcr_pid = static_cast<std::uint16_t>(
                ((static_cast<unsigned int>(part[8]) & 0x1FU) << 8U) |
                part[9]);
            const std::size_t body_end = part.size() - 4;
            const std::size_t program_info_length =
                ((static_cast<std::size_t>(part[10]) & 0x0FU) << 8U) |
                part[11];
            std::size_t offset = 12 + program_info_length;
            if (offset > body_end) {
                valid = false;
                break;
            }
            while (offset < body_end) {
                if (offset + 5 > body_end) {
                    valid = false;
                    break;
                }
                const auto component_pid = static_cast<std::uint16_t>(
                    ((static_cast<unsigned int>(part[offset + 1]) & 0x1FU)
                     << 8U) |
                    part[offset + 2]);
                const std::size_t info_length =
                    ((static_cast<std::size_t>(part[offset + 3]) & 0x0FU)
                     << 8U) |
                    part[offset + 4];
                if (offset + 5 + info_length > body_end) {
                    valid = false;
                    break;
                }
                components.push_back({component_pid, part[offset]});
                offset += 5 + info_length;
            }
            if (!valid) {
                break;
            }
        }
        if (!valid) {
            pmt_tables.reject(completed->key, completed->version);
            return;
        }
        auto current = services.find(service_id);
        if (current != services.end() && current->second.pmt_pid == pid) {
            current->second.pcr_pid = pcr_pid;
            current->second.components = std::move(components);
            pmt_tables.commit(completed->key, completed->version);
        }
    }

    void collect_sdt(const std::uint16_t pid,
                     const std::span<const std::uint8_t> section) {
        auto completed = sdt_tables.add(table_key(pid, section), section);
        if (!completed.has_value()) {
            return;
        }
        std::map<std::uint16_t, std::pair<std::string, std::string>> names;
        bool valid = true;
        for (const auto &bytes : completed->sections) {
            const std::span<const std::uint8_t> part(bytes);
            if (part.size() < 15) {
                valid = false;
                break;
            }
            const std::size_t body_end = part.size() - 4;
            std::size_t offset = 11;
            while (offset < body_end) {
                if (offset + 5 > body_end) {
                    valid = false;
                    break;
                }
                const auto service_id = static_cast<std::uint16_t>(
                    (static_cast<unsigned int>(part[offset]) << 8U) |
                    part[offset + 1]);
                const std::size_t descriptors_length =
                    ((static_cast<std::size_t>(part[offset + 3]) & 0x0FU)
                     << 8U) |
                    part[offset + 4];
                const std::size_t descriptors_end =
                    offset + 5 + descriptors_length;
                if (descriptors_end > body_end) {
                    valid = false;
                    break;
                }
                std::size_t descriptor = offset + 5;
                while (descriptor < descriptors_end) {
                    if (descriptor + 2 > descriptors_end) {
                        valid = false;
                        break;
                    }
                    const std::size_t length = part[descriptor + 1];
                    if (descriptor + 2 + length > descriptors_end) {
                        valid = false;
                        break;
                    }
                    if (part[descriptor] == 0x48U && length >= 3) {
                        const auto payload =
                            part.subspan(descriptor + 2, length);
                        const std::size_t provider_length = payload[1];
                        if (2 + provider_length >= payload.size()) {
                            valid = false;
                            break;
                        }
                        const std::size_t name_length =
                            payload[2 + provider_length];
                        if (3 + provider_length + name_length >
                            payload.size()) {
                            valid = false;
                            break;
                        }
                        names[service_id] = {
                            si::dvb_text(payload.subspan(2, provider_length)),
                            si::dvb_text(payload.subspan(
                                3 + provider_length, name_length))};
                    }
                    descriptor += 2 + length;
                }
                if (!valid) {
                    break;
                }
                offset = descriptors_end;
            }
            if (!valid) {
                break;
            }
        }
        if (!valid) {
            sdt_tables.reject(completed->key, completed->version);
            return;
        }
        for (auto &[service_id, service] : services) {
            if (const auto found = names.find(service_id);
                found != names.end()) {
                service.provider = found->second.first;
                service.name = found->second.second;
            } else {
                service.provider.clear();
                service.name.clear();
            }
        }
        sdt_tables.commit(completed->key, completed->version);
    }

    void discard_pending() {
        pat_tables.discard_pending();
        pmt_tables.discard_pending();
        sdt_tables.discard_pending();
    }

    void reset_tables() {
        pat_tables.reset();
        pmt_tables.reset();
        sdt_tables.reset();
    }
};

TransportStreamModel::TransportStreamModel()
    : impl_(std::make_unique<Impl>()) {}
TransportStreamModel::~TransportStreamModel() noexcept = default;

void TransportStreamModel::reset() {
    const std::scoped_lock lock(impl_->mutex);
    impl_->services.clear();
    impl_->feed.reset();
    impl_->reset_tables();
}

void TransportStreamModel::on_discontinuity(
    const TransportDiscontinuity discontinuity) {
    const std::scoped_lock lock(impl_->mutex);
    impl_->feed.reset();
    impl_->discard_pending();
    if (discontinuity == TransportDiscontinuity::retune) {
        impl_->services.clear();
        impl_->reset_tables();
    }
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
