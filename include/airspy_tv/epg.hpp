#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace airspy_tv {

struct EpgEvent {
    std::uint16_t event_id{};
    // Start of the event, UTC unix seconds.
    std::uint64_t start_time_utc{};
    std::uint32_t duration_seconds{};
    // ETSI EN 300 468 running status (0 = undefined, 4 = running).
    std::uint8_t running_status{};
    std::string name;
    std::string description;
    std::string genre;
};

struct EpgSnapshot {
    // EIT present/following events for one service, sorted by start time.
    std::vector<EpgEvent> events;
    // Current UTC wall clock as last broadcast by TDT/TOT (unix seconds);
    // empty until the first TDT/TOT section arrives.
    std::optional<std::uint64_t> utc_now;
};

// Incremental EPG model fed with the same transport stream as the service
// model. Parses EIT present/following (now/next) and TDT/TOT clock data with
// its own section assembler, so it is independent of the SDR layer. Each EIT
// section names its service via the table_id extension, so no PAT is needed.
// Corrupt (TEI) packets and sections with an invalid MPEG-2 CRC are ignored;
// snapshots are safe to read from the UI while the decoder callback updates
// the model.
class EpgModel {
  public:
    EpgModel();
    ~EpgModel() noexcept;
    EpgModel(const EpgModel &) = delete;
    EpgModel &operator=(const EpgModel &) = delete;
    EpgModel(EpgModel &&) = delete;
    EpgModel &operator=(EpgModel &&) = delete;

    void reset();
    void consume(std::span<const std::uint8_t> transport_stream);
    [[nodiscard]] EpgSnapshot snapshot(std::uint16_t service_id) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
