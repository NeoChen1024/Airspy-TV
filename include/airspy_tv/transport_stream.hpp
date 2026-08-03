#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace airspy_tv {

struct TransportStreamComponent {
    std::uint16_t pid{};
    std::uint8_t stream_type{};

    bool operator==(const TransportStreamComponent &) const = default;
};

struct TransportService {
    std::uint16_t service_id{};
    std::uint16_t pmt_pid{0x1FFF};
    std::uint16_t pcr_pid{0x1FFF};
    std::string name;
    std::string provider;
    std::vector<TransportStreamComponent> components;
};

// Incremental MPEG-TS PSI/SI model. Corrupt (TEI) packets and sections with an
// invalid MPEG-2 CRC are ignored; snapshots are safe to read from the UI while
// the decoder callback updates the model.
class TransportStreamModel {
  public:
    TransportStreamModel();
    ~TransportStreamModel() noexcept;
    TransportStreamModel(const TransportStreamModel &) = delete;
    TransportStreamModel &operator=(const TransportStreamModel &) = delete;
    TransportStreamModel(TransportStreamModel &&) = delete;
    TransportStreamModel &operator=(TransportStreamModel &&) = delete;

    void reset();
    void consume(std::span<const std::uint8_t> transport_stream);
    [[nodiscard]] std::vector<TransportService> services() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
