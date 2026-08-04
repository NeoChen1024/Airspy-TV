#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace airspy_tv {

// Why the decoded transport stream may contain a seam. Ordinary per-packet
// corruption stays invisible here — it is marked in-band by the TS
// transport_error_indicator (TEI) bit and the demuxer discards those
// payloads. These are stream-level events that bound clean regions and drive
// the playback recovery policy.
enum class TransportDiscontinuity {
    // A gated/hopeless region was dropped and the FEC trellis + outer state
    // were re-seeded: the packet stream has a gap (continuity counters
    // jump). The demuxer error-conceals the seam; playback continues.
    fec_region_reset,
    // The input ended (EOF / source drop). The queued tail is still valid
    // and plays out; the next read returns EOF and playback stops cleanly.
    stream_end,
    // The receiver was reset (retune, source switch, or dropped-block
    // recovery): the content may have changed entirely. Playback must
    // restart so the demuxer re-parses the new channel's tables instead of
    // concatenating two unrelated streams.
    retune,
};

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
