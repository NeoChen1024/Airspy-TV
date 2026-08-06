#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace airspy_tv::fec {

struct OuterFecStats {
    std::uint64_t rs_packets{};
    std::uint64_t rs_uncorrectable_packets{};
    std::uint64_t tei_packets{};
    std::uint64_t ts_packets{};
    // RS-derived payload error estimate and attempted payload bits. Corrected
    // bits are counted for valid codewords; an uncorrectable codeword is
    // conservatively charged as a fully invalid 188-byte payload so the
    // post-Viterbi BER remains live during an outer-lock failure.
    std::uint64_t corrected_payload_bits{};
    std::uint64_t compared_payload_bits{};
    int outer_deinterleaver_phase{-1};
    unsigned int outer_sync_distance{};
    std::uint32_t outer_rs_evidence{};
    bool rs_synchronized{};
    bool energy_synchronized{};
};

// Streaming DVB outer FEC: 12-branch convolutional deinterleaver with
// alignment search, shortened Reed-Solomon (204,188), energy descrambling,
// and MPEG-TS packet recovery. Input is hard bytes in transmission order:
// Viterbi output for DVB-T, unpacked QAM symbols for DVB-C. While the
// deinterleaver phase is unknown all 12 phases are tracked in parallel.
// Uncorrectable codewords preserve packet cadence and are emitted with the
// transport error indicator set instead of creating a continuity gap.
class OuterFec {
  public:
    OuterFec();
    ~OuterFec() noexcept;
    OuterFec(const OuterFec &) = delete;
    OuterFec &operator=(const OuterFec &) = delete;
    OuterFec(OuterFec &&) noexcept;
    OuterFec &operator=(OuterFec &&) noexcept;

    void reset();
    [[nodiscard]] std::vector<std::uint8_t>
    process(std::span<const std::uint8_t> hard_bytes);
    [[nodiscard]] OuterFecStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::fec
