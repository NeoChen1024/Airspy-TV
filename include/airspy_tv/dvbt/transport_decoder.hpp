#pragma once

#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/fec/soft_viterbi.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

struct TransportDecoderStats {
    std::uint64_t viterbi_bits{};
    std::uint64_t pre_viterbi_error_bits{};
    std::uint64_t pre_viterbi_compared_bits{};
    std::uint64_t post_viterbi_error_bits{};
    std::uint64_t post_viterbi_compared_bits{};
    std::uint64_t rs_packets{};
    std::uint64_t rs_uncorrectable_packets{};
    std::uint64_t tei_packets{};
    std::uint64_t ts_packets{};
    int outer_deinterleaver_phase{-1};
    unsigned int outer_sync_distance{};
    std::uint32_t outer_rs_evidence{};
    bool rs_synchronized{};
    bool energy_synchronized{};
    std::size_t viterbi_workers{};
};

// Zero in user-facing configuration means this portable hardware-concurrency
// default. std::thread reports logical processors and may return zero.
[[nodiscard]] inline std::size_t default_viterbi_worker_count() noexcept {
    return fec::default_viterbi_worker_count();
}

// Streaming DVB-T inner/outer FEC after soft bit deinterleaving. Input contains
// the punctured convolutional-code metrics in transmission order. Positive LLR
// values favour one; zero is an erasure.
class TransportDecoder {
  public:
    explicit TransportDecoder(CodeRate code_rate,
                              std::size_t viterbi_workers = 0);
    ~TransportDecoder() noexcept;

    TransportDecoder(const TransportDecoder &) = delete;
    TransportDecoder &operator=(const TransportDecoder &) = delete;
    TransportDecoder(TransportDecoder &&) noexcept;
    TransportDecoder &operator=(TransportDecoder &&) noexcept;

    void reset();
    [[nodiscard]] std::vector<std::uint8_t>
    process(std::span<const float> punctured_llrs);
    [[nodiscard]] std::vector<std::uint8_t>
    process_soft(std::span<const std::uint8_t> mother_metrics);
    [[nodiscard]] std::vector<std::uint8_t> flush();
    [[nodiscard]] TransportDecoderStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
