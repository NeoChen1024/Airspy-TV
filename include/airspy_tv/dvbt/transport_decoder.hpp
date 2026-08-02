#pragma once

#include "airspy_tv/dvbt/inner_decoder.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

struct TransportDecoderStats {
    std::uint64_t viterbi_bits{};
    std::uint64_t rs_packets{};
    std::uint64_t rs_uncorrectable_packets{};
    std::uint64_t ts_packets{};
    int outer_deinterleaver_phase{-1};
    bool rs_synchronized{};
    bool energy_synchronized{};
};

// Streaming DVB-T inner/outer FEC after soft bit deinterleaving. Input contains
// the punctured convolutional-code metrics in transmission order. Positive LLR
// values favour one; zero is an erasure.
class TransportDecoder {
  public:
    explicit TransportDecoder(CodeRate code_rate);
    ~TransportDecoder() noexcept;

    TransportDecoder(const TransportDecoder &) = delete;
    TransportDecoder &operator=(const TransportDecoder &) = delete;
    TransportDecoder(TransportDecoder &&) noexcept;
    TransportDecoder &operator=(TransportDecoder &&) noexcept;

    void reset();
    [[nodiscard]] std::vector<std::uint8_t>
    process(std::span<const float> punctured_llrs);
    [[nodiscard]] TransportDecoderStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
