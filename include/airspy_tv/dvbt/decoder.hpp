#pragma once

#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/soft_demapper.hpp"
#include "airspy_tv/dvbt/transport_decoder.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

struct DecoderParameters {
    TransmissionMode mode{TransmissionMode::k8};
    Constellation constellation{Constellation::qam64};
    CodeRate code_rate{CodeRate::rate_2_3};
    // Zero selects default_viterbi_worker_count().
    std::size_t viterbi_workers{};

    bool operator==(const DecoderParameters &) const = default;
};

struct DecoderTiming {
    float demap_time_ms{};
    float deinterleave_time_ms{};
    float transport_time_ms{};
};

// Native DVB-T data path beginning at the equalized payload-carrier boundary.
// The OFDM synchronizer/equalizer supplies exactly 1512 (2K) or 6048 (8K)
// payload carriers and an effective |H|^2/noise-variance reliability per
// carrier. symbol_index is the TPS frame's 0..67 OFDM symbol index.
class Decoder {
  public:
    explicit Decoder(DecoderParameters parameters);
    ~Decoder() noexcept;

    Decoder(const Decoder &) = delete;
    Decoder &operator=(const Decoder &) = delete;
    Decoder(Decoder &&) noexcept;
    Decoder &operator=(Decoder &&) noexcept;

    void reset();
    [[nodiscard]] std::vector<std::uint8_t>
    process_symbol(std::span<const std::complex<float>> equalized_carriers,
                   std::span<const float> reliability,
                   std::size_t symbol_index);
    [[nodiscard]] std::vector<std::uint8_t>
    process_metrics(std::span<const float> punctured_llrs);
    [[nodiscard]] std::vector<std::uint8_t>
    process_soft_metrics(std::span<const std::uint8_t> mother_metrics);
    [[nodiscard]] std::vector<std::uint8_t> flush();
    [[nodiscard]] DecoderParameters parameters() const noexcept;
    [[nodiscard]] DecoderTiming timing() const noexcept;
    [[nodiscard]] TransportDecoderStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
