#pragma once

#include <cstddef>
#include <span>
#include <vector>

namespace airspy_tv::dvbt {

enum class TransmissionMode {
    k2,
    k8,
};

enum class CodeRate {
    rate_1_2,
    rate_2_3,
    rate_3_4,
    rate_5_6,
    rate_7_8,
};

[[nodiscard]] std::size_t payload_carrier_count(TransmissionMode mode);

class SymbolDeinterleaver {
  public:
    explicit SymbolDeinterleaver(TransmissionMode mode);

    [[nodiscard]] TransmissionMode mode() const noexcept;
    [[nodiscard]] std::span<const std::size_t> permutation() const noexcept;

    // Input and output contain bits_per_carrier soft metrics per carrier.
    void process(std::span<const float> input, std::size_t bits_per_carrier,
                 std::size_t symbol_index, std::span<float> output) const;

  private:
    TransmissionMode mode_;
    std::vector<std::size_t> permutation_;
};

// DVB-T non-hierarchical inner bit deinterleaving. Input and output use
// carrier-major, MSB-first soft metrics and may contain multiple OFDM symbols.
void bit_deinterleave(std::span<const float> input,
                      std::size_t bits_per_carrier, std::span<float> output);

[[nodiscard]] std::size_t depunctured_size(std::size_t transmitted_metrics,
                                           CodeRate rate);

// Restores the rate-1/2 mother-code stream. Punctured bits receive neutral_llr.
void depuncture(std::span<const float> input, CodeRate rate,
                std::span<float> output, float neutral_llr = 0.0F);

} // namespace airspy_tv::dvbt
