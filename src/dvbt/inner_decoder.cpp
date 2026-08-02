#include "airspy_tv/dvbt/inner_decoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr std::size_t bit_interleaver_size = 126;
constexpr std::array<std::size_t, 6> bit_offsets{0, 63, 105, 42, 21, 84};
constexpr std::array<int, 2> puncture_1_2{1, 1};
constexpr std::array<int, 4> puncture_2_3{1, 1, 0, 1};
constexpr std::array<int, 6> puncture_3_4{1, 1, 0, 1, 1, 0};
constexpr std::array<int, 10> puncture_5_6{1, 1, 0, 1, 1, 0, 0, 1, 1, 0};
constexpr std::array<int, 14> puncture_7_8{1, 1, 0, 1, 0, 1, 0,
                                           1, 1, 0, 0, 1, 1, 0};

[[nodiscard]] std::span<const int> puncture_pattern(const CodeRate rate) {
    switch (rate) {
    case CodeRate::rate_1_2:
        return puncture_1_2;
    case CodeRate::rate_2_3:
        return puncture_2_3;
    case CodeRate::rate_3_4:
        return puncture_3_4;
    case CodeRate::rate_5_6:
        return puncture_5_6;
    case CodeRate::rate_7_8:
        return puncture_7_8;
    }
    throw std::invalid_argument("unsupported DVB-T code rate");
}

[[nodiscard]] std::vector<std::size_t>
make_symbol_permutation(const TransmissionMode mode) {
    static constexpr std::array<std::size_t, 10> bit_permutation_2k{
        4, 3, 9, 6, 2, 8, 1, 5, 7, 0};
    static constexpr std::array<std::size_t, 12> bit_permutation_8k{
        7, 1, 4, 2, 9, 6, 8, 10, 0, 3, 11, 5};

    const std::size_t fft_length = mode == TransmissionMode::k2 ? 2048 : 8192;
    const std::size_t register_bits = mode == TransmissionMode::k2 ? 11 : 13;
    const std::span<const std::size_t> bit_permutation =
        mode == TransmissionMode::k2
            ? std::span<const std::size_t>{bit_permutation_2k}
            : std::span<const std::size_t>{bit_permutation_8k};
    std::vector<std::size_t> permutation;
    permutation.reserve(payload_carrier_count(mode));
    std::size_t shift_register = 0;

    for (std::size_t index = 0; index < fft_length; ++index) {
        if (index < 2) {
            shift_register = 0;
        } else if (index == 2) {
            shift_register = 1;
        } else {
            const std::size_t new_bit =
                mode == TransmissionMode::k2
                    ? (shift_register ^ (shift_register >> 3)) & 1U
                    : (shift_register ^ (shift_register >> 1) ^
                       (shift_register >> 4) ^ (shift_register >> 6)) &
                          1U;
            shift_register =
                ((shift_register >> 1) | (new_bit << (register_bits - 2))) &
                ((std::size_t{1} << register_bits) - 1);
        }

        std::size_t permuted_register = 0;
        for (std::size_t bit = 0; bit < bit_permutation.size(); ++bit) {
            permuted_register |= ((shift_register >> bit) & 1U)
                                 << bit_permutation[bit];
        }
        const std::size_t value =
            ((index & 1U) << (register_bits - 1)) + permuted_register;
        if (value < payload_carrier_count(mode)) {
            permutation.push_back(value);
        }
    }
    if (permutation.size() != payload_carrier_count(mode)) {
        throw std::runtime_error("failed to generate DVB-T symbol permutation");
    }
    return permutation;
}

} // namespace

std::size_t payload_carrier_count(const TransmissionMode mode) {
    return mode == TransmissionMode::k2 ? 1512 : 6048;
}

SymbolDeinterleaver::SymbolDeinterleaver(const TransmissionMode mode)
    : mode_(mode), permutation_(make_symbol_permutation(mode)) {}

TransmissionMode SymbolDeinterleaver::mode() const noexcept { return mode_; }

std::span<const std::size_t> SymbolDeinterleaver::permutation() const noexcept {
    return permutation_;
}

void SymbolDeinterleaver::process(const std::span<const float> input,
                                  const std::size_t bits_per_carrier,
                                  const std::size_t symbol_index,
                                  const std::span<float> output) const {
    const std::size_t expected = permutation_.size() * bits_per_carrier;
    if (bits_per_carrier == 0 || input.size() != expected ||
        output.size() != expected) {
        throw std::invalid_argument("symbol deinterleaver size mismatch");
    }
    if (input.data() == output.data()) {
        throw std::invalid_argument(
            "in-place symbol deinterleaving is unsupported");
    }

    for (std::size_t carrier = 0; carrier < permutation_.size(); ++carrier) {
        const std::size_t input_carrier =
            (symbol_index & 1U) != 0U ? carrier : permutation_[carrier];
        const std::size_t output_carrier =
            (symbol_index & 1U) != 0U ? permutation_[carrier] : carrier;
        std::ranges::copy(
            input.subspan(input_carrier * bits_per_carrier, bits_per_carrier),
            output.begin() +
                static_cast<std::ptrdiff_t>(output_carrier * bits_per_carrier));
    }
}

void bit_deinterleave(const std::span<const float> input,
                      const std::size_t bits_per_carrier,
                      const std::span<float> output) {
    if (bits_per_carrier != 2 && bits_per_carrier != 4 &&
        bits_per_carrier != 6) {
        throw std::invalid_argument("unsupported DVB-T modulation order");
    }
    const std::size_t metrics_per_block =
        bit_interleaver_size * bits_per_carrier;
    if (input.size() != output.size() ||
        input.size() % metrics_per_block != 0) {
        throw std::invalid_argument("bit deinterleaver size mismatch");
    }
    if (input.data() == output.data()) {
        throw std::invalid_argument(
            "in-place bit deinterleaving is unsupported");
    }

    for (std::size_t block = 0; block < input.size() / metrics_per_block;
         ++block) {
        const std::size_t block_start = block * metrics_per_block;
        for (std::size_t position = 0; position < bit_interleaver_size;
             ++position) {
            for (std::size_t output_bit = 0; output_bit < bits_per_carrier;
                 ++output_bit) {
                const std::size_t half = bits_per_carrier / 2;
                const std::size_t source_bit =
                    (output_bit / half) + (2 * (output_bit % half));
                const std::size_t source_position =
                    (position + bit_interleaver_size -
                     bit_offsets[source_bit]) %
                    bit_interleaver_size;
                output[block_start + (position * bits_per_carrier) +
                       output_bit] =
                    input[block_start + (source_position * bits_per_carrier) +
                          source_bit];
            }
        }
    }
}

std::size_t depunctured_size(const std::size_t transmitted_metrics,
                             const CodeRate rate) {
    const auto pattern = puncture_pattern(rate);
    const std::size_t transmitted_per_period =
        static_cast<std::size_t>(std::ranges::count(pattern, 1));
    if (transmitted_per_period == 0) {
        throw std::invalid_argument("empty DVB-T puncture pattern");
    }
    if (transmitted_metrics % transmitted_per_period != 0) {
        throw std::invalid_argument("punctured metric stream is misaligned");
    }
    return (transmitted_metrics / transmitted_per_period) * pattern.size();
}

void depuncture(const std::span<const float> input, const CodeRate rate,
                const std::span<float> output, const float neutral_llr) {
    const auto pattern = puncture_pattern(rate);
    if (output.size() != depunctured_size(input.size(), rate)) {
        throw std::invalid_argument("depuncturer output size mismatch");
    }

    std::size_t input_index = 0;
    for (std::size_t output_index = 0; output_index < output.size();
         ++output_index) {
        if (pattern[output_index % pattern.size()] != 0) {
            output[output_index] = input[input_index++];
        } else {
            output[output_index] = neutral_llr;
        }
    }
}

} // namespace airspy_tv::dvbt
