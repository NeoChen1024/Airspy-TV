#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/soft_demapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::dvbt::CodeRate;
using airspy_tv::dvbt::Constellation;
using airspy_tv::dvbt::MaxLogDemapper;
using airspy_tv::dvbt::SymbolDeinterleaver;
using airspy_tv::dvbt::TransmissionMode;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

void test_constellations() {
    for (const auto constellation :
         {Constellation::qpsk, Constellation::qam16, Constellation::qam64}) {
        const MaxLogDemapper demapper{constellation};
        const auto points = demapper.constellation_points();
        const std::size_t bit_count = demapper.bits_per_symbol();
        require(points.size() == (std::size_t{1} << bit_count),
                "constellation point count");

        float average_power = 0.0F;
        for (const auto point : points) {
            average_power += std::norm(point);
        }
        average_power /= static_cast<float>(points.size());
        require(std::abs(average_power - 1.0F) < 1.0e-5F,
                "constellation normalization");

        const std::vector<float> reliability(points.size(), 3.0F);
        std::vector<float> llrs(points.size() * bit_count);
        demapper.demap(points, reliability, llrs);
        for (std::size_t label = 0; label < points.size(); ++label) {
            for (std::size_t bit = 0; bit < bit_count; ++bit) {
                const bool expected_one =
                    ((label >> (bit_count - bit - 1)) & 1U) != 0U;
                const float llr = llrs[(label * bit_count) + bit];
                require(expected_one ? llr > 0.0F : llr < 0.0F,
                        "DVB-T constellation bit ordering");
            }
        }

        std::vector<float> erased(points.size(), 0.0F);
        demapper.demap(points, erased, llrs);
        require(std::ranges::all_of(
                    llrs, [](const float llr) { return llr == 0.0F; }),
                "zero reliability must erase a carrier");
    }
}

void test_symbol_deinterleaver(const TransmissionMode mode) {
    const SymbolDeinterleaver deinterleaver{mode};
    const auto permutation = deinterleaver.permutation();
    const std::size_t carrier_count =
        airspy_tv::dvbt::payload_carrier_count(mode);
    require(permutation.size() == carrier_count, "symbol permutation length");
    std::vector<std::size_t> sorted(permutation.begin(), permutation.end());
    std::ranges::sort(sorted);
    for (std::size_t index = 0; index < sorted.size(); ++index) {
        require(sorted[index] == index, "symbol permutation must be bijective");
    }

    constexpr std::size_t bits = 6;
    std::vector<float> expected(carrier_count * bits);
    std::iota(expected.begin(), expected.end(), 0.0F);
    std::vector<float> interleaved(expected.size());
    std::vector<float> recovered(expected.size());
    for (const std::size_t parity : {0U, 1U}) {
        for (std::size_t carrier = 0; carrier < carrier_count; ++carrier) {
            const std::size_t input_carrier =
                parity != 0U ? carrier : permutation[carrier];
            const std::size_t output_carrier =
                parity != 0U ? permutation[carrier] : carrier;
            std::ranges::copy(
                expected.begin() +
                    static_cast<std::ptrdiff_t>(output_carrier * bits),
                expected.begin() +
                    static_cast<std::ptrdiff_t>((output_carrier + 1) * bits),
                interleaved.begin() +
                    static_cast<std::ptrdiff_t>(input_carrier * bits));
        }
        deinterleaver.process(interleaved, bits, parity, recovered);
        require(recovered == expected, "symbol deinterleaver round trip");
    }
}

void test_symbol_permutation_prefixes() {
    constexpr std::array<std::size_t, 12> expected_2k{
        0, 1024, 16, 1025, 128, 1056, 2, 1280, 4, 1088, 513, 1160};
    constexpr std::array<std::size_t, 12> expected_8k{
        0, 4096, 128, 4128, 2048, 4104, 1, 5120, 256, 4192, 2560, 4140};
    const SymbolDeinterleaver deinterleaver_2k{TransmissionMode::k2};
    const SymbolDeinterleaver deinterleaver_8k{TransmissionMode::k8};
    require(std::ranges::equal(expected_2k,
                               deinterleaver_2k.permutation().first(12)),
            "ETSI 2K symbol permutation prefix");
    require(std::ranges::equal(expected_8k,
                               deinterleaver_8k.permutation().first(12)),
            "ETSI 8K symbol permutation prefix");
}

void test_bit_deinterleaver() {
    constexpr std::array<std::size_t, 6> offsets{0, 63, 105, 42, 21, 84};
    for (const std::size_t bits : {2U, 4U, 6U}) {
        std::vector<float> input(126 * bits);
        std::iota(input.begin(), input.end(), 0.0F);
        std::vector<float> output(input.size());
        airspy_tv::dvbt::bit_deinterleave(input, bits, output);

        for (std::size_t position = 0; position < 126; ++position) {
            for (std::size_t output_bit = 0; output_bit < bits; ++output_bit) {
                const std::size_t half = bits / 2;
                const std::size_t source_bit =
                    (output_bit / half) + (2 * (output_bit % half));
                const std::size_t source_position =
                    (position + 126 - offsets[source_bit]) % 126;
                require(output[(position * bits) + output_bit] ==
                            input[(source_position * bits) + source_bit],
                        "soft bit deinterleaver mapping");
            }
        }
    }
}

void test_depuncturer() {
    const std::array rates{CodeRate::rate_1_2, CodeRate::rate_2_3,
                           CodeRate::rate_3_4, CodeRate::rate_5_6,
                           CodeRate::rate_7_8};
    const std::array transmitted_per_period{2U, 3U, 4U, 6U, 8U};
    const std::array mother_period{2U, 4U, 6U, 10U, 14U};
    for (std::size_t index = 0; index < rates.size(); ++index) {
        std::vector<float> input(transmitted_per_period[index] * 3);
        std::iota(input.begin(), input.end(), 1.0F);
        std::vector<float> output(
            airspy_tv::dvbt::depunctured_size(input.size(), rates[index]));
        airspy_tv::dvbt::depuncture(input, rates[index], output);
        require(output.size() == mother_period[index] * 3, "depunctured size");
        require(std::ranges::count(output, 0.0F) ==
                    static_cast<std::ptrdiff_t>(output.size() - input.size()),
                "neutral punctured metrics");
    }

    const std::array<float, 6> input{1, 2, 3, 4, 5, 6};
    std::array<float, 8> output{};
    airspy_tv::dvbt::depuncture(input, CodeRate::rate_2_3, output);
    require(output == std::array<float, 8>{1, 2, 0, 3, 4, 5, 0, 6},
            "DVB-T 2/3 puncture pattern");
}

} // namespace

int main() {
    try {
        test_constellations();
        test_symbol_deinterleaver(TransmissionMode::k2);
        test_symbol_deinterleaver(TransmissionMode::k8);
        test_symbol_permutation_prefixes();
        test_bit_deinterleaver();
        test_depuncturer();
    } catch (const std::exception &error) {
        std::cerr << "DVB-T inner decoder test failed: " << error.what()
                  << '\n';
        return 1;
    }
    std::cout << "DVB-T inner decoder tests passed\n";
    return 0;
}
