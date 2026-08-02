#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/transport_decoder.hpp"

extern "C" {
#include <correct.h>
}

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::dvbt::CodeRate;
using airspy_tv::dvbt::Constellation;
using airspy_tv::dvbt::Decoder;
using airspy_tv::dvbt::DecoderParameters;
using airspy_tv::dvbt::MaxLogDemapper;
using airspy_tv::dvbt::SymbolDeinterleaver;
using airspy_tv::dvbt::TransmissionMode;
using airspy_tv::dvbt::TransportDecoder;

constexpr std::size_t ts_packet_size = 188;
constexpr std::size_t rs_packet_size = 204;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

std::uint8_t clock_prbs(std::uint16_t &shift_register) {
    std::uint8_t result = 0;
    for (int bit = 0; bit < 8; ++bit) {
        const std::uint16_t feedback =
            ((shift_register >> 13) ^ (shift_register >> 14)) & 1U;
        shift_register = static_cast<std::uint16_t>(
            ((shift_register << 1) | feedback) & 0x7FFFU);
        result = static_cast<std::uint8_t>((result << 1) | feedback);
    }
    return result;
}

std::vector<std::uint8_t>
energy_scramble(const std::span<const std::uint8_t> transport_stream) {
    require(transport_stream.size() % (8 * ts_packet_size) == 0,
            "energy frame alignment");
    std::vector<std::uint8_t> output(transport_stream.size());
    for (std::size_t group = 0; group < transport_stream.size();
         group += 8 * ts_packet_size) {
        std::uint16_t shift_register = 0x00A9;
        for (std::size_t packet = 0; packet < 8; ++packet) {
            const std::size_t offset = group + (packet * ts_packet_size);
            output[offset] = packet == 0 ? 0xB8 : 0x47;
            for (std::size_t byte = 1; byte < ts_packet_size; ++byte) {
                output[offset + byte] = transport_stream[offset + byte] ^
                                        clock_prbs(shift_register);
            }
            static_cast<void>(clock_prbs(shift_register));
        }
    }
    return output;
}

std::vector<std::uint8_t>
rs_encode(const std::span<const std::uint8_t> randomized) {
    correct_reed_solomon *codec = correct_reed_solomon_create(
        correct_rs_primitive_polynomial_8_4_3_2_0, 0, 1, 16);
    require(codec != nullptr, "create RS encoder");
    std::vector<std::uint8_t> output((randomized.size() / ts_packet_size) *
                                     rs_packet_size);
    for (std::size_t input = 0, encoded = 0; input < randomized.size();
         input += ts_packet_size, encoded += rs_packet_size) {
        const ssize_t size = correct_reed_solomon_encode(
            codec, randomized.data() + input, ts_packet_size,
            output.data() + encoded);
        // libcorrect reports the parent RS(255,239) block length even when it
        // emits a shortened 204-byte codeword.
        require(size == 255, "RS encode");
    }
    correct_reed_solomon_destroy(codec);
    return output;
}

std::vector<std::uint8_t>
outer_interleave(const std::span<const std::uint8_t> input) {
    constexpr std::size_t branches = 12;
    constexpr std::size_t step = 17;
    std::array<std::deque<std::uint8_t>, branches> queues;
    for (std::size_t branch = 0; branch < branches; ++branch) {
        queues[branch].assign(branch * step, 0);
    }

    std::vector<std::uint8_t> output;
    output.reserve(input.size());
    std::size_t branch = 0;
    for (const std::uint8_t byte : input) {
        auto &queue = queues[branch];
        queue.push_front(byte);
        output.push_back(queue.back());
        queue.pop_back();
        branch = (branch + 1) % branches;
    }
    return output;
}

std::vector<std::uint8_t>
convolutional_encode(const std::span<const std::uint8_t> input) {
    constexpr std::array<std::uint8_t, 2> polynomials{0117, 0155};
    std::vector<std::uint8_t> output;
    output.reserve(input.size() * 16);
    std::uint8_t shift_register = 0;
    for (const std::uint8_t byte : input) {
        for (int bit = 7; bit >= 0; --bit) {
            shift_register = static_cast<std::uint8_t>(
                ((shift_register << 1) | ((byte >> bit) & 1U)) & 0x7FU);
            for (const std::uint8_t polynomial : polynomials) {
                output.push_back(static_cast<std::uint8_t>(
                    std::popcount(static_cast<unsigned int>(shift_register &
                                                            polynomial)) &
                    1));
            }
        }
    }
    return output;
}

std::span<const int> puncture_pattern(const CodeRate rate) {
    static constexpr std::array<int, 2> rate_1_2{1, 1};
    static constexpr std::array<int, 4> rate_2_3{1, 1, 0, 1};
    static constexpr std::array<int, 6> rate_3_4{1, 1, 0, 1, 1, 0};
    static constexpr std::array<int, 10> rate_5_6{1, 1, 0, 1, 1, 0, 0, 1, 1, 0};
    static constexpr std::array<int, 14> rate_7_8{1, 1, 0, 1, 0, 1, 0,
                                                  1, 1, 0, 0, 1, 1, 0};
    switch (rate) {
    case CodeRate::rate_1_2:
        return rate_1_2;
    case CodeRate::rate_2_3:
        return rate_2_3;
    case CodeRate::rate_3_4:
        return rate_3_4;
    case CodeRate::rate_5_6:
        return rate_5_6;
    case CodeRate::rate_7_8:
        return rate_7_8;
    }
    return {};
}

std::vector<float> make_metrics(const std::span<const std::uint8_t> bits,
                                const CodeRate rate) {
    const auto pattern = puncture_pattern(rate);
    std::vector<float> metrics;
    for (std::size_t index = 0; index < bits.size(); ++index) {
        if (pattern[index % pattern.size()] != 0) {
            metrics.push_back(bits[index] != 0U ? 16.0F : -16.0F);
        }
    }
    return metrics;
}

std::vector<float> bit_interleave(const std::span<const float> input,
                                  const std::size_t bits_per_carrier) {
    constexpr std::array<std::size_t, 6> offsets{0, 63, 105, 42, 21, 84};
    constexpr std::size_t block_carriers = 126;
    const std::size_t block_metrics = block_carriers * bits_per_carrier;
    require(input.size() % block_metrics == 0, "bit interleaver alignment");
    std::vector<float> output(input.size());
    for (std::size_t block = 0; block < input.size(); block += block_metrics) {
        for (std::size_t position = 0; position < block_carriers; ++position) {
            for (std::size_t output_bit = 0; output_bit < bits_per_carrier;
                 ++output_bit) {
                const std::size_t half = bits_per_carrier / 2;
                const std::size_t source_bit =
                    (output_bit / half) + (2 * (output_bit % half));
                const std::size_t source_position =
                    (position + block_carriers - offsets[source_bit]) %
                    block_carriers;
                output[block + (source_position * bits_per_carrier) +
                       source_bit] =
                    input[block + (position * bits_per_carrier) + output_bit];
            }
        }
    }
    return output;
}

std::vector<float> symbol_interleave(const std::span<const float> input,
                                     const std::size_t bits_per_carrier,
                                     const SymbolDeinterleaver &deinterleaver,
                                     const std::size_t symbol_index) {
    const auto permutation = deinterleaver.permutation();
    std::vector<float> output(input.size());
    for (std::size_t carrier = 0; carrier < permutation.size(); ++carrier) {
        const std::size_t input_carrier =
            (symbol_index & 1U) != 0U ? carrier : permutation[carrier];
        const std::size_t output_carrier =
            (symbol_index & 1U) != 0U ? permutation[carrier] : carrier;
        std::ranges::copy(
            input.subspan(output_carrier * bits_per_carrier, bits_per_carrier),
            output.begin() +
                static_cast<std::ptrdiff_t>(input_carrier * bits_per_carrier));
    }
    return output;
}

std::vector<std::uint8_t> make_transport_stream() {
    constexpr std::size_t packet_count = 192;
    std::vector<std::uint8_t> stream(packet_count * ts_packet_size);
    for (std::size_t packet = 0; packet < packet_count; ++packet) {
        const std::size_t offset = packet * ts_packet_size;
        stream[offset] = 0x47;
        stream[offset + 1] = 0x1F;
        stream[offset + 2] = 0xFF;
        stream[offset + 3] = static_cast<std::uint8_t>(
            0x10U | static_cast<std::uint8_t>(packet & 0x0FU));
        for (std::size_t byte = 4; byte < ts_packet_size; ++byte) {
            stream[offset + byte] =
                static_cast<std::uint8_t>((packet * 29U + byte * 17U) & 0xFFU);
        }
    }
    return stream;
}

void test_transport_decoder(const CodeRate rate) {
    const auto expected = make_transport_stream();
    const auto randomized = energy_scramble(expected);
    const auto rs = rs_encode(randomized);
    const auto interleaved = outer_interleave(rs);
    const auto encoded = convolutional_encode(interleaved);
    const auto metrics = make_metrics(encoded, rate);

    TransportDecoder decoder{rate};
    std::vector<std::uint8_t> recovered;
    constexpr std::size_t chunk_size = 997;
    for (std::size_t offset = 0; offset < metrics.size();
         offset += chunk_size) {
        const auto decoded =
            decoder.process(std::span<const float>{metrics}.subspan(
                offset, std::min(chunk_size, metrics.size() - offset)));
        recovered.insert(recovered.end(), decoded.begin(), decoded.end());
    }

    if (recovered.empty()) {
        const auto failed_stats = decoder.stats();
        throw std::runtime_error(
            "decoder produced no transport packets; outer phase=" +
            std::to_string(failed_stats.outer_deinterleaver_phase) +
            ", RS=" + std::to_string(failed_stats.rs_packets) + ", failures=" +
            std::to_string(failed_stats.rs_uncorrectable_packets) +
            ", energy=" + std::to_string(failed_stats.energy_synchronized));
    }
    require(recovered.size() % ts_packet_size == 0, "TS packet alignment");
    require(
        std::ranges::all_of(
            std::views::iota(std::size_t{0}, recovered.size() / ts_packet_size),
            [&](const std::size_t packet) {
                return recovered[packet * ts_packet_size] == 0x47;
            }),
        "TS sync bytes");
    const auto match = std::search(expected.begin(), expected.end(),
                                   recovered.begin(), recovered.end());
    require(match != expected.end(), "recovered TS is an original subsequence");

    const auto stats = decoder.stats();
    require(stats.rs_synchronized, "RS synchronized");
    require(stats.energy_synchronized, "energy synchronized");
    require(stats.rs_uncorrectable_packets == 0, "no RS failures");
    require(stats.ts_packets == recovered.size() / ts_packet_size,
            "TS packet statistics");
}

void test_equalized_symbol_decoder(const DecoderParameters parameters) {
    const auto expected = make_transport_stream();
    const auto randomized = energy_scramble(expected);
    const auto rs = rs_encode(randomized);
    const auto interleaved = outer_interleave(rs);
    const auto encoded = convolutional_encode(interleaved);
    const auto metrics = make_metrics(encoded, parameters.code_rate);

    const std::size_t bits =
        airspy_tv::dvbt::bits_per_symbol(parameters.constellation);
    const std::size_t carriers =
        airspy_tv::dvbt::payload_carrier_count(parameters.mode);
    const std::size_t metrics_per_symbol = carriers * bits;
    const std::size_t symbol_count = metrics.size() / metrics_per_symbol;
    require(symbol_count > 0, "enough symbols for equalized decoder test");

    Decoder decoder{parameters};
    const MaxLogDemapper mapper{parameters.constellation};
    const SymbolDeinterleaver symbol_permutation{parameters.mode};
    const auto points = mapper.constellation_points();
    std::vector<float> reliability(carriers, 32.0F);
    std::vector<std::uint8_t> recovered;

    for (std::size_t symbol = 0; symbol < symbol_count; ++symbol) {
        const auto decoded_metrics = std::span<const float>{metrics}.subspan(
            symbol * metrics_per_symbol, metrics_per_symbol);
        const auto bit_interleaved = bit_interleave(decoded_metrics, bits);
        const auto transmitted = symbol_interleave(bit_interleaved, bits,
                                                   symbol_permutation, symbol);
        std::vector<std::complex<float>> equalized(carriers);
        for (std::size_t carrier = 0; carrier < carriers; ++carrier) {
            std::size_t label = 0;
            for (std::size_t bit = 0; bit < bits; ++bit) {
                label = (label << 1) |
                        (transmitted[(carrier * bits) + bit] > 0.0F ? 1U : 0U);
            }
            equalized[carrier] = points[label];
        }
        const auto output =
            decoder.process_symbol(equalized, reliability, symbol % 68);
        recovered.insert(recovered.end(), output.begin(), output.end());
    }

    require(!recovered.empty(), "equalized symbols produced TS");
    const auto match = std::search(expected.begin(), expected.end(),
                                   recovered.begin(), recovered.end());
    require(match != expected.end(), "equalized-symbol TS round trip");
    require(decoder.stats().rs_uncorrectable_packets == 0,
            "equalized-symbol RS failures");
}

} // namespace

int main() {
    try {
        test_transport_decoder(CodeRate::rate_1_2);
        test_transport_decoder(CodeRate::rate_2_3);
        test_transport_decoder(CodeRate::rate_3_4);
        test_transport_decoder(CodeRate::rate_5_6);
        test_transport_decoder(CodeRate::rate_7_8);
        test_equalized_symbol_decoder(
            {TransmissionMode::k2, Constellation::qpsk, CodeRate::rate_1_2});
        test_equalized_symbol_decoder(
            {TransmissionMode::k8, Constellation::qam64, CodeRate::rate_2_3});
    } catch (const std::exception &error) {
        std::cerr << "DVB-T transport decoder test failed: " << error.what()
                  << '\n';
        return 1;
    }
    std::cout << "DVB-T transport decoder tests passed\n";
    return 0;
}
