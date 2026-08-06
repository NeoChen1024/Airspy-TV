#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/transport_decoder.hpp"
#include "airspy_tv/fec/reed_solomon.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <optional>
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
using airspy_tv::fec::DvbReedSolomon;

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
    DvbReedSolomon codec;
    std::vector<std::uint8_t> output((randomized.size() / ts_packet_size) *
                                     rs_packet_size);
    for (std::size_t input = 0, encoded = 0; input < randomized.size();
         input += ts_packet_size, encoded += rs_packet_size) {
        codec.encode(randomized.subspan(input, ts_packet_size),
                     std::span{output}.subspan(encoded, rs_packet_size));
    }
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

void test_transport_decoder(const CodeRate rate,
                            const std::size_t viterbi_workers = 1) {
    const auto expected = make_transport_stream();
    const auto randomized = energy_scramble(expected);
    const auto rs = rs_encode(randomized);
    const auto interleaved = outer_interleave(rs);
    const auto encoded = convolutional_encode(interleaved);
    const auto metrics = make_metrics(encoded, rate);

    TransportDecoder decoder{rate, viterbi_workers};
    std::vector<std::uint8_t> recovered;
    constexpr std::size_t chunk_size = 997;
    for (std::size_t offset = 0; offset < metrics.size();
         offset += chunk_size) {
        const auto decoded =
            decoder.process(std::span<const float>{metrics}.subspan(
                offset, std::min(chunk_size, metrics.size() - offset)));
        recovered.insert(recovered.end(), decoded.begin(), decoded.end());
    }
    const auto tail = decoder.flush();
    recovered.insert(recovered.end(), tail.begin(), tail.end());

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
    require(stats.pre_viterbi_compared_bits != 0,
            "pre-Viterbi BER sample count");
    require(stats.pre_viterbi_error_bits == 0,
            "clean pre-Viterbi BER estimate");
    require(stats.post_viterbi_compared_bits != 0,
            "post-Viterbi BER sample count");
    require(stats.post_viterbi_error_bits * 100 <
                stats.post_viterbi_compared_bits,
            "clean post-Viterbi BER remains below one percent");
    require(stats.viterbi_workers == viterbi_workers,
            "Viterbi worker statistics");
}

void test_prepared_soft_transport() {
    constexpr CodeRate rate = CodeRate::rate_2_3;
    const auto expected = make_transport_stream();
    const auto randomized = energy_scramble(expected);
    const auto rs = rs_encode(randomized);
    const auto interleaved = outer_interleave(rs);
    const auto encoded = convolutional_encode(interleaved);
    const auto metrics = make_metrics(encoded, rate);
    require(metrics.size() % 3 == 0, "prepared puncture-period alignment");

    std::vector<float> mother(
        airspy_tv::dvbt::depunctured_size(metrics.size(), rate));
    airspy_tv::dvbt::depuncture(metrics, rate, mother);
    std::vector<std::uint8_t> soft(mother.size());
    for (std::size_t index = 0; index < mother.size(); ++index) {
        const float value =
            std::clamp(127.5F + (mother[index] * 8.0F), 0.0F, 255.0F);
        soft[index] = static_cast<std::uint8_t>(value + 0.5F);
    }

    TransportDecoder float_decoder{rate, 4};
    auto float_output = float_decoder.process(metrics);
    const auto float_tail = float_decoder.flush();
    float_output.insert(float_output.end(), float_tail.begin(),
                        float_tail.end());

    TransportDecoder soft_decoder{rate, 4};
    auto soft_output = soft_decoder.process_soft(soft);
    const auto soft_tail = soft_decoder.flush();
    soft_output.insert(soft_output.end(), soft_tail.begin(), soft_tail.end());
    require(soft_output == float_output,
            "prepared soft metrics must match float transport path");
}

void test_uncorrectable_packet_is_emitted_with_tei() {
    constexpr CodeRate rate = CodeRate::rate_2_3;
    const auto expected = make_transport_stream();
    const auto randomized = energy_scramble(expected);
    auto rs = rs_encode(randomized);
    constexpr std::size_t corrupt_packet = 120;
    for (std::size_t byte = 0; byte < 9; ++byte) {
        rs[(corrupt_packet * rs_packet_size) + 20 + byte] ^= 0xFFU;
    }
    const auto interleaved = outer_interleave(rs);
    const auto encoded = convolutional_encode(interleaved);
    const auto metrics = make_metrics(encoded, rate);

    TransportDecoder decoder{rate, 1};
    auto recovered = decoder.process(metrics);
    const auto tail = decoder.flush();
    recovered.insert(recovered.end(), tail.begin(), tail.end());

    const auto stats = decoder.stats();
    require(stats.rs_uncorrectable_packets == 1,
            "one deliberately uncorrectable RS packet");
    require(stats.post_viterbi_compared_bits != 0,
            "outer BER denominator advances on an uncorrectable packet");
    require(stats.post_viterbi_error_bits >= ts_packet_size * 8,
            "outer BER records the uncorrectable payload penalty");
    require(stats.tei_packets == 1,
            "uncorrectable synchronized packet is emitted with TEI");
    require(stats.ts_packets == recovered.size() / ts_packet_size,
            "TEI packet participates in TS packet statistics");

    std::size_t tei_packets = 0;
    std::optional<unsigned int> previous_counter;
    for (std::size_t offset = 0; offset < recovered.size();
         offset += ts_packet_size) {
        require(recovered[offset] == 0x47, "TEI stream remains TS aligned");
        tei_packets += (recovered[offset + 1] & 0x80U) != 0U ? 1U : 0U;
        const unsigned int counter = recovered[offset + 3] & 0x0FU;
        if (previous_counter.has_value()) {
            require(counter == ((*previous_counter + 1U) & 0x0FU),
                    "TEI output preserves continuity-counter cadence");
        }
        previous_counter = counter;
    }
    require(tei_packets == 1, "exactly one output packet has TEI set");
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
    const auto tail = decoder.flush();
    recovered.insert(recovered.end(), tail.begin(), tail.end());

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
        test_transport_decoder(CodeRate::rate_2_3, 4);
        test_prepared_soft_transport();
        test_uncorrectable_packet_is_emitted_with_tei();
        test_equalized_symbol_decoder(
            {TransmissionMode::k2, Constellation::qpsk, CodeRate::rate_1_2, 1});
        test_equalized_symbol_decoder({TransmissionMode::k8,
                                       Constellation::qam64, CodeRate::rate_2_3,
                                       4});
    } catch (const std::exception &error) {
        std::cerr << "DVB-T transport decoder test failed: " << error.what()
                  << '\n';
        return 1;
    }
    std::cout << "DVB-T transport decoder tests passed\n";
    return 0;
}
