#include "airspy_tv/dvbt/transport_decoder.hpp"

extern "C" {
#include <correct.h>
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string_view>
#include <vector>

namespace {

int decode_single_viterbi_block(const std::filesystem::path &source,
                                const std::filesystem::path &destination,
                                const std::string_view polynomial_mode) {
    std::ifstream input(source, std::ios::binary);
    std::ofstream output(destination, std::ios::binary | std::ios::trunc);
    std::vector<std::uint8_t> metrics{std::istreambuf_iterator<char>{input},
                                      std::istreambuf_iterator<char>{}};
    std::array<correct_convolutional_polynomial_t, 2> polynomials{0171, 0133};
    if (polynomial_mode == "--viterbi-spec-swap") {
        polynomials = {0133, 0171};
    } else if (polynomial_mode == "--viterbi-gr") {
        polynomials = {0117, 0155};
    } else if (polynomial_mode == "--viterbi-gr-swap") {
        polynomials = {0155, 0117};
    }
    correct_convolutional *decoder =
        correct_convolutional_create(2, 7, polynomials.data());
    if (!input || !output || decoder == nullptr || metrics.size() % 2 != 0) {
        return 1;
    }
    std::vector<std::uint8_t> decoded((metrics.size() / 2 + 7) / 8);
    const ssize_t count = correct_convolutional_decode_soft(
        decoder, metrics.data(), metrics.size(), decoded.data());
    correct_convolutional_destroy(decoder);
    if (count <= 0) {
        return 2;
    }
    output.write(reinterpret_cast<const char *>(decoded.data()), count);
    std::cerr << "single-block Viterbi output=" << count << " bytes\n";
    return 0;
}

} // namespace

int main(const int argc, const char *const argv[]) {
    if (argc == 4 && std::string_view{argv[3]}.starts_with("--viterbi-")) {
        return decode_single_viterbi_block(argv[1], argv[2], argv[3]);
    }
    if (argc != 3) {
        std::cerr << "usage: airspy-tv-dvbt-soft INPUT.u8 OUTPUT.ts "
                     "[--viterbi-spec|--viterbi-spec-swap|--viterbi-gr|"
                     "--viterbi-gr-swap]\n";
        return 1;
    }
    std::ifstream input(std::filesystem::path{argv[1]}, std::ios::binary);
    std::ofstream output(std::filesystem::path{argv[2]},
                         std::ios::binary | std::ios::trunc);
    if (!input || !output) {
        return 1;
    }

    airspy_tv::dvbt::TransportDecoder decoder{
        airspy_tv::dvbt::CodeRate::rate_2_3};
    constexpr std::array<int, 4> puncture{1, 1, 0, 1};
    std::vector<std::uint8_t> bytes(64 * 1024);
    std::vector<float> punctured;
    std::size_t mother_index = 0;
    std::uint64_t written = 0;
    while (input) {
        input.read(reinterpret_cast<char *>(bytes.data()),
                   static_cast<std::streamsize>(bytes.size()));
        const std::size_t count = static_cast<std::size_t>(input.gcount());
        punctured.clear();
        for (std::size_t index = 0; index < count; ++index, ++mother_index) {
            if (puncture[mother_index % puncture.size()] == 0) {
                continue;
            }
            punctured.push_back((static_cast<float>(bytes[index]) - 127.5F) /
                                8.0F);
        }
        const auto transport_stream = decoder.process(punctured);
        output.write(reinterpret_cast<const char *>(transport_stream.data()),
                     static_cast<std::streamsize>(transport_stream.size()));
        written += transport_stream.size();
    }
    const auto stats = decoder.stats();
    std::cerr << "output=" << written << ", RS=" << stats.rs_packets
              << ", failures=" << stats.rs_uncorrectable_packets
              << ", TS=" << stats.ts_packets << '\n';
    return written == 0 ? 2 : 0;
}
