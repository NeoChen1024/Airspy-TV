#include "airspy_tv/fec/reed_solomon.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using airspy_tv::fec::DvbReedSolomon;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

[[nodiscard]] std::array<std::uint8_t, DvbReedSolomon::payload_size>
make_payload() {
    std::array<std::uint8_t, DvbReedSolomon::payload_size> payload{};
    for (std::size_t index = 0; index < payload.size(); ++index) {
        payload[index] =
            static_cast<std::uint8_t>(((index * 73U) + 19U) & 0xFFU);
    }
    return payload;
}

void test_correction_limit() {
    DvbReedSolomon codec;
    const auto payload = make_payload();
    std::array<std::uint8_t, DvbReedSolomon::codeword_size> encoded{};
    codec.encode(payload, encoded);
    require(std::equal(payload.begin(), payload.end(), encoded.begin()),
            "RS encoder is not systematic");

    for (std::size_t error_count = 0; error_count <= 8; ++error_count) {
        auto damaged = encoded;
        std::uint64_t expected_payload_bits = 0;
        for (std::size_t error = 0; error < error_count; ++error) {
            const std::size_t position =
                ((error * 23U) + (error_count * 7U)) % damaged.size();
            const auto mask = static_cast<std::uint8_t>(0x41U + error);
            damaged[position] ^= mask;
            if (position < payload.size()) {
                expected_payload_bits += static_cast<std::uint64_t>(
                    std::popcount(static_cast<unsigned int>(mask)));
            }
        }

        std::array<std::uint8_t, DvbReedSolomon::payload_size> decoded{};
        std::uint64_t corrected_payload_bits = 0;
        require(codec.decode(damaged, decoded, &corrected_payload_bits),
                "RS decoder rejected a correctable codeword");
        require(decoded == payload, "RS decoder produced incorrect payload");
        require(corrected_payload_bits == expected_payload_bits,
                "RS corrected payload bit count mismatch");
    }
}

void test_uncorrectable_codeword() {
    DvbReedSolomon codec;
    const auto payload = make_payload();
    std::array<std::uint8_t, DvbReedSolomon::codeword_size> damaged{};
    codec.encode(payload, damaged);
    for (std::size_t error = 0; error < 9; ++error) {
        damaged[error * 23U] ^= static_cast<std::uint8_t>(0x81U + error);
    }

    std::array<std::uint8_t, DvbReedSolomon::payload_size> decoded{};
    require(!codec.decode(damaged, decoded),
            "RS decoder accepted a nine-symbol error pattern");
}

} // namespace

int main() {
    try {
        test_correction_limit();
        test_uncorrectable_codeword();
        std::cout << "Reed-Solomon tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Reed-Solomon test failure: " << error.what() << '\n';
        return 1;
    }
}
