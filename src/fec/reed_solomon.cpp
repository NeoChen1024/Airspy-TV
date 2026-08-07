#include "airspy_tv/fec/reed_solomon.hpp"

#include <libfec_rs.h>

#include <algorithm>
#include <array>
#include <bit>
#include <stdexcept>

namespace airspy_tv::fec {

DvbReedSolomon::DvbReedSolomon()
    : codec_(init_rs_char(8, 0x11D, 0, 1, static_cast<int>(parity_size), 51)) {
    if (codec_ == nullptr) {
        throw std::runtime_error("failed to create DVB Reed-Solomon codec");
    }
}

DvbReedSolomon::~DvbReedSolomon() noexcept { free_rs_char(codec_); }

void DvbReedSolomon::encode(const std::span<const std::uint8_t> payload,
                            const std::span<std::uint8_t> codeword) const {
    if (payload.size() != payload_size || codeword.size() != codeword_size) {
        throw std::invalid_argument("DVB RS block size mismatch");
    }
    std::copy(payload.begin(), payload.end(), codeword.begin());
    encode_rs_char(codec_, codeword.data(), codeword.data() + payload_size);
}

bool DvbReedSolomon::decode(const std::span<const std::uint8_t> codeword,
                            const std::span<std::uint8_t> payload,
                            std::uint64_t *const corrected_payload_bits) const {
    if (codeword.size() != codeword_size || payload.size() != payload_size) {
        throw std::invalid_argument("DVB RS block size mismatch");
    }

    std::array<std::uint8_t, codeword_size> corrected{};
    std::copy(codeword.begin(), codeword.end(), corrected.begin());
    if (decode_rs_char(codec_, corrected.data(), nullptr, 0) < 0) {
        return false;
    }

    std::copy_n(corrected.begin(), payload_size, payload.begin());
    if (corrected_payload_bits != nullptr) {
        *corrected_payload_bits = 0;
        for (std::size_t index = 0; index < payload_size; ++index) {
            *corrected_payload_bits += static_cast<std::uint64_t>(std::popcount(
                static_cast<unsigned int>(codeword[index] ^ corrected[index])));
        }
    }
    return true;
}

} // namespace airspy_tv::fec
