#include "airspy_tv/fec/reed_solomon.hpp"

#include <libfec_rs.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
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
                            std::uint64_t *const corrected_payload_bits,
                            int *const corrected_symbols,
                            DvbReedSolomonTiming *const timing) const {
    if (codeword.size() != codeword_size || payload.size() != payload_size) {
        throw std::invalid_argument("DVB RS block size mismatch");
    }

    using Clock = std::chrono::steady_clock;
    const auto elapsed_ms = [](const Clock::time_point started_at) {
        return std::chrono::duration<double, std::milli>(Clock::now() -
                                                         started_at)
            .count();
    };
    if (timing != nullptr) {
        *timing = {};
    }
    const auto copy_started_at =
        timing == nullptr ? Clock::time_point{} : Clock::now();
    std::array<std::uint8_t, codeword_size> corrected;
    std::copy(codeword.begin(), codeword.end(), corrected.begin());
    if (timing != nullptr) {
        timing->codeword_copy_ms = elapsed_ms(copy_started_at);
    }

    libfec_rs_decode_profile profile{};
    const int result =
        timing == nullptr
            ? decode_rs_dvb_char(codec_, corrected.data())
            : decode_rs_dvb_char_profiled(codec_, corrected.data(), &profile);
    if (timing != nullptr) {
        constexpr double nanoseconds_per_millisecond = 1'000'000.0;
        timing->syndrome_ms =
            static_cast<double>(profile.syndrome_ns) /
            nanoseconds_per_millisecond;
        timing->error_locator_ms =
            static_cast<double>(profile.error_locator_ns) /
            nanoseconds_per_millisecond;
        timing->correction_ms =
            static_cast<double>(profile.correction_ns) /
            nanoseconds_per_millisecond;
    }
    if (corrected_symbols != nullptr) {
        *corrected_symbols = result;
    }
    if (result < 0) {
        return false;
    }

    const auto payload_copy_started_at =
        timing == nullptr ? Clock::time_point{} : Clock::now();
    std::copy_n(corrected.begin(), payload_size, payload.begin());
    if (timing != nullptr) {
        timing->payload_copy_ms = elapsed_ms(payload_copy_started_at);
    }
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
