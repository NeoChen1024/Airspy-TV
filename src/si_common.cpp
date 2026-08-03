#include "si_common.hpp"

#include <iconv.h>

#include <array>
#include <cstddef>
#include <string>

namespace airspy_tv::si {
namespace {

std::string passthrough_text(const std::span<const std::uint8_t> bytes) {
    std::string text;
    text.reserve(bytes.size());
    for (const std::uint8_t byte : bytes) {
        text.push_back(byte < 0x20U && byte != '\t' ? '?'
                                                    : static_cast<char>(byte));
    }
    return text;
}

// ETSI EN 300 468: ISO/IEC 10646 Basic Multilingual Plane, encoded as
// big-endian 16-bit code units.
std::string utf16_text(const std::span<const std::uint8_t> bytes) {
    std::string text;
    for (std::size_t offset = 0; offset + 1 < bytes.size(); offset += 2) {
        const std::uint32_t codepoint =
            (static_cast<std::uint32_t>(bytes[offset]) << 8U) |
            bytes[offset + 1];
        if (codepoint < 0x80U) {
            text.push_back(static_cast<char>(codepoint));
        } else if (codepoint < 0x800U) {
            text.push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
            text.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        } else {
            text.push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
            text.push_back(
                static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
            text.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        }
    }
    return text;
}

// Converts a legacy DVB encoding to UTF-8 via iconv. Returns an empty string
// when the codeset is unavailable or the input is invalid, letting the caller
// fall back to a lossless passthrough.
std::string iconv_text(const char *codeset,
                       const std::span<const std::uint8_t> bytes) {
    if (bytes.empty()) {
        return {};
    }
    iconv_t cd = iconv_open("UTF-8", codeset);
    if (cd == reinterpret_cast<iconv_t>(-1)) {
        return {};
    }
    std::string output(bytes.size() * 3U + 16U, '\0');
    auto *input = const_cast<char *>(
        reinterpret_cast<const char *>(bytes.data()));
    std::size_t input_left = bytes.size();
    char *cursor = output.data();
    std::size_t output_left = output.size();
    const std::size_t result =
        iconv(cd, &input, &input_left, &cursor, &output_left);
    iconv_close(cd);
    if (result == static_cast<std::size_t>(-1)) {
        return {}; // EILSEQ/E2BIG: fall back to passthrough.
    }
    output.resize(static_cast<std::size_t>(cursor - output.data()));
    return output;
}

} // namespace

std::uint32_t crc32_mpeg(const std::span<const std::uint8_t> bytes) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint32_t>(byte) << 24U;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80000000U) != 0U ? (crc << 1U) ^ 0x04C11DB7U
                                            : crc << 1U;
        }
    }
    return crc;
}

std::string dvb_text(std::span<const std::uint8_t> bytes) {
    if (bytes.empty()) {
        return {};
    }
    // Some Taiwan broadcasts label UTF-16BE data as Big5 (0x14); an
    // impossible Big5 lead byte (Big5 leads are >= 0x81) makes that case safe
    // to recognize without mis-decoding a conforming Big5 string.
    if (bytes.front() == 0x14U && bytes.size() >= 3 && bytes[1] < 0x81U) {
        return utf16_text(bytes.subspan(1));
    }
    const std::uint8_t encoding = bytes.front();
    const auto payload = bytes.subspan(1);
    switch (encoding) {
    case 0x11U: // ISO/IEC 10646, UTF-16BE
        return utf16_text(payload);
    case 0x15U: // UTF-8: preserve bytes verbatim
        return passthrough_text(payload);
    case 0x14U: // Big5 (Traditional Chinese)
        if (const auto text = iconv_text("BIG5", payload); !text.empty()) {
            return text;
        }
        return passthrough_text(payload);
    case 0x13U: // GB-2312 (Simplified Chinese)
        if (const auto text = iconv_text("GB2312", payload); !text.empty()) {
            return text;
        }
        return passthrough_text(payload);
    case 0x12U: // KS X 1001 (Korean)
        if (const auto text = iconv_text("EUC-KR", payload); !text.empty()) {
            return text;
        }
        return passthrough_text(payload);
    default:
        // EN 300 468 Annex A: 0x01..0x0F select ISO/IEC 8859 parts 5..16 then
        // 2..4. Unknown codesets (e.g. nonexistent ISO-8859-12) fall through
        // to the passthrough.
        if (encoding >= 0x01U && encoding <= 0x0FU) {
            static constexpr std::array codesets{
                "ISO-8859-5",  "ISO-8859-6",  "ISO-8859-7",  "ISO-8859-8",
                "ISO-8859-9",  "ISO-8859-10", "ISO-8859-11", "ISO-8859-12",
                "ISO-8859-13", "ISO-8859-14", "ISO-8859-15", "ISO-8859-16",
                "ISO-8859-2",  "ISO-8859-3",  "ISO-8859-4"};
            if (const auto text =
                    iconv_text(codesets[encoding - 1U], payload);
                !text.empty()) {
                return text;
            }
            return passthrough_text(payload);
        }
        // No encoding marker: plain ASCII or raw UTF-8, preserved verbatim.
        return passthrough_text(bytes);
    }
}

} // namespace airspy_tv::si
