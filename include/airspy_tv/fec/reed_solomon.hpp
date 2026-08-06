#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace airspy_tv::fec {

class DvbReedSolomon {
  public:
    static constexpr std::size_t payload_size = 188;
    static constexpr std::size_t parity_size = 16;
    static constexpr std::size_t codeword_size = payload_size + parity_size;

    DvbReedSolomon();
    ~DvbReedSolomon() noexcept;
    DvbReedSolomon(const DvbReedSolomon &) = delete;
    DvbReedSolomon &operator=(const DvbReedSolomon &) = delete;
    DvbReedSolomon(DvbReedSolomon &&) = delete;
    DvbReedSolomon &operator=(DvbReedSolomon &&) = delete;

    void encode(std::span<const std::uint8_t> payload,
                std::span<std::uint8_t> codeword) const;
    [[nodiscard]] bool
    decode(std::span<const std::uint8_t> codeword,
           std::span<std::uint8_t> payload,
           std::uint64_t *corrected_payload_bits = nullptr) const;

  private:
    void *codec_{};
};

} // namespace airspy_tv::fec
