#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace airspy_tv::fec {

// Zero in user-facing configuration means this portable hardware-concurrency
// default. std::thread reports logical processors and may return zero.
[[nodiscard]] std::size_t default_viterbi_worker_count() noexcept;

// Decoded size of one Viterbi window (7680 bits). The DVB-T transport decoder
// feeds the outer stage window-aligned; callers with other chunk shapes may
// feed arbitrary sizes instead.
inline constexpr std::size_t viterbi_output_bytes = 7680 / 8;

// Soft-input rate-1/2 convolutional (171,133) decoder built on libcorrect with
// an ordered worker pool. Input is mother-code metrics in transmission order;
// punctured streams must be depunctured by the caller before process().
// process() quantizes float LLRs onto the internal 0..255 scale, while
// process_soft() accepts pre-quantized bytes (128 is the neutral erasure
// inserted for punctures). Results are returned in transmission order and
// flush() drains the pool.
class SoftViterbi {
  public:
    explicit SoftViterbi(std::size_t requested_workers);
    ~SoftViterbi() noexcept;
    SoftViterbi(const SoftViterbi &) = delete;
    SoftViterbi &operator=(const SoftViterbi &) = delete;
    SoftViterbi(SoftViterbi &&) noexcept;
    SoftViterbi &operator=(SoftViterbi &&) noexcept;

    void reset();
    [[nodiscard]] std::vector<std::uint8_t>
    process(std::span<const float> llrs);
    [[nodiscard]] std::vector<std::uint8_t>
    process_soft(std::span<const std::uint8_t> soft_metrics);
    [[nodiscard]] std::vector<std::uint8_t> flush();
    [[nodiscard]] std::size_t worker_count() const noexcept;
    [[nodiscard]] std::pair<std::uint64_t, std::uint64_t> error_counts() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::fec
