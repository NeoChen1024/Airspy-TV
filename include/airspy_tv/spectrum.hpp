#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace airspy_tv {

inline constexpr std::size_t spectrum_fft_size = 4096;

struct SpectrumSnapshot {
    std::array<float, spectrum_fft_size> bins_dbfs{};
    std::array<float, spectrum_fft_size> waterfall_bins_dbfs{};
    float signal_power_dbfs{-140.0F};
    float rf_snr_db{};
    float deepest_notch_db{};
    std::uint32_t sample_rate_hz{};
    std::uint64_t sequence{};
    bool channel_metrics_valid{};
    bool valid{};
};

class SpectrumAnalyzer {
  public:
    SpectrumAnalyzer();
    ~SpectrumAnalyzer() noexcept;

    SpectrumAnalyzer(const SpectrumAnalyzer &) = delete;
    SpectrumAnalyzer &operator=(const SpectrumAnalyzer &) = delete;
    SpectrumAnalyzer(SpectrumAnalyzer &&) = delete;
    SpectrumAnalyzer &operator=(SpectrumAnalyzer &&) = delete;

    void submit(std::span<const std::int16_t> interleaved_iq,
                std::uint32_t sample_rate_hz);
    void reset();
    void set_smoothing(bool fft_enabled, int fft_speed, bool snr_enabled,
                       int snr_speed);

    [[nodiscard]] SpectrumSnapshot snapshot() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
