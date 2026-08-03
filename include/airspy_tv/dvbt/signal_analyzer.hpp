#pragma once

#include "airspy_tv/dvbt/receiver_parameters.hpp"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace airspy_tv::dvbt {

inline constexpr std::size_t constellation_snapshot_size = 2048;

struct SignalAnalysisSnapshot {
    std::array<std::complex<float>, constellation_snapshot_size> points{};
    std::size_t point_count{};
    float mer_db{};
    float cp_snr_db{};
    float deepest_notch_db{};
    float carrier_offset_hz{};
    TransmissionMode mode{TransmissionMode::k8};
    GuardInterval guard_interval{GuardInterval::gi_1_4};
    Constellation constellation{Constellation::qam64};
    std::uint64_t sequence{};
    bool locked{};
};

// Low-rate one-symbol monitoring path for the GUI. It shares CS16 resampling
// and CP acquisition with StreamDecoder, while retaining its own worker so UI
// updates never wait for the complete FEC pipeline.
class SignalAnalyzer {
  public:
    SignalAnalyzer();
    ~SignalAnalyzer() noexcept;

    SignalAnalyzer(const SignalAnalyzer &) = delete;
    SignalAnalyzer &operator=(const SignalAnalyzer &) = delete;
    SignalAnalyzer(SignalAnalyzer &&) = delete;
    SignalAnalyzer &operator=(SignalAnalyzer &&) = delete;

    void submit(std::span<const std::int16_t> interleaved_iq,
                std::uint32_t sample_rate_hz,
                std::uint32_t channel_bandwidth_hz = 6'000'000);
    void reset();
    void set_parameters(const ReceiverParameters &parameters);
    void set_snr_smoothing(bool enabled, int speed);
    [[nodiscard]] SignalAnalysisSnapshot snapshot() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
