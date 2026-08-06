#pragma once

#include "airspy_tv/dvbt/receiver_parameters.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace airspy_tv::dvbt {

struct TpsParameters {
    Constellation constellation{Constellation::qpsk};
    CodeRate high_priority_code_rate{CodeRate::rate_1_2};
    CodeRate low_priority_code_rate{CodeRate::rate_1_2};
    GuardInterval guard_interval{GuardInterval::gi_1_32};
    TransmissionMode mode{TransmissionMode::k2};
    std::uint8_t hierarchy{};
    std::uint8_t frame_number{};
    std::uint8_t cell_id_byte{};
};

struct TpsSnapshot {
    // Parameters come from the most recent valid TPS frame and survive later
    // decode failures. `currently_valid` is held between expected frame
    // boundaries and reflects whether the latest complete frame passed its
    // BCH/sync check.
    bool ever_locked{};
    bool currently_valid{};
    std::size_t symbol_index{};
    TpsParameters parameters{};
};

class TpsDecoder {
  public:
    TpsDecoder();
    ~TpsDecoder() noexcept;
    TpsDecoder(const TpsDecoder &) = delete;
    TpsDecoder &operator=(const TpsDecoder &) = delete;
    TpsDecoder(TpsDecoder &&) = delete;
    TpsDecoder &operator=(TpsDecoder &&) = delete;

    void reset();
    [[nodiscard]] TpsSnapshot
    process(std::span<const std::complex<float>> carriers);
    [[nodiscard]] TpsSnapshot snapshot() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv::dvbt
