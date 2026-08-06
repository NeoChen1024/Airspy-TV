#include "airspy_tv/dvbt/tps_decoder.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <optional>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr std::array<std::uint8_t, 16> sync_even{0, 0, 1, 1, 0, 1, 0, 1,
                                                 1, 1, 1, 0, 1, 1, 1, 0};

unsigned int value(const std::deque<std::uint8_t> &bits,
                   const std::size_t first, const std::size_t count) {
    unsigned int result = 0;
    for (std::size_t index = 0; index < count; ++index) {
        result = (result << 1U) | bits[first + index];
    }
    return result;
}

bool valid_bch(const std::deque<std::uint8_t> &bits) {
    unsigned int reg = 0;
    for (std::size_t input = 0; input < 113; ++input) {
        const unsigned int data = input < 60 ? 0U : bits[1 + input - 60];
        const unsigned int feedback = (data ^ reg) & 1U;
        reg >>= 1U;
        reg |= feedback << 13U;
        reg ^= (feedback << 12U) ^ (feedback << 11U) ^ (feedback << 9U) ^
               (feedback << 8U) ^ (feedback << 7U) ^ (feedback << 5U) ^
               (feedback << 4U);
    }
    for (std::size_t bit = 0; bit < 14; ++bit) {
        if (bits[54 + bit] != ((reg >> bit) & 1U)) {
            return false;
        }
    }
    return true;
}

std::optional<CodeRate> code_rate(const unsigned int code) {
    switch (code) {
    case 0:
        return CodeRate::rate_1_2;
    case 1:
        return CodeRate::rate_2_3;
    case 2:
        return CodeRate::rate_3_4;
    case 3:
        return CodeRate::rate_5_6;
    case 4:
        return CodeRate::rate_7_8;
    default:
        return std::nullopt;
    }
}

std::optional<TpsParameters> decode(const std::deque<std::uint8_t> &bits) {
    if (bits.size() != 68) {
        return std::nullopt;
    }
    bool even = true;
    bool odd = true;
    for (std::size_t index = 0; index < sync_even.size(); ++index) {
        even = even && bits[1 + index] == sync_even[index];
        odd = odd && bits[1 + index] != sync_even[index];
    }
    if ((!even && !odd) || !valid_bch(bits)) {
        return std::nullopt;
    }
    const unsigned int constellation_code = value(bits, 25, 2);
    const unsigned int hierarchy = value(bits, 27, 3);
    const auto hp = code_rate(value(bits, 30, 3));
    const auto lp = code_rate(value(bits, 33, 3));
    const unsigned int guard_code = value(bits, 36, 2);
    const unsigned int mode_code = value(bits, 38, 2);
    if (constellation_code > 2 || hierarchy > 3 || !hp || !lp ||
        guard_code > 3 || mode_code > 1) {
        return std::nullopt;
    }
    const std::array constellations{Constellation::qpsk, Constellation::qam16,
                                    Constellation::qam64};
    const std::array guards{GuardInterval::gi_1_32, GuardInterval::gi_1_16,
                            GuardInterval::gi_1_8, GuardInterval::gi_1_4};
    return TpsParameters{
        .constellation = constellations[constellation_code],
        .high_priority_code_rate = *hp,
        .low_priority_code_rate = *lp,
        .guard_interval = guards[guard_code],
        .mode = mode_code == 0 ? TransmissionMode::k2 : TransmissionMode::k8,
        .hierarchy = static_cast<std::uint8_t>(hierarchy),
        .frame_number = static_cast<std::uint8_t>(value(bits, 23, 2)),
        .cell_id_byte = static_cast<std::uint8_t>(value(bits, 40, 8)),
    };
}

} // namespace

struct TpsDecoder::Impl {
    std::vector<std::complex<float>> previous;
    std::deque<std::uint8_t> bits;
    TpsSnapshot latest;
    bool frame_synchronized{};
};

TpsDecoder::TpsDecoder() : impl_(std::make_unique<Impl>()) {}
TpsDecoder::~TpsDecoder() noexcept = default;

void TpsDecoder::reset() { *impl_ = Impl{}; }

TpsSnapshot
TpsDecoder::process(const std::span<const std::complex<float>> carriers) {
    if (carriers.empty()) {
        return impl_->latest;
    }
    if (impl_->previous.size() == carriers.size()) {
        float vote = 0.0F;
        for (std::size_t index = 0; index < carriers.size(); ++index) {
            const auto difference =
                carriers[index] * std::conj(impl_->previous[index]);
            vote += difference.real() >= 0.0F ? 1.0F : -1.0F;
        }
        impl_->bits.push_back(vote < 0.0F ? 1U : 0U);
        if (impl_->bits.size() > 68) {
            impl_->bits.pop_front();
        }
        if (impl_->frame_synchronized) {
            impl_->latest.symbol_index = (impl_->latest.symbol_index + 1) % 68;
        }
        const bool should_validate =
            !impl_->frame_synchronized || impl_->latest.symbol_index == 67;
        if (should_validate) {
            if (const auto parameters = decode(impl_->bits); parameters) {
                impl_->latest = {.ever_locked = true,
                                 .currently_valid = true,
                                 .symbol_index = 67,
                                 .parameters = *parameters};
                impl_->frame_synchronized = true;
            } else if (impl_->frame_synchronized) {
                // Only an expected frame boundary can invalidate a healthy
                // lock. Resume the sliding search so a phase slip or the next
                // valid frame can re-establish synchronization.
                impl_->latest.currently_valid = false;
                impl_->frame_synchronized = false;
            }
        }
    }
    impl_->previous.assign(carriers.begin(), carriers.end());
    return impl_->latest;
}

TpsSnapshot TpsDecoder::snapshot() const noexcept { return impl_->latest; }

} // namespace airspy_tv::dvbt
