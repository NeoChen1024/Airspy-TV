#include "airspy_tv/dvbt/transport_decoder.hpp"

extern "C" {
#include <correct.h>
}

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

constexpr std::size_t convolutional_rate = 2;
constexpr std::size_t viterbi_window_bits = 8192;
constexpr std::size_t viterbi_margin_bits = 256;
constexpr std::size_t viterbi_output_bits =
    viterbi_window_bits - (2 * viterbi_margin_bits);
constexpr std::size_t rs_packet_size = 204;
constexpr std::size_t ts_packet_size = 188;
constexpr std::size_t packets_per_energy_frame = 8;
constexpr std::size_t outer_interleaver_branches = 12;
constexpr std::size_t outer_interleaver_step = 17;

class SoftViterbi {
  public:
    SoftViterbi() {
        constexpr std::array<correct_convolutional_polynomial_t, 2> polynomials{
            0117, 0155};
        decoder_ = correct_convolutional_create(convolutional_rate, 7,
                                                polynomials.data());
        if (decoder_ == nullptr) {
            throw std::runtime_error("failed to create libcorrect Viterbi");
        }
    }

    ~SoftViterbi() {
        if (decoder_ != nullptr) {
            correct_convolutional_destroy(decoder_);
        }
    }

    SoftViterbi(const SoftViterbi &) = delete;
    SoftViterbi &operator=(const SoftViterbi &) = delete;

    void reset() { metrics_.clear(); }

    [[nodiscard]] std::vector<std::uint8_t>
    process(const std::span<const float> llrs) {
        metrics_.reserve(metrics_.size() + llrs.size());
        for (const float llr : llrs) {
            const float soft = 127.5F + (llr * 8.0F);
            metrics_.push_back(static_cast<std::uint8_t>(
                std::clamp(std::lround(soft), 0L, 255L)));
        }

        std::vector<std::uint8_t> output;
        constexpr std::size_t window_metrics =
            viterbi_window_bits * convolutional_rate;
        constexpr std::size_t consumed_metrics =
            viterbi_output_bits * convolutional_rate;
        while (metrics_.size() >= window_metrics) {
            std::array<std::uint8_t, viterbi_window_bits / 8> decoded{};
            const ssize_t decoded_bytes = correct_convolutional_decode_soft(
                decoder_, metrics_.data(), window_metrics, decoded.data());
            constexpr std::size_t margin_bytes = viterbi_margin_bits / 8;
            constexpr std::size_t output_bytes = viterbi_output_bits / 8;
            if (decoded_bytes <
                static_cast<ssize_t>(margin_bytes + output_bytes)) {
                throw std::runtime_error("libcorrect Viterbi decode failed");
            }

            output.insert(output.end(), decoded.begin() + margin_bytes,
                          decoded.begin() + margin_bytes + output_bytes);
            metrics_.erase(metrics_.begin(),
                           metrics_.begin() +
                               static_cast<std::ptrdiff_t>(consumed_metrics));
        }
        return output;
    }

  private:
    correct_convolutional *decoder_{};
    std::vector<std::uint8_t> metrics_;
};

class ByteDeinterleaver {
  public:
    ByteDeinterleaver() { reset(0); }

    void reset(const std::size_t initial_branch) {
        for (std::size_t branch = 0; branch < queues_.size(); ++branch) {
            queues_[branch].assign((outer_interleaver_branches - 1 - branch) *
                                       outer_interleaver_step,
                                   0);
        }
        branch_ = initial_branch % outer_interleaver_branches;
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process(const std::span<const std::uint8_t> input) {
        std::vector<std::uint8_t> output;
        output.reserve(input.size());
        for (const std::uint8_t byte : input) {
            auto &queue = queues_[branch_];
            queue.push_back(byte);
            output.push_back(queue.front());
            queue.pop_front();
            branch_ = (branch_ + 1) % outer_interleaver_branches;
        }
        return output;
    }

  private:
    std::array<std::deque<std::uint8_t>, outer_interleaver_branches> queues_;
    std::size_t branch_{};
};

class ReedSolomon {
  public:
    ReedSolomon() {
        codec_ = correct_reed_solomon_create(
            correct_rs_primitive_polynomial_8_4_3_2_0, 0, 1, 16);
        if (codec_ == nullptr) {
            throw std::runtime_error("failed to create DVB-T Reed-Solomon");
        }
    }

    ~ReedSolomon() {
        if (codec_ != nullptr) {
            correct_reed_solomon_destroy(codec_);
        }
    }

    ReedSolomon(const ReedSolomon &) = delete;
    ReedSolomon &operator=(const ReedSolomon &) = delete;

    [[nodiscard]] bool decode(const std::span<const std::uint8_t> encoded,
                              const std::span<std::uint8_t> decoded) {
        if (encoded.size() != rs_packet_size ||
            decoded.size() != ts_packet_size) {
            throw std::invalid_argument("DVB-T RS block size mismatch");
        }
        return correct_reed_solomon_decode(codec_, encoded.data(),
                                           encoded.size(), decoded.data()) ==
               static_cast<ssize_t>(decoded.size());
    }

  private:
    correct_reed_solomon *codec_{};
};

class EnergyDescrambler {
  public:
    void reset() {
        synchronized_ = false;
        packet_index_ = 0;
        shift_register_ = 0x00A9;
    }

    [[nodiscard]] bool synchronized() const noexcept { return synchronized_; }

    void skip_packet() {
        if (!synchronized_) {
            return;
        }
        if (packet_index_ == 0) {
            shift_register_ = 0x00A9;
        }
        for (std::size_t index = 1; index < ts_packet_size; ++index) {
            static_cast<void>(clock_byte());
        }
        static_cast<void>(clock_byte());
        packet_index_ = (packet_index_ + 1) % packets_per_energy_frame;
    }

    [[nodiscard]] bool process(const std::span<const std::uint8_t> input,
                               const std::span<std::uint8_t> output) {
        if (input.size() != ts_packet_size || output.size() != ts_packet_size) {
            throw std::invalid_argument("DVB-T energy block size mismatch");
        }
        if (!synchronized_) {
            if (input.front() != 0xB8) {
                return false;
            }
            synchronized_ = true;
            packet_index_ = 0;
        }

        const std::uint8_t expected_sync = packet_index_ == 0 ? 0xB8 : 0x47;
        if (input.front() != expected_sync) {
            synchronized_ = false;
            return false;
        }
        if (packet_index_ == 0) {
            shift_register_ = 0x00A9;
        }

        output.front() = 0x47;
        for (std::size_t index = 1; index < ts_packet_size; ++index) {
            output[index] = input[index] ^ clock_byte();
        }
        // The PRBS advances across the seven ordinary sync bytes even though
        // those bytes themselves are not randomized.
        static_cast<void>(clock_byte());
        packet_index_ = (packet_index_ + 1) % packets_per_energy_frame;
        return true;
    }

  private:
    [[nodiscard]] std::uint8_t clock_byte() {
        std::uint8_t result = 0;
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint16_t feedback =
                ((shift_register_ >> 13) ^ (shift_register_ >> 14)) & 1U;
            shift_register_ = static_cast<std::uint16_t>(
                ((shift_register_ << 1) | feedback) & 0x7FFFU);
            result = static_cast<std::uint8_t>((result << 1) | feedback);
        }
        return result;
    }

    bool synchronized_{};
    std::size_t packet_index_{};
    std::uint16_t shift_register_{0x00A9};
};

struct AlignmentEvidence {
    std::size_t start{std::numeric_limits<std::size_t>::max()};
    unsigned int sync_distance{std::numeric_limits<unsigned int>::max()};
    std::size_t rs_successes{};
};

[[nodiscard]] AlignmentEvidence
find_rs_alignment(const std::span<const std::uint8_t> bytes,
                  ReedSolomon &reed_solomon) {
    constexpr std::size_t required_packets = 16;
    constexpr std::size_t required_bytes = required_packets * rs_packet_size;
    if (bytes.size() < required_bytes) {
        return {};
    }
    AlignmentEvidence best;
    for (std::size_t start = 0; start + required_bytes <= bytes.size();
         ++start) {
        for (std::size_t energy_phase = 0;
             energy_phase < packets_per_energy_frame; ++energy_phase) {
            unsigned int distance = 0;
            for (std::size_t packet = 0; packet < required_packets; ++packet) {
                const std::uint8_t expected =
                    (packet + energy_phase) % packets_per_energy_frame == 0
                        ? 0xB8
                        : 0x47;
                distance += static_cast<unsigned int>(
                    std::popcount(static_cast<unsigned int>(
                        bytes[start + (packet * rs_packet_size)] ^ expected)));
            }
            if (best.rs_successes == 0 && distance < best.sync_distance) {
                best = {start, distance, 0};
            }
            if (distance > 24) {
                continue;
            }
            std::size_t rs_successes = 0;
            for (std::size_t packet = 0; packet < required_packets; ++packet) {
                const std::uint8_t expected =
                    (packet + energy_phase) % packets_per_energy_frame == 0
                        ? 0xB8
                        : 0x47;
                std::array<std::uint8_t, ts_packet_size> decoded{};
                rs_successes +=
                    reed_solomon.decode(
                        bytes.subspan(start + (packet * rs_packet_size),
                                      rs_packet_size),
                        decoded) &&
                    decoded.front() == expected;
            }
            if (rs_successes > best.rs_successes ||
                (rs_successes == best.rs_successes &&
                 distance < best.sync_distance)) {
                best = {start, distance, rs_successes};
            }
        }
    }
    return best;
}

} // namespace

struct TransportDecoder::Impl {
    explicit Impl(const CodeRate selected_code_rate)
        : code_rate(selected_code_rate) {
        reset();
    }

    void reset() {
        punctured_metrics.clear();
        rs_bytes.clear();
        viterbi.reset();
        selected_outer_phase = outer_interleaver_branches;
        for (std::size_t phase = 0; phase < outer_interleavers.size();
             ++phase) {
            outer_interleavers[phase].reset(phase);
            outer_candidates[phase].clear();
        }
        energy_descrambler.reset();
        statistics = {};
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process(const std::span<const float> input) {
        punctured_metrics.insert(punctured_metrics.end(), input.begin(),
                                 input.end());
        const std::size_t period = [&] {
            switch (code_rate) {
            case CodeRate::rate_1_2:
                return std::size_t{2};
            case CodeRate::rate_2_3:
                return std::size_t{3};
            case CodeRate::rate_3_4:
                return std::size_t{4};
            case CodeRate::rate_5_6:
                return std::size_t{6};
            case CodeRate::rate_7_8:
                return std::size_t{8};
            }
            return std::size_t{0};
        }();
        const std::size_t aligned =
            punctured_metrics.size() - (punctured_metrics.size() % period);
        if (aligned == 0) {
            return {};
        }

        std::vector<float> mother(depunctured_size(aligned, code_rate));
        depuncture(std::span<const float>{punctured_metrics}.first(aligned),
                   code_rate, mother);
        punctured_metrics.erase(punctured_metrics.begin(),
                                punctured_metrics.begin() +
                                    static_cast<std::ptrdiff_t>(aligned));

        auto decoded = viterbi.process(mother);
        statistics.viterbi_bits += decoded.size() * 8;
        constexpr std::size_t maximum_search = 32 * rs_packet_size;
        if (selected_outer_phase == outer_interleaver_branches) {
            std::array<AlignmentEvidence, outer_interleaver_branches>
                evidence{};
            for (std::size_t phase = 0; phase < outer_interleavers.size();
                 ++phase) {
                auto deinterleaved = outer_interleavers[phase].process(decoded);
                auto &candidate = outer_candidates[phase];
                candidate.insert(candidate.end(), deinterleaved.begin(),
                                 deinterleaved.end());
                if (candidate.size() > maximum_search) {
                    candidate.erase(
                        candidate.begin(),
                        candidate.end() -
                            static_cast<std::ptrdiff_t>(maximum_search));
                }
                evidence[phase] = find_rs_alignment(candidate, reed_solomon);
            }
            std::size_t selected_phase = outer_interleaver_branches;
            AlignmentEvidence selected_evidence;
            unsigned int global_sync_distance =
                std::numeric_limits<unsigned int>::max();
            std::size_t global_rs_evidence = 0;
            for (std::size_t phase = 0; phase < evidence.size(); ++phase) {
                const auto &candidate_evidence = evidence[phase];
                global_sync_distance = std::min(
                    global_sync_distance, candidate_evidence.sync_distance);
                global_rs_evidence = std::max(global_rs_evidence,
                                              candidate_evidence.rs_successes);
                if (candidate_evidence.rs_successes < 4) {
                    continue;
                }
                if (selected_phase == outer_interleaver_branches ||
                    candidate_evidence.rs_successes >
                        selected_evidence.rs_successes ||
                    (candidate_evidence.rs_successes ==
                         selected_evidence.rs_successes &&
                     candidate_evidence.sync_distance <
                         selected_evidence.sync_distance)) {
                    selected_phase = phase;
                    selected_evidence = candidate_evidence;
                }
            }
            statistics.outer_sync_distance =
                global_sync_distance == std::numeric_limits<unsigned int>::max()
                    ? 0U
                    : global_sync_distance;
            statistics.outer_rs_evidence =
                static_cast<std::uint32_t>(global_rs_evidence);
            const bool candidates_full = std::ranges::all_of(
                outer_candidates, [](const auto &candidate) {
                    return candidate.size() >= maximum_search;
                });
            if (selected_phase == outer_interleaver_branches &&
                candidates_full) {
                for (std::size_t phase = 0; phase < evidence.size(); ++phase) {
                    const auto &candidate_evidence = evidence[phase];
                    if (candidate_evidence.sync_distance > 20) {
                        continue;
                    }
                    if (selected_phase == outer_interleaver_branches ||
                        candidate_evidence.sync_distance <
                            selected_evidence.sync_distance) {
                        selected_phase = phase;
                        selected_evidence = candidate_evidence;
                    }
                }
            }
            if (selected_phase != outer_interleaver_branches) {
                selected_outer_phase = selected_phase;
                statistics.outer_deinterleaver_phase =
                    static_cast<int>(selected_phase);
                const auto &candidate = outer_candidates[selected_phase];
                rs_bytes.assign(
                    candidate.begin() +
                        static_cast<std::ptrdiff_t>(selected_evidence.start),
                    candidate.end());
                statistics.rs_synchronized = true;
            }
            if (!statistics.rs_synchronized) {
                return {};
            }
        } else {
            auto deinterleaved =
                outer_interleavers[selected_outer_phase].process(decoded);
            rs_bytes.insert(rs_bytes.end(), deinterleaved.begin(),
                            deinterleaved.end());
        }

        std::vector<std::uint8_t> transport_stream;
        while (rs_bytes.size() >= rs_packet_size) {
            std::array<std::uint8_t, ts_packet_size> randomized{};
            const bool valid = reed_solomon.decode(
                std::span<const std::uint8_t>{rs_bytes}.first(rs_packet_size),
                randomized);
            rs_bytes.erase(rs_bytes.begin(), rs_bytes.begin() + rs_packet_size);
            ++statistics.rs_packets;
            if (!valid) {
                ++statistics.rs_uncorrectable_packets;
                energy_descrambler.skip_packet();
                continue;
            }

            std::array<std::uint8_t, ts_packet_size> packet{};
            if (energy_descrambler.process(randomized, packet)) {
                transport_stream.insert(transport_stream.end(), packet.begin(),
                                        packet.end());
                ++statistics.ts_packets;
            }
        }
        statistics.energy_synchronized = energy_descrambler.synchronized();
        return transport_stream;
    }

    CodeRate code_rate;
    SoftViterbi viterbi;
    std::array<ByteDeinterleaver, outer_interleaver_branches>
        outer_interleavers;
    std::array<std::vector<std::uint8_t>, outer_interleaver_branches>
        outer_candidates;
    std::size_t selected_outer_phase{outer_interleaver_branches};
    ReedSolomon reed_solomon;
    EnergyDescrambler energy_descrambler;
    std::vector<float> punctured_metrics;
    std::vector<std::uint8_t> rs_bytes;
    TransportDecoderStats statistics;
};

TransportDecoder::TransportDecoder(const CodeRate code_rate)
    : impl_(std::make_unique<Impl>(code_rate)) {}

TransportDecoder::~TransportDecoder() noexcept = default;
TransportDecoder::TransportDecoder(TransportDecoder &&) noexcept = default;
TransportDecoder &
TransportDecoder::operator=(TransportDecoder &&) noexcept = default;

void TransportDecoder::reset() { impl_->reset(); }

std::vector<std::uint8_t>
TransportDecoder::process(const std::span<const float> punctured_llrs) {
    return impl_->process(punctured_llrs);
}

TransportDecoderStats TransportDecoder::stats() const {
    return impl_->statistics;
}

} // namespace airspy_tv::dvbt
