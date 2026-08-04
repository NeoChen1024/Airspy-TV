#include "airspy_tv/fec/outer_fec.hpp"

extern "C" {
#include <correct.h>
}

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

namespace airspy_tv::fec {

namespace {

constexpr std::size_t rs_packet_size = 204;
constexpr std::size_t ts_packet_size = 188;
constexpr std::size_t packets_per_energy_frame = 8;
constexpr std::size_t outer_interleaver_branches = 12;
constexpr std::size_t outer_interleaver_step = 17;

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
            throw std::runtime_error("failed to create DVB Reed-Solomon");
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
                              const std::span<std::uint8_t> decoded,
                              std::uint64_t *corrected_payload_bits = nullptr) {
        if (encoded.size() != rs_packet_size ||
            decoded.size() != ts_packet_size) {
            throw std::invalid_argument("DVB RS block size mismatch");
        }
        const bool valid =
            correct_reed_solomon_decode(codec_, encoded.data(), encoded.size(),
                                        decoded.data()) ==
            static_cast<ssize_t>(decoded.size());
        if (valid && corrected_payload_bits != nullptr) {
            *corrected_payload_bits = 0;
            for (std::size_t index = 0; index < decoded.size(); ++index) {
                *corrected_payload_bits += static_cast<std::uint64_t>(
                    std::popcount(static_cast<unsigned int>(encoded[index] ^
                                                            decoded[index])));
            }
        }
        return valid;
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
            throw std::invalid_argument("DVB energy block size mismatch");
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

    // RS(204,188) is systematic, so an uncorrectable codeword still carries
    // the received randomized TS bytes in its first 188 positions. Once the
    // energy-frame phase is known, preserve packet cadence and mark the output
    // as corrupt instead of silently creating a continuity-counter gap.
    [[nodiscard]] bool
    process_corrupt(const std::span<const std::uint8_t> input,
                    const std::span<std::uint8_t> output) {
        if (input.size() != ts_packet_size || output.size() != ts_packet_size) {
            throw std::invalid_argument("DVB energy block size mismatch");
        }
        if (!synchronized_) {
            return false;
        }
        if (packet_index_ == 0) {
            shift_register_ = 0x00A9;
        }
        output.front() = 0x47;
        for (std::size_t index = 1; index < ts_packet_size; ++index) {
            output[index] = input[index] ^ clock_byte();
        }
        static_cast<void>(clock_byte());
        packet_index_ = (packet_index_ + 1) % packets_per_energy_frame;
        output[1] = static_cast<std::uint8_t>(output[1] | 0x80U);
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

struct OuterFec::Impl {
    Impl() { reset(); }

    void reset() {
        selected_outer_phase = outer_interleaver_branches;
        for (std::size_t phase = 0; phase < outer_interleavers.size();
             ++phase) {
            outer_interleavers[phase].reset(phase);
            outer_candidates[phase].clear();
        }
        energy_descrambler.reset();
        statistics = {};
        uncorrectable_since_sync = 0;
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process(const std::span<const std::uint8_t> decoded) {
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
                    // The sync-distance-only heuristic must still show at
                    // least one successful RS codeword. Locking onto a
                    // zero-evidence phase (a random 0x47 alignment through
                    // fade garbage) silently scrambles every subsequent
                    // packet; keep searching until the sliding window picks
                    // up a genuinely decodable run.
                    if (candidate_evidence.rs_successes == 0 ||
                        candidate_evidence.sync_distance > 20) {
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
            std::array<std::uint8_t, ts_packet_size> received_randomized{};
            std::ranges::copy_n(rs_bytes.begin(), ts_packet_size,
                                received_randomized.begin());
            std::array<std::uint8_t, ts_packet_size> randomized{};
            std::uint64_t corrected_payload_bits = 0;
            const bool valid = reed_solomon.decode(
                std::span<const std::uint8_t>{rs_bytes}.first(rs_packet_size),
                randomized, &corrected_payload_bits);
            rs_bytes.erase(rs_bytes.begin(), rs_bytes.begin() + rs_packet_size);
            ++statistics.rs_packets;
            if (!valid) {
                ++statistics.rs_uncorrectable_packets;
                if (++uncorrectable_since_sync >= 100) {
                    // ~7 symbols of consecutive RS failures: the selected
                    // outer phase is almost certainly wrong (a false lock
                    // from a fade-corrupted search window). Drop it and let
                    // the sliding-window search re-select from the data that
                    // has accumulated since; the healthy phase re-locks with
                    // real RS evidence once the garbage has slid out.
                    uncorrectable_since_sync = 0;
                    selected_outer_phase = outer_interleaver_branches;
                    rs_bytes.clear();
                    for (std::size_t phase = 0; phase < outer_candidates.size();
                         ++phase) {
                        outer_candidates[phase].clear();
                    }
                    statistics.rs_synchronized = false;
                    break;
                }
                std::array<std::uint8_t, ts_packet_size> packet{};
                if (energy_descrambler.process_corrupt(received_randomized,
                                                       packet)) {
                    transport_stream.insert(transport_stream.end(),
                                            packet.begin(), packet.end());
                    ++statistics.tei_packets;
                    ++statistics.ts_packets;
                }
                continue;
            }
            uncorrectable_since_sync = 0;
            std::array<std::uint8_t, ts_packet_size> packet{};
            if (energy_descrambler.process(randomized, packet)) {
                statistics.corrected_payload_bits += corrected_payload_bits;
                statistics.compared_payload_bits += ts_packet_size * 8;
                transport_stream.insert(transport_stream.end(), packet.begin(),
                                        packet.end());
                ++statistics.ts_packets;
            }
        }
        statistics.energy_synchronized = energy_descrambler.synchronized();
        return transport_stream;
    }

    std::array<ByteDeinterleaver, outer_interleaver_branches>
        outer_interleavers;
    std::array<std::vector<std::uint8_t>, outer_interleaver_branches>
        outer_candidates;
    std::size_t selected_outer_phase{outer_interleaver_branches};
    ReedSolomon reed_solomon;
    EnergyDescrambler energy_descrambler;
    std::vector<std::uint8_t> rs_bytes;
    OuterFecStats statistics;
    // Consecutive uncorrectable RS packets since the last selection. A false
    // sync lock (a zero-evidence phase selected from fade garbage) would
    // otherwise stay latched forever; after a sustained failure run the
    // selection is dropped and the sliding-window search resumes on the
    // current (healthier) data.
    std::size_t uncorrectable_since_sync{};
};

OuterFec::OuterFec() : impl_(std::make_unique<Impl>()) {}
OuterFec::~OuterFec() noexcept = default;
OuterFec::OuterFec(OuterFec &&) noexcept = default;
OuterFec &OuterFec::operator=(OuterFec &&) noexcept = default;

void OuterFec::reset() { impl_->reset(); }

std::vector<std::uint8_t>
OuterFec::process(const std::span<const std::uint8_t> hard_bytes) {
    return impl_->process(hard_bytes);
}

OuterFecStats OuterFec::stats() const { return impl_->statistics; }

} // namespace airspy_tv::fec
