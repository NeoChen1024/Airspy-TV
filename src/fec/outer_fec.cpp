#include "airspy_tv/fec/outer_fec.hpp"
#include "airspy_tv/fec/reed_solomon.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace airspy_tv::fec {

namespace {

constexpr std::size_t rs_packet_size = 204;
constexpr std::size_t ts_packet_size = 188;
constexpr std::size_t packets_per_energy_frame = 8;
constexpr std::size_t outer_interleaver_branches = 12;
constexpr std::size_t outer_interleaver_step = 17;
constexpr std::size_t minimum_alignment_rs_evidence = 4;
// A marginal but correctly aligned stream can produce a short burst of
// uncorrectable RS blocks. Keep the known phase long enough for
// EnergyDescrambler::process_corrupt() to preserve TS cadence; only a much
// longer run is treated as evidence of a false outer-phase lock.
constexpr std::size_t uncorrectable_reset_threshold = 512;

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

class EnergyDescrambler {
  public:
    void reset() {
        synchronized_ = false;
        packet_index_ = 0;
        shift_register_ = 0x00A9;
    }

    [[nodiscard]] bool synchronized() const noexcept { return synchronized_; }

    void start_at_energy_phase(const std::size_t phase) {
        reset();
        synchronized_ = true;
        for (std::size_t packet = 0; packet < phase; ++packet) {
            skip_packet();
        }
    }

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
    std::size_t energy_phase{};
};

[[nodiscard]] AlignmentEvidence
find_rs_alignment(const std::span<const std::uint8_t> bytes,
                  DvbReedSolomon &reed_solomon) {
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
                best = {start, distance, 0, energy_phase};
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
                best = {start, distance, rs_successes, energy_phase};
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
        alignment_search_bytes = 0;
        alignment_search_count = 0;
        alignment_search_done = false;
        pending_alignment_phase = outer_interleaver_branches;
    }

    [[nodiscard]] bool diagnostics_enabled() const {
        return diagnostic_handler.is_enabled();
    }

    void emit_diagnostic(std::string name,
                         const DiagnosticEventSeverity severity,
                         DiagnosticEventFields fields) {
        diagnostic_handler.emit({.name = std::move(name),
                                 .severity = severity,
                                 .fields = std::move(fields)});
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process(const std::span<const std::uint8_t> decoded) {
        constexpr std::size_t maximum_search = 32 * rs_packet_size;
        if (selected_outer_phase == outer_interleaver_branches) {
            // Searching all 12 deinterleaver branches is deliberately kept
            // out of the per-symbol hot path.  A marginal channel can lose
            // an already selected RS phase for a short burst; repeatedly
            // scanning the same 6.5 KiB window in that state makes the FEC
            // worker spend seconds in Reed-Solomon checks and back-pressure
            // the demod queue.  The window itself is retained, so a real
            // alignment remains available for the next search.
            alignment_search_bytes += decoded.size();
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
            }
            const bool candidates_full = std::ranges::all_of(
                outer_candidates, [](const auto &candidate) {
                    return candidate.size() >= maximum_search;
                });
            // Once a candidate is pending confirmation, do the second check
            // on the next decoded chunk. Keep the longer cadence only while
            // no phase has presented usable RS evidence; waiting a full
            // window here would slide the candidate past the first packets
            // of a short finite stream before the lock is confirmed.
            const std::size_t next_search_interval =
                pending_alignment_phase != outer_interleaver_branches
                    ? 1U
                    : alignment_search_interval;
            const bool search_requested =
                candidates_full &&
                (!alignment_search_done ||
                 alignment_search_bytes >= next_search_interval);
            if (!search_requested) {
                return {};
            }
            alignment_search_done = true;
            alignment_search_bytes = 0;
            ++alignment_search_count;
            std::array<AlignmentEvidence, outer_interleaver_branches>
                evidence{};
            for (std::size_t phase = 0; phase < outer_candidates.size();
                 ++phase) {
                evidence[phase] =
                    find_rs_alignment(outer_candidates[phase], reed_solomon);
            }
            if (diagnostics_enabled()) {
                DiagnosticEventFields fields{
                    {"search_count", alignment_search_count}};
                for (std::size_t phase = 0; phase < outer_candidates.size();
                     ++phase) {
                    const auto &candidate_evidence = evidence[phase];
                    const std::string prefix =
                        "phase_" + std::to_string(phase) + '_';
                    const bool available =
                        candidate_evidence.start !=
                        std::numeric_limits<std::size_t>::max();
                    fields.emplace(prefix + "available", available);
                    fields.emplace(prefix + "sync_distance",
                                   static_cast<std::uint64_t>(
                                       candidate_evidence.sync_distance));
                    fields.emplace(prefix + "rs_successes",
                                   static_cast<std::uint64_t>(
                                       candidate_evidence.rs_successes));
                    fields.emplace(prefix + "start",
                                   static_cast<std::uint64_t>(
                                       available ? candidate_evidence.start
                                                 : 0));
                    fields.emplace(prefix + "energy_phase",
                                   static_cast<std::uint64_t>(
                                       candidate_evidence.energy_phase));
                }
                emit_diagnostic("outer_fec_alignment_search",
                                DiagnosticEventSeverity::info,
                                std::move(fields));
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
                if (candidate_evidence.rs_successes <
                    minimum_alignment_rs_evidence) {
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
            if (selected_phase != outer_interleaver_branches) {
                if (pending_alignment_phase != selected_phase) {
                    pending_alignment_phase = selected_phase;
                    statistics.rs_synchronized = false;
                    if (diagnostics_enabled()) {
                        emit_diagnostic(
                            "outer_fec_alignment_pending",
                            DiagnosticEventSeverity::info,
                            {{"phase", static_cast<std::uint64_t>(selected_phase)},
                             {"sync_distance",
                              static_cast<std::uint64_t>(
                                  selected_evidence.sync_distance)},
                             {"rs_successes",
                              static_cast<std::uint64_t>(
                                  selected_evidence.rs_successes)}});
                    }
                    return {};
                }
                pending_alignment_phase = outer_interleaver_branches;
                selected_outer_phase = selected_phase;
                statistics.outer_deinterleaver_phase =
                    static_cast<int>(selected_phase);
                const auto &candidate = outer_candidates[selected_phase];
                rs_bytes.assign(
                    candidate.begin() +
                        static_cast<std::ptrdiff_t>(selected_evidence.start),
                    candidate.end());
                energy_descrambler.start_at_energy_phase(
                    selected_evidence.energy_phase);
                statistics.rs_synchronized = true;
                if (diagnostics_enabled()) {
                    emit_diagnostic(
                        "outer_fec_alignment_locked",
                        DiagnosticEventSeverity::info,
                        {{"phase", static_cast<std::uint64_t>(selected_phase)},
                         {"start", static_cast<std::uint64_t>(
                                       selected_evidence.start)},
                         {"sync_distance",
                          static_cast<std::uint64_t>(
                              selected_evidence.sync_distance)},
                         {"rs_successes",
                          static_cast<std::uint64_t>(
                              selected_evidence.rs_successes)},
                         {"energy_phase",
                          static_cast<std::uint64_t>(
                              selected_evidence.energy_phase)},
                         {"candidate_bytes",
                          static_cast<std::uint64_t>(
                              outer_candidates[selected_phase].size())},
                         {"confirmation_count", std::uint64_t{2}}});
                }
            } else {
                pending_alignment_phase = outer_interleaver_branches;
                if (diagnostics_enabled()) {
                    emit_diagnostic(
                        "outer_fec_alignment_missed",
                        DiagnosticEventSeverity::info,
                        {{"search_count", alignment_search_count},
                         {"best_sync_distance",
                          static_cast<std::uint64_t>(
                              statistics.outer_sync_distance)},
                         {"best_rs_evidence",
                          static_cast<std::uint64_t>(
                              statistics.outer_rs_evidence)}});
                }
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
            if (statistics.rs_packets <= 4 && diagnostics_enabled()) {
                emit_diagnostic(
                    "outer_fec_rs_attempt", DiagnosticEventSeverity::info,
                    {{"packet", statistics.rs_packets},
                     {"valid", valid},
                     {"received_sync_byte", static_cast<std::uint64_t>(
                                                received_randomized.front())},
                     {"buffered_bytes",
                      static_cast<std::uint64_t>(rs_bytes.size())},
                     {"energy_synchronized",
                      energy_descrambler.synchronized()}});
            }
            // Count every codeword once it has been selected by a valid outer
            // phase. The old accounting only advanced after a successful RS
            // decode *and* energy descramble, which made the GUI Outer BER
            // freeze exactly when a false lock began producing only bad
            // codewords. An uncorrectable shortened RS block has no reliable
            // payload estimate; charging all 188 payload bytes is a
            // deliberate conservative indicator of outer-lock failure.
            statistics.compared_payload_bits += ts_packet_size * 8;
            if (!valid) {
                ++statistics.rs_uncorrectable_packets;
                statistics.corrected_payload_bits += ts_packet_size * 8;
                ++uncorrectable_since_sync;
                const bool failure_milestone =
                    uncorrectable_since_sync == 1 ||
                    uncorrectable_since_sync == 8 ||
                    uncorrectable_since_sync == 32 ||
                    uncorrectable_since_sync == 128 ||
                    uncorrectable_since_sync == 256 ||
                    uncorrectable_since_sync == uncorrectable_reset_threshold;
                if (failure_milestone && diagnostics_enabled()) {
                    emit_diagnostic(
                        "outer_fec_rs_failure_streak",
                        DiagnosticEventSeverity::warning,
                        {{"streak", static_cast<std::uint64_t>(
                                        uncorrectable_since_sync)},
                         {"uncorrectable_packets",
                          statistics.rs_uncorrectable_packets},
                         {"rs_packets", statistics.rs_packets},
                         {"phase", static_cast<std::uint64_t>(
                                       selected_outer_phase)},
                         {"buffered_bytes",
                          static_cast<std::uint64_t>(rs_bytes.size())},
                         {"energy_synchronized",
                          energy_descrambler.synchronized()}});
                }
                if (uncorrectable_since_sync >= uncorrectable_reset_threshold) {
                    // A long run of consecutive RS failures means the
                    // selected outer phase is almost certainly wrong (a
                    // false lock from a fade-corrupted search window). Drop
                    // it and let the sliding-window search re-select from the
                    // data that has accumulated since; the healthy phase
                    // re-locks with real RS evidence once the garbage has
                    // slid out.
                    uncorrectable_since_sync = 0;
                    selected_outer_phase = outer_interleaver_branches;
                    rs_bytes.clear();
                    for (std::size_t phase = 0; phase < outer_candidates.size();
                         ++phase) {
                        outer_candidates[phase].clear();
                    }
                    energy_descrambler.reset();
                    alignment_search_bytes = 0;
                    alignment_search_done = false;
                    pending_alignment_phase = outer_interleaver_branches;
                    statistics.rs_synchronized = false;
                    statistics.energy_synchronized = false;
                    statistics.outer_deinterleaver_phase = -1;
                    statistics.outer_sync_distance = 0;
                    statistics.outer_rs_evidence = 0;
                    if (diagnostics_enabled()) {
                        emit_diagnostic(
                            "outer_fec_reset",
                            DiagnosticEventSeverity::warning,
                            {{"reason", std::string{"rs_failure_streak"}},
                             {"threshold", static_cast<std::uint64_t>(
                                               uncorrectable_reset_threshold)},
                             {"uncorrectable_packets",
                              statistics.rs_uncorrectable_packets}});
                    }
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
            if (uncorrectable_since_sync >= 8 && diagnostics_enabled()) {
                emit_diagnostic(
                    "outer_fec_rs_recovered", DiagnosticEventSeverity::info,
                    {{"previous_streak", static_cast<std::uint64_t>(
                                             uncorrectable_since_sync)},
                     {"uncorrectable_packets",
                      statistics.rs_uncorrectable_packets},
                     {"phase", static_cast<std::uint64_t>(
                                   selected_outer_phase)}});
            }
            uncorrectable_since_sync = 0;
            std::array<std::uint8_t, ts_packet_size> packet{};
            if (energy_descrambler.process(randomized, packet)) {
                statistics.corrected_payload_bits += corrected_payload_bits;
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
    DvbReedSolomon reed_solomon;
    EnergyDescrambler energy_descrambler;
    std::vector<std::uint8_t> rs_bytes;
    OuterFecStats statistics;
    // Consecutive uncorrectable RS packets since the last selection. A false
    // sync lock (a zero-evidence phase selected from fade garbage) would
    // otherwise stay latched forever; after a sustained failure run the
    // selection is dropped and the sliding-window search resumes on the
    // current (healthier) data.
    std::size_t uncorrectable_since_sync{};
    // Search cadence while the selected RS phase is unavailable.  The first
    // search is made as soon as the sliding candidates are full; subsequent
    // searches are throttled so a noisy interval cannot monopolize the FEC
    // worker.  This is many times longer than one RS alignment window, while
    // the candidate buffers continue to retain the newest window.
    static constexpr std::size_t alignment_search_interval = 32 * 32 * 204;
    std::size_t alignment_search_bytes{};
    std::uint64_t alignment_search_count{};
    std::size_t pending_alignment_phase{outer_interleaver_branches};
    bool alignment_search_done{};
    DiagnosticEventHandler diagnostic_handler;
};

OuterFec::OuterFec() : impl_(std::make_unique<Impl>()) {}
OuterFec::~OuterFec() noexcept = default;
OuterFec::OuterFec(OuterFec &&) noexcept = default;
OuterFec &OuterFec::operator=(OuterFec &&) noexcept = default;

void OuterFec::reset() { impl_->reset(); }

void OuterFec::set_diagnostic_handler(DiagnosticEventHandler handler) {
    impl_->diagnostic_handler = std::move(handler);
}

std::vector<std::uint8_t>
OuterFec::process(const std::span<const std::uint8_t> hard_bytes) {
    return impl_->process(hard_bytes);
}

OuterFecStats OuterFec::stats() const { return impl_->statistics; }

} // namespace airspy_tv::fec
