#include "airspy_tv/dvbt/transport_decoder.hpp"

extern "C" {
#include <correct.h>
#if defined(HAVE_SSE)
#include <correct-sse.h>
#endif
}

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <mutex>
#include <span>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {

std::size_t default_viterbi_worker_count() noexcept {
    const unsigned int detected = std::thread::hardware_concurrency();
    return detected == 0 ? 1 : static_cast<std::size_t>(detected);
}

namespace {

constexpr std::size_t convolutional_rate = 2;
constexpr std::size_t viterbi_window_bits = 8192;
constexpr std::size_t viterbi_margin_bits = 256;
constexpr std::size_t viterbi_output_bits =
    viterbi_window_bits - (2 * viterbi_margin_bits);
// Retains more than 200 ms at DVB-T's maximum useful bit rate.
constexpr std::size_t viterbi_buffer_windows = 1024;
constexpr std::size_t rs_packet_size = 204;
constexpr std::size_t ts_packet_size = 188;
constexpr std::size_t packets_per_energy_frame = 8;
constexpr std::size_t outer_interleaver_branches = 12;
constexpr std::size_t outer_interleaver_step = 17;

class SoftViterbi {
  public:
    explicit SoftViterbi(const std::size_t requested_workers)
        : worker_count_(requested_workers == 0 ? default_viterbi_worker_count()
                                               : requested_workers),
          maximum_queued_windows_(std::max<std::size_t>(
              worker_count_ * 2, viterbi_buffer_windows)) {
        workers_.reserve(worker_count_);
        try {
            for (std::size_t index = 0; index < worker_count_; ++index) {
                workers_.emplace_back([this] { run_worker(); });
            }
        } catch (...) {
            {
                const std::scoped_lock lock(mutex_);
                stopping_ = true;
            }
            task_ready_.notify_all();
            for (auto &worker : workers_) {
                worker.join();
            }
            throw;
        }
    }

    ~SoftViterbi() {
        {
            const std::scoped_lock lock(mutex_);
            stopping_ = true;
        }
        task_ready_.notify_all();
        queue_space_.notify_all();
        for (auto &worker : workers_) {
            worker.join();
        }
    }

    SoftViterbi(const SoftViterbi &) = delete;
    SoftViterbi &operator=(const SoftViterbi &) = delete;

    void reset() {
        static_cast<void>(flush());
        metrics_.clear();
        metric_offset_ = 0;
        std::scoped_lock lock(mutex_);
        completed_.clear();
        next_sequence_ = 0;
        next_result_ = 0;
        hard_decision_errors_ = 0;
        compared_metrics_ = 0;
        worker_error_ = nullptr;
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process(const std::span<const float> llrs) {
        compact_for(llrs.size());
        const std::size_t old_size = metrics_.size();
        metrics_.resize(old_size + llrs.size());
        for (std::size_t index = 0; index < llrs.size(); ++index) {
            const float soft =
                std::clamp(127.5F + (llrs[index] * 8.0F), 0.0F, 255.0F);
            metrics_[old_size + index] = static_cast<std::uint8_t>(soft + 0.5F);
        }

        return dispatch_ready_windows();
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_soft(const std::span<const std::uint8_t> soft_metrics) {
        compact_for(soft_metrics.size());
        metrics_.insert(metrics_.end(), soft_metrics.begin(),
                        soft_metrics.end());
        return dispatch_ready_windows();
    }

  private:
    void compact_for(const std::size_t incoming_size) {
        if (metric_offset_ != 0 &&
            (metric_offset_ >= 65'536 ||
             metrics_.capacity() - metrics_.size() < incoming_size)) {
            metrics_.erase(metrics_.begin(),
                           metrics_.begin() +
                               static_cast<std::ptrdiff_t>(metric_offset_));
            metric_offset_ = 0;
        }
    }

    [[nodiscard]] std::vector<std::uint8_t> dispatch_ready_windows() {
        constexpr std::size_t window_metrics =
            viterbi_window_bits * convolutional_rate;
        constexpr std::size_t consumed_metrics =
            viterbi_output_bits * convolutional_rate;
        while (metrics_.size() - metric_offset_ >= window_metrics) {
            Task task;
            task.metrics.assign(
                metrics_.begin() + static_cast<std::ptrdiff_t>(metric_offset_),
                metrics_.begin() + static_cast<std::ptrdiff_t>(metric_offset_ +
                                                               window_metrics));
            {
                std::unique_lock lock(mutex_);
                queue_space_.wait(lock, [this] {
                    return stopping_ || worker_error_ ||
                           tasks_.size() < maximum_queued_windows_;
                });
                rethrow_worker_error();
                if (stopping_) {
                    throw std::runtime_error("Viterbi worker pool stopped");
                }
                task.sequence = next_sequence_++;
                ++outstanding_;
                tasks_.push_back(std::move(task));
            }
            task_ready_.notify_one();
            metric_offset_ += consumed_metrics;
        }
        return take_ready();
    }

  public:
    [[nodiscard]] std::vector<std::uint8_t> flush() {
        std::unique_lock lock(mutex_);
        all_finished_.wait(
            lock, [this] { return outstanding_ == 0 || worker_error_; });
        rethrow_worker_error();
        return take_ready_locked();
    }

    [[nodiscard]] std::size_t worker_count() const noexcept {
        return worker_count_;
    }

  private:
    struct Task {
        std::uint64_t sequence{};
        std::vector<std::uint8_t> metrics;
    };

    struct Result {
        std::vector<std::uint8_t> bytes;
        std::uint64_t hard_decision_errors{};
        std::uint64_t compared_metrics{};
    };

#if defined(HAVE_SSE)
    using DecoderHandle = correct_convolutional_sse;
#else
    using DecoderHandle = correct_convolutional;
#endif

    static DecoderHandle *create_decoder() {
        constexpr std::array<correct_convolutional_polynomial_t, 2> polynomials{
            0117, 0155};
#if defined(HAVE_SSE)
        return correct_convolutional_sse_create(convolutional_rate, 7,
                                                polynomials.data());
#else
        return correct_convolutional_create(convolutional_rate, 7,
                                            polynomials.data());
#endif
    }

    static void destroy_decoder(DecoderHandle *decoder) {
#if defined(HAVE_SSE)
        correct_convolutional_sse_destroy(decoder);
#else
        correct_convolutional_destroy(decoder);
#endif
    }

    static Result decode(DecoderHandle *decoder, const Task &task) {
        std::array<std::uint8_t, viterbi_window_bits / 8> decoded{};
#if defined(HAVE_SSE)
        const ssize_t decoded_bytes = correct_convolutional_sse_decode_soft(
            decoder, task.metrics.data(), task.metrics.size(), decoded.data());
#else
        const ssize_t decoded_bytes = correct_convolutional_decode_soft(
            decoder, task.metrics.data(), task.metrics.size(), decoded.data());
#endif
        constexpr std::size_t margin_bytes = viterbi_margin_bits / 8;
        constexpr std::size_t output_bytes = viterbi_output_bits / 8;
        if (decoded_bytes < static_cast<ssize_t>(margin_bytes + output_bytes)) {
            throw std::runtime_error("libcorrect Viterbi decode failed");
        }
        Result result;
        result.bytes.assign(decoded.begin() + margin_bytes,
                            decoded.begin() + margin_bytes + output_bytes);

        // Estimate pre-Viterbi BER by re-encoding the survivor path and
        // comparing it with hard decisions from the received mother-code
        // metrics. The traceback margins establish encoder state and are not
        // counted. Metric 128 is the neutral value inserted for punctures.
        constexpr std::array<std::uint8_t, 2> polynomials{0117, 0155};
        std::uint8_t shift_register = 0;
        for (std::size_t bit = 0; bit < viterbi_window_bits; ++bit) {
            const std::uint8_t decoded_bit = static_cast<std::uint8_t>(
                (decoded[bit / 8] >> (7U - (bit % 8))) & 1U);
            shift_register = static_cast<std::uint8_t>(
                ((shift_register << 1U) | decoded_bit) & 0x7FU);
            if (bit < viterbi_margin_bits ||
                bit >= viterbi_margin_bits + viterbi_output_bits) {
                continue;
            }
            for (std::size_t branch = 0; branch < polynomials.size();
                 ++branch) {
                const std::uint8_t metric =
                    task.metrics[(bit * convolutional_rate) + branch];
                if (metric == 128U) {
                    continue;
                }
                const std::uint8_t encoded_bit = static_cast<std::uint8_t>(
                    std::popcount(static_cast<unsigned int>(
                        shift_register & polynomials[branch])) &
                    1U);
                result.hard_decision_errors += static_cast<std::uint64_t>(
                    (metric > 127U) != (encoded_bit != 0U));
                ++result.compared_metrics;
            }
        }
        return result;
    }

    void run_worker() noexcept {
        DecoderHandle *decoder = create_decoder();
        if (decoder == nullptr) {
            const std::scoped_lock lock(mutex_);
            worker_error_ = std::make_exception_ptr(
                std::runtime_error("failed to create libcorrect Viterbi"));
            all_finished_.notify_all();
            queue_space_.notify_all();
            return;
        }
        while (true) {
            Task task;
            {
                std::unique_lock lock(mutex_);
                task_ready_.wait(
                    lock, [this] { return stopping_ || !tasks_.empty(); });
                if (stopping_ && tasks_.empty()) {
                    break;
                }
                task = std::move(tasks_.front());
                tasks_.pop_front();
            }
            queue_space_.notify_one();
            try {
                auto output = decode(decoder, task);
                const std::scoped_lock lock(mutex_);
                completed_.emplace(task.sequence, std::move(output));
                --outstanding_;
            } catch (...) {
                const std::scoped_lock lock(mutex_);
                if (!worker_error_) {
                    worker_error_ = std::current_exception();
                }
                --outstanding_;
            }
            all_finished_.notify_all();
            queue_space_.notify_all();
        }
        destroy_decoder(decoder);
    }

    void rethrow_worker_error() const {
        if (worker_error_) {
            std::rethrow_exception(worker_error_);
        }
    }

    [[nodiscard]] std::vector<std::uint8_t> take_ready() {
        std::scoped_lock lock(mutex_);
        rethrow_worker_error();
        return take_ready_locked();
    }

    [[nodiscard]] std::vector<std::uint8_t> take_ready_locked() {
        std::vector<std::uint8_t> output;
        auto found = completed_.find(next_result_);
        while (found != completed_.end()) {
            output.insert(output.end(), found->second.bytes.begin(),
                          found->second.bytes.end());
            hard_decision_errors_ += found->second.hard_decision_errors;
            compared_metrics_ += found->second.compared_metrics;
            completed_.erase(found);
            ++next_result_;
            found = completed_.find(next_result_);
        }
        return output;
    }

  public:
    [[nodiscard]] std::pair<std::uint64_t, std::uint64_t> error_counts() const {
        const std::scoped_lock lock(mutex_);
        return {hard_decision_errors_, compared_metrics_};
    }

  private:
    std::vector<std::uint8_t> metrics_;
    std::size_t metric_offset_{};
    const std::size_t worker_count_;
    const std::size_t maximum_queued_windows_;
    std::vector<std::thread> workers_;
    mutable std::mutex mutex_;
    std::condition_variable task_ready_;
    std::condition_variable queue_space_;
    std::condition_variable all_finished_;
    std::deque<Task> tasks_;
    std::map<std::uint64_t, Result> completed_;
    std::uint64_t next_sequence_{};
    std::uint64_t next_result_{};
    std::uint64_t hard_decision_errors_{};
    std::uint64_t compared_metrics_{};
    std::size_t outstanding_{};
    std::exception_ptr worker_error_;
    bool stopping_{};
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
                              const std::span<std::uint8_t> decoded,
                              std::uint64_t *corrected_payload_bits = nullptr) {
        if (encoded.size() != rs_packet_size ||
            decoded.size() != ts_packet_size) {
            throw std::invalid_argument("DVB-T RS block size mismatch");
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

    // RS(204,188) is systematic, so an uncorrectable codeword still carries
    // the received randomized TS bytes in its first 188 positions. Once the
    // energy-frame phase is known, preserve packet cadence and mark the output
    // as corrupt instead of silently creating a continuity-counter gap.
    [[nodiscard]] bool
    process_corrupt(const std::span<const std::uint8_t> input,
                    const std::span<std::uint8_t> output) {
        if (input.size() != ts_packet_size || output.size() != ts_packet_size) {
            throw std::invalid_argument("DVB-T energy block size mismatch");
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

struct TransportDecoder::Impl {
    Impl(const CodeRate selected_code_rate, const std::size_t viterbi_workers)
        : code_rate(selected_code_rate), viterbi(viterbi_workers) {
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
        statistics.viterbi_workers = viterbi.worker_count();
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
        return process_viterbi_output(decoded);
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_soft(const std::span<const std::uint8_t> mother_metrics) {
        return process_viterbi_output(viterbi.process_soft(mother_metrics));
    }

    [[nodiscard]] std::vector<std::uint8_t> flush() {
        return process_viterbi_output(viterbi.flush());
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_viterbi_output(const std::span<const std::uint8_t> decoded) {
        constexpr std::size_t output_bytes = viterbi_output_bits / 8;
        if (decoded.size() % output_bytes != 0) {
            throw std::runtime_error("misaligned Viterbi window output");
        }
        std::vector<std::uint8_t> transport_stream;
        for (std::size_t offset = 0; offset < decoded.size();
             offset += output_bytes) {
            auto packets =
                process_decoded(decoded.subspan(offset, output_bytes));
            transport_stream.insert(transport_stream.end(), packets.begin(),
                                    packets.end());
        }
        return transport_stream;
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_decoded(const std::span<const std::uint8_t> decoded) {
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
            std::array<std::uint8_t, ts_packet_size> packet{};
            if (energy_descrambler.process(randomized, packet)) {
                statistics.post_viterbi_error_bits += corrected_payload_bits;
                statistics.post_viterbi_compared_bits += ts_packet_size * 8;
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

TransportDecoder::TransportDecoder(const CodeRate code_rate,
                                   const std::size_t viterbi_workers)
    : impl_(std::make_unique<Impl>(code_rate, viterbi_workers)) {}

TransportDecoder::~TransportDecoder() noexcept = default;
TransportDecoder::TransportDecoder(TransportDecoder &&) noexcept = default;
TransportDecoder &
TransportDecoder::operator=(TransportDecoder &&) noexcept = default;

void TransportDecoder::reset() { impl_->reset(); }

std::vector<std::uint8_t>
TransportDecoder::process(const std::span<const float> punctured_llrs) {
    return impl_->process(punctured_llrs);
}

std::vector<std::uint8_t> TransportDecoder::process_soft(
    const std::span<const std::uint8_t> mother_metrics) {
    return impl_->process_soft(mother_metrics);
}

std::vector<std::uint8_t> TransportDecoder::flush() { return impl_->flush(); }

TransportDecoderStats TransportDecoder::stats() const {
    auto statistics = impl_->statistics;
    const auto [errors, compared] = impl_->viterbi.error_counts();
    statistics.pre_viterbi_error_bits = errors;
    statistics.pre_viterbi_compared_bits = compared;
    return statistics;
}

} // namespace airspy_tv::dvbt
