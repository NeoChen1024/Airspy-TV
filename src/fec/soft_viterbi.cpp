#include "airspy_tv/fec/soft_viterbi.hpp"
#include "airspy_tv/thread_name.hpp"

#include "viterbi/viterbi_decoder_core.h"
#include "viterbi/viterbi_decoder_scalar.h"
#if defined(AIRSPY_TV_USE_SIMD_VITERBI) && defined(__AVX2__)
#include "viterbi/x86/viterbi_decoder_avx_u16.h"
#elif defined(AIRSPY_TV_USE_SIMD_VITERBI) && defined(__SSE4_1__)
#include "viterbi/x86/viterbi_decoder_sse_u16.h"
#elif defined(AIRSPY_TV_USE_SIMD_VITERBI) && defined(__aarch64__) &&           \
    defined(__ARM_NEON)
#include "viterbi/arm/viterbi_decoder_neon_u16.h"
#endif

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <map>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace airspy_tv::fec {

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
static_assert(viterbi_output_bits / 8 == viterbi_output_bytes);
// Retains more than 200 ms at DVB-T's maximum useful bit rate.
constexpr std::size_t viterbi_buffer_windows = 1024;

} // namespace

struct SoftViterbi::Impl {
    explicit Impl(const std::size_t requested_workers)
        : worker_count_(requested_workers == 0 ? default_viterbi_worker_count()
                                               : requested_workers),
          maximum_queued_windows_(std::max<std::size_t>(
              worker_count_ * 2, viterbi_buffer_windows)) {
        workers_.reserve(worker_count_);
        try {
            for (std::size_t index = 0; index < worker_count_; ++index) {
                workers_.emplace_back([this, index] {
                    set_current_thread_name("dvbt-vit-" +
                                            std::to_string(index));
                    run_worker();
                });
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

    ~Impl() {
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

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;

    void reset() {
        static_cast<void>(flush());
        metrics_.clear();
        metric_offset_ = 0;
        std::scoped_lock const lock(mutex_);
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
            metrics_[old_size + index] =
                static_cast<std::uint8_t>(std::lround(soft));
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

    struct Task {
        std::uint64_t sequence{};
        std::vector<std::uint8_t> metrics;
    };

    struct Result {
        std::vector<std::uint8_t> bytes;
        std::uint64_t hard_decision_errors{};
        std::uint64_t compared_metrics{};
    };

    static constexpr std::size_t constraint_length = 7;
    using DecoderHandle =
        ViterbiDecoder_Core<constraint_length, convolutional_rate,
                            std::uint16_t, std::int16_t>;
#if defined(AIRSPY_TV_USE_SIMD_VITERBI) && defined(__AVX2__)
    using DecoderType =
        ViterbiDecoder_AVX_u16<constraint_length, convolutional_rate>;
#elif defined(AIRSPY_TV_USE_SIMD_VITERBI) && defined(__SSE4_1__)
    using DecoderType =
        ViterbiDecoder_SSE_u16<constraint_length, convolutional_rate>;
#elif defined(AIRSPY_TV_USE_SIMD_VITERBI) && defined(__aarch64__) &&           \
    defined(__ARM_NEON)
    using DecoderType =
        ViterbiDecoder_NEON_u16<constraint_length, convolutional_rate>;
#else
    using DecoderType =
        ViterbiDecoder_Scalar<constraint_length, convolutional_rate,
                              std::uint16_t, std::int16_t>;
#endif
    using BranchTable =
        ViterbiBranchTable<constraint_length, convolutional_rate, std::int16_t>;

    // DVB-T generator polynomials (171, 133 octal) in Phil Karn's reversed
    // convention. The branch table is read-only once built, so all workers
    // share one instance.
    static BranchTable &shared_branch_table() {
        static constexpr std::array<std::uint8_t, 2> polynomials{79, 109};
        static BranchTable table(polynomials.data(), 127, -127);
        return table;
    }

    // soft_decision_max_error = (127 - (-127)) * R = 508; a margin of five
    // max errors keeps the u16 accumulator far from saturation between
    // renormalisations (mirrors the upstream SOFT16 example).
    static ViterbiDecoder_Config<std::uint16_t> decoder_config() {
        constexpr auto max_error =
            static_cast<std::uint16_t>(254U * convolutional_rate);
        constexpr auto error_margin =
            static_cast<std::uint16_t>(max_error * 5U);
        return {max_error, 0U, error_margin,
                static_cast<std::uint16_t>(65'535U - error_margin)};
    }

    static std::unique_ptr<DecoderHandle> create_decoder() {
        auto decoder = std::make_unique<DecoderHandle>(shared_branch_table(),
                                                       decoder_config());
        decoder->set_traceback_length(viterbi_window_bits);
        return decoder;
    }

    static Result decode(DecoderHandle *decoder, const Task &task) {
        // ViterbiDecoderCpp consumes signed soft decisions in [-127, +127];
        // the pipeline stores unsigned 0..255 metrics with 128 as the neutral
        // puncture value, so subtract 128 and clamp to keep the domain
        // symmetric with the +-127 branch references.
        thread_local std::vector<std::int16_t> soft;
        // ViterbiDecoderCpp's chainback needs (K-1) more stored decisions
        // than the output bits it returns, so pad the window with (K-1)
        // neutral (0) symbols. Their decoded bits fall inside the discarded
        // back margin, and a constant soft value contributes the same branch
        // error to every state, so the 7680-bit output is unaffected. The
        // pad length must stay in lockstep with
        // set_traceback_length(viterbi_window_bits) in create_decoder():
        // update() feeds window + (K-1) bits into a buffer sized for
        // window + (K-1).
        constexpr std::size_t tail_symbols =
            (constraint_length - 1U) * convolutional_rate;
        soft.assign(task.metrics.size() + tail_symbols, 0);
        for (std::size_t index = 0; index < task.metrics.size(); ++index) {
            soft[index] = static_cast<std::int16_t>(std::clamp(
                static_cast<int>(task.metrics[index]) - 128, -127, 127));
        }
        // reset() starts each window from state 0; the 256-bit leading margin
        // gives the trellis room to converge before the kept output begins.
        decoder->reset(0);
        static_cast<void>(DecoderType::template update<std::uint64_t>(
            *decoder, soft.data(), soft.size()));
        std::array<std::uint8_t, viterbi_window_bits / 8> decoded{};
        decoder->chainback(decoded.data(), viterbi_window_bits);
        return extract_output(decoded, task);
    }
    // Shared by both backends: slice the 7680-bit output out of the 8192-bit
    // window and estimate the pre-Viterbi BER by re-encoding the survivor
    // path against the received mother-code metrics. The traceback margins
    // establish encoder state and are not counted. Metric 128 is the neutral
    // value inserted for punctures.
    static Result extract_output(
        const std::array<std::uint8_t, viterbi_window_bits / 8> &decoded,
        const Task &task) {
        constexpr std::size_t margin_bytes = viterbi_margin_bits / 8;
        constexpr std::size_t output_bytes = viterbi_output_bits / 8;
        Result result;
        result.bytes.assign(decoded.begin() + margin_bytes,
                            decoded.begin() + margin_bytes + output_bytes);

        constexpr std::array<std::uint8_t, 2> polynomials{0117, 0155};
        std::uint8_t shift_register = 0;
        for (std::size_t bit = 0; bit < viterbi_window_bits; ++bit) {
            const auto decoded_bit = static_cast<std::uint8_t>(
                (decoded[bit / 8] >> (7U - (bit % 8))) & 1U);
            shift_register = static_cast<std::uint8_t>(
                ((static_cast<unsigned int>(shift_register) << 1U) |
                 decoded_bit) &
                0x7FU);
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
                const auto encoded_bit = static_cast<std::uint8_t>(
                    static_cast<unsigned int>(
                        std::popcount(static_cast<unsigned int>(
                            shift_register & polynomials[branch]))) &
                    1U);
                result.hard_decision_errors += static_cast<std::uint64_t>(
                    (metric > 127U) != (encoded_bit != 0U));
                ++result.compared_metrics;
            }
        }
        return result;
    }

    void run_worker() noexcept {
        std::unique_ptr<DecoderHandle> decoder;
        try {
            decoder = create_decoder();
        } catch (...) {
            const std::scoped_lock lock(mutex_);
            if (!worker_error_) {
                worker_error_ = std::current_exception();
            }
            all_finished_.notify_all();
            queue_space_.notify_all();
            return;
        }
        if (decoder == nullptr) {
            const std::scoped_lock lock(mutex_);
            worker_error_ = std::make_exception_ptr(
                std::runtime_error("failed to create Viterbi decoder"));
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
                auto output = decode(decoder.get(), task);
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
    }

    void rethrow_worker_error() const {
        if (worker_error_) {
            std::rethrow_exception(worker_error_);
        }
    }

    [[nodiscard]] std::vector<std::uint8_t> take_ready() {
        std::scoped_lock const lock(mutex_);
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

    [[nodiscard]] std::pair<std::uint64_t, std::uint64_t> error_counts() const {
        const std::scoped_lock lock(mutex_);
        return {hard_decision_errors_, compared_metrics_};
    }

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

SoftViterbi::SoftViterbi(const std::size_t requested_workers)
    : impl_(std::make_unique<Impl>(requested_workers)) {}

SoftViterbi::~SoftViterbi() noexcept = default;
SoftViterbi::SoftViterbi(SoftViterbi &&) noexcept = default;
SoftViterbi &SoftViterbi::operator=(SoftViterbi &&) noexcept = default;

void SoftViterbi::reset() { impl_->reset(); }

std::vector<std::uint8_t>
SoftViterbi::process(const std::span<const float> llrs) {
    return impl_->process(llrs);
}

std::vector<std::uint8_t>
SoftViterbi::process_soft(const std::span<const std::uint8_t> soft_metrics) {
    return impl_->process_soft(soft_metrics);
}

std::vector<std::uint8_t> SoftViterbi::flush() { return impl_->flush(); }

std::size_t SoftViterbi::worker_count() const noexcept {
    return impl_->worker_count();
}

std::pair<std::uint64_t, std::uint64_t> SoftViterbi::error_counts() const {
    return impl_->error_counts();
}

} // namespace airspy_tv::fec
