#include "airspy_tv/fec/soft_viterbi.hpp"

extern "C" {
#include <correct.h>
#if defined(HAVE_SSE)
#include <correct-sse.h>
#endif
}

#include <algorithm>
#include <array>
#include <bit>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <map>
#include <mutex>
#include <span>
#include <stdexcept>
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

std::vector<std::uint8_t> SoftViterbi::process_soft(
    const std::span<const std::uint8_t> soft_metrics) {
    return impl_->process_soft(soft_metrics);
}

std::vector<std::uint8_t> SoftViterbi::flush() { return impl_->flush(); }

std::size_t SoftViterbi::worker_count() const noexcept {
    return impl_->worker_count();
}

std::pair<std::uint64_t, std::uint64_t>
SoftViterbi::error_counts() const {
    return impl_->error_counts();
}

} // namespace airspy_tv::fec
