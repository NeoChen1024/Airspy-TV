// Internal behavior-preserving components used by StreamDecoder::Impl.
// Kept in the coordinator's anonymous namespace by the include site.
struct PostprocessedSymbol {
    std::vector<std::complex<float>> carriers;
    std::vector<float> reliabilities;
    std::vector<std::uint8_t> mother_metrics;
    std::size_t symbol_index{};
    float mer_db{};
    float preprocess_time_ms{};
    float demap_time_ms{};
    float deinterleave_time_ms{};
    float depuncture_time_ms{};
};

struct WorkerAllocation {
    std::size_t resample{};
    std::size_t symbol{};
    std::size_t viterbi{};
};

[[nodiscard]] WorkerAllocation
allocate_workers(const std::size_t requested_threads) noexcept {
    const std::size_t total = requested_threads == 0
                                  ? default_viterbi_worker_count()
                                  : requested_threads;
    if (total <= 2) {
        return {1, 1, 1};
    }
    const std::size_t resample = std::max<std::size_t>(1, total / 4);
    const std::size_t remaining = total - resample;
    const std::size_t symbol = std::max<std::size_t>(1, remaining / 2);
    return {resample, symbol, std::max<std::size_t>(1, remaining - symbol)};
}

[[nodiscard]] std::size_t
buffered_symbol_count(const std::uint32_t bandwidth,
                      const std::size_t symbol_samples) noexcept {
    // Nominal DVB-T sample rate is bandwidth * 8 / 7; retain one fifth of a
    // second at the current OFDM symbol duration.
    const std::uint64_t numerator = static_cast<std::uint64_t>(bandwidth) * 8U;
    const std::uint64_t denominator =
        7U * buffer_duration_denominator * symbol_samples;
    return std::max<std::size_t>(
        1,
        static_cast<std::size_t>((numerator + denominator - 1) / denominator));
}

[[nodiscard]] std::size_t
buffered_input_samples(const std::uint32_t sample_rate) noexcept {
    return std::max<std::size_t>(1, (static_cast<std::size_t>(sample_rate) +
                                     buffer_duration_denominator - 1) /
                                        buffer_duration_denominator);
}

[[nodiscard]] float
duration_ms(const std::chrono::steady_clock::time_point started_at) {
    return std::chrono::duration<float, std::milli>(
               std::chrono::steady_clock::now() - started_at)
        .count();
}

class SymbolPostprocessorPool {
  public:
    SymbolPostprocessorPool(const std::size_t requested_workers,
                            const TransmissionMode mode,
                            const Constellation constellation,
                            const CodeRate code_rate,
                            const std::size_t queue_capacity)
        : worker_count_(requested_workers == 0 ? default_viterbi_worker_count()
                                               : requested_workers),
          mode_(mode), constellation_(constellation), reference_(constellation),
          symbol_deinterleaver_(mode),
          bits_per_carrier_(bits_per_symbol(constellation)),
          code_rate_(code_rate), maximum_queued_(std::max<std::size_t>(
                                     queue_capacity, worker_count_ * 2)) {
        workers_.reserve(worker_count_);
        try {
            for (std::size_t index = 0; index < worker_count_; ++index) {
                workers_.emplace_back([this, index] {
                    set_current_thread_name("dvbt-sym-" +
                                            std::to_string(index));
                    run_worker();
                });
            }
        } catch (...) {
            stop_and_join();
            throw;
        }
    }

    ~SymbolPostprocessorPool() { stop_and_join(); }
    SymbolPostprocessorPool(const SymbolPostprocessorPool &) = delete;
    SymbolPostprocessorPool &
    operator=(const SymbolPostprocessorPool &) = delete;

    [[nodiscard]] bool
    compatible(const std::size_t worker_count, const TransmissionMode mode,
               const Constellation constellation, const CodeRate code_rate,
               const std::size_t queue_capacity) const noexcept {
        return worker_count_ == worker_count && mode_ == mode &&
               constellation_ == constellation && code_rate_ == code_rate &&
               maximum_queued_ ==
                   std::max<std::size_t>(queue_capacity, worker_count * 2);
    }

    void submit(std::vector<std::complex<float>> carriers,
                std::vector<float> equalizer_power,
                const std::size_t symbol_index) {
        Task task{.carriers = std::move(carriers),
                  .equalizer_power = std::move(equalizer_power),
                  .symbol_index = symbol_index};
        {
            std::unique_lock lock(mutex_);
            space_available_.wait(lock, [this] {
                return stopping_ || worker_error_ ||
                       tasks_.size() < maximum_queued_;
            });
            rethrow_worker_error();
            if (stopping_) {
                return;
            }
            task.sequence = next_sequence_++;
            ++outstanding_;
            tasks_.push_back(std::move(task));
        }
        task_ready_.notify_one();
    }

    [[nodiscard]] std::vector<PostprocessedSymbol> take_ready() {
        std::scoped_lock lock(mutex_);
        rethrow_worker_error();
        return take_ready_locked();
    }

    [[nodiscard]] std::vector<PostprocessedSymbol> flush() {
        std::unique_lock lock(mutex_);
        finished_.wait(lock,
                       [this] { return outstanding_ == 0 || worker_error_; });
        rethrow_worker_error();
        return take_ready_locked();
    }

  private:
    struct Task {
        std::uint64_t sequence{};
        std::vector<std::complex<float>> carriers;
        std::vector<float> equalizer_power;
        std::size_t symbol_index{};
    };

    [[nodiscard]] PostprocessedSymbol process(Task task) const {
        const auto preprocess_started_at = std::chrono::steady_clock::now();
        std::vector<std::complex<float>> nearest(task.carriers.size());
        for (int iteration = 0; iteration < 2; ++iteration) {
            reference_.slice_nearest(task.carriers, nearest);
            std::complex<double> numerator{};
            double denominator = 0.0;
            for (std::size_t index = 0; index < task.carriers.size(); ++index) {
                const auto value = task.carriers[index];
                const auto reference = nearest[index];
                numerator +=
                    std::conj(static_cast<std::complex<double>>(reference)) *
                    static_cast<std::complex<double>>(value);
                denominator += std::norm(reference);
            }
            const auto gain =
                static_cast<std::complex<float>>(numerator / denominator);
            if (std::abs(gain) > 1.0e-6F) {
                const std::complex<float> inverse_gain = 1.0F / gain;
                for (auto &value : task.carriers) {
                    value *= inverse_gain;
                }
            }
        }

        std::vector<float> errors;
        errors.reserve(task.carriers.size());
        reference_.slice_nearest(task.carriers, nearest);
        for (std::size_t index = 0; index < task.carriers.size(); ++index) {
            errors.push_back(std::norm(task.carriers[index] - nearest[index]));
        }
        const double mean_error =
            std::accumulate(errors.begin(), errors.end(), 0.0) /
            static_cast<double>(errors.size());
        auto middle =
            errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2);
        std::ranges::nth_element(errors, middle);
        const float reliability =
            1.0F / std::max(*middle / std::log(2.0F), 1.0e-4F);

        auto equalizer_power_order = task.equalizer_power;
        auto equalizer_middle =
            equalizer_power_order.begin() +
            static_cast<std::ptrdiff_t>(equalizer_power_order.size() / 2);
        std::ranges::nth_element(equalizer_power_order, equalizer_middle);
        const float median_equalizer_power =
            std::max(*equalizer_middle, minimum_power);
        std::vector<float> reliabilities(task.carriers.size());
        for (std::size_t index = 0; index < reliabilities.size(); ++index) {
            const float relative_channel_power =
                median_equalizer_power /
                std::max(task.equalizer_power[index], minimum_power);
            reliabilities[index] =
                reliability * std::clamp(relative_channel_power, 0.01F, 16.0F);
        }

        const std::size_t metric_count =
            task.carriers.size() * bits_per_carrier_;
        std::vector<float> demapped(metric_count);
        std::vector<float> symbol_metrics(metric_count);
        std::vector<float> bit_metrics(metric_count);
        const auto demap_started_at = std::chrono::steady_clock::now();
        reference_.demap(task.carriers, reliabilities, demapped);
        const auto deinterleave_started_at = std::chrono::steady_clock::now();
        symbol_deinterleaver_.process(demapped, bits_per_carrier_,
                                      task.symbol_index, symbol_metrics);
        bit_deinterleave(symbol_metrics, bits_per_carrier_, bit_metrics);
        const auto depuncture_started_at = std::chrono::steady_clock::now();
        std::vector<float> depunctured(
            depunctured_size(bit_metrics.size(), code_rate_));
        depuncture(bit_metrics, code_rate_, depunctured);
        std::vector<std::uint8_t> mother_metrics(depunctured.size());
        for (std::size_t index = 0; index < depunctured.size(); ++index) {
            const float soft =
                std::clamp(127.5F + (depunctured[index] * 8.0F), 0.0F, 255.0F);
            mother_metrics[index] = static_cast<std::uint8_t>(soft + 0.5F);
        }
        const auto finished_at = std::chrono::steady_clock::now();
        return {.carriers = std::move(task.carriers),
                .reliabilities = std::move(reliabilities),
                .mother_metrics = std::move(mother_metrics),
                .symbol_index = task.symbol_index,
                .mer_db = static_cast<float>(
                    -10.0 * std::log10(std::max(mean_error, 1.0e-12))),
                .preprocess_time_ms =
                    std::chrono::duration<float, std::milli>(
                        demap_started_at - preprocess_started_at)
                        .count(),
                .demap_time_ms = std::chrono::duration<float, std::milli>(
                                     deinterleave_started_at - demap_started_at)
                                     .count(),
                .deinterleave_time_ms =
                    std::chrono::duration<float, std::milli>(
                        depuncture_started_at - deinterleave_started_at)
                        .count(),
                .depuncture_time_ms = std::chrono::duration<float, std::milli>(
                                          finished_at - depuncture_started_at)
                                          .count()};
    }

    void run_worker() {
        while (true) {
            Task task;
            {
                std::unique_lock lock(mutex_);
                task_ready_.wait(
                    lock, [this] { return stopping_ || !tasks_.empty(); });
                if (stopping_ && tasks_.empty()) {
                    return;
                }
                task = std::move(tasks_.front());
                tasks_.pop_front();
            }
            space_available_.notify_one();
            try {
                const std::uint64_t sequence = task.sequence;
                auto result = process(std::move(task));
                const std::scoped_lock lock(mutex_);
                completed_.emplace(sequence, std::move(result));
                --outstanding_;
            } catch (...) {
                const std::scoped_lock lock(mutex_);
                if (!worker_error_) {
                    worker_error_ = std::current_exception();
                }
                --outstanding_;
            }
            finished_.notify_all();
            space_available_.notify_all();
        }
    }

    [[nodiscard]] std::vector<PostprocessedSymbol> take_ready_locked() {
        std::vector<PostprocessedSymbol> output;
        auto found = completed_.find(next_result_);
        while (found != completed_.end()) {
            output.push_back(std::move(found->second));
            completed_.erase(found);
            ++next_result_;
            found = completed_.find(next_result_);
        }
        return output;
    }

    void stop_and_join() {
        {
            const std::scoped_lock lock(mutex_);
            stopping_ = true;
        }
        task_ready_.notify_all();
        space_available_.notify_all();
        for (auto &worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    void rethrow_worker_error() const {
        if (worker_error_) {
            std::rethrow_exception(worker_error_);
        }
    }

    const std::size_t worker_count_;
    const TransmissionMode mode_;
    const Constellation constellation_;
    const MaxLogDemapper reference_;
    const SymbolDeinterleaver symbol_deinterleaver_;
    const std::size_t bits_per_carrier_;
    const CodeRate code_rate_;
    const std::size_t maximum_queued_;
    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable task_ready_;
    std::condition_variable space_available_;
    std::condition_variable finished_;
    std::deque<Task> tasks_;
    std::map<std::uint64_t, PostprocessedSymbol> completed_;
    std::uint64_t next_sequence_{};
    std::uint64_t next_result_{};
    std::size_t outstanding_{};
    std::exception_ptr worker_error_;
    bool stopping_{};
};

class OverwriteComplexBuffer {
  public:
    ~OverwriteComplexBuffer() {
        if (data_ != nullptr) {
            ::operator delete(data_, alignment);
        }
    }
    OverwriteComplexBuffer() = default;
    OverwriteComplexBuffer(const OverwriteComplexBuffer &) = delete;
    OverwriteComplexBuffer &operator=(const OverwriteComplexBuffer &) = delete;
    OverwriteComplexBuffer(OverwriteComplexBuffer &&other) noexcept {
        *this = std::move(other);
    }
    OverwriteComplexBuffer &operator=(OverwriteComplexBuffer &&other) noexcept {
        if (this != &other) {
            if (data_ != nullptr) {
                ::operator delete(data_, alignment);
            }
            data_ = std::exchange(other.data_, nullptr);
            size_ = std::exchange(other.size_, 0);
            capacity_ = std::exchange(other.capacity_, 0);
        }
        return *this;
    }

    void resize_for_overwrite(const std::size_t size) {
        static_assert(std::is_trivially_copyable_v<std::complex<float>> &&
                      std::is_trivially_destructible_v<std::complex<float>>);
        if (size > capacity_) {
            if (size > std::numeric_limits<std::size_t>::max() /
                           sizeof(std::complex<float>)) {
                throw std::bad_array_new_length{};
            }
            auto *next = static_cast<std::complex<float> *>(::operator new(
                sizeof(std::complex<float>) * size, alignment));
            if (data_ != nullptr) {
                ::operator delete(data_, alignment);
            }
            data_ = next;
            capacity_ = size;
        }
        size_ = size;
    }

    [[nodiscard]] std::complex<float> *data() noexcept { return data_; }
    [[nodiscard]] std::span<const std::complex<float>> view() const noexcept {
        return {data_, size_};
    }

  private:
    static constexpr std::align_val_t alignment{64};
    std::complex<float> *data_{};
    std::size_t size_{};
    std::size_t capacity_{};
};

// Parallel rational resampler with one persistent primary filter. Partition 0
// carries the exact stream state across calls; the other filters are reset and
// primed from the preceding m*Q input samples before processing their disjoint
// ranges. This preserves the continuous serial result while allowing the FIR
// work in a large input block to scale across cores.
class StreamingResampler {
  public:
    explicit StreamingResampler(const std::size_t worker_count)
        : worker_count_(std::max<std::size_t>(worker_count, 1)),
          filters_(worker_count_, nullptr) {
        workers_.reserve(worker_count_);
        try {
            for (std::size_t index = 0; index < worker_count_; ++index) {
                workers_.emplace_back([this, index] {
                    set_current_thread_name("dvbt-resamp-" +
                                            std::to_string(index));
                    run_worker(index);
                });
            }
        } catch (...) {
            stop_and_join();
            throw;
        }
    }
    ~StreamingResampler() {
        stop_and_join();
        destroy_filters();
    }
    StreamingResampler(const StreamingResampler &) = delete;
    StreamingResampler &operator=(const StreamingResampler &) = delete;
    StreamingResampler(StreamingResampler &&) = delete;
    StreamingResampler &operator=(StreamingResampler &&) = delete;

    [[nodiscard]] std::size_t worker_count() const noexcept {
        return worker_count_;
    }

    void reset() {
        destroy_filters();
        rate_ = 0;
        bandwidth_ = 0;
        interpolation_ = 0;
        decimation_ = 0;
        residual_.clear();
        history_.clear();
        output_.resize_for_overwrite(0);
    }

    void configure(const std::uint32_t rate, const std::uint32_t bandwidth) {
        if (configured() && rate == rate_ && bandwidth == bandwidth_) {
            return;
        }
        reset();
        // Nominal DVB-T output rate is bandwidth * 8 / 7 samples/second.
        const std::uint64_t interpolation =
            static_cast<std::uint64_t>(bandwidth) * 8U;
        const std::uint64_t decimation = static_cast<std::uint64_t>(rate) * 7U;
        const std::uint64_t divisor = std::gcd(interpolation, decimation);
        const unsigned int p =
            static_cast<unsigned int>(interpolation / divisor);
        const unsigned int q = static_cast<unsigned int>(decimation / divisor);
        for (auto &filter : filters_) {
            filter = rresamp_crcf_create_kaiser(p, q, resampler_semi_length,
                                                -1.0F, 60.0F);
            if (filter == nullptr) {
                destroy_filters();
                throw std::runtime_error(
                    "failed to create streaming resampler");
            }
        }
        rate_ = rate;
        bandwidth_ = bandwidth;
        interpolation_ = p;
        decimation_ = q;
        residual_.clear();
        residual_.reserve(decimation_);
        history_.clear();
        history_.reserve(resampler_semi_length * decimation_);
    }

    // Full decimation blocks are processed directly from the caller's input;
    // only the short tail crossing a call boundary is copied into residual_.
    // output_ is raw overwrite storage because liquid fills every output
    // sample, avoiding std::vector's redundant value initialization.
    [[nodiscard]] std::span<const std::complex<float>>
    process(std::span<const std::complex<float>> input) {
        if (!configured()) {
            output_.resize_for_overwrite(0);
            return {};
        }
        const std::size_t blocks =
            (residual_.size() + input.size()) / decimation_;
        if (blocks == 0) {
            residual_.insert(residual_.end(), input.begin(), input.end());
            output_.resize_for_overwrite(0);
            return {};
        }
        if (blocks > std::numeric_limits<unsigned int>::max()) {
            throw std::length_error("resampler block count exceeds liquid API");
        }
        output_.resize_for_overwrite(blocks * interpolation_);
        std::size_t output_offset = 0;

        if (!residual_.empty()) {
            const std::size_t needed = decimation_ - residual_.size();
            residual_.insert(residual_.end(), input.begin(),
                             input.begin() +
                                 static_cast<std::ptrdiff_t>(needed));
            if (rresamp_crcf_execute_block(filters_.front(), residual_.data(),
                                           1, output_.data()) != LIQUID_OK) {
                throw std::runtime_error("streaming resampler boundary failed");
            }
            output_offset += interpolation_;
            input = input.subspan(needed);
            append_history(residual_);
            residual_.clear();
        }

        const std::size_t direct_blocks = input.size() / decimation_;
        if (direct_blocks != 0) {
            process_direct(input.first(direct_blocks * decimation_),
                           direct_blocks, output_.data() + output_offset);
            output_offset += direct_blocks * interpolation_;
            append_history(input.first(direct_blocks * decimation_));
            input = input.subspan(direct_blocks * decimation_);
        }
        residual_.assign(input.begin(), input.end());
        if (output_offset != blocks * interpolation_) {
            throw std::logic_error("resampler output size mismatch");
        }
        return output_.view();
    }

    [[nodiscard]] bool configured() const noexcept {
        return !filters_.empty() && filters_.front() != nullptr;
    }
    [[nodiscard]] std::uint32_t rate() const noexcept { return rate_; }
    [[nodiscard]] std::uint32_t bandwidth() const noexcept {
        return bandwidth_;
    }

  private:
    static constexpr std::size_t minimum_partition_blocks = 1024;

    void destroy_filters() noexcept {
        for (auto &filter : filters_) {
            if (filter != nullptr) {
                rresamp_crcf_destroy(filter);
                filter = nullptr;
            }
        }
    }

    void append_history(const std::span<const std::complex<float>> samples) {
        const std::size_t capacity = resampler_semi_length * decimation_;
        if (samples.size() >= capacity) {
            history_.assign(samples.end() -
                                static_cast<std::ptrdiff_t>(capacity),
                            samples.end());
            return;
        }
        if (history_.size() + samples.size() > capacity) {
            const std::size_t discard =
                history_.size() + samples.size() - capacity;
            history_.erase(history_.begin(),
                           history_.begin() +
                               static_cast<std::ptrdiff_t>(discard));
        }
        history_.insert(history_.end(), samples.begin(), samples.end());
    }

    void process_direct(const std::span<const std::complex<float>> input,
                        const std::size_t blocks,
                        std::complex<float> *const output) {
        const std::size_t active_workers = std::min(
            worker_count_,
            std::max<std::size_t>(1, blocks / minimum_partition_blocks));
        if (active_workers == 1) {
            if (rresamp_crcf_execute_block(
                    filters_.front(),
                    const_cast<std::complex<float> *>(input.data()),
                    static_cast<unsigned int>(blocks), output) != LIQUID_OK) {
                throw std::runtime_error("streaming resampler failed");
            }
            return;
        }

        {
            const std::scoped_lock lock(worker_mutex_);
            task_input_ = input.data();
            task_output_ = output;
            task_blocks_ = blocks;
            task_active_workers_ = active_workers;
            task_history_blocks_ = history_.size() / decimation_;
            task_error_ = nullptr;
            task_remaining_ = active_workers;
            ++task_generation_;
        }
        task_ready_.notify_all();
        std::unique_lock lock(worker_mutex_);
        task_done_.wait(lock, [this] { return task_remaining_ == 0; });
        if (task_error_ != nullptr) {
            std::rethrow_exception(task_error_);
        }
        // The final partition owns the filter state at the end of this input
        // range. Promote it to primary so the next process() call continues
        // from exactly the same state as a serial resampler.
        std::swap(filters_.front(), filters_[active_workers - 1]);
    }

    void run_worker(const std::size_t index) {
        std::uint64_t seen_generation = 0;
        while (true) {
            const std::complex<float> *input = nullptr;
            std::complex<float> *output = nullptr;
            std::size_t blocks = 0;
            std::size_t active_workers = 0;
            std::size_t history_blocks = 0;
            {
                std::unique_lock lock(worker_mutex_);
                task_ready_.wait(lock, [this, seen_generation] {
                    return stopping_ || task_generation_ != seen_generation;
                });
                if (stopping_) {
                    return;
                }
                seen_generation = task_generation_;
                if (index >= task_active_workers_) {
                    continue;
                }
                input = task_input_;
                output = task_output_;
                blocks = task_blocks_;
                active_workers = task_active_workers_;
                history_blocks = task_history_blocks_;
            }

            try {
                const std::size_t begin = blocks * index / active_workers;
                const std::size_t end = blocks * (index + 1) / active_workers;
                auto filter = filters_[index];
                if (index != 0) {
                    if (rresamp_crcf_reset(filter) != LIQUID_OK) {
                        throw std::runtime_error(
                            "resampler partition reset failed");
                    }
                    const std::size_t current_history =
                        std::min<std::size_t>(begin, resampler_semi_length);
                    const std::size_t prior_history =
                        std::min(history_blocks,
                                 resampler_semi_length - current_history);
                    for (std::size_t block = history_blocks - prior_history;
                         block < history_blocks; ++block) {
                        if (rresamp_crcf_write(
                                filter, history_.data() +
                                            block * decimation_) != LIQUID_OK) {
                            throw std::runtime_error(
                                "resampler history priming failed");
                        }
                    }
                    for (std::size_t block = begin - current_history;
                         block < begin; ++block) {
                        if (rresamp_crcf_write(
                                filter, const_cast<std::complex<float> *>(
                                            input + block * decimation_)) !=
                            LIQUID_OK) {
                            throw std::runtime_error(
                                "resampler partition priming failed");
                        }
                    }
                }
                if (rresamp_crcf_execute_block(
                        filter,
                        const_cast<std::complex<float> *>(input +
                                                          begin * decimation_),
                        static_cast<unsigned int>(end - begin),
                        output + begin * interpolation_) != LIQUID_OK) {
                    throw std::runtime_error("resampler partition failed");
                }
            } catch (...) {
                const std::scoped_lock lock(worker_mutex_);
                if (task_error_ == nullptr) {
                    task_error_ = std::current_exception();
                }
            }

            {
                const std::scoped_lock lock(worker_mutex_);
                --task_remaining_;
                if (task_remaining_ == 0) {
                    task_done_.notify_one();
                }
            }
        }
    }

    void stop_and_join() noexcept {
        {
            const std::scoped_lock lock(worker_mutex_);
            stopping_ = true;
        }
        task_ready_.notify_all();
        for (auto &worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    const std::size_t worker_count_;
    std::vector<rresamp_crcf> filters_;
    std::vector<std::thread> workers_;
    std::mutex worker_mutex_;
    std::condition_variable task_ready_;
    std::condition_variable task_done_;
    const std::complex<float> *task_input_{};
    std::complex<float> *task_output_{};
    std::size_t task_blocks_{};
    std::size_t task_active_workers_{};
    std::size_t task_history_blocks_{};
    std::size_t task_remaining_{};
    std::uint64_t task_generation_{};
    std::exception_ptr task_error_;
    bool stopping_{};
    std::uint32_t rate_{};
    std::uint32_t bandwidth_{};
    std::size_t interpolation_{};
    std::size_t decimation_{};
    std::vector<std::complex<float>> residual_;
    std::vector<std::complex<float>> history_;
    OverwriteComplexBuffer output_;
};
