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
                       outstanding_ < maximum_queued_;
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

    [[nodiscard]]
    std::vector<PostprocessedSymbol> take_ordered(const std::size_t count) {
        std::unique_lock lock(mutex_);
        finished_.wait(lock, [this, count] {
            return ordered_ready_locked(count) || worker_error_;
        });
        rethrow_worker_error();
        auto output = take_ready_locked(count);
        lock.unlock();
        space_available_.notify_all();
        return output;
    }

    [[nodiscard]] std::vector<PostprocessedSymbol> flush() {
        std::unique_lock lock(mutex_);
        finished_.wait(lock, [this] {
            return completed_.size() == outstanding_ || worker_error_;
        });
        rethrow_worker_error();
        auto output = take_ready_locked(outstanding_);
        lock.unlock();
        space_available_.notify_all();
        return output;
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
            } catch (...) {
                const std::scoped_lock lock(mutex_);
                if (!worker_error_) {
                    worker_error_ = std::current_exception();
                }
            }
            finished_.notify_all();
        }
    }

    [[nodiscard]] bool
    ordered_ready_locked(const std::size_t count) const noexcept {
        if (count > outstanding_) {
            return false;
        }
        for (std::size_t offset = 0; offset < count; ++offset) {
            if (!completed_.contains(next_result_ + offset)) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]]
    std::vector<PostprocessedSymbol>
    take_ready_locked(const std::size_t count) {
        std::vector<PostprocessedSymbol> output;
        output.reserve(count);
        for (std::size_t offset = 0; offset < count; ++offset) {
            auto found = completed_.find(next_result_);
            if (found == completed_.end()) {
                break;
            }
            output.push_back(std::move(found->second));
            completed_.erase(found);
            ++next_result_;
            --outstanding_;
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
    // Submitted results remain outstanding until the demod thread consumes
    // them. This bounds queued, running, and completed-but-not-yet-joined work
    // with one budget instead of letting the completed map grow invisibly.
    std::size_t outstanding_{};
    std::exception_ptr worker_error_;
    bool stopping_{};
};

class StreamingResampler {
  public:
    explicit StreamingResampler(const std::size_t worker_count)
        : resampler_(worker_count, "dvbt-resamp-") {
        resampler_.set_max_slew_rate(sro_slew_rate_ppm_per_second);
    }

    [[nodiscard]] std::size_t worker_count() const noexcept {
        return resampler_.worker_count();
    }

    void reset() {
        if (configured()) {
            resampler_.set_ratio(resampler_.nominal_ratio());
        }
        resampler_.reset();
    }

    void configure(const std::uint32_t rate, const std::uint32_t bandwidth) {
        if (configured() && rate == rate_ && bandwidth == bandwidth_) {
            return;
        }
        resampler_.configure(make_resampler_config(rate, bandwidth));
        rate_ = rate;
        bandwidth_ = bandwidth;
    }

    [[nodiscard]] std::span<const std::complex<float>>
    process(std::span<const std::complex<float>> input) {
        return resampler_.process(input);
    }

    void set_sro_correction_ppm(const double correction_ppm) {
        const double scale = 1.0 + correction_ppm * 1.0e-6;
        if (!(scale > 0.0) || !std::isfinite(scale)) {
            throw std::invalid_argument("invalid SRO resampler correction");
        }
        resampler_.set_ratio(resampler_.nominal_ratio() / scale);
    }

    [[nodiscard]] double applied_sro_correction_ppm() const noexcept {
        const double ratio = resampler_.effective_ratio();
        return ratio > 0.0 ? (resampler_.nominal_ratio() / ratio - 1.0) * 1.0e6
                           : 0.0;
    }

    [[nodiscard]] double requested_ratio() const noexcept {
        return resampler_.requested_ratio();
    }

    [[nodiscard]] double effective_ratio() const noexcept {
        return resampler_.effective_ratio();
    }

    [[nodiscard]] bool configured() const noexcept {
        return resampler_.configured();
    }
    [[nodiscard]] std::uint32_t rate() const noexcept { return rate_; }
    [[nodiscard]] std::uint32_t bandwidth() const noexcept {
        return bandwidth_;
    }

  private:
    static constexpr double sro_slew_rate_ppm_per_second = 0.5;
    liquid_resampler::ArbitraryResampler resampler_;
    std::uint32_t rate_{};
    std::uint32_t bandwidth_{};
};
