// Demod-thread-owned pilot-slope unwrapping and robust filtering.
struct TimingSlopeTracker {
    void reset(const std::size_t fft_size = 0) noexcept {
        ambiguity_period = fft_size == 0
                               ? 0.0
                               : static_cast<double>(fft_size) /
                                     static_cast<double>(timing_pilot_spacing);
        history.fill(0.0);
        history_head = 0;
        accepted_count = 0;
        filtered_tau = 0.0;
        initialized = false;
    }

    [[nodiscard]] std::optional<double>
    observe(const double measured_tau) noexcept {
        if (!std::isfinite(measured_tau) || ambiguity_period <= 0.0) {
            return std::nullopt;
        }
        if (!initialized) {
            // Seed the whole short history with the first valid observation so
            // median_history() never has to sort a partially initialized
            // buffer.
            history.fill(measured_tau);
            history_head = 1 % history.size();
            filtered_tau = measured_tau;
            initialized = true;
            accepted_count = 1;
            return filtered_tau;
        }
        double candidate = measured_tau;
        candidate += ambiguity_period *
                     std::round((filtered_tau - candidate) / ambiguity_period);
        const double center = median_history();
        if (std::abs(candidate - center) > timing_outlier_limit_samples) {
            // A sample-clock drift cannot move the FFT boundary by dozens of
            // samples in a handful of OFDM symbols. A persistent jump here
            // is therefore a phase-slope ambiguity or a multipath outlier,
            // not a new timing branch to adopt. Keeping the last valid slope
            // is safe because the closed loop below prevents the true offset
            // from approaching this ambiguity in the first place. Adopting
            // the old branch after a few confirmations was what turned the
            // capture's 282 -> 405 -> 680 sequence into a phase storm.
            return std::nullopt;
        }
        history[history_head] = candidate;
        history_head = (history_head + 1) % history.size();
        const double robust_tau = median_history();
        filtered_tau = 0.25 * robust_tau + 0.75 * filtered_tau;
        ++accepted_count;
        return filtered_tau;
    }

    void rebase_window(const double applied_shift_samples) noexcept {
        if (!initialized || !std::isfinite(applied_shift_samples)) {
            return;
        }
        // Moving the FFT start by d changes the measured pilot slope by -d.
        // Preserve the same physical timing branch across intentional window
        // recentering so the outlier gate does not reject every later symbol.
        for (double &sample : history) {
            sample -= applied_shift_samples;
        }
        filtered_tau -= applied_shift_samples;
    }

    [[nodiscard]] std::optional<double> filtered() const noexcept {
        // A few accepted symbols make the shared verify ramp independent of
        // the first noisy pilot observation after acquisition.
        return accepted_count >= 4 && initialized
                   ? std::optional<double>{filtered_tau}
                   : std::nullopt;
    }

  private:
    [[nodiscard]] double median_history() const noexcept {
        std::array<double, timing_filter_history_size> sorted = history;
        std::ranges::sort(sorted);
        const std::size_t middle = sorted.size() / 2;
        if (sorted.size() % 2 != 0) {
            return sorted[middle];
        }
        return 0.5 * (sorted[middle - 1] + sorted[middle]);
    }

    double ambiguity_period{};
    std::array<double, timing_filter_history_size> history{};
    std::size_t history_head{};
    std::size_t accepted_count{};
    double filtered_tau{};
    bool initialized{};
};
