// Demod-thread-owned state that persists across OFDM symbols.

enum class DemodFlow { proceed, restart, stop };
enum class DemodInputFlow { ready, retry, end, stop };

struct DemodWindowMetrics {
    float wall_time_ms{};
    std::uint64_t symbol_count{};
    double sample_count{};
    float input_seconds{};
    float timing_offset{};
    double cir_offset{};
    double observed_drift{};
    double corrected_drift{};
    double timing_shift{};
    double rolling_shift_rate_ppm{};
    double smoothed_timing_drift{};
};

struct DemodRuntimeState {
    struct PendingSymbol {
        std::vector<std::complex<float>> payload;
        std::vector<float> equalizer_power;
    };

    // Carrier grid, FFT storage, and postprocessing configuration.
    ReceiverParameters selected_parameters;
    std::size_t maximum{6816};
    std::size_t fft_size{8192};
    std::size_t guard_size{2048};
    std::size_t period{10240};
    std::vector<std::complex<float>> fft_in;
    std::vector<std::complex<float>> fft_out;
    FftwfPlan plan;
    std::vector<std::size_t> continual_indices;
    std::vector<std::size_t> tps_indices;
    std::vector<std::complex<float>> tps_values;
    std::array<std::vector<std::size_t>, 4> pilot_indices;
    std::array<std::vector<std::size_t>, 4> payload_indices;
    std::optional<DecoderParameters> decoder_parameters;
    WorkerAllocation workers{1, 1, 1};
    SymbolPostprocessorPool *postprocessor{};
    std::deque<PendingSymbol> pending_symbols;
    std::deque<PostprocessedSymbol> gate_buffer;
    std::size_t symbol_queue_capacity{initial_symbol_queue_capacity};
    bool in_hopeless_region{};

    // Stream position, lock continuity, and recovery policy.
    std::uint64_t demod_generation{};
    bool last_reanchor_carried{};
    std::uint64_t seen_sync_version{};
    bool have_grid{};
    std::uint64_t next_symbol_start{};
    float nco_phase{};
    std::uint64_t symbol_count{};
    std::uint64_t analysis_symbol_count{};
    float latest_cp_snr_db{};
    float latest_deepest_notch_db{};
    std::uint64_t frozen_symbol_count{};
    std::uint64_t hopeless_window_count{};
    int stable_pending_offset{std::numeric_limits<int>::max()};
    int stable_pending_count{};
    std::uint64_t tps_mismatch_symbols{};
    int lock_hold{};

    // Per-statistics-window accumulators.
    std::uint64_t window_symbol_count{};
    double mer_sum{};
    float preprocess_time_sum{};
    float demap_time_sum{};
    float deinterleave_time_sum{};
    float depuncture_time_sum{};
    double timing_acc{};
    std::uint64_t timing_count{};
    double latest_raw_timing{};
    std::uint64_t timing_raw_count{};
    std::uint64_t timing_rejected_count{};
    float fade_indicator{1.0F};
    std::chrono::steady_clock::time_point demod_busy_started_at{};
    std::chrono::steady_clock::time_point window_started_at{
        std::chrono::steady_clock::now()};

    // Sample-clock estimator and integer/fractional timing actuator.
    static constexpr std::size_t tau_history_n = 24;
    static constexpr std::size_t tau_history_min = 16;
    static constexpr std::size_t shift_rate_history_n = 64;
    TimingSlopeTracker timing_tracker;
    double last_windowed_timing{};
    double smoothed_sample_clock_ppm{};
    double fractional_timing{};
    std::array<double, tau_history_n> tau_history{};
    std::array<double, tau_history_n> tau_sample_history{};
    std::size_t tau_history_head{};
    std::size_t tau_history_count{};
    double timing_elapsed_samples{};
    double accumulated_window_shift{};
    double last_windowed_cir_avg{};
    double last_timing_window_shift{};
    double last_telemetry_window_shift{};
    double window_cir_offset_sum{};
    std::array<double, shift_rate_history_n> shift_rate_steps{};
    std::array<double, shift_rate_history_n> shift_rate_samples{};
    std::size_t shift_rate_history_head{};
    std::size_t shift_rate_history_count{};
    double rolling_shift_steps{};
    double rolling_shift_samples{};
    int applied_cir_offset{};
    double cir_confidence{};
    float acquisition_time_ms{};

    explicit DemodRuntimeState(const std::uint64_t generation)
        : demod_generation(generation) {}
};
