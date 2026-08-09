// DemodStage worker state that persists across OFDM symbols.

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
    double smoothed_timing_drift{};
    std::uint64_t output_begin_sample{};
    std::uint64_t output_midpoint_sample{};
    std::uint64_t output_end_sample{};
    std::uint64_t source_epoch{};
    std::uint64_t source_begin_sample{};
    std::uint64_t source_end_sample{};
    std::uint32_t source_sample_rate_hz{};
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
    std::vector<std::complex<float>> channel_scratch;
    std::array<double, 1024> timing_estimates_scratch{};
    FftwfPlan plan;
    std::vector<std::size_t> continual_indices;
    std::vector<std::size_t> tps_indices;
    std::vector<std::complex<float>> tps_values;
    std::array<std::vector<std::size_t>, 4> pilot_indices;
    std::array<std::vector<std::size_t>, 4> payload_indices;
    std::optional<DecoderParameters> decoder_parameters;
    WorkerAllocation workers{1, 1, 1};
    SymbolPostprocessorPool *postprocessor{};
    std::size_t postprocessor_pending_symbols{};
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
    std::uint64_t symbol_count{};
    std::uint64_t analysis_symbol_count{};
    float latest_cp_snr_db{};
    float latest_deepest_notch_db{};
    std::uint64_t frozen_symbol_count{};
    std::uint64_t cfo_recovery_symbol_count{};
    std::uint64_t cfo_healthy_symbol_count{};
    std::uint64_t hopeless_window_count{};
    int stable_pending_offset{std::numeric_limits<int>::max()};
    int stable_pending_count{};
    std::uint64_t tps_mismatch_symbols{};
    int lock_hold{};

    // Per-statistics-window accumulators.
    std::uint64_t window_symbol_count{};
    std::uint64_t window_sequence{};
    std::uint64_t window_source_epoch{};
    std::uint64_t window_output_begin_sample{};
    double mer_sum{};
    float preprocess_time_sum{};
    float demap_time_sum{};
    float deinterleave_time_sum{};
    float depuncture_time_sum{};
    double demod_busy_time_sum_ms{};
    double ring_wait_time_sum_ms{};
    double ring_copy_time_sum_ms{};
    double fft_cfo_time_sum_ms{};
    double fft_execute_time_sum_ms{};
    double cfo_track_time_sum_ms{};
    double pilot_lock_time_sum_ms{};
    std::uint64_t pilot_expected_phase_checks{};
    std::uint64_t pilot_expected_phase_fast_accepts{};
    std::uint64_t pilot_expected_phase_fallbacks{};
    double pilot_expected_confidence_sum{};
    float pilot_expected_confidence_min{1.0F};
    double reacquisition_time_sum_ms{};
    double channel_estimate_time_sum_ms{};
    double channel_pilot_time_sum_ms{};
    double channel_notch_time_sum_ms{};
    double channel_timing_time_sum_ms{};
    double channel_timing_generate_time_sum_ms{};
    double channel_timing_select_time_sum_ms{};
    double channel_cir_time_sum_ms{};
    double channel_interpolate_time_sum_ms{};
    double channel_tps_extract_time_sum_ms{};
    double tps_time_sum_ms{};
    double payload_extract_time_sum_ms{};
    double symbol_submit_time_sum_ms{};
    double postprocess_wait_time_sum_ms{};
    double output_time_sum_ms{};
    double timing_acc{};
    std::uint64_t timing_count{};
    double latest_raw_timing{};
    std::uint64_t timing_raw_count{};
    std::uint64_t timing_rejected_count{};
    float fade_indicator{1.0F};
    std::chrono::steady_clock::time_point demod_busy_started_at{};
    bool demod_busy_active{};
    std::chrono::steady_clock::time_point window_started_at{
        std::chrono::steady_clock::now()};

    // Sample-clock estimator and variable-rate resampler actuator.
    static constexpr std::size_t tau_history_n = 24;
    static constexpr std::size_t tau_history_min = 16;
    TimingSlopeTracker timing_tracker;
    double last_windowed_timing{};
    double smoothed_sample_clock_ppm{};
    std::array<double, tau_history_n> tau_history{};
    std::array<std::uint64_t, tau_history_n> tau_sample_history{};
    std::array<double, tau_history_n> tau_interval_correction_history{};
    std::size_t tau_history_head{};
    std::size_t tau_history_count{};
    std::optional<std::uint64_t> last_timing_sample_position;
    double last_windowed_cir_avg{};
    double window_cir_offset_sum{};
    int applied_cir_offset{};
    double cir_confidence{};
    float acquisition_time_ms{};
    float fft_plan_time_ms{};

    explicit DemodRuntimeState(const std::uint64_t generation)
        : demod_generation(generation) {}
};
