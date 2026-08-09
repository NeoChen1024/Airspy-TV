// State carried exclusively by the demod thread between OFDM symbols.
// Continuous front-end tracking state owned by the demod thread and
// carried for the life of a stream. The resampled symbol stream is
// contiguous (no overlap rewind), so the residual CFO loop, centered pilot
// validation,
// continual-carrier reference, AND TPS superframe decoder all carry
// continuously; they are re-seeded only on cold starts (mode/guard
// changes or resets). The TPS carry is the key weak-signal win: with the
// old chunked pipeline every chunk re-locked TPS from scratch (~68
// symbols) and the per-chunk CFO boundary overshoot perturbed tracking.
struct OfdmTrackingState {
    bool valid{false};
    TransmissionMode mode{TransmissionMode::k8};
    GuardInterval guard{GuardInterval::gi_1_4};
    std::size_t fft_size{};
    std::size_t guard_size{};
    float residual_phase_ema{0.0F};
    int carrier_offset{std::numeric_limits<int>::max()};
    // Values captured while the tracking was last healthy; a fade never
    // moves the carrier grid (the LO is stable), so a cold re-anchor
    // restores these instead of re-running the ambiguous wide pilot lock.
    int stable_carrier_offset{std::numeric_limits<int>::max()};
    int stable_phase{-1};
    std::vector<std::complex<float>> previous_continual;
    std::vector<std::complex<float>> current_continual;
    bool have_previous_continual{};
    int previous_phase{-1};
    std::uint64_t phase_discontinuities{0};
    // Absolute stream position of the most recent processed symbol; the
    // CFO loop only updates from contiguous symbol pairs (start ==
    // last_symbol_start + period), so the first symbol after a re-anchor
    // is skipped exactly like the first symbol of the old chunks.
    std::uint64_t last_symbol_start{0};
    bool just_seeded{false};
    TpsDecoder tps_decoder;
    TpsSnapshot tps_snapshot;
    // CIR / delay-spread estimation for adaptive FFT-window placement.
    // Updated once per TPS frame (68 symbols) from the scattered pilots,
    // so the added work is negligible; `cir_offset` is the smoothed FFT
    // window offset (samples, <= 0) relative to the effective symbol
    // start (start_pos + guard_size), sliding the window toward the
    // latest strong tap when the delay spread leaves guard margin.
    std::vector<std::complex<float>> cir_grid;
    std::vector<std::complex<float>> cir_response;
    std::vector<double> cir_energy;
    FftwfPlan cir_plan;
    std::size_t cir_n{0};
    float cir_offset{0.0F};
    int cir_symbol_count{0};
};
