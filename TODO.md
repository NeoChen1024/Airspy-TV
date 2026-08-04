# TODO

## Completed — pending review

The following work is implemented and awaits final review/acceptance. The
commit shown is the implementation reference; review should verify the code,
regressions, and the stated acceptance evidence before these items are
considered closed.

### Continuous DVB-T pipeline correctness

- [x] **Reset all per-stream front-end state** on every reset, retune, and
  source replacement. (`938e5b1` / `e51c7eb`)
  `reset_frontend_state()` clears the fade-recovery grid and is called by the
  reset handler; retune clears the grid and TPS-fixed mode/guard. This includes
  `stable_carrier_offset`, `stable_phase`, CFO, continual reference, pilot
  phase, TPS state, and related counters, so a grid from one frequency/file
  cannot be restored in a later stream.

- [x] **Make event-driven re-anchor an explicit symbol-loop boundary.**
  (`e51c7eb`) A successful mid-symbol acquisition discards the in-flight
  symbol and restarts from the published `next_symbol_start`; old payload/NCO
  advancement is never combined with the newly anchored grid.

- [x] **Implement closed-loop sample-clock/timing correction.**
  (`938e5b1`) Pilot phase-slope drift between statistics windows feeds a
  fractional timing accumulator that nudges the symbol period by +/-1 sample.
  The correction is bounded to +/-4 samples and reset on grid rebuild,
  re-anchor, and reset; the multipath-biased absolute tau is not used as a
  P-loop error.

- [x] **Split TPS lock state into `ever_locked` and `currently_valid`.**
  (`e51c7eb`) Parameters are fixed after the first valid lock, while failed
  later BCH/sync checks no longer report a currently healthy TPS lock.

### Pipeline pacing and playback

- [x] **Use a common 0.2-second ingestion/jitter budget.** (`c0e9f27`)
  Large file reads are split into `sample_rate / 5` complex-sample spans before
  `submit_blocking()`. The historical 0.7-second
  `processing_chunk_samples` constant was removed; chunk size is derived from
  the active sample rate.

- [x] **Size the resampled ring from the active rate.** (`c0e9f27`)
  The ring retains approximately 0.2 seconds of input/baseband data, has a
  proven 1 Mi floor, and resizes only while empty so absolute read/write
  counters remain wrap-safe.

- [x] **Define and propagate transport discontinuity semantics.**
  (`87e912b`) `TransportDiscontinuity` events (`fec_region_reset`,
  `stream_end`, `retune`) are out-of-band and distinct from per-packet TEI.
  `MpvPlayer::on_discontinuity()` restarts the demuxer on retune, plays the
  valid tail through EOF on stream end, and leaves FEC-region concealment to
  the demuxer. The bounded queue drop-old fallback remains for a stalled
  player.

- [x] **Add initial playback telemetry.** (`87e912b`)
  `MpvPlayer::telemetry()` samples libmpv playback time, A/V offset, dropped
  frames, pause state, TS queue depth, and decoder discontinuity count; these
  are displayed in the GUI Playback panel.

### Verification and documentation

- [x] **Complete the first 8K StreamDecoder integration-test slice.**
  (`b5d325a`) Added a deterministic synthetic 8K DVB-T signal generator and
  end-to-end CTest coverage for 8K/GI-1/4/QPSK/1/2, persistent resampling,
  arbitrary CS16 input block boundaries, OFDM acquisition/demodulation/FEC/TS
  output, finite-stream flush, reset followed by a new stream, retune and
  stream-end events, and a zero-I/Q non-acquirable stream that must not
  deadlock. The test also verifies FFT/guard selection and queue drain.
  Verification: CTest 4/4 and 10 repeated integration runs.

- [x] **Synchronize architecture documentation.** (`87e912b`)
  `docs/worker-pools-and-dataflow.md` and `ROADMAP.md` document event-driven
  acquisition, phase-only re-lock, TPS fix-once, closed-loop sample clock,
  0.2-second ingestion/rate-sized ring, deterministic recovery, deadlock
  fixes, discontinuity semantics, and playback telemetry.

## Remaining implementation

### StreamDecoder integration coverage

- [ ] **Extend the synthetic StreamDecoder integration test beyond the 8K
  clean/lifecycle slice.** The remaining matrix is:
  - 2K mode and its guard intervals;
  - explicit sample-clock/timing offset and slow clock drift;
  - carrier-frequency offset and residual CFO convergence;
  - deep fades, MER gating, phase-only re-lock, and long-fade re-anchor;
  - transport continuity and exact TS output where the synthetic fixture is
    error-free, rather than only a known post-warmup TS run;
  - repeated block-boundary/reset combinations under parallel worker load.

### Playback telemetry follow-up

- [ ] Add PCR/PTS/DTS monotonicity and discontinuity tracking inside the TS
  parser, including timestamp-wrap handling.
- [ ] Add multi-hour live/file playback regressions with explicit bounds on
  sustained A/V offset, queue growth, decoder stalls, and intentional live
  catch-up drops.

### Performance and hardware coverage

- [ ] Add long-running live-reception regressions for Airspy R2 and selected
  SoapySDR devices.
- [ ] Profile a modern AVX2 Viterbi implementation; libcorrect's SSE decoder
  remains the dominant CPU hotspot after the ordered-pipeline optimizations.
