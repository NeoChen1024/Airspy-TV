# TODO

## Continuous DVB-T pipeline correctness

- [x] Reset all per-stream front-end state on every reset, retune, and source
  replacement. (`938e5b1`/`e51c7eb`: `reset_frontend_state()` now clears the
  fade-recovery grid and is called by the reset handler; a retune clears the
  grid and the TPS-fixed mode/guard.)  In particular, clear the saved fade-recovery carrier grid
  (`stable_carrier_offset` / `stable_phase`) as well as the CFO, continual
  reference, pilot phase, TPS state, and related counters.  A grid captured
  from one frequency or file must never be restored in a later stream.

- [x] Make an event-driven re-anchor an explicit symbol-loop boundary.
  (`e51c7eb`: a successful mid-symbol acquisition discards the in-flight
  symbol and restarts from the published `next_symbol_start`.)  If
  acquisition changes the grid while a symbol is being processed, discard that
  in-flight symbol and restart from the newly published `next_symbol_start`.
  Do not combine the old symbol's payload/NCO advancement with the newly
  anchored grid.

- [x] Implement a closed-loop sample-clock / timing correction path.
  (`938e5b1`: the pilot phase-slope drift between stats windows feeds a
  fractional timing accumulator that nudges the symbol period +/-1 sample;
  bounded +/-4 samples, reset on grid rebuild/re-anchor/reset. The absolute
  tau is multipath-biased and must not be used as a P-loop error.)  The
  pilot phase-slope estimate is currently diagnostic-only while symbol starts
  advance by a fixed integer period.  Feed a filtered timing-error estimate
  into a fractional timing accumulator and resampler-rate or interpolator
  control, with bounded corrections and reset/reacquisition behaviour.

- [x] Split TPS state into `ever_locked` and `currently_valid` (or equivalent
  confidence/state fields).  Fixed parameters after an initial valid TPS lock
  are acceptable, but a failed later BCH/sync check must not be reported as a
  currently healthy TPS lock.

## Pipeline pacing and playback

- [x] Make 0.2 seconds the common ingestion and jitter budget.  Keep large
  file reads if they improve I/O efficiency, but split them into
  `sample_rate / 5` complex-sample spans before `submit_blocking()`.  The
  historical 0.7-second `processing_chunk_samples` constant was removed;
  `StreamDecoder::chunk_samples_for(rate)` derives the span from the sample
  rate so chunking never defines DSP latency or queue behaviour.

- [x] Size the resampled sample ring from the active DVB-T baseband rate so it
  also retains approximately 0.2 seconds.  The ring grows at run time to
  ~0.2 s of the input rate (the baseband is always <= input), floored at the
  proven 1 Mi behaviour; it only resizes while empty so the absolute
  read/write counters never corrupt the wrap.

- [ ] Define and propagate transport discontinuity semantics to playback.
  FEC-region resets, source drops, and retunes must be distinguishable from
  ordinary TEI-marked packets, and the libmpv path needs a controlled recovery
  policy rather than only dropping old packets from a bounded queue.

- [ ] Add long-running playback telemetry and tests: selected-service PCR,
  audio/video PTS/DTS monotonicity, playback queue depth, decoder stalls, and
  measured A/V offset.  Preserve broadcast timestamps; distinguish genuine
  clock drift from RF loss or an intentional live catch-up drop.

## Verification and documentation

- [ ] Add deterministic `StreamDecoder` integration tests using synthetic
  DVB-T I/Q.  Cover arbitrary input block boundaries, 2K and 8K modes,
  sample-clock offset, CFO, deep fades/re-anchors, reset followed by a new
  stream, finite-stream flush, and a non-acquirable stream that must not
  deadlock.  Assert transport continuity and expected TS output where the
  fixture is error-free.

- [ ] Bring `docs/worker-pools-and-dataflow.md` in sync with the implemented
  architecture.  It still describes front-end-owned periodic acquisition,
  obsolete fade timing, and an outdated TPS pending-buffer length.
