# TODO

## Continuous DVB-T pipeline correctness

- [ ] Reset all per-stream front-end state on every reset, retune, and source
  replacement.  In particular, clear the saved fade-recovery carrier grid
  (`stable_carrier_offset` / `stable_phase`) as well as the CFO, continual
  reference, pilot phase, TPS state, and related counters.  A grid captured
  from one frequency or file must never be restored in a later stream.

- [ ] Make an event-driven re-anchor an explicit symbol-loop boundary.  If
  acquisition changes the grid while a symbol is being processed, discard that
  in-flight symbol and restart from the newly published `next_symbol_start`.
  Do not combine the old symbol's payload/NCO advancement with the newly
  anchored grid.

- [ ] Implement a closed-loop sample-clock / timing correction path.  The
  pilot phase-slope estimate is currently diagnostic-only while symbol starts
  advance by a fixed integer period.  Feed a filtered timing-error estimate
  into a fractional timing accumulator and resampler-rate or interpolator
  control, with bounded corrections and reset/reacquisition behaviour.

- [ ] Split TPS state into `ever_locked` and `currently_valid` (or equivalent
  confidence/state fields).  Fixed parameters after an initial valid TPS lock
  are acceptable, but a failed later BCH/sync check must not be reported as a
  currently healthy TPS lock.

## Pipeline pacing and playback

- [ ] Make 0.2 seconds the common ingestion and jitter budget.  Keep large
  file reads if they improve I/O efficiency, but split them into
  `sample_rate / 5` complex-sample spans before `submit_blocking()`.  Remove
  the historical 0.7-second `processing_chunk_samples` constant from the
  decoder API; it should not define DSP latency or queue behaviour.

- [ ] Size the resampled sample ring from the active DVB-T baseband rate so it
  also retains approximately 0.2 seconds.  The fixed 1 Mi-sample ring is only
  about 146 ms at 6 MHz and 109 ms at 8 MHz.

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
