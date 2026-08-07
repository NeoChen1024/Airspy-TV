# Remaining Architecture and Soundness Work

Date: 2026-08-06
Maintained against: current `dev` worktree

## Status

This maintained copy lists only unresolved work. Findings corrected after the
original review have been removed rather than retained as an implementation
history. The remaining work is regression coverage, portability, and
file-level cleanup rather than a known unsafe data-flow design.

## Remaining findings

### High: regression coverage is still narrower than the supported behavior

The current tests cover the main 8K stream path, reset/reopen and rapid-retune
shapes, inner/outer FEC, pipeline-load telemetry, EPG, and recorder byte-rate
tracking. They do not yet cover several important contracts:

- 2K end-to-end decoding;
- every guard-interval and automatic/manual mode transition;
- continuous-resampler equivalence over random block boundaries and rate
  changes;
- repeated concurrent submit/reset/flush/stop stress;
- analyzer reset/submit stress under ThreadSanitizer;
- portable non-AVX Viterbi as a regular CI configuration;
- mpv queue hysteresis and discontinuity ordering with a fake reader;
- recorder short-write, filesystem-error, and shutdown behavior;
- PAT/PMT/SDT version changes and malformed-section handling;
- long synthetic SRO/CFO fixtures as retained opt-in regressions.

Priority should go to lifecycle stress, 2K coverage, and fake-reader playback
tests because these protect the broadest behavior during further refactoring.

### Medium: full runtime and sanitizer validation remains incomplete

The existing optimized, assertion-enabled, portable, and targeted TSAN runs
are useful, but the following validation is still needed after substantial
pipeline or GUI changes:

- a complete ASan/UBSan/LSan run;
- longer repeated TSAN lifecycle stress rather than only deterministic cases;
- a fresh full 363.9 GB 545 MHz replay after major DSP changes;
- 557/581 MHz multipath replay;
- live Airspy and SoapySDR start/stop/retune testing;
- interactive GUI and mpv playback/recording testing.

Long-capture timing requirements remain in
[clock-tracking.md](clock-tracking.md) rather than in this architecture review.

### Medium: build defaults are optimized for this workstation, not portability

The following policies remain deliberate but should be isolated or exercised
in CI:

- default Debug uses `-O3 -DNDEBUG` unless `AIRSPY_TV_OPTIMIZED_DEBUG=OFF`;
- `AIRSPY_TV_NATIVE_ARCH=ON` adds `-march=native`;
- non-AVX builds select a different ViterbiDecoderCpp backend and need regular
  coverage.

Recommended direction:

1. define explicit optimized-debug, assertion-debug, portable-release, and
   sanitizer presets;
2. make host-native optimization an explicit release/profile choice for
   distributable builds;
3. keep SIMD and scalar Viterbi paths buildable until portable coverage is
   routine.

### Medium: GUI code is type-separated but still physically concentrated

The GUI no longer owns a concrete decoder pointer, and common panels are
separated from DVB-T state. However, `src/main_gui.hpp` still contains most
application state and panel implementations in one large include file.

Further cleanup should move, without changing behavior:

- common source and receiver controls;
- spectrum/waterfall and common signal-quality panels;
- DVB-T settings and diagnostics;
- playback, EPG, and recorder panels;
- shared ImGui formatting helpers.

Prefer ordinary translation units over more large inline functions. Keep
`ReceiverSession` as the only owner of source start/restart/retune and
demodulator replacement while splitting the files.

### Low: demodulator APIs contain transitional overlap

`DemodulatorStats` remains alongside the newer `SignalSnapshot` and
`PipelineSnapshot`, although the common GUI now uses the snapshots. Before
adding another standard:

- decide whether `DemodulatorStats` still has a non-GUI consumer;
- remove it if the common snapshots fully replace it;
- keep standard-specific parameters and diagnostics in separate typed state;
- avoid a universal parameter structure containing fields for multiple
  standards;
- add the next standard through the `ReceiverSession` factory/transaction and
  common snapshot contracts.

### Low: long-lived diagnostics need explicit retention policy

User-visible and fatal errors remain unconditional; decoder diagnostics, event
logs, and FEC traces now share the `--debug` flag. These verbose logs can still
produce very large files during clock research. Define which measurements
should also be emitted as compact, machine-readable long-run regression
summaries, and rate-limit repeated human-readable events where appropriate.

This should reduce ad-hoc log parsing without removing the detailed data needed
for AFC and FEC investigations.

## Recommended order

1. Add 2K and lifecycle/playback regression coverage.
2. Add build presets and routine portable/sanitizer configurations.
3. Split `main_gui.hpp` along the existing common versus standard-specific
   boundaries.
4. Remove redundant transitional APIs after confirming there are no remaining
   consumers.
5. Perform full-capture and live-hardware validation after major DSP or source
   changes.

## Refactoring acceptance criteria

Future cleanup should preserve:

- bounded live queues and blocking offline backpressure;
- persistent worker pools and ordered joins before stateful consumers;
- one owner for demodulator lifetime and same-instance retune;
- serialized outer-FEC and transport output;
- generation-safe reset and stale-output suppression;
- production-path signal telemetry with an optional pre-lock monitor;
- standard-neutral common GUI state and typed per-standard diagnostics;
- current throughput, TS continuity, queue latency, and clock-tracking
  baselines.
