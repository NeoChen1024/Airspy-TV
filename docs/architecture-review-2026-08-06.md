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
tracking. An opt-in GNU Radio validator also streams synthetic 2K/8K input
through stdin and checks exact recovered TS packet identity. Several important
contracts remain outside the routine suite:

- the full 2K/8K, guard-interval, bandwidth, modulation, and code-rate matrix;
- automatic/manual mode transitions;
- continuous-resampler equivalence over random block boundaries and rate
  changes;
- repeated concurrent submit/reset/flush/stop stress;
- analyzer reset/submit stress under ThreadSanitizer;
- portable non-AVX Viterbi as a regular CI configuration;
- mpv queue hysteresis and discontinuity ordering with a fake reader;
- recorder short-write, filesystem-error, and shutdown behavior;
- PAT/PMT/SDT version changes and malformed-section handling;
- long synthetic SRO/CFO fixtures as retained opt-in regressions.

Priority should go to making the synthetic mode matrix routine, lifecycle
stress, and fake-reader playback tests because these protect the broadest
behavior during further refactoring.

### Medium: extended runtime and sanitizer validation remains incomplete

The full current CTest suite passes under the checked-in ASan/UBSan/LSan and
standalone TSan presets. The following longer or hardware-dependent validation
is still needed after substantial pipeline or GUI changes:

- longer repeated TSAN lifecycle stress rather than only deterministic cases;
- a fresh full 363.9 GB 545 MHz replay after major DSP changes;
- 557/581 MHz multipath replay;
- live Airspy and SoapySDR start/stop/retune testing;
- interactive GUI and mpv playback/recording testing.

Long-capture timing requirements remain in
[clock-tracking.md](clock-tracking.md) rather than in this architecture review.

### Medium: portable and sanitizer profiles are not yet exercised in CI

`CMakePresets.json` now defines optimized-debug, assertion-debug,
portable-release, ASan/UBSan/LSan, and standalone TSan configure/build/test
profiles. The remaining gap is routine CI execution across these deliberate
policy choices:

- default Debug uses `-O3 -DNDEBUG` unless `AIRSPY_TV_OPTIMIZED_DEBUG=OFF`;
- `AIRSPY_TV_NATIVE_ARCH=ON` adds `-march=native`;
- non-AVX builds select a different ViterbiDecoderCpp backend and need regular
  coverage.

Recommended direction:

1. exercise portable-release and both sanitizer profiles in routine CI;
2. keep host-native optimization confined to explicit local performance
   profiles and use portable-release for distributable builds;
3. retain both SIMD and scalar Viterbi build coverage.

## Recommended order

1. Make the synthetic DVB-T matrix routine and add lifecycle/playback coverage.
2. Add routine CI execution for portable and sanitizer presets.
3. Perform full-capture and live-hardware validation after major DSP or source
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
- no GUI implementation headers included into `main.cpp` or another source;
- SDL/OpenGL/ImGui teardown ordering and GUI-thread-only rendering;
- current throughput, TS continuity, queue latency, and clock-tracking
  baselines.
