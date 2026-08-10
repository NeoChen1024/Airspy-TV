# Remaining Architecture and Soundness Work

Date: 2026-08-06
Maintained against: current worktree

## Status

This maintained copy lists only unresolved work. Findings corrected after the
original review are removed rather than retained as implementation history.
The remaining work is extended runtime baselines and hardware validation
rather than a known unsafe data-flow design.

## Remaining findings

### Medium: extended runtime and hardware validation remains incomplete

Real-signal corpus results do not yet have enforced regression thresholds. The
remaining longer or hardware-dependent validation is:

- longer repeated TSan lifecycle stress rather than only deterministic cases;
- accepted per-capture baselines for usable TS packets, RS/TEI counts, and
  processing ratio so real-signal runs are evaluated rather than reported as
  `not_evaluated`;
- a fresh full 363.9 GB 545 MHz replay after major DSP changes;
- live Airspy and SoapySDR start/stop/retune testing;
- interactive GUI and mpv playback/recording testing.

Long-capture timing requirements remain in
[clock-tracking.md](clock-tracking.md) rather than in this architecture review.

## Recommended order

1. Establish real-signal thresholds for the retained capture corpus.
2. Perform full-capture and live-hardware validation after major DSP or source
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
