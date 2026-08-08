# Roadmap

Airspy TV is a native SDR television receiver. DVB-T is the implemented
standard; the current priority is to harden that receiver and its playback
path before expanding to other standards.

This document intentionally tracks unfinished work. Detailed timing research
is in [docs/clock-tracking.md](docs/clock-tracking.md), current pipeline
mechanics are in
[docs/worker-pools-and-dataflow.md](docs/worker-pools-and-dataflow.md), and
cross-cutting cleanup is in
[docs/architecture-review-2026-08-06.md](docs/architecture-review-2026-08-06.md).

## Current baseline

- Airspy, SoapySDR, and CS16 file sources feed the same asynchronous
  `Demodulator` boundary.
- The DVB-T path supports 2K/8K, guard intervals 1/4 through 1/32, QPSK,
  16-QAM, 64-QAM, and all non-hierarchical convolutional code rates.
- Continuous resampling, OFDM acquisition/tracking, symbol workers,
  ViterbiDecoderCpp workers with a portable scalar fallback, outer FEC, and
  MPEG-TS output are integrated in `dvbt::StreamDecoder`.
- Playback, TS recording, service discovery, now/next EPG, and common signal
  and pipeline snapshots run in process.
- `ReceiverSession` owns source and demodulator lifecycle. Common GUI panels
  use standard-neutral snapshots; DVB-T-only state remains typed and separate.

The baseline is functional, not feature-complete. The remaining work is
primarily broader regression coverage, difficult-channel validation,
transport/playback observability, portability, and new-standard DSP.

## 1. Regression coverage

### StreamDecoder integration

- Run the checked-in streaming 2K end-to-end validator routinely and extend it
  across every guard interval, bandwidth, modulation, and code rate.
- Cover automatic and forced transmission-mode transitions.
- Verify continuous-resampler equivalence across random input block boundaries
  and supported rate changes.
- Exercise initial timing offset, positive and negative sample-clock offset,
  sample-clock drift, initial CFO, and LO drift. Keep long loop experiments in
  the AFC test set rather than slowing the default unit suite.
- Add deep fades, MER-gate entry/exit, phase-only relock, and long-fade
  re-anchor cases.
- Require exact TS continuity and byte identity for error-free fixtures after
  the defined trellis warm-up.
- Repeat block-boundary, flush, reset, retune, source-reopen, and stop sequences
  under parallel worker load.

### Playback, recorder, and transport

- Test mpv queue hysteresis and discontinuity ordering with a fake reader. The
  current playback queue is 8 MiB, enters buffering at 1 MiB, and resumes at
  2 MiB.
- Test recorder short writes, filesystem errors, queue overflow, and shutdown
  while data is pending.
- Add PAT/PMT/SDT version-change and malformed-section regressions.
- Add retained multi-hour file and live playback tests with bounds for A/V
  offset, queue growth, decoder stalls, and intentional drop-old recovery.

### Runtime and portability

- Run full ASan/UBSan/LSan validation after substantial pipeline changes.
- Extend ThreadSanitizer lifecycle stress beyond deterministic short cases.
- Exercise portable non-AVX builds regularly so the scalar Viterbi backend does
  not silently regress.
- Maintain explicit optimized-debug, assertion-debug, portable-release, and
  sanitizer build configurations.

Further cleanup and validation details are maintained in the architecture
review rather than duplicated here.

## 2. DVB-T completeness and channel robustness

### Reference fixtures

- Add retained 5, 7, and 8 MHz reference fixtures; the strongest current
  regression coverage remains centered 6 MHz.
- Add difficult 2K and short-guard fixtures, not only ideal 8K/GI-1/4 input.
- Keep weak, multipath, discontinuous, and SFN-like captures separate from the
  clean functional baseline.

### TPS and hierarchical transmission

- Validate TPS loss and sliding reacquisition under isolated bit errors,
  bursts, phase slips, and mode changes. Add hysteresis only if those tests
  show unstable user-visible lock state.
- Assemble the complete superframe/cell-ID information instead of exposing
  only the currently decoded cell-ID byte.
- Implement hierarchical QAM alpha 1/2/4, independent HP/LP code rates, and an
  explicit HP/LP stream-selection policy.

### Multipath and SFN behavior

- Validate adaptive FFT-window placement on captured long-delay/SFN signals.
- Replace or augment the current scattered-pilot CIR spread estimate when
  echoes exceed its unaliased delay range or form separated clusters.
- Compare candidate energy-CDF or multi-cluster estimators without allowing
  noisy CIR movement to bias the sample-clock loop.
- Retain hard-versus-soft comparisons only where they help explain weak-signal
  or multipath behavior; the production path remains soft-decision decoding.

### Clock tracking

Complete the timing-coordinate bias checks, confidence gating experiments,
fractional timing experiments, second-order clock model, and independent
sample/LO drift matrix in
[docs/clock-tracking.md](docs/clock-tracking.md). Any adopted change must retain
the current long-capture TS continuity and queue-latency baseline.

## 3. Transport stream and playback

- Track PCR, PTS, and DTS monotonicity and discontinuities, including timestamp
  wrap handling, so RF loss, decoder stalls, and A/V clock drift can be
  distinguished automatically.
- Add EIT schedule table assembly (0x50-0x5F), including segment handling.
- Add multilingual service/event descriptors, parental ratings, subtitles,
  teletext, and alternate audio/language selection as demand requires.
- Preserve the distinction between corrupt packets (TEI), FEC-region resets,
  retunes, and finite stream end. New sinks must not infer these events from a
  temporarily empty byte queue.
- Keep full-MPTS recording independent from selected-service playback.

## 4. Live hardware validation

- Run long start/stop/retune/recovery sessions on Airspy R2 and Mini.
- Select representative SoapySDR devices and verify rate negotiation, overflow
  recovery, frequency correction, gain control, and shutdown.
- Repeat the 545 MHz full replay and 557/581 MHz multipath captures after major
  timing, FEC, resampler, or queue changes.
- Retain machine-readable summaries for lock, MER, BER, RS/TEI counts, TS
  bytes, processing ratio, queue watermarks, dropped blocks, timing/SRO, and
  CFO.

## 5. Multi-standard expansion

The application boundary is ready for another digital standard:
`ReceiverSession` replaces a `Demodulator`, `SdrDevice` supplies I/Q and routes
MPEG-TS, and common GUI panels consume `SignalSnapshot` and
`PipelineSnapshot`. Standard-specific parameters and diagnostics must remain
typed rather than growing a universal mode structure.

### DVB-T2

DVB-T2 requires a separate OFDM/signalling chain and LDPC+BCH FEC. Reuse should
stop at source/session infrastructure, common telemetry contracts, MPEG-TS
routing, SI/EPG, recording, and playback. Do not add T2 branches inside the
DVB-T decoder.

### ATSC and analog television

ATSC requires an 8VSB receiver and an ATSC-specific PSIP branch; it may also
require hardware faster than the current 10 MS/s Airspy operating point.
NTSC/PAL/SECAM need a separate video-and-audio output family rather than the
MPEG-TS `Demodulator` contract. Both are deferred until the digital receiver
and lifecycle APIs are stable.

## Recommended implementation order

1. Add 2K, lifecycle, playback-queue, and portable-backend regressions.
2. Complete the AFC bias-validation set and long-capture clock experiments.
3. Validate or replace the CIR estimator on long-delay/SFN input.
4. Add hierarchical DVB-T and remaining TPS metadata.
5. Complete timestamp/playback diagnostics and multi-hour regressions.
6. Finish the architecture-review cleanup without changing data-flow
   invariants.
7. Begin the first second-standard implementation.

## Release gates

A receiver milestone should not be treated as complete until it has:

- deterministic clean-fixture TS output in Debug and Release builds;
- no stale output or deadlock across reset, retune, flush, and shutdown;
- bounded live queues and zero offline input drops;
- stable playback recovery after genuine decoder gaps;
- no unexplained regression on the retained 545/557/581 MHz captures;
- acceptable realtime margin on the reference host;
- portable and sanitizer coverage appropriate to the changed subsystem;
- current documentation with completed implementation history removed.
