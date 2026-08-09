# Roadmap

Airspy TV is a native SDR television receiver. DVB-T is the implemented
standard; the current priority is to harden that receiver and its playback
path before expanding to other standards.

This document intentionally tracks unfinished work. Detailed timing research
is in [docs/clock-tracking.md](docs/clock-tracking.md), current pipeline
mechanics are in
[docs/worker-pools-and-dataflow.md](docs/worker-pools-and-dataflow.md), and
cross-cutting cleanup is in
[docs/architecture-refactor-review-2026-08-09.md](docs/architecture-refactor-review-2026-08-09.md),
while remaining validation gaps are maintained in
[docs/architecture-review-2026-08-06.md](docs/architecture-review-2026-08-06.md).

## Current baseline

- Airspy, SoapySDR, CS16 file, and stdin inputs are separated behind the
  `IqSource` boundary and feed the same receiver pipeline. Source workers own
  backend handles and pacing; source-only `SdrDevice`, `ReceiverPipeline`, and
  `TransportPipeline` have separate ownership.
- The DVB-T path supports 2K/8K, guard intervals 1/4 through 1/32, QPSK,
  16-QAM, 64-QAM, and all non-hierarchical convolutional code rates.
- Continuous resampling, OFDM acquisition/tracking, symbol workers,
  ViterbiDecoderCpp workers with a portable scalar fallback, outer FEC, and
  MPEG-TS output are integrated in `dvbt::StreamDecoder`. `FecStage` now owns
  its bounded queue, worker, decoder session, and transport counters.
  `SampleChannel` owns the generation-aware input queue, absolute sample ring,
  and reset handshake, while `ClockControlTimeline` owns SRO/CFO scheduling and
  output-to-input coordinate mapping. `FrontendStage` owns conversion,
  bootstrap acquisition, and resampling; `DemodStage` owns OFDM tracking,
  symbol postprocessing, statistics, and telemetry. `StreamDecoder::Impl`
  retains lifecycle and external callback wiring.
- Playback, TS recording, service discovery, now/next EPG, and common signal
  and pipeline snapshots run in process. Every transport consumer owns an
  independent bounded queue: 8 MiB for mpv/RTP/live CLI, 24 MiB for the TS
  recorder, and 256 KiB for each metadata observer. Offline exact output uses
  a separate 24 MiB blocking queue.
- `ReceiverSession` coordinates source and demodulator lifecycle for GUI, live
  CLI, and offline decoding. File pacing and decoder backpressure are explicit
  policies: live paths are realtime and may drop when busy, while offline
  decoding is unpaced and blocks for exact output. `DecodeRunReporter` provides
  the common source-session, telemetry-drain, and report-finalization lifecycle.
- Common GUI panels use standard-neutral snapshots; DVB-T-only state remains
  typed and separate.
- GUI and CLI live decoding share bounded TS output and IPv4/IPv6 RTP/UDP
  streaming paths.
- Machine-readable decode reports and the validation runner cover all 480 ideal
  DVB-T parameter combinations under portable Release and ASan/UBSan, plus a
  deterministic all-pairs TSan set and optional real-signal corpora.
- Machine-readable reports include homogeneous per-output queue/counter
  telemetry and per-source/run sink totals. Common stream routing, DVB-T JSON
  encoding, and report lifecycle have separate owners.
- GUI runtime state is grouped by responsibility; panels read one per-frame
  snapshot and invoke source/report/output transitions through a headless
  controller.

The baseline is functional, not feature-complete. The remaining work is
primarily targeted lifecycle and playback regression coverage,
difficult-channel validation, transport/playback observability, retained
real-signal baselines, application maintainability, and new-standard DSP.

## 1. Regression coverage

### StreamDecoder integration

- Cover automatic and forced transmission-mode transitions.
- Exercise initial timing offset, positive and negative sample-clock offset,
  sample-clock drift, initial CFO, and LO drift. Keep long loop experiments in
  the AFC test set rather than slowing the default unit suite.
- Add deep fades, MER-gate entry/exit, phase-only relock, and long-fade
  re-anchor cases.
- Stress concurrent submit/reset/flush/stop and repeated retune sequences under
  parallel worker load.

### Playback, recorder, and transport

- Test mpv queue hysteresis and discontinuity ordering with a fake reader. The
  current playback queue is 8 MiB, enters buffering at 1 MiB, and resumes at
  2 MiB.
- Add deterministic partial-write injection for the recorder writer loop. Open
  failures, terminal write failures, queue overflow, and pending-data drain are
  already covered.
- Add PAT/PMT/SDT version-change and malformed-section regressions.
- Add retained multi-hour file and live playback tests with bounds for A/V
  offset, queue growth, decoder stalls, and intentional drop-old recovery.

Further cleanup and validation details are maintained in the architecture
reviews rather than duplicated here. The report, GUI-state, source, and
transport-observer boundaries are complete; follow-up should be driven by
measured regressions or a concrete second receiver implementation.

## 2. DVB-T completeness and channel robustness

### Reference fixtures

- Establish accepted per-capture thresholds for usable TS packets, RS/TEI
  counts, and processing ratio in the existing real-signal corpus.
- Add impaired 2K and short-guard fixtures; the ideal parameter matrix already
  covers their configuration paths.
- Keep weak, multipath, discontinuous, and SFN-like captures separate from the
  exact-match synthetic baseline.

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

## 4. Live hardware validation

- Run long start/stop/retune/recovery sessions on Airspy R2 and Mini.
- Select representative SoapySDR devices and verify rate negotiation, overflow
  recovery, frequency correction, gain control, and shutdown.
- Run the full 545 MHz recording with `--include-long-real-signals` and repeat
  the 557/581 MHz corpus after major timing, FEC, resampler, or queue changes.

## 5. Multi-standard expansion

The public demodulator and snapshot contracts, shared receiver lifecycle, and
extracted `IqSource` backends are a sufficient starting point for another
digital standard. `ReceiverSession` may keep explicit typed per-standard
configuration and telemetry access; a small switch or variant is preferable to
a pre-emptive plugin framework. Extract another shared boundary only after a
second implementation reveals concrete duplication. Standard-specific
parameters and diagnostics must remain typed rather than growing a universal
mode structure.

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

1. Add lifecycle, playback-queue, and SI-table regressions around the current
   source, transport, and report boundaries.
2. Establish real-signal thresholds and rerun the synthetic, sanitizer, and
   retained-capture gates after each affected phase.
3. Complete the AFC bias-validation set and long-capture clock experiments.
4. Validate or replace the CIR estimator on long-delay/SFN input.
5. Add hierarchical DVB-T, remaining TPS metadata, and timestamp/playback
   diagnostics.
6. Begin the first second-standard implementation using the existing common
   lifecycle and extract additional shared code only where duplication appears.

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
