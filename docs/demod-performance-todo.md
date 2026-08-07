# DVB-T Serial Demod Performance TODO

## Scope and baseline

The serial `dvbt-demod` thread is now the limiting pipeline stage during
offline decoding. This document tracks measured optimization work for that
thread. It does not change the ownership rule: carrier, timing, channel, and
TPS state remain symbol-ordered and owned by one demod thread.

The initial profile used a 20-second, 200,000,000-sample prefix of
`557mhz-horizontal.cs16` at 10 MS/s and 6 MHz channel bandwidth. A Release
build with the automatic 16-worker allocation produced 37,104,244 TS bytes,
197,363 packets, no RS failures, and no TEI packets. A repeated instrumented
run produced byte-identical TS. The percentages below are means over 30 steady
400-symbol windows, excluding startup and the final partial window. They are
shares of serial demod busy wall time, not whole-process CPU usage.

| Serial demod stage | Busy share |
| --- | ---: |
| Channel estimation | 31.9% |
| FFT and CFO processing | 26.5% |
| Pilot phase lock | 14.8% |
| Payload extraction and equalization | 11.2% |
| Sample-ring copy | 10.5% |
| Ordered 68-symbol postprocessor wait | 3.5% |
| Symbol submission | 1.0% |
| Output, analysis callback, and FEC enqueue | 0.4% |
| TPS processing | 0.1% |

Nested measurements identify the largest individual operations:

| Operation | Share of parent | Approximate demod share |
| --- | ---: | ---: |
| Scattered-pilot timing-slope estimate | 60.1% of channel | 19.2% |
| Pilot phase lock | top-level stage | 14.8% |
| FFTW execution | 50.4% of FFT/CFO | 13.3% |
| Time-domain NCO rotation | 46.9% of FFT/CFO | 12.4% |
| Channel interpolation | 23.2% of channel | 7.4% |
| Pilot inverse-channel estimates | 15.0% of channel | 4.8% |

The ordered symbol barrier and FEC enqueue are not the current bottleneck.
Splitting the stateful demod loop into another worker stage is therefore not a
priority until the serial DSP work below has been reduced and remeasured.

## Phase 1: behavior-preserving changes

These changes should retain exact decoded output and should be implemented and
measured one at a time.

### Exact median selection for timing slope

`estimate_scattered_timing_tau()` currently calculates about 568 adjacent
scattered-pilot slopes per 8K symbol and fully sorts them to obtain one median.

- replace the full sort with `std::ranges::nth_element()` for the upper median;
- for an even count, take the lower median as the maximum of the lower
  partition;
- retain `double`, the current slope samples, branch handling, and filtering;
- separately measure slope generation and median selection if the resulting
  gain is smaller than expected.

The selected values should be numerically identical to the current median.
Do not combine this change with pilot decimation or a new robust estimator.

### Contiguous sample-ring copy

The current symbol read performs one modulo operation per copied FFT sample.

- use `AbsoluteSampleRing::data()` and split a wrapping read into at most two
  contiguous `std::copy_n()` operations;
- keep the existing synchronization and absolute-position checks;
- minimize time under the coordinator mutex;
- verify symbol data and emitted TS remain byte-identical.

### Reusable demod scratch storage

Avoid repeated allocation where ownership does not leave the demod thread.

- retain the channel vector in `DemodRuntimeState` and overwrite every carrier
  before use;
- retain continual-carrier and temporary timing-estimate buffers where doing
  so does not change stream-reset behavior;
- measure allocation removal separately from arithmetic changes;
- do not reuse payload buffers through shared mutable ownership. A bounded
  buffer pool may be evaluated later because payload vectors move to symbol
  workers.

After Phase 1, rerun the detailed telemetry before choosing later work. A
combined 10--20% serial-demod reduction is plausible but is not an acceptance
criterion; measured end-to-end throughput and output correctness decide.

## Phase 2: tracking-aware changes

These changes alter when or how tracking work is evaluated. They require the
full difficult-channel and clock-drift validation matrix.

### Expected pilot-phase fast path

With a valid grid, pilot phase should normally advance as
`(previous_phase + 1) % 4`. The current fixed-offset verifier scores all four
phases every symbol.

- score the expected phase first with a normalized confidence measure;
- accept it only above a threshold established from retained captures;
- fall back to the current four-phase search on low confidence, discontinuity,
  cold start, or recovery;
- retain a periodic full-search sanity check if it catches otherwise silent
  phase errors;
- preserve fade freeze, stable-grid capture, phase-discontinuity telemetry,
  and event-driven re-anchor behavior.

The threshold must not make a marginal channel remain confidently locked to a
wrong phase.

### Reduced timing-estimator cadence

Sample-clock drift is slow relative to the OFDM symbol rate. Evaluate updating
the expensive scattered-pilot slope estimate every two or four symbols while
using the last accepted filtered timing value for pilot de-rotation between
updates.

- use full-rate updates during cold lock, immediately after re-anchor, and
  while timing confidence is recovering;
- keep control-loop publication and sample-domain command scheduling
  deterministic;
- compare estimator noise, branch rejection, residual timing, SRO convergence,
  MER, and recovered TS packets;
- do not reduce cadence solely because displayed `tau` remains smooth.

Pilot subsampling inside one timing estimate is a separate experiment. If it
is attempted, compare 1/2 and 1/4 pilot sets against cadence reduction rather
than changing both dimensions together.

### SIMD NCO rotation

The scalar NCO recurrence prevents straightforward compiler vectorization.

- keep a portable scalar reference path;
- evaluate multi-lane oscillators using `step^W` for SSE4.1, AVX2/FMA, and
  AArch64 NEON kernels;
- retain bounded phase normalization and verify long-run phase error;
- dispatch by supported ISA in the same manner as other optimized DSP paths;
- compare MER, CFO telemetry, timing slope, and TS output because operation
  order will no longer be bit-identical.

## Phase 3: secondary work

### FFT planning

The OFDM plan currently uses `FFTW_ESTIMATE`.

- benchmark `FFTW_MEASURE` for 2K and 8K plans;
- account for planning latency and the fact that measured planning may modify
  its input buffer;
- preserve safe plan creation/destruction and stable buffer addresses;
- do not enable FFTW threading unless a complete pipeline benchmark shows a
  gain. One 2K/8K FFT per symbol is likely too small to amortize thread
  synchronization.

### Channel interpolation and payload storage

- evaluate incremental complex interpolation to avoid repeated division and
  integer-to-float conversion;
- evaluate a bounded payload/equalizer buffer pool only after allocation cost
  is measured independently;
- keep carrier order and equalizer-power correspondence exact.

## Validation requirements

Every optimization must report both the affected timing bucket and complete
decoder throughput. Retain the non-overlapping top-level demod buckets and mark
nested FFT/channel measurements as non-additive.

Minimum validation:

- Debug and Release unit/integration suites;
- clean `557mhz-horizontal`, `581mhz-2`, and `581mhz-3` captures;
- marginal/recovery `545M-DVB-T-公視-3`, `581mhz`, and
  `557mhz-vertical` captures;
- synthetic independent positive/negative SRO and CFO, linear drift, and
  reversal cases;
- 2K plus available guard-interval and bandwidth fixtures;
- repeated one-worker and automatic-worker runs for deterministic symbol and
  recovery boundaries.

Behavior-preserving changes require byte-identical TS on clean captures.
Tracking-aware changes must compare emitted packets, usable non-TEI packets,
RS/TEI counts, MER, timing confidence, branch rejection, phase
discontinuities, re-anchor positions, SRO command/applied error, and queue
pressure. A faster implementation is not acceptable if it changes recovery
position nondeterministically or reduces long-capture usable packet yield.

## Deferred targets

Do not prioritize the following based on the current profile:

- splitting the serial demod loop into another thread;
- TPS processing optimization;
- CIR-analysis optimization;
- postprocessor barrier changes;
- FEC queue or transport output changes;
- FFTW worker threads without a measured end-to-end gain.

Reconsider them only after a new profile shows that the bottleneck moved.
