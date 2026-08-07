# DVB-T Clock-Tracking Research Plan

## Status

The original long-run timing failure is fixed. On the 151.6-minute
`545M-DVB-T-公視-3.cs16` capture, the current decoder keeps residual timing
bounded, maintains MPEG-TS output, and no longer loses outer-FEC alignment when
the old implementation would let `tau` approach a branch boundary.

The production baseline now includes:

- one robust scattered-pilot timing estimate shared by tracking and pilot
  verification;
- branch unwrapping and outlier rejection;
- feedback-debiased physical timing and SRO telemetry;
- variable-rate resampler correction as the sole long-term SRO actuator;
- independent sample-clock and LO/CFO tracking;
- adaptive CIR-based FFT-window placement;
- synthetic sample-clock and LO drift generation and validation tools.

Implementation history and the detailed diagnosis are available in the git
history. This document tracks only remaining validation and experimental work.

## Regression baseline

Use the following as the reference before changing timing control:

- Full capture: `545M-DVB-T-公視-3.cs16`, 10 MS/s CS16, approximately 363.9
  GB and 151.6 minutes.
- Offline decoder: the native `StreamDecoder` through `--decode-iq`.
- Event logging: pass `--debug` to the receiver CLI.
- Synthetic tools:
  - `tools/generate_dvbt_fixture.py`
  - `tools/validate_dvbt_clock_drift.py`
- Independently configurable fixture impairments:
  - initial sample-clock offset and ppm/minute drift;
  - initial LO offset and Hz/minute drift.

The short synthetic regression proves fixture generation, independent SRO/CFO
measurement, and continuous TS output. It is not long enough to characterize
loop convergence: the SRO estimator needs roughly ten seconds of history, so
control-loop experiments require longer retained fixtures.

## Open validation work

### 1. CIR timing-coordinate rebasing

The timing coordinate uses the exact time-weighted mean of
`applied_cir_offset`, but the 545 MHz capture did not exercise CIR movement.
Remaining work:

- add a synthetic CIR move in the middle of a statistics window;
- test repeated and opposing CIR moves;
- validate captures with naturally changing CIR placement;
- verify the available guard-interval margin before and after each move.

Record raw `tau`, filtered timing, physical timing, residual drift, estimated
SRO, commanded/applied correction, and CIR offset/confidence. A stable non-zero
residual `tau` is acceptable; continuity and unbiased drift are the criteria.

### 2. Confidence-gated timing updates

Evaluate holding or reducing timing-loop gain when measurements are unreliable.
Candidate inputs are:

- accepted scattered-pilot coverage;
- timing and CIR confidence;
- channel consistency and fade indicator;
- MER and pre/post-Viterbi error indicators;
- recent branch-rejection rate.

Test fades, rapidly changing multipath, low MER, and genuine signal loss
separately. Confidence gating must not turn loss of signal into a permanently
frozen stale lock; normal acquisition must still take over.

### 3. Variable-rate SRO correction

The frontend uses the vendored common variable-ratio arbitrary resampler for
both nominal input-rate conversion and long-term SRO correction. This is the
only DVB-T sample-clock actuator; the former fractional accumulator and
integer FFT-window sample steps have been removed. Adaptive CIR placement
remains a separate channel-delay operation.

The resampler should combine the nominal input-to-DVB-baseband rate conversion
with a continuously adjustable fractional-delay polyphase FIR. Its input phase
increment is approximately:

```text
nominal_input_samples_per_output
    * (1 + sro_correction_ppm * 1e-6)
```

This resampler is a common source-tree DSP routine, not a DVB-T-specific
component. DVB-T is only the first consumer. The same implementation should be
usable by future DVB-T2, DVB-S/S2, and other broadcast demodulators, with each
standard supplying its own nominal input/output rate profile, bandwidth
constraints, filter requirements, and timing-loop command. Standard-specific
code must not depend on the resampler's internal phase, history, or worker
implementation.

The common implementation provides persistent FIR history, block-local Q32.32
phase, variable output counts, block-boundary target updates, a configurable
ppm/second slew limit, explicit reset behavior, output-range parallelism, and
portable/SSE4.1/AVX2-FMA/NEON kernels. Callers provide absolute passband and
stopband edges; the filter is automatically sized to a 16-tap boundary for an
80 dB Kaiser target. Focused tests cover arbitrary input segmentation,
worker-count equivalence, reset, steady-state gain, ratio quantization, bounded
rate changes, and end-to-end stopband tone sweeps for DVB-T 5/6/7/8 MHz
configurations at 10 MS/s.

The DVB-T controller uses a 0.5 ppm/second slew and a confidence gate of 0.75.
Once the timing history is ready, the demod thread schedules source SRO as
`nominal_output_per_input / (1 + sro_ppm * 1e-6)`. Commands are stamped with
the demod read position at which the completed timing-window estimate becomes
available, mapped back to the corresponding source input position, and
activated after a fixed sample-domain horizon. The horizon is at least 0.5
seconds and is increased when necessary to exceed the ring plus one bounded
frontend quantum. It therefore does not change with instantaneous queue
occupancy or offline replay speed.

Every source block carries a monotonic input-sample range and stream epoch.
The frontend records input/output resampler spans and the correction that
actually generated each span. Timing-history de-bias uses the weighted
correction over the exact output interval between measurements, rather than a
wall-clock snapshot of the frontend's current ratio. Arbitrary caller blocks
are split into at most 50 ms resampler quanta so generated-ahead data has a
provable bound. Adaptive CIR placement remains independent.

Twenty-second synthetic 8K/6 MHz tests established both signs. For targets of
about +0.189 and -0.189 ppm, applied correction reached about +0.157 and -0.157
ppm by the final window, residual drift fell from roughly 0.75 to 0.16 samples
per statistics window, cumulative integer timing shift stayed at zero, and
both runs produced 37,045,776 TS bytes with no RS failures or TEI packets.

A 30-second prefix of the real 545 MHz PTV capture produced byte-identical
55,929,248-byte transport streams with feedback on and off. Both paths saw the
same 27 RS/TEI packets in this marginal-MER segment. Feedback converged to
about +0.163 ppm, held cumulative integer timing shift at zero instead of 18
samples, and kept filtered timing near 15.8 samples. This is a smoke test, not
a substitute for the full 2.5-hour replay.

The full 151.6-minute replay decoded all 90,973,175,808 complex input samples
into 16,977,139,404 TS bytes and 90,303,933 packets. There was one initial
outer-FEC alignment lock and no later `align-miss`, bad lock, timing-branch
event, outer-FEC reset, or decoder exception. After timing-history warm-up,
filtered timing stayed between -6.052 and +17.249 samples, cumulative integer
timing shift remained exactly zero, and the SRO command ranged from about
+0.149 to +0.195 ppm. The median absolute command-to-applied error after the
first 30 seconds was 0.000175 ppm. TS output continued for the entire replay;
the longest interval with an unchanged TS counter in the 0.6-second telemetry
was about 0.8 seconds.

The replay reported 700 RS failures and 700 TEI packets. Of these, 27 were at
startup, 672 were concentrated in one approximately 48.2--49.4-second
marginal-MER burst, and only one occurred during the remaining two hours. A
same-build 60-second A/B replay covering that burst was deterministic: the
integer-actuator baseline reported 679 failures and resampler feedback reported
699, while both produced the same 595,284 packets and 111,913,392 bytes. Only
216 packets differed; 658 failed in both runs, with 21 baseline-only and 41
feedback-only failures. This localized 20-packet difference is not evidence of
a timing runaway and is small relative to the common failure burst. The
resampler path is therefore the production path, while marginal-signal
characterization remains useful for future loop tuning.

Additional same-build full-capture A/B replays produced the following results.
Packet and TEI counts in this table were measured directly from the emitted
188-byte transport streams so that they remain cumulative across demodulator
session resets.

| Capture | Baseline packets / TEI | Feedback packets / TEI | Result |
| --- | ---: | ---: | --- |
| `557mhz-horizontal` | 1,487,603 / 0 | 1,487,603 / 0 | Byte-identical; no bad lock or outer reset |
| `557mhz-vertical` | 724,527 / 84,932 | 723,700 / 112,582 | Severe loss/recovery stress case; 16 outer resets in both paths |
| `581mhz` | 973,084 / 1,537 | 973,084 / 1,558 | Same output count; feedback had 21 additional TEI packets |
| `581mhz-2` | 1,257,810 / 0 | 1,257,810 / 0 | Byte-identical; no bad lock or outer reset |
| `581mhz-3` | 1,437,707 / 26 | 1,437,707 / 26 | Byte-identical |

The clean captures show that feedback does not alter decoded output when the
channel has adequate margin. `581mhz` contains a deep disturbance and recovers
in both modes; feedback recorded 52 bad-lock events versus 55 in the baseline,
with one outer reset in each. `557mhz-vertical` has a median MER near 17 dB,
extended intervals below 18 dB, repeated reacquisition, and is not a valid
steady-state SRO measurement. It is nevertheless useful for validating command
hold, reset, and recovery behavior. The separate `557M-DVB-T.cs16` capture has
intrinsically insufficient MER to decode and is retained only as an expected
acquisition-failure fixture.

The A/B runs also exposed a diagnostics limitation: `transport_bytes` is
cumulative, while final CLI RS/TEI/packet counters reflect only the current FEC
session after a demodulator reset. Until those counters are made cumulative,
cross-session validation must inspect the emitted TS and event log rather than
the final summary alone.

A later repeated `557mhz-vertical` sweep exposed a separate scheduling issue:
the symbol pool previously delivered whatever ordered prefix happened to be
ready after each submit, so the asynchronous MER gate could reach its fourth
hopeless window at different demod symbol positions. With eight decoder
threads, identical replays varied by roughly 12,000 usable packets. The demod
now consumes a fixed 68-symbol ordered batch and the pool bounds all submitted
but unconsumed work. Repeated eight-thread runs are byte-identical at 707,049
emitted packets, 103,187 TEI packets, and 603,862 usable packets; the first
hopeless windows and re-anchor positions are also identical across one and
eight-thread allocations. Five- and seven-thread allocations form a second
deterministic numerical path with 605,277 usable packets, a remaining 0.23%
cross-configuration difference rather than a scheduling-dependent recovery
shift. The clean `557mhz-horizontal` output remains byte-identical across one
and eight threads and against the earlier reference stream.

Remaining integration work is to validate flush, retune, queue pressure,
ramping/reversing SRO, and 2K mode with variable-rate correction. The
short 557/581 MHz matrix is complete, but the marginal-signal FEC delta and
loss/recovery hold policy still need characterization.

Use a block-local Q32.32 phase accumulator for the scalar reference
implementation. The upper 32 bits identify the input sample within the current
work buffer and the lower 32 bits hold the fractional phase. Rebase the phase
whenever consumed input is discarded, retaining only the FIR context required
by future outputs. The DSP phase remains block-local, while a separate
`uint64_t` source timeline stamps each input block for deterministic control
scheduling, latency telemetry, dropout epochs, and input/output span mapping.
A practical polyphase bank may use 1024 or 4096 phases and derive its index,
plus optional interpolation, from the Q32.32 fractional field.

The SRO estimator and second-order clock model remain owned by the consuming
demodulator, while the arbitrary resampler remains a common actuator. The
interface should carry a standard-neutral target rate and SRO correction
command. Standard-specific timing movement remains available for acquisition
and channel-delay placement, but it must not integrate the same long-term SRO
estimate and double-correct the clock error.

Benchmark the portable, SSE4.1, AVX2/FMA, and NEON kernels on their supported
architectures, retaining the portable path as the numerical reference.
Preserve the current resampler worker-budget model only where parallel output
partitions provide a measured benefit.

Compare residual timing jitter, pilot phase modulation, MER, FEC errors, CPU
cost, output-rate error, block-boundary continuity, and queue pressure. Do not
adopt a fractional path merely because it makes the displayed `tau`
numerically closer to zero.

### 4. Second-order clock model

The current loop handles an approximately constant sample-clock offset but
does not explicitly estimate a changing drift rate. Prototype an opt-in model
with:

- timing phase: residual FFT-window error;
- timing frequency: sample-clock offset or long-term shift rate;
- optionally, a much slower thermal drift-rate state.

Updates must be confidence-gated and substantially slower than per-symbol OFDM
tracking. Compare it against the current loop on constant offsets, linear
ppm/minute ramps, reversals, and long real captures before considering a new
default.

### 5. Sample-clock and LO-clock relationship

Treat SRO and CFO as independent estimator states and independent control loops
without exception. Do not rely on a shared hardware reference and do not add
feed-forward between the loops. SDR front ends may be superheterodyne, low-IF,
zero-IF, direct-sampling, or include additional digital conversion stages, so
the observed relationship between LO error and sample-clock error is
hardware- and topology-dependent. A second-order drift model makes an assumed
cross-loop relationship still less reliable.

Continue generating independent, correlated, and deliberately conflicting
sample/LO drift fixtures, but use them to prove that each loop remains correct
when the other changes. Correlation is a test dimension, not a controller
input.

## Experiment matrix

Each new algorithm should cover at least:

| Dimension          | Cases                                                              |
| ------------------ | ------------------------------------------------------------------ |
| DVB-T mode         | 2K, 8K                                                             |
| Channel bandwidth  | supported 5/6/7/8 MHz cases where fixtures exist                   |
| Sample clock       | zero, positive/negative offset, linear ramp, reversal              |
| LO clock           | zero, positive/negative offset, linear ramp                        |
| Clock relationship | independent, correlated, conflicting                               |
| Channel            | clean synthetic, static multipath, moving CIR, fade, real captures |
| Input path         | deterministic offline replay; live SDR where practical             |

Retain machine-readable summaries for SRO error, CFO error, timing confidence,
branch rejections, actuator rate, MER, FEC failures, TS bytes, processing ratio,
queue watermarks, and dropped blocks.

## Recommended order

1. Complete 2K, 557/581 MHz, and synthetic CIR-bias validation.
2. Establish expected confidence and ppm ranges for those inputs.
3. Add confidence gating as an isolated control change.
4. Run the remaining 2K, queue-pressure, retune, and ramping/reversal matrix
   against the variable-rate resampler path.
5. Characterize the marginal-signal FEC delta against a same-build baseline.
6. Evaluate the second-order SRO loop behind a separate opt-in setting.
7. Repeat the full regression matrix after each controller change.

## Acceptance criteria

Any timing-loop change must satisfy all of the following:

- no timing branch flip or runaway residual timing on the full 545 MHz capture;
- no timing-induced `align-miss` or permanent outer-FEC loss;
- continuous TS output through normal 19--20 dB MER regions;
- correct recovery after genuine fades and signal loss;
- no regression on 557/581 MHz captures or synthetic SRO/CFO fixtures;
- no unexplained bias across actuator or CIR-placement changes;
- continuous arbitrary-resampler phase and FIR state across input block
  boundaries;
- bounded output-rate error for constant, ramping, and reversing SRO;
- scalar and enabled SIMD resampler paths remain numerically equivalent within
  the defined FIR tolerance;
- no regression in the native decoder tests;
- no unacceptable increase in CPU load, queue depth, latency, or dropped input.
