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
- integer timing corrections driven by a fractional accumulator;
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

### 1. Timing-coordinate bias

Two compensation paths are implemented but still need broader validation.

#### Window-step response

`timing_window_shift_response = 1.0` has the correct sign and gain on the 8K
545 MHz capture. Remaining work:

- repeat commanded `+1/-1` window-step tests in 2K mode;
- run known positive and negative SRO fixtures in both 2K and 8K modes;
- repeat the measurement on the 557 and 581 MHz multipath captures;
- verify that physical timing remains continuous across actuator steps.

#### CIR rebasing

The timing coordinate uses the exact time-weighted mean of
`applied_cir_offset`, but the 545 MHz capture did not exercise CIR movement.
Remaining work:

- add a synthetic CIR move in the middle of a statistics window;
- test repeated and opposing CIR moves;
- validate captures with naturally changing CIR placement;
- verify the available guard-interval margin before and after each move.

For both paths, record raw `tau`, filtered timing, physical timing, corrected
drift, SRO ppm, CIR offset/confidence, cumulative shift, and actuator rate. A
stable non-zero residual `tau` is acceptable; continuity and unbiased drift are
the criteria.

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

### 3. Fractional timing correction

The current integer-step actuator is stable but produces pulse-density
corrections and a small residual sawtooth. The selected experiment is an
opt-in variable-ratio arbitrary resampler that can eventually replace the
fixed-ratio liquid-dsp rational resampler.

The resampler should combine the nominal input-to-DVB-baseband rate conversion
with a continuously adjustable fractional-delay polyphase FIR. Its input phase
increment is approximately:

```text
nominal_input_samples_per_output
    * (1 + sro_correction_ppm * 1e-6)
```

This resampler is a common source-tree DSP routine, not a DVB-T-specific
component. DVB-T is only the first consumer. The same implementation should be
usable by future DVB-T2, DVB-S/S2, DVB-C, and other broadcast demodulators,
with each standard supplying its own nominal input/output rate profile,
bandwidth constraints, filter requirements, and timing-loop command. Standard
specific code must not depend on the resampler's internal phase, history, or
worker implementation.

The correction sign must be established with positive and negative synthetic
SRO fixtures. The implementation must:

- retain fractional phase and FIR history across input blocks;
- snapshot the target SRO correction at a block boundary and slew toward it
  instead of changing the rate abruptly within a block;
- emit a variable number of output samples while preserving the configured
  nominal baseband time axis for the consuming demodulator;
- remain continuous across arbitrary input block boundaries, flushes, and
  normal streaming operation;
- keep reset and retune behavior explicit rather than carrying stale phase or
  filter state into a new stream generation.

Use a block-local Q32.32 phase accumulator for the scalar reference
implementation. The upper 32 bits identify the input sample within the current
work buffer and the lower 32 bits hold the fractional phase. Rebase the phase
whenever consumed input is discarded, retaining only the FIR context required
by future outputs. An unbounded absolute input position is not required by the
DSP algorithm; a separate `uint64_t` cumulative sample counter is optional for
telemetry and deterministic test diagnostics. A practical polyphase bank may
use 1024 or 4096 phases and derive its index, plus optional interpolation, from
the Q32.32 fractional field.

The SRO estimator and second-order clock model remain owned by the consuming
demodulator, while the arbitrary resampler remains a common actuator. The
interface should carry a standard-neutral target rate and SRO correction
command; DVB-T's FFT-window actuator is only one possible downstream residual
timing actuator. Once that feedback path is enabled, do not also integrate the
same SRO estimate into the existing integer FFT-window actuator: that would
double-correct the clock error. Keep standard-specific timing movement for
residual timing phase and channel-delay placement.

Implement and validate a portable scalar kernel first. Then benchmark optional
SSE4.1-compatible, AVX2, and AVX2/FMA FIR kernels while retaining the scalar
path as the numerical reference. Preserve the current resampler worker-budget
model where parallel output partitions provide a measured benefit.

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

| Dimension | Cases |
| --- | --- |
| DVB-T mode | 2K, 8K |
| Channel bandwidth | supported 5/6/7/8 MHz cases where fixtures exist |
| Sample clock | zero, positive/negative offset, linear ramp, reversal |
| LO clock | zero, positive/negative offset, linear ramp |
| Clock relationship | independent, correlated, conflicting |
| Channel | clean synthetic, static multipath, moving CIR, fade, real captures |
| Input path | deterministic offline replay; live SDR where practical |

Retain machine-readable summaries for SRO error, CFO error, timing confidence,
branch rejections, actuator rate, MER, FEC failures, TS bytes, processing ratio,
queue watermarks, and dropped blocks.

## Recommended order

1. Complete 2K, 557/581 MHz, and synthetic CIR-bias validation.
2. Establish expected confidence and ppm ranges for those inputs.
3. Add confidence gating as an isolated control change.
4. Implement the portable scalar arbitrary resampler behind an opt-in setting
   and validate its fixed-rate response before enabling feedback.
5. Validate phase/history continuity across random block boundaries, then
   connect the SRO estimate while separating the long-term resampler actuator
   from residual FFT-window correction.
6. Evaluate the second-order SRO loop behind a separate opt-in setting.
7. Add and benchmark optional SIMD resampler kernels against the scalar
   numerical reference.
8. Repeat the full regression matrix before changing any default.

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
