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
corrections and a small residual sawtooth. Evaluate, behind an opt-in setting:

1. distributing the estimated timing rate with a per-symbol phase accumulator;
2. a fractional-delay filter;
3. a continuously adjustable polyphase resampling path.

Compare residual timing jitter, pilot phase modulation, MER, FEC errors, CPU
cost, and queue pressure. Do not adopt a fractional path merely because it
makes the displayed `tau` numerically closer to zero.

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

Continue treating timing and CFO as independent control loops. Use fixtures
with independent, correlated, and deliberately conflicting sample/LO drift to
measure whether a shared hardware reference produces useful correlation.

Only consider feed-forward between the loops after telemetry demonstrates a
stable relationship across receivers and temperatures. A shared reference is
not sufficient evidence by itself.

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
4. Evaluate per-symbol/fractional correction behind an opt-in setting.
5. Evaluate the second-order loop behind a separate opt-in setting.
6. Measure sample/LO correlation only after both estimators are independently
   validated.
7. Repeat the full regression matrix before changing any default.

## Acceptance criteria

Any timing-loop change must satisfy all of the following:

- no timing branch flip or runaway residual timing on the full 545 MHz capture;
- no timing-induced `align-miss` or permanent outer-FEC loss;
- continuous TS output through normal 19--20 dB MER regions;
- correct recovery after genuine fades and signal loss;
- no regression on 557/581 MHz captures or synthetic SRO/CFO fixtures;
- no unexplained bias across actuator or CIR-placement changes;
- no regression in the native decoder tests;
- no unacceptable increase in CPU load, queue depth, latency, or dropped input.
