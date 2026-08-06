# Sample-Clock Timing Loop and AFC Plan

## Status

The long-running DVB-T timing failure has been reproduced, diagnosed, and
fixed in the current native decoder. The remaining items in this document are
incremental improvements and research directions; they are not required to
recover the current 545 MHz capture.

The main validation capture is:

- `545M-DVB-T-公視-3.cs16`: 363.9 GB, 10 MS/s CS16, approximately 151.6
  minutes of real DVB-T signal recorded by `airspy_rx`.
- The capture contains real sample-clock and LO-clock drift, dynamic
  multipath, and short signal-quality degradations.
- Offline replay uses `--decode-iq`, the same native `StreamDecoder` used by
  the GUI, and normally `--decoder-threads 16`.
- `AIRSPYTV_EVENT_DEBUG=1` enables timing and FEC event diagnostics.

## 1. Observed failure

The original failure looked like a transport/FEC problem:

1. The signal was usually around 19--20 dB MER and had stable power.
2. The MPEG-TS stream decoded normally for a long period.
3. `tau` slowly accumulated to roughly 170 samples.
4. RS failures then increased rapidly, the outer FEC lock was lost, and the TS
   output stopped.

The important point is that MER did not need to remain low for the FEC to
fail. MER measures constellation decision quality after equalization; it does
not by itself prove that the FFT window, scattered-pilot phase branch, symbol
parity, and FEC alignment are still consistent. A timing/phase branch error
can therefore precede the visible MER collapse.

Representative pre-fix timing output was:

```text
[evt] tloop tau=170.66 shift=46.0 drift=0.804 smooth=0.358 frac=0.46
[evt] tloop tau=171.11 shift=46.0 drift=0.449 smooth=0.359 frac=0.82
[evt] tloop tau=170.41 shift=47.0 drift=-0.700 smooth=0.360 frac=0.18
[evt] tloop tau=-163.89 shift=47.0 drift=-334.307 smooth=0.026 frac=0.20
[evt] tloop tau=-29.16 shift=47.0 drift=134.738 smooth=-0.447 frac=-0.24
```

After this transition, the decoder produced repeated `align-miss` events and
eventually reported `outer=-1` while the TS byte counter stopped advancing.

## 2. Root cause

The failure was caused by the interaction of three timing-loop problems, not
by a permanently bad MPEG-TS queue or by RS decoding alone.

### 2.1 Pilot-slope ambiguity

For an 8K DVB-T symbol, adjacent scattered pilots are 12 carriers apart. The
phase-slope estimate is therefore periodic in:

```text
N / 12 = 8192 / 12 samples per full 2-pi slope cycle
N / 24 = 341.3 samples per pi branch ambiguity
```

The old implementation estimated the phase ramp independently at multiple
points in the pipeline. A wrapped or multipath-contaminated estimate could
change the phase branch used by pilot verification even while the carrier
quality indicator and MER still looked healthy.

### 2.2 Dynamic multipath outliers

The capture contains moving group-delay structure. A short-lived channel
change can make the phase difference of a pilot pair cross the `arg()` wrap
boundary. This produces a large apparent timing jump even though a
sample-clock cannot move the FFT boundary by hundreds of samples in one short
window.

The previous least-squares history fit was too sensitive to one such outlier:
one bad point in a 24-window history could reverse the estimated drift, drive
the fractional accumulator into its clamp, and make the window move in the
wrong direction.

### 2.3 Feedback contamination

`shift` is the cumulative number of integer sample corrections applied by the
timing loop. If those corrections are not removed from the measured timing
history, the loop interprets its own action as physical clock drift. This
creates a sawtooth or runaway feedback path.

The sample-clock loop and the LO/CFO loop are separate control paths:

- the timing loop changes the FFT symbol-window position;
- the CFO loop changes the complex NCO phase/frequency.

A shared reference clock may correlate their physical drift, but the decoder
must still estimate and control the two observables separately.

## 3. Current implementation

The following changes are implemented in
`src/dvbt/stream_decoder.cpp`.

### 3.1 One robust scattered-pilot timing estimate

Timing is estimated only from the fixed-spacing scattered-pilot grid. The
channel estimate is formed as sent-pilot divided by received-pilot, so the
known pilot polarity is not applied a second time. The fixed spacing makes the
branch period explicit and consistent for every observation.

### 3.2 Shared timing-slope tracker

`TimingSlopeTracker` now:

- unwraps each measurement to the branch nearest the previous filtered value;
- rejects implausible jumps larger than 24 samples;
- maintains a short robust median history;
- applies a slow low-pass filter to the accepted value;
- supplies the same filtered timing value to both the timing loop and pilot
  phase verification.

This prevents one multipath click from simultaneously corrupting the timing
feedback and the phase decision.

### 3.3 Corrected feedback accounting

The statistics-window update explicitly removes the timing loop's own integer
window corrections from the drift history. The current shared timing
coordinate uses a one-sample window-step response, and the loop's cumulative
step count is added back when reconstructing the physical timing coordinate.

The drift estimator is the median of recent consecutive differences rather
than an unprotected least-squares slope. The smoothed drift remains clamped to
`+/-4`, and the existing fractional accumulator converts it into occasional
`+/-1` sample symbol-window steps.

The control variables have different meanings:

- `tau`: the filtered residual pilot-slope timing coordinate, in processed
  complex samples;
- `shift`: the cumulative integer timing correction, also in processed
  complex samples;
- `fractional_timing`: the bounded sub-sample accumulator that decides when
  the next integer step is due.

`shift` is expected to grow during a long stream when a persistent sample-clock
offset exists. It resets when a stream is re-anchored, retuned, or reset. A
bounded residual `tau`, stable MER, and continuous FEC are the meaningful
health indicators; `shift` itself is not expected to remain near zero.

## 4. Validation results

The full 545 MHz capture was replayed through the current native decoder.
During the replay:

- all 90,973,175,808 complex samples were processed;
- the TS byte counter advanced continuously to approximately 16.98 GB;
- `tau` stayed within approximately 0--20 samples rather than climbing toward
  170 or 341 samples;
- MER remained near 20 dB for the normal signal regions;
- no `outer-reset`, `align-miss`, `badlock`, or `timing-branch` event occurred;
- the only `phase-jump` event was the normal initial acquisition transition.

The TS sink for this validation was `/dev/null`; the decoder and FEC pipeline
still ran normally and the transport byte counter was monitored. A shorter
debug-enabled replay produced 1.60 GB of TS with 624 isolated RS failures and
no permanent FEC loss. The previous implementation produced 179,532 RS
failures and stopped TS output over the same 32 GiB replay segment.

The repository test suite remains green:

```text
100% tests passed out of 4
```

## 5. Improvement opportunities and TODOs

The current loop is stable enough for the validation capture. The following
items must be evaluated one at a time against this baseline.

### 5.1 Timing-coordinate bias validation

The two known bias sources now have explicit accounting:

1. `timing_window_shift_response` remains fixed at `1.0`. The full 545 MHz
   replay validates its sign and gain in the current 8K path: windows without
   an actuator step averaged 0.6807 corrected samples of drift, windows with a
   step averaged 0.7103, and reconstructed physical timing never moved
   backward. The remaining TODO is to repeat commanded-step and known-SRO
   validation in 2K mode and on the 557/581 MHz multipath captures.
2. CIR rebasing now accumulates the exact `applied_cir_offset` used for every
   symbol and uses that time-weighted mean for each full or partial statistics
   window. This replaces the beginning/end-point average. The 545 MHz replay
   had zero CIR movement throughout, so the implementation still needs a
   synthetic mid-window CIR-step test and validation on captures with dynamic
   CIR placement.

These tests should compare raw `tau`, physical timing, corrected drift, SRO ppm,
CIR offset, and shift rate. A stable non-zero `tau` remains valid; this task is
about removing actuator and placement bias from the derivative.

### 5.2 Fractional-delay or polyphase timing correction (TODO)

The current actuator is an integer-step window correction driven by a
fractional accumulator. This is robust and cheap, but it quantizes the timing
correction and applies pending steps near statistics-window boundaries, which
can leave a small residual sawtooth. The instantaneous actuator telemetry is
therefore pulse-density data, not a smooth clock estimate.

First evaluate distributing the estimated timing rate through a per-symbol
phase accumulator. Then prototype a fractional-delay filter or continuously
adjustable polyphase path behind an opt-in configuration. Expected benefits
are lower residual `tau` jitter and less periodic phase modulation.

This must not be introduced merely to force `tau` to zero. The acceptance
criterion is improved timing/FEC stability, not a numerically small `tau`.

### 5.3 CIR-aware initial and adaptive window placement

Adaptive CIR placement is already implemented once per TPS frame. It estimates
the main-lobe spread, moves the FFT anchor by at most four samples per update,
and rebases the timing loop onto the window's average position.

The remaining TODO is to validate guard-interval margins and the exact rebasing
described in Section 5.1. The CIR placement path must remain separate from the
sample-clock drift path: its target is a safe FFT window, not zero channel
delay.

### 5.4 A genuine second-order timing loop (TODO)

The present implementation filters timing differences and integrates them into
integer window corrections. It handles approximately constant clock offset
well, but it does not explicitly model changes in the drift rate.

Prototype an opt-in loop that maintains:

1. timing phase: residual window-position error;
2. timing frequency: sample-clock offset or `shift` rate;
3. optionally, a very slow drift-rate estimate for thermal changes.

The update gains must be much slower than the OFDM symbol loop and gated by
pilot confidence. The CFO/NCO loop remains separate. Correlation caused by a
shared hardware reference clock should first be measured as telemetry and only
later considered as a carefully validated feed-forward input.

### 5.5 Confidence-weighted timing updates (TODO)

The current branch rejection is deliberately conservative. Evaluate weighting
or holding timing updates using scattered-pilot coverage, channel consistency,
CIR confidence, fade indicator, MER, and post-Viterbi error indicators. A fade
or rapidly changing multipath condition should slow or freeze the timing loop;
genuine signal loss should still use re-acquisition.

### 5.6 Timing and clock telemetry

The decoder now publishes these values independently for each statistics
window without changing control behavior:

- latest raw pilot-slope timing and the accepted, unwrapped, filtered `tau`;
- physical timing after CIR and cumulative-shift rebasing;
- observed, feedback-corrected, and smoothed drift;
- estimated sample-clock offset and instantaneous actuator shift rate in ppm;
- actuator shift rate averaged over the latest 64 statistics windows;
- cumulative `shift` and fractional-timing accumulator;
- timing measurement count, rejection count, and coverage confidence;
- CIR offset and CIR peak-energy confidence;
- tracked CFO, residual CFO, FEC state, and TS continuity.

The SRO estimator stores physical timing together with the nominal sample
position of each observation and computes slopes directly in ppm. Consequently,
a final partial statistics window is normalized by its actual sample span and
cannot turn a full-window correction into a false ppm spike.

The GUI surfaces filtered timing, SRO ppm, timing confidence, and the rolling
actuator rate. Complete values are available in periodic diagnostics,
`--decode-iq` output, and `AIRSPYTV_EVENT_DEBUG=1` timing events. The raw
instantaneous actuator rate may alternate between zero and one integer-step
pulse; the rolling rate is the useful long-term comparison with SRO.

## 6. Synthetic clock-drift fixtures

`tools/generate_dvbt_fixture.py` can inject sample-clock and LO impairments
independently while its sidecar retains the nominal 10 MS/s decoder rate:

- `--sample-clock-ppm` sets the initial receiver sample-clock offset;
- `--sample-clock-drift-ppm-per-minute` applies a linear thermal-like change;
- `--lo-offset-hz` sets an independent initial carrier offset;
- `--lo-drift-hz-per-minute` applies an independent linear LO drift.

The sample-clock impairment is a time warp and the LO impairment is a
time-varying complex rotation. This permits independent, correlated, and
deliberately conflicting clock trajectories. Fixtures should cover zero drift,
both offset signs, slow ramps, and sample/LO combinations before any new loop
is enabled by default.

`tools/validate_dvbt_clock_drift.py` automates three 20-second end-to-end
fixtures. The current results are:

| Scenario | Measured SRO | Expected SRO | Measured CFO | Expected CFO | TS bytes |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sample clock only | +0.1339 ppm | +0.1885 ppm | +0.01 Hz | 0.00 Hz | 37,276,828 |
| LO only | +0.0000 ppm | 0.0000 ppm | +108.22 Hz | +108.45 Hz | 37,276,828 |
| Independent clocks | -0.0844 ppm | -0.1087 ppm | -107.69 Hz | -107.32 Hz | 37,276,828 |

The sample-clock estimate starts after an approximately 10-second history
warm-up and is still converging in a 20-second fixture. The regression therefore
uses a convergence-aware SRO tolerance while keeping the CFO check independent.
Use a retained work directory and a longer duration when evaluating loop
dynamics rather than regression correctness.

## 7. Recommended implementation order

1. Keep the current robust tracker and full-capture replay as the regression
   baseline.
2. Repeat timing-response validation in 2K mode and on the 557/581 MHz
   multipath captures; add a synthetic mid-window CIR movement case.
3. Record the completed telemetry on those remaining inputs and establish
   expected confidence and ppm ranges.
4. Add confidence gating as an isolated control change.
5. Evaluate per-symbol and fractional-delay correction behind an opt-in
   configuration.
6. Prototype the second-order timing loop behind a separate opt-in
   configuration; keep CFO control independent.
7. Validate every change against all captures and synthetic fixtures before
   changing the default.

## 8. Acceptance criteria

Any future timing-loop change should satisfy all of the following:

- no timing-loop `tau` branch flip or runaway drift on the full 545 MHz
  capture;
- no timing-induced `align-miss` or permanent outer-FEC loss;
- continuous TS output through normal 19--20 dB MER regions;
- stable recovery after genuine fading, without treating a fade as a timing
  failure;
- no regression on the 557/581 MHz captures;
- no regression in the native decoder test suite;
- no unexplained increase in CPU load, queue depth, or dropped input blocks.
