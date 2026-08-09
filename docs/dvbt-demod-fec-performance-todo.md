# DVB-T Demodulator and FEC Performance TODO

## Scope and current conclusion

This document tracks measured optimization work for the serial DVB-T
demodulator and FEC coordinator threads. Carrier, timing, channel, and TPS
state remain symbol-ordered on `dvbt-demod`; convolutional decoding remains in
the Viterbi worker pool; outer deinterleaving, Reed-Solomon decoding, energy
descrambling, and transport recovery remain ordered by `dvbt-fec`.

The current pipeline has two closely matched limiting stages:

- `dvbt-demod` is CPU-bound. It is busy for 98.9% of sampled wall time, and a
  steady 400-symbol window takes 64.42 ms wall / 63.84 ms serial busy on
  average.
- `dvbt-fec` is now a near co-bottleneck. The matching FEC window takes
  60.72 ms on average, its input queue averages 69.0% full, and 10 of 89
  one-second snapshots find the queue completely full.
- Demod channel estimation is the largest serial demod bucket. Inside it, the
  scattered-pilot timing estimator is the largest measured operation.
- Reed-Solomon decoding is the dominant `dvbt-fec` CPU hotspot. Queue capacity
  and additional Viterbi workers do not address that serial work.

Optimizing only the demodulator will move the hard limit to FEC. Work should
therefore alternate between the highest-confidence demod and FEC changes and
remeasure end-to-end throughput after each one.

## 545 MHz report baseline

The primary data source is the interrupted machine-readable report at
`~/ram/545MHz-公視-perf1`, reread on 2026-08-09. Because the run did not close
normally, `stats.json` still says `running` and `source-sessions.jsonl` is
empty. Detailed streams are internally aligned and usable for steady-state
analysis, but this directory is not a completed acceptance report.

| Property | Value |
| --- | ---: |
| Source rate / DVB-T channel bandwidth | 10 MS/s / 6 MHz |
| Mode / guard / modulation / code rate | 8K / 1/4 / 64-QAM / 2/3 |
| Resampler / symbol / Viterbi workers | 4 / 6 / 6 |
| Completed demod and FEC windows | 1,378 each |
| Symbols per window | 400 |
| Processed signal at final pipeline sample | 823.6 s |
| Wall time at final pipeline sample | 89.241 s |
| Aggregate report-run speed | 9.229x realtime |
| Transport output at final FEC record | 1,535,839,116 bytes |
| TS packets / RS failures / TEI packets | 8,169,357 / 604 / 604 |
| Decoder input drops / phase discontinuities | 0 / 0 |

The 604 TEI packets are 0.0074% of emitted packets. The event stream contains
266 RS-failure-streak notices and 10 recoveries, but no outer-FEC reset or
demod reacquisition. This is a useful marginal-signal workload rather than a
clean-output equivalence fixture.

The first demod/FEC record includes startup, alignment, and initial backlog
(415.96 ms demod wall and 228.80 ms FEC wall), so steady statistics below use
sequences 2 through 1,378: 1,377 windows covering 822.528 seconds of signal.
Percentages for demod buckets use the sum of non-overlapping serial-busy
timers. Nested timers and aggregate worker work are never added to that total.

The report run includes machine-readable telemetry overhead. Use a completed
non-reporting Release run for the headline realtime acceptance number; use
these records to locate and compare stage costs.

## Demodulator bottleneck

### Serial stage distribution

| Serial demod bucket | Mean per window | p95 | Serial busy share |
| --- | ---: | ---: | ---: |
| Channel estimation | 20.64 ms | 21.37 ms | 32.33% |
| Pilot phase lock | 10.13 ms | 10.83 ms | 15.86% |
| FFT and CFO tracking | 10.10 ms | 10.51 ms | 15.81% |
| Payload extraction/equalization | 7.22 ms | 7.76 ms | 11.30% |
| Sample-ring copy | 6.32 ms | 6.70 ms | 9.90% |
| Ordered batch output / FEC enqueue | 5.63 ms | 12.05 ms | 8.83% |
| Ordered symbol postprocessor wait | 2.90 ms | 3.69 ms | 4.54% |
| Symbol submission | 0.71 ms | 0.87 ms | 1.11% |
| Other, TPS, reacquisition | 0.19 ms | -- | 0.30% |

The output/FEC-enqueue bucket is no longer negligible: its p95 is 12.05 ms
and maximum is 25.12 ms. `demod_process_batch()` combines statistics,
optional analysis publication, gate-buffer management, and synchronous FEC
enqueue. Full-queue snapshots make FEC backpressure a plausible contributor
to the tail, but the current timer cannot attribute all of it to queue wait.
Split this bucket before optimizing it; do not hide confirmed queue wait in a
larger queue.

### Nested operations

| Nested operation | Mean per window | Share of parent | Approx. serial demod share |
| --- | ---: | ---: | ---: |
| Scattered-pilot timing estimate | 12.38 ms | 60.00% of channel | 19.40% |
| FFTW execution | 9.65 ms | 95.63% of FFT/CFO | 15.12% |
| Channel interpolation | 4.73 ms | 22.94% of channel | 7.42% |
| Pilot inverse-channel estimates | 3.16 ms | 15.30% of channel | 4.95% |
| CFO tracking | 0.41 ms | 4.02% of FFT/CFO | 0.64% |

The six symbol workers perform an aggregate 187.46 ms of CPU work per window
(73.02 ms preprocess, 45.67 ms deinterleave, 40.02 ms depuncture, and
28.75 ms demap). That number is intentionally non-additive because the work
runs in parallel. The serial postprocessor wait is only 4.54% of demod busy,
so splitting the ordered demod loop or adding symbol workers is not the first
target.

### Supplementary thread profile

A 499 Hz cycle profile of the same signal's 100-second prefix was collected
with the current Release build. Relative self samples on `dvbt-demod` were:

| Symbol | Demod-thread self cycles |
| --- | ---: |
| `DemodStage::Impl::demod_dispatch_payload` | 12.40% |
| `DemodStage::Impl::demod_estimate_channel` | 11.98% |
| `estimate_scattered_timing_tau` | 10.75% |
| `SampleChannel::read_symbol` | 10.13% |
| `lock_phase_at_offset` | 9.91% |
| floating-point introsort used by the timing median | 7.04% |
| `atan2f` | 4.10% |
| complex division helper | 3.91% |

This independently confirms the telemetry priorities: timing-slope
generation plus median sorting, pilot lock, ring access, channel work, and
payload dispatch dominate the serial thread. Perf self percentages and
telemetry bucket percentages describe different scopes and must not be added.

## FEC bottleneck

### Window timing and queue pressure

| FEC measure | Mean | p50 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| `fec::total` per window | 60.72 ms | 61.70 ms | 73.01 ms | 79.91 ms | 85.77 ms |
| `fec::transport` per window | 60.68 ms | 61.67 ms | 73.04 ms | 79.88 ms | 85.53 ms |
| TS packets per window | 5,929 | 6,047 | 7,290 | 7,850 | 8,579 |

Across the steady records, `fec::transport` accounts for 83,558.03 of
83,608.87 ms, or 99.94% of FEC wall timing. The remaining FEC-stage work is
only 0.037 ms per window. Packet-normalized cost is 10.24 ms per 1,000 emitted
TS packets.

The current soft-metric path treats synchronous Viterbi completion, decoded
byte handoff, outer deinterleaving, Reed-Solomon, energy descrambling, and
output assembly as one nested `fec::transport` bucket. Depuncture is already
performed by the symbol workers. Queue snapshots make clear that the
aggregate FEC stage matters, but the report alone cannot assign wall latency
among its transport operations:

| One-second pipeline snapshots | Result |
| --- | ---: |
| FEC queue mean / p50 / p95 occupancy | 69.0% / 71.6% / 100% |
| Queue completely full | 10 / 89 snapshots |
| Queue empty / FEC waiting for input | 7 / 89 snapshots |
| Demod processing | 88 / 89 snapshots |
| Frontend waiting for ring space | 38 / 89 snapshots |

The queue alternates between empty and full because the two stages are close
and their per-window costs vary. Increasing the 134-item capacity would only
absorb a longer burst; it would not improve sustained throughput.

### FEC coordinator profile

The supplementary cycle profile resolves the broad transport bucket. Relative
self samples on `dvbt-fec` were:

| Symbol | FEC-thread self cycles |
| --- | ---: |
| generic `decode_rs_char` | 61.22% |
| inlined/general `OuterFec::Impl::process` work | 23.04% |
| `ByteDeinterleaver::process` | 8.68% |
| `DvbReedSolomon::decode` wrapper | 1.63% |

These four entries account for 94.57% of the coordinator's self cycles.
Reed-Solomon is therefore the primary FEC CPU bottleneck, followed by outer
FEC byte/buffer processing. The locked path currently performs per-byte deque
push/pop in the 12-branch deinterleaver, repeatedly erases 204 bytes from the
front of `rs_bytes`, copies each codeword into a correction buffer, and grows
transport output vectors.

The six `dvbt-vit-*` threads are separate from the percentages above; 98.53%
of their combined self samples are in `SoftViterbi::Impl::run_worker()`. Their
synchronous completion latency is included in `fec::transport` wall time, but
the current telemetry cannot separate it from coordinator work. Do not infer
that adding Viterbi workers will help merely from their aggregate CPU usage:
the serial coordinator is already hot and Reed-Solomon dominates it.

## Phase 0: improve FEC timing attribution

FEC status (2026-08-09): implemented. Reports now preserve
`fec::transport` and add Viterbi submit/collection and blocking wait, outer-FEC
stage attribution, window-level FEC coordinator thread CPU, and aggregate
Viterbi worker work. Packet-local outer-FEC sub-stages use deterministic
1-in-32 sampling; the parent wall timers remain exact. Detailed timing is only
enabled while telemetry/reporting is active. A 60-second A/B decode of the
545 MHz capture measured 9.6x realtime both before and after with reporting
enabled; reporting-disabled runs remained in the same 9.4x--10.0x observed
range. The demod-output split below remains open.

Add non-overlapping or explicitly nested FEC timers before evaluating larger
algorithmic changes:

- Viterbi submit/wait and decoded-output collection;
- decoded-byte handoff and outer-FEC call overhead;
- bit repacking and byte deinterleaving;
- Reed-Solomon decoding;
- energy descrambling and TEI handling;
- buffer compaction and transport output assembly.

Also expose coordinator active CPU time separately from synchronous Viterbi
wait. Preserve `fec::transport` as the parent metric so old and new reports
remain comparable. Validate that instrumentation overhead is negligible with
reporting disabled and enabled.

Split `demod::output` into analysis publication, gate-buffer maintenance, FEC
enqueue work, and queue wait at the same time. That will distinguish local
demod bookkeeping from downstream backpressure.

## Phase 1: behavior-preserving changes

Phase 1 status (2026-08-09): implemented. The demodulator now uses exact
selection for the timing median, two-span ring copies, and demod-owned channel,
continual-carrier, timing, and CIR scratch. The locked outer-FEC path uses
fixed circular delay lines, zero-copy offset-zero repacking, one `rs_bytes`
compaction per input batch, a codeword cursor, and reserved output. The median
selection matches a fully sorted reference across edge cases and 2,000
deterministic randomized inputs. A 20-second `557mhz-horizontal` run reduced
single-worker user CPU from 15.41 s to 14.88 s before the Phase 2 changes.
The 20-second outputs for `557mhz-horizontal`, `581mhz-2`, and `581mhz-3`
remain byte-identical to the pre-Phase-1 binary.

Implement and measure these changes separately. They should retain
byte-identical TS output on clean captures and identical packet/TEI decisions
on the 545 MHz capture.

### Demod: exact median selection

`estimate_scattered_timing_tau()` calculates about 568 adjacent
scattered-pilot slopes per 8K symbol and fully sorts them to obtain a median.

- replace the full sort with `std::ranges::nth_element()` for the upper
  median;
- for an even count, take the lower median as the maximum of the lower
  partition;
- retain `double`, current slope samples, branch handling, and filtering;
- measure slope generation and selection separately after the change.

The selected median must be numerically identical. Do not combine this with
pilot decimation or a different robust estimator.

### Demod: contiguous ring reads and reusable scratch

- split a wrapping `AbsoluteSampleRing` read into at most two contiguous copy
  operations rather than performing one modulo operation per FFT sample;
- retain channel, continual-carrier, and timing-estimate scratch buffers in
  demod-owned state and overwrite every element before reuse;
- keep synchronization and absolute-position validation unchanged;
- do not introduce shared mutable payload ownership. Payload vectors leave the
  demod thread and need a separately bounded pool if allocation profiling later
  justifies one.

### FEC: remove locked-path buffer churn

- replace the 12 per-branch byte deques with fixed-size circular delay buffers
  while preserving branch phase exactly;
- consume complete 204-byte RS codewords through a cursor/span and compact
  `rs_bytes` once per input batch instead of erasing its front per packet;
- reserve or directly size transport output for the number of complete
  codewords in the batch;
- avoid an extra bit-repacker vector when the locked bit offset is zero, as it
  is throughout this report;
- measure codeword copy/correction-buffer cost separately before attempting
  ownership changes in `DvbReedSolomon`.

The named `OuterFec::process` and `ByteDeinterleaver::process` symbols account
for 31.72% of FEC self samples, so these changes have meaningful headroom
without changing correction decisions.

## Phase 2: algorithmic and tracking-aware work

Phase 2 status (2026-08-09): implemented, except RS parallel batching was
evaluated and rejected for the current call granularity. Detailed results are
recorded under each item below. The final completed, reporting-enabled
60-second `557mhz-horizontal` run reached 10.4x realtime (5.82 s wall,
50.58 s user, 1.21 s system), emitted 595,447 packets with zero TEI, and was
byte-identical to the Phase 1 baseline. This restores and slightly exceeds the
pre-regression 10.1x target while retaining detailed attribution.

### Reed-Solomon kernel and batching

- instrument syndrome calculation, error-location/correction, and clean-block
  exit inside the current generic kernel;
- benchmark a tuned DVB RS(204,188) implementation against `decode_rs_char`
  using the actual corrected/uncorrectable distribution;
- evaluate batched independent codeword decoding with per-worker codec state,
  followed by ordered energy descrambling and emission;
- keep exact corrected-payload-bit accounting and uncorrectable/TEI behavior;
- avoid one task dispatch per 204-byte codeword; any parallel version must
  batch enough codewords to amortize synchronization.

The acceptance decision is end-to-end throughput, not isolated RS packets per
second. A faster RS kernel that increases queueing, changes correction results,
or destabilizes recovery is unacceptable.

The clean 20-second capture contained 198,384 clean codewords and no corrected
or uncorrectable codewords. Syndrome calculation accounted for an estimated
1,092.55 ms of 1,152.36 ms RS time (94.8%); codeword and payload copies were
4.96 ms and 4.89 ms. A fixed-parameter DVB RS(204,188) specialization of the
same Karn algorithm is now used. It is 1.213x faster on clean blocks and
1.215x faster on a 90% clean / 8% correctable / 2% uncorrectable microbenchmark.
It matches the generic decoder's decision and corrected bytes for 2,000
deterministic 0--12-error cases. Clean/corrected/uncorrectable counts and the
sampled copy, syndrome, locator, correction, and payload-copy timings are now
reported.

One locked `OuterFec::process()` call receives 960 bytes, only four complete RS
codewords plus carry. Splitting that batch over multiple RS workers would leave
only about two codewords per worker, contrary to the required synchronization
amortization. Parallel RS is therefore not enabled; reconsider it only if the
upstream handoff is coalesced into materially larger ordered batches.

### Expected pilot-phase fast path

With a valid grid, phase normally advances as `(previous_phase + 1) % 4`.

- score the expected phase first with a normalized confidence measure;
- fall back to the current four-phase search on low confidence,
  discontinuity, cold start, or recovery;
- preserve fade freeze, stable-grid capture, phase-discontinuity telemetry,
  and event-driven re-anchor behavior;
- establish thresholds from retained clean and marginal captures.

The expected phase now uses an energy-normalized coherent pilot score and is
accepted at confidence >= 0.90. Observe-only probes on 20 seconds of
`557mhz-horizontal` matched the full four-phase search 197/197 times with
minimum confidence 0.9822. The first 60 seconds of `545M-DVB-T-公視-3`
matched 591/591 sampled checks with minimum confidence 0.9447. In the enabled
run, all 40,159 expected-phase checks used the fast path (minimum 0.9435) with
no phase discontinuity; low confidence still falls back to the unchanged full
search and is reported as an event.

### Reduced timing-estimator cadence

Evaluate the scattered-pilot timing slope every two or four symbols while
using the last accepted filtered value between updates.

- retain full-rate updates during cold lock, after re-anchor, and while timing
  confidence recovers;
- keep sample-domain SRO command scheduling deterministic;
- compare estimator noise, branch rejection, residual timing, SRO convergence,
  MER, and recovered TS packets;
- compare cadence reduction against pilot subsampling rather than changing
  both dimensions in one experiment.

The steady-state cadence is four symbols. Measurement remains full-rate until
the 16-window timing history is ready, and returns to full-rate during fades,
hopeless/frozen lock, and CFO recovery. On the same 60-second 545 MHz segment,
full-rate, cadence-2, and cadence-4 runs took 40,159, 23,280, and 14,840 timing
measurements. Cadence 2 and 4 produced byte-identical TS and identical RS/TEI
decisions; cadence 4 reduced measured slope generation/selection from
362.61/184.01 ms to 138.10/70.31 ms. The combined expected-phase and cadence
changes produced 595,282 usable packets and zero TEI on that segment, compared
with 594,678 usable packets and 604 TEI in the observe-only full-search run.

A longer 180-second validation exposed a cadence integration regression that
the 60-second comparison did not catch. Timing confidence was still calculated
as accepted measurements divided by all symbols, so cadence 4 permanently
reported 0.25 after the initial full-rate history. The SRO command gate requires
at least 0.75 confidence and consequently stopped updating after window 16;
the FFT window drifted to about 164 samples by 155 seconds, generated 28,560
uncorrectable RS packets, reset outer FEC, and left the coordinator in repeated
alignment searches. Confidence now means accepted divided by attempted timing
measurements. A repeated 180-second run kept confidence at 1.0, commanded and
applied SRO near the 0.18 ppm estimate, completed without an outer-FEC reset,
and retained only the capture's short marginal burst (711 TEI packets, followed
by recovery). This long-prefix regression check is required for future cadence
changes.

### FFT planning

- benchmark `FFTW_MEASURE` against `FFTW_ESTIMATE` for 2K and 8K plans;
- account for planning latency and input-buffer modification during planning;
- preserve stable buffer addresses and safe plan lifecycle;
- do not enable FFTW worker threads without an end-to-end gain. Per-symbol FFT
  work may be too small to amortize another synchronization boundary.

`FFTW_MEASURE` reduced isolated execution from 1.836 us to 1.693 us for 2K and
from 21.65 us to 13.08 us for 8K. Planning increased from about 0.29 ms to
78.36 ms and 141.33 ms respectively and modified the input array. The main
demod plan now uses `MEASURE` once per grid lifetime, then explicitly restores
the stable input/output buffers; acquisition and CIR plans remain
`ESTIMATE`. A 60-second 8K decode measured 43.27 s user CPU with `ESTIMATE`
and 42.88 s with `MEASURE`, with essentially unchanged wall time
(13.93/13.96 s). The 20-second clean TS remained byte-identical. The planning
cost is exposed once in the first demod window's `setup_ms.fft_plan` field and
is not repeated per symbol or ordinary re-anchor.

## Phase 3: secondary work

- evaluate incremental complex channel interpolation to avoid repeated
  division and integer-to-float conversion;
- profile the remaining allocation sites before implementing a bounded
  payload/equalizer buffer pool. A process-wide allocator A/B measured a small
  CPU cost but no critical-path throughput gain, as detailed below;
- tune symbol and Viterbi worker allocation only after serial demod and RS
  costs fall enough for worker waits to dominate.

### Offline queue-capacity multiplier

The earlier conclusion that the demod-to-FEC queue could only improve burst
tolerance was revisited after the allocation work made the two stages more
closely matched. The current 10 MS/s, 6 MHz, 8K, guard-1/4 pipeline has these
buffer boundaries at the default 1x setting:

| Boundary | Capacity | Approximate duration | Multiplied offline |
| --- | ---: | ---: | --- |
| File/source to frontend IQ queue | 2,000,000 complex samples | 0.200 s at 10 MS/s | no |
| Frontend to demod resampled ring | 2,000,000 complex samples | 0.292 s at 48/7 MS/s | no |
| Demod symbol-worker outstanding work | 134 symbols | 0.200 s | yes |
| Demod to serial FEC queue | 134 items/symbols | 0.200 s | yes |
| FEC to Viterbi worker queue | 1,024 windows | more than 0.200 s at maximum useful bitrate | no |
| Offline asynchronous TS output | 24 MiB | output-rate dependent | no |

The resampled ring is intentionally excluded: increasing it changes the safe
application delay for sample-clock and carrier-frequency corrections, so it is
not merely queue tuning. The source IQ queue is also excluded after an
isolation test showed that extra file prefetch was not needed for the gain. The
already-large Viterbi and exact-output queues were not implicated by the
observed backpressure either.

`--offline-queue-multiplier N`, limited to 1 through 16, scales the symbol
worker and FEC capacities. It is rejected unless `--decode-iq` is selected, so
GUI and CLI live decoding remain at 1x. Offline decoding defaults to 4x;
passing 1 restores the previous capacities.

An initial capacity-range experiment scaled the IQ and symbol/FEC queues
together. The complete 150.283-second `557mhz-horizontal` capture was measured
in three interleaved runs per setting after one warm-up:

| Multiplier | Wall mean / sample SD | Realtime | User + system CPU | Mean maximum RSS |
| --- | ---: | ---: | ---: | ---: |
| 1x | 12.633 / 0.172 s | 11.896x | 122.830 s | 167.9 MiB |
| 2x | 12.277 / 0.586 s | 12.241x | 123.993 s | 187.1 MiB |
| 4x | 11.850 / 0.079 s | 12.682x | 123.453 s | 224.3 MiB |
| 8x | 11.800 / 0.115 s | 12.736x | 123.320 s | 273.8 MiB |

Four times was the useful range: 8x saved only another 0.4% and used about
49.5 MiB more memory. Two times crossed the burst-size threshold inconsistently
and had much higher run-to-run variation. A follow-up held the IQ queue at 1x
and scaled only the symbol/FEC queues, with five runs at each endpoint:

| Final scope | Wall mean / sample SD | Realtime | User + system CPU | Mean maximum RSS |
| --- | ---: | ---: | ---: | ---: |
| 1x | 12.626 / 0.147 s | 11.903x | 122.702 s | 167.9 MiB |
| FEC 4x | 11.976 / 0.165 s | 12.549x | 123.590 s | 192.8 MiB |

FEC-only 4x is therefore the default tradeoff. It reduced wall time by 5.15%,
at a 24.9 MiB RSS cost. Total CPU increased by 0.72%, confirming that the wall
gain comes from greater stage overlap rather than less computation. Compared
with scaling both queues, it recovered about 31.5 MiB with no material loss of
throughput.

The final reporting-enabled pair observed the FEC capacity change from 134 to
536 symbols, while both the IQ queue and resampled ring stayed at 2,000,000
samples. The FEC queue was completely full in 5 of 13 one-second snapshots at
1x versus 0 of 12 at 4x; mean occupancy changed from 72.4% to 60.1%. Complete
1x and FEC-only 4x transport outputs were byte-identical
(`bb23ce20e8263b92f772320e9919fcfb1a2c09d3a84470c15272114712b2ca28`).

This does not make an arbitrarily large queue a sustained-throughput fix. The
gain is specific to preventing the current gate-window and FEC work bursts
from repeatedly synchronizing two near-equal stages; capacity should be
remeasured after either stage's service-time distribution changes.

### Process allocator A/B

The allocation-cost screening test used the Release build at `95894ed`, the
complete 150.283-second `557mhz-horizontal.cs16` capture, automatic worker
selection, reporting disabled, and transport output to `/dev/null`. Each
allocator received one warm-up followed by five timed runs in an interleaved
order. Only `LD_PRELOAD` changed. The alternatives were gperftools 2.18.1
`libtcmalloc_minimal.so.4` and jemalloc 5.3.1 `libjemalloc.so.2`; glibc 2.44 was
the baseline.

| Allocator | Wall mean / sample SD | Realtime | User CPU mean | System CPU mean | User + system mean | Median RSS | Median minor faults |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| glibc | 12.586 / 0.103 s | 11.940x | 121.894 s | 2.590 s | 124.484 s | 167.8 MiB | 22,719 |
| tcmalloc minimal | 12.574 / 0.061 s | 11.952x | 120.610 s | 2.530 s | 123.140 s | 161.9 MiB | 12,395 |
| jemalloc | 12.680 / 0.084 s | 11.852x | 122.198 s | 2.756 s | 124.954 s | 242.7 MiB | 36,052 |

tcmalloc reduced total process CPU by 1.08%, minor faults by 45.4%, and median
RSS by 3.5%, but its 0.095% realtime improvement is much smaller than the
run-to-run wall variation. The CPU saving therefore did not shorten the
serial demod/FEC critical path in this workload. Default jemalloc was 0.75%
slower than glibc and increased median RSS by 44.6%.

Do not add an allocator dependency or change the default based on this result.
The tcmalloc CPU reduction is useful evidence that allocation churn still
exists, but a payload/equalizer pool needs allocation-site profiling and a
measured reduction in the relevant demod timing bucket before implementation.

### Allocation count and call-site profile

Allocation counts were measured separately from the allocator A/B. A temporary
glibc interposer counted `malloc`, `calloc`, `realloc`, aligned allocation, and
C++ `new` calls by thread without taking stack traces. Two complete
150.283-second decodes took 12.66 and 12.46 seconds, so the counter did not add
visible wall overhead relative to the 12.586-second uninstrumented mean. The
two runs made 2,882,864 and 2,890,862 allocation requests; the mean is
2,886,863 requests, or 19,210 requests per second of input signal. They also
requested 101.70 GB cumulatively. This is requested-size churn, not resident
memory or proof that every byte is touched.

| Thread group | Mean requests | Share | Requested bytes | Mean requested size |
| --- | ---: | ---: | ---: | ---: |
| Six symbol workers | 1,004,240 | 34.79% | 80.19 GB | 79,848 bytes |
| Serial FEC coordinator | 919,575 | 31.85% | 7.59 GB | 8,259 bytes |
| Six Viterbi workers | 632,701 | 21.92% | 329.58 MB | 521 bytes |
| Serial demodulator | 327,385 | 11.34% | 7.45 GB | 22,764 bytes |
| Other threads | 2,962 | 0.10% | 6.14 GB | 2.07 MB |

The "other" bytes are dominated by the file-source input blocks, which are
large but rare. Symbol and Viterbi worker counts are deterministic in these
runs. FEC request count varied by 0.74% with scheduler-dependent handoff batch
sizes; demod varied by 0.36%, mainly in startup/planning work.

A gperftools cumulative heap profile of the 20-second prefix attributed
441,828 requests versus 443,680 from the exact counter in a separate decode,
covering 99.58% of requests. The leading application call sites were:

| Allocation call site | Requests | Requested bytes | Interpretation |
| --- | ---: | ---: | --- |
| `SymbolPostprocessorPool::process` | 120,456 | 10.68 GB | Nine work vectors for each of 13,384 symbols |
| `SoftViterbi::Impl::run_worker` | 84,350 | 44.44 MB | Per-window decoder result allocation |
| `SoftViterbi::Impl::dispatch_ready_windows` | 45,672 | 692.42 MB | Ordered Viterbi input/window assembly |
| `OuterFec::Impl::process` | 42,624 | 39.33 MB | Outer-FEC intermediate/output vectors |
| `DemodStage::Impl::demod_dispatch_payload` | 26,775 | 971.36 MB | Payload and equalizer-power vectors; normally two per symbol |
| `TransportDecoder::Impl::process_viterbi_output` | 16,282 | 98.95 MB | Per-window packet and aggregate output vectors |
| `SoftViterbi::Impl::take_ready_locked` | 15,878 | 106.72 MB | Ordered decoded-byte collection |
| `SymbolPostprocessorPool::run_worker` | 13,384 | 1.93 MB | One completed-map node per symbol |
| `DemodStage::Impl::demod_process_batch` | 3,346 | 1.39 MB | Ordered batch aggregation |
| `FecStage::enqueue` | 2,684 | 1.18 MB | Queue-node allocation |
| `AsyncTransportOutput::submit` | 1,709 | 77.89 MB | Required offline output block copy |

The 62,910 requests not attributed to an application frame were predominantly
the one-time FFTW `MEASURE` plan and accounted for 109.81 MB. They do not recur
per symbol or ordinary re-anchor and are not a buffer-pool target.

Seven of the nine allocations in `SymbolPostprocessorPool::process` are pure
scratch. The reliability and mother-metric vectors leave the worker and require
an ownership-transfer design; they cannot use the same worker scratch safely.
The two payload vectors allocated by the demod thread also leave that thread.

### Implemented allocation-churn reduction

Allocation status (2026-08-09): the low-risk worker-local and FEC-owned reuse
paths are implemented. Each symbol worker now owns pre-sized nearest-point,
error, equalizer-order, demap, deinterleave, bit-order, and depuncture scratch.
The reliability and mother-metric outputs retain their existing move ownership.
This reduced `SymbolPostprocessorPool::process` from 120,456 to 26,768
allocations on the 20-second profile: exactly seven removals for each of 13,384
symbols. The six workers perform 42 scratch allocations during startup.

The FEC data path now supports caller-retained output capacity while preserving
the original return-by-value APIs for existing callers and tests. `FecStage`
owns the reusable callback buffer, `TransportDecoder` owns its decoded Viterbi
scratch, and `OuterFec` appends directly to the final buffer. Soft-Viterbi task
metric and result-byte vectors are recycled inside the bounded worker pool.
Buffer ownership does not cross the GUI/CLI boundary; both continue to use the
same `FecStage` and synchronous transport callback.

| 20-second cumulative heap profile | Requests | Requested bytes | Request reduction |
| --- | ---: | ---: | ---: |
| Before churn work | 441,828 | 13.78 GB | -- |
| Symbol worker scratch | 351,863 | 4.06 GB | 20.36% |
| Direct/reusable FEC output | 274,129 | 3.82 GB | 37.95% |
| Viterbi task/result recycling | 192,396 | 3.10 GB | 56.45% |

The final profile reduced cumulative requested bytes by 77.48%. The remaining
leading steady call sites are two required symbol outputs per OFDM symbol, one
completed-map node per symbol and Viterbi window, the two demod payload vectors,
and the required asynchronous output copy. Replacing the out-of-order completed
maps with custom rings or recycling buffers across the demod/FEC callback
boundary would change backpressure or ownership semantics and is deferred until
the smaller measured gain justifies that complexity.

Five complete non-reporting Release runs of `557mhz-horizontal` averaged
12.524 seconds with a 0.179-second sample standard deviation, or 12.000x
realtime. The pre-change allocator baseline was 12.586 / 0.103 seconds and
11.940x. Mean user-plus-system CPU fell from 124.484 to 122.422 seconds (1.66%)
while wall time improved 0.49%. The 20-second TS was byte-identical to the
pre-change binary.

The reporting-enabled 180-second `545M-DVB-T-公視-3` prefix completed at
12.014x realtime with the expected 711-packet marginal burst, two initial
alignment searches, no alignment miss or outer-FEC reset, and final RS/energy
synchronization. Timing confidence remained 1.0; near 155 seconds the estimated,
commanded, and applied SRO were 0.1810, 0.1810, and 0.1808 ppm. This preserves
the long-prefix cadence regression guard.

## Validation requirements

Every optimization must report the affected timing bucket, complete decoder
throughput, and queue pressure. Nested values must remain marked non-additive.

Minimum validation:

- Debug and Release unit/integration suites;
- clean `557mhz-horizontal`, `581mhz-2`, and `581mhz-3` captures;
- marginal/recovery `545M-DVB-T-公視-3`, `581mhz`, and
  `557mhz-vertical` captures;
- synthetic independent positive/negative SRO and CFO, linear drift, and
  reversal cases;
- 2K plus available guard-interval and bandwidth fixtures;
- repeated one-worker and automatic-worker runs for deterministic symbol and
  recovery boundaries;
- completed report and non-reporting Release runs, because the interrupted
  report used for this analysis cannot serve as the final throughput record.

Behavior-preserving changes require byte-identical TS on clean captures.
Tracking-aware or parallel RS changes must compare emitted packets, usable
non-TEI packets, RS/TEI counts, MER, timing confidence, branch rejection,
phase discontinuities, re-anchor positions, SRO command/applied error, queue
pressure, and deterministic recovery boundaries.

## Deferred or rejected shortcuts

Do not prioritize the following based on the current evidence:

- splitting the serial demod ownership loop into another stage;
- increasing the offline queue multiplier beyond the measured 4x default
  without a workload-specific A/B;
- adding Viterbi workers before separating Viterbi wait from RS coordinator
  time;
- TPS, CIR-analysis, CFO-tracking, or reacquisition optimization;
- transport-output changes: the report shows no output drops and negligible
  byte-sink queue occupancy;
- FFTW threading without a measured complete-pipeline gain.
