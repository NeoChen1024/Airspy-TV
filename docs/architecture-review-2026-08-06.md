# Architecture and Soundness Review

Date: 2026-08-06
Reviewed revision: `3c11053` (`dev`)
Scope: native DVB-T DSP/FEC pipeline, source ingestion, playback/recording,
transport metadata, concurrency, build policy, diagnostics, and tests.

## Executive assessment

The receiver has a sound high-level data-flow design. Live input is copied into
bounded queues, expensive DSP is moved onto persistent workers, independent
symbol and Viterbi work is re-ordered before it reaches stateful consumers, and
the outer FEC remains serialized. The recent continuous resampler and carried
OFDM tracking design are also materially better foundations than chunk-local
re-acquisition.

The code is not yet a safe target for broad structural refactoring, however.
The control plane does not have ownership rules as strong as the data plane.
Reset currently allows the front-end thread to mutate demod-owned state and
rewind the sample ring while the demod thread can still read both. ThreadSanitizer
also confirms an unsynchronized status write, and the portable libcorrect
Viterbi fallback concurrently runs a non-thread-safe global initializer. FFTW
plan creation/destruction is performed from multiple threads without planner
serialization. These should be fixed before moving code between modules.

Recommended posture: preserve the current DSP behavior and performance
baseline, make lifecycle and ownership explicit, establish sanitizer-clean
tests, and only then extract components from `stream_decoder.cpp` and
`main.cpp`.

## Current architecture

The implemented pipeline has three persistent serial stages and three worker
pools:

```text
Airspy / Soapy / file
        |
        v
source thread or callback
        |-- SpectrumAnalyzer (independent monitor)
        |-- RawIqRecorder
        `-- StreamDecoder input queue
                  |
                  v
          frontend thread
          CS16 conversion -> StreamingResampler pool -> sample ring
                  |
                  v
          demod thread
          acquisition -> NCO/FFT -> pilot/channel/TPS/timing tracking
                  |
                  v
          SymbolPostprocessorPool (ordered join)
          gain/MER -> demap -> deinterleave -> depuncture
                  |
                  v
          FEC queue -> FEC thread -> SoftViterbi pool (ordered join)
                  |
                  v
          byte deinterleave -> RS -> descramble -> MPEG-TS callback
                  |
                  +-- service/EPG models
                  +-- TS recorder
                  `-- mpv playback queue
```

`StreamDecoder::Impl` is the effective pipeline coordinator, but it also owns
most DSP algorithms, all queue state, acquisition/recovery policy, telemetry,
and callback dispatch. This is why its 3,900-line implementation is now the
main refactoring pressure point.

## Findings

### High: reset crosses ownership boundaries and can race active demodulation

Evidence:

- The front-end reset branch clears and rewinds the input queue, FEC queue,
  sample ring positions, sync state, and `FrontendState` in
  `src/dvbt/stream_decoder.cpp:1394`.
- `reset_frontend_state()` modifies tracking vectors, TPS state, carrier state,
  and other fields that the demod thread reads and writes without taking the
  shared mutex (`src/dvbt/stream_decoder.cpp:1310`).
- The demod copies FFT input from `ring` after releasing the mutex
  (`src/dvbt/stream_decoder.cpp:2760`). A concurrent reset may rewind the ring,
  and the next source block may resize it while this copy is in progress.
- `cancel_requested` is not checked in the normal symbol loop. The front-end
  may set it back to false after consuming the reset before the demod observes
  it.

Impact: these are C++ data races and possible invalid vector access, not only
stale telemetry. Current tests make the usual reset schedules work, but the
behavior is undefined when reset overlaps active symbol processing.

Required direction:

1. Make the demod thread the only writer of demod/tracking state.
2. Publish a reset generation/request; have each stage acknowledge it at a
   defined boundary.
3. Do not rewind or resize the ring until the demod has stopped reading the old
   generation.
4. Separate queue-control state from demod state instead of protecting both
   with one partially observed mutex.

### High: the portable Viterbi fallback has a confirmed initialization race

Each `SoftViterbi` worker creates its own libcorrect decoder concurrently at
`src/fec/soft_viterbi.cpp:359`. libcorrect's `bit_reader_create()` lazily fills
a process-global reverse table behind an ordinary function-local `bool`
(`contrib/libcorrect/src/convolutional/bit.c:176`). Multiple worker starts
therefore read and write both the flag and table concurrently.

ThreadSanitizer reproduced this race with the AVX2 backend disabled. It affects
portable builds and hosts where `AIRSPY_TV_USE_AVX2_VITERBI` is enabled but
`__AVX2__` is unavailable. The writes happen to be deterministic, but the C
memory model still makes the behavior undefined.

Required direction: initialize one libcorrect decoder context before launching
the pool, or serialize all context construction once. Patching the vendored
initializer with a thread-safe primitive is possible, but keeping the workaround
at the Airspy TV ownership boundary is smaller and easier to validate.

### High: FFTW planner use is neither serialized nor RAII-managed

The spectrum analyzer plans on construction, the GUI signal analyzer creates
and destroys an FFT plan for each analysis, and the demod creates/destroys main
and CIR plans on grid changes. FFTW permits concurrent execution of independent
plans, but planner creation and destruction require external serialization
unless its planner-thread-safety facility is explicitly enabled. The project
does neither.

Relevant sites are `src/dvbt/signal_analyzer.cpp:71`,
`src/dvbt/stream_decoder.cpp:1787`, and `src/spectrum.cpp:201`.

The two plans local to `run_demod()` are also raw handles. A normal stop returns
from the worker loop without destroying the current main FFT plan or
`frontend.cir_plan`; null plan results are not checked before later execution.

Required direction: introduce one small FFTW plan RAII wrapper and one
process-wide planner mutex (or a verified one-time FFTW thread-safe planner
initialization). Keep `fftwf_execute()` outside that mutex.

### Medium: additional shared-state races are already visible

ThreadSanitizer confirms that `frontend_busy` is written without the decoder
mutex on the flush path (`src/dvbt/stream_decoder.cpp:1450`) while
`wait_until_idle()` reads it under the mutex. `ring_closed` is likewise read
outside the mutex in multiple control-flow decisions.

The monitor paths have the same pattern: `SignalAnalyzer::submit()` reads
`next_analysis` before acquiring `pending_mutex`, and `SpectrumAnalyzer::submit()`
does the same with `next_capture`; reset writes those values while holding the
mutex. UI-triggered reset can therefore race a source submission.

These fields should either be consistently mutex-owned or atomic. Using
atomics for an entire lifecycle protocol is not recommended; the state
transitions and their associated buffers should stay under one explicit owner.

### Medium: callback and source control contracts can block or deadlock

The architecture document says device callbacks only copy samples. The Airspy
callback instead calls blocking `Demodulator::reset()` when dropped samples are
reported (`src/sdr.cpp:114`). This can extend callback latency after an
overflow and cause another overflow. The callback should enqueue a discontinuity
or reset request and return.

`SdrDevice::set_demodulator()` replaces the owned decoder without stopping or
synchronizing a running source. The current application only calls it before
streaming, but the public API does not enforce that precondition and a future
standard switch could race a source dereference.

The MPEG-TS fanout also invokes the external `transport_sink` while holding
`transport_sink_mutex` (`src/sdr.cpp:721`). A sink that calls back into
`set_transport_sink()` deadlocks, and all EPG/player work extends FEC callback
latency. Copy the callable under the mutex and invoke it after unlocking.

### Medium: GUI signal quality is a second demodulator, not pipeline telemetry

`StreamDecoder` owns a low-rate `SignalAnalyzer` that independently resamples,
acquires, FFTs, locks pilots, equalizes, and computes MER/SNR. It does not
observe the carried CFO, timing loop, adaptive CIR placement, pilot branch, or
actual postprocessor decisions used to produce TS.

This is safe as a best-effort pre-lock monitor, but it has two architectural
costs:

- GUI MER/constellation/lock can disagree with the path that is actually
  decoding.
- It duplicates acquisition and FFT work and introduces the FFTW planner race
  above.

The long-term source of truth should be snapshots published from the real
demod/postprocessor path. A small independent acquisition monitor can remain
only for the pre-lock state if that behavior is still useful.

### Medium: discontinuity ordering is not a defined stream contract

At finite-stream completion the demod enqueues an FEC `end` item, immediately
fires `TransportDiscontinuity::stream_end`, and only later does the FEC thread
flush and emit its final TS bytes (`src/dvbt/stream_decoder.cpp:3544`). A
consumer therefore cannot interpret `stream_end` as "all preceding transport
bytes have been delivered," even though the enum documentation reads like an
ordered stream event.

Today mpv does not close the source on that event, which avoids losing the FEC
tail but also means the event itself cannot provide EOF. Define one serialized
output event stream, or fire `stream_end` from the FEC thread after the final TS
callback.

### Medium: worker exceptions terminate the process instead of failing a stage

Symbol and Viterbi pools capture worker exceptions, but the demod rethrows an
exception from its `std::thread`, the front-end and FEC loops have no top-level
exception boundary, and transport/equalized/discontinuity callbacks may throw.
Any such exception calls `std::terminate` rather than transitioning the decoder
to a visible failed state and releasing blocked producers.

For a local receiver, fatal failure may be an acceptable policy, but it must be
intentional and consistent. A more useful policy is a stored `exception_ptr`
plus a terminal pipeline state that wakes every condition variable and exposes
the error through stats/runtime status.

### Medium: component boundaries have drifted after optimization

There are now two partially overlapping post-equalization APIs:

- `Decoder::process_symbol()` performs demap and deinterleave serially.
- `StreamDecoder` performs the same work in `SymbolPostprocessorPool` and calls
  only `Decoder::process_soft_metrics()`.

Likewise, `Cs16Resampler` tests the finite acquisition resampler, while the
continuous `StreamingResampler` is private to `stream_decoder.cpp` and has no
direct serial-equivalence test across arbitrary submission boundaries.

These are understandable products of incremental optimization, but keeping
both paths indefinitely invites behavioral drift. Select one production path,
retain lower-level APIs only where tests/tools need them, and test the actual
streaming implementation directly before extraction.

### Low: build and test policy hides portability and concurrency risk

- Default Debug is `-O3 -DNDEBUG`; assertion-enabled Debug requires an opt-out.
  This is documented, but CI should include both optimized and assertion builds.
- Global `-march=native` makes the default build host-specific. The fallback is
  documented but currently exercises the libcorrect race above.
- The libcorrect subdirectory is configured by temporarily changing the global
  `CMAKE_BUILD_TYPE`, which is fragile for multi-config generators.
- Four tests cover inner decoding, transport/FEC, stream integration, and EPG,
  but there are no playback, recorder, source-lifecycle, analyzer-concurrency,
  or transport-model tests.
- Stream reset tests use a fixed one-second sleep. Under ThreadSanitizer the
  mid-wait reset test becomes timing-dependent and fails before a complete
  sanitizer pass. Tests should wait for an observable worker/ring state with a
  bounded timeout.

## What is already sound

The following design decisions should be preserved during cleanup:

- Live submission is bounded and allowed to drop, while offline submission
  applies backpressure. This matches the physical source semantics.
- The resampler, symbol postprocessor, and Viterbi pools are persistent rather
  than recreated per block.
- Out-of-order parallel work is joined by sequence before reaching stateful
  consumers.
- The byte deinterleaver, RS alignment, energy descrambler, and TS emission stay
  on one FEC thread.
- FEC items carry a stream generation, preventing queued stale symbols from
  crossing a reset after they reach the FEC queue.
- Queues and the sample ring are capacity-bounded, and telemetry exposes their
  watermarks and worker states.
- PSI/SI parsing validates TS sync, TEI, continuity, section length, and CRC
  before updating models.
- The integration test verifies a known contiguous TS run, reset/reopen, closed
  non-acquirable input, worker allocation, timing telemetry, and discontinuity
  emission.
- The full-capture and synthetic drift baselines documented in `AFC-plan.md`
  provide valuable behavioral acceptance criteria. They should remain unchanged
  while architecture-only work is underway.

## Refactoring plan

### Phase 0: freeze the baseline

Record the current 20-second drift fixture output, short integration-test
timings, and one representative live/offline throughput result. Do not combine
DSP tuning with ownership cleanup.

### Phase 1: make lifecycle behavior defined

1. Replace the reset booleans with an explicit generation/state protocol and
   per-stage acknowledgements.
2. Assign one writer to the sample ring and one owner to demod tracking state.
3. Fix all TSAN-reported project races and the libcorrect fallback initializer.
4. Add FFTW plan RAII and planner serialization.
5. Define callback threading, exception, and discontinuity ordering contracts.

Acceptance: optimized tests pass, assertion-enabled tests pass, and the stream
test completes under TSAN without project or vendored-library race reports.

### Phase 2: extract behavior-preserving DSP components

Extract in dependency order, with no algorithm changes:

1. `StreamingResampler` plus a direct serial/parallel, cross-block equivalence
   test.
2. A bounded absolute-position sample ring with reset/close semantics tested in
   isolation.
3. `TimingSlopeTracker` and the sample-clock controller, using recorded
   telemetry vectors as deterministic tests.
4. `SymbolPostprocessor` and its ordered pool.
5. OFDM tracking state (NCO, pilot lock, channel/CIR, TPS) behind one
   demod-thread-owned object.
6. FEC stage orchestration and ordered output events.

After this phase, `StreamDecoder::Impl` should coordinate components and queues;
it should not contain their DSP implementations.

### Phase 3: make observability follow the real path

Publish immutable analysis snapshots from the demod and symbol postprocessor.
Use those for GUI constellation, MER, carrier/timing, and lock status. Decide
separately whether the independent pre-lock analyzer still earns its CPU and
maintenance cost.

Telemetry should distinguish counters, gauges, window aggregates, and
cumulative timings in their types/names. Avoid one large mutable stats object
written by several stages.

### Phase 4: split application orchestration

Split `main.cpp` by responsibility:

- CLI parsing and offline decode/record commands;
- receiver/session controller;
- GUI state and individual panels;
- decoder diagnostics formatting.

Keep `SdrDevice` responsible for source lifetime, but move TS fanout to a small
explicit sink/router object so playback, recording, service discovery, and EPG
do not execute under a source-owned mutex.

### Phase 5: broaden regression coverage

Add:

- 2K end-to-end stream coverage and all guard-interval/mode transitions;
- continuous resampler equivalence across random block sizes and rate changes;
- concurrent submit/reset/flush/stop stress tests;
- analyzer reset/submit TSAN coverage;
- portable non-AVX Viterbi coverage;
- mpv queue hysteresis and discontinuity ordering tests with a fake reader;
- recorder short-write/failure tests;
- PAT/PMT/SDT version/change and malformed-section tests;
- retained synthetic SRO/CFO fixtures as opt-in longer regressions.

## Implementation status

Phases 1 through 4 were implemented after this review, without intentional DSP
algorithm changes:

- Reset now uses a generation and demod acknowledgement handshake. Input blocks
  carry their generation, stale FEC output is suppressed after decode, the
  demod owns OFDM tracking state, and ring rewind waits until the demod has
  abandoned the old generation.
- FFTW plans use one move-only RAII wrapper with serialized planner lifecycle.
  Analyzer submission timestamps are mutex-owned, and portable libcorrect
  decoder construction is serialized.
- Source callbacks can publish a nonblocking reset request. Decoder worker
  exceptions enter a visible terminal state and wake blocked pipeline waits.
  Finite-stream discontinuities are emitted by the FEC thread after final TS
  bytes. The GUI reports terminal decoder failures instead of presenting them
  as an ordinary acquire/idle state.
- Reset completion now wakes every idle waiter, including the transition from
  a reset boundary into an empty first-acquisition wait. Invalid generations
  return directly to the parked state instead of briefly treating an empty
  ring as active acquisition work.
- Streaming resampling, ordered symbol postprocessing, timing-slope tracking,
  absolute-position ring storage, OFDM tracking state, and FEC queue events are
  separated from the coordinator into internal components.
- GUI constellation, MER, CP SNR, channel-notch, and carrier-offset snapshots
  come from the production demodulator and postprocessor after lock. Production
  snapshots retain the configured smoothing behavior. The independent
  analyzer remains only as a pre-lock monitor and stops receiving work once
  production telemetry is available.
- MPEG-TS fanout is owned by `TransportStreamRouter`, outside source-owned
  locks. GUI, CLI command handlers, decoder diagnostics, and CLI parsing are
  split out of `main.cpp` by responsibility.

Phase 5 remains intentionally deferred. The existing stream cases are isolated
as separate CTest processes so assertion, portable, and sanitizer builds can
run the same reset/lifecycle scenarios without one sanitizer process retaining
all test metadata.

## Validation performed for this review

- Optimized build: 8/8 CTest cases passed.
- Assertion-enabled Debug build: 8/8 CTest cases passed.
- Portable Debug build with AVX2 Viterbi disabled: 8/8 CTest cases passed.
- GCC ThreadSanitizer build with AVX2 Viterbi disabled: 8/8 CTest cases passed
  without a project or vendored-library race report.
- The TSAN `reset-new` lifecycle case passed 10 consecutive repetitions. This
  stress run exposed and then verified the fix for a missed idle notification
  after reset acknowledgement.
- The standard 20-second independent sample-clock/LO-clock fixture passed:
  measured SRO `-0.0844 ppm` versus expected `-0.1089 ppm`, measured CFO
  `-107.69 Hz` versus expected `-107.47 Hz`, and `37,276,828` TS bytes.

Not performed: a full 363.9 GB capture replay, live SDR hardware testing, GUI
interaction testing, or a complete ASan/UBSan/LSan pass. Phase 5 coverage
expansion also remains deferred.
