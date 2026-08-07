# Worker Pools and Data Flow

This document is the maintained architecture reference for the current DVB-T
receive path. It describes ownership, thread boundaries, ordering, reset
semantics, and queue policy. Historical pipeline versions and benchmark logs
belong in git history; remaining experiments are listed at the end.

## Architectural rules

- Device callbacks copy data into bounded consumers and return; they do not
  run DVB-T DSP.
- Stateful operations have a single ordered owner. Parallel pools are used
  only where work can be rejoined before the next stateful stage.
- Live input is bounded and may drop on overload. Offline decoding applies
  backpressure and must not drop input.
- Reset and retune advance a generation. Work from an older generation must
  not mutate current state or reach transport sinks.
- Common application and GUI code see only the standard-neutral
  `Demodulator`, `SignalSnapshot`, and `PipelineSnapshot` contracts.

## End-to-end flow

```text
ReceiverSession
  owns SdrDevice and the active Demodulator
        |
        v
Airspy callback / Soapy worker / I/Q-file worker
  +--> SpectrumAnalyzer                latest display data
  +--> RawIqRecorder                   independent disk queue
  `--> dvbt::StreamDecoder::submit     bounded CS16 input queue
             |
             v
       dvbt-frontend thread
         CS16 -> complex<float>
         StreamingResampler pool
         continuous absolute-position sample ring
             |
             v
       dvbt-demod thread
         acquisition and re-anchor
         NCO / FFT / pilot and CFO tracking / channel estimate / TPS
         payload extraction
         SymbolPostprocessorPool
         ordered symbol join and MER-region gate
             |
             v
       bounded FEC-item queue
             |
             v
       dvbt-fec thread
         stateful TransportDecoder
         SoftViterbi pool + ordered window join
         convolutional byte deinterleave
         RS(204,188) / energy descramble / TS recovery
             |
             v
       transport model + TS recorder + mpv/CLI sink
```

`ReceiverSession` preserves the decoder object for a same-standard retune and
performs an explicit replacement transaction when the receive standard
changes. `SdrDevice` owns the standard-neutral demodulator and routes its TS
output to common consumers.

The DVB-T decoder also owns a lightweight `SignalAnalyzer`. It is used only
while the production demodulator has not published a locked signal snapshot.
After lock, constellation, MER, SNR, notch, and carrier-offset telemetry come
from the actual demod/postprocessor path rather than an independent duplicate
calculation.

## Threads and worker pools

| Execution context | Responsibility | Ordering requirement |
| --- | --- | --- |
| Source callback/worker | Submit I/Q to spectrum, demodulator, and recorder | Must return promptly |
| `dvbt-frontend` | Convert CS16, resample, and fill the sample ring | Serial input order |
| Resampler pool | Process independent output ranges | Rejoined exactly before ring write |
| `dvbt-demod` | Own acquisition, symbol position, carrier/timing loops, channel state, and TPS | Strict symbol order |
| Symbol pool (`dvbt-sym-*`) | Reliability, MER, demap, deinterleave, and depuncture independent symbols | Results joined by sequence |
| `dvbt-fec` | Own decoder regions, outer FEC, TS emission, and final stream-end delivery | Strict FEC/TS order |
| Viterbi pool (`dvbt-vit-*`) | Decode overlapping mother-code windows | Results joined by sequence before outer FEC |
| Recorder workers | Write raw I/Q or TS without blocking source/DSP workers | Preserve submitted byte order |

The named DVB-T threads are visible in tools such as `htop`. Pool instances
are persistent for a compatible stream configuration; they are not recreated
for every block or symbol.

## Worker budget

`ReceiverParameters::worker_threads` defaults to
`std::thread::hardware_concurrency()` and is fixed while a source is open.
For a normal budget greater than two threads, allocation is:

```text
resample = floor(total / 4)
symbol   = floor((total - resample) / 2)
viterbi  = total - resample - symbol
```

This gives the intended 2/8 resample, 3/8 symbol, and 3/8 Viterbi split when
the budget is divisible by eight. Every pool receives at least one worker; a
budget of one or two therefore still creates one worker for each stage and is
not a literal cap on total decoder threads. The three coordinator threads and
source/recorder/UI threads are outside this DSP worker budget.

## Stage responsibilities

### Source and input queue

Airspy calls `submit()` from its callback. SoapySDR and file playback call it
from their source workers. `submit()` copies valid CS16 blocks into a queue
sized to roughly 0.2 seconds of input, with capacity enlarged to fit one
incoming block.

- Live `submit()` rejects a block when the queue is full and increments
  `dropped_blocks`.
- Offline `submit_blocking()` waits for capacity.
- Source-reported sample loss requests an asynchronous decoder reset so the
  following samples cannot be concatenated to stale DSP state.

### Frontend and sample ring

`dvbt-frontend` converts interleaved CS16 to normalized complex samples and
runs `StreamingResampler` at the DVB-T baseband rate (`bandwidth * 8 / 7`).
The common arbitrary resampler retains Q32.32 phase and FIR history across
input blocks. Caller blocks are split into bounded 50 ms processing quanta;
parallel workers evaluate independent output ranges inside each quantum and
rejoin exactly before publication. DVB-T configures its passband through the
outermost active carrier and its stopband at the lower of the input and output
Nyquist edges. The Kaiser filter is automatically sized and SIMD-aligned for
an 80 dB stopband target.

The source assigns every input block a monotonic `uint64_t` sample range and a
stream epoch. Resampler spans retain the corresponding input range, absolute
ring output range, and applied SRO correction. The timing loop schedules each
new ratio at a future input-sample boundary beyond the maximum generated-ahead
lead, so queue occupancy changes wall-clock delivery time but not the
signal-domain control delay.

Resampled data enters an `AbsoluteSampleRing`. Read and write positions are
monotonic 64-bit stream coordinates; wrapping affects storage only. The ring
holds roughly 0.2 seconds at the active rate and has a 1,048,576-sample floor.
It is resized only while empty. When full, the frontend waits for the demod
thread rather than overwriting unread samples.

### Serial demod and tracking

`dvbt-demod` is the owner of all state that depends on symbol history:

- initial acquisition and event-driven cold re-anchor;
- useful-symbol position and timing state;
- CFO NCO, residual phase loop, and integer carrier grid;
- FFT, continual/scattered-pilot tracking, and channel estimation;
- adaptive CIR-based FFT-window placement;
- differential TPS state and decoder configuration;
- ordered delivery into the MER gate and FEC queue.

The normal path does not repeatedly perform a full carrier/TPS search. It
keeps the established grid, verifies the rotating pilot phase, tracks sub-bin
CFO, and validates TPS at expected frame boundaries. Fades freeze vulnerable
tracking state; prolonged failure can request a bounded re-anchor at a symbol
boundary.

Timing and clock-loop details are maintained in
[clock-tracking.md](clock-tracking.md). They should not be duplicated here
because their thresholds and experimental states change more frequently than
the ownership model.

### Symbol postprocessing pool

Once the demod thread has produced a payload-carrier symbol, independent work
is dispatched to `SymbolPostprocessorPool`:

- decision-directed gain and error/MER measurement;
- per-carrier reliability;
- Max-Log QPSK/16-QAM/64-QAM demapping;
- DVB-T symbol and bit deinterleaving;
- depuncturing and soft-metric quantization.

Workers may finish out of order. Sequence numbers restore symbol order before
the windowed MER gate and stateful transport decoder. The demod thread consumes
results at a fixed 68-symbol barrier, so MER-driven freeze and re-acquisition
decisions occur at the same signal position regardless of worker completion
timing. Pool backpressure counts queued, running, and completed-but-unconsumed
symbols together. A TPS-frame-sized gate brackets hopeless regions with FEC
end/begin items so corrupted state does not poison later recovery.

### FEC and transport

`dvbt-fec` consumes ordered region-control, symbol, statistics, and stream-end
items. It owns the stateful `dvbt::Decoder`/`TransportDecoder` and is the only
thread that publishes recovered TS bytes.

`SoftViterbi` divides the mother-code metric stream into overlapping windows
with traceback margins. The default optimized build uses the AVX2-u16
`ViterbiDecoderCpp` backend. Other compile targets select SSE4.1, AArch64 NEON,
or the portable scalar backend. Each worker owns its mutable decoder context;
completed windows are joined in sequence before the stateful outer chain.

After Viterbi, processing is serialized:

```text
ordered decoded bytes
  -> 12-branch convolutional byte deinterleaver
  -> libfec RS(204,188)
  -> energy descrambler
  -> 188-byte MPEG-TS packets
```

Uncorrectable codewords retain packet cadence and are emitted with TEI set.
Outer-FEC phase acquisition and recovery remain inside this serialized stage.

For a finite input, `stream_end` travels through the FEC queue. The FEC thread
flushes and delivers final TS bytes first, then fires
`TransportDiscontinuity::stream_end`. This ordering is part of the sink
contract.

## Queues and backpressure

| Buffer | Policy | Current sizing |
| --- | --- | --- |
| StreamDecoder input | Live drop; offline block | About 0.2 s of source samples |
| Resampled sample ring | Frontend blocks | About 0.2 s, minimum 1,048,576 complex samples |
| Symbol/FEC-item queue | Demod blocks | About 0.2 s at current mode/bandwidth |
| Viterbi task queue | Producer blocks | `max(2 * workers, 1024)` windows |
| mpv TS queue | Drop oldest only at hard capacity; buffering hysteresis | 8 MiB capacity, 1 MiB low, 2 MiB resume |
| Raw I/Q recorder | Drop/reject on recorder overload | At least 5 s from active sample rate |
| TS recorder | Drop/reject on recorder overload | 24 MiB independent write queue |
| Spectrum/pre-lock analyzer | Latest-data behavior | Display-oriented, not a history queue |

The mpv queue and TS-recorder queue serve different purposes and must not
share watermarks. An empty playback queue is not itself a transport
discontinuity; retune, finite stream end, and FEC-region reset are explicit
out-of-band events.

## Ordering, generations, and reset

Parallel stages attach monotonically increasing sequence numbers and join
before the next stateful consumer. No downstream component may infer order
from worker completion time.

Reset/retune advances `latest_generation` and cancels the current generation:

1. queued input and FEC items are discarded;
2. the demod abandons its old absolute ring reader at a controlled boundary;
3. the frontend acknowledges the reset and rewinds the ring only after the old
   reader is no longer active;
4. workers may finish stale tasks, but generation checks suppress their state
   updates and output;
5. blocking `reset()` waits for the acknowledgement before a replacement
   source can submit fresh data.

This handshake prevents fresh samples from being swept by a delayed clear and
prevents a reader parked on an old absolute position from waiting forever
after ring rewind.

## Live and offline lifecycle

- Live GUI/capture uses `submit()`. Overload is visible through input queue
  pressure and `dropped_blocks`.
- `--decode-iq` uses `submit_blocking()`, then `flush()`. A nonzero dropped
  block count is an internal failure for this path.
- `flush()` closes the current finite ring tail, drains symbol and FEC work,
  emits final TS, and delivers `stream_end` from the FEC thread.
- A same-standard retune preserves the demodulator instance but resets its
  stream state and transport model. Standard replacement is owned by
  `ReceiverSession`.

## Telemetry interpretation

The common `PipelineSnapshot` currently publishes three GUI stages:

- `IQ queue`: input queue occupancy; its worker count is the resampler pool.
- `Demod`: measured busy fraction of the serial demod thread; its associated
  worker count is the symbol pool.
- `FEC queue`: queued FEC items; its worker count is the Viterbi pool.

Worker counts are configuration, not load. Detailed `StreamDecoderStats`
separates frontend conversion/resampling/ring wait, acquisition, serial demod
busy time, aggregate symbol-pool work, FEC work, and the transport subset.
`fec_work_time_ms` already contains `transport_work_time_ms`; they must not be
added. Asynchronous pool completions may cross a statistics-window boundary.

Signal telemetry after lock is published from production carriers. The
pre-lock analyzer is a fallback only. BER, RS/TEI counters, timing/SRO/CFO,
fade indication, queue depths, worker states, and event logs remain
DVB-T-specific diagnostics.

## Code map

| Concern | Primary files |
| --- | --- |
| Session and demodulator lifecycle | `src/receiver_session.cpp`, `src/receiver_session.hpp`, `include/airspy_tv/demodulator.hpp` |
| Source fan-out and common snapshots | `src/sdr.cpp`, `include/airspy_tv/sdr.hpp` |
| StreamDecoder coordination and public stats | `src/dvbt/stream_decoder.cpp`, `include/airspy_tv/dvbt/stream_decoder.hpp` |
| Frontend, demod session, tracking, and output helpers | `src/dvbt/stream_decoder_frontend.hpp`, `src/dvbt/stream_decoder_demod*.hpp` |
| OFDM acquisition and resampling | `src/dvbt/ofdm_acquisition.cpp`, `include/airspy_tv/dvbt/ofdm_acquisition.hpp` |
| TPS and inner decoding | `src/dvbt/tps_decoder.cpp`, `src/dvbt/inner_decoder.cpp`, `src/dvbt/soft_demapper.cpp` |
| FEC queue and transport decoder | `src/dvbt/stream_decoder_fec.hpp`, `src/dvbt/transport_decoder.cpp` |
| Viterbi pool/backend selection | `src/fec/soft_viterbi.cpp`, `include/airspy_tv/fec/soft_viterbi.hpp` |
| Outer FEC | `src/fec/outer_fec.cpp`, `include/airspy_tv/fec/outer_fec.hpp` |
| Signal snapshot publication/fallback | `src/dvbt/analysis_publisher.cpp`, `src/dvbt/signal_analyzer.cpp` |
| Playback and discontinuities | `src/mpv_player.cpp`, `include/airspy_tv/mpv_player.hpp`, `include/airspy_tv/transport_stream.hpp` |
| Raw/TS recording | `src/recorder.cpp`, `include/airspy_tv/recorder.hpp` |

## Remaining architecture work

- Add queue, ordering, lifecycle, and portable-backend regressions listed in
  [architecture-review-2026-08-06.md](architecture-review-2026-08-06.md).
- Validate worker allocation on smaller and non-uniform CPU topologies; the
  current fraction is a policy, not an automatically tuned optimum.
- Retain stage telemetry that identifies overload without exposing
  implementation-specific counters through common GUI code.
- Keep timing/CIR/second-order-loop experiments in
  [clock-tracking.md](clock-tracking.md).
- Preserve these ordering, generation, and backpressure invariants while
  splitting large implementation or GUI files.
