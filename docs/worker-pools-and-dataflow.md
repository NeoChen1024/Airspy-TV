# Worker Pools and Data Flow

This document describes how the `airspy-tv-dvbt` decoder library and the
`airspy-tv` application are structured around bounded queues, ordered worker
pools, and a continuous three-stage CS16 → MPEG-TS pipeline. It covers the
streaming I/Q resampler, the OFDM front end, Max-Log QAM demodulation, the
convolutional/RS FEC chain, and the windowed MER gate.

The single architectural rule: **no DSP ever runs inside a device callback.**
Every source callback (Airspy, SoapySDR, file) only copies samples into a
bounded queue and returns. All processing happens on dedicated worker threads
and pools.

## 1. End-to-end overview

```text
Airspy native  ─┐
SoapySDR       ─┤ (callback / worker thread)
I/Q file       ─┘        │ copies CS16 blocks, no DSP
                         ▼
        ┌────────────────────────────────────────────┐
        │  SdrDevice::Impl (sdr.cpp)                 │
        │  SpectrumAnalyzer.submit      (FFT display)│
        │  SignalAnalyzer.submit        (GUI monitor)│
        │  StreamDecoder.submit         (this doc)   │
        │  RawIqRecorder.submit         (I/Q capture)│
        └────────────────────────────────────────────┘
                         ▼ StreamDecoder (bounded input queue, ~200 ms)
        ┌─────────────────────────────────────────────────────────┐
        │ front-end thread (StreamDecoder::Impl::frontend_thread) │
        │   Stage 1  StreamingResampler (one persistent filter)   │
        │   Stage 2  acquisition monitor (start + every ~2 s)     │
        │            └─ publishes SyncState to the demod thread   │
        │   ring buffer of resampled samples (1 Mi complex,       │
        │   absolute uint64 stream positions)                     │
        └─────────────────────────────────────────────────────────┘
                         ▼ ring
        ┌─────────────────────────────────────────────────────────┐
        │ demod thread (StreamDecoder::Impl::demod_thread)        │
        │   Stage 3  continuous per-symbol loop: NCO, FFT,        │
        │            pilot/CFO tracking, channel, TPS (carried),  │
        │            payload extraction                           │
        │   Stage 4  → SymbolPostprocessorPool (≈1/3 budget)      │
        │            gain, MER, reliability, Max-Log demap,       │
        │            symbol/bit deinterleave, depuncture,         │
        │            soft-byte quantization                       │
        │   windowed MER gate (68-symbol windows, end/begin FEC   │
        │   resets at hopeless region edges)                      │
        └─────────────────────────────────────────────────────────┘
                         ▼ FEC queue (bounded, ~200 ms of symbols)
        ┌─────────────────────────────────────────────────────────┐
        │ FEC thread (StreamDecoder::Impl::fec_thread)            │
        │   stateful Decoder (region generations)                 │
        │   └─ Stage 5  TransportDecoder                          │
        │        SoftViterbi pool (≈2/3 budget)                   │
        │        12-branch byte deinterleaver                     │
        │        RS(204,188) → energy descrambler → TS packets    │
        └─────────────────────────────────────────────────────────┘
                         ▼ sinks
        transport_model (service discovery) · TS recorder · player / CLI output
```

The pipeline is **continuous**: the resampled symbol stream has no seams, so
the CFO loop, integer carrier offset, continual reference, and TPS superframe
decoder all carry for the life of a stream. There is no per-chunk re-acquisition
(the monitor only re-anchors on boundary moves, mode/guard changes, resets, or
long fades) and no TS overlap dedup.

## 2. Thread and pool inventory

| Thread / pool                                          | Where                 | Count           | Work                                                          |
| ------------------------------------------------------ | --------------------- | --------------- | ------------------------------------------------------------- |
| `SdrDevice::airspy_rx_callback`                        | sdr.cpp               | libairspy-owned | copies one block to 4 bounded sinks                           |
| `run_soapy` / `run_file`                               | sdr.cpp               | 1 each          | blocking read loop, same fan-out                              |
| `StreamDecoder::Impl::frontend_thread`                 | stream_decoder.cpp    | 1, persistent   | resample, acquisition monitor, ring producer                  |
| `StreamDecoder::Impl::demod_thread`                    | stream_decoder.cpp    | 1, persistent   | continuous symbol loop, tracking, pool dispatch, MER gate     |
| `StreamDecoder::Impl::fec_thread`                      | stream_decoder.cpp    | 1, persistent   | stateful FEC decode per region, callback                      |
| `SymbolPostprocessorPool`                              | stream_decoder.cpp    | S ≈ N/3         | per-symbol postprocessing                                     |
| `SoftViterbi` pool                                     | transport_decoder.cpp | V = N − S       | overlapping Viterbi windows                                   |
| `SignalAnalyzer` worker                                | signal_analyzer.cpp   | 1               | one-symbol GUI monitor snapshot                               |
| `RawIqRecorder` / `TransportStreamRecorder` writer     | recorder.cpp          | 1 each          | disk I/O, 5 s / 24 MiB buffers                                |

The front-end and demod threads are split so resampling and acquisition never
block symbol extraction (or vice versa). The `fec_thread` is intentionally
**stateful**: the outer deinterleaver phase, RS alignment, and
energy-descrambler phase must run serially, so they live on a single thread
that never competes with the symbol pools.

## 3. Worker budget allocation

`ReceiverParameters::worker_threads` (0 = auto) is the only knob. It is
**fixed before a source opens**; changing it reconstructs both pools.

`allocate_workers(total)` in stream_decoder.cpp:

```text
total   = requested == 0 ? hardware_concurrency : requested
if total <= 1             -> { symbol = 1, viterbi = 1 }
symbol  = max(1, total / 3)          (SymbolPostprocessorPool)
viterbi = max(1, total - symbol)     (SoftViterbi pool)
resample workers = 1                 (streaming resampler is serial)
```

Example at 16 threads: resampler 1, symbol pool 5, Viterbi pool 11.

## 4. Source paths (sdr.cpp)

Every block from every backend is fanned out to the same four sinks:

```cpp
analyzer.submit(block, rate, bandwidth);        // spectrum display
signal_analyzer.submit(block, rate, bandwidth); // GUI monitor
stream_decoder.submit(block, rate, bandwidth);  // this pipeline
recorder.submit(block);                          // raw CS16 capture
```

The `StreamDecoder` transport callback (set in `Impl`'s constructor) fans the
recovered TS out again:

```cpp
transport_model.consume(ts);   // PAT/PMT/SDT service discovery
ts_recorder.submit(ts);        // MPEG-TS capture
transport_sink(ts);            // mpv player (GUI) or file write (--decode-iq)
```

On `dropped_samples` (Airspy) or `SOAPY_SDR_OVERFLOW`, the monitor and decoder
are `reset()` and the raw recorder's `source_dropped_samples` counter is
incremented; the pipeline re-acquires cleanly instead of decoding garbage.

## 5. Stage-by-stage data flow

### 5.1 Input queueing (front-end thread)

- `submit()` copies into `std::deque<Block>`; `submit_blocking()` waits for
  capacity instead of dropping (offline path). Capacity =
  `sample_rate / 5` (≈ 200 ms). Live callbacks use `submit()`; drops are
  counted in `StreamDecoderStats::dropped_blocks`.
- The front-end thread pops blocks, converts CS16→complex (VOLK), and feeds
  them to the streaming resampler. A `flush()` closes the ring so the demod
  drains the remaining symbols; submitting after a flush reopens the ring and
  the demod continues seamlessly (replays stay on the same grid).

### 5.2 Stage 1 — streaming resampling (`StreamingResampler`)

- Target rate is the DVB-T nominal baseband rate: `bandwidth × 8/7`.
- One liquid-dsp `rresamp_crcf` rational resampler (`create_kaiser(p, q, 12,
  −1, 60)` with `p = (bw×8)/gcd`, `q = (rate×7)/gcd`) whose polyphase filter
  state and input accumulator carry across the whole capture. Input is
  accumulated until a full decimation block is available, then executed in
  blocks; there is no per-chunk filter reset, so the output is one continuous
  stream (the old partitioned resampler re-warmed 16 filter states on every
  7 M-sample chunk). Resampling is memory-bandwidth-bound, so the single
  serial filter is both simpler and correct.

### 5.3 Stage 2 — acquisition monitor (`acquire_ofdm`)

- Runs once on the head of the first resampled block, then every ~2 s on the
  last `acquisition_samples = 350,000` resampled samples (a rolling window).
- Searches cyclic-prefix periodicity across all allowed (or user-selected)
  `TransmissionMode` × `GuardInterval` combinations, returning the best
  `{start, fft_size, guard_size, phase, score}`.
- `score < 0.20` → keep the current sync (the demod keeps tracking).
- The result is published to the demod thread as a `SyncState` with a version
  counter. The version bumps only on mode/guard changes or a boundary move
  beyond a symbol's alignment tolerance, so ordinary estimator drift is a
  silent refresh and never disturbs continuous tracking. A validated
  mode/guard is cached (`stable_mode`/`stable_guard`) and reused across
  transient misses.
- Long fades: when the demod has been frozen for ~470 symbols (~0.7 s, the old
  chunk cadence) it requests an immediate forced acquisition; the publish
  forces a cold re-anchor (§5.4.5).

### 5.4 Stage 3 — continuous per-symbol tracking loop (demod thread, serial)

The demod reads symbols from the ring at absolute positions
`sync.start_pos + guard_size + n × period` (period = fft + guard). The grid is
a pure counter, so it stays aligned with the transmitter's symbol lattice for
the whole capture; re-anchors only re-seed tracking state, they never move the
grid. Per symbol:

1. **NCO**: mix the FFT window with a complex oscillator at the tracked CFO
   (`tracked_cfo_phase`), renormalized every 512 taps. The NCO phase is an
   absolute accumulator carried across symbols.
2. **FFT**: `fftwf` forward transform (plan created once per mode).
3. **Continual correlation + fade gate**: the continual-pilot temporal
   correlation drives the CFO loop (loop gain 0.20) only when the symbol pair
   is contiguous (`start == last_symbol_start + period`) and the normalized
   correlation is healthy. During a deep fade the correlation collapses and
   the CFO loop, pilot phase, and carrier lock all **freeze on their carried
   values** — the old chunked pipeline got this for free (a failed chunk
   acquisition skipped the whole chunk), the continuous demod must do it
   itself, otherwise noise-latched offsets and drifted CFOs leave the
   demodulator permanently rotated when the signal returns.
4. **Pilot lock**: correlate scattered pilots (PRBS-modulated, ±4/3) to find
   the 4-phase scattered-pilot phase and integer carrier offset (first lock
   ±48 bins, afterwards ±2). While frozen, the carried phase/offset are used
   verbatim; every 68 frozen symbols a wide re-lock runs and is accepted only
   if the phase-coherent scattered-pilot correlation verifies (≥ 0.40),
   otherwise the carried values are kept.
5. **Channel estimation**: divide received pilots by the known PRBS value, then
   linearly interpolate between pilots per phase (pilot indices are cached per
   mode once per stream). Per-carrier `equalizer_power = |ĥ|²` is kept. A
   fractional timing estimate (pilot phase slope) is measured per symbol as a
   sample-clock diagnostic (not yet corrected — see ROADMAP).
6. **TPS**: decode the TPS carriers. The TPS decoder is **carried** (never
   reset except on cold starts), because the contiguous symbol sequence keeps
   its differential frame sync intact — the key weak-signal win over the
   per-chunk re-lock. If no manual constellation/code rate was set, a locked,
   non-hierarchical TPS frame auto-selects the decoder parameters.
7. **Payload extraction**: the 1512 (2K) or 6048 (8K) payload carriers are
   equalized (`× ĥ⁻¹`) and submitted to the symbol pool together with their
   reliability inputs. Before TPS lock, up to 136 symbols are buffered
   (`pending_symbols`) with a fallback symbol index (pilot phase); once TPS
   locks, buffered symbols are re-indexed from the TPS frame index so
   deinterleaving stays coherent across the lock boundary.

#### 5.4.5 Cold re-anchor (fade recovery)

When the demod has been frozen for ~470 symbols it requests a forced
acquisition. The front-end publishes the fresh sync (version bumped) and the
demod cold-reanchors: the CFO is re-seeded from the acquisition's CP phase, the
pre-fade carrier grid is restored (the LO never moves during a fade, so the
restored offset/phase are deterministic — the ambiguous wide pilot lock is not
trusted), the TPS superframe is reset for a clean re-lock, and the phase is
advanced by the full fade length mod 4 (the scattered-pilot phase rotates once
per symbol). This makes recovery deterministic: full-capture runs decode
byte-identical TS.

### 5.5 Stage 4 — symbol postprocessing (`SymbolPostprocessorPool`)

A `PostprocessedSymbol` carries `{carriers, reliabilities, mother_metrics, symbol_index, mer_db, per-stage timings}`. Each pool worker, per task:

1. **Decision-directed gain** (2 iterations): slice to nearest constellation
   point, solve a least-squares complex gain, divide it out.
2. **MER / reliability**: mean squared error vs. the sliced constellation gives
   MER; the reliability per carrier is

   ```text
   reliability = (median-error-derived scale) × clamp(ĥ_median / ĥ_k, 0.01, 16)
   ```

   i.e. the effective post-equalization inverse-noise-variance
   (`|H|²/σ²`-style), so carriers in deep fades produce low-confidence metrics.
3. **QAM demod**: `MaxLogDemapper` (QPSK/16-QAM/64-QAM) emits per-bit soft LLRs,
   sample-major, MSB-first (positive = bit 1).
4. **Symbol deinterleave** then **bit deinterleave** (both native, 2K/8K),
   preserving soft metrics.
5. **Depuncture**: restore the rate-1/2 mother code (rates 1/2, 2/3, 3/4, 5/6,
   7/8); punctured bits get a neutral metric (128).
6. **Quantize** to `uint8_t` (`127.5 + llr×8`, clamped 0..255) so the FEC side
   shares one soft format with `SoftViterbi`.

Results may complete **out of order**; an ordered join (§7) hands
`mother_metrics` blocks to the FEC thread in original sequence.

**Windowed MER gate**: postprocessed symbols are held in 68-symbol windows (one
TPS frame). A window whose mean MER falls below the constellation floor (QPSK 5
/ 16-QAM 10 / 64-QAM 14 dB + 4 dB margin) is hopeless — deep fades — and is
dropped; an `end`/`begin` FEC pair is enqueued at the region edges so the
trellis never grinds through noise and the recovered region starts on a fresh
decoder. Tracking is unaffected, and faded-head/recovered-tail regions still
decode. A window reports its statistics every 400 symbols (~0.6 s) and enqueues
a lightweight `stats` item so the FEC thread can publish its counters.

### 5.6 Stage 5 — FEC path (FEC thread + `SoftViterbi` pool)

`Decoder::process_soft_metrics()` → `TransportDecoder::process_soft()`:

1. **`SoftViterbi` pool**: the mother-code metric stream is cut into
   overlapping 8192-bit windows (256-bit traceback margin each side → 7680
   output bits/window). Each worker owns an independent libcorrect
   convolutional decoder (K=7, rate-1/2; stored for libcorrect's convention
   as polynomials 0117/0155 octal — the bit-reversed DVB-T 171/133 — with an
   SSE decoder when `HAVE_SSE`). The queue holds `max(2×workers, 1024)`
   windows (≈ 200 ms+).
2. **Pre-Viterbi BER**: each window's survivor path is re-encoded and compared
   with hard decisions from the received metrics (neutral 128 metrics skipped).
3. **Outer deinterleave**: a 12-branch, step-17 convolutional byte
   deinterleaver. The correct branch phase is **acquired** by trying all 12
   phases, running RS trials, and keeping the phase with the most RS successes
   (≥4) / best sync distance; alignment is then held for the rest of the
   generation.
4. **RS(204,188)**: shortened Reed–Solomon over GF(256), t=8. Successes count
   post-Viterbi corrected bits; uncorrectable codewords are NOT dropped —
   they are cadence-preserved with the TEI bit set (§ energy descrambler).
5. **Energy descrambler**: the DVB PRBS (init 0xA9) with 8-packet frames
   (sync 0xB8 at frame start, 0x47 elsewhere). Output TS sync is normalized to
   0x47. `process_corrupt()` keeps cadence for uncorrectable packets and marks
   the payload corrupt instead of creating continuity-counter gaps.
6. Output: contiguous 188-byte TS packets, emitted directly to the
   `TransportCallback` (no chunk buffering or overlap dedup — the stream has
   no seams).

### 5.7 MER-gate region resets and flush

`FecItem` kinds are `begin` (create/reset the decoder for a region), `symbol`
(one symbol's soft metrics), `end` (flush the decoder tail at a region end or
stream end), and `stats` (publish transport counters for a stats window).
`begin` items after the first only occur at hopeless-region recoveries and
after a flushed stream is resumed, so the stateful `Decoder` lives across
whole regions instead of being torn down every 0.7 s.

## 6. Queues and backpressure

All queues are **bounded by stream duration, not item count** (≈ 200 ms per
pipeline stage), so memory stays flat and scheduler overload surfaces as
dropped blocks instead of unbounded growth:

| Queue               | Capacity                                                                         | Producer → consumer               |
| ------------------- | -------------------------------------------------------------------------------- | --------------------------------- |
| Input blocks        | `sample_rate / 5` complex samples                                                | source thread → front-end thread  |
| Resampled ring      | 1 Mi complex samples (~150 ms at 8K/guard-1/4) — the front-end pushes            | front-end → demod thread          |
|                     | incrementally and paces itself against the demod's consumption                   |                                   |
| FEC items (symbols) | `buffered_symbol_count(bw, symbol_duration)` ≈ 1/5 s of symbols (initial 256)    | demod → FEC thread                |
| Viterbi windows     | `max(2×workers, 1024)` windows                                                   | FEC thread → pool                 |
| Raw I/Q recorder    | 5 s of samples                                                                   | source thread → writer thread     |
| TS recorder         | 24 MiB                                                                           | FEC thread → writer thread        |
| mpv playback        | 24 MiB                                                                           | FEC thread → mpv queue            |

The ring holds absolute stream positions (`uint64`); the demod only frees what
it has consumed (`ring_read_pos`), so the front-end can run ahead without ever
overwriting unread samples. The spectrum and signal monitors are
latest-snapshot mailboxes (no buffering — buffering old displays only adds GUI
latency).

## 7. Ordering and joins

- **Symbol pool**: tasks carry a monotonic `sequence`; results land in a
  `std::map<sequence, result>` and the demod drains via
  `take_ready()`/`flush()` only in order (`next_result_`).
- **Viterbi pool**: identical pattern — `completed_` keyed by window sequence,
  `take_ready_locked()` joins windows in order before outer deinterleaving.
- **FEC items**: `FecItem{begin, symbol, end, stats}`; the stateful `Decoder`
  is (re)created on `begin`, flushed on `end`. Items are ordered by
  construction (single-producer demod thread), so the FEC thread processes the
  symbol stream in exact sequence.

## 8. Generation and reset semantics

- `latest_generation` increments on every `reset()` (parameter change, source
  switch, dropped-block recovery, explicit reset).
- Every FEC item carries its generation. `enqueue_fec()` and `run_fec()` drop
  items whose generation no longer matches, so stale state can never leak
  metrics into the current decoder.
- `reset()` sets `cancel_requested` and the front-end thread's reset handler
  clears the queues, ring, sync (invalid publish → the demod abandons its
  per-stream state), stats, and generation, then re-acquires from scratch.

## 9. Diagnostics (`StreamDecoderStats`)

Per-window stage timings (visible in the GUI "Signal Quality" panel and via
`--decode-iq -d`); a window is ~400 symbols (~0.6 s):

```text
resample_time_ms / acquisition_time_ms / equalization_time_ms
demap_time_ms / deinterleave_time_ms / depuncture_time_ms   (summed over symbols)
fec_time_ms / transport_time_ms
processing_realtime_ratio = wall_time / input_seconds
resample_workers (1) / symbol_workers / viterbi_workers
```

Quality counters: `mer_db`, pre/post-Viterbi BER (from survivor re-encoding and
RS corrections), `rs_uncorrectable_packets`, `tei_packets`, `ts_packets`,
`dropped_blocks`, `pilot_phase_discontinuities`,
`tracked_carrier_offset_hz` (CFO loop estimate), `acquisition_start` (boundary
phase within a symbol), `timing_offset_samples` (pilot-slope fractional timing
diagnostic), plus live queue depths (`queued_input_samples`, `queued_symbols`,
and their capacities).

## 10. Live vs. offline usage

- **Live GUI / capture**: `StreamDecoder::submit()` from the source callback
  (drops on overload are counted), sink = mpv player + TS recorder.
- **Offline `--decode-iq`**: `submit_blocking()` (decoder-paced — waits for
  queue capacity, so the input can never outrun the decoder), then `flush()`;
  a nonzero `dropped_blocks` after flush is treated as an internal error.
- **GUI monitor** (`SignalAnalyzer`) shares the resampler/acquisition code but
  keeps its own one-symbol worker so UI snapshots never wait on the FEC
  pipeline.

## 11. Code map

| Concern                                           | Files                                                                                |
| ------------------------------------------------- | ------------------------------------------------------------------------------------ |
| StreamDecoder (threads, ring, sync, pools, gate)  | `src/dvbt/stream_decoder.cpp`, `include/airspy_tv/dvbt/stream_decoder.hpp`           |
| Resampler + OFDM acquisition                      | `src/dvbt/ofdm_acquisition.cpp`, `include/airspy_tv/dvbt/ofdm_acquisition.hpp`       |
| QAM demapper (Max-Log)                            | `src/dvbt/soft_demapper.cpp`, `include/airspy_tv/dvbt/soft_demapper.hpp`             |
| Symbol/bit deinterleave, depuncture               | `src/dvbt/inner_decoder.cpp`, `include/airspy_tv/dvbt/inner_decoder.hpp`             |
| TPS decoding                                      | `src/dvbt/tps_decoder.cpp`, `include/airspy_tv/dvbt/tps_decoder.hpp`                 |
| Per-symbol Decoder facade                         | `src/dvbt/decoder.cpp`, `include/airspy_tv/dvbt/decoder.hpp`                         |
| Viterbi pool, outer deinterleave, RS, descrambler | `src/dvbt/transport_decoder.cpp`, `include/airspy_tv/dvbt/transport_decoder.hpp`     |
| GUI one-symbol monitor                            | `src/dvbt/signal_analyzer.cpp`, `include/airspy_tv/dvbt/signal_analyzer.hpp`         |
| Source fan-out, sinks, recorders                  | `src/sdr.cpp`, `include/airspy_tv/sdr.hpp`, `src/recorder.cpp`                       |
| Playback queue                                    | `src/mpv_player.cpp`, `include/airspy_tv/mpv_player.hpp`                             |

Historical design notes, measured before/after numbers, and future work (e.g.
shrinking fade-recovery latency, sample-clock tracking) live in `ROADMAP.md`.
