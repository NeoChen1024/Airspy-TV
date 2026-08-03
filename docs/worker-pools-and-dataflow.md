# Worker Pools and Data Flow

This document describes how the `airspy-tv-dvbt` decoder library and the
`airspy-tv` application are structured around bounded queues, ordered worker
pools, and a staged CS16 → MPEG-TS pipeline. It covers the I/Q resampler, the
OFDM front end, Max-Log QAM demodulation, the convolutional/RS FEC chain, and
the chunk-continuity join.

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
                         ▼ StreamDecoder (bounded input queue, 200 ms)
        ┌─────────────────────────────────────────────────────────┐
        │ front-end thread (StreamDecoder::Impl::worker)          │
        │   accumulate → 7 M-sample chunks (100 ms overlap)       │
        │   ┌─ Stage 1  Cs16Resampler pool (full budget)          │
        │   ├─ Stage 2  acquire_ofdm (first 350 k samples)        │
        │   ├─ Stage 3  per-symbol loop: FFT, pilot lock, CFO,    │
        │   │           channel estimate, TPS                     │
        │   └─ Stage 4  → SymbolPostprocessorPool (≈1/3 budget)   │
        │                  gain, MER, reliability, Max-Log demap, │
        │                  symbol/bit deinterleave, depuncture,   │
        │                  soft-byte quantization                 │
        └─────────────────────────────────────────────────────────┘
                         ▼ FEC queue (bounded, 200 ms of symbols)
        ┌─────────────────────────────────────────────────────────┐
        │ FEC thread (StreamDecoder::Impl::fec_worker)            │
        │   stateful Decoder (one per chunk generation)           │
        │   └─ Stage 5  TransportDecoder                          │
        │        SoftViterbi pool (≈2/3 budget)                   │
        │        12-branch byte deinterleaver                     │
        │        RS(204,188) → energy descrambler → TS packets    │
        │   └─ Stage 6  chunk-overlap join → TransportCallback    │
        └─────────────────────────────────────────────────────────┘
                         ▼ sinks
        transport_model (service discovery) · TS recorder · player / CLI output
```

## 2. Thread and pool inventory

| Thread / pool                                          | Where                 | Count           | Work                                                          |
| ------------------------------------------------------ | --------------------- | --------------- | ------------------------------------------------------------- |
| `SdrDevice::airspy_rx_callback`                      | sdr.cpp               | libairspy-owned | copies one block to 4 bounded sinks                           |
| `run_soapy` / `run_file`                           | sdr.cpp               | 1 each          | blocking read loop, same fan-out                              |
| `StreamDecoder::Impl::worker`                        | stream_decoder.cpp    | 1, persistent   | chunking, resample, acquisition, tracking loop, pool dispatch |
| `StreamDecoder::Impl::fec_worker`                    | stream_decoder.cpp    | 1, persistent   | stateful FEC decode per generation, chunk join, callback      |
| `Cs16Resampler` pool                                 | ofdm_acquisition.cpp  | N = full budget | partitioned rational resampling                               |
| `SymbolPostprocessorPool`                            | stream_decoder.cpp    | S ≈ N/3        | per-symbol postprocessing                                     |
| `SoftViterbi` pool                                   | transport_decoder.cpp | V = N − S      | overlapping Viterbi windows                                   |
| `SignalAnalyzer` worker                              | signal_analyzer.cpp   | 1               | one-symbol GUI monitor snapshot                               |
| `RawIqRecorder` / `TransportStreamRecorder` writer | recorder.cpp          | 1 each          | disk I/O, 5 s / 24 MiB buffers                                |

The `fec_worker` is intentionally **stateful**: the outer deinterleaver phase,
RS alignment, and energy-descrambler phase must run serially, so they live on a
single thread that never competes with the symbol pools.

## 3. Worker budget allocation

`ReceiverParameters::worker_threads` (0 = auto) is the only knob. It is
**fixed before a source opens**; changing it reconstructs both pools.

`allocate_workers(total)` in stream_decoder.cpp:

```text
total   = requested == 0 ? hardware_concurrency : requested
if total <= 1             -> { symbol = 1, viterbi = 1 }
symbol  = max(1, total / 3)          (SymbolPostprocessorPool)
viterbi = max(1, total - symbol)     (SoftViterbi pool)
resample workers = total             (Cs16Resampler uses the full budget)
```

Example at 16 threads: resampler 16, symbol pool 5, Viterbi pool 11.

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

### 5.1 Input queueing and chunking (front-end thread)

- `submit()` copies into `std::deque<Block>`; `submit_blocking()` waits for
  capacity instead of dropping (offline path). Capacity =
  `sample_rate / 5` (≈ 200 ms). Live callbacks use `submit()`; drops are
  counted in `StreamDecoderStats::dropped_blocks`.
- The front-end thread accumulates blocks into chunks of
  `processing_chunk_samples = 7,000,000` complex samples (≈ 0.7 s at 10 MSPS),
  keeping `chunk_overlap_samples = sample_rate / 10` (100 ms) of overlap so the
  next chunk starts from a clean acquisition point.
- Each chunk is decoded independently (acquisition → TPS → FEC) and the overlap
  prefix is removed downstream by an exact packet join (§5.7).

### 5.2 Stage 1 — I/Q resampling (`Cs16Resampler`)

- Target rate is the DVB-T nominal baseband rate: `bandwidth × 8/7`.
- One liquid-dsp `rresamp_crcf` rational resampler partitioned across the pool
  at exact input/output block boundaries: `next_p = (bw×8)/gcd`, `next_q = (rate×7)/gcd`; each worker gets a contiguous slice.
- Every worker restores the previous 12 input blocks (`resampler_semi_length`)
  of FIR history, which is what makes partitioned output **bit-identical** to
  serial resampling across 1/2/4/8/16-thread budgets.
- CS16→complex conversion uses VOLK (`volk_16i_s32f_convert_32f`). `process()`
  is synchronous — only the front-end thread calls it; internally the pool
  parallelizes the block slices.

### 5.3 Stage 2 — OFDM acquisition (`acquire_ofdm`)

- Runs on the first `acquisition_samples = 350,000` resampled samples.
- Searches cyclic-prefix periodicity across all allowed (or user-selected)
  `TransmissionMode` × `GuardInterval` combinations, returning the best
  `{start, fft_size, guard_size, phase, score}`.
- `score < 0.20` → chunk abandoned (no FEC work wasted on noise).
- A validated mode/guard is cached (`stable_mode`/`stable_guard`) and reused
  across transient misses; full searching resumes only after a source/tuning/
  parameter reset.

### 5.4 Stage 3 — per-symbol tracking loop (front-end thread, serial)

For each OFDM symbol (`start += fft_size + guard_size`):

1. **NCO**: mix the FFT window with a complex oscillator at the tracked CFO
   (`tracked_cfo_phase`), renormalized every 512 taps.
2. **FFT**: `fftwf` forward transform (plan created once per chunk).
3. **Pilot lock**: correlate scattered pilots (PRBS-modulated, ±4/3) to find
   the 4-phase scattered-pilot phase and integer carrier offset. First lock
   searches ±48 bins, afterwards ±2.
4. **CFO tracking**: continual-pilot temporal correlation drives a first-order
   loop (`loop_gain = 0.20`); a phase EMA gives the residual carrier offset.
5. **Channel estimation**: divide received pilots by the known PRBS value, then
   linearly interpolate between pilots per phase (pilot indices are cached per
   mode once per chunk). Per-carrier `equalizer_power = |ĥ|²` is kept.
6. **TPS**: decode the TPS carriers. If no manual constellation/code rate was
   set, a locked, non-hierarchical TPS frame auto-selects the decoder
   parameters (`DecoderParameters`).
7. **Payload extraction**: the 1512 (2K) or 6048 (8K) payload carriers are
   equalized (`× ĥ⁻¹`) and submitted to the symbol pool together with their
   reliability inputs. Before TPS lock, up to 136 symbols are buffered
   (`pending_symbols`) with a fallback symbol index (pilot phase); once TPS
   locks, buffered symbols are re-indexed from the TPS frame index so
   deinterleaving stays coherent across the lock boundary.

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

Results may complete **out of order**; an ordered join (§6) hands
`mother_metrics` blocks to the FEC worker in original sequence.

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
6. Output: contiguous 188-byte TS packets for the chunk.

### 5.7 Stage 6 — chunk-overlap join → callback

Because consecutive chunks overlap by 100 ms of input, their TS tails/heads
repeat. The FEC thread deduplicates with an **exact packet-sequence join**:

1. Hash every 188-byte packet (FNV-1a) and find the longest common suffix of
   `transport_history` vs. prefix of the new chunk using a KMP prefix table on
   packet hashes (min 32 packets, verified by byte equality).
2. The expected overlap packet count is derived from the time ratio
   (`overlap_input_seconds / chunk_seconds × packets`).
3. The redundant prefix is removed and only the new suffix is emitted to the
   `TransportCallback`.
4. `transport_history` retains the last `retained_ts_packets = 32,768` packets
   per generation for the next join. Counters track joined packets and failed
   joins.

This makes internal chunk boundaries invisible to the transport stream: on the
full 557 MHz field capture it reduced continuity-counter gaps on every active
PID from ~170–200 to zero.

## 6. Queues and backpressure

All queues are **bounded by stream duration, not item count** (≈ 200 ms per
pipeline stage), so memory stays flat and scheduler overload surfaces as
dropped blocks instead of unbounded growth:

| Queue               | Capacity                                                                         | Producer → consumer              |
| ------------------- | -------------------------------------------------------------------------------- | --------------------------------- |
| Input blocks        | `sample_rate / 5` complex samples                                              | source thread → front-end thread |
| FEC items (symbols) | `buffered_symbol_count(bw, symbol_duration)` ≈ 1/5 s of symbols (initial 256) | front-end → FEC thread           |
| Viterbi windows     | `max(2×workers, 1024)` windows                                                | FEC thread → pool                |
| Raw I/Q recorder    | 5 s of samples                                                                   | source thread → writer thread    |
| TS recorder         | 24 MiB                                                                           | FEC thread → writer thread       |
| mpv playback        | 24 MiB                                                                           | FEC thread → mpv queue           |

The spectrum and signal monitors are latest-snapshot mailboxes (no buffering —
buffering old displays only adds GUI latency).

## 7. Ordering and joins

- **Symbol pool**: tasks carry a monotonic `sequence`; results land in a
  `std::map<sequence, result>` and the front-end drains via
  `take_ready()`/`flush()` only in order (`next_result_`).
- **Viterbi pool**: identical pattern — `completed_` keyed by window sequence,
  `take_ready_locked()` joins windows in order before outer deinterleaving.
- **FEC items**: `FecItem{begin, symbol, end}` per chunk generation; the
  stateful `Decoder` is (re)created on `begin` and flushed on `end`.

## 8. Generation and reset semantics

- `latest_generation` increments on every `reset()` (parameter change, source
  switch, dropped-block recovery, explicit reset).
- Every FEC item carries its generation. `enqueue_fec()` and `run_fec()` drop
  items whose generation no longer matches, so a stale chunk can never leak
  metrics into the current decoder, and `transport_history` is cleared when the
  generation actually changes.
- `reset()` sets `cancel_requested` so the front-end thread exits its symbol
  loop early; pending queues are cleared under the mutex.

## 9. Diagnostics (`StreamDecoderStats`)

Per-chunk stage timings (visible in the GUI "Signal Quality" panel and via
`--decode-iq -d`):

```text
resample_time_ms / acquisition_time_ms / equalization_time_ms
demap_time_ms / deinterleave_time_ms / depuncture_time_ms   (summed over symbols)
fec_time_ms / transport_time_ms
processing_realtime_ratio = wall_time / input_seconds
resample_workers / symbol_workers / viterbi_workers
```

Quality counters: `mer_db`, pre/post-Viterbi BER (from survivor re-encoding and
RS corrections), `rs_uncorrectable_packets`, `tei_packets`, `ts_packets`,
`ts_overlap_packets`, `ts_overlap_join_failures`, `dropped_blocks`,
`pilot_phase_discontinuities`, plus live queue depths
(`queued_input_samples`, `queued_symbols`, and their capacities).

## 10. Live vs. offline usage

- **Live GUI / capture**: `StreamDecoder::submit()` from the source callback
  (drops on overload are counted), sink = mpv player + TS recorder.
- **Offline `--decode-iq`**: `submit_blocking()` (decoder-paced — waits for
  queue capacity, so the input can never outrun the decoder), then `flush()`;
  a nonzero `dropped_blocks` after flush is treated as an internal error.
  Output is byte-identical across tested 1/2/4/8/16-thread budgets.
- **GUI monitor** (`SignalAnalyzer`) shares the resampler/acquisition code but
  keeps its own one-symbol worker so UI snapshots never wait on the FEC
  pipeline.

## 11. Code map

| Concern                                           | Files                                                                                |
| ------------------------------------------------- | ------------------------------------------------------------------------------------ |
| StreamDecoder (queues, pools, chunking, join)     | `src/dvbt/stream_decoder.cpp`, `include/airspy_tv/dvbt/stream_decoder.hpp`       |
| Resampler + OFDM acquisition                      | `src/dvbt/ofdm_acquisition.cpp`, `include/airspy_tv/dvbt/ofdm_acquisition.hpp`   |
| QAM demapper (Max-Log)                            | `src/dvbt/soft_demapper.cpp`, `include/airspy_tv/dvbt/soft_demapper.hpp`         |
| Symbol/bit deinterleave, depuncture               | `src/dvbt/inner_decoder.cpp`, `include/airspy_tv/dvbt/inner_decoder.hpp`         |
| TPS decoding                                      | `src/dvbt/tps_decoder.cpp`, `include/airspy_tv/dvbt/tps_decoder.hpp`             |
| Per-symbol Decoder facade                         | `src/dvbt/decoder.cpp`, `include/airspy_tv/dvbt/decoder.hpp`                     |
| Viterbi pool, outer deinterleave, RS, descrambler | `src/dvbt/transport_decoder.cpp`, `include/airspy_tv/dvbt/transport_decoder.hpp` |
| GUI one-symbol monitor                            | `src/dvbt/signal_analyzer.cpp`, `include/airspy_tv/dvbt/signal_analyzer.hpp`     |
| Source fan-out, sinks, recorders                  | `src/sdr.cpp`, `include/airspy_tv/sdr.hpp`, `src/recorder.cpp`                 |
| Playback queue                                    | `src/mpv_player.cpp`, `include/airspy_tv/mpv_player.hpp`                         |

Historical design notes and future work (e.g. persistent tracking instead of
overlap-save reacquisition) live in `ROADMAP.md`.
