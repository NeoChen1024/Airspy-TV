# Roadmap

Airspy TV is being developed in stages so that RF acquisition, DVB-T PHY
decoding, transport-stream handling, and video playback can be validated
independently. The end goal is a standalone terrestrial broadcast decoder with
the Airspy R2 as a first-class SDR source.

## Implemented foundation

- SDL3 and Dear ImGui single-window receiver UI with a scrollable diagnostics
  sidebar and reserved video surface.
- Native Airspy and generic SoapySDR sources, including Airspy gain profiles,
  Bias-T control, device-loss reporting, and source shutdown.
- CS16 recording with JSON sidecars, raw `airspy_rx` INT16_IQ compatibility,
  file playback, and decoder-paced offline extraction.
- Live FFT spectrum, waterfall, signal-power telemetry, constellation, OFDM
  lock, carrier offset, MER, CP-SNR, deepest-notch, and decoder CPU status.
- Native raw-I/Q-to-MPEG-TS decoding and TS recording through the same
  `StreamDecoder` used by GUI and CLI sources.
- Overlap-save processing across raw-I/Q chunks with exact 188-byte TS packet
  joining, so independently acquired frontend/FEC windows do not create an
  output discontinuity at every internal chunk boundary.
- A sidebar EPG panel shows the selected service's now/next guide, parsed from
  EIT present/following and TDT/TOT clock data in the decoded transport
  stream, with iconv-based DVB text decoding (Big5, GB2312, EUC-KR,
  ISO-8859-x, and the Taiwan mislabeled-UTF-16 quirk).

## DVB-T reference receiver

- Live and file sources already use a common CS16 stream boundary and exact
  rational resampling; the tested centered 6 MHz path runs at 48/7 MSPS.
- Native 2K/8K cyclic-prefix acquisition, carrier tracking, pilot channel
  estimation, and equalization feed the complete soft-decoding chain.
- Finish continuous sample-clock tracking. Differential TPS demodulation,
  synchronization, BCH validation, and non-hierarchical modulation/HP-code-rate
  discovery are implemented; add lock-loss hysteresis, superframe/cell-ID
  assembly, and hierarchical HP/LP selection.
- The UI and native resampling/monitor/decoder paths accept 5, 6, 7, and 8 MHz
  channel raster rates and guard intervals 1/4, 1/8, 1/16, and 1/32. Add
  reference fixtures for the non-6-MHz bandwidths; the current Taiwan
  regression baseline remains centered 6 MHz.
- Continue improving continuous carrier, channel, and sample-clock tracking so
  the frontend remains locked on weak, multipath, and SFN captures.

## Production soft-decoding pipeline

The native decoder does not depend on a generic modem abstraction at its core.
Its inner-decoder path is:

```text
equalized DVB-T carriers + per-carrier reliability
    -> native DVB-T Max-Log LLR demapper
    -> native soft bit and symbol deinterleavers
    -> DVB-T depuncturer
    -> libcorrect soft-input Viterbi decoder
    -> convolutional byte deinterleaver
    -> RS(204,188) decoder
    -> energy descrambler
    -> MPEG transport stream
```

Implementation requirements:

- Support QPSK, 16-QAM, and 64-QAM with DVB-T bit ordering; begin with
  non-hierarchical transmission and add hierarchical HP/LP streams separately.
- Treat hierarchical 16-QAM/64-QAM with alpha 1, 2, and 4 as a required DVB-T
  compatibility stage, including independent HP and LP code rates. Do not let
  this delay the non-hierarchical receiver used by the current capture.
- Generate Max-Log LLRs using the measured noise and channel reliability.
  Carriers in deep fades must produce low-confidence metrics instead of
  confident hard errors.
- Preserve soft metrics through DVB-T bit deinterleaving.
- Support code rates 1/2, 2/3, 3/4, 5/6, and 7/8. Insert a neutral metric for
  punctured bits before decoding the K=7, rate-1/2 mother code (libcorrect
  stores the polynomials as 0117/0155 octal, the bit-reversed DVB-T 171/133).
- Wrap libcorrect with streaming traceback and deterministic superframe reset
  behavior; do not assume its packet-oriented API directly matches DVB-T.
- Keep the decoder API independent of Airspy, SoapySDR, the UI, and the
  transport-stream consumer.

Current native implementation:

- `airspy-tv-dvbt` provides normalized QPSK, 16-QAM, and 64-QAM Max-Log LLR
  demapping with explicit per-carrier reliability.
- Native soft symbol and bit deinterleavers support both DVB-T 2K and 8K
  modes.
- Native depuncturing covers all DVB-T convolutional code rates and preserves
  neutral metrics for punctured bits.
- The equalized-carrier decoder now connects Max-Log demapping, 2K/8K symbol
  and bit deinterleaving, all five puncturing rates, an overlapping
  `libcorrect` soft-Viterbi wrapper, automatic 12-phase outer deinterleaver
  acquisition, shortened RS(204,188), and DVB energy descrambling.
- Synthetic end-to-end tests cover all five code rates plus representative
  2K/QPSK and 8K/64-QAM equalized-symbol paths. The decoder preserves neutral
  puncture metrics and recovers byte-identical 188-byte transport packets.
- A GNU Radio equalizer/native-decoder cross-check on 500 symbols from the
  557 MHz capture recovered 1,809 aligned TS packets with no TEI flags, a
  valid PAT (services 300, 301, 302, and 304), and PMT packets. This validates
  the native data/FEC path; GNU Radio is still providing OFDM acquisition and
  equalization for this checkpoint.
- GNU Radio remains an offline reference during validation; it is not part of
  the native decoder library API.
- A low-rate GUI monitor performs 2K/8K cyclic-prefix acquisition, integer and
  fractional carrier-offset estimation, scattered-pilot channel interpolation,
  and publishes equalized constellation, MER, CP-SNR, and deepest-notch
  snapshots without blocking the live I/Q callback. Its acquisition and
  channel tracking remain diagnostic until checked against TPS and standard
  vectors.
- The live Airspy, SoapySDR, and CS16 file paths now also feed a bounded,
  asynchronous native decoder worker. Its first raw-IQ frontend uses exact
  rational resampling and repeatedly acquires 2K/8K OFDM symbols before
  entering the validated equalized-carrier/FEC pipeline; recovered bytes are
  routed to the MPEG-TS recorder without running DSP in a source callback.
- A deterministic GNU Radio reference transmitter now exercises the complete
  raw-IQ boundary with a centered 10 MSPS CS16 6 MHz, 8K, guard-1/4, 64-QAM,
  rate-2/3 waveform. The native frontend deterministically recovers
  packet-aligned TS with a valid PAT/PMT. This exposed and fixed an FFT-window
  error where cyclic-prefix acquisition was incorrectly treated as the start
  of useful symbol data.
- The older 557 MHz capture remains a weak/multipath robustness case rather
  than the functional baseline. It needs to be re-evaluated after continuous
  sample-clock/channel tracking is implemented; ideal-signal lock does not yet
  imply reliable field reception.
- New Airspy field captures validate the native raw-IQ path beyond the ideal
  fixture: the horizontal 557 MHz recording and three 581 MHz recordings
  recover thousands of aligned TS packets. The vertical 557 MHz recording has
  materially lower MER and still fails RS, making it a useful
  antenna/multipath regression case.
- Offline raw-I/Q extraction is exposed by the main `airspy-tv --decode-iq`
  command and uses the same `StreamDecoder` and JSON/raw-file resolver as GUI
  playback. The older equalized-carrier, soft-byte, and separate raw-IQ debug
  executables have been removed so they cannot diverge into alternate decoder
  paths.
- The displayed deepest-notch estimate excludes active-channel filter skirts
  and uses the lower first percentile relative to the median channel response.
  This avoids reporting a single noisy pilot or FFT-bin outlier as a deep fade.
- The GUI reports OFDM-monitor lock, TS-decoder lock, and native-decoder CPU
  load independently. CPU load is measured as decoder wall time divided by the
  duration of its input samples, with raw-block queue depth and drops exposed so
  scheduler overload cannot be mistaken for RF unlock. Transient FEC queue
  bursts are displayed for diagnostics but are not treated as CPU overload;
  overload requires slower-than-realtime processing, a nearly full raw-input
  queue, or an actual dropped block.
- Bounded queues are provisioned by stream duration, not item count. Raw I/Q,
  OFDM-symbol/FEC, and Viterbi-window pipeline stages retain approximately 200
  ms. Disk recorders use independent buffers of at least five seconds. The
  spectrum and quality monitors remain latest-snapshot mailboxes because
  buffering old displays would only increase GUI latency.
- GUI monitoring and the complete CLI/TS decoder share the same liquid-dsp
  CS16 resampler and CP acquisition implementation. Once automatic acquisition
  validates a mode and guard interval, both paths retain that configuration
  across transient misses; full searching resumes only after an explicit
  source, tuning, or parameter reset. The GUI keeps a separate one-symbol
  equalizer worker so the slower FEC path cannot stall interactive diagnostics.
- Viterbi decoding uses a bounded CPU window pool. Each worker owns an
  independent libcorrect context, overlapping windows may complete out of
  order, and an ordered join preserves the original byte stream before outer
  deinterleaving. The pool intentionally trades latency for throughput.
- A bounded symbol postprocessing pool now moves the independent
  decision-directed gain, MER/error, per-carrier reliability, Max-Log
  demapping, symbol/bit deinterleaving, depuncturing, and soft-byte quantization
  off the serial OFDM tracking loop. Results may finish out of order but
  mother-code metric blocks are joined by sequence before the stateful
  transport decoder. The logical-CPU worker budget is split between this pool
  and the Viterbi pool and is immutable while a source is open.
  Carrier topology is cached per mode/phase, the CFO NCO uses a normalized
  complex recurrence, and CS16 conversion uses VOLK. The liquid-dsp rational
  resampler is partitioned at exact input/output block boundaries; each worker
  restores the preceding 12-block FIR history, producing bit-identical output
  while reusing the full worker budget during this otherwise serial phase. On
  the 0.7-second 557 MHz field fixture, Release wall time fell from about 0.68
  seconds to about 0.16 seconds with a 16-thread budget while retaining
  byte-identical TS output across 1/2/4/8/16-thread configurations.
- Spectrum and quality smoothing follow SDR++'s speed model
  (`alpha = min(speed / (update_rate * 10), 1)`). Raw FFT rows reach the
  waterfall before FFT smoothing is applied to the spectrum trace.
- Pre-Viterbi BER is estimated by re-encoding each decoded survivor path and
  comparing it with non-punctured hard decisions. Post-Viterbi BER counts
  payload-bit corrections in successfully decoded RS(204,188) codewords;
  uncorrectable RS packets remain a separate counter because their bit-error
  count is unknowable.
- Raw-I/Q chunks retain 100 ms of input overlap. Each chunk is decoded through
  acquisition, TPS, and FEC independently, then an exact packet-sequence join
  removes the redundant prefix before delivery. This is a deterministic
  continuity bridge while retaining bounded state and simple reset behavior;
  diagnostics count joined packets and failed joins. On the full horizontal
  557 MHz capture it reduced continuity-counter gaps on every active PID from
  roughly 170--200 to zero. The added DSP work still runs faster than realtime
  on the current 16-thread test host.

## Verification status

Completed checks:

- Synthetic inner/FEC vectors cover all five code rates, both transmission
  modes, and representative QPSK/16-QAM/64-QAM paths.
- The ideal raw-IQ fixture and clean Airspy field captures deterministically
  recover packet-aligned TS with valid service tables.
- Serial and partitioned resampling are bit-identical, and offline decoding is
  byte-identical across tested 1/2/4/8/16-thread budgets.
- Optimized Debug and Release builds run the same decoder tests and raw-IQ
  pipeline.
- EIT present/following and TDT/TOT parsing plus DVB text decoding are
  verified against decoded captures of two Taiwanese muxes (581 MHz TTV,
  557 MHz FTV). Program names, start times, durations, and running status
  match the broadcast schedules; both muxes use the 14-byte EIT header with
  the service id in the table_id extension.

Remaining receiver validation:

- Track TPS lock, corrected RS packets, and uncorrectable/TEI packets
  consistently across processing chunks without double-counting overlap-save
  work.
- Compare hard- and soft-decision behavior on clean, weak-signal, multipath,
  SFN, and discontinuous captures.
- Add long-running live reception regressions for Airspy and selected SoapySDR
  devices.

Next decoder step:

Completed (validated 2026-08 on 557M/581mhz field captures):

- Continuous three-stage pipeline (this restructure): a front-end thread runs
  one persistent streaming rational resampler (liquid rresamp, single filter
  state across the whole capture — resampling is memory-bandwidth-bound, so
  the old 16-way partition is gone) and an event-driven acquisition monitor;
  a demod thread extracts a fully contiguous symbol stream through a 1 Mi-
  sample ring (absolute uint64 stream positions); a stateful transport worker
  feeds the Viterbi pool and emits continuous TS with no per-chunk begin/end
  seams and no overlap dedup.
- The carried states (CFO loop, integer carrier offset, continual reference,
  AND the TPS superframe decoder) now carry for the life of a stream: the
  symbol sequence is contiguous (no overlap rewind), so the differential TPS
  decoder locks once instead of re-locking every chunk — the key weak-signal
  win of this design.
- Fade handling (the old pipeline was immune because a failed chunk
  acquisition froze all tracking; the continuous demod must do it itself):
  the normalized continual-carrier temporal correlation is a fade indicator;
  below 0.25 the CFO loop, pilot phase, and carrier lock freeze on their
  carried values (a noise-latched offset cannot escape the +/-2-bin lock and
  permanently scrambled the channel estimate in early builds). After 1400
  frozen symbols (~2.1 s) the demod runs an event-driven re-anchor at a
  symbol-loop boundary: it restores the pre-fade carrier grid (the LO never
  moves), re-seeds the CFO from the CP phase, and re-locks TPS; the in-flight
  symbol is discarded so no symbol is built from mixed grids. Recovery is now
  deterministic: repeated full-capture runs decode byte-identical TS under
  parallel-build load.
- Windowed MER gate: 68-symbol windows (one TPS frame) whose mean MER falls
  below the constellation floor are dropped and bracket end/begin FEC resets;
  hopeless regions never grind the Viterbi, and faded-head/recovered-tail
  regions still decode. The outer FEC never locks a zero-evidence phase
  (sync distance alone), and 100 consecutive uncorrectable RS codewords
  trigger a fresh outer-phase search — a fade-corrupted false lock that
  silently scrambled every later packet is gone.
- Closed-loop sample-clock tracking: the pilot phase-slope estimate's
  windowed mean is dominated by the channel's mean group delay (multipath
  bias ~+35 samples on the 581), so only the slow drift between stats windows
  is tracked and accumulated into a fractional timing that nudges the symbol
  period by +/-1 sample (bounded +/-4, reset on grid rebuild / re-anchor /
  reset). On the 581 MHz capture this keeps the FFT window centred against
  the 0.5 ppm TCXO drift.

Measured before/after (121 s 581 MHz + 135 s 557 MHz captures, 64-QAM):

- 581 MHz (multipath valley 10.3-33.7 s): the old chunked pipeline decoded
  183,999,360 bytes; the continuous pipeline decodes 178,362,556 bytes
  (97%) deterministically — the sample-clock correction keeps the FFT
  window centred and recovers the tail the clock drift was eroding. Valley
  gap ~23 s unchanged (signal physically undecodable). Wall time ~36 s at 8
  threads.
- 557 MHz (uniform MER 8-12 dB, ~10 dB below the 64-QAM threshold): the
  horizontal capture decodes 270,898,976 bytes with the closed-loop timing
  (weak-signal regions lock where the absolute-tau P-loop churned); the
  vertical capture stays TS 0 (physics — the gate spares the Viterbi).
- Clean-signal regression: the 557-first-chunk fixture still decodes
  byte-identical (MD5 eabba87cccf3dd29a3ef18a8e23ecdd9); 3/3 ctest; the ideal
  synthetic fixture decodes within 16 packets of the old output (first-decode
  trellis warmup only).
- Determinism: the 40 MB fixture decodes byte-identical across 30 consecutive
  runs (MD5 df5549f0f0e16b7bb6feba819001c08d); full captures are byte-identical
  across repeated runs and under parallel-build load.

Remaining:

- Profile a modern AVX2 Viterbi implementation; libcorrect's SSE decoder is now
  the dominant CPU hotspot after the ordered-pipeline optimizations.
- Add long-running live reception regressions for Airspy and selected SoapySDR
  devices; add StreamDecoder integration tests over synthetic I/Q.

## Implementation status and follow-up

The following completed items are pending final review and acceptance. The
listed commits are implementation references; review should verify the code,
regressions, and acceptance evidence.

### Continuous DVB-T pipeline correctness

- [x] Reset all per-stream front-end state on every reset, retune, and source
  replacement. (`938e5b1` / `e51c7eb`) The fade-recovery grid, stable carrier
  state, CFO, continual reference, pilot phase, TPS state, and related counters
  are cleared so state cannot leak between frequencies or files.
- [x] Make event-driven re-anchor an explicit symbol-loop boundary.
  (`e51c7eb`) A successful mid-symbol acquisition discards the in-flight symbol
  and restarts at the published `next_symbol_start`.
- [x] Implement closed-loop sample-clock/timing correction. (`938e5b1`)
  Pilot phase-slope drift nudges the symbol period by +/-1 sample, bounded to
  +/-4 samples and reset on grid rebuild, re-anchor, and reset.
- [x] Split TPS lock state into `ever_locked` and `currently_valid`.
  (`e51c7eb`) Parameters remain fixed after the first valid lock while later
  failed checks no longer report a healthy current lock.

### Pipeline pacing and playback

- [x] Use a common 0.2-second ingestion/jitter budget. (`c0e9f27`) File reads
  are split into `sample_rate / 5` complex-sample spans before submission.
- [x] Size the resampled ring from the active rate. (`c0e9f27`) The ring holds
  approximately 0.2 seconds of input/baseband data, has a 1 Mi-sample floor,
  and resizes only while empty.
- [x] Expose SDR frequency correction in the Source panel. Airspy Native and
  SoapySDR use `hardware_frequency = nominal_frequency * (1 + ppm / 1e6)` with
  correction bounded to +/-1000 ppm; applying it live retunes and resets
  decoder tracking, while I/Q metadata remains unchanged.
- [x] Define and propagate transport discontinuity semantics. (`87e912b`)
  `TransportDiscontinuity` events are out-of-band and distinct from per-packet
  TEI marking; retune, stream-end, and FEC-region recovery are handled
  independently by `MpvPlayer` and the demuxer.
- [x] Add initial playback telemetry. (`87e912b`) Libmpv playback time, A/V
  offset, dropped frames, pause state, TS queue depth, and decoder
  discontinuity count are shown in the GUI Playback panel.

### Verification and documentation

- [x] Complete the first 8K `StreamDecoder` integration-test slice. (`b5d325a`)
  Coverage includes synthetic 8K/GI-1/4/QPSK/1/2 signals, persistent
  resampling, arbitrary CS16 boundaries, acquisition/demodulation/FEC/TS
  output, finite-stream flush, reset, retune, stream end, queue drain, and a
  non-acquirable zero-I/Q stream. Verification: CTest 4/4 and 10 repeated
  integration runs.
- [x] Synchronize architecture documentation. (`87e912b`)
  `docs/worker-pools-and-dataflow.md` and this roadmap document event-driven
  acquisition, phase-only re-lock, TPS fix-once, closed-loop sample clock,
  bounded buffering, deterministic recovery, discontinuities, and playback
  telemetry.

### Remaining implementation

- [ ] Extend synthetic `StreamDecoder` integration coverage beyond the 8K
  clean/lifecycle slice:
  - 2K mode and its guard intervals;
  - explicit timing offset, sample-clock drift, carrier-frequency offset, and
    residual CFO convergence;
  - deep fades, MER gating, phase-only re-lock, and long-fade re-anchor;
  - transport continuity and exact TS output for error-free synthetic input;
  - repeated block-boundary/reset combinations under parallel worker load.

### Robustness and security-review follow-up (2026-08)

Findings from a read-only security review of the DVB-T receiver (raw-IQ
frontend, demodulator, FEC, TS/SI, and playback paths). No memory-corruption,
injection, or authentication issues were found; the items below are
robustness/availability defects. File/line references are current as of the
audit.

- [x] Handle multipath path length / delay spread in the OFDM frontend.
  (MEDIUM, RF-side DoS vector: a delay spread approaching the guard interval
  silently degrades or kills decoding — the code history records both
  "permanent lock loss" and "silent payload scrambling".) **Implemented
  (2026-08):** per-TPS-frame CIR / delay-spread estimation (scattered-pilot
  channel estimate -> 1024/256-point IFFT -> main-lobe width) with adaptive
  FFT-window placement — the window slides to the middle of the ISI-free
  range [spread, guard] in steps of at most +/-4 samples per frame, only
  when the measured spread reaches guard/4, EMA-smoothed
  (`stream_decoder.cpp:2031-2166`, window offset applied at the
  `anchored_start()` sites `:1223`, `:1241`). Verified: 581mhz
  capture decodes byte-identical to the pre-change build (178,362,556
  bytes); synthetic 8K fixtures pass. **Known limitations, unresolved:** (a)
  the scattered-pilot spacing (12 carriers) caps the observable delay at
  Tu/12, so longer echoes alias into the window; (b) the main-lobe spread
  estimate misses well-separated echo clusters, so a 1200-sample synthetic
  echo reads as spread ~3. **Resolved (2026-08) — decoupled the window
  slides from the timing loop:** a CIR slide of d samples shifts the pilot
  phase-slope estimate of tau by exactly d, so the drift estimate read the
  slides as fake sample-clock steps. Both references are now expressed
  relative to the window's average position (window-averaged CIR offset,
  `stream_decoder.cpp:1507-1523`), making the adaptive placement invisible
  to the sample-clock loop. Measured on the 581 capture with forced
  slides: the per-window drift injected by the slides dropped from +24 to
  +2 samples (the residual is the step-vs-linear window-average
  approximation, at most one slide, absorbed by the 0.02 smoothing); the
  normal path is bit-identical to the pre-decoupling build (178,362,556
  bytes). The conservative guard/4 threshold means none of the current
  test signals trigger the adaptive path (557M is deep-fade multipath, not
  long-delay); the mechanism is in place but unproven on real long-delay
  multipath. Forced-slide experiments on 581 also confirmed the
  threshold's necessity: the main-lobe spread estimate reads ~4 samples on
  a channel with longer echoes, so sliding to the computed target moved
  the window into the ISI region (MER 24->5 dB, ~2/3 of TS lost); the CIR
  estimate itself is side-effect-free (estimate-only runs are
  byte-identical to no-estimate runs). The 581 capture's deep fades
  (MER 24 -> 1.8 dB, ~26 s of degraded periods across the 121 s file)
  also exercised fade robustness: deep fades skip the estimate via the
  `fade_indicator` gate, and during shallower fades the noise-inflated
  spread (main lobe 2.7-4 -> up to 20 samples) never crossed the guard/4
  threshold, so the window never moved (delta=0 throughout).
  Follow-up candidates: 90%-energy CDF spread estimate (fixes the
  well-separated-echo underestimate), and re-test on a captured
  long-delay (SFN-style) signal.
- [x] Verify and fix the `rresamp_crcf_execute_block` `_n` argument at both
  call sites. (LOW — **resolved as a false positive.**) The liquid-dsp API
  documents `_n` as a *block count*, not an input-sample count: each block
  consumes `Q` input samples and produces `P` output samples, with the input
  buffer sized `Q*n` and the output buffer `P*n`
  (`contrib/liquid-dsp/src/filter/src/rresamp.proto.c:329-341`,
  `include/liquid.h:5039-5048`). Both call sites match that contract exactly:
  `ofdm_acquisition.cpp:160-163` passes `end - begin` blocks with
  `job_q`/`job_p` steps, and `stream_decoder.cpp:546-548` passes `blocks`
  blocks with `decimation_`/`interpolation_` steps and erases exactly
  `blocks * decimation_` input samples. No change needed; 557M/581M
  byte-identical baselines were passing with this code as-is.

### Playback telemetry follow-up

- [ ] Add PCR/PTS/DTS monotonicity and discontinuity tracking inside the TS
  parser, including timestamp-wrap handling.
- [ ] Add multi-hour live/file playback regressions with explicit bounds on
  sustained A/V offset, queue growth, decoder stalls, and intentional live
  catch-up drops.

### Performance and hardware coverage

- [ ] Add long-running live-reception regressions for Airspy R2 and selected
  SoapySDR devices.
- [ ] Profile a modern AVX2 Viterbi implementation; libcorrect's SSE decoder
  remains the dominant CPU hotspot after the ordered-pipeline optimizations.
- [ ] Ensure the TS buffer maintains approximately 1.5 MiB of data after the
  signal returns following a dropout.
- [ ] Continue improving decoder robustness as a long-term target.

## Transport stream and playback

- PAT, PMT, and SDT section assembly, CRC validation, and service/component
  discovery are implemented. The video-footer service selector feeds the
  selected service's PMT, PCR, audio, and video PIDs to libmpv (with
  `MpvPlayer::packet_selected`) while retaining required PSI/SI packets;
  selecting the same service a second time is a no-op.
- EIT present/following (now/next) and TDT/TOT clock data are parsed into the
  sidebar EPG panel, with DVB text decoding (Big5, GB2312, EUC-KR,
  ISO-8859-x, and the Taiwan mislabeled-UTF-16 quirk) through glibc iconv.
  Real EIT sections put the service id in the table_id extension and carry
  transport_stream_id, original_network_id, segment_last_section_number, and
  last_table_id before the event loop, so events begin at byte 14 of the
  section. Still to add: EIT schedule (0x50-0x5F) with segment reassembly,
  multilingual service/event descriptors, parental ratings, subtitles,
  teletext, and alternate audio/language tracks.
- Feed a selected service to libmpv and render video into the application-owned
  OpenGL framebuffer.
- Reset playback state cleanly after source discontinuities, retunes, or
  service changes. Transport seams are now explicit: the decoder reports
  `TransportDiscontinuity` events (`fec_region_reset` / `stream_end` /
  `retune`) out of band — distinct from per-packet TEI marking — and
  `MpvPlayer::on_discontinuity` applies the controlled recovery: a live
  retune restarts the demuxer (the per-frame source-active toggle alone would
  concatenate two unrelated streams), a stream end plays out the tail and
  hits EOF, and FEC-region resets are left to the demuxer's error
  concealment. The bounded TS queue keeps its drop-old fallback for a
  stalled player.
- Long-running libmpv playback is now observable against the selected
  service's clocks: `MpvPlayer::telemetry()` samples `playback-time`,
  `avsync` (the measured audio-versus-video presentation offset), libmpv's
  dropped-frame counters, the TS queue depth, and the decoder's discontinuity
  count, drawn in the GUI "Playback" panel — so RF/sample loss, decoder
  stalls, and genuine A/V clock drift are distinguishable at a glance.
  Remaining: PCR-discontinuity tracking inside the TS parser, timestamp wrap
  handling, multi-hour live/file regressions with an explicit bound on
  sustained A/V offset and queue growth.
- Preserve uncorrectable RS codewords as cadence-correct TS packets with TEI
  set, expose their count, and let the demuxer discard corrupt payload instead
  of silently manufacturing continuity-counter gaps.
- Keep an elementary-stream decode check in long-capture validation. FFmpeg's
  initial multi-program TS probing still prints misleading SPS/PPS diagnostics,
  but extracting service 300 from the joined 557 MHz regression and decoding
  from its first SPS leaves only one macroblock error over roughly 150 seconds.
  The same check on the pre-join `a.ts` produces hundreds of missing-reference
  and damaged-frame errors, confirming that chunk continuity was the dominant
  playback corruption.

## Possible future work

### Multi-standard architecture readiness

The TS/SI/EPG/recorder/playback layer (`transport_stream`, `si_common`, `epg`,
`recorder`, `mpv_player`) is modulation-standard-agnostic: every candidate
standard below outputs an MPEG transport stream (except analog), and
DVB-T2/DTMB/DVB-C use the same EN 300 468 PSI/SI tables the EPG already
parses. The structural groundwork is complete:

- `airspy-tv-common` holds the shared, standard-agnostic FEC: the DVB outer
  stage (12-branch convolutional deinterleaver, RS(204,188), energy
  descrambler, TS recovery) and the libcorrect SoftViterbi worker pool.
  DVB-C feeds the same `fec::OuterFec` directly from a QAM slicer, skipping
  Viterbi; `dvbt::TransportDecoder` is now glue (depuncture + SoftViterbi
  -> OuterFec) with an unchanged public API.
- The `Demodulator` interface (I/Q in, MPEG-TS callback out, generic stats)
  is implemented by `dvbt::StreamDecoder`, which now owns the DVB-T GUI
  analysis path. Later standards get their own modules, mirroring the
  `airspy-tv-dvbt` static-library precedent rather than a mode switch inside
  the DVB-T inner decoder.
- `SdrDevice` is a pure SDR source/tuner (no `dvbt::` types in its public
  interface): it owns a `unique_ptr<Demodulator>` injected via
  `set_demodulator()` and exposes `set_channel_bandwidth()`; standard-
  specific configuration happens on the concrete type before injection.

Still future:

- Define a second output family for analog standards (I/Q to video frames +
  audio) that bypasses the TS layer entirely; the existing GL-texture video
  surface is reused for rendering.

Per-standard deltas once those seams exist:

- **DVB-C** (smallest delta, closest to DVB-T beyond the PHY): single-carrier
  QAM 16/32/64/128/256. The outer FEC is byte-identical to DVB-T
  (RS(204,188) + I=12/M=17 convolutional interleaver + energy descrambler),
  so the stateful outer stage of the DVB-T transport decoder is shared
  verbatim; DVB-C feeds it directly from a hard-decision QAM slicer (no
  Viterbi, no soft decisions). New work is limited to symbol timing/carrier
  recovery, a small equalizer, and the cross-QAM (32/128) bit-to-symbol
  mapping; liquid-dsp already provides the single-carrier modem primitives.
  6/7/8 MHz cable channels fit the current bandwidth model and the 10 MSPS
  Airspy rates. Validation follows the GNU Radio fixture pattern.
- **DVB-T2 / DTMB**: new OFDM chains (T2: P1/P2 and L1 signalling; DTMB:
  PN-sequence TDS-OFDM) plus LDPC+BCH FEC are the bulk of the work; SI/EPG
  carry over unchanged (DTMB also uses the DVB SI family).
- **ATSC**: single-carrier 8VSB with a decision-feedback equalizer and trellis
  coding. PSIP (VCT/MGT/STT/EIT/ETT) replaces DVB SI, so the SI/EPG layer
  needs an ATSC table branch (section-assembly infrastructure reuses).
  10 MSPS is below the roughly 11 MSPS complex-sampling floor for 8VSB, so
  ATSC additionally requires faster hardware. ATSC table parsing will follow
  the hand-rolled `si_common`/`EpgModel` pattern: TSDuck was evaluated as a
  cross-standard PSI/SI alternative and rejected (large dependency whose
  charset layer does not cover the Big5/GB2312/EUC-KR and mislabeled-UTF-16
  text that Taiwanese broadcasts rely on); its source remains a spec
  reference only.
- **NTSC/PAL/SECAM**: an independent analog pipeline (vision demod, sync
  separation, chroma decoding, FM sound) that never produces TS; reuse is
  limited to the SDR frontend, resampler, spectrum/waterfall monitors, raw
  I/Q recording, and the UI shell. Rendered through the existing GL-texture
  video surface with an SDL3 audio sink.

### Other

- Additional native SDR backends when they provide useful capabilities that a
  generic SoapySDR path cannot expose.
