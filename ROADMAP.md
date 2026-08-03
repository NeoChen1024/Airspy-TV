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

- Continuous front-end tracking across processing chunks: the CFO loop
  (tracked_cfo_phase/residual_phase_ema), integer carrier offset, and
  continual-carrier reference now carry across chunks; the per-chunk CP
  acquisition is a monitor that re-anchors the window and verifies mode/guard.
  Carrier-frequency tracking resumes warm instead of re-converging from the
  noisy acquisition phase estimate every 0.7 s.
- The CFO loop only updates from a contiguous symbol pair; the first symbol of
  a chunk is ~65 symbols earlier than the previous chunk's last symbol (the
  100 ms overlap), and feeding that rewind to the temporal-correlation loop
  overshot the frequency by ~13x at every chunk head. A start-contiguity
  guard (start == previous_symbol_start + period) skips the update instead.
- TPS superframe state is deliberately NOT carried: the differential TPS
  decoder is also sequence-sensitive, and the overlap rewind corrupts its
  frame sync and symbol index for the whole chunk (locked stays true while
  the index drifts). Each chunk re-locks TPS (~68 symbols) and the
  pending-symbol buffer absorbs the gap losslessly.
- MER gate: equalized symbols are buffered until the chunk's own symbol
  quality is known, then enqueued to the FEC (or discarded). The gate skips
  the Viterbi only when even the chunk's best-10% symbols fall below the
  constellation floor (QPSK 5 / 16-QAM 10 / 64-QAM 14 dB + 4 dB margin), so
  faded-head/recovered-tail chunks still decode while hopeless chunks are
  spared the Viterbi grind. Front-end tracking continues regardless.

Measured before/after (121 s 581 MHz + 135 s 557 MHz captures, 64-QAM):

- 581 MHz (multipath valley 10.3-33.7 s): TS 183,999,360 bytes before and
  after (identical), RS failures 4/4, join-failures 41/41, valley gap ~23 s
  unchanged (signal physically undecodable there), carried state 155/157
  chunks; wall time 171.5 s -> 25.1 s (6.8x) with the MER gate.
- 557 MHz (uniform MER 8-12 dB, ~10 dB below the 64-QAM threshold): TS 0
  before and after (physics), wall time 800 s -> 19.3 s (41x) via the gate.
- Clean-signal regression: the 557-first-chunk fixture still decodes
  byte-identical (MD5 eabba87cccf3dd29a3ef18a8e23ecdd9); 3/3 ctest.

Remaining:

- Replace overlap-save reacquisition with persistent rational-resampler,
  OFDM/TPS tracking, and FEC/outer-sync state where that improves throughput or
  weak-signal robustness. Exact TS packet joining already prevents internal
  chunk boundaries from creating multiplex-wide continuity gaps, so this is
  now an optimization and tracking-quality task rather than an output-
  correctness blocker.
- Add continuous sample-clock and channel tracking across processing chunks;
  the current frontend is measurably less robust on captured multipath signals
  than the reference receiver. (On Airspy R2 the 0.5 ppm TCXO drifts only
  ~2.4 samples per chunk, so fractional timing is a SoapySDR-generic path
  concern rather than an R2 one.)
- Turn the remaining stateful frontend into one continuous stream pipeline:

  ```text
  streaming rational resampler
      -> sample-clock / fractional-timing loop
      -> carrier NCO and residual-CFO loop
      -> OFDM symbol extraction
      -> time/frequency pilot-channel tracker
      -> TPS frame/superframe state
  ```

  Preserve resampler phase and filter history, fractional symbol position,
  sample-clock-rate estimate, and channel history across input blocks (carrier
  phase/frequency already carries, with a contiguity guard; TPS cannot carry
  while the overlap rewinds the symbol sequence). Keep this time-ordered
  frontend serial (or use an explicit ordered state handoff), then dispatch
  FFT/equalization/demapping and FEC work that is safe to parallelize. The
  current 100 ms overlap remains the fallback reacquisition and discontinuity
  bridge until this path is validated; afterwards reduce or remove routine
  overlap and reserve full reacquisition for source drops, seeks, retunes,
  parameter changes, and genuine lock loss.
- Carry validated TPS frame/superframe index and cell ID across chunks, and add
  deterministic decoder reset tags when TPS parameters change.
- Eliminate duplicated GUI-monitor/frontend work by publishing constellation
  and quality snapshots from the complete decoder where practical.
- Profile a modern AVX2 Viterbi implementation; libcorrect's SSE decoder is now
  the dominant CPU hotspot after the ordered-pipeline optimizations.

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
  service changes.
- Validate long-running libmpv playback against the selected service's PCR and
  audio/video PTS/DTS clocks. Track PCR discontinuities, timestamp wrap and
  monotonicity, TS queue depth, and the measured audio-versus-video presentation
  offset so RF/sample loss, decoder stalls, and genuine A/V clock drift can be
  distinguished. Keep the live custom stream blocking and bounded, preserve
  broadcast timestamps instead of synthesizing a wall-clock timeline, and
  perform a controlled libmpv stream reload when a discontinuity cannot be
  recovered without unbounded drift. Add multi-hour live/file regressions with
  an explicit bound on sustained A/V offset and queue growth.
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
