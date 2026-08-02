# Roadmap

Airspy TV is being developed in stages so that RF acquisition, DVB-T PHY
decoding, transport-stream handling, and video playback can be validated
independently.

## Current milestone

- SDL3 and Dear ImGui application shell.
- Native Airspy and generic SoapySDR sources.
- CS16 live recording and file playback.
- Live spectrum, waterfall, and signal-power telemetry.
- Mock decoder metrics, constellation, transport-stream recorder, and video
  surface.

## DVB-T reference receiver

- Normalize live and file sources to a common complex-sample stream.
- Resample a centered 6 MHz channel to 48/7 MSPS.
- Implement 2K/8K OFDM acquisition, carrier and sample-clock tracking, pilot
  channel estimation, equalization, and TPS decoding.
- Cover 5, 6, 7, and 8 MHz channel raster rates and guard intervals 1/4, 1/8,
  1/16, and 1/32. The initial Taiwan regression path remains centered 6 MHz.
- Use `liquid-dsp` as a proof-of-concept backend for soft QAM demapping and
  punctured K=7 convolutional decoding. Before adopting it in the release
  decoder, verify DVB-T constellation bit ordering, puncturing patterns, and
  continuous-trellis/reset semantics against known vectors and captured IQ.

## Production soft-decoding pipeline

The long-term decoder should not depend on a generic modem abstraction at its
core. Its inner-decoder path is intended to be:

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
  punctured bits before decoding the K=7, G1=171 octal, G2=133 octal mother
  code.
- Wrap libcorrect with streaming traceback and deterministic superframe reset
  behavior; do not assume its packet-oriented API directly matches DVB-T.
- Keep the decoder API independent of Airspy, SoapySDR, the UI, and the
  transport-stream consumer.

Initial native implementation:

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
- On the 557 MHz capture this frontend currently acquires 8K, guard 1/4 with a
  CP score around 0.96 and stable scattered-pilot phase, but produces only
  about 8.4 dB equalized MER versus roughly 16.2 dB from the GNU Radio
  reference equalizer. It therefore does not yet acquire the outer RS phase or
  emit valid TS. This checkpoint must not be presented as decoder lock.
- Spectrum and quality smoothing follow SDR++'s speed model
  (`alpha = min(speed / (update_rate * 10), 1)`). Raw FFT rows reach the
  waterfall before FFT smoothing is applied to the spectrum trace.

Acceptance criteria:

- Decode synthetic standard vectors without uncorrected RS errors.
- Recover valid PAT and PMT tables from the 557 MHz regression capture, with
  plausible PIDs and a low transport-error rate.
- Track acquisition time, TPS lock, MER, pre-Viterbi BER, post-Viterbi BER,
  corrected RS packets, and uncorrectable packets as regression metrics.
- Compare hard- and soft-decision paths on clean, weak-signal, multipath, and
  discontinuous captures.

Next decoder step:

- Promote the diagnostic cyclic-prefix/pilot analyzer into a continuous
  native frontend that emits exactly 1512/6048 payload carriers per symbol.
- Add continual-pilot common-phase correction and carrier/sample-clock loops;
  the current scattered-pilot-only equalizer is measurably below the reference
  receiver on the captured multipath channel.
- Add TPS differential demodulation, BCH validation, frame/superframe index,
  modulation/code-rate discovery, and deterministic decoder reset tags.
- Add continuous carrier and sample-clock tracking before connecting live/file
  CS16 sources directly to the validated equalized-carrier decoder.

## Transport stream and playback

- Parse PAT, PMT, SDT, and EIT and expose service selection.
- Record the recovered MPEG-TS with duration, size, and throughput telemetry.
- Feed a selected service to libmpv and render video into the application-owned
  OpenGL framebuffer.
- Reset decoder and playback state cleanly after source discontinuities,
  retunes, or service changes.

## Possible future work

- DVB-T2 support after the DVB-T receiver is stable and covered by regression
  captures. DVB-T2 has a distinct framing, pilot, interleaving, and FEC chain
  and therefore belongs in a separate decoder module rather than a mode switch
  inside the DVB-T inner decoder.
- Additional native SDR backends when they provide useful capabilities that a
  generic SoapySDR path cannot expose.
