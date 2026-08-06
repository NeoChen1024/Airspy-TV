# Airspy TV

Airspy TV is a standalone C++20 DVB-T receiver that turns live SDR or recorded
I/Q samples into watchable television. It includes a native DVB-T decoder,
real-time RF diagnostics, service selection, a now/next EPG guide, embedded
libmpv playback, and raw I/Q/MPEG-TS recording in one application.

![Airspy TV receiving and playing a Taiwanese DVB-T service](images/Screenshot_20260803_142304.jpg)

The current receiver can lock and play clean 6 MHz Taiwanese DVB-T captures
from an Airspy R2, including automatic TPS parameter discovery and
PAT/PMT/SDT-based channel selection. The native decoder and GUI share the same
processing path used by the command-line I/Q-to-TS tool.

## Project vision

Airspy TV's goal is a standalone terrestrial broadcast decoder with the Airspy
R2 as a first-class SDR: one application that turns the R2's I/Q stream into
watchable television without external demodulators or signal-processing tools.
DVB-T is the implemented standard; DVB-T2, DTMB, ATSC, and analog
(NTSC/PAL/SECAM), plus DVB-C, are documented long-term targets in the
[ROADMAP](ROADMAP.md). Decoding, TS/PSI/SI/EPG parsing, recording, and playback
all run in-process, so GNU Radio appears only as an offline test fixture.

## Highlights

- Native Airspy R2/Mini support through libairspy, including sensitivity and
  linearity gain profiles, Bias-T control, and dropped-sample reporting.
- Generic SDR support through SoapySDR for compatible devices with sufficient
  sample rate and usable bandwidth.
- Live 4096-bin spectrum and waterfall with Blackman-Harris windowing, FFT
  smoothing, adjustable dBFS range, and selectable tinycolormap palettes.
- Live DVB-T constellation, signal power, CP SNR, MER, deepest-notch estimate,
  carrier offset, pre-/post-Viterbi BER, and decoder/CPU queue diagnostics.
- Native 2K/8K DVB-T demodulation with QPSK, 16-QAM, 64-QAM, all
  non-hierarchical code rates, soft Viterbi, and RS(204,188) decoding.
- BCH-validated TPS discovery of transmission mode, guard interval,
  constellation, and high-priority code rate, with manual overrides for
  testing.
- Embedded libmpv video/audio playback rendered into the application OpenGL
  surface, with service selection, volume, and mute controls.
- Sidebar EPG now/next guide for the selected service, decoded from EIT
  present/following and TDT/TOT clock data with iconv-based DVB text support
  (Big5, GB2312, EUC-KR, ISO-8859-x, and the Taiwan mislabeled-UTF-16 quirk).
- Raw CS16 I/Q and full-multiplex MPEG-TS recording with duration, size,
  throughput, and drop telemetry.
- Real-time replay of application sidecars and raw `airspy_rx` INT16_IQ files.
- Decoder-paced offline I/Q-to-MPEG-TS extraction without realtime throttling
  or file-input drops.

## Current status

The end-to-end path is operational for clean, centered DVB-T signals:

```text
Airspy / SoapySDR / CS16 file
        -> native DVB-T OFDM + FEC decoder
        -> MPEG transport stream
        -> service selection
        -> libmpv video and audio
```

Clean 557 MHz and 581 MHz Airspy recordings recover valid transport streams and
play in the GUI. EIT present/following data from the same streams drives the
EPG panel, which shows the selected service's current and next programs with
correct names, times, and durations for both Taiwanese broadcasters (581 MHz
TTV and 557 MHz FTV). Weak signals and difficult multipath environments remain
experimental while carrier, sample-clock, and channel tracking are improved.
DVB-T2 is not currently implemented.

## Build

Required system libraries:

- SDL3 and OpenGL
- libairspy and SoapySDR
- libmpv and Fontconfig
- FFTW3f and VOLK
- a C++20 compiler and CMake 3.25 or newer

Dear ImGui, nlohmann/json, tinycolormap, liquid-dsp, and libcorrect are pinned
under `contrib/` as Git submodules.

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/airspy-tv
```

Debug builds default to `-O3 -g -DNDEBUG`, retaining debugger symbols while the
DSP and libcorrect SIMD paths run at Release-like speed. Configure with
`-DAIRSPY_TV_OPTIMIZED_DEBUG=OFF` for an assertion-enabled, unoptimized Debug
build.

Builds also default to `-march=native`. Use
`-DAIRSPY_TV_NATIVE_ARCH=OFF` when producing a portable binary for a different
CPU.

## Using the receiver

The Source panel exposes the decoder worker budget and available input sources.
Open a native Airspy, a compatible SoapySDR device, or an I/Q recording, then:

1. Select the DVB-T channel bandwidth (5, 6, 7, or 8 MHz).
2. Tune the always-centered frequency control; the mouse wheel changes the
   digit currently under the pointer.
3. Leave DVB-T mode parameters on Auto for TPS discovery, or set them manually
   for diagnostics.
4. Wait for OFDM and TS lock, then choose a service in the video footer.
5. Adjust volume or mute playback, and optionally record raw I/Q or the full
   MPEG transport stream. The EPG sidebar panel follows the selected service
   and shows its current and next programs.

The service selector filters playback to the selected service's PMT, PCR,
audio, and video PIDs while retaining required PSI/SI packets. MPEG-TS recording
intentionally writes the complete MPTS rather than only the selected service.

## I/Q files and recording

Application recordings use raw interleaved little-endian signed 16-bit I/Q
(`ci16_le`). Stopping a recording writes a JSON sidecar beside the sample data.
It records the data filename, sample rate, center frequency, source, duration,
sample count, and drop counters.

The sidecar and data file must remain in the same directory. Open the JSON file
from the Source panel and Airspy TV resolves the data filename from its
metadata. Bare `.cs16` and `.iq` files are also supported and use the
INT16_IQ layout produced by `airspy_rx -t 2`; specify their sample rate and
center frequency manually.

| Input | Metadata | Playback behavior |
|---|---|---|
| Airspy native | Device/driver supplied | Live |
| SoapySDR | Device/driver supplied | Live |
| Airspy TV `.json` sidecar | Sample rate, center frequency, data filename | Real time |
| Bare `.cs16` / `.iq` | Enter sample rate and center frequency manually | Real time |

The raw I/Q recorder has a five-second queue. DSP queues retain approximately
200 ms of their respective streams to tolerate ordinary scheduler jitter; RF
input remains non-blocking because live hardware cannot accept backpressure.

## Command-line tools

List visible SDR devices without starting the GUI:

```sh
./build/airspy-tv --enumerate
```

Run a short source/recorder diagnostic using the first native Airspy, falling
back to the first SoapySDR device:

```sh
./build/airspy-tv --record-first /tmp/airspy-tv-smoke.cs16 --duration 250
```

Inspect a sidecar or bare I/Q file without starting the GUI:

```sh
./build/airspy-tv --inspect-iq capture.cs16.json
./build/airspy-tv --inspect-iq airspy-rx-output.iq \
  --sample-rate 10000000 --frequency 545000000
```

Decode finite I/Q input to MPEG-TS as quickly as the CPU permits:

```sh
./build/airspy-tv --decode-iq capture.cs16.json --ts-output output.ts
./build/airspy-tv --decode-iq airspy-rx-output.iq \
  --ts-output output.ts --sample-rate 10000000
./build/airspy-tv --decode-iq capture.cs16.json \
  --ts-output output.ts --decoder-threads 8
./build/airspy-tv --decode-iq capture.cs16.json \
  --ts-output output.ts --debug
```

JSON input uses the same sidecar resolver as the GUI. A sample rate is only
needed for bare INT16_IQ files; pass it with `--sample-rate HZ`. Offline
decoding uses blocking submission and does not drop input when the decoder is
slower than the file reader.

The decoder worker budget defaults to `std::thread::hardware_concurrency()`.
`--decoder-threads N` overrides it; the same option appears in the GUI and is
fixed while a source is open. `0` selects the automatic default. The DVB-T
transmission parameters (`--dvbt-mode`, `--dvbt-channel-bandwidth`,
`--dvbt-guard`, `--dvbt-modulation`, `--dvbt-code-rate`) default to
auto-detection from the TPS and can be forced when the signal is marginal or
the capture metadata is incomplete. Pass `-d` or `--debug` to include worker
allocation, per-stage timings, and detailed FEC statistics.

## Decoder architecture

<details>
<summary>Native DVB-T processing pipeline</summary>

The shared GUI/CLI decoder is a bounded ordered pipeline. Its front end performs
partitioned rational resampling, OFDM acquisition, FFT, pilot tracking, channel
interpolation, decision-directed gain correction, MER estimation, carrier
reliability calculation, Max-Log demapping, symbol/bit deinterleaving, and
depuncturing.

Independent OFDM symbols are processed by a worker pool and rejoined in input
order. The FEC worker dispatches overlapping Viterbi windows to independent
libcorrect contexts, performs an ordered join, then runs convolutional
deinterleaving, Reed-Solomon decoding, energy descrambling, and TS recovery.

Raw-I/Q chunks use a 100 ms overlap-save boundary. Packet-aligned overlap
matching emits the shared transport region only once and preserves continuity
across processing chunks. Backpressure is applied to offline decoding instead
of dropping symbols, because a missing symbol invalidates the stateful
convolutional and outer-interleaver stream.

</details>

## Reproducible test signal

An offline GNU Radio reference transmitter generates a deterministic ideal
6 MHz, 8K, guard-1/4, 64-QAM, rate-2/3 fixture. This helper requires GNU
Radio's Python bindings, NumPy, and FFmpeg, but they are not application runtime
dependencies.

```sh
XDG_CACHE_HOME=/tmp/airspy-tv-gnuradio-cache \
  python3 tools/generate_dvbt_fixture.py /tmp/airspy-tv-ideal.cs16
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target airspy-tv
./build-release/airspy-tv --decode-iq \
  /tmp/airspy-tv-ideal.cs16.json --ts-output /tmp/airspy-tv-ideal-native.ts
```

The generator writes an application-compatible JSON sidecar and the
unmodulated source stream as `airspy-tv-ideal.expected.ts`. The current ideal
regression deterministically recovers packet-aligned TS with a valid PAT and
PMT.

Sample-clock and LO impairments can be injected independently. Offsets set the
initial error; drift rates change that error linearly over the fixture:

```sh
XDG_CACHE_HOME=/tmp/airspy-tv-gnuradio-cache \
  python3 tools/generate_dvbt_fixture.py /tmp/airspy-tv-drift.cs16 \
    --duration 120 \
    --sample-clock-ppm 1.5 \
    --sample-clock-drift-ppm-per-minute 0.2 \
    --lo-offset-hz 750 \
    --lo-drift-hz-per-minute -25
```

The JSON sidecar records all four impairment parameters while retaining the
nominal 10 MS/s sample rate expected by the decoder.
