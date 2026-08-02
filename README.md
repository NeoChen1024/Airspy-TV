# Airspy TV

Early C++20 mockup for a standalone DVB-T receiver application. The current milestone provides:

- a single-window SDL3 + Dear ImGui interface;
- an always-centered, per-digit mouse-wheel frequency control;
- native Airspy and generic SoapySDR device enumeration/opening;
- Airspy sensitivity/linearity gain profiles;
- bounded-queue CS16 raw I/Q recording with a JSON metadata sidecar;
- real-time playback of recorded sidecars and raw `airspy_rx` INT16_IQ files;
- live 4096-bin FFTW/VOLK spectrum, selectable-colormap waterfall, and dBFS
  signal-power telemetry, with an adjustable display range defaulting to
  -100/-20 dBFS;
- live diagnostic DVB-T constellation and OFDM quality metrics;
- a native equalized-carrier-to-MPEG-TS decoder with soft Viterbi,
  RS(204,188), and all non-hierarchical DVB-T modulation/code-rate modes;
- mock video playback areas.

The native path recovers MPEG-TS directly from an ideal centered 10 MSPS CS16
DVB-T waveform. Live/file I/Q sources feed the same asynchronous native
OFDM/FEC worker and its output is routed to the GUI's TS recorder. Robust
carrier/sample-clock tracking and TPS parameter discovery are still under
development, so recorded weak/multipath signals remain experimental.

## Build

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

Required system libraries are SDL3, OpenGL, libairspy, SoapySDR, FFTW3f, VOLK, and a C++20 compiler.
Dear ImGui, nlohmann/json, tinycolormap, liquid-dsp, and libcorrect are pinned
submodules under `contrib/`. The native decoder uses libcorrect for soft
Viterbi and shortened Reed-Solomon decoding, and liquid-dsp for exact rational
input resampling. Cubehelix is the default waterfall colormap.

List visible SDR devices without starting the GUI:

```sh
./build/airspy-tv --enumerate
```

For a short end-to-end source/recorder diagnostic using the first native Airspy (or first Soapy device
when no native Airspy is present):

```sh
./build/airspy-tv --record-first /tmp/airspy-tv-smoke.cs16 250
```

Recordings are raw interleaved little-endian signed 16-bit I/Q (`ci16_le`). Stopping a recording writes
`<recording>.json` with the raw data filename, sample rate, center frequency, source, duration, sample
count, and drop counters. The filename is a basename only; the sidecar and raw data remain in the same
directory.

The Source panel can open either the JSON sidecar or a raw `.cs16`/`.iq` file. Sidecars supply the
sample rate and center frequency. A bare file is interpreted as the same interleaved signed 16-bit I/Q
layout written by `airspy_rx -t 2`; set its sample rate in the Source panel and its center frequency in
the top bar before opening it. A headless file-source check is also available:

```sh
./build/airspy-tv --inspect-iq capture.cs16.json
./build/airspy-tv --inspect-iq airspy-rx-output.iq 10000000 545000000
```

For the current GNU Radio equalizer/native decoder cross-check, decode an
8K/64-QAM/rate-2/3 file containing 6048 `complex<float>` payload carriers per
symbol with:

```sh
./build/airspy-tv-dvbt-equalized equalized.cfile output.ts 0
```

The final argument is the first OFDM symbol index; it determines the symbol
deinterleaver parity. This tool is a validation boundary, not the eventual
user-facing I/Q decoder.

An offline GNU Radio reference transmitter can generate a deterministic ideal
6 MHz, 8K, guard-1/4, 64-QAM, rate-2/3 fixture. This validation helper requires
GNU Radio's Python bindings, NumPy, and FFmpeg; none are application runtime
dependencies.

```sh
XDG_CACHE_HOME=/tmp/airspy-tv-gnuradio-cache \
  python3 tools/generate_dvbt_fixture.py /tmp/airspy-tv-ideal.cs16
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target airspy-tv-dvbt-iq
./build-release/airspy-tv-dvbt-iq \
  /tmp/airspy-tv-ideal.cs16 /tmp/airspy-tv-ideal-native.ts 1
```

The generator also writes an application-compatible JSON I/Q sidecar and the
unmodulated source transport stream as `airspy-tv-ideal.expected.ts`. The
current ideal regression recovers packet-aligned TS with a valid PAT/PMT and
no uncorrectable RS packets.
