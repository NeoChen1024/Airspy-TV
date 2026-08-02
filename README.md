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

The native data/FEC path can recover MPEG-TS from equalized DVB-T carriers.
Live/file I/Q sources now feed an asynchronous native OFDM/FEC worker and its
output is routed to the GUI's TS recorder. Carrier/sample-clock tracking and
TPS parameter discovery are still under development, however, so the current
raw-IQ frontend does not yet recover valid TS from the regression capture.

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
