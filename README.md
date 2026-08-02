# Airspy TV

Early C++20 mockup for a standalone DVB-T receiver application. The current milestone provides:

- a single-window SDL3 + Dear ImGui interface;
- native Airspy and generic SoapySDR device enumeration/opening;
- Airspy sensitivity/linearity gain profiles;
- bounded-queue CS16 raw I/Q recording with a JSON metadata sidecar;
- mock spectrum, waterfall, constellation, decoder metrics, and video areas.

The project does **not** decode DVB-T or play video yet. Values in the visualization and signal-quality
sections are explicitly mock data.

## Build

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

Required system libraries are SDL3, OpenGL, libairspy, SoapySDR, and a C++20 compiler. Dear ImGui and
nlohmann/json are pinned submodules under `contrib/`.

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
`<recording>.json` with its sample rate, center frequency, source, sample count, and drop counters.
