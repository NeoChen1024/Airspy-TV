# Airspy TV

Airspy TV is a standalone C++20 DVB-T receiver and diagnostics application.
The current implementation provides:

- a single-window SDL3 + Dear ImGui interface with embedded libmpv playback;
- an always-centered, per-digit mouse-wheel frequency control;
- selectable 5/6/7/8 MHz DVB-T channel bandwidth applied consistently to
  spectrum metrics, OFDM monitoring, and transport decoding;
- native Airspy and generic SoapySDR device enumeration/opening;
- Airspy sensitivity/linearity gain profiles;
- bounded-queue CS16 raw I/Q recording with a JSON metadata sidecar;
- real-time playback of recorded sidecars and raw `airspy_rx` INT16_IQ files;
- decoder-paced offline I/Q-to-MPEG-TS extraction through the main executable;
- live 4096-bin FFTW/VOLK spectrum, selectable-colormap waterfall, and dBFS
  signal-power telemetry, with an adjustable display range defaulting to
  -100/-20 dBFS;
- live diagnostic DVB-T constellation, OFDM quality metrics, and measured
  pre-/post-Viterbi BER;
- a native raw-I/Q-to-MPEG-TS decoder with 2K/8K OFDM acquisition, soft
  Viterbi, RS(204,188), and all non-hierarchical DVB-T modulation/code-rate
  modes;
- MPEG-TS recording with duration, size, and throughput telemetry.
- BCH-validated TPS parameter discovery and PAT/PMT/SDT service discovery with
  a live service-selection drop-down.

The native path recovers MPEG-TS directly from an ideal centered 10 MSPS CS16
6 MHz DVB-T waveform. Live/file I/Q sources feed the same asynchronous native
OFDM/FEC worker and its output is routed to the GUI's TS recorder. Robust
carrier/sample-clock tracking is still under development. Automatic transport
decoding now waits for differential TPS synchronization and BCH validation,
then uses the advertised constellation and high-priority code rate; manual UI
parameters remain available as test overrides. Clean 557 MHz and 581 MHz Airspy
recordings recover valid TS, while weak/multipath recordings remain
experimental.

Raw-I/Q processing uses a 100 ms overlap-save boundary. The overlapping region
is decoded independently, matched as an exact sequence of 188-byte transport
packets, and emitted only once. This preserves multiplex continuity across
internal processing chunks without making file input lossy; `--debug` reports
the joined-packet and failed-join counters.

The right-side video surface feeds the decoded transport stream to libmpv
through a bounded custom stream and renders video into an application-owned
OpenGL framebuffer. PAT, PMT, and SDT populate the service drop-down; selecting
a service restarts playback with only its PMT, PCR, audio, and video PIDs while
retaining the required PSI/SI packets. Volume and mute are controlled directly
from the video footer. TS recording intentionally continues to write the full
MPTS.

## Build

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

Debug builds default to `-O3 -g -DNDEBUG` so the binary retains debugger
symbols while the realtime DSP and libcorrect SIMD paths run at Release-like
speed. Configure with `-DAIRSPY_TV_OPTIMIZED_DEBUG=OFF` when assertion-enabled,
unoptimized debugging is more important than receiver throughput.

Builds also default to `-march=native` so FFT/equalizer/demapper code can use
the build host's instruction set. Use `-DAIRSPY_TV_NATIVE_ARCH=OFF` for a
portable binary intended to run on other CPUs.

Required system libraries are SDL3, OpenGL, libairspy, SoapySDR, libmpv,
Fontconfig, FFTW3f, VOLK, and a C++20 compiler.
Dear ImGui, nlohmann/json, tinycolormap, liquid-dsp, and libcorrect are pinned
submodules under `contrib/`. The native decoder uses libcorrect for soft
Viterbi and shortened Reed-Solomon decoding, and liquid-dsp for exact rational
input resampling. Cubehelix is the default waterfall colormap.

List visible SDR devices without starting the GUI:

```sh
./build/airspy-tv --enumerate
```

For a short end-to-end source/recorder diagnostic using the first native Airspy
(or first Soapy device when no native Airspy is present):

```sh
./build/airspy-tv --record-first /tmp/airspy-tv-smoke.cs16 250
```

Recordings are raw interleaved little-endian signed 16-bit I/Q (`ci16_le`).
Stopping a recording writes `<recording>.json` with the raw data filename,
sample rate, center frequency, source, duration, sample count, and drop
counters. The filename is a basename only; the sidecar and raw data remain in
the same directory.

The Source panel can open either the JSON sidecar or a raw `.cs16`/`.iq` file.
Sidecars supply the sample rate and center frequency. A bare file is interpreted
as the same interleaved signed 16-bit I/Q layout written by `airspy_rx -t 2`;
set its sample rate in the Source panel and its center frequency in the top bar
before opening it. A headless file-source check is also available:

```sh
./build/airspy-tv --inspect-iq capture.cs16.json
./build/airspy-tv --inspect-iq airspy-rx-output.iq 10000000 545000000
```

Decode a finite capture to MPEG-TS without throttling it to its recorded sample
rate with:

```sh
./build-release/airspy-tv --decode-iq capture.cs16.json output.ts
./build-release/airspy-tv --decode-iq airspy-rx-output.iq output.ts 10000000
./build-release/airspy-tv --decode-iq capture.cs16.json output.ts \
  --decoder-threads 8
./build-release/airspy-tv --decode-iq capture.cs16.json output.ts --debug
```

The decoder uses a blocking submission path for offline input and applies
backpressure at its native processing-chunk boundary. It runs as quickly as the
CPU permits without dropping file blocks or simulating 10 MSPS wall-clock
playback. JSON input uses the same sidecar resolver as the GUI; the optional
sample rate is only needed for bare raw INT16_IQ files.

The decoder's parallel-worker budget defaults to
`std::thread::hardware_concurrency()` (logical CPUs, with a one-worker fallback
when unavailable). It is divided between an independent-symbol postprocessing
pool and the Viterbi window pool; the partitioned resampler reuses the full
budget before those stages begin. `--decoder-threads N` overrides the budget
for offline decoding; the same setting is available in the GUI Source panel and
is deliberately locked while an SDR or I/Q file source is open. `0` selects
the automatic default. The old `--viterbi-threads` spelling remains an alias.
Normal progress output is one compact line per processing chunk. Pass `-d` or
`--debug` to also print worker allocation, per-stage timing, and the final
decoder/FEC summary.

The shared GUI/CLI decoder is a bounded ordered pipeline. Its front-end performs
partitioned rational resampling, acquisition, FFT, pilot tracking and channel
interpolation. Resampler partitions reconstruct their FIR history and therefore
remain bit-identical to serial liquid-dsp output. A symbol pool performs
decision-directed gain correction, MER/error estimation and carrier reliability
calculation, Max-Log demapping, symbol/bit deinterleaving, depuncturing, and
soft-byte quantization out of order, then rejoins mother-code metric blocks in
input order. The FEC worker dispatches overlapping Viterbi windows to
independent libcorrect contexts and rejoins results before Reed-Solomon and TS
output. A full queue applies backpressure instead of dropping symbols, because
a single missing symbol would invalidate the stateful convolutional and
outer-interleaver stream.

Backpressure queues are sized by payload duration rather than callback block
count. Raw I/Q, OFDM-symbol/FEC, and Viterbi-window queues retain approximately
200 ms of their respective streams to tolerate ordinary OS scheduler jitter.
Disk recorders have their own deeper buffers: five seconds of raw I/Q and more
than five seconds of maximum-rate DVB-T transport stream. Spectrum and
signal-quality workers remain latest-snapshot mailboxes intentionally, so a
delayed GUI never works through stale displays.

An offline GNU Radio reference transmitter can generate a deterministic ideal
6 MHz, 8K, guard-1/4, 64-QAM, rate-2/3 fixture. This validation helper requires
GNU Radio's Python bindings, NumPy, and FFmpeg; none are application runtime
dependencies.

```sh
XDG_CACHE_HOME=/tmp/airspy-tv-gnuradio-cache \
  python3 tools/generate_dvbt_fixture.py /tmp/airspy-tv-ideal.cs16
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target airspy-tv
./build-release/airspy-tv --decode-iq \
  /tmp/airspy-tv-ideal.cs16.json /tmp/airspy-tv-ideal-native.ts
```

The generator also writes an application-compatible JSON I/Q sidecar and the
unmodulated source transport stream as `airspy-tv-ideal.expected.ts`. The
current ideal regression deterministically recovers packet-aligned TS with a
valid PAT and PMT.
