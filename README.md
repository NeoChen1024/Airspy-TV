# Airspy TV

Airspy TV is a standalone C++20 DVB-T receiver that turns live SDR or recorded
I/Q samples into watchable television. It includes a native DVB-T decoder,
real-time RF diagnostics, service selection, a now/next EPG guide, embedded
libmpv playback, and raw I/Q/MPEG-TS recording in one application.

![Airspy TV receiving and playing a Taiwanese DVB-T service](images/ptv.jpg)

The current receiver can lock and play clean 6 MHz Taiwanese DVB-T captures
from an Airspy R2, including automatic TPS parameter discovery and
PAT/PMT/SDT-based channel selection. The native decoder and GUI share the same
processing path used by the command-line I/Q-to-TS tool.

## Project vision

Airspy TV's goal is a standalone terrestrial broadcast decoder with the Airspy
R2 as a first-class SDR: one application that turns the R2's I/Q stream into
watchable television without external demodulators or signal-processing tools.
DVB-T is the implemented standard; DVB-T2, ATSC, and analog
(NTSC/PAL/SECAM) are documented long-term targets in the [ROADMAP](ROADMAP.md).
Decoding, TS/PSI/SI/EPG parsing, recording, and playback all run in-process,
so GNU Radio appears only as an offline test fixture.

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
- Headless live SDR or real-time I/Q replay to MPEG-TS file/stdout and
  RTP/UDP, using the same receiver, decoder, reporting, and bounded output
  paths as the GUI.
- Continuous DVB-T sample-clock correction through the common variable-rate
  arbitrary resampler, independently of LO/CFO tracking.

## Current status

The end-to-end path is operational for clean, centered DVB-T signals:

```text
Airspy / SoapySDR / CS16 file
        -> native DVB-T OFDM + FEC decoder
        -> MPEG transport stream
        -> service selection
        -> libmpv video and audio
```

Clean and multipath 557/581 MHz Airspy recordings recover valid transport
streams and play in the GUI. The 151.6-minute 545 MHz replay is the current
long-run timing baseline: sample-clock tracking remains bounded and TS output
continues past the point where the earlier timing loop failed. EIT
present/following data drives the selected service's now/next guide.

Hierarchical DVB-T and captured long-delay/SFN validation remain unfinished.
DVB-T2 and the other roadmap standards are not implemented.

## Build

Required system libraries:

- SDL3 and OpenGL
- libairspy and SoapySDR
- libmpv and Fontconfig
- FFTW3f
- a C++20 compiler and CMake 3.25 or newer

Dear ImGui, nlohmann/json, tinycolormap, and ViterbiDecoderCpp are pinned under
`contrib/` as Git submodules. The minimized MIT-licensed solid-resampler and
LGPL-licensed libfec Reed-Solomon subsets are vendored directly under
`contrib/`.

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/airspy-tv
```

Debug builds default to `-O3 -g -DNDEBUG`, retaining debugger symbols while the
optimized DSP and Viterbi paths run at Release-like speed. Configure with
`-DAIRSPY_TV_OPTIMIZED_DEBUG=OFF` for an assertion-enabled, unoptimized Debug
build.

Builds also default to `-march=native`. Use
`-DAIRSPY_TV_NATIVE_ARCH=OFF` when producing a portable binary for a different
CPU. The default soft-Viterbi backend is ViterbiDecoderCpp's AVX2-u16
implementation when the compiler target supports AVX2, with SSE4.1 and AArch64
NEON selected on suitable targets and a scalar backend used otherwise.
Configure with `-DAIRSPY_TV_USE_SIMD_VITERBI=OFF` to force the scalar backend.

Checked-in CMake presets provide reproducible optimized, assertion-enabled,
portable, and sanitizer builds. Each configure preset has a matching build and
test preset:

```sh
cmake --preset optimized-debug
cmake --build --preset optimized-debug
ctest --preset optimized-debug

cmake --preset asan-ubsan
cmake --build --preset asan-ubsan
ctest --preset asan-ubsan

cmake --preset tsan
cmake --build --preset tsan
ctest --preset tsan
```

The available names are `optimized-debug`, `assertion-debug`,
`portable-release`, `asan-ubsan`, and `tsan`. The ASan preset also enables
UBSan and integrated LeakSanitizer. TSan uses a separate binary because its
runtime cannot be combined with ASan. Sanitizer and portable presets disable
host-native code generation and the optional SIMD Viterbi backend; they are
correctness configurations, not realtime throughput baselines.

The repository validation runner configures, builds, and runs CTest for
`portable-release`, `asan-ubsan`, and `tsan`, then exercises the full synthetic
DVB-T matrix under portable and ASan builds and a deterministic all-pairs set
under TSan. It also validates exact recovered TS identity and every JSON/JSONL
decode report. Each run emits a versioned machine-readable manifest, bounded
summary and environment description, homogeneous stage/CTest/fixture JSONL
streams, optional real-signal corpus benchmarks, native JUnit, full stage logs,
and complete failure artifacts. See
[scripts/README.md](scripts/README.md) for the report schemas, reproducible venv
bootstrap, smoke mode, and independently adjustable parallelism.

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

| Input                      | Metadata                                        | Playback behavior |
| -------------------------- | ----------------------------------------------- | ----------------- |
| Airspy native              | Device/driver supplied                          | Live              |
| SoapySDR                   | Device/driver supplied                          | Live              |
| Airspy TV`.json` sidecar | Sample rate, center frequency, data filename    | Real time         |
| Bare`.cs16` / `.iq`    | Enter sample rate and center frequency manually | Real time         |

The raw I/Q recorder has a five-second queue and the full-MPTS recorder has an
independent 24 MiB write queue. DSP queues retain approximately 200 ms of their
respective streams to tolerate ordinary scheduler jitter; RF input remains
non-blocking because live hardware cannot accept backpressure. The separate
mpv queue has an 8 MiB capacity, enters buffering below 1 MiB, and resumes at
2 MiB so a short decoder dropout does not immediately become playback
stutter.

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

Decode a live SDR source until `SIGINT` or `SIGTERM`. With no `--device`, the
first native Airspy is preferred and the first remaining SDR is the fallback.
RTP follows RFC 2250 with payload type 33 and normally carries seven TS packets
per UDP datagram:

```sh
./build/airspy-tv --decode-live --frequency 545000000 \
  --sample-rate 10000000 --ts-output live.ts
./build/airspy-tv --decode-live --device DEVICE_ID \
  --frequency 545000000 --ts-output - | mpv -
./build/airspy-tv --decode-live --frequency 545000000 \
  --rtp-output 192.0.2.10:5004
./build/airspy-tv --decode-live --frequency 545000000 \
  --rtp-output '[2001:db8::10]:5004'
```

`DEVICE_ID` is the exact third column printed by `--enumerate`. A finite I/Q
capture can use the same live path with wall-clock pacing, which is useful for
testing an RTP receiver without SDR hardware:

```sh
./build/airspy-tv --decode-live --iq-input capture.cs16.json \
  --rtp-output 127.0.0.1:5004
./build/airspy-tv --decode-live --iq-input capture.cs16 \
  --sample-rate 10000000 --ts-output replay.ts \
  --rtp-output '[::1]:5004'
producer | ./build/airspy-tv --decode-live --iq-input - \
  --sample-rate 10000000 --rtp-output 127.0.0.1:5004
```

I/Q sidecar metadata is honored. Stdin requires an explicit sample rate and is
cancellable even while its producer is idle. Normal file or stdin EOF completes
the live command. The live input callback remains non-blocking. File and stdout
output use an independent 8 MiB queue. If downstream output cannot keep up, the
CLI warns on stderr and drops the oldest queued TS blocks so reception stays at
the live edge. An actual file/stdout write error fails the command. RTP/UDP
queue pressure or send errors instead drop datagrams, update counters, and
leave reception and other outputs running. The GUI exposes the same IPv4/IPv6
RTP output between the MPEG-TS and raw I/Q recorder panels.

The decoder worker budget defaults to `std::thread::hardware_concurrency()`.
`--decoder-threads N` overrides it; the same option appears in the GUI and is
fixed while a source is open. `0` selects the automatic default. The DVB-T
transmission parameters (`--dvbt-mode`, `--dvbt-channel-bandwidth`,
`--dvbt-guard`, `--dvbt-modulation`, `--dvbt-code-rate`) default to
auto-detection from the TPS and can be forced when the signal is marginal or
the capture metadata is incomplete. Pass `-d` or `--debug` to include worker
allocation, per-stage timings, tracking events, and detailed FEC diagnostics.

Pass `--report-dir DIR` to offline decoding, live decoding, or the GUI receiver
to write the machine-readable report described in
[`docs/machine-readable-performance-report.md`](docs/machine-readable-performance-report.md).
In GUI mode the report starts with the first opened source and is finalized at
application shutdown. Each I/Q replay, source reopen, sample-rate restart, or
retune appends one final record to `source-sessions.jsonl`; `stats.json` remains
a run-wide summary. The directory must be absent or empty when the run starts.

## Decoder architecture

<details>
<summary>Native DVB-T processing pipeline</summary>

The shared GUI/CLI decoder is a continuous bounded pipeline. A frontend thread
converts and resamples CS16 input into an absolute-position sample ring. The
serial demod thread owns acquisition, FFT-window position, carrier and timing
loops, channel state, TPS, and symbol order.

Independent symbol postprocessing is dispatched to a worker pool and rejoined
by sequence. A separate FEC thread sends overlapping mother-code windows to
the Viterbi pool, performs another ordered join, then serializes convolutional
byte deinterleaving, RS(204,188), energy descrambling, and MPEG-TS output. The
soft Viterbi stage uses the best compile-time ViterbiDecoderCpp backend, with a
portable scalar fallback. A minimized libfec subset provides RS(204,188).

There are no per-chunk decoder seams or packet-overlap joins. Generation-tagged
reset and retune handling suppress stale worker results before they can reach
stateful FEC or output sinks. Live sources drop and report an input block on
overload; offline decoding applies backpressure instead.

See [docs/worker-pools-and-dataflow.md](docs/worker-pools-and-dataflow.md) for
thread, queue, ordering, and reset details. Clock-tracking experiments and
remaining validation are maintained in
[docs/clock-tracking.md](docs/clock-tracking.md).

</details>

## Reproducible test signal

A GNU Radio reference transmitter generates deterministic ideal DVB-T fixtures
in 2K or 8K mode, every guard interval, 5/6/7/8 MHz bandwidth, QPSK/16-QAM/
64-QAM, and every non-hierarchical code rate. This helper requires GNU Radio's
Python bindings, NumPy, and FFmpeg, but they are not application runtime
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

Use `-` to stream CS16 directly to offline decoding without creating a large
I/Q file. Generator and GNU Radio diagnostics remain on stderr. The fixed CS16
scale leaves frontend-like headroom at approximately -20 dBFS RMS instead of
normalizing each finite fixture to its largest sample:

```sh
XDG_CACHE_HOME=/tmp/airspy-tv-gnuradio-cache \
  python3 tools/generate_dvbt_fixture.py - \
    --duration 3 --dvbt-mode 2k --dvbt-guard 1/32 |
./build/airspy-tv --decode-iq - --sample-rate 10000000 \
  --ts-output /tmp/airspy-tv-2k.ts
```

The same producer can exercise wall-clock pacing and RTP through the live stdin
source:

```sh
XDG_CACHE_HOME=/tmp/airspy-tv-gnuradio-cache \
  python3 tools/generate_dvbt_fixture.py - \
    --duration 10 --dvbt-mode 2k --dvbt-guard 1/32 |
./build/airspy-tv --decode-live --iq-input - --sample-rate 10000000 \
  --rtp-output 127.0.0.1:5004
```

Run the checked-in streaming end-to-end validator to require zero TEI packets
and exact cyclic packet identity against the source transport stream:

```sh
python3 tools/validate_dvbt_fixture_stream.py \
  --build-dir build --dvbt-mode 2k --dvbt-guard 1/32
```

Pass `--work-dir PATH` to retain each run in a new child directory containing
its expected TS, recovered TS, logs, and machine-readable report. Without that
option, no CS16 fixture is written and all small supporting artifacts are
temporary.

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

Run the end-to-end clock regression to generate and decode sample-clock-only,
LO-only, and independent sample/LO drift fixtures:

```sh
python3 tools/validate_dvbt_clock_drift.py \
  --build-dir build \
  --duration 20
```

The decoder always feeds the DVB-T timing estimator's SRO command into the
common variable-rate resampler. Commands use a fixed sample-domain activation
horizon and each applied ratio is tracked against the exact input/output sample
span which it generated. The validator checks both the estimated and applied
correction; there is no separate legacy integer timing-actuator mode.

By default, generated fixtures, decoder logs, and transport streams use a
temporary directory that is removed after the run. Pass `--work-dir PATH` to
retain those artifacts for inspection. The validator checks the recovered SRO
and CFO independently, requires non-empty TS output, and rejects timing/FEC
failure events.
