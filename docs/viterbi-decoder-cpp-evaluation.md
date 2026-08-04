# ViterbiDecoderCpp as a libcorrect replacement — feasibility evaluation

Status: **implemented and verified (both backends pass).** Submodule at
`contrib/viterbi-decoder-cpp` (pinned `2696ffe`, v1.4-49-g2696ffe). The AVX2
soft-Viterbi backend is integrated behind the build-time option
`AIRSPY_TV_USE_AVX2_VITERBI` (default ON); libcorrect remains for the RS outer
FEC and as the no-AVX2 fallback for the convolutional path.

## Motivation

`ROADMAP.md` lists as a remaining item:

> Profile a modern AVX2 Viterbi implementation; libcorrect's SSE decoder is now
> the dominant CPU hotspot after the ordered-pipeline optimizations.

[ViterbiDecoderCpp](https://github.com/williamyang98/ViterbiDecoderCpp) is a C++
port of Phil Karn's libfec Viterbi decoder with AVX2/SSE/NEON intrinsics and
measured speedups up to 32x over scalar.

## Library overview

- Header-only, C++17, LGPL-2.1 (Phil Karn's libfec derivative — compatible with
  this GPL-3.0 project).
- Template parameters fix constraint length `K` and code rate `R` at compile
  time; the generator polynomials (`G[]`) are passed at runtime via a
  `ViterbiBranchTable` that can be shared between decoders.
- Decoder implementations: scalar, `x86` SSE4.1 (`u8`/`u16`) and AVX2
  (`u8`/`u16`), `arm` NEON (`u8`/`u16`).
- Minimum `K` for each SIMD path: AVX2-u16 needs `K>=6`, AVX2-u8 needs `K>=7`,
  SSE4.1-u16 needs `K>=5`, SSE4.1-u8 needs `K>=6`.
- Upstream README caveat: "This code is not considered heavily tested".

## API paradigm comparison

| | libcorrect (current) | ViterbiDecoderCpp |
| --- | --- | --- |
| Language | C | C++17 templates, header-only |
| Code config | runtime (`correct_convolutional_sse_create(rate, order, polys)`) | compile time (`ViterbiDecoder_Core<K,R,error_t,soft_t>`) + runtime `G[]` |
| Handle model | opaque `correct_convolutional_sse*`, state persists across calls | explicit `ViterbiDecoder_Core` value object; `reset()` / `update()` / `chainback()` |
| Decode call | one-shot `decode_soft(dec, soft, len, out)` | three explicit stages: `reset(state)`, `Decoder::update(dec, symbols, N)`, `chainback(out, bits)` |
| Traceback | internal (fixed depth inside libcorrect) | caller-owned: `set_traceback_length()` sizes a decision buffer; over-running it is only an `assert` (silent UB with `NDEBUG`) |
| Soft input | `uint8_t` 0..255, 128 = neutral (punctures) | `int8_t`/`int16_t` signed; punctures filled with any constant (constant branch error is neutral for path selection) |
| Failure mode | returns `-1` on length error | no error returns; caller must manage sizes exactly |
| Threading | one decoder per worker (current design) | one `ViterbiDecoder_Core` per worker + one shared `ViterbiBranchTable` (const) |

## Integration impact on the current architecture

Impact is small and confined to `src/fec/soft_viterbi.cpp` internals. The
`SoftViterbi` public API, the worker pool, the overlapping-window scheme
(8192-bit window, 256-bit margins, 7680-bit output), the puncturing contract
("caller depunctures by inserting 128"), and the pre-Viterbi BER estimator can
all stay as they are.

### 1. Generator polynomials — zero conversion

libcorrect currently receives `{0117, 0155}` (octal literals = 79, 109
decimal). ViterbiDecoderCpp's `G[]` uses the same libfec "reversed" convention
(see its DAB example: octal 171 → 79, octal 133 → 109). DVB-T's G1=171/G2=133
therefore maps directly to `{79, 109}` — the identical values. No
reordering/bit-reversal needed, but must be confirmed by fixture comparison.

### 2. Soft metric domain

`SoftViterbi::Impl` stores quantized `uint8_t` 0..255 (128 = neutral,
`127.5 + llr*8.0`).

- **u16 backend (recommended):** convert with `soft_i16 = soft_u8 - 128`
  (range −128..127, neutral = 0). Branch table `±127`; max branch error
  `254*2 = 508` fits `uint16_t` comfortably. Keeps current LLR quantization
  gain (8.0) unchanged — important for the weak-signal MER 8–12 dB cases the
  roadmap tracks.
- **u8 backend (fastest):** AVX2-u8 is 32-way but the upstream README warns
  8-bit metrics need a small soft range (`max_error ≤ 255` ⇒ soft span ≤ 127,
  i.e. about ±63 here). That would force lowering the quantization gain to
  ~4.0, losing 1 bit of LLR resolution. Only attractive for hard-decision
  decoding; not recommended for DVB-T soft decoding.

### 3. Puncturing — mathematically neutral

`decode_punctured_symbols()` (upstream example) fills punctured positions with
a constant (`unpunctured_symbol_value = 0` in their DAB demo). Adding a
constant to every state's accumulated error does not change survivor selection,
so the current "insert 128, feed full rate-1/2 stream" contract produces the
same result: 128 − 128 = 0, exactly the upstream neutral convention. The
depuncture layer and `SoftViterbi` interface remain untouched.

Note: we should NOT adopt the upstream helper's per-bit `update()` loop (one
`update` per R symbols); a single `update()` over the full 16384-symbol window
keeps the AVX2 path efficient.

### 4. Window/traceback management

Current design: each worker owns one libcorrect handle; every 16384-symbol
window is decoded through one `decode_soft` call with continuous internal state.

Replacement: per window, `reset(0)` → `update(dec, metrics, 16384)` →
`chainback(out, 8192)` → take the middle 7680 bits. Because `ViterbiDecoder_Core`
has no ring buffer (over-running the decision buffer is only an assert), each
window must be decoded independently — which the 256-bit margins already make
equivalent, and which *improves* determinism (no cross-window state). The 256-bit
margins (K=7) are ample for trellis convergence.

### 5. CMake integration

The repo has no root `CMakeLists.txt`, only `viterbi-config.cmake`
(`find_package(viterbi CONFIG)`). Follow the existing `imgui` precedent and add
a header-only INTERFACE target in the top-level `CMakeLists.txt`:

```cmake
add_library(viterbi-decoder INTERFACE)
target_include_directories(viterbi-decoder SYSTEM INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/contrib/viterbi-decoder-cpp/include")
target_compile_features(viterbi-decoder INTERFACE cxx_std_17)
```

**Decision (2026-02): compile-time SIMD, no runtime dispatch.** The decoder
type is a compile-time template parameter, and the AVX2/SSE intrinsics require
their ISA flags at compile time. Airspy-TV already builds the whole pipeline
with `-march=native` (`AIRSPY_TV_NATIVE_ARCH` default ON) and libcorrect's
`HAVE_SSE` is likewise a compile-time probe, so the binary was never portable
anyway. We follow the same convention: `using Decoder = ViterbiDecoder_AVX_u16<K,R>`
fixed at compile time, zero runtime overhead, no CPUID dispatch. The SIMD
headers dispatch per-instantiation type (`ViterbiDecoder_AVX_u8/u16`,
`_SSE_u8/u16`, `ViterbiDecoder_Scalar`, NEON), so an `#if defined(__AVX2__)`
etc. chain picks the type.

### 6. What "replace libcorrect" actually means

libcorrect provides both Reed–Solomon and convolutional codes. ViterbiDecoderCpp
only covers convolutional decoding, so **libcorrect must stay** for the RS outer
FEC (`src/fec/outer_fec.cpp`, the two RS tests). Only the Viterbi path in
`soft_viterbi.cpp` is replaceable. Estimated footprint: ~1 header include +
rewrite of `create_decoder`/`destroy_decoder`/`decode` plus a shared branch
table — all inside `SoftViterbi::Impl`.

## Expected performance

| Path | Width | Speedup vs scalar | vs current libcorrect SSE |
| --- | --- | --- | --- |
| libcorrect SSE (16-bit metrics) | 8-way | ~8x | 1x (baseline) |
| ViterbiDecoderCpp AVX2 u16 | 16-way | 16x | ~2x |
| ViterbiDecoderCpp AVX2 u8 | 32-way | 32x | ~4x (precision risk, see above) |

libcorrect's SSE decoder is reported as the dominant CPU hotspot; an AVX2-u16
swap should roughly halve that hot-spot share. `chainback` and the BER estimator
remain scalar, so end-to-end gain will be less than the theoretical SIMD ratio;
measure with the existing 40 MB fixture before/after.

## Risks and open items

1. **u8 metric range** (upstream-documented) — prefer u16 for soft DVB-T.
2. **No error returns / assert-only bounds** — keep the fixed window sizes and
   wrap the three-stage call; do not feed variable lengths.
3. **Upstream maturity** — "not considered heavily tested"; pin the submodule
   and verify with the repo's own `run_tests.cpp`/`run_snr_ber.cpp` (examples)
   plus Airspy-TV fixtures.
4. **Bit ordering / traceback direction** — must reproduce the current
   byte-identical output on the 557 MHz and 40 MB fixtures (MD5 checks) before
   switching the default backend.
5. **License** — LGPL-2.1 headers compiled into a GPL-3.0 program is
   compatible; keep the Phil Karn / William Yang copyright headers intact.
6. **Non-x86** — NEON/scalar fallback for ARM, or keep libcorrect there.

## Implementation status

Done:

1. CMake: `AIRSPY_TV_USE_AVX2_VITERBI` option (default ON) + `viterbi-decoder`
   header-only INTERFACE target; `airspy-tv-common` links it and defines the
   macro when enabled (`CMakeLists.txt`).
2. `src/fec/soft_viterbi.cpp`: new backend selected by
   `#if defined(AIRSPY_TV_USE_AVX2_VITERBI) && defined(__AVX2__)`. Each worker
   owns a `ViterbiDecoder_Core<7,2,uint16_t,int16_t>` (AVX2-u16) sharing one
   const `ViterbiBranchTable<7,2,int16_t>`; per window `reset(0)` →
   `update(16396 symbols)` (window + (K-1) zeroed tail symbols) →
   `chainback(8192)`. Soft metrics are converted 0..255 → −127..127 (128 →
   0, clamped) with a per-worker `thread_local` buffer. The window slicing and
   pre-Viterbi BER estimator moved to backend-independent `extract_output()`.
   Without `__AVX2__` the file falls back to the libcorrect scalar/SSE path.
3. Verified:
   - Default (AVX2) build: `ctest` 4/4 pass (incl. full-pipeline
     `dvbt-stream-decoder`).
   - `-DAIRSPY_TV_USE_AVX2_VITERBI=OFF` (libcorrect): `ctest` 4/4 pass.
   - A/B harness (temporary, not committed) decoded the same 8192-bit
     AWGN-noisy stream with both backends: 0 bit mismatches, including with a
     rate-2/3 depuncture pattern (4099 symbols set to neutral 128) — confirms
     the constant-insertion puncturing equivalence.
4. Remaining: `perf` profile before/after (ROADMAP's AVX2 item), real-fixture
   (557 MHz / 40 MB) MD5 comparison once available, then optionally drop
   libcorrect's convolutional part and keep it for RS only.

## Remaining / suggested next steps

1. Sanity-check the upstream library itself: build and run the submodule's own
   `examples/run_tests.cpp` (and `run_snr_ber.cpp`) once.
2. Real-fixture MD5 comparison (557 MHz / 40 MB captures) between the two
   backends before any future default switch.
3. Profile (`perf`) before/after; update ROADMAP's AVX2 item.
4. Once stable and fixture-verified, drop libcorrect's convolutional part and
   keep it for RS only (or keep the option for no-AVX2 builds).
