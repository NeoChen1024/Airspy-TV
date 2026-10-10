# Airspy TV — Agent guide

Airspy R2 is the primary hardware target for this C++20 terrestrial television
receiver. DVB-T is implemented. DVB-T2 is the next digital standard but is on
hold until off-air T2 recordings are available for validation.

## Scope and ownership

- Preserve the multi-standard `Demodulator` interface and receiver lifecycle
  needed for DVB-T2. Keep shared source, transport, recording and playback code
  independent of a particular modulation or FEC chain. Keep standard-specific
  parameters and diagnostics typed rather than building a universal schema.
- Prefer straightforward computation and existing shared mechanisms. Remove
  unused adapters, obsolete interfaces and duplicated state; do not add generic
  frameworks or compatibility layers without a concrete caller.
- Native Airspy, SoapySDR and recorded I/Q are supported sources. GNU Radio is
  an independent fixture generator, not an application runtime dependency.
- Do not change vendored code or update submodules as incidental cleanup.
  Preserve third-party licenses and notices.

## Signal and lifecycle integrity

- Preserve sample rates, units, I/Q ordering, sample positions, discontinuities
  and missing-data semantics. Never fabricate lock, measurements or TS packets.
- Live input must remain non-blocking with bounded buffering and reported drops;
  offline decoding must apply backpressure without silently losing samples.
- Preserve ordered joins before stateful FEC, generation-safe reset, and source
  worker shutdown before decoder replacement. Do not simplify these guarantees
  merely to reduce line count.
- Keep disk I/O, report serialization and GUI work outside DSP hot paths.
- Treat input captures as read-only. Keep generated I/Q, TS, reports and large
  fixtures out of Git; preserve unrelated files and existing recordings.

## Tests and verification

- Keep tests for observable correctness: reference DSP/FEC results, recovered TS
  bytes, framing and metadata, partial I/O, failure propagation, resource bounds,
  backpressure, cancellation and reset/lifetime safety.
- Do not freeze incidental choices such as worker splits, default queue sizes,
  callback block sizes, GUI labels, alert thresholds or exact internal event
  counts. A configured capacity being respected is a contract; its default value
  is usually a tuning choice.
- Remove redundant tests instead of preserving obsolete code for their sake.
  Do not add or expand tests unless requested. Use representative inputs and
  direct output inspection for proportionate verification.
- Use the configured CMake build in `build/`; see `CMakePresets.json` for portable
  and sanitizer configurations. Run relevant tests after changing behavior and
  the full CTest suite for shared runtime/build changes.
- Synthetic round trips are not independent proof of RF correctness. Use the
  GNU Radio fixtures for end-to-end validation when changing DSP. State whether
  real captures, Airspy hardware and GUI playback were actually exercised.

## Documentation and style

- Write project files, comments and documentation in English. Follow the root
  `.clang-format` and `.clang-tidy`; review behavior-changing tidy suggestions.
- Describe current supported behavior and distinguish it from planned work.
  Keep this guide about project-wide conventions, not per-tool defaults.
- Give each topic one primary home: `README.md` for usage, `ROADMAP.md` for
  planned scope, `docs/` for architecture and signal/report interpretation, and
  `scripts/README.md` for the extended validation workflow.
- Preserve scientific interpretation details, units and report validity rules.
  Avoid duplicating source code, debugging histories and test transcripts in
  documentation. Git history is the archive; report validation in the task reply.
