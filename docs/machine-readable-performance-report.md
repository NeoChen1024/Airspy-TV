# Machine-readable decode reports

## Status and goals

This document specifies the proposed long-run report format for offline I/Q
decoding. It is a design for the next implementation phase, not a description
of an existing CLI feature.

The report must support performance comparisons and clock/FEC failure analysis
without parsing the verbose `--debug` output. A report is a directory containing
a manifest, several homogeneous detailed telemetry streams, and one concise
final summary. For a DVB-T run:

```text
REPORT_DIR/
  manifest.json
  stats.json
  frontend.jsonl
  pipeline.jsonl
  dvbt-demod.jsonl
  dvbt-fec.jsonl
```

Each JSONL file contains exactly one record schema. Different subsystems or
emission cadences are not interleaved in one generic telemetry file. Future
modes add mode-specific streams such as `dvbt2-demod.jsonl` without changing
the common frontend or pipeline schemas.

Human-readable debug timing remains available during the transition, but its
format is allowed to change. It must become a renderer of the same typed
telemetry records used by JSONL and the summary aggregator, rather than retain
a second flat timing schema or independent field mapping. Important decoder
events may remain on their existing debug path until structured event records
are introduced.

## Existing JSON support

The project already vendors nlohmann/json 3.12.0 and links it through the
`nlohmann_json::nlohmann_json` target. `iq_file.cpp` uses it for metadata input
and `recorder.cpp` uses it for metadata output. The common
`airspy_tv::JsonlReader` and `airspy_tv::JsonlWriter` wrappers own JSONL framing,
line/record numbering, parse and I/O errors, and the explicit non-finite-number
policy. They accept non-owning standard streams, so report files, stdin/stdout,
and in-memory tests share the same code path. `JsonlWriter::write_batch()`
accepts a span, allowing vectors or fixed arrays of records to be validated and
serialized before one stream write. A JSON array passed to `write()` remains
one JSONL record; batch framing is always explicit.

The writer produces one compact JSON value per line and does not retain the
JSONL file as one in-memory array. Report code uses it as follows:

```cpp
JsonlWriter telemetry(stream);
telemetry.write(record);
```

Report conversion must use `json_finite_or_null()` to write non-finite
floating-point values as `null`. The writer rejects any unhandled NaN or
infinity rather than relying on a serializer's implicit replacement behavior.

## CLI contract

Add the following offline-decode option:

```text
--report-dir DIR
```

The directory may be absent or empty. Decoding refuses a non-empty directory
so a new run cannot silently mix with or overwrite an older report.
`manifest.json` and all streams listed in it are created immediately. Every
JSONL writer is flushed at least once per wall-clock second and at shutdown.
`stats.json` is initially written with `"status": "running"`, then atomically
replaced at shutdown with one of:

- `completed`: decoding completed and emitted transport packets;
- `no_transport`: decoding completed but emitted no transport packets;
- `failed`: an input, decoder, report, or transport-output error occurred.

The final object includes the process exit code and an error string or `null`.
If the process is interrupted, the running status and the valid JSONL prefix
make the incomplete run identifiable and still analyzable.

`manifest.json` is ordinary JSON and remains immutable after report creation.
It contains the report-format version, receiver mode, run/tool identity,
effective configuration, and the stream inventory. Each inventory entry names
the path, one `record_type`, and its independent `schema_version`, for example:

```json
{
  "report_format_version": 0,
  "mode": "dvbt",
  "streams": [
    {
      "path": "frontend.jsonl",
      "record_type": "frontend_block",
      "schema_version": 0
    },
    {
      "path": "dvbt-demod.jsonl",
      "record_type": "demod_window",
      "schema_version": 0
    }
  ]
}
```

An empty listed JSONL file means that the stage emitted no records; readers do
not need to infer whether a missing file is intentional.

All report-owned version fields start at `0` while the format is pre-alpha:
`report_format_version` in the manifest, every JSONL `schema_version`, and the
`schema_version` in `stats.json`. They may evolve independently, but none is
promoted to `1` before its corresponding format is stable.

`--decode-iq -` reads raw interleaved little-endian CS16 from standard input.
Because standard input has no sidecar metadata, this form requires an explicit
`--sample-rate`; the argument parser therefore needs to remember whether the
option was supplied rather than accepting the current implicit 10 MHz default.
Input size and signal duration remain unknown until EOF.

`--ts-output -` writes only MPEG-TS bytes to standard output. All progress,
warnings, debug diagnostics, and errors continue to use standard error, so the
following is valid:

```sh
producer | airspy-tv --decode-iq - --sample-rate 10000000 \
  --ts-output - | consumer
```

Path collision checks apply only when both operands are real paths.

## Human-readable progress

Offline decoding prints one concise line to standard error at most once per
wall-clock second, regardless of `--debug`. The rate limiter uses
`std::chrono::steady_clock`, not input chunk count or decoded signal time. A
final line is always printed. The first implementation should use newline
records for predictable redirected logs.

Example:

```text
wall=12.0s input=111.6s speed=9.3x MER=23.6dB OFDM=lock TPS=lock TS=207.0MiB TEI=0 IQ= 42% Demod= 98% FEC=  3%
```

The three pipeline percentages retain the GUI order and fixed-width format:
IQ queue, demod busy, then FEC queue. `--debug` additionally renders each typed
telemetry record with the same record type, metric names, scopes, units, and
validity rules used by JSONL. Its text layout does not need to remain compatible
with the current multiline timing dump.

## Telemetry delivery

`StreamDecoderStats::stats()` is a latest-state snapshot. It cannot be the sole
source for JSONL because offline decoding may publish multiple 400-symbol
windows between polls. Its frontend, demod, and FEC values also describe
different intervals:

- frontend timing describes the most recently completed input block;
- demod timing describes one 400-symbol window (or a final partial window);
- FEC timing is complete only when the corresponding marker reaches the FEC
  worker.

The decoder should therefore expose an opt-in, drainable queue of typed DVB-T
telemetry records. Each owning stage appends its completed record under the
existing coordinator mutex. The CLI drains records after submissions and after
`flush()`, then performs JSON conversion and file I/O on the CLI thread.
Normal GUI and non-report users leave collection disabled, so they incur no
record retention. No decoder worker may serialize JSON or write the report.

Collection is enabled when either `--report-dir` or `--debug` requires detailed
records. After each drain, the CLI passes the same record batch to zero or more
consumers:

1. a report router that batches records by their homogeneous JSONL stream;
2. the human-readable debug formatter;
3. the bounded `stats.json` aggregator.

No consumer re-reads `StreamDecoderStats` to reconstruct stage timing. When
both report and debug output are enabled, one drained record is formatted twice
without duplicating telemetry extraction or maintaining parallel names. The
latest-state `StreamDecoderStats` remains useful for GUI and one-second pipeline
progress, but it is not the canonical detailed timing representation.

This remains a standard-specific API on `dvbt::StreamDecoder`; it should not be
added to the standard-neutral `Demodulator` interface. A future DVB-T2, DVB-S,
or DVB-C decoder can expose its own typed records while sharing the report
writer's common envelope and timing conventions.

Every record carries:

- `schema_version` (initially `0` while the format is pre-alpha);
- `record_type`;
- `mode`, using stable lowercase receiver-mode names such as `dvbt`, `dvbt2`,
  `dvbc`, `dvbs`, or `dvbs2`;
- a monotonically increasing sequence in that record domain;
- decoder generation and source stream epoch where applicable;
- wall-clock elapsed time from the run start;
- source and/or resampled sample positions sufficient for correlation.

`mode` is a required top-level field on every record, including records whose
payload is mode-specific. A reader can therefore dispatch or reject a detached
line without first reading `manifest.json` or retaining session state. JSON
object key order has no semantic meaning, so readers must look up the field by
name rather than require it to be the first serialized key. DVB-T's 2K/8K
parameter is named `transmission_mode` to avoid overloading the receiver-mode
field.

The first report version uses these homogeneous streams:

| File | Record type | Emission point | Purpose |
| --- | --- | --- | --- |
| `frontend.jsonl` | `frontend_block` | one input block completes | CS16 conversion, resampling, SRO actuator, and source/output sample spans |
| `pipeline.jsonl` | `pipeline_sample` | once per wall-clock second | Queue occupancy, worker states, latest locks/quality, and cumulative progress |
| `dvbt-demod.jsonl` | `demod_window` | one stats window completes | Lock, RF/OFDM quality, CFO/SRO/timing state, and demod/symbol timing |
| `dvbt-fec.jsonl` | `fec_window` | a numbered stats marker reaches FEC | FEC timing, session identity, output deltas, BER, RS, TEI, and sync state |

Lifecycle and configuration belong in `manifest.json` and `stats.json`, not as
different `run_start`/`run_end` schemas mixed into a telemetry stream. A future
`events.jsonl` may be added only after it has one stable event-envelope schema;
existing human-readable event diagnostics remain outside the first version.

FEC stats markers need the associated demod-window sequence. FEC decoder
creation/reset also needs a monotonically increasing `fec_session` identifier.
This avoids pretending that a later FEC snapshot belongs to whichever demod
window happens to be in `latest` at drain time.

There is no total ordering across files. Correlation uses the common monotonic
wall timestamp, decoder generation, source epoch, source/resampled sample
positions, and explicit demod-window sequence carried into `fec_window`.

## Timing names and scopes

Metric names use `subsystem::operation`, for example `demod::fft`. Prefixes
remain present even inside a subsystem-specific record so tools can merge and
compare maps without inventing names.

Timing values are milliseconds, but they are separated by scope so consumers
cannot accidentally sum nested values or aggregate worker work as serial wall
time:

```json
{
  "schema_version": 0,
  "record_type": "demod_window",
  "mode": "dvbt",
  "timing_ms": {
    "serial_busy": {
      "demod::ring_copy": 6.50,
      "demod::fft_cfo": 17.31,
      "demod::pilot_lock": 9.87,
      "demod::channel": 24.04,
      "demod::other": 2.13
    },
    "wait": {
      "demod::ring_wait": 1.72
    },
    "nested": {
      "demod::nco": 7.72,
      "demod::fft": 8.30,
      "demod::cfo_track": 0.91,
      "demod::channel::timing": 12.45,
      "demod::channel::interpolate": 4.83
    },
    "aggregate_worker_work": {
      "symbol::preprocess": 57.28,
      "symbol::demap": 21.43
    }
  }
}
```

The complete first-version metric set is:

- frontend serial wall: `frontend::total`, `frontend::convert`,
  `frontend::resample`, `frontend::ring_copy`, `frontend::ring_wait`, and
  `frontend::other`;
- demod serial busy: `demod::ring_copy`, `demod::fft_cfo`,
  `demod::pilot_lock`, `demod::reacquisition`, `demod::channel`, `demod::tps`,
  `demod::payload_extract`, `demod::symbol_submit`,
  `demod::postprocess_wait`, `demod::output`, and `demod::other`;
- demod outside busy: `demod::ring_wait`;
- FFT/CFO nested: `demod::nco`, `demod::fft`, `demod::cfo_track`, and
  `demod::fft_cfo_other`;
- channel nested: `demod::channel::pilots`, `demod::channel::notch`,
  `demod::channel::timing`, `demod::channel::cir`,
  `demod::channel::interpolate`, `demod::channel::tps_extract`, and
  `demod::channel::other`;
- aggregate symbol-worker work: `symbol::preprocess`, `symbol::demap`,
  `symbol::deinterleave`, and `symbol::depuncture`;
- FEC serial wall: `fec::total`, with `fec::transport` explicitly marked as a
  nested subset rather than an additive peer.

Each record also carries its applicable total wall time, sample/symbol count,
and interval endpoints. Percentages are derived values and should not be
stored in detailed records when the numerator and denominator are available.

## Detailed values

In addition to timing, the first JSONL schema records the following data when
it is valid:

- input: source epoch, sample rate, source begin/end sample, submitted and
  processed complex samples, and discontinuity state;
- resampler: resampled begin/mid/end sample, requested/effective ratio,
  commanded/applied SRO, command/effective/applied input positions, fixed
  delay, late samples, and pending commands;
- OFDM/TPS: lock state, ever-locked state, FFT/guard size, constellation, code
  rate, hierarchy, carrier-bin offset, acquisition score/start, and whether a
  re-anchor carried state;
- signal and tracking: MER, fade indicator, tracked and residual CFO, raw,
  filtered and physical timing, observed/smoothed drift, timing/CIR confidence,
  and accepted/rejected measurement counts;
- pipeline: queue occupancy and capacity, worker state/count, dropped blocks,
  FEC gating state, overlap joins/failures, and cumulative TS bytes;
- FEC: session ID, pre/post-Viterbi error and compared bits, RS packets and
  uncorrectable packets, TEI and TS packets, outer-deinterleaver phase/evidence,
  and RS/energy synchronization.

Unavailable measurements are `null`, not magic zero values. Enum values and
states are stable lowercase strings rather than implementation ordinals.

## `stats.json`

The final summary is intentionally smaller than the telemetry stream. It
contains:

- `schema_version: 0`, status, program version, and optional build/revision
  identity;
- top-level receiver `mode`, source/destination descriptors, and effective
  decoder configuration;
- wall duration, decoded signal duration, submitted/processed samples, and
  average realtime speed;
- output bytes, actual emitted 188-byte packet count, emitted partial bytes,
  and usable packet count after TEI when that cumulative counter is available;
- cumulative dropped blocks, OFDM symbols, lock-weighted window counts, phase
  discontinuities, overlap joins/failures, FEC resets/sessions, Viterbi bit
  errors, RS packets/failures, and TEI packets;
- MER and SRO/CFO/timing count/min/mean/max over valid demod windows;
- for each timing key, sample count, total, mean, and maximum milliseconds;
- final lock and clock-tracking state;
- exit code and error.

`transport_bytes / 188` (plus a separately reported remainder) is the
authoritative count of packets actually delivered to the TS callback. The
current `TransportDecoderStats` values describe only the active FEC decoder
session and can reset on a gated region or parameter change. Before report
summaries claim run-wide RS, TEI, BER, or TS totals, the FEC stage must preserve
final per-session counters and expose monotonic cumulative values. Inferring a
reset only from a decreasing polled counter is not exact and is not acceptable.

Percentiles are deliberately omitted from the first in-process summary. The
online aggregator remains bounded by keeping count, total, minimum, and
maximum; p50/p95 can be calculated exactly from the relevant homogeneous JSONL
stream by regression tools without growing decoder memory during an indefinite
run.

## Implementation order

1. Add stdin/stdout stream ownership, explicit sample-rate tracking, and the
   steady-clock one-second progress line without changing decoder telemetry.
2. Add a report-directory writer with immutable `manifest.json`, homogeneous
   JSONL stream routing, `pipeline_sample`, atomic `stats.json`, explicit
   finite-number conversion, and unit tests for valid JSON/JSONL and failure
   status.
3. Add the opt-in typed telemetry queue, exact frontend/demod sample spans, FEC
   window IDs/sessions, and monotonic run-wide transport/FEC counters.
4. Serialize all stage records, compute bounded summary aggregates, and replace
   the current debug timing field dump with a human formatter over those same
   records and timing maps.
5. Validate a short stdin/stdout pipeline and compare TS output byte-for-byte
   with file input/output. Then run the long 545 MHz capture and confirm that
   report mode does not change TS output, dropped-block count, or throughput.
6. Remove the old flat debug timing formatter after byte-for-byte decoder
   validation. Keep rare human-readable event diagnostics until equivalent
   structured event records exist.

Tests must cover a non-empty report-directory refusal, output/report write
failure, no-transport exit status, final partial input, and JSON validity for
unlocked/invalid measurements. Schema tests must also reject a mode-specific
record with a missing or unsupported top-level `mode` and must distinguish
`mode: "dvbt"` from DVB-T `transmission_mode: "2k"` or `"8k"`.
