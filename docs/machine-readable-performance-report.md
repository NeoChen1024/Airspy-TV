# Machine-readable decode reports

## Status and goals

This document specifies the implemented long-run report format for offline I/Q
decoding and interactive GUI receiver sessions. The first pre-alpha
implementation landed on August 7, 2026. All report-owned schema versions
remain `0` while field names and stream contents are still allowed to evolve.

The report must support performance comparisons and clock/FEC failure analysis
without parsing the verbose `--debug` output. A report is a directory containing
a manifest, several homogeneous detailed telemetry streams, and one concise
final summary. For a DVB-T run:

```text
REPORT_DIR/
  manifest.json
  stats.json
  source-sessions.jsonl
  frontend.jsonl
  pipeline.jsonl
  dvbt-demod.jsonl
  dvbt-fec.jsonl
  events.jsonl
```

Each JSONL file contains exactly one record schema. Different subsystems or
emission cadences are not interleaved in one generic telemetry file. Future
modes add mode-specific streams such as `dvbt2-demod.jsonl` without changing
the common frontend or pipeline schemas.

Human-readable debug timing remains available during the transition, but its
format is allowed to change. It must become a renderer of the same typed
telemetry records used by JSONL and the summary aggregator, rather than retain
a second flat timing schema or independent field mapping. Decoder events use
the same typed queue and are routed to one homogeneous `decoder_event` stream;
`--debug` renders those records instead of receiving direct worker-thread
`fprintf` output.

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

## Report activation

The same option enables reports for offline decoding and the GUI receiver:

```text
--report-dir DIR
```

The directory may be absent or empty. Decoding refuses a non-empty directory so
a new run cannot silently mix with or overwrite an older report. Offline mode
creates the report immediately and writes exactly one final source-session
record. GUI mode arms the report at startup and creates it when the first SDR
or I/Q-file source opens. The writer remains open until GUI shutdown. I/Q EOF,
explicit source close, replay, sample-rate restart, and retune end the current
source session without finalizing the run; the next successful stream start
appends another record to `source-sessions.jsonl`.

Every JSONL writer is flushed at least once per wall-clock second and at
shutdown. `stats.json` is initially written with `"status": "running"`, then
atomically replaced at shutdown with one of:

- `completed`: decoding completed and emitted transport packets;
- `no_transport`: decoding completed but emitted no transport packets;
- `failed`: an input, decoder, report, or transport-output error occurred.

The final object includes the process exit code and an error string or `null`.
If the process is interrupted, the running status and the valid JSONL prefix
make the incomplete run identifiable and still analyzable.

`manifest.json` is ordinary JSON and remains immutable after report creation.
It contains the report-format version, receiver mode, run context/tool
identity, and the stream inventory. Per-source descriptors and decoder
configuration are deliberately excluded. Each inventory entry names the path,
one `record_type`, and its independent `schema_version`, for example:

```json
{
  "report_format_version": 0,
  "mode": "dvbt",
  "streams": [
    {
      "path": "source-sessions.jsonl",
      "record_type": "source_session",
      "schema_version": 0
    },
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

`--ts-output -` writes only MPEG-TS bytes to standard output for both
`--decode-iq` and `--decode-live`. All progress,
warnings, debug diagnostics, and errors continue to use standard error, so the
following is valid:

```sh
producer | airspy-tv --decode-iq - --sample-rate 10000000 \
  --ts-output - | consumer
```

`--decode-live --iq-input PATH|-` uses the same sidecar-aware file source as the
GUI, or raw little-endian CS16 from stdin, but paces samples against wall clock
rather than decoding as fast as possible. Stdin requires an explicit sample
rate and uses cancellable reads. It may write file/stdout MPEG-TS, RTP/UDP, or
both; normal file or stdin EOF finalizes one source-session record without
classifying the run as an input failure.

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

The decoder exposes an opt-in, drainable queue of typed DVB-T telemetry
records. Each owning stage appends its completed record under the existing
coordinator mutex. Offline decoding drains records after submissions and after
`flush()`; the GUI drains them during frame-state refresh. Both paths perform
JSON conversion and file I/O on their application thread. GUI sessions without
a report or `--debug`, and other non-report users, leave collection disabled,
so they incur no record retention. No decoder worker may serialize JSON or
write the report.

Collection is enabled when either `--report-dir` or `--debug` requires detailed
records. After each drain, the owning application path passes the same record
batch to zero or more consumers:

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
| `source-sessions.jsonl` | `source_session` | one source session ends | Source/destination/configuration, correlation ranges, per-source totals, outcome, and final decoder state |
| `frontend.jsonl` | `frontend_block` | one input block completes | CS16 conversion, resampling, SRO actuator, and source/output sample spans |
| `pipeline.jsonl` | `pipeline_sample` | once per wall-clock second | Queue occupancy, worker states, latest locks/quality, and cumulative progress |
| `dvbt-demod.jsonl` | `demod_window` | one stats window completes | Lock, RF/OFDM quality, CFO/SRO/timing state, and demod/symbol timing |
| `dvbt-fec.jsonl` | `fec_window` | a numbered stats marker reaches FEC | FEC timing, session identity, output deltas, BER, RS, TEI, and sync state |
| `events.jsonl` | `decoder_event` | a diagnostic state transition occurs | Acquisition, lock, fade, phase, timing-rejection, and FEC-gating events |

Run lifecycle belongs in `manifest.json` and `stats.json`. Per-source lifecycle
and configuration belong in the homogeneous `source_session` stream, not as
different `run_start`/`run_end` schemas mixed into detailed telemetry.

Every event record has the same envelope: `event`, `severity`, nullable source,
resampled-sample and OFDM-symbol positions, and a `fields` object containing
event-specific typed values. Event names and enum-valued fields use stable
lowercase strings. Different events do not introduce different top-level
schemas. The initial event names are:

- `acquisition_succeeded` and `acquisition_wait_timeout`;
- `reanchor_triggered` and `reanchor_result`;
- `fade_enter`, `fade_exit`, and `bad_lock_enter`;
- `pilot_phase_jump`;
- `timing_branch_change` and `timing_measurement_rejected`;
- `tps_lock` and `hopeless_gate_window`;
- `outer_fec_alignment_search`, `outer_fec_alignment_pending`,
  `outer_fec_alignment_locked`, and `outer_fec_alignment_missed`;
- `outer_fec_rs_attempt`, `outer_fec_rs_failure_streak`,
  `outer_fec_rs_recovered`, and `outer_fec_reset`.

Common FEC components publish mode-independent typed diagnostic events through
an optional handler. The DVB-T FEC worker adds its current `fec_session`,
`demod_window_sequence`, source epoch, and TPS symbol index before placing the
event on the receiver telemetry queue. The enabled predicate is checked before
event fields are constructed, so an inactive report/debug path does not add
map allocation to the normal FEC hot path.

Periodic timing-loop state is not an event. It remains part of each
`demod_window` record and the shared `--debug` demod-window formatter.

### Diagnostic-output boundary

Decoder algorithm diagnostics and state transitions must use typed telemetry;
leaf DSP/FEC components must not read the process-global debug flag or format
their own `stderr` records. This keeps `events.jsonl`, event counts, and
`--debug` as different consumers of one representation.

The following output deliberately remains outside that event stream:

- command-line errors, warnings, and once-per-second offline progress;
- the demod worker's last-resort synchronous exception message immediately
  before the exception is propagated to the coordinator.

The process-global debug flag therefore only selects human-readable rendering.
It is not an input to common decoding components.

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
      "demod::fft": 16.02,
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
- FFT/CFO nested: `demod::fft`, `demod::cfo_track`, and
  `demod::fft_cfo_other`; carrier translation is performed by the frontend
  resampler and therefore has no separate demod NCO bucket;
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

Frontend records also carry the common resampler's independent SRO and CFO
actuator state. Demod-window records separate acquisition CFO, estimated
absolute CFO, residual CFO, latest scheduled command coordinates, and the
effective/actual source positions of the most recently applied command. This
distinction is required when a newer command is still pending behind the fixed
sample-domain horizon. Frequency-tracking records also expose abrupt-CFO
rebootstrap request/completion counts, the last trigger residual, and its
source/output sample coordinates. The corresponding
`cfo_rebootstrap_requested` event records the trigger reason and recovery
state.

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

## Source-session correlation

`source-sessions.jsonl` contains one compact final record per successful source
start. Its source and destination descriptors, center/sample frequencies,
DVB-T decoder configuration, wall and source-sample ranges, status/error, and
final decoder state apply only to that source session. The record also contains
the same bounded measurements, timing, event, transport, and FEC summaries
scoped to that source.

Detailed records do not repeat file names or device descriptors. A source
session stores first/last `source_epoch` and first/last `decoder_generation`;
readers map detailed records into those ranges. One input can span multiple
source epochs because a dropped-sample discontinuity advances the epoch.
Decoder generations can also advance independently when acquisition or
parameters reset without changing the input source.

FEC and emitted-TS totals are accumulated from `FecWindowTelemetry::delta` and
`output_bytes_delta`. They are not inferred from the final decoder snapshot,
because active FEC-session counters can reset within one source session.

## `stats.json`

The final summary is intentionally smaller than the telemetry stream. It
contains:

- `schema_version: 0`, status, program version, and optional build/revision
  identity;
- top-level receiver `mode`, run context, and source-session outcome counts;
- wall duration, decoded signal duration, submitted/processed samples, and
  average realtime speed;
- output bytes, actual emitted 188-byte packet count, emitted partial bytes,
  and usable packet count after TEI when that cumulative counter is available;
- cumulative dropped blocks, OFDM symbols, lock-weighted window counts, phase
  discontinuities, overlap joins/failures, FEC resets/sessions, Viterbi bit
  errors, RS packets/failures, and TEI packets;
- MER and SRO/CFO/timing count/min/mean/max over valid demod windows;
- for each timing key, sample count, total, mean, and maximum milliseconds;
- exit code and error.

It intentionally contains no source descriptor, decoder configuration,
per-source final state, or source array. Consumers that need those fields read
`source-sessions.jsonl`; this keeps the ordinary JSON summary bounded even for
an indefinitely running GUI.

`transport_bytes / 188` (plus a separately reported remainder) is the
authoritative count of packets actually delivered to the TS callback. The
current `TransportDecoderStats` values describe only the active FEC decoder
session and can reset on a gated region or parameter change. Report summaries
therefore aggregate typed FEC-window deltas. Inferring a reset only from a
decreasing polled counter is not exact and is not used.

Percentiles are deliberately omitted from the first in-process summary. The
online aggregator remains bounded by keeping count, total, minimum, and
maximum; p50/p95 can be calculated exactly from the relevant homogeneous JSONL
stream by regression tools without growing decoder memory during an indefinite
run.

## Implementation status

Implemented:

- stdin/stdout stream ownership, explicit stdin sample-rate validation, and a
  steady-clock one-second progress line;
- report-directory creation/refusal rules, immutable manifest, homogeneous
  JSONL routing, periodic flushing, and atomic running/final `stats.json`;
- opt-in frontend/demod/FEC typed telemetry with source and resampled sample
  spans, demod-window IDs, FEC sessions, and exact run-wide counters;
- bounded summary aggregation and one canonical timing representation shared
  by JSONL output and the human-readable `--debug` renderer;
- homogeneous structured decoder events shared by `events.jsonl`, summary
  event counts, and the `--debug` event renderer;
- GUI report lifecycle integration using the same telemetry queue, JSONL
  writers, periodic pipeline samples, and final summary as offline decoding;
- one run-wide `stats.json` plus homogeneous per-source summaries across GUI
  EOF, close, replay, sample-rate restart, and retune boundaries;
- headless schema coverage for two source sessions, epoch/generation ranges,
  non-empty-directory refusal, and exact FEC-delta aggregation;
- short real-capture validation of stdin/stdout, JSON validity, failure and
  no-transport statuses, and byte-identical TS output compared with file I/O.

Remaining validation and follow-up:

- run the complete 545 MHz capture and compare report-disabled/report-enabled
  TS output, dropped-block count, and throughput;
- validate GUI reports across I/Q end-of-file, explicit source close, replay,
  retune, and live receiver shutdown;
- add standalone schema-validation tooling once the version-0 fields settle.

Tests must cover a non-empty report-directory refusal, output/report write
failure, no-transport exit status, final partial input, and JSON validity for
unlocked/invalid measurements. Schema tests must also reject a mode-specific
record with a missing or unsupported top-level `mode` and must distinguish
`mode: "dvbt"` from DVB-T `transmission_mode: "2k"` or `"8k"`.
