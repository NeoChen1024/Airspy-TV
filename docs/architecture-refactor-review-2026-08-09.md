# Architecture Refactor Review

Date: 2026-08-09

## Result

The ownership findings from the original review are resolved. The refactor
keeps the established DVB-T DSP behavior while separating source control,
receiver processing, transport fanout, report serialization, and GUI
orchestration.

No generic mode-plugin framework or shared transport-block allocator was
introduced. DVB-T state remains explicitly typed, and each transport consumer
owns its copied bytes and overload policy.

## Runtime ownership

```text
ReceiverSession
  +-- TransportPipeline
  |     +-- service-model observer (256 KiB, drop oldest)
  |     +-- EPG observer (256 KiB, drop oldest)
  |     +-- TS recorder (24 MiB, drop oldest)
  |     +-- RTP/UDP sender (8 MiB, drop oldest)
  |     `-- external queued sink adapter (mpv or CLI output)
  `-- ReceiverPipeline
        +-- SdrDevice
        |     `-- Airspy / Soapy / file / stdin source only
        +-- InputSampleTimeline
        +-- SpectrumAnalyzer
        +-- RawIqRecorder
        `-- active Demodulator
```

`SdrDevice` owns only source selection, backend configuration, tuning, gain,
bias, pacing, and source callbacks. It does not own a demodulator, analyzer,
timeline, recorder, transport parser, or TS output.

`ReceiverPipeline` stamps source blocks, runs optional display analysis,
submits to the demodulator with the selected backpressure policy, and forwards
typed transport data/discontinuities to `TransportPipeline`.

`TransportPipeline` keeps metadata parsing off the FEC thread through two
independent asynchronous observer queues. Recorder, RTP, mpv, and CLI outputs
cannot consume each other's queue budget. Retune clears queued live output
where old bytes must not cross the source boundary; other discontinuities
preserve valid queued data and reset only incomplete parser assembly.

Offline exact output is deliberately outside the lossy live policy: it uses a
24 MiB bounded blocking queue and treats any drop as an internal error.

## Transport policies

| Consumer | Capacity | Overflow policy | Completion role |
| --- | ---: | --- | --- |
| mpv playback | 8 MiB | drop oldest at hard capacity | optional live sink |
| RTP/UDP | 8 MiB | drop oldest | optional live sink |
| live CLI TS file/stdout | 8 MiB | drop oldest and warn | required destination, lossy live policy |
| TS recorder | 24 MiB | drop oldest | optional live sink |
| service model | 256 KiB | drop oldest plus local parser reset | latest-state observer |
| EPG model | 256 KiB | drop oldest plus local parser reset | latest-state observer |
| offline exact TS file/stdout | 24 MiB | block producer | required exact sink |

mpv retains its existing 1 MiB low and 2 MiB resume watermarks. No transport
queue uses shared blocks; a machine capable of running the decoder can afford
the bounded copied queues, and independent ownership keeps failure and lifetime
rules obvious.

## Reporting boundaries

Report code now has three explicit layers:

- `DecodeReport` owns the manifest, source-session lifecycle, aggregation,
  atomic `stats.json`, and final status;
- `TelemetryStreamRouter` owns homogeneous JSONL files, batching, and flush;
- `dvbt_report_codec` owns DVB-T record field mapping, enum names, and the
  human-readable `--debug` renderer.

`transport-outputs.jsonl` samples active sink queues and counters once per wall
second. `source-sessions.jsonl` and `stats.json` contain per-sink counter deltas,
drop/error totals, capacity, and maximum observed queue use. Schema versions
remain `0`.

## GUI boundaries

GUI state is grouped into source, display, standard, output, reporting, and UI
shell objects. `AppFrameSnapshot` captures receiver, DVB-T, transport, recorder,
RTP, service/EPG, and mpv telemetry once before drawing a frame. Panels read
that immutable frame copy instead of repeatedly polling runtime objects.

`ReceiverController` owns multi-step open, replay, retune, close, report, raw
recording, TS recording, and RTP commands. It depends on neither SDL, ImGui,
nor mpv. Panels retain only presentation, direct scalar setting changes, and
command invocation.

## Preserved invariants

- Generation cancellation, sample-counter scheduling, symbol/FEC order, and TS
  cadence are unchanged.
- SRO/CFO loops, resampler filter policy, and demod/FEC worker allocation are
  outside this refactor.
- Same-standard retune preserves the demodulator instance.
- `stream_end` delivers final TS bytes before its discontinuity callback.
- Report records retain `mode: "dvbt"`, schema version `0`, source epoch, and
  decoder-generation correlation.
- Existing mpv PID filtering and RTP packetization remain at their previous
  owners.

## Validation status

The 31-test CTest suite passes in optimized, assertion Debug, portable Release,
ASan/UBSan, and TSan builds. The post-refactor fixture run completed 480/480
portable Release cases, 480/480 ASan/UBSan cases, and the 23/23 deterministic
TSan set. All 983 cases recovered exact transport, passed their decode-report
validation, and produced a machine-readable validation summary with zero
failures. A separate short 2K smoke recovered 2,972 exact TS packets with zero
TEI.

Retained real-signal and manual GUI/mpv runs were not performed for this
structural change and remain release gates. They are validation work, not
unresolved architecture findings.
