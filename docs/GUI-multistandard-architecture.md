# GUI and Multi-Standard Receiver Architecture

DVB-T is implemented; DVB-T2 is the next planned decoder. The common receiver
and GUI interfaces support additional demodulators without moving source,
transport or playback responsibilities into each decoder.

## Ownership

- `SdrDevice` owns an `IqSource` backend: hardware/file handles, pacing and source
  workers. It delivers samples and source events through `SdrSourceCallbacks`.
- `ReceiverPipeline` owns the source, raw I/Q recorder, spectrum analyzer, sample
  timeline and active `Demodulator`. It submits samples and routes decoded TS
  and discontinuities into `TransportPipeline`.
- `TransportPipeline` owns service/EPG observers, TS recording and RTP output,
  and forwards data and discontinuities to an external sink such as playback.
- `ReceiverSession` owns the receiver and transport pipelines. It constructs and
  configures the selected decoder, and exposes common and typed snapshots to
  the GUI/CLI. Its private DVB-T pointer is non-owning; the receiver pipeline
  owns that decoder.
- `ReceiverController` coordinates source transitions with `DecodeRunReporter`.
  The GUI owns presentation and user input, not decoder lifetime.

## Lifecycle

A same-standard retune preserves the decoder instance and its callbacks.
After the source accepts the tuning change, the receiver resets spectrum and
sample continuity, resets the demodulator, and notifies the transport pipeline.
Frequency-correction changes follow the same reset path.

Standard replacement constructs the new decoder before changing the session,
then stops and joins the source, destroys the previous decoder, installs and
configures its replacement, and optionally restarts reception. Unsupported
standards leave the existing session untouched. If restarting fails after a
successful replacement, the source remains open but stopped with the new
configured decoder. No callback may outlive its decoder.

## GUI and telemetry

Common panels consume `SignalSnapshot` and `PipelineSnapshot` through the
`Demodulator` interface. Signal measurements include lock, constellation,
MER/SNR, carrier offset and channel-notch estimates. Pipeline snapshots publish
named stages with optional queue pressure, busy fraction and worker counts.
Common panels must not assume a particular OFDM/FEC pipeline shape.

Standard-specific parameters and diagnostics remain typed. `DvbTSessionSnapshot`
and the DVB-T panels expose TPS, timing/SRO, Viterbi and RS details. Detailed
report encoding also belongs to the standard, while source timelines and
transport-output reporting are shared.

## Adding DVB-T2

1. Implement `Demodulator`, including reset/flush semantics, transport and
   discontinuity callbacks, and common signal/pipeline snapshots.
2. Add typed DVB-T2 parameters, diagnostics and report encoding.
3. Add construction and configuration in `ReceiverSession`, keeping the same
   source shutdown and callback lifetime guarantees.
4. Enable the DVB-T2 selector entry and add its settings/diagnostics panels.

DVB-T2 and the other unimplemented selector entries remain disabled until their
processing paths are available. Shared source, spectrum, transport, recording,
EPG and playback code should not need standard-specific branches.
