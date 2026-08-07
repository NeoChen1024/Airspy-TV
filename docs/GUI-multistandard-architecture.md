# GUI and Multi-Standard Receiver Architecture

## Scope

This review covers the application/GUI boundary needed to add another
television standard such as DVB-T2. It does not implement that DSP chain. The
goal is to make a future demodulator replaceable without exposing its lifetime
or standard-specific types to common GUI code.

## Result

The lower receiver layer was already mostly standard-neutral: `SdrDevice`
owns a `Demodulator`, routes MPEG-TS through the common transport model, and
feeds the active engine with I/Q blocks. The problematic coupling was in the
GUI, which retained a raw `dvbt::StreamDecoder *`, directly configured it from
several source controls, and polled DVB-T snapshots every frame.

The application now uses `ReceiverSession` as the lifecycle boundary:

- `ReceiverSession` owns `SdrDevice` and privately tracks the active concrete
  demodulator.
- GUI code never receives a concrete demodulator pointer.
- Source start/restart, I/Q file start, same-standard retune, frequency
  correction, demodulator configuration, and future standard replacement pass
  through the session.
- Common GUI panels consume `SignalSnapshot` and `PipelineSnapshot` through
  the `Demodulator` interface.
- DVB-T-only TPS, OFDM timing, and FEC detail remains in
  `DvbTSessionSnapshot` and the DVB-T diagnostics panel.

## Lifecycle Invariants

### Same-standard retune

A frequency retune preserves the active demodulator object. `SdrDevice` tunes
the hardware, resets its spectrum analyzer, resets the existing demodulator,
and clears the transport model. The decoder's existing discontinuity callback
therefore remains bound and emits `TransportDiscontinuity::retune`, preserving
the current mpv restart behavior.

Frequency-correction changes use the same in-place retune path.

### Standard replacement

The replacement transaction is:

1. Construct the replacement demodulator before changing the active session.
2. Stop the source and join its workers.
3. Disconnect and destroy the old demodulator.
4. Bind the stored discontinuity callback to the replacement.
5. Inject it into `SdrDevice`, apply its typed parameters, and publish the
   active channel bandwidth.
6. Restart the previously streaming source when requested.

If construction is unsupported or fails, the existing session is untouched.
If restart fails after a successful replacement, the source remains open but
stopped with a fully configured demodulator and rebound callbacks.

## Telemetry Boundary

`SignalSnapshot` contains only measurements that a common signal panel can
display across standards:

- constellation points;
- signal and transport lock;
- MER and SNR;
- carrier offset and its display scale;
- deepest channel notch;
- sequence number.

`PipelineSnapshot` publishes a bounded list of named stages. Each stage may
provide queue pressure, busy fraction, and worker count. The GUI and overload
monitor no longer assume that every demodulator has exactly an I/Q queue,
serial OFDM demodulator, and FEC queue.

Typed snapshots remain appropriate for information whose meaning is defined
by one standard. For DVB-T this includes transmission mode, guard interval,
TPS state, pilot timing/SRO, continual-carrier fade indication, Viterbi and RS
statistics, and detailed worker state.

## GUI Boundaries

The GUI state is divided into:

- common source, spectrum, signal, pipeline, transport, recorder, and playback
  state;
- a nested DVB-T state containing `ReceiverParameters`,
  `SignalAnalysisSnapshot`, and `StreamDecoderStats`.

The constellation and signal-quality panels use common snapshots. DVB-T FEC
and timing information is displayed in a separately labelled diagnostics
panel. The demodulator panel owns the standard selector and conditionally
shows DVB-T settings; future standard settings should be implemented as
separate typed sections rather than added to a universal parameter structure.

## Adding another standard

The remaining integration steps for a new standard are intentionally local:

1. Implement `Demodulator`, including common signal and pipeline snapshots.
2. Add a typed parameter/detail state for that standard.
3. Add construction and configuration in `ReceiverSession`.
4. Enable the standard selector entry.
5. Add a standard-specific settings panel and optional diagnostics panel.

The source, spectrum, MPEG-TS router/model, EPG, recorder, mpv player, common
constellation, common signal quality, and common pipeline-load UI should not
need standard-specific changes.

## Remaining Constraints

- `ReceiverSession` currently knows how to build only DVB-T; disabled selector
  entries are roadmap placeholders.
- The private DVB-T pointer inside `ReceiverSession` is a non-owning cache of
  the object owned by `SdrDevice`. Its creation, invalidation, and replacement
  are confined to the session transaction, so it cannot dangle in GUI state.
