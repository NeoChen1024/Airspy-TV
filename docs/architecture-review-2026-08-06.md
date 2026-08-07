#Remaining Architecture and Soundness Work

Date : 2026 - 08 -
    06 Maintained against
    : current `dev` worktree

      ##Status

          This maintained copy lists only unresolved
              work.Findings corrected after the original review have been
                  removed rather than retained as an implementation
                      history.The remaining work is regression coverage,
    portability,
    and file - level cleanup rather than a known unsafe data -
        flow design.

        ##Remaining findings

        ## #High
    : regression coverage is still narrower than the supported behavior

          The current tests cover the main 8K stream path,
    reset / reopen and rapid - retune shapes, inner / outer FEC,
    pipeline - load telemetry, EPG,
    and recorder byte -
        rate tracking.They do not yet cover several important contracts :

    -2K end -
        to - end decoding;
- every guard - interval and automatic / manual mode transition;
- continuous -
    resampler equivalence over random block boundaries and rate changes;
- repeated concurrent submit / reset / flush / stop stress;
- analyzer reset / submit stress under ThreadSanitizer;
- portable non - AVX Viterbi as a regular CI configuration;
- mpv queue hysteresis and discontinuity ordering with a fake reader;
- recorder short - write, filesystem - error, and shutdown behavior;
- PAT / PMT / SDT version changes and malformed - section handling;
- long synthetic SRO /
        CFO fixtures as retained opt -
    in regressions.

    Priority should go to lifecycle stress,
    2K coverage,
    and fake - reader playback tests because these protect the broadest behavior
                   during further refactoring.

               ## #
     Medium:full runtime and sanitizer validation remains incomplete

                The existing optimized,
            assertion - enabled, portable, and targeted TSAN runs are useful,
            but the following validation is still needed after substantial
                    pipeline or
                GUI
    changes:

        -a complete ASan / UBSan / LSan run;
- longer repeated TSAN lifecycle stress rather than only deterministic cases;
- a fresh full 363.9 GB 545 MHz replay after major DSP changes;
- 557 / 581 MHz multipath replay;
- live Airspy and SoapySDR start / stop / retune testing;
- interactive GUI and mpv playback/recording testing.

Long-capture timing requirements remain in
[clock-tracking.md](clock-tracking.md) rather than in this architecture review.

### Medium: build defaults are optimized for this workstation, not portability

The following policies remain deliberate but should be isolated or exercised
in CI:

- default Debug uses `-O3 -DNDEBUG` unless `AIRSPY_TV_OPTIMIZED_DEBUG=OFF`;
- `AIRSPY_TV_NATIVE_ARCH = ON` adds `- march = native`;
- non -
    AVX builds select a different ViterbiDecoderCpp backend and need regular
        coverage.

    Recommended direction:

    1. define explicit optimized - debug, assertion - debug, portable - release,
    and sanitizer presets;
2. make host-native optimization an explicit release/profile choice for
   distributable builds;
3. keep SIMD and scalar Viterbi paths buildable until portable coverage is
        routine.

    ## #GUI path is split into mode -
    aware translation units

            The former `src /
        main_gui.hpp` include -
    fragment and its anonymous -
    namespace dependencies have been removed.The desktop path now uses ordinary
            translation units in `src /
        gui /`,
    with the following ownership and dependency direction :

```text main.cpp->gui / app.hpp->gui / app.cpp->gui / app_state.hpp->gui /
        widgets.cpp->gui / spectrum_panels.cpp->gui / source_panels.cpp->gui /
        receiver_panels.cpp->gui / dvbt_panels.cpp->gui /
        media_panels.cpp
```

`gui / app.hpp` exposes only the desktop entry point. `gui / app.cpp` owns SDL,
    OpenGL, ImGui initialization and shutdown, frame polling, snapshot refresh,
    and top - level composition.Shared widgets, theme / font loading,
    frequency input,
    and file - dialog plumbing belong in `gui / widgets.cpp`.Spectrum /
                   waterfall / constellation,
    source controls, standard - neutral receiver status,
    DVB - T settings / diagnostics,
    and transport / media panels are separate compilation units. `panels
                        .hpp` is the small internal declaration surface; no panel implementation is included into another source file.

The behavior-preserving move retains one internal `AppState`. Its GUI-only
members may later be grouped into source, spectrum, recorder, transport, and
typed per-standard sub-states. Do not add a virtual panel framework: free draw
functions and explicit standard dispatch are sufficient. Common GUI code
should call standard dispatch points for settings, state refresh, and
diagnostics;
DVB - T types must remain confined to the DVB -
    T state and panel implementation
        .

`decoder_diagnostics
        .hpp` is now a declaration header backed by
`decoder_diagnostics.cpp`;
it no longer receives declarations from
`main.cpp`'s anonymous namespace. `ReceiverSession` remains the only owner of source
            start /
        restart /
        retune and demodulator replacement.

        The split preserves these GUI -
    specific lifecycle constraints :

    -SDL file -
    dialog callbacks retain shared state until asynchronous completion;
- all ImGui and OpenGL calls remain on the GUI thread;
- the waterfall texture is destroyed while its OpenGL context is still live;
- transport sinks are disconnected and source /
    player workers are stopped before GUI backend teardown;
- frame refresh and rendering are separate functions,
    so state polling and debug telemetry do not become implicit panel side
        effects.

    ## #Low:long -
                lived diagnostics need explicit retention policy

                    User -
                visible and
            fatal errors remain unconditional; decoder diagnostics, event
logs, and FEC traces now share the `--debug` flag. These verbose logs can still
produce very large files during clock research. The proposed report directory,
JSONL schema, summary statistics, and debug-log transition are specified in
[machine-readable-performance-report.md](machine-readable-performance-report.md).
Implement that design and rate-limit repeated human-readable events where
appropriate.

This should reduce ad-hoc log parsing without removing the detailed data needed
for AFC and FEC investigations.

## Recommended order

1. Add 2K and lifecycle/playback regression coverage.
2. Add build presets and routine portable/sanitizer configurations.
3. Perform full-capture and live-hardware validation after major DSP or source
   changes.

## Refactoring acceptance criteria

Future cleanup should preserve:

- bounded live queues and blocking offline backpressure;
- persistent worker pools and ordered joins before stateful consumers;
- one owner for demodulator lifetime and same-instance retune;
- serialized outer - FEC and transport output;
- generation - safe reset and stale - output suppression;
- production - path signal telemetry with an optional pre - lock monitor;
- standard - neutral common GUI state and typed per - standard diagnostics;
- no GUI implementation headers included into `main.cpp` or another source;
- SDL / OpenGL / ImGui teardown ordering and GUI - thread - only rendering;
- current throughput,
    TS continuity, queue latency, and clock - tracking baselines.
