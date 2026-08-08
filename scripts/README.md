# Validation runner

`run_validation.py` builds and tests the three correctness profiles, then runs
streaming synthetic DVB-T fixtures through the native decoder:

- `portable-release`: all 480 mode/bandwidth/guard/modulation/code-rate cases;
- `asan-ubsan`: the same 480 cases;
- `tsan`: 23 deterministic all-pairs cases including explicit boundary and
  high-load seeds.

Every fixture requires exact recovered MPEG-TS packet identity, zero TEI and
uncorrectable RS packets, consistent sample/packet accounting, final decoder
lock, and semantically valid version-0 JSON/JSONL decode reports.

## Machine-readable report

Each run creates `build/validation/<UTC timestamp>/` by default:

```text
manifest.json
summary.json
environment.json
stages.jsonl
ctest.jsonl
fixtures.jsonl
real-signal-corpus.json
real-signals.jsonl
junit/<profile>.xml
logs/<profile>-<stage>.log
failures/<profile>/<case>/
failures/real-signals/<profile>/<case>/
```

All validation schemas have version `0` while the project is pre-alpha.
`manifest.json` inventories the bounded JSON documents, homogeneous JSONL
streams, and artifact path conventions.

`summary.json` is atomically checkpointed after every stage and fixture. It
contains only bounded run/profile totals, current status, elapsed time, binary
SHA-256, selected CMake cache values, and stage outcomes; it does not grow by
one object per fixture.

`environment.json` captures the host and CPU, Python/venv and native Python
package versions, compiler/build-tool versions, Git revision/branch/dirty
paths, requested concurrency, skip policy, and exact sanitizer environment.

The JSONL files use one schema each:

- `stages.jsonl`: configure/build/CTest command, timestamps, status, return
  code, duration, working directory, and full-log path;
- `ctest.jsonl`: one parsed JUnit record per CTest case, including duration,
  pass/fail/skip state, properties, failure detail, and captured output;
- `fixtures.jsonl`: one record per completed matrix case, written immediately
  in completion order and carrying a stable `case_index`.
- `real-signals.jsonl`: one record per real recording, with execution status,
  decode outcome, regression status, complete bounded decode aggregates, and a
  stable corpus index. Synthetic exact-match rules do not apply to this stream.

`real-signal-corpus.json` inventories every recognized recording and ignored
regular file, sidecar pairing, fallback sample rate, file size, estimated
signal duration, and the long-recording selection policy. A `.json` sidecar
must identify a same-directory `.cs16` file with `ci16_le`, `IQ`, and a positive
sample rate. Remaining `.cs16` files use `--real-sample-rate`; `.ts`, `.log`,
and all other extensions are ignored.

A successful fixture record retains the validated decoder `stats.json`, source
session, final lock/tracking state, timing and measurement aggregates, event
and stream record counts, sample/TS/FEC counters, and exact cyclic TS match.
The large per-window decoder JSONL streams and TS files are removed after these
aggregates are captured. A failed fixture instead retains its complete expected
and recovered TS, generator/decoder logs, partial decoder report, structured
failure result, and runner log below `failures/`.

CTest's original JUnit XML and complete text logs are retained even though
their semantic contents are also represented in JSONL. Validate a finished or
interrupted report independently with:

```sh
python3 scripts/validate_validation_report.py \
  build/validation/<UTC timestamp>
```

The validator checks schema/record types, increasing stream sequences, bounded
summary counts against every stream status, and referenced log, JUnit, and
failure-artifact existence.

## Environment

The orchestration and report validators intentionally use only the Python
standard library. The fixture generator needs NumPy and GNU Radio's native
Python bindings. Those bindings are normally supplied by the operating system
and may be ABI-coupled to its Python and NumPy, so a pip-isolated venv is not a
portable replacement for them.

Create and enter the project validation venv automatically with:

```sh
python3 scripts/run_validation.py --bootstrap --smoke
```

The venv is created as `.venv-validation` with `--system-site-packages`; the
runner does not upgrade pip or replace the distro NumPy. It checks for Python
3.10+, NumPy, GNU Radio, Git, CMake, CTest, Ninja, and FFmpeg as required by the
selected stages. If future pure-Python orchestration dependencies are added,
pin them in a dedicated lock file and install them during an explicit bootstrap
step, not on every validation run. A container or a pinned conda-forge
environment is the appropriate next layer when the native GNU Radio, FFTW,
FFmpeg, or SDR stack must also be reproducible across distributions.

## Usage

Run the complete policy:

```sh
python3 scripts/run_validation.py --bootstrap
```

Run a quick one-case check of every profile:

```sh
python3 scripts/run_validation.py --bootstrap --smoke
```

Select profiles or inspect the deterministic case sets without running them:

```sh
python3 scripts/run_validation.py --profiles portable-release asan-ubsan
python3 scripts/run_validation.py --profiles tsan --list-cases
```

`--jobs N` sets build, CTest, and fixture-case concurrency together. Each can
be tuned independently with `--build-jobs`, `--ctest-jobs`, and
`--fixture-jobs`; the specific option overrides `--jobs`. The default is one
fixture at a time because each decoder instance has its own worker pool. Large
hosts can raise case concurrency, and `--decoder-threads N` can cap each
decoder while finding the best aggregate throughput. TSan CTest defaults to
one job unless a job option explicitly overrides it.

Build and CTest failures stop subsequent work but still finalize a valid
machine-readable report. Fixture failures are collected across the selected
matrix so one bad combination does not hide the others. Existing builds can be
reused with `--skip-configure --skip-build`; individual stages can also be
disabled with `--skip-ctest` or `--skip-fixtures`.

## Real-signal benchmarks

Inventory a corpus without building or decoding anything:

```sh
python3 scripts/run_validation.py \
  --real-signals-dir ~/scratchpad/DVB-T --list-real-signals
```

Run the routine corpus on the portable profile. DVB-T transmission mode,
guard, constellation, and code rate remain automatic and come from TPS;
channel bandwidth is explicit because TPS does not signal it:

```sh
python3 scripts/run_validation.py --bootstrap \
  --profiles portable-release --skip-fixtures \
  --real-signals-dir ~/scratchpad/DVB-T \
  --real-channel-bandwidth 6M
```

Bare recordings default to 10 MS/s; override that with
`--real-sample-rate HZ`. Real-signal concurrency defaults to one so concurrent
large reads do not distort storage and decoder timing, and can be changed with
`--real-jobs N` (or the global `--jobs N`). Files longer than 600 seconds are
inventoried but excluded by default. `--include-long-real-signals` includes
them; use it deliberately for the 2.5-hour regression recording.

The decoder writes TS to `/dev/null`, so a benchmark does not duplicate a large
transport stream. Successful and `no_transport` runs retain their validated
stats and final source session in `real-signals.jsonl`, then discard detailed
per-window reports. Execution/report failures retain all artifacts below
`failures/real-signals/`. `no_transport` is a valid measured outcome for a
known weak recording. Baseline thresholds are intentionally not enforced yet,
so every completed record initially has `regression_status: not_evaluated`.
