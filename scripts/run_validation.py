#!/usr/bin/env python3

"""Build and exercise the portable and sanitizer validation profiles."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import importlib.util
from itertools import combinations, product
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any, Iterable

from real_signal_corpus import discover_real_signal_corpus
from real_signal_validation import run_real_signal_matrix
from validation_report import (
    CommandResult,
    ValidationReport,
    cmake_cache_values,
    parse_junit,
    sha256_file,
    utc_now,
    validate_report_directory,
)

MODES = ("2k", "8k")
BANDWIDTHS = ("5M", "6M", "7M", "8M")
GUARDS = ("1/32", "1/16", "1/8", "1/4")
MODULATIONS = ("qpsk", "qam16", "qam64")
CODE_RATES = ("1/2", "2/3", "3/4", "5/6", "7/8")
PROFILE_NAMES = ("portable-release", "asan-ubsan", "tsan")
TSAN_SEEDS = (
    ("2k", "8M", "1/32", "qam64", "7/8"),
    ("8k", "5M", "1/4", "qpsk", "1/2"),
    ("8k", "6M", "1/4", "qam64", "2/3"),
    ("2k", "6M", "1/32", "qam64", "2/3"),
)


@dataclass(frozen=True, order=True)
class FixtureCase:
    mode: str
    bandwidth: str
    guard: str
    modulation: str
    code_rate: str

    @property
    def slug(self) -> str:
        return "-".join(
            (
                self.mode,
                self.bandwidth.lower(),
                self.guard.replace("/", "_"),
                self.modulation,
                self.code_rate.replace("/", "_"),
            )
        )


@dataclass(frozen=True)
class ProfileSpec:
    name: str
    duration: float
    timeout: float
    cases: tuple[FixtureCase, ...]


def full_matrix() -> tuple[FixtureCase, ...]:
    return tuple(
        FixtureCase(*values)
        for values in product(MODES, BANDWIDTHS, GUARDS, MODULATIONS, CODE_RATES)
    )


def case_pairs(case: FixtureCase) -> set[tuple[int, str, int, str]]:
    values = (case.mode, case.bandwidth, case.guard, case.modulation, case.code_rate)
    return {
        (left, values[left], right, values[right])
        for left, right in combinations(range(len(values)), 2)
    }


def tsan_pairwise_cases() -> tuple[FixtureCase, ...]:
    candidates = full_matrix()
    uncovered: set[tuple[int, str, int, str]] = set()
    for case in candidates:
        uncovered.update(case_pairs(case))

    selected = [FixtureCase(*values) for values in TSAN_SEEDS]
    for case in selected:
        uncovered.difference_update(case_pairs(case))

    while uncovered:
        scores = {
            case: len(case_pairs(case) & uncovered)
            for case in candidates
            if case not in selected
        }
        best_score = max(scores.values())
        best = max(case for case, score in scores.items() if score == best_score)
        selected.append(best)
        uncovered.difference_update(case_pairs(best))
    return tuple(selected)


def positive_integer(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def bandwidth_hz(value: str) -> int:
    if value not in BANDWIDTHS:
        raise argparse.ArgumentTypeError("must be one of 5M, 6M, 7M, or 8M")
    return int(value[:-1]) * 1_000_000


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--profiles", nargs="+", choices=PROFILE_NAMES, default=PROFILE_NAMES
    )
    parser.add_argument(
        "--smoke", action="store_true", help="run one 1-second fixture per profile"
    )
    parser.add_argument("--list-cases", action="store_true")
    parser.add_argument(
        "--bootstrap",
        action="store_true",
        help="create and re-enter the validation venv",
    )
    parser.add_argument("--venv", type=Path, default=root / ".venv-validation")
    parser.add_argument("--results-dir", type=Path)
    parser.add_argument(
        "--jobs", type=positive_integer, help="set all parallel job counts"
    )
    parser.add_argument("--build-jobs", type=positive_integer)
    parser.add_argument("--ctest-jobs", type=positive_integer)
    parser.add_argument("--fixture-jobs", type=positive_integer)
    parser.add_argument("--real-jobs", type=positive_integer)
    parser.add_argument("--decoder-threads", type=int, default=0)
    parser.add_argument("--skip-configure", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--skip-ctest", action="store_true")
    parser.add_argument("--skip-fixtures", action="store_true")
    parser.add_argument("--real-signals-dir", type=Path)
    parser.add_argument("--real-sample-rate", type=positive_integer, default=10_000_000)
    parser.add_argument(
        "--real-channel-bandwidth", type=bandwidth_hz, default=6_000_000
    )
    parser.add_argument(
        "--real-profile", choices=PROFILE_NAMES, default="portable-release"
    )
    parser.add_argument("--real-long-threshold", type=positive_float, default=600.0)
    parser.add_argument("--include-long-real-signals", action="store_true")
    parser.add_argument("--list-real-signals", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.decoder_threads <= 256:
        parser.error("decoder threads must be between 0 and 256")
    if args.list_real_signals and args.real_signals_dir is None:
        parser.error("--list-real-signals requires --real-signals-dir")
    if (
        args.real_signals_dir is not None
        and not args.list_real_signals
        and args.real_profile not in args.profiles
    ):
        parser.error("--real-profile must also be selected by --profiles")
    return args


def bootstrap_venv(venv: Path) -> None:
    venv = venv.resolve()
    configuration = venv / "pyvenv.cfg"
    if not configuration.exists():
        print(f"creating validation venv: {venv}", flush=True)
        subprocess.run(
            [sys.executable, "-m", "venv", "--system-site-packages", str(venv)],
            check=True,
        )
    config_text = configuration.read_text(encoding="utf-8").lower()
    if "include-system-site-packages = true" not in config_text:
        raise RuntimeError(
            f"{venv} does not expose system site packages; recreate it with "
            "python3 -m venv --system-site-packages"
        )
    venv_python = venv / "bin" / "python"
    if Path(sys.prefix).resolve() != venv:
        os.execv(
            str(venv_python),
            [str(venv_python), str(Path(__file__).resolve()), *sys.argv[1:]],
        )


def preflight(args: argparse.Namespace) -> None:
    if sys.version_info < (3, 10):
        raise RuntimeError("validation requires Python 3.10 or newer")
    required_commands: list[str] = ["git"]
    if not args.skip_configure or not args.skip_build:
        required_commands.extend(("cmake", "ninja"))
    if not args.skip_ctest:
        required_commands.append("ctest")
    if not args.skip_fixtures:
        required_commands.append("ffmpeg")
    missing_commands = [
        command for command in required_commands if shutil.which(command) is None
    ]
    required_modules = ("numpy", "gnuradio") if not args.skip_fixtures else ()
    missing_modules = [
        module
        for module in required_modules
        if importlib.util.find_spec(module) is None
    ]
    if missing_commands or missing_modules:
        details = []
        if missing_commands:
            details.append("commands: " + ", ".join(missing_commands))
        if missing_modules:
            details.append("Python modules: " + ", ".join(missing_modules))
        raise RuntimeError(
            "validation prerequisites are missing (" + "; ".join(details) + ")"
        )
    for module in required_modules:
        __import__(module)


def profile_specs(smoke: bool) -> dict[str, ProfileSpec]:
    matrix = full_matrix()
    smoke_case = (FixtureCase("2k", "6M", "1/32", "qam64", "2/3"),)
    return {
        "portable-release": ProfileSpec(
            "portable-release",
            1.0 if smoke else 3.0,
            30.0,
            smoke_case if smoke else matrix,
        ),
        "asan-ubsan": ProfileSpec(
            "asan-ubsan", 1.0 if smoke else 3.0, 60.0, smoke_case if smoke else matrix
        ),
        "tsan": ProfileSpec(
            "tsan", 1.0, 120.0, smoke_case if smoke else tsan_pairwise_cases()
        ),
    }


def sanitizer_environment(profile: str) -> dict[str, str]:
    environment = os.environ.copy()
    if profile == "asan-ubsan":
        environment["ASAN_OPTIONS"] = (
            "detect_leaks=1:halt_on_error=1:strict_string_checks=1"
        )
        environment["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    elif profile == "tsan":
        environment["TSAN_OPTIONS"] = "halt_on_error=1:second_deadlock_stack=1"
    return environment


def run_stage(
    report: ValidationReport,
    root: Path,
    profile: str,
    stage: str,
    command: list[str],
) -> CommandResult:
    relative_log = Path("logs") / f"{profile}-{stage}.log"
    log_path = report.directory / relative_log
    print("+ " + " ".join(command), flush=True)
    started_at = utc_now()
    started = time.monotonic()
    return_code = 127
    with log_path.open("w", encoding="utf-8") as log:
        try:
            process = subprocess.Popen(
                command,
                cwd=root,
                env=sanitizer_environment(profile),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                errors="replace",
                bufsize=1,
            )
            if process.stdout is None:
                raise RuntimeError("unable to capture stage output")
            for line in process.stdout:
                log.write(line)
                print(line, end="", flush=True)
            return_code = process.wait()
        except OSError as exception:
            message = f"unable to start command: {exception}\n"
            log.write(message)
            print(message, end="", flush=True)
    elapsed = time.monotonic() - started
    finished_at = utc_now()
    report.write_stage(
        {
            "profile": profile,
            "stage": stage,
            "status": "passed" if return_code == 0 else "failed",
            "return_code": return_code,
            "started_at": started_at,
            "finished_at": finished_at,
            "elapsed_seconds": elapsed,
            "command": command,
            "working_directory": str(root),
            "log_path": str(relative_log),
        }
    )
    return CommandResult(
        return_code=return_code,
        elapsed_seconds=elapsed,
        started_at=started_at,
        finished_at=finished_at,
        log_path=relative_log,
    )


def load_result(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"fixture result is not an object: {path}")
    return value


def run_fixture(
    root: Path,
    report: ValidationReport,
    spec: ProfileSpec,
    case_index: int,
    case: FixtureCase,
    decoder_threads: int,
) -> dict[str, Any]:
    artifact_root = report.failures_dir / spec.name / case.slug
    artifact_root.mkdir(parents=True, exist_ok=False)
    result_json = artifact_root / "result.json"
    command = [
        sys.executable,
        str(root / "tools" / "validate_dvbt_fixture_stream.py"),
        "--build-dir",
        str(root / "build" / spec.name),
        "--work-dir",
        str(artifact_root),
        "--result-json",
        str(result_json),
        "--duration",
        str(spec.duration),
        "--timeout",
        str(spec.timeout),
        "--decoder-threads",
        str(decoder_threads),
        "--dvbt-mode",
        case.mode,
        "--dvbt-channel-bandwidth",
        case.bandwidth,
        "--dvbt-guard",
        case.guard,
        "--dvbt-modulation",
        case.modulation,
        "--dvbt-code-rate",
        case.code_rate,
    ]
    started_at = utc_now()
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=root,
            env=sanitizer_environment(spec.name),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        return_code = completed.returncode
        output = completed.stdout
    except OSError as exception:
        return_code = 127
        output = f"unable to start fixture validator: {exception}\n"
    elapsed = time.monotonic() - started
    if result_json.is_file():
        try:
            validation = load_result(result_json)
        except (OSError, json.JSONDecodeError, RuntimeError) as exception:
            validation = {
                "schema_version": 0,
                "record_type": "fixture_validation",
                "status": "failed",
                "error": {
                    "type": type(exception).__name__,
                    "message": str(exception),
                },
            }
            return_code = return_code or 1
    else:
        validation = {
            "schema_version": 0,
            "record_type": "fixture_validation",
            "status": "failed",
            "error": {
                "type": "MissingResult",
                "message": "fixture validator did not produce result.json",
            },
        }
        return_code = return_code or 1
    status = (
        "passed"
        if return_code == 0 and validation.get("status") == "passed"
        else "failed"
    )
    record: dict[str, Any] = {
        "profile": spec.name,
        "case_index": case_index,
        "case_id": case.slug,
        "case": asdict(case),
        "status": status,
        "return_code": return_code,
        "started_at": started_at,
        "finished_at": utc_now(),
        "elapsed_seconds": elapsed,
        "command": command,
        "validation": validation,
    }
    if status == "passed":
        shutil.rmtree(artifact_root)
    else:
        runner_log = artifact_root / "runner.log"
        runner_log.write_text(output, encoding="utf-8", errors="replace")
        record["artifacts"] = str(artifact_root.relative_to(report.directory))
        record["output_tail"] = output[-4000:]
    return record


def run_fixture_matrix(
    root: Path,
    report: ValidationReport,
    summary: dict[str, Any],
    spec: ProfileSpec,
    fixture_jobs: int,
    decoder_threads: int,
) -> int:
    total = len(spec.cases)
    failures = 0
    print(
        f"[{spec.name}] {total} fixtures, duration={spec.duration:g}s, "
        f"parallel={fixture_jobs}, timeout={spec.timeout:g}s",
        flush=True,
    )
    with ThreadPoolExecutor(max_workers=fixture_jobs) as executor:
        futures = {
            executor.submit(
                run_fixture,
                root,
                report,
                spec,
                case_index,
                case,
                decoder_threads,
            ): (case_index, case)
            for case_index, case in enumerate(spec.cases)
        }
        for completed_count, future in enumerate(as_completed(futures), start=1):
            case_index, case = futures[future]
            try:
                record = future.result()
            except Exception as exception:
                artifact_root = report.failures_dir / spec.name / case.slug
                artifact_root.mkdir(parents=True, exist_ok=True)
                (artifact_root / "runner.log").write_text(
                    f"fixture worker failed: {type(exception).__name__}: {exception}\n",
                    encoding="utf-8",
                )
                record = {
                    "profile": spec.name,
                    "case_index": case_index,
                    "case_id": case.slug,
                    "case": asdict(case),
                    "status": "failed",
                    "return_code": 1,
                    "error": {
                        "type": type(exception).__name__,
                        "message": str(exception),
                    },
                    "artifacts": str(artifact_root.relative_to(report.directory)),
                }
            report.write_fixture(record)
            status = record["status"]
            failures += status == "failed"
            update_fixture_summary(summary, spec.name, status)
            report.write_summary(summary)
            marker = "PASS" if status == "passed" else "FAIL"
            print(
                f"[{spec.name}] {completed_count:3}/{total} {marker} "
                f"{case.slug} ({record.get('elapsed_seconds', 0.0):.2f}s)",
                flush=True,
            )
    return failures


def initial_summary(
    profiles: list[str],
    specs: dict[str, ProfileSpec],
    configuration: dict[str, Any],
    real_signal_count: int,
) -> dict[str, Any]:
    planned_fixtures = (
        0
        if configuration["skip_fixtures"]
        else sum(len(specs[profile].cases) for profile in profiles)
    )
    return {
        "schema_version": 0,
        "record_type": "validation_summary",
        "status": "running",
        "started_at": utc_now(),
        "finished_at": None,
        "elapsed_seconds": None,
        "configuration": configuration,
        "totals": {
            "stages": {"total": 0, "passed": 0, "failed": 0},
            "ctest": {"total": 0, "passed": 0, "failed": 0, "skipped": 0},
            "fixtures": {
                "planned": planned_fixtures,
                "completed": 0,
                "passed": 0,
                "failed": 0,
            },
            "real_signals": {
                "planned": real_signal_count,
                "completed": 0,
                "failed": 0,
                "decode_completed": 0,
                "no_transport": 0,
                "regressed": 0,
                "not_evaluated": 0,
            },
        },
        "profiles": {
            profile: {
                "status": "pending",
                "stages": {},
                "ctest": {"total": 0, "passed": 0, "failed": 0, "skipped": 0},
                "fixtures": {
                    "planned": (
                        0
                        if configuration["skip_fixtures"]
                        else len(specs[profile].cases)
                    ),
                    "completed": 0,
                    "passed": 0,
                    "failed": 0,
                    "duration_seconds": specs[profile].duration,
                    "timeout_seconds": specs[profile].timeout,
                },
                "real_signals": {
                    "planned": (
                        real_signal_count
                        if profile == configuration["real_profile"]
                        else 0
                    ),
                    "completed": 0,
                    "failed": 0,
                    "decode_completed": 0,
                    "no_transport": 0,
                    "regressed": 0,
                    "not_evaluated": 0,
                },
                "binary": None,
            }
            for profile in profiles
        },
        "error": None,
    }


def update_stage_summary(
    summary: dict[str, Any], profile: str, stage: str, result: CommandResult
) -> None:
    status = "passed" if result.return_code == 0 else "failed"
    summary["profiles"][profile]["stages"][stage] = {
        "status": status,
        "return_code": result.return_code,
        "elapsed_seconds": result.elapsed_seconds,
        "log_path": str(result.log_path),
    }
    stages = summary["totals"]["stages"]
    stages["total"] += 1
    stages[status] += 1


def update_ctest_summary(
    summary: dict[str, Any], profile: str, records: list[dict[str, Any]]
) -> None:
    profile_counts = summary["profiles"][profile]["ctest"]
    total_counts = summary["totals"]["ctest"]
    for record in records:
        status = record["status"]
        profile_counts["total"] += 1
        profile_counts[status] += 1
        total_counts["total"] += 1
        total_counts[status] += 1


def update_fixture_summary(summary: dict[str, Any], profile: str, status: str) -> None:
    profile_counts = summary["profiles"][profile]["fixtures"]
    total_counts = summary["totals"]["fixtures"]
    profile_counts["completed"] += 1
    profile_counts[status] += 1
    total_counts["completed"] += 1
    total_counts[status] += 1


def record_binary(summary: dict[str, Any], root: Path, profile: str) -> None:
    binary = root / "build" / profile / "airspy-tv"
    summary["profiles"][profile]["binary"] = {
        "path": str(binary.relative_to(root)),
        "size_bytes": binary.stat().st_size if binary.is_file() else None,
        "sha256": sha256_file(binary),
    }
    summary["profiles"][profile]["cmake_cache"] = cmake_cache_values(
        root / "build" / profile / "CMakeCache.txt"
    )


def run_required_stage(
    report: ValidationReport,
    summary: dict[str, Any],
    root: Path,
    profile: str,
    stage: str,
    command: list[str],
) -> CommandResult:
    result = run_stage(report, root, profile, stage, command)
    update_stage_summary(summary, profile, stage, result)
    report.write_summary(summary)
    return result


def print_cases(profiles: Iterable[str], specs: dict[str, ProfileSpec]) -> None:
    for profile in profiles:
        print(f"{profile}: {len(specs[profile].cases)} cases")
        for case in specs[profile].cases:
            print(f"  {case.slug}")


def empty_real_signal_inventory(configuration: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": 0,
        "record_type": "real_signal_corpus",
        "root": None,
        "configuration": {
            "fallback_sample_rate_hz": configuration["real_sample_rate_hz"],
            "channel_bandwidth_hz": configuration["real_channel_bandwidth_hz"],
            "include_long_recordings": configuration["include_long_real_signals"],
            "long_recording_threshold_seconds": configuration[
                "real_long_threshold_seconds"
            ],
            "recognized_iq_extension": ".cs16",
            "recognized_metadata_extension": ".json",
            "ignored_file_policy": "ignore every other regular file",
        },
        "totals": {
            "recordings": 0,
            "selected": 0,
            "excluded_long": 0,
            "file_size_bytes": 0,
            "signal_seconds": 0.0,
            "selected_signal_seconds": 0.0,
            "ignored_files": 0,
        },
        "recordings": [],
        "ignored_files": [],
    }


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    if args.bootstrap:
        bootstrap_venv(args.venv)

    specs = profile_specs(args.smoke)
    if args.list_cases:
        print_cases(args.profiles, specs)
        return 0
    real_corpus = (
        discover_real_signal_corpus(
            args.real_signals_dir,
            fallback_sample_rate_hz=args.real_sample_rate,
            channel_bandwidth_hz=args.real_channel_bandwidth,
            include_long_recordings=args.include_long_real_signals,
            long_recording_threshold_seconds=args.real_long_threshold,
        )
        if args.real_signals_dir is not None
        else None
    )
    if args.list_real_signals:
        if real_corpus is None:
            raise RuntimeError("real-signal corpus was not discovered")
        print(
            json.dumps(real_corpus.to_json(), indent=2, sort_keys=True, allow_nan=False)
        )
        return 0
    preflight(args)

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    results_dir = (
        args.results_dir or root / "build" / "validation" / timestamp
    ).resolve()
    if results_dir.exists() and any(results_dir.iterdir()):
        raise RuntimeError(f"results directory is not empty: {results_dir}")
    results_dir.mkdir(parents=True, exist_ok=True)

    cpu_count = os.cpu_count() or 1
    build_jobs = args.build_jobs or args.jobs
    default_ctest_jobs = min(cpu_count, 8)
    ctest_jobs = args.ctest_jobs or args.jobs
    fixture_jobs = args.fixture_jobs or args.jobs or 1
    real_jobs = args.real_jobs or args.jobs or 1
    configuration = {
        "profiles": args.profiles,
        "smoke": args.smoke,
        "build_jobs": build_jobs,
        "ctest_jobs": ctest_jobs,
        "fixture_jobs": fixture_jobs,
        "real_jobs": real_jobs,
        "decoder_threads": args.decoder_threads,
        "skip_configure": args.skip_configure,
        "skip_build": args.skip_build,
        "skip_ctest": args.skip_ctest,
        "skip_fixtures": args.skip_fixtures,
        "real_signals_dir": (
            str(real_corpus.root) if real_corpus is not None else None
        ),
        "real_sample_rate_hz": args.real_sample_rate,
        "real_channel_bandwidth_hz": args.real_channel_bandwidth,
        "real_profile": args.real_profile,
        "real_long_threshold_seconds": args.real_long_threshold,
        "include_long_real_signals": args.include_long_real_signals,
        "runtime_environment": {
            profile: {
                key: value
                for key, value in sanitizer_environment(profile).items()
                if key in {"ASAN_OPTIONS", "UBSAN_OPTIONS", "TSAN_OPTIONS"}
            }
            for profile in args.profiles
        },
    }
    real_signal_count = (
        len(real_corpus.selected_cases) if real_corpus is not None else 0
    )
    summary = initial_summary(args.profiles, specs, configuration, real_signal_count)
    report = ValidationReport(results_dir, root, args.profiles, configuration)
    report.write_real_signal_corpus(
        real_corpus.to_json()
        if real_corpus is not None
        else empty_real_signal_inventory(configuration)
    )
    report.write_summary(summary)

    failed = False
    interrupted = False
    caught: Exception | None = None
    started = time.monotonic()
    try:
        for profile in args.profiles:
            profile_summary = summary["profiles"][profile]
            profile_summary["status"] = "running"
            report.write_summary(summary)
            if not args.skip_configure:
                result = run_required_stage(
                    report,
                    summary,
                    root,
                    profile,
                    "configure",
                    ["cmake", "--preset", profile],
                )
                if result.return_code != 0:
                    raise RuntimeError(f"{profile} configure failed")
            if not args.skip_build:
                command = ["cmake", "--build", "--preset", profile]
                if build_jobs is not None:
                    command.extend(("--parallel", str(build_jobs)))
                result = run_required_stage(
                    report, summary, root, profile, "build", command
                )
                if result.return_code != 0:
                    raise RuntimeError(f"{profile} build failed")
            record_binary(summary, root, profile)
            report.write_summary(summary)
            if not args.skip_ctest:
                selected_jobs = ctest_jobs or (
                    1 if profile == "tsan" else default_ctest_jobs
                )
                relative_junit = Path("junit") / f"{profile}.xml"
                junit_path = results_dir / relative_junit
                command = [
                    "ctest",
                    "--preset",
                    profile,
                    "--parallel",
                    str(selected_jobs),
                    "--output-junit",
                    str(junit_path),
                ]
                result = run_required_stage(
                    report, summary, root, profile, "ctest", command
                )
                if junit_path.is_file():
                    ctest_records = parse_junit(junit_path, profile)
                    for record in ctest_records:
                        record["junit_path"] = str(relative_junit)
                        report.write_ctest(record)
                    update_ctest_summary(summary, profile, ctest_records)
                    report.write_summary(summary)
                if result.return_code != 0:
                    raise RuntimeError(f"{profile} CTest failed")
                if not junit_path.is_file():
                    raise RuntimeError(f"{profile} CTest produced no JUnit report")
            if not args.skip_fixtures:
                fixture_failures = run_fixture_matrix(
                    root,
                    report,
                    summary,
                    specs[profile],
                    fixture_jobs,
                    args.decoder_threads,
                )
                failed = failed or fixture_failures != 0
            if real_corpus is not None and profile == args.real_profile:
                real_failures = run_real_signal_matrix(
                    root,
                    report,
                    summary,
                    real_corpus,
                    profile,
                    real_jobs,
                    args.decoder_threads,
                    sanitizer_environment(profile),
                )
                failed = failed or real_failures != 0
            profile_failed = (
                profile_summary["ctest"]["failed"] != 0
                or profile_summary["fixtures"]["failed"] != 0
                or profile_summary["real_signals"]["failed"] != 0
                or profile_summary["real_signals"]["regressed"] != 0
            )
            profile_summary["status"] = "failed" if profile_failed else "completed"
            report.write_summary(summary)
    except KeyboardInterrupt:
        interrupted = True
        failed = True
    except Exception as exception:
        caught = exception
        failed = True
        summary["error"] = {
            "type": type(exception).__name__,
            "message": str(exception),
        }
    finally:
        for profile_summary in summary["profiles"].values():
            if profile_summary["status"] == "running":
                profile_summary["status"] = "interrupted" if interrupted else "failed"
        summary["status"] = (
            "interrupted" if interrupted else "failed" if failed else "completed"
        )
        summary["elapsed_seconds"] = time.monotonic() - started
        summary["finished_at"] = utc_now()
        report.write_summary(summary)
        report.close()

    report_counts = validate_report_directory(results_dir)
    print(
        f"validation {summary['status']}: {results_dir / 'summary.json'} "
        f"({report_counts['stages']} stages, {report_counts['ctest']} CTest cases, "
        f"{report_counts['fixtures']} fixtures, "
        f"{report_counts['real_signals']} real signals)"
    )
    if caught is not None:
        raise caught
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
