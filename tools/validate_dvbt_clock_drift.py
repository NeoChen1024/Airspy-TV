#!/usr/bin/env python3

"""Run native DVB-T decoding against known sample-clock and LO drift."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import statistics
import subprocess
import tempfile


@dataclass(frozen=True)
class Scenario:
    name: str
    sample_clock_ppm: float
    sample_clock_drift_ppm_per_minute: float
    lo_offset_hz: float
    lo_drift_hz_per_minute: float


SCENARIOS = {
    scenario.name: scenario
    for scenario in (
        Scenario("sample-clock", 0.18, 0.03, 0.0, 0.0),
        Scenario("lo", 0.0, 0.0, 100.0, 30.0),
        Scenario("independent", -0.12, 0.04, -120.0, 45.0),
    )
}

NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
INPUT_RE = re.compile(r"\binput=(\d+) samples\b")
TS_RE = re.compile(r"\bTS=(\d+) bytes\b")
TRACKED_RE = re.compile(rf"\btracked=({NUMBER}) Hz\b")
SRO_RE = re.compile(rf"\bsro=({NUMBER}) ppm\b")
READY_RE = re.compile(r"\bsro-ready=([01])\b")
FINAL_TS_RE = re.compile(r"\bTS=(\d+) bytes, symbols=")


@dataclass(frozen=True)
class ClockSample:
    time_seconds: float
    sro_ppm: float
    tracked_cfo_hz: float


def run_checked(
    command: list[str], env: dict[str, str]
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"command failed ({result.returncode}): {' '.join(command)}\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return result


def parse_decoder_log(text: str, sample_rate: int) -> tuple[list[ClockSample], int]:
    samples: list[ClockSample] = []
    final_ts_bytes = 0
    for line in text.splitlines():
        if line.startswith("chunk="):
            input_match = INPUT_RE.search(line)
            ts_match = TS_RE.search(line)
            tracked_match = TRACKED_RE.search(line)
            sro_match = SRO_RE.search(line)
            ready_match = READY_RE.search(line)
            if ts_match:
                final_ts_bytes = max(final_ts_bytes, int(ts_match.group(1)))
            if not all((input_match, tracked_match, sro_match, ready_match)):
                continue
            if ready_match.group(1) != "1":
                continue
            samples.append(
                ClockSample(
                    time_seconds=int(input_match.group(1)) / sample_rate,
                    sro_ppm=float(sro_match.group(1)),
                    tracked_cfo_hz=float(tracked_match.group(1)),
                )
            )
        elif line.startswith("decoded "):
            final_match = FINAL_TS_RE.search(line)
            if final_match:
                final_ts_bytes = max(final_ts_bytes, int(final_match.group(1)))
    return samples, final_ts_bytes


def expected_at(
    initial: float, rate_per_minute: float, time_seconds: float
) -> float:
    return initial + rate_per_minute * time_seconds / 60.0


def validate_scenario(
    root: Path,
    build_dir: Path,
    work_dir: Path,
    scenario: Scenario,
    duration: float,
    decoder_threads: int,
) -> None:
    fixture = work_dir / f"dvbt-clock-{scenario.name}.cs16"
    transport = work_dir / f"dvbt-clock-{scenario.name}.ts"
    log_path = work_dir / f"dvbt-clock-{scenario.name}.log"
    env = os.environ.copy()
    env["XDG_CACHE_HOME"] = str(work_dir / "cache")
    env["AIRSPYTV_EVENT_DEBUG"] = "1"

    generator = run_checked(
        [
            "python3",
            str(root / "tools/generate_dvbt_fixture.py"),
            str(fixture),
            "--duration",
            str(duration),
            "--sample-clock-ppm",
            str(scenario.sample_clock_ppm),
            "--sample-clock-drift-ppm-per-minute",
            str(scenario.sample_clock_drift_ppm_per_minute),
            "--lo-offset-hz",
            str(scenario.lo_offset_hz),
            "--lo-drift-hz-per-minute",
            str(scenario.lo_drift_hz_per_minute),
        ],
        env,
    )
    decoder = run_checked(
        [
            str(build_dir / "airspy-tv"),
            "--decode-iq",
            str(fixture) + ".json",
            "--ts-output",
            str(transport),
            "--decoder-threads",
            str(decoder_threads),
            "--debug",
        ],
        env,
    )
    decoder_log = decoder.stdout + decoder.stderr
    log_path.write_text(generator.stdout + generator.stderr + decoder_log)
    metadata = json.loads(Path(str(fixture) + ".json").read_text())
    clock_samples, ts_bytes = parse_decoder_log(
        decoder_log, int(metadata["sample_rate"])
    )
    if len(clock_samples) < 4:
        raise RuntimeError(
            f"{scenario.name}: only {len(clock_samples)} SRO-ready samples; "
            "increase --duration"
        )
    if ts_bytes == 0 or transport.stat().st_size == 0:
        raise RuntimeError(f"{scenario.name}: decoder produced no transport stream")
    failure_markers = (
        "outer-reset",
        "align-miss",
        "badlock",
        "timing-branch",
        "decoder thread exception",
    )
    found = [marker for marker in failure_markers if marker in decoder_log]
    if found:
        raise RuntimeError(f"{scenario.name}: failure events: {', '.join(found)}")

    tail_count = min(12, len(clock_samples))
    tail = clock_samples[-tail_count:]
    measured_sro = statistics.median(sample.sro_ppm for sample in tail)
    expected_sro = statistics.median(
        expected_at(
            scenario.sample_clock_ppm,
            scenario.sample_clock_drift_ppm_per_minute,
            sample.time_seconds,
        )
        for sample in tail
    )
    measured_cfo = statistics.median(sample.tracked_cfo_hz for sample in tail)
    expected_cfo = statistics.median(
        expected_at(
            scenario.lo_offset_hz,
            scenario.lo_drift_hz_per_minute,
            sample.time_seconds,
        )
        for sample in tail
    )
    sro_tolerance = max(0.05, abs(expected_sro) * 0.35)
    cfo_tolerance = 8.0
    if not math.isclose(measured_sro, expected_sro, abs_tol=sro_tolerance):
        raise RuntimeError(
            f"{scenario.name}: SRO {measured_sro:+.4f} ppm, expected "
            f"{expected_sro:+.4f} +/- {sro_tolerance:.4f} ppm"
        )
    if not math.isclose(measured_cfo, expected_cfo, abs_tol=cfo_tolerance):
        raise RuntimeError(
            f"{scenario.name}: CFO {measured_cfo:+.2f} Hz, expected "
            f"{expected_cfo:+.2f} +/- {cfo_tolerance:.2f} Hz"
        )

    print(
        f"PASS {scenario.name}: SRO={measured_sro:+.4f} ppm "
        f"(expected {expected_sro:+.4f}), CFO={measured_cfo:+.2f} Hz "
        f"(expected {expected_cfo:+.2f}), TS={ts_bytes} bytes"
    )
    if generator.stderr:
        print(f"  generator diagnostics retained in {log_path.parent}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--build-dir", type=Path, default=Path("build"), help="CMake build tree"
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--decoder-threads", type=int, default=4)
    parser.add_argument(
        "--scenario", choices=("all", *SCENARIOS), default="all"
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        help="retain generated fixtures and logs in this directory",
    )
    args = parser.parse_args()
    if args.duration < 12.0:
        parser.error("duration must be at least 12 seconds for SRO warm-up")
    if args.decoder_threads < 1:
        parser.error("decoder threads must be positive")

    root = Path(__file__).resolve().parent.parent
    build_dir = args.build_dir.resolve()
    if not (build_dir / "airspy-tv").is_file():
        parser.error(f"decoder not found: {build_dir / 'airspy-tv'}")
    selected = SCENARIOS.values() if args.scenario == "all" else (
        SCENARIOS[args.scenario],
    )

    if args.work_dir is not None:
        work_dir = args.work_dir.resolve()
        work_dir.mkdir(parents=True, exist_ok=True)
        for scenario in selected:
            validate_scenario(
                root,
                build_dir,
                work_dir,
                scenario,
                args.duration,
                args.decoder_threads,
            )
        print(f"artifacts retained in {work_dir}")
        return 0

    with tempfile.TemporaryDirectory(prefix="airspy-tv-clock-") as temporary:
        work_dir = Path(temporary)
        for scenario in selected:
            validate_scenario(
                root,
                build_dir,
                work_dir,
                scenario,
                args.duration,
                args.decoder_threads,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
