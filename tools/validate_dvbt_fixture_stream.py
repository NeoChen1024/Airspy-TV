#!/usr/bin/env python3

"""Stream a synthetic DVB-T fixture through the native decoder end to end."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any

from validate_decode_report import validate_decode_report

TS_PACKET_BYTES = 188


def read_packets(path: Path) -> list[bytes]:
    payload = path.read_bytes()
    if not payload or len(payload) % TS_PACKET_BYTES != 0:
        raise RuntimeError(f"{path} is not a non-empty packet-aligned MPEG-TS")
    packets = [
        payload[offset : offset + TS_PACKET_BYTES]
        for offset in range(0, len(payload), TS_PACKET_BYTES)
    ]
    if any(packet[0] != 0x47 for packet in packets):
        raise RuntimeError(f"{path} contains a packet without sync byte 0x47")
    return packets


def require_cyclic_match(expected: list[bytes], recovered: list[bytes]) -> int:
    candidates = [
        index for index, packet in enumerate(expected) if packet == recovered[0]
    ]
    for start in candidates:
        if all(
            packet == expected[(start + offset) % len(expected)]
            for offset, packet in enumerate(recovered)
        ):
            return start
    raise RuntimeError(
        "recovered MPEG-TS is not an exact cyclic subsequence of the source"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, default=Path("build"))
    parser.add_argument("--work-dir", type=Path)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument(
        "--timeout",
        type=float,
        help="pipeline timeout in seconds (default: max(30, duration * 10))",
    )
    parser.add_argument("--decoder-threads", type=int, default=0)
    parser.add_argument("--result-json", type=Path)
    parser.add_argument("--center-frequency", type=int, default=557_000_000)
    parser.add_argument("--sample-clock-ppm", type=float, default=0.0)
    parser.add_argument(
        "--sample-clock-drift-ppm-per-minute", type=float, default=0.0
    )
    parser.add_argument(
        "--sample-clock-knot", action="append", metavar="SECONDS:PPM"
    )
    parser.add_argument("--lo-offset-hz", type=float, default=0.0)
    parser.add_argument("--lo-drift-hz-per-minute", type=float, default=0.0)
    parser.add_argument("--lo-ppm-knot", action="append", metavar="SECONDS:PPM")
    parser.add_argument("--require-no-rebootstrap", action="store_true")
    parser.add_argument("--dvbt-mode", choices=("2k", "8k"), default="2k")
    parser.add_argument(
        "--dvbt-channel-bandwidth",
        choices=("5M", "6M", "7M", "8M"),
        default="6M",
    )
    parser.add_argument(
        "--dvbt-guard",
        choices=("1/32", "1/16", "1/8", "1/4"),
        default="1/32",
    )
    parser.add_argument(
        "--dvbt-modulation",
        choices=("qpsk", "qam16", "qam64"),
        default="qam64",
    )
    parser.add_argument(
        "--dvbt-code-rate",
        choices=("1/2", "2/3", "3/4", "5/6", "7/8"),
        default="2/3",
    )
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("duration must be positive")
    if args.timeout is not None and args.timeout <= 0.0:
        parser.error("timeout must be positive")
    if not 0 <= args.decoder_threads <= 256:
        parser.error("decoder threads must be between 0 and 256")
    return args


def run_validation(args: argparse.Namespace, work_dir: Path) -> dict[str, Any]:
    root = Path(__file__).resolve().parents[1]
    decoder = args.build_dir.resolve() / "airspy-tv"
    if not decoder.is_file():
        raise RuntimeError(f"decoder executable was not found: {decoder}")

    expected_ts = work_dir / "source.expected.ts"
    recovered_ts = work_dir / "recovered.ts"
    generator_log = work_dir / "generator.log"
    decoder_log = work_dir / "decoder.log"
    report_dir = work_dir / "report"
    env = os.environ.copy()
    env["XDG_CACHE_HOME"] = str(work_dir / "cache")

    generator_command = [
        sys.executable,
        str(root / "tools/generate_dvbt_fixture.py"),
        "-",
        "--duration",
        str(args.duration),
        "--center-frequency",
        str(args.center_frequency),
        "--expected-ts",
        str(expected_ts),
        "--dvbt-mode",
        args.dvbt_mode,
        "--dvbt-channel-bandwidth",
        args.dvbt_channel_bandwidth,
        "--dvbt-guard",
        args.dvbt_guard,
        "--dvbt-modulation",
        args.dvbt_modulation,
        "--dvbt-code-rate",
        args.dvbt_code_rate,
    ]
    for option, value in (
        ("--sample-clock-ppm", args.sample_clock_ppm),
        (
            "--sample-clock-drift-ppm-per-minute",
            args.sample_clock_drift_ppm_per_minute,
        ),
        ("--lo-offset-hz", args.lo_offset_hz),
        ("--lo-drift-hz-per-minute", args.lo_drift_hz_per_minute),
    ):
        if value != 0.0:
            generator_command.extend((option, str(value)))
    for knot in args.sample_clock_knot or ():
        generator_command.extend(("--sample-clock-knot", knot))
    for knot in args.lo_ppm_knot or ():
        generator_command.extend(("--lo-ppm-knot", knot))
    decoder_command = [
        str(decoder),
        "--decode-iq",
        "-",
        "--sample-rate",
        "10000000",
        "--dvbt-channel-bandwidth",
        args.dvbt_channel_bandwidth,
        "--ts-output",
        str(recovered_ts),
        "--report-dir",
        str(report_dir),
    ]
    if args.decoder_threads != 0:
        decoder_command.extend(("--decoder-threads", str(args.decoder_threads)))

    with generator_log.open("wb") as generator_stderr, decoder_log.open(
        "wb"
    ) as decoder_output:
        generator = subprocess.Popen(
            generator_command,
            cwd=root,
            env=env,
            stdout=subprocess.PIPE,
            stderr=generator_stderr,
        )
        if generator.stdout is None:
            raise RuntimeError("failed to open generator stdout pipe")
        native_decoder = subprocess.Popen(
            decoder_command,
            cwd=root,
            env=env,
            stdin=generator.stdout,
            stdout=decoder_output,
            stderr=subprocess.STDOUT,
        )
        generator.stdout.close()
        timeout = args.timeout or max(30.0, args.duration * 10.0)
        try:
            decoder_result = native_decoder.wait(timeout=timeout)
            generator_result = generator.wait(timeout=10.0)
        except subprocess.TimeoutExpired as exception:
            native_decoder.kill()
            generator.kill()
            native_decoder.wait()
            generator.wait()
            raise RuntimeError("fixture pipeline timed out") from exception

    if generator_result != 0 or decoder_result != 0:
        generator_text = generator_log.read_text(errors="replace")
        decoder_text = decoder_log.read_text(errors="replace")
        raise RuntimeError(
            f"fixture pipeline failed: generator={generator_result}, "
            f"decoder={decoder_result}\n"
            f"generator log:\n{generator_text}\n"
            f"decoder log:\n{decoder_text}"
        )

    expected = read_packets(expected_ts)
    recovered = read_packets(recovered_ts)
    tei_packets = sum((packet[1] & 0x80) != 0 for packet in recovered)
    if tei_packets != 0:
        raise RuntimeError(f"recovered MPEG-TS contains {tei_packets} TEI packets")
    source_offset = require_cyclic_match(expected, recovered)
    report = validate_decode_report(
        report_dir,
        expected_samples=round(args.duration * 10_000_000),
        expected_packets=len(recovered),
        expected_sample_rate=10_000_000,
        expected_bandwidth_hz=int(args.dvbt_channel_bandwidth[:-1]) * 1_000_000,
        expected_transmission_mode=args.dvbt_mode,
        expected_guard_interval=args.dvbt_guard,
        expected_constellation=args.dvbt_modulation,
        expected_code_rate=args.dvbt_code_rate,
    )
    if args.require_no_rebootstrap:
        rebootstrap = report["stats"]["counters"]["cfo_rebootstrap_count"]
        if rebootstrap != 0:
            raise RuntimeError(
                "smooth clock profile triggered "
                f"{rebootstrap} unnecessary frontend rebootstrap(s)"
            )
    print(
        f"passed: {args.dvbt_mode.upper()} GI {args.dvbt_guard}, "
        f"{len(recovered)} exact TS packets, source offset {source_offset}, "
        f"TEI=0, {sum(report['stream_records'].values())} report records"
    )
    return {
        "schema_version": 0,
        "record_type": "fixture_validation",
        "status": "passed",
        "configuration": {
            "duration_seconds": args.duration,
            "sample_rate_hz": 10_000_000,
            "decoder_threads": args.decoder_threads,
            "transmission_mode": args.dvbt_mode,
            "channel_bandwidth": args.dvbt_channel_bandwidth,
            "guard_interval": args.dvbt_guard,
            "constellation": args.dvbt_modulation,
            "code_rate": args.dvbt_code_rate,
            "center_frequency_hz": args.center_frequency,
            "sample_clock_knots_ppm": args.sample_clock_knot or [],
            "lo_knots_ppm": args.lo_ppm_knot or [],
        },
        "transport_validation": {
            "source_cycle_packets": len(expected),
            "recovered_packets": len(recovered),
            "source_offset_packets": source_offset,
            "tei_packets": tei_packets,
            "exact_cyclic_match": True,
        },
        "decode_report": {
            "manifest": report["manifest"],
            "record_counts": report["stream_records"],
            "stats": report["stats"],
            "source_session": report["source_session"],
        },
    }


def write_result(path: Path, result: dict[str, Any]) -> None:
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    try:
        with ExitStack() as stack:
            if args.work_dir is None:
                work_dir = Path(
                    stack.enter_context(
                        tempfile.TemporaryDirectory(prefix="airspy-tv-e2e-")
                    )
                )
            else:
                retained_root = args.work_dir.resolve()
                retained_root.mkdir(parents=True, exist_ok=True)
                work_dir = Path(
                    tempfile.mkdtemp(prefix="dvbt-fixture-", dir=retained_root)
                )
            result = run_validation(args, work_dir)
            if args.result_json is not None:
                write_result(args.result_json, result)
            if args.work_dir is not None:
                print(f"artifacts: {work_dir}")
    except Exception as exception:
        if args.result_json is not None:
            write_result(
                args.result_json,
                {
                    "schema_version": 0,
                    "record_type": "fixture_validation",
                    "status": "failed",
                    "configuration": {
                        "duration_seconds": args.duration,
                        "decoder_threads": args.decoder_threads,
                        "transmission_mode": args.dvbt_mode,
                        "channel_bandwidth": args.dvbt_channel_bandwidth,
                        "guard_interval": args.dvbt_guard,
                        "constellation": args.dvbt_modulation,
                        "code_rate": args.dvbt_code_rate,
                        "center_frequency_hz": args.center_frequency,
                        "sample_clock_knots_ppm": args.sample_clock_knot or [],
                        "lo_knots_ppm": args.lo_ppm_knot or [],
                    },
                    "error": {
                        "type": type(exception).__name__,
                        "message": str(exception),
                    },
                },
            )
        raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
