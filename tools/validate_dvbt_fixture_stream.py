#!/usr/bin/env python3

"""Stream a synthetic DVB-T fixture through the native decoder end to end."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import subprocess
import tempfile

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
    return args


def run_validation(args: argparse.Namespace, work_dir: Path) -> None:
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
        "python3",
        str(root / "tools/generate_dvbt_fixture.py"),
        "-",
        "--duration",
        str(args.duration),
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
        timeout = max(30.0, args.duration * 10.0)
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
    print(
        f"passed: {args.dvbt_mode.upper()} GI {args.dvbt_guard}, "
        f"{len(recovered)} exact TS packets, source offset {source_offset}, "
        "TEI=0"
    )


def main() -> int:
    args = parse_args()
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
            work_dir = Path(tempfile.mkdtemp(prefix="dvbt-fixture-", dir=retained_root))
        run_validation(args, work_dir)
        if args.work_dir is not None:
            print(f"artifacts: {work_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
