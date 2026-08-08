#!/usr/bin/env python3

"""Validate an airspy-tv machine-readable DVB-T decode report."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


class ReportValidationError(RuntimeError):
    """Raised when a decode report violates its schema or invariants."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ReportValidationError(message)


def reject_constant(value: str) -> None:
    raise ReportValidationError(f"non-finite JSON number: {value}")


def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ReportValidationError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def parse_json(text: str, source: str) -> Any:
    try:
        value = json.loads(
            text,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (json.JSONDecodeError, ReportValidationError) as exception:
        raise ReportValidationError(
            f"invalid JSON in {source}: {exception}"
        ) from exception
    require_finite_numbers(value, source)
    return value


def require_finite_numbers(value: Any, location: str) -> None:
    if isinstance(value, float):
        require(math.isfinite(value), f"non-finite number at {location}")
    elif isinstance(value, dict):
        for key, child in value.items():
            require_finite_numbers(child, f"{location}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            require_finite_numbers(child, f"{location}[{index}]")


def load_json(path: Path) -> dict[str, Any]:
    require(path.is_file(), f"missing report file: {path.name}")
    value = parse_json(path.read_text(encoding="utf-8"), path.name)
    require(isinstance(value, dict), f"{path.name} must contain a JSON object")
    return value


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    require(path.is_file(), f"missing report stream: {path.name}")
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            value = parse_json(line, f"{path.name}:{line_number}")
            require(
                isinstance(value, dict),
                f"{path.name}:{line_number} must contain a JSON object",
            )
            records.append(value)
    return records


def field(value: dict[str, Any], *path: str) -> Any:
    current: Any = value
    joined = ".".join(path)
    for key in path:
        require(
            isinstance(current, dict) and key in current, f"missing field: {joined}"
        )
        current = current[key]
    return current


def require_zero_counters(value: dict[str, Any], location: str) -> None:
    counters = field(value, "counters")
    for name in (
        "dropped_blocks",
        "overlap_join_failures",
        "phase_discontinuities",
        "rs_uncorrectable_packets",
        "tei_packets",
    ):
        require(
            field(counters, name) == 0,
            f"{location}.counters.{name} is not zero",
        )


def require_transport(
    value: dict[str, Any], location: str, expected_packets: int | None
) -> int:
    transport = field(value, "transport")
    decoded = field(transport, "decoded_packets")
    emitted = field(transport, "emitted_packets")
    usable = field(transport, "usable_packets")
    require(isinstance(decoded, int) and decoded > 0, f"{location} decoded no TS")
    require(decoded == emitted == usable, f"{location} TS packet counts disagree")
    require(field(transport, "tei_packets") == 0, f"{location} contains TEI packets")
    require(
        field(transport, "emitted_partial_bytes") == 0,
        f"{location} emitted a partial TS packet",
    )
    require(
        field(transport, "bytes") == emitted * 188,
        f"{location} TS byte count disagrees with packet count",
    )
    if expected_packets is not None:
        require(
            emitted == expected_packets,
            f"{location} reports {emitted} packets, expected {expected_packets}",
        )
    return emitted


def require_samples(
    value: dict[str, Any], location: str, expected_samples: int | None
) -> None:
    samples = field(value, "samples")
    submitted = field(samples, "submitted")
    processed = field(samples, "processed")
    require(submitted == processed, f"{location} submitted/processed samples disagree")
    if expected_samples is not None:
        require(
            submitted == expected_samples,
            f"{location} reports {submitted} samples, expected {expected_samples}",
        )


def require_real_samples(
    value: dict[str, Any], location: str, expected_samples: int
) -> None:
    samples = field(value, "samples")
    submitted = field(samples, "submitted")
    processed = field(samples, "processed")
    require(
        submitted == expected_samples,
        f"{location} reports {submitted} samples, expected {expected_samples}",
    )
    require(
        isinstance(processed, int) and 0 <= processed <= submitted,
        f"{location} processed sample count is out of range",
    )


def load_decode_report(report_dir: Path) -> dict[str, Any]:
    report_dir = report_dir.resolve()
    manifest = load_json(report_dir / "manifest.json")
    require(field(manifest, "report_format_version") == 0, "unsupported report version")
    require(field(manifest, "mode") == "dvbt", "manifest mode is not dvbt")

    inventory = field(manifest, "streams")
    require(isinstance(inventory, list), "manifest streams must be an array")
    paths: set[str] = set()
    streams: dict[str, list[dict[str, Any]]] = {}
    for index, entry in enumerate(inventory):
        require(isinstance(entry, dict), f"manifest stream {index} is not an object")
        relative = field(entry, "path")
        require(
            isinstance(relative, str) and relative,
            f"manifest stream {index} has no path",
        )
        require(relative not in paths, f"duplicate manifest stream path: {relative}")
        paths.add(relative)
        stream_path = (report_dir / relative).resolve()
        require(
            stream_path.parent == report_dir,
            f"manifest stream path is not a direct report child: {relative}",
        )
        records = load_jsonl(stream_path)
        record_type = field(entry, "record_type")
        require(
            record_type not in streams, f"duplicate manifest record type: {record_type}"
        )
        previous_sequence = -1
        for line_number, record in enumerate(records, start=1):
            prefix = f"{relative}:{line_number}"
            require(
                field(record, "schema_version") == field(entry, "schema_version"),
                f"{prefix} schema version disagrees with manifest",
            )
            require(
                field(record, "record_type") == field(entry, "record_type"),
                f"{prefix} record type disagrees with manifest",
            )
            require(field(record, "mode") == "dvbt", f"{prefix} mode is not dvbt")
            sequence = field(record, "sequence")
            require(isinstance(sequence, int), f"{prefix} sequence is not an integer")
            require(
                sequence > previous_sequence, f"{prefix} sequence is not increasing"
            )
            previous_sequence = sequence
        streams[record_type] = records

    expected_types = {
        "source_session",
        "frontend_block",
        "pipeline_sample",
        "demod_window",
        "fec_window",
        "decoder_event",
    }
    require(set(streams) == expected_types, "manifest stream inventory is incomplete")
    stats = load_json(report_dir / "stats.json")
    require(field(stats, "schema_version") == 0, "stats schema version is not 0")
    require(field(stats, "mode") == "dvbt", "stats mode is not dvbt")
    sessions = streams["source_session"]
    require(len(sessions) == 1, "source-sessions.jsonl must contain exactly one record")
    return {
        "manifest": manifest,
        "streams": streams,
        "stats": stats,
        "source_session": sessions[0],
    }


def validate_real_decode_report(
    report_dir: Path,
    *,
    expected_samples: int,
    expected_sample_rate: int,
    expected_bandwidth_hz: int,
) -> dict[str, Any]:
    result = load_decode_report(report_dir)
    stats = result["stats"]
    session = result["source_session"]
    status = field(stats, "status")
    require(status in {"completed", "no_transport"}, "real decode did not complete")
    require(field(stats, "error") is None, "real decode report contains an error")
    require(field(session, "status") == status, "source and run status disagree")
    require(field(session, "error") is None, "source session contains an error")
    expected_exit_code = 0 if status == "completed" else 2
    require(
        field(stats, "exit_code") == expected_exit_code, "decode exit code disagrees"
    )

    source_counts = field(stats, "source_sessions")
    require(field(source_counts, "total") == 1, "source session total is not one")
    require(field(source_counts, "failed") == 0, "stats contains a failed source")
    require(
        field(source_counts, status) == 1,
        f"stats source session does not report {status}",
    )
    require_real_samples(stats, "stats", expected_samples)
    require_real_samples(session, "source session", expected_samples)
    require(
        field(stats, "samples") == field(session, "samples"),
        "run and source sample summaries disagree",
    )
    require(
        field(session, "source", "sample_rate_hz") == expected_sample_rate,
        "source session sample rate is incorrect",
    )
    require(
        field(session, "decoder", "channel_bandwidth_hz") == expected_bandwidth_hz,
        "source session channel bandwidth is incorrect",
    )
    require(
        field(stats, "transport") == field(session, "transport"),
        "run and source transport summaries disagree",
    )
    require(
        field(stats, "counters") == field(session, "counters"),
        "run and source counters disagree",
    )
    transport = field(stats, "transport")
    for name in (
        "decoded_packets",
        "emitted_packets",
        "usable_packets",
        "tei_packets",
        "emitted_partial_bytes",
        "bytes",
    ):
        value = field(transport, name)
        require(
            isinstance(value, int) and value >= 0,
            f"transport.{name} is not a non-negative integer",
        )
    require(
        field(transport, "bytes") == field(transport, "emitted_packets") * 188,
        "transport byte and packet counts disagree",
    )
    require(
        field(transport, "emitted_partial_bytes") == 0,
        "transport contains a partial packet",
    )
    if status == "no_transport":
        require(
            field(transport, "emitted_packets") == 0,
            "no-transport decode emitted packets",
        )
    result["stream_records"] = {
        record_type: len(records)
        for record_type, records in result.pop("streams").items()
    }
    return result


def validate_decode_report(
    report_dir: Path,
    *,
    expected_samples: int | None = None,
    expected_packets: int | None = None,
    expected_sample_rate: int | None = None,
    expected_bandwidth_hz: int | None = None,
    expected_transmission_mode: str | None = None,
    expected_guard_interval: str | None = None,
    expected_constellation: str | None = None,
    expected_code_rate: str | None = None,
) -> dict[str, Any]:
    loaded = load_decode_report(report_dir)
    manifest = loaded["manifest"]
    streams = loaded["streams"]
    expected_types = {
        "source_session",
        "frontend_block",
        "pipeline_sample",
        "demod_window",
        "fec_window",
        "decoder_event",
    }
    require(set(streams) == expected_types, "manifest stream inventory is incomplete")
    for record_type in expected_types - {"decoder_event"}:
        require(streams[record_type], f"{record_type} stream is empty")

    stats = loaded["stats"]
    require(field(stats, "status") == "completed", "decode did not complete")
    require(field(stats, "exit_code") == 0, "decode exit code is not zero")
    require(field(stats, "error") is None, "decode report contains an error")
    source_counts = field(stats, "source_sessions")
    require(field(source_counts, "total") == 1, "stats source session total is not one")
    require(field(source_counts, "completed") == 1, "source session did not complete")
    require(field(source_counts, "failed") == 0, "stats contains a failed source")
    require(
        field(source_counts, "no_transport") == 0,
        "stats contains a no-transport source",
    )
    require_samples(stats, "stats", expected_samples)
    require_zero_counters(stats, "stats")
    packet_count = require_transport(stats, "stats", expected_packets)
    windows = field(stats, "windows")
    require(field(windows, "demod") > 0, "stats contains no demod windows")
    require(field(windows, "ofdm_locked") > 0, "stats contains no OFDM lock")
    require(field(windows, "tps_locked") > 0, "stats contains no TPS lock")

    session = loaded["source_session"]
    require(field(session, "status") == "completed", "source session did not complete")
    require(field(session, "error") is None, "source session contains an error")
    require_samples(session, "source session", expected_samples)
    require_zero_counters(session, "source session")
    require_transport(session, "source session", packet_count)
    if expected_sample_rate is not None:
        require(
            field(session, "source", "sample_rate_hz") == expected_sample_rate,
            "source session sample rate is incorrect",
        )
    if expected_bandwidth_hz is not None:
        require(
            field(session, "decoder", "channel_bandwidth_hz") == expected_bandwidth_hz,
            "source session channel bandwidth is incorrect",
        )
    final_state = field(session, "final_state")
    for name in ("ofdm_locked", "tps_locked", "rs_synchronized", "energy_synchronized"):
        require(
            field(final_state, name) is True,
            f"final decoder state is not locked: {name}",
        )

    warning_events = [
        field(record, "event")
        for record in streams["decoder_event"]
        if field(record, "severity") == "warning"
    ]
    require(
        not warning_events,
        "decoder report contains warning events: " + ", ".join(warning_events),
    )

    expected_parameters = {
        "transmission_mode": expected_transmission_mode,
        "guard_interval": expected_guard_interval,
        "constellation": expected_constellation,
        "code_rate": expected_code_rate,
    }
    requested_parameters = {
        name: value for name, value in expected_parameters.items() if value is not None
    }
    if requested_parameters:
        locked_records = [
            record
            for record in streams["demod_window"]
            if field(record, "lock", "ofdm") is True
            and field(record, "lock", "tps") is True
        ]
        require(locked_records, "demod report contains no locked OFDM/TPS window")
        parameters = field(locked_records[-1], "parameters")
        for name, expected in requested_parameters.items():
            require(
                field(parameters, name) == expected,
                f"decoded {name} is {field(parameters, name)!r}, expected {expected!r}",
            )

    return {
        "packets": packet_count,
        "stream_records": {
            record_type: len(records) for record_type, records in streams.items()
        },
        "manifest": manifest,
        "stats": stats,
        "source_session": session,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--expected-samples", type=int)
    parser.add_argument("--expected-packets", type=int)
    parser.add_argument("--expected-sample-rate", type=int)
    parser.add_argument("--expected-bandwidth-hz", type=int)
    parser.add_argument("--expected-transmission-mode", choices=("2k", "8k"))
    parser.add_argument(
        "--expected-guard-interval", choices=("1/32", "1/16", "1/8", "1/4")
    )
    parser.add_argument("--expected-constellation", choices=("qpsk", "qam16", "qam64"))
    parser.add_argument(
        "--expected-code-rate", choices=("1/2", "2/3", "3/4", "5/6", "7/8")
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = validate_decode_report(
        args.report_dir,
        expected_samples=args.expected_samples,
        expected_packets=args.expected_packets,
        expected_sample_rate=args.expected_sample_rate,
        expected_bandwidth_hz=args.expected_bandwidth_hz,
        expected_transmission_mode=args.expected_transmission_mode,
        expected_guard_interval=args.expected_guard_interval,
        expected_constellation=args.expected_constellation,
        expected_code_rate=args.expected_code_rate,
    )
    print(
        f"valid report: {result['packets']} TS packets, "
        f"{sum(result['stream_records'].values())} JSONL records"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
