"""Discover and describe recorded I/Q files used for real-signal validation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 0
CS16_SAMPLE_BYTES = 4


class CorpusError(RuntimeError):
    """Raised when a recognized corpus file is malformed or ambiguous."""


@dataclass(frozen=True)
class RealSignalCase:
    case_index: int
    case_id: str
    input_path: Path
    data_path: Path
    metadata_path: Path | None
    sample_rate_hz: int
    center_frequency_hz: int | None
    file_size_bytes: int
    modified_time_ns: int
    complex_samples: int
    duration_seconds: float
    selected: bool
    exclusion_reason: str | None

    def to_inventory_record(self, root: Path) -> dict[str, Any]:
        record = asdict(self)
        for name in ("input_path", "data_path", "metadata_path"):
            path = record[name]
            record[name] = (
                str(Path(path).relative_to(root)) if path is not None else None
            )
        return record


@dataclass(frozen=True)
class RealSignalCorpus:
    root: Path
    cases: tuple[RealSignalCase, ...]
    ignored_files: tuple[dict[str, Any], ...]
    fallback_sample_rate_hz: int
    channel_bandwidth_hz: int
    include_long_recordings: bool
    long_recording_threshold_seconds: float

    @property
    def selected_cases(self) -> tuple[RealSignalCase, ...]:
        return tuple(case for case in self.cases if case.selected)

    def to_json(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "record_type": "real_signal_corpus",
            "root": str(self.root),
            "configuration": {
                "fallback_sample_rate_hz": self.fallback_sample_rate_hz,
                "channel_bandwidth_hz": self.channel_bandwidth_hz,
                "include_long_recordings": self.include_long_recordings,
                "long_recording_threshold_seconds": (
                    self.long_recording_threshold_seconds
                ),
                "recognized_iq_extension": ".cs16",
                "recognized_metadata_extension": ".json",
                "ignored_file_policy": "ignore every other regular file",
            },
            "totals": {
                "recordings": len(self.cases),
                "selected": len(self.selected_cases),
                "excluded_long": sum(
                    case.exclusion_reason == "long_recording" for case in self.cases
                ),
                "file_size_bytes": sum(case.file_size_bytes for case in self.cases),
                "signal_seconds": sum(case.duration_seconds for case in self.cases),
                "selected_signal_seconds": sum(
                    case.duration_seconds for case in self.selected_cases
                ),
                "ignored_files": len(self.ignored_files),
            },
            "recordings": [case.to_inventory_record(self.root) for case in self.cases],
            "ignored_files": list(self.ignored_files),
        }


def _positive_integer(value: Any, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise CorpusError(f"{location} must be a positive integer")
    return value


def _load_sidecar(path: Path) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite number: {value}")

    def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate object key: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (OSError, UnicodeError, ValueError) as exception:
        raise CorpusError(
            f"unable to parse metadata {path}: {exception}"
        ) from exception
    if not isinstance(value, dict):
        raise CorpusError(f"metadata must contain a JSON object: {path}")
    return value


def _resolve_sidecar(path: Path, value: dict[str, Any]) -> tuple[Path, int, int | None]:
    data_file = value.get("data_file")
    if not isinstance(data_file, str) or not data_file:
        raise CorpusError(f"metadata data_file must be a filename: {path}")
    relative = Path(data_file)
    if relative.is_absolute() or relative.name != data_file:
        raise CorpusError(f"metadata data_file must name a same-directory file: {path}")
    data_path = path.parent / relative
    if data_path.suffix.lower() != ".cs16" or not data_path.is_file():
        raise CorpusError(f"metadata target is not a regular .cs16 file: {data_path}")
    if value.get("datatype") != "ci16_le":
        raise CorpusError(f"metadata datatype must be ci16_le: {path}")
    if value.get("iq_order") != "IQ":
        raise CorpusError(f"metadata iq_order must be IQ: {path}")
    sample_rate_hz = _positive_integer(value.get("sample_rate"), f"{path}:sample_rate")
    center_frequency = value.get("center_frequency")
    if center_frequency is not None:
        center_frequency = _positive_integer(
            center_frequency, f"{path}:center_frequency"
        )
    return data_path, sample_rate_hz, center_frequency


def _make_case(
    *,
    case_index: int,
    input_path: Path,
    data_path: Path,
    metadata_path: Path | None,
    sample_rate_hz: int,
    center_frequency_hz: int | None,
    declared_samples: Any,
    include_long_recordings: bool,
    long_recording_threshold_seconds: float,
) -> RealSignalCase:
    file_status = data_path.stat()
    file_size_bytes = file_status.st_size
    if file_size_bytes % CS16_SAMPLE_BYTES != 0:
        raise CorpusError(f"I/Q file has an incomplete CS16 sample: {data_path}")
    complex_samples = file_size_bytes // CS16_SAMPLE_BYTES
    if complex_samples == 0:
        raise CorpusError(f"I/Q file is empty: {data_path}")
    if declared_samples is not None:
        declared = _positive_integer(
            declared_samples, f"{metadata_path}:complex_samples"
        )
        if declared != complex_samples:
            raise CorpusError(
                f"metadata complex_samples is {declared}, file contains "
                f"{complex_samples}: {metadata_path}"
            )
    duration_seconds = complex_samples / sample_rate_hz
    selected = include_long_recordings or (
        duration_seconds <= long_recording_threshold_seconds
    )
    return RealSignalCase(
        case_index=case_index,
        case_id=data_path.name,
        input_path=input_path,
        data_path=data_path,
        metadata_path=metadata_path,
        sample_rate_hz=sample_rate_hz,
        center_frequency_hz=center_frequency_hz,
        file_size_bytes=file_size_bytes,
        modified_time_ns=file_status.st_mtime_ns,
        complex_samples=complex_samples,
        duration_seconds=duration_seconds,
        selected=selected,
        exclusion_reason=None if selected else "long_recording",
    )


def discover_real_signal_corpus(
    root: Path,
    *,
    fallback_sample_rate_hz: int = 10_000_000,
    channel_bandwidth_hz: int = 6_000_000,
    include_long_recordings: bool = False,
    long_recording_threshold_seconds: float = 600.0,
) -> RealSignalCorpus:
    root = root.resolve()
    if not root.is_dir():
        raise CorpusError(f"real-signal corpus is not a directory: {root}")
    _positive_integer(fallback_sample_rate_hz, "fallback sample rate")
    _positive_integer(channel_bandwidth_hz, "channel bandwidth")
    if (
        not math.isfinite(long_recording_threshold_seconds)
        or long_recording_threshold_seconds <= 0.0
    ):
        raise CorpusError("long-recording threshold must be positive")

    regular_files = sorted(
        (path for path in root.iterdir() if path.is_file()), key=lambda path: path.name
    )
    sidecars = [path for path in regular_files if path.suffix.lower() == ".json"]
    cs16_files = {
        path.resolve(): path for path in regular_files if path.suffix.lower() == ".cs16"
    }
    claims: dict[Path, tuple[Path, dict[str, Any], int, int | None]] = {}
    for sidecar in sidecars:
        metadata = _load_sidecar(sidecar)
        data_path, sample_rate_hz, center_frequency_hz = _resolve_sidecar(
            sidecar, metadata
        )
        resolved_data = data_path.resolve()
        if resolved_data in claims:
            previous = claims[resolved_data][0]
            raise CorpusError(
                f"metadata files claim the same recording: {previous}, {sidecar}"
            )
        claims[resolved_data] = (
            sidecar,
            metadata,
            sample_rate_hz,
            center_frequency_hz,
        )

    specifications: list[tuple[Path, Path, Path | None, int, int | None, Any]] = []
    for data_path in sorted(cs16_files.values(), key=lambda path: path.name):
        claim = claims.get(data_path.resolve())
        if claim is None:
            specifications.append(
                (data_path, data_path, None, fallback_sample_rate_hz, None, None)
            )
            continue
        sidecar, metadata, sample_rate_hz, center_frequency_hz = claim
        specifications.append(
            (
                sidecar,
                data_path,
                sidecar,
                sample_rate_hz,
                center_frequency_hz,
                metadata.get("complex_samples"),
            )
        )

    cases = tuple(
        _make_case(
            case_index=index,
            input_path=input_path,
            data_path=data_path,
            metadata_path=metadata_path,
            sample_rate_hz=sample_rate_hz,
            center_frequency_hz=center_frequency_hz,
            declared_samples=declared_samples,
            include_long_recordings=include_long_recordings,
            long_recording_threshold_seconds=long_recording_threshold_seconds,
        )
        for index, (
            input_path,
            data_path,
            metadata_path,
            sample_rate_hz,
            center_frequency_hz,
            declared_samples,
        ) in enumerate(specifications)
    )
    recognized = {path.resolve() for path in cs16_files.values()} | {
        path.resolve() for path in sidecars
    }
    ignored_files = tuple(
        {
            "path": str(path.relative_to(root)),
            "suffix": path.suffix,
            "file_size_bytes": path.stat().st_size,
        }
        for path in regular_files
        if path.resolve() not in recognized
    )
    return RealSignalCorpus(
        root=root,
        cases=cases,
        ignored_files=ignored_files,
        fallback_sample_rate_hz=fallback_sample_rate_hz,
        channel_bandwidth_hz=channel_bandwidth_hz,
        include_long_recordings=include_long_recordings,
        long_recording_threshold_seconds=long_recording_threshold_seconds,
    )
