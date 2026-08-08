"""Machine-readable report support for the repository validation runner."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
from typing import Any
import xml.etree.ElementTree as ElementTree

REPORT_VERSION = 0


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_write_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class JsonlStream:
    def __init__(self, path: Path, record_type: str) -> None:
        self.path = path
        self.record_type = record_type
        self.sequence = 0
        self.stream = path.open("w", encoding="utf-8", buffering=1)

    def write(self, value: dict[str, Any]) -> dict[str, Any]:
        self.sequence += 1
        record = {
            **value,
            "schema_version": REPORT_VERSION,
            "record_type": self.record_type,
            "sequence": self.sequence,
        }
        self.stream.write(
            json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False)
            + "\n"
        )
        self.stream.flush()
        return record

    def close(self) -> None:
        self.stream.close()


@dataclass(frozen=True)
class CommandResult:
    return_code: int
    elapsed_seconds: float
    started_at: str
    finished_at: str
    log_path: Path


class ValidationReport:
    def __init__(
        self,
        directory: Path,
        root: Path,
        profiles: list[str],
        configuration: dict[str, Any],
    ) -> None:
        self.directory = directory
        self.root = root
        self.profiles = profiles
        self.logs_dir = directory / "logs"
        self.junit_dir = directory / "junit"
        self.failures_dir = directory / "failures"
        self.logs_dir.mkdir(parents=True)
        self.junit_dir.mkdir(parents=True)
        self.failures_dir.mkdir(parents=True)
        self.stages = JsonlStream(directory / "stages.jsonl", "validation_stage")
        self.ctest = JsonlStream(directory / "ctest.jsonl", "ctest_case")
        self.fixtures = JsonlStream(directory / "fixtures.jsonl", "fixture_case")
        self._write_manifest()
        atomic_write_json(
            directory / "environment.json",
            collect_environment(root, profiles, configuration),
        )

    def _write_manifest(self) -> None:
        manifest = {
            "report_format_version": REPORT_VERSION,
            "record_type": "validation_manifest",
            "tool": {"name": "airspy-tv-validation", "version": "0.1.0"},
            "files": [
                {
                    "path": "summary.json",
                    "format": "json",
                    "record_type": "validation_summary",
                    "schema_version": REPORT_VERSION,
                },
                {
                    "path": "environment.json",
                    "format": "json",
                    "record_type": "validation_environment",
                    "schema_version": REPORT_VERSION,
                },
            ],
            "streams": [
                {
                    "path": "stages.jsonl",
                    "record_type": "validation_stage",
                    "schema_version": REPORT_VERSION,
                },
                {
                    "path": "ctest.jsonl",
                    "record_type": "ctest_case",
                    "schema_version": REPORT_VERSION,
                },
                {
                    "path": "fixtures.jsonl",
                    "record_type": "fixture_case",
                    "schema_version": REPORT_VERSION,
                },
            ],
            "artifacts": {
                "stage_logs": "logs/<profile>-<stage>.log",
                "ctest_junit": "junit/<profile>.xml",
                "fixture_failures": "failures/<profile>/<case>/",
            },
        }
        atomic_write_json(self.directory / "manifest.json", manifest)

    def write_summary(self, summary: dict[str, Any]) -> None:
        atomic_write_json(self.directory / "summary.json", summary)

    def write_stage(self, value: dict[str, Any]) -> dict[str, Any]:
        return self.stages.write(value)

    def write_ctest(self, value: dict[str, Any]) -> dict[str, Any]:
        return self.ctest.write(value)

    def write_fixture(self, value: dict[str, Any]) -> dict[str, Any]:
        return self.fixtures.write(value)

    def close(self) -> None:
        self.stages.close()
        self.ctest.close()
        self.fixtures.close()


def command_output(command: list[str], root: Path) -> str | None:
    try:
        result = subprocess.run(
            command,
            cwd=root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return result.stdout.rstrip()


def module_version(name: str) -> str | None:
    try:
        module = importlib.import_module(name)
    except ImportError:
        return None
    version = getattr(module, "__version__", None)
    if version is not None:
        return str(version)
    if name == "gnuradio":
        try:
            gr = importlib.import_module("gnuradio.gr")
            return str(gr.version())
        except (ImportError, AttributeError):
            return None
    return None


def collect_environment(
    root: Path, profiles: list[str], configuration: dict[str, Any]
) -> dict[str, Any]:
    status = command_output(["git", "status", "--porcelain=v1"], root)
    cpu_model = None
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("model name") and ":" in line:
                cpu_model = line.split(":", 1)[1].strip()
                break
    tools: dict[str, Any] = {}
    for name, command in {
        "cmake": ["cmake", "--version"],
        "ctest": ["ctest", "--version"],
        "ninja": ["ninja", "--version"],
        "ffmpeg": ["ffmpeg", "-version"],
        "c_compiler": ["cc", "--version"],
        "cxx_compiler": ["c++", "--version"],
    }.items():
        executable = shutil.which(command[0])
        output = command_output(command, root) if executable is not None else None
        tools[name] = {
            "executable": executable,
            "version_output": output.splitlines()[0] if output else None,
        }
    return {
        "schema_version": REPORT_VERSION,
        "record_type": "validation_environment",
        "captured_at": utc_now(),
        "host": {
            "platform": platform.platform(),
            "hostname": platform.node(),
            "machine": platform.machine(),
            "cpu_model": cpu_model,
            "logical_cpu_count": os.cpu_count(),
        },
        "python": {
            "version": sys.version,
            "executable": sys.executable,
            "prefix": sys.prefix,
            "base_prefix": sys.base_prefix,
            "packages": {
                "numpy": module_version("numpy"),
                "gnuradio": module_version("gnuradio"),
            },
        },
        "source_control": {
            "commit": command_output(["git", "rev-parse", "HEAD"], root),
            "branch": command_output(["git", "branch", "--show-current"], root),
            "dirty": bool(status),
            "status_porcelain": status.splitlines() if status else [],
        },
        "tools": tools,
        "profiles": profiles,
        "configuration": configuration,
    }


def sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def cmake_cache_values(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    selected: dict[str, str] = {}
    exact_names = {
        "BUILD_TESTING",
        "CMAKE_BUILD_TYPE",
        "CMAKE_C_COMPILER",
        "CMAKE_C_FLAGS",
        "CMAKE_CXX_COMPILER",
        "CMAKE_CXX_FLAGS",
        "CMAKE_EXE_LINKER_FLAGS",
        "CMAKE_GENERATOR",
    }
    flag_prefixes = (
        "CMAKE_C_FLAGS_",
        "CMAKE_CXX_FLAGS_",
        "CMAKE_EXE_LINKER_FLAGS_",
    )
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith(("//", "#")) or "=" not in line:
            continue
        typed_name, value = line.split("=", 1)
        name = typed_name.split(":", 1)[0]
        if name.endswith(("-ADVANCED", "-STRINGS")):
            continue
        if (
            name in exact_names
            or name.startswith("AIRSPY_TV_")
            or name.startswith(flag_prefixes)
        ):
            selected[name] = value
    return dict(sorted(selected.items()))


def parse_junit(path: Path, profile: str) -> list[dict[str, Any]]:
    root = ElementTree.parse(path).getroot()
    records: list[dict[str, Any]] = []
    for testcase in root.iter("testcase"):
        failure = testcase.find("failure")
        error = testcase.find("error")
        skipped = testcase.find("skipped")
        if failure is not None or error is not None:
            status = "failed"
        elif skipped is not None:
            status = "skipped"
        else:
            status = "passed"
        properties = {
            item.get("name", ""): item.get("value")
            for item in testcase.findall("./properties/property")
            if item.get("name")
        }
        records.append(
            {
                "profile": profile,
                "name": testcase.get("name"),
                "classname": testcase.get("classname"),
                "status": status,
                "ctest_status": testcase.get("status"),
                "elapsed_seconds": float(testcase.get("time", "0")),
                "properties": properties,
                "failure": element_details(failure if failure is not None else error),
                "skipped": element_details(skipped),
                "system_out": element_text(testcase.find("system-out")),
                "system_err": element_text(testcase.find("system-err")),
                "junit_path": str(path),
            }
        )
    return records


def element_text(element: ElementTree.Element | None) -> str | None:
    if element is None:
        return None
    text = "".join(element.itertext())
    return text if text else None


def element_details(element: ElementTree.Element | None) -> dict[str, Any] | None:
    if element is None:
        return None
    return {
        "message": element.get("message"),
        "type": element.get("type"),
        "text": element_text(element),
    }


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda value: (_ for _ in ()).throw(
            ValueError(f"non-finite number {value}")
        ),
    )
    if not isinstance(value, dict):
        raise RuntimeError(f"{path.name} is not a JSON object")
    return value


def load_jsonl(path: Path, record_type: str) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    previous_sequence = 0
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            value = json.loads(
                line,
                parse_constant=lambda constant: (_ for _ in ()).throw(
                    ValueError(f"non-finite number {constant}")
                ),
            )
            if not isinstance(value, dict):
                raise RuntimeError(f"{path.name}:{line_number} is not an object")
            if value.get("schema_version") != REPORT_VERSION:
                raise RuntimeError(f"{path.name}:{line_number} has wrong schema")
            if value.get("record_type") != record_type:
                raise RuntimeError(f"{path.name}:{line_number} has wrong record type")
            sequence = value.get("sequence")
            if not isinstance(sequence, int) or sequence <= previous_sequence:
                raise RuntimeError(
                    f"{path.name}:{line_number} sequence is not increasing"
                )
            previous_sequence = sequence
            records.append(value)
    return records


def validate_report_directory(directory: Path) -> dict[str, int]:
    manifest = load_json(directory / "manifest.json")
    if manifest.get("report_format_version") != REPORT_VERSION:
        raise RuntimeError("unsupported validation report version")
    summary = load_json(directory / "summary.json")
    environment = load_json(directory / "environment.json")
    if summary.get("record_type") != "validation_summary":
        raise RuntimeError("summary has wrong record type")
    if environment.get("record_type") != "validation_environment":
        raise RuntimeError("environment has wrong record type")
    if summary.get("schema_version") != REPORT_VERSION:
        raise RuntimeError("summary has wrong schema version")
    if environment.get("schema_version") != REPORT_VERSION:
        raise RuntimeError("environment has wrong schema version")
    if summary.get("status") not in {"completed", "failed", "interrupted"}:
        raise RuntimeError("summary does not have a terminal status")
    streams = {
        entry["record_type"]: load_jsonl(
            directory / entry["path"], entry["record_type"]
        )
        for entry in manifest["streams"]
    }
    counts = {
        "stages": len(streams["validation_stage"]),
        "ctest": len(streams["ctest_case"]),
        "fixtures": len(streams["fixture_case"]),
    }
    totals = summary.get("totals", {})
    for name, count in counts.items():
        reported = totals.get(name, {}).get(
            "completed" if name == "fixtures" else "total"
        )
        if reported != count:
            raise RuntimeError(
                f"summary {name} count is {reported}, stream contains {count}"
            )
    status_fields = {
        "stages": ("passed", "failed"),
        "ctest": ("passed", "failed", "skipped"),
        "fixtures": ("passed", "failed"),
    }
    for name, fields in status_fields.items():
        records = streams[
            {
                "stages": "validation_stage",
                "ctest": "ctest_case",
                "fixtures": "fixture_case",
            }[name]
        ]
        for status in fields:
            actual = sum(record.get("status") == status for record in records)
            if totals[name].get(status) != actual:
                raise RuntimeError(
                    f"summary {name}.{status} is {totals[name].get(status)}, "
                    f"stream contains {actual}"
                )
    fixtures = totals["fixtures"]
    if fixtures.get("planned", 0) < fixtures.get("completed", 0):
        raise RuntimeError("completed fixture count exceeds planned count")
    profiles = summary.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise RuntimeError("summary contains no profiles")
    profile_names = set(profiles)
    if set(environment.get("profiles", [])) != profile_names:
        raise RuntimeError("environment profile inventory disagrees with summary")
    for stream_name, records in streams.items():
        for record in records:
            if record.get("profile") not in profile_names:
                raise RuntimeError(
                    f"{stream_name} record has unknown profile: {record.get('profile')}"
                )
    for profile, profile_summary in profiles.items():
        profile_ctest = [
            record for record in streams["ctest_case"] if record["profile"] == profile
        ]
        profile_fixtures = [
            record for record in streams["fixture_case"] if record["profile"] == profile
        ]
        profile_stages = [
            record
            for record in streams["validation_stage"]
            if record["profile"] == profile
        ]
        require_status_counts(
            profile_summary["ctest"],
            profile_ctest,
            ("passed", "failed", "skipped"),
            f"profile {profile} CTest",
        )
        require_status_counts(
            profile_summary["fixtures"],
            profile_fixtures,
            ("passed", "failed"),
            f"profile {profile} fixtures",
            total_field="completed",
        )
        if len(profile_summary["stages"]) != len(profile_stages):
            raise RuntimeError(f"profile {profile} stage count disagrees")
        for stage in profile_stages:
            stage_summary = profile_summary["stages"].get(stage["stage"])
            if stage_summary is None or stage_summary["status"] != stage["status"]:
                raise RuntimeError(
                    f"profile {profile} stage summary disagrees: {stage['stage']}"
                )
        if len({record["name"] for record in profile_ctest}) != len(profile_ctest):
            raise RuntimeError(f"profile {profile} contains duplicate CTest names")
        if len({record["case_index"] for record in profile_fixtures}) != len(
            profile_fixtures
        ):
            raise RuntimeError(f"profile {profile} contains duplicate fixture indices")
        if len({record["case_id"] for record in profile_fixtures}) != len(
            profile_fixtures
        ):
            raise RuntimeError(f"profile {profile} contains duplicate fixture IDs")
    if summary["status"] == "completed":
        if totals["stages"]["failed"] or totals["ctest"]["failed"]:
            raise RuntimeError("completed report contains a failed stage or CTest")
        if fixtures["failed"] or fixtures["completed"] != fixtures["planned"]:
            raise RuntimeError("completed report has incomplete or failed fixtures")
    for stage in streams["validation_stage"]:
        log_path = directory / stage["log_path"]
        if not log_path.is_file():
            raise RuntimeError(f"missing stage log: {stage['log_path']}")
    for record in streams["ctest_case"]:
        junit_path = directory / record["junit_path"]
        if not junit_path.is_file():
            raise RuntimeError(f"missing CTest JUnit file: {record['junit_path']}")
    for record in streams["fixture_case"]:
        if record.get("status") == "passed":
            validation = record.get("validation", {})
            transport = validation.get("transport_validation", {})
            decode = validation.get("decode_report", {})
            stats = decode.get("stats", {})
            final_state = decode.get("source_session", {}).get("final_state", {})
            if validation.get("status") != "passed":
                raise RuntimeError(
                    f"passed fixture has failed validation: {record.get('case_id')}"
                )
            if not transport.get("exact_cyclic_match") or transport.get("tei_packets"):
                raise RuntimeError(
                    f"passed fixture has invalid TS result: {record.get('case_id')}"
                )
            if stats.get("status") != "completed" or stats.get("exit_code") != 0:
                raise RuntimeError(
                    f"passed fixture has invalid decode stats: {record.get('case_id')}"
                )
            for lock in (
                "ofdm_locked",
                "tps_locked",
                "rs_synchronized",
                "energy_synchronized",
            ):
                if final_state.get(lock) is not True:
                    raise RuntimeError(
                        f"passed fixture final state is not locked: "
                        f"{record.get('case_id')} {lock}"
                    )
        else:
            artifacts = record.get("artifacts")
            if not isinstance(artifacts, str) or not (directory / artifacts).is_dir():
                raise RuntimeError(
                    f"failed fixture has no retained artifacts: {record.get('case_id')}"
                )
    return counts


def require_status_counts(
    summary: dict[str, Any],
    records: list[dict[str, Any]],
    statuses: tuple[str, ...],
    location: str,
    total_field: str = "total",
) -> None:
    if summary.get(total_field) != len(records):
        raise RuntimeError(f"{location} count disagrees")
    for status in statuses:
        actual = sum(record.get("status") == status for record in records)
        if summary.get(status) != actual:
            raise RuntimeError(f"{location} {status} count disagrees")
