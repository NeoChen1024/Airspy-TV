"""Execute real-signal corpus cases and write bounded validation records."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any

from real_signal_corpus import RealSignalCase, RealSignalCorpus
from validation_report import ValidationReport, utc_now

TOOLS_DIRECTORY = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_DIRECTORY))
from validate_decode_report import validate_real_decode_report  # noqa: E402


def run_real_signal(
    root: Path,
    report: ValidationReport,
    profile: str,
    case: RealSignalCase,
    channel_bandwidth_hz: int,
    decoder_threads: int,
    environment: dict[str, str],
) -> dict[str, Any]:
    artifact_root = (
        report.failures_dir / "real-signals" / profile / f"case-{case.case_index:03d}"
    )
    artifact_root.mkdir(parents=True, exist_ok=False)
    decode_report_dir = artifact_root / "decode-report"
    runner_log = artifact_root / "runner.log"
    timeout = max(300.0, case.duration_seconds * 10.0)
    command = [
        str(root / "build" / profile / "airspy-tv"),
        "--decode-iq",
        str(case.input_path),
        "--sample-rate",
        str(case.sample_rate_hz),
        "--ts-output",
        os.devnull,
        "--report-dir",
        str(decode_report_dir),
        "--dvbt-channel-bandwidth",
        f"{channel_bandwidth_hz // 1_000_000}M",
        "--decoder-threads",
        str(decoder_threads),
    ]
    started_at = utc_now()
    started = time.monotonic()
    return_code = 127
    execution_error: dict[str, str] | None = None
    with runner_log.open("w", encoding="utf-8") as output:
        try:
            completed = subprocess.run(
                command,
                cwd=root,
                env=environment,
                text=True,
                stdout=output,
                stderr=subprocess.STDOUT,
                timeout=timeout,
            )
            return_code = completed.returncode
        except subprocess.TimeoutExpired:
            return_code = 124
            execution_error = {
                "type": "TimeoutExpired",
                "message": f"decode exceeded {timeout:g} seconds",
            }
        except OSError as exception:
            execution_error = {
                "type": type(exception).__name__,
                "message": str(exception),
            }

    validation: dict[str, Any] | None = None
    decode_outcome = "failed"
    if execution_error is None and return_code in {0, 2}:
        try:
            parsed = validate_real_decode_report(
                decode_report_dir,
                expected_samples=case.complex_samples,
                expected_sample_rate=case.sample_rate_hz,
                expected_bandwidth_hz=channel_bandwidth_hz,
            )
            validation = {
                "manifest": parsed["manifest"],
                "record_counts": parsed["stream_records"],
                "stats": parsed["stats"],
                "source_session": parsed["source_session"],
            }
            decode_outcome = parsed["stats"]["status"]
            expected_return_code = 0 if decode_outcome == "completed" else 2
            if return_code != expected_return_code:
                raise RuntimeError(
                    f"decoder returned {return_code}, report requires "
                    f"{expected_return_code}"
                )
        except Exception as exception:
            execution_error = {
                "type": type(exception).__name__,
                "message": str(exception),
            }
    elif execution_error is None:
        execution_error = {
            "type": "DecoderFailure",
            "message": f"decoder exited with status {return_code}",
        }

    execution_status = "completed" if execution_error is None else "failed"
    record: dict[str, Any] = {
        "profile": profile,
        "case_index": case.case_index,
        "case_id": case.case_id,
        "case": {
            "input_path": str(case.input_path),
            "data_path": str(case.data_path),
            "metadata_path": (
                str(case.metadata_path) if case.metadata_path is not None else None
            ),
            "sample_rate_hz": case.sample_rate_hz,
            "center_frequency_hz": case.center_frequency_hz,
            "channel_bandwidth_hz": channel_bandwidth_hz,
            "file_size_bytes": case.file_size_bytes,
            "modified_time_ns": case.modified_time_ns,
            "complex_samples": case.complex_samples,
            "signal_seconds": case.duration_seconds,
        },
        "execution_status": execution_status,
        "decode_outcome": decode_outcome,
        "regression_status": "not_evaluated",
        "return_code": return_code,
        "started_at": started_at,
        "finished_at": utc_now(),
        "elapsed_seconds": time.monotonic() - started,
        "timeout_seconds": timeout,
        "command": command,
    }
    if validation is not None:
        record["decode_report"] = validation
    if execution_error is None:
        shutil.rmtree(artifact_root)
    else:
        record["error"] = execution_error
        record["artifacts"] = str(artifact_root.relative_to(report.directory))
        record["output_tail"] = runner_log.read_text(
            encoding="utf-8", errors="replace"
        )[-4000:]
    return record


def update_real_signal_summary(
    summary: dict[str, Any], profile: str, record: dict[str, Any]
) -> None:
    profile_counts = summary["profiles"][profile]["real_signals"]
    total_counts = summary["totals"]["real_signals"]
    for counts in (profile_counts, total_counts):
        if record["execution_status"] == "completed":
            counts["completed"] += 1
            outcome_field = (
                "decode_completed"
                if record["decode_outcome"] == "completed"
                else record["decode_outcome"]
            )
            counts[outcome_field] += 1
        else:
            counts["failed"] += 1
        counts[record["regression_status"]] += 1


def run_real_signal_matrix(
    root: Path,
    report: ValidationReport,
    summary: dict[str, Any],
    corpus: RealSignalCorpus,
    profile: str,
    real_jobs: int,
    decoder_threads: int,
    environment: dict[str, str],
) -> int:
    cases = corpus.selected_cases
    failures = 0
    print(
        f"[{profile}] {len(cases)} real signals, parallel={real_jobs}, "
        f"bandwidth={corpus.channel_bandwidth_hz // 1_000_000}M",
        flush=True,
    )
    with ThreadPoolExecutor(max_workers=real_jobs) as executor:
        futures = {
            executor.submit(
                run_real_signal,
                root,
                report,
                profile,
                case,
                corpus.channel_bandwidth_hz,
                decoder_threads,
                environment,
            ): case
            for case in cases
        }
        for completed_count, future in enumerate(as_completed(futures), start=1):
            case = futures[future]
            try:
                record = future.result()
            except Exception as exception:
                artifact_root = (
                    report.failures_dir
                    / "real-signals"
                    / profile
                    / f"case-{case.case_index:03d}"
                )
                artifact_root.mkdir(parents=True, exist_ok=True)
                (artifact_root / "runner.log").write_text(
                    f"real-signal worker failed: "
                    f"{type(exception).__name__}: {exception}\n",
                    encoding="utf-8",
                )
                record = {
                    "profile": profile,
                    "case_index": case.case_index,
                    "case_id": case.case_id,
                    "execution_status": "failed",
                    "decode_outcome": "failed",
                    "regression_status": "not_evaluated",
                    "return_code": 1,
                    "error": {
                        "type": type(exception).__name__,
                        "message": str(exception),
                    },
                    "artifacts": str(artifact_root.relative_to(report.directory)),
                }
            report.write_real_signal(record)
            update_real_signal_summary(summary, profile, record)
            report.write_summary(summary)
            failures += record["execution_status"] == "failed"
            marker = (
                "FAIL"
                if record["execution_status"] == "failed"
                else "NO_TS" if record["decode_outcome"] == "no_transport" else "PASS"
            )
            print(
                f"[{profile}] {completed_count:3}/{len(cases)} {marker} "
                f"{case.case_id} ({record.get('elapsed_seconds', 0.0):.2f}s)",
                flush=True,
            )
    return failures
