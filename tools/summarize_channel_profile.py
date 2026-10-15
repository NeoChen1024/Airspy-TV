#!/usr/bin/env python3
"""Aggregate `channel_profile` records of decode reports into channel profiles.

Each input is a decode report directory (or its `dvbt-channel.jsonl`). The
per-frame tap lists are re-referenced to the recording's dominant path, then
clustered by delay; every cluster that is present in enough frames becomes one
tap of the profile with its presence fraction, power spread and relative
Doppler. The output is a JSON array with one profile per input, in the
`airspy-tv-channel-profile/0` format consumed by `generate_dvbt_fixture.py`.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any

PROFILE_FORMAT = "airspy-tv-channel-profile/0"


def load_records(path: Path) -> list[dict[str, Any]]:
    if path.is_dir():
        path = path / "dvbt-channel.jsonl"
    if not path.is_file():
        raise SystemExit(f"missing channel profile stream: {path}")
    records = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get("record_type") != "channel_profile":
            raise SystemExit(f"{path}:{line_number}: not a channel_profile record")
        records.append(record)
    return records


def quantiles(values: list[float]) -> dict[str, float]:
    ordered = sorted(values)
    count = len(ordered)
    return {
        "median": statistics.median(ordered),
        "p10": ordered[count // 10],
        "p90": ordered[min(count - 1, (count * 9) // 10)],
    }


def reference_tap(taps: list[dict[str, Any]], margin_db: float) -> dict[str, Any]:
    """The frame's reference path: the earliest tap within `margin_db` of the
    strongest one, so a late echo that briefly outgrows the direct path does
    not flip the delay origin, and the FFT window anchor drifting between
    frames does not matter."""
    strongest = taps[0]["power_db"]
    candidates = [tap for tap in taps if tap["power_db"] >= strongest - margin_db]
    return min(candidates, key=lambda tap: tap["delay_us"])


def build_profile(
    name: str,
    records: list[dict[str, Any]],
    cluster_width_us: float,
    min_presence: float,
    reference_margin_db: float,
) -> dict[str, Any]:
    frames = [record for record in records if record["taps"]]
    if not frames:
        raise SystemExit(f"{name}: no frames with taps")
    first = frames[0]

    # Relative taps: delay and power against the frame's reference path,
    # Doppler against the reference path's Doppler.
    relative: list[tuple[float, float, float | None, float | None, int]] = []
    reference_delays = []
    reference_not_strongest = 0
    for index, frame in enumerate(frames):
        reference = reference_tap(frame["taps"], reference_margin_db)
        reference_delays.append(reference["delay_us"])
        if reference is not frame["taps"][0]:
            reference_not_strongest += 1
        for tap in frame["taps"]:
            doppler = None
            if tap["doppler_hz"] is not None and reference["doppler_hz"] is not None:
                doppler = tap["doppler_hz"] - reference["doppler_hz"]
            relative.append(
                (
                    tap["delay_us"] - reference["delay_us"],
                    tap["power_db"] - reference["power_db"],
                    doppler,
                    tap["coherence"],
                    index,
                )
            )

    relative.sort(key=lambda item: item[1], reverse=True)
    clusters: list[list[tuple[float, float, float | None, float | None, int]]] = []
    for item in relative:
        for cluster in clusters:
            if abs(item[0] - cluster[0][0]) <= cluster_width_us:
                cluster.append(item)
                break
        else:
            clusters.append([item])
    # Seeds that landed one or two bins apart describe the same path whose
    # delay wandered between frames; merge clusters whose medians are close.
    clusters.sort(key=lambda cluster: statistics.median(item[0] for item in cluster))
    merged: list[list[tuple[float, float, float | None, float | None, int]]] = []
    for cluster in clusters:
        if merged and abs(
            statistics.median(item[0] for item in cluster)
            - statistics.median(item[0] for item in merged[-1])
        ) <= cluster_width_us:
            merged[-1].extend(cluster)
        else:
            merged.append(cluster)
    clusters = merged

    taps = []
    for cluster in clusters:
        frame_indices = {item[4] for item in cluster}
        presence = len(frame_indices) / len(frames)
        if presence < min_presence:
            continue
        powers = [item[1] for item in cluster]
        dopplers = [item[2] for item in cluster if item[2] is not None]
        coherences = [item[3] for item in cluster if item[3] is not None]
        tap = {
            "delay_us": round(statistics.median(item[0] for item in cluster), 3),
            "power_db": round(statistics.median(powers), 2),
            "power_db_p10": round(quantiles(powers)["p10"], 2),
            "power_db_p90": round(quantiles(powers)["p90"], 2),
            "presence": round(presence, 3),
            "doppler_hz": round(statistics.median(dopplers), 3) if dopplers else None,
            "coherence": round(statistics.median(coherences), 4) if coherences else None,
        }
        taps.append(tap)
    taps.sort(key=lambda tap: tap["power_db"], reverse=True)

    def stat(key: str) -> dict[str, float]:
        return {k: round(v, 3) for k, v in quantiles([frame[key] for frame in frames]).items()}

    return {
        "format": PROFILE_FORMAT,
        "name": name,
        "source": {
            "frames": len(frames),
            "records": len(records),
            "delay_window_us": first["delay_window_us"],
            "delay_resolution_us": first["delay_resolution_us"],
            "response_interval_us": first["response_interval_us"],
            "reference_delay_us": round(statistics.median(reference_delays), 3),
            "reference_not_strongest_frames": reference_not_strongest,
        },
        "statistics": {
            "rms_delay_spread_us": stat("rms_delay_spread_us"),
            "max_excess_delay_us": stat("max_excess_delay_us"),
            "main_tap_energy_share": stat("main_tap_energy_share"),
            "noise_floor_db": stat("noise_floor_db"),
        },
        "taps": taps,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Aggregate channel_profile records of decode reports into channel profiles."
    )
    parser.add_argument("inputs", nargs="+", type=Path, help="report directories or dvbt-channel.jsonl files")
    parser.add_argument("--output", type=Path, help="write the JSON array here instead of stdout")
    parser.add_argument(
        "--cluster-width-us",
        type=float,
        default=0.3,
        help="taps closer than this (relative delay) join one cluster (default: 0.3)",
    )
    parser.add_argument(
        "--min-presence",
        type=float,
        default=0.2,
        help="drop clusters seen in fewer than this fraction of frames (default: 0.2)",
    )
    parser.add_argument(
        "--reference-margin-db",
        type=float,
        default=3.0,
        help="the delay origin is the earliest tap within this of the strongest (default: 3)",
    )
    args = parser.parse_args()

    profiles = []
    for path in args.inputs:
        name = path.name if path.is_dir() else path.parent.name
        profiles.append(
            build_profile(
                name,
                load_records(path),
                args.cluster_width_us,
                args.min_presence,
                args.reference_margin_db,
            )
        )
    text = json.dumps(profiles, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
