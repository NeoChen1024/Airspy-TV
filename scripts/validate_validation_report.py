#!/usr/bin/env python3

"""Validate a repository validation report directory."""

from __future__ import annotations

import argparse
from pathlib import Path

from validation_report import validate_report_directory


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report_dir", type=Path)
    args = parser.parse_args()
    counts = validate_report_directory(args.report_dir.resolve())
    print(
        f"valid validation report: {counts['stages']} stages, "
        f"{counts['ctest']} CTest cases, {counts['fixtures']} fixtures"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
