#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from real_signal_corpus import CorpusError, discover_real_signal_corpus  # noqa: E402


class RealSignalCorpusTest(unittest.TestCase):
    def test_discovers_sidecars_bare_files_and_ignored_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "paired.cs16").write_bytes(bytes(16))
            (root / "paired.cs16.json").write_text(
                json.dumps(
                    {
                        "data_file": "paired.cs16",
                        "datatype": "ci16_le",
                        "iq_order": "IQ",
                        "sample_rate": 4,
                        "center_frequency": 545_000_000,
                        "complex_samples": 4,
                    }
                ),
                encoding="utf-8",
            )
            (root / "bare.cs16").write_bytes(bytes(32))
            (root / "old.ts").write_bytes(bytes(188))
            (root / "debug.log").write_text("ignored\n", encoding="utf-8")

            corpus = discover_real_signal_corpus(
                root,
                fallback_sample_rate_hz=8,
                long_recording_threshold_seconds=0.75,
            )

            self.assertEqual(
                [case.case_id for case in corpus.cases],
                [
                    "bare.cs16",
                    "paired.cs16",
                ],
            )
            bare, paired = corpus.cases
            self.assertEqual(bare.sample_rate_hz, 8)
            self.assertIsNone(bare.metadata_path)
            self.assertFalse(bare.selected)
            self.assertEqual(bare.exclusion_reason, "long_recording")
            self.assertEqual(paired.sample_rate_hz, 4)
            self.assertEqual(paired.center_frequency_hz, 545_000_000)
            self.assertEqual(paired.input_path.name, "paired.cs16.json")
            self.assertFalse(paired.selected)
            self.assertEqual(
                [item["path"] for item in corpus.ignored_files],
                ["debug.log", "old.ts"],
            )

    def test_rejects_duplicate_sidecar_claims(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "signal.cs16").write_bytes(bytes(4))
            metadata = {
                "data_file": "signal.cs16",
                "datatype": "ci16_le",
                "iq_order": "IQ",
                "sample_rate": 10_000_000,
            }
            for name in ("one.json", "two.json"):
                (root / name).write_text(json.dumps(metadata), encoding="utf-8")
            with self.assertRaisesRegex(CorpusError, "claim the same recording"):
                discover_real_signal_corpus(root)

    def test_rejects_malformed_recognized_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "broken.json").write_text("{", encoding="utf-8")
            with self.assertRaisesRegex(CorpusError, "unable to parse metadata"):
                discover_real_signal_corpus(root)

    def test_rejects_sidecar_path_traversal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "bad.json").write_text(
                json.dumps(
                    {
                        "data_file": "../signal.cs16",
                        "datatype": "ci16_le",
                        "iq_order": "IQ",
                        "sample_rate": 10_000_000,
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(CorpusError, "same-directory"):
                discover_real_signal_corpus(root)


if __name__ == "__main__":
    unittest.main()
