from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from propulsion.models import DataProvenance
from validation.holdouts import load_graupner_cam_9x4_holdout


class ValidationHoldoutTests(unittest.TestCase):
    def _fixture(self, root: Path) -> tuple[Path, Path]:
        contents = {
            "geometry.txt": "r/R c/R beta\n0.2 0.16 30\n0.6 0.12 12\n1.0 0.03 6\n",
            "static.txt": "RPM CT CP\n3000 0.05 0.03\n4000 0.06 0.03\n",
            "forward4015.txt": "J CT CP eta\n0.2 0.04 0.025 0.32\n0.5 -0.0025 0.0103 -0.127\n",
            "forward5009.txt": "J CT CP eta\n0.2 0.045 0.026 0.35\n0.5 0.0011 0.0115 0.049\n",
        }
        sources = {}
        for key, filename in (
            ("geometry", "geometry.txt"),
            ("static", "static.txt"),
            ("forward_4015rpm", "forward4015.txt"),
            ("forward_5009rpm", "forward5009.txt"),
        ):
            payload = contents[filename].encode("utf-8")
            (root / filename).write_bytes(payload)
            sources[key] = {
                "filename": filename,
                "url": f"https://example.invalid/{filename}",
                "size_bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "rpm": 4015 if key == "forward_4015rpm" else 5009 if key == "forward_5009rpm" else None,
            }
        metadata = {
            "manufacturer_family": "Graupner CAM Prop",
            "diameter_m": 0.2286,
            "diameter_basis": "nominal 9 in fixture",
            "blade_count": 2,
            "sources": sources,
        }
        manifest = root / "manifest.json"
        manifest.write_text(json.dumps({"holdouts": {"graupner_cam_9x4": metadata}}), encoding="utf-8")
        return manifest, root

    def test_loader_keeps_all_holdout_rows_and_source_provenance(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest, source_root = self._fixture(Path(temp))
            cases, metadata = load_graupner_cam_9x4_holdout(source_root, manifest_path=manifest)

        self.assertEqual(len(cases), 6)
        self.assertEqual({case.family for case in cases}, {"Graupner CAM Prop"})
        negative = next(case for case in cases if case.measurement.ct < 0)
        self.assertEqual(negative.measurement.provenance, DataProvenance.MEASURED)
        self.assertEqual(negative.measurement.rpm, 4015)
        self.assertTrue(negative.measurement.source_uri.endswith("forward4015.txt"))
        self.assertEqual(metadata["manufacturer_family"], "Graupner CAM Prop")

    def test_loader_rejects_changed_holdout_input(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest, source_root = self._fixture(Path(temp))
            (source_root / "static.txt").write_text("RPM CT CP\n3000 0.1 0.1\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "failed size/SHA-256"):
                load_graupner_cam_9x4_holdout(source_root, manifest_path=manifest)


if __name__ == "__main__":
    unittest.main()
