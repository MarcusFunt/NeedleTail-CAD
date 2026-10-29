from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
import zipfile
from pathlib import Path

from validation.verify_source_bundle import verify_source_bundle


class ValidationBundleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.archive = self.root / "bundle.zip"
        self.extracted = self.root / "source"
        self.extracted.mkdir()
        self.content = b"rpm,CT,CP\n6000,0.1,0.05\n"
        with zipfile.ZipFile(self.archive, "w") as bundle:
            bundle.writestr("suite/static.csv", self.content)
            bundle.writestr("plan.md", "supporting plan")
        (self.extracted / "suite").mkdir()
        (self.extracted / "suite/static.csv").write_bytes(self.content)
        self.manifest = self.root / "manifest.json"
        self.write_manifest()

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def write_manifest(self) -> None:
        self.manifest.write_text(json.dumps({
            "bundle_filename": self.archive.name,
            "bundle_size_bytes": self.archive.stat().st_size,
            "bundle_sha256": hashlib.sha256(self.archive.read_bytes()).hexdigest(),
            "source_member_count": 1,
            "source_roots": ["suite"],
            "source_members": [{
                "path": "suite/static.csv",
                "size_bytes": len(self.content),
                "sha256": hashlib.sha256(self.content).hexdigest(),
            }],
        }), encoding="utf-8")

    def test_verifies_archive_and_extracted_source_bytes(self):
        result = verify_source_bundle(self.archive, self.manifest, self.extracted)

        self.assertEqual(result["verified_source_members"], 1)
        self.assertEqual(result["bundle_sha256"], hashlib.sha256(self.archive.read_bytes()).hexdigest())

    def test_rejects_modified_extracted_source(self):
        (self.extracted / "suite/static.csv").write_text("modified", encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "extracted member hash mismatch"):
            verify_source_bundle(self.archive, self.manifest, self.extracted)

    def test_rejects_archive_hash_mismatch(self):
        manifest = json.loads(self.manifest.read_text(encoding="utf-8"))
        manifest["bundle_sha256"] = "0" * 64
        self.manifest.write_text(json.dumps(manifest), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "archive SHA-256 mismatch"):
            verify_source_bundle(self.archive, self.manifest)


if __name__ == "__main__":
    unittest.main()
