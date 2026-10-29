"""Verify an immutable source ZIP and optional extracted copy against its manifest."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
import zipfile


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_member_path(value: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if not value or path.is_absolute() or ".." in path.parts or "\\" in value:
        raise ValueError(f"unsafe source member path: {value!r}")
    return path


def verify_source_bundle(
    archive_path: str | Path,
    manifest_path: str | Path,
    extracted_root: str | Path | None = None,
) -> dict[str, object]:
    """Check archive and extracted source members without modifying either."""
    archive = Path(archive_path)
    manifest_file = Path(manifest_path)
    manifest = json.loads(manifest_file.read_text(encoding="utf-8-sig"))
    archive_bytes = archive.read_bytes()
    archive_digest = _sha256(archive_bytes)
    if archive.stat().st_size != manifest.get("bundle_size_bytes"):
        raise ValueError("archive size does not match source manifest")
    if archive_digest.lower() != str(manifest.get("bundle_sha256", "")).lower():
        raise ValueError("archive SHA-256 mismatch")

    expected = manifest.get("source_members")
    if not isinstance(expected, list) or len(expected) != manifest.get("source_member_count"):
        raise ValueError("source member list/count is invalid")
    expected_by_path = {}
    roots = set(manifest.get("source_roots", []))
    for member in expected:
        member_path = _safe_member_path(member["path"])
        if not member_path.parts or member_path.parts[0] not in roots:
            raise ValueError(f"source member is outside declared roots: {member['path']}")
        if member["path"] in expected_by_path:
            raise ValueError(f"duplicate source manifest path: {member['path']}")
        expected_by_path[member["path"]] = member

    with zipfile.ZipFile(archive) as bundle:
        names = [entry.filename for entry in bundle.infolist()]
        if len(names) != len(set(names)):
            raise ValueError("archive contains duplicate member names")
        if bundle.testzip() is not None:
            raise ValueError("archive CRC validation failed")
        for entry in bundle.infolist():
            if entry.is_dir():
                continue
            safe_name = _safe_member_path(entry.filename)
            if safe_name.suffix.lower() == ".zip":
                raise ValueError(f"nested archive is not expected: {entry.filename}")
            unix_mode = (entry.external_attr >> 16) & 0xFFFF
            if unix_mode and stat.S_ISLNK(unix_mode):
                raise ValueError(f"symbolic link member is not allowed: {entry.filename}")
        missing = expected_by_path.keys() - set(names)
        if missing:
            raise ValueError(f"source member missing from archive: {sorted(missing)[0]}")
        for name, member in expected_by_path.items():
            content = bundle.read(name)
            if len(content) != member["size_bytes"] or _sha256(content).lower() != member["sha256"].lower():
                raise ValueError(f"archive member hash/size mismatch: {name}")

    if extracted_root is not None:
        root = Path(extracted_root).resolve()
        for name, member in expected_by_path.items():
            local_path = root.joinpath(*PurePosixPath(name).parts).resolve()
            if root not in local_path.parents:
                raise ValueError(f"extracted path escapes source root: {name}")
            if not local_path.is_file():
                raise ValueError(f"extracted member missing: {name}")
            content = local_path.read_bytes()
            if len(content) != member["size_bytes"] or _sha256(content).lower() != member["sha256"].lower():
                raise ValueError(f"extracted member hash mismatch: {name}")

    return {
        "bundle_filename": archive.name,
        "bundle_size_bytes": len(archive_bytes),
        "bundle_sha256": archive_digest,
        "verified_source_members": len(expected_by_path),
        "extracted_copy_verified": extracted_root is not None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--extracted-root", type=Path)
    args = parser.parse_args()
    print(json.dumps(verify_source_bundle(args.archive, args.manifest, args.extracted_root), indent=2))


if __name__ == "__main__":
    main()
