"""Fetch or install the checksum-pinned, MIT-licensed qprop.c portable release."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import urllib.request
import zipfile


LOCK_PATH = Path(__file__).with_name("qpropc_lock.json")


def verify_archive_sha256(data: bytes, expected: str) -> str:
    actual = hashlib.sha256(data).hexdigest()
    if actual.lower() != expected.lower():
        raise ValueError(f"release archive SHA-256 mismatch: expected {expected}, found {actual}")
    return actual


def install_portable(
    archive_path: str | Path,
    destination: str | Path,
    *,
    expected_sha256: str,
    release_tag: str,
    source_commit: str,
) -> dict[str, str]:
    archive = Path(archive_path)
    dest = Path(destination)
    if dest.exists():
        raise FileExistsError(f"refusing to overwrite existing qprop.c destination: {dest}")
    payload = archive.read_bytes()
    digest = verify_archive_sha256(payload, expected_sha256)
    with zipfile.ZipFile(archive) as bundle:
        candidates = {}
        for info in bundle.infolist():
            name = PurePosixPath(info.filename)
            if name.is_absolute() or ".." in name.parts or "\\" in info.filename:
                raise ValueError(f"unsafe path in qprop.c release archive: {info.filename}")
            if not info.is_dir() and len(name.parts) == 2 and name.parts[0] == "qprop-portable":
                candidates[name.name] = info
        required = {"qprop.py", "LICENSE"}
        if not required.issubset(candidates):
            raise ValueError("pinned qprop.c archive is missing its Python binding or MIT license")
        libraries = {
            "qprop-lib-windows-x64.dll",
            "qprop-lib-linux-x64.so",
            "qprop-lib-macos-arm64.dylib",
        }
        if not libraries.intersection(candidates):
            raise ValueError("pinned qprop.c archive contains no supported native library")
        dest.mkdir(parents=True)
        installed_file_hashes = {}
        for name, info in candidates.items():
            content = bundle.read(info)
            (dest / name).write_bytes(content)
            installed_file_hashes[name] = hashlib.sha256(content).hexdigest()
    installed = {
        "release_tag": release_tag,
        "source_commit": source_commit,
        "archive_sha256": digest,
        "license": "MIT",
        "files": installed_file_hashes,
    }
    (dest / "release.json").write_text(json.dumps(installed, indent=2) + "\n", encoding="utf-8")
    return installed


def main() -> None:
    lock = json.loads(LOCK_PATH.read_text(encoding="utf-8"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dest", required=True, type=Path)
    parser.add_argument("--archive", type=Path, help="use a previously downloaded release ZIP")
    args = parser.parse_args()
    if args.archive is None:
        request = urllib.request.Request(
            lock["release_url"], headers={"User-Agent": "NeedleTail-qprop-validation"}
        )
        with urllib.request.urlopen(request, timeout=60) as response:
            payload = response.read()
        digest = verify_archive_sha256(payload, lock["portable_sha256"])
        temporary = args.dest.with_name(args.dest.name + ".download.zip")
        temporary.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_bytes(payload)
        archive = temporary
    else:
        archive = args.archive
        digest = None
    result = install_portable(
        archive,
        args.dest,
        expected_sha256=lock["portable_sha256"],
        release_tag=lock["source_ref"],
        source_commit=lock["source_commit"],
    )
    if digest is not None and result["archive_sha256"] != digest:
        raise RuntimeError("downloaded qprop.c release digest changed during installation")
    print(json.dumps({"destination": str(args.dest.resolve()), **result}, indent=2))


if __name__ == "__main__":
    main()
