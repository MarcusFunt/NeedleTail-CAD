"""Coordinate catalog loaded from vendored UIUC Selig-format airfoil files."""

from __future__ import annotations

from functools import lru_cache
from hashlib import sha256
from pathlib import Path


PROFILE_DIRECTORY = Path(__file__).with_name("original")
UIUC_PROFILE_URL = "https://m-selig.ae.illinois.edu/ads/coord_seligFmt/"


def _read_profile(path: Path) -> tuple[tuple[float, float], ...]:
    """Read a Selig .dat coordinate file, ignoring its title and comments."""
    points = []
    for line in path.read_text(encoding="ascii").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.replace(",", " ").split()
        if len(fields) < 2:
            continue
        try:
            x, y = float(fields[0]), float(fields[1])
        except ValueError:
            continue
        points.append((x, y))
    if len(points) < 4:
        raise ValueError(f"Airfoil file {path} contains fewer than four coordinates")
    return tuple(points)


@lru_cache(maxsize=1)
def load_airfoil_library() -> dict[str, tuple[tuple[float, float], ...]]:
    """Return all vendored .dat profiles, keyed by uppercase file stem."""
    profiles = {
        path.stem.upper(): _read_profile(path)
        for path in sorted(PROFILE_DIRECTORY.glob("*.dat"))
    }
    if not profiles:
        raise FileNotFoundError(f"No airfoil .dat files found in {PROFILE_DIRECTORY}")
    return profiles


AIRFOIL_COORDINATES = load_airfoil_library()
AIRFOIL_CATALOG = {
    name: {
        "file": f"{name.lower()}.dat",
        "source_url": f"{UIUC_PROFILE_URL}{name.lower()}.dat",
        "sha256": sha256(
            (PROFILE_DIRECTORY / f"{name.lower()}.dat").read_bytes()
        ).hexdigest(),
    }
    for name in AIRFOIL_COORDINATES
}

# Backwards-compatible coordinate names used by existing scripts.
SD7003_COORDINATES = AIRFOIL_COORDINATES["SD7003"]
SD7037_COORDINATES = AIRFOIL_COORDINATES["SD7037"]
SD8020_COORDINATES = AIRFOIL_COORDINATES["SD8020"]
E423_COORDINATES = AIRFOIL_COORDINATES["E423"]
NACA0010_COORDINATES = AIRFOIL_COORDINATES["NACA0010"]
NACA0015_COORDINATES = AIRFOIL_COORDINATES["NACA0015"]
NACA2412_COORDINATES = AIRFOIL_COORDINATES["NACA2412"]
NACA2415_COORDINATES = AIRFOIL_COORDINATES["NACA2415"]
NACA4412_COORDINATES = AIRFOIL_COORDINATES["NACA4412"]

# Preserve the former source metadata names for the two original sections.
PROFILE_SOURCES = {
    name: (metadata["source_url"], metadata["sha256"])
    for name, metadata in AIRFOIL_CATALOG.items()
}
SD7003_SOURCE_URL, SD7003_SOURCE_SHA256 = PROFILE_SOURCES["SD7003"]
SD8020_SOURCE_URL, SD8020_SOURCE_SHA256 = PROFILE_SOURCES["SD8020"]
