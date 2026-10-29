"""Provenance-preserving readers for coefficient and radial-geometry tables."""
from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path
import re
import shlex

from propulsion.models import (
    BladeStation,
    DataProvenance,
    GeometryProvenance,
    MeasurementRecord,
    PolarProvenance,
    PropellerGeometry,
    SourceConventions,
)


def _normalized_header(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", value.strip().lower())


def _table(path: Path) -> tuple[list[str], list[tuple[int, dict[str, str]]]]:
    """Read one CSV or whitespace table, preserving physical source line numbers."""
    text = path.read_text(encoding="utf-8-sig")
    lines = [(i, line.strip()) for i, line in enumerate(text.splitlines(), start=1)
             if line.strip() and not line.lstrip().startswith("#")]
    if not lines:
        raise ValueError(f"{path}: table is empty")

    is_csv = path.suffix.lower() == ".csv"
    first_line = lines[0][1]
    if is_csv:
        header = next(csv.reader([first_line]))
        rows = []
        for line_number, line in lines[1:]:
            cells = next(csv.reader([line]))
            if len(cells) != len(header):
                raise ValueError(
                    f"{path}:{line_number}: expected {len(header)} columns, found {len(cells)}"
                )
            rows.append((line_number, dict(zip(header, cells))))
    else:
        header = shlex.split(first_line)
        rows = []
        for line_number, line in lines[1:]:
            cells = shlex.split(line)
            if len(cells) != len(header):
                raise ValueError(
                    f"{path}:{line_number}: expected {len(header)} columns, found {len(cells)}"
                )
            rows.append((line_number, dict(zip(header, cells))))
    if not rows:
        raise ValueError(f"{path}: table has a header but no data rows")
    return header, rows


def _find_column(header: list[str], names: tuple[str, ...]) -> str | None:
    aliases = {_normalized_header(name) for name in names}
    return next((name for name in header if _normalized_header(name) in aliases), None)


def _number(row: dict[str, str], column: str, path: Path, line_number: int) -> float:
    raw = row.get(column, "").strip()
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}:{line_number}: {column} must contain a number, found {raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{path}:{line_number}: {column} must be finite")
    return value


def parse_coefficient_measurements(
    path: str | Path,
    *,
    propeller_id: str,
    regime: str,
    provenance: DataProvenance,
    source_description: str,
    rpm_default: float | None = None,
    source_uri: str | None = None,
) -> tuple[MeasurementRecord, ...]:
    """Parse measured/transcribed CT and CP rows without changing their values."""
    source = Path(path)
    if regime not in {"static", "forward"}:
        raise ValueError("regime must be 'static' or 'forward'")
    header, rows = _table(source)
    rpm_column = _find_column(header, ("rpm", "rotation_rpm"))
    j_column = _find_column(header, ("j", "advance_ratio"))
    ct_column = _find_column(header, ("ct", "ct_measured", "ct_experimental"))
    cp_column = _find_column(header, ("cp", "cp_measured", "cp_experimental"))
    eta_column = _find_column(header, ("eta", "efficiency"))
    if ct_column is None:
        raise ValueError(f"{source}: required CT coefficient column is missing")
    if cp_column is None:
        raise ValueError(f"{source}: required CP coefficient column is missing")
    if regime == "forward" and j_column is None:
        raise ValueError(f"{source}: forward table requires an advance-ratio J column")
    if rpm_column is None and rpm_default is None:
        raise ValueError(f"{source}: RPM column or explicit rpm_default is required")

    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    source_name = source_uri if source_uri is not None else source.as_posix()
    result = []
    for line_number, row in rows:
        rpm = _number(row, rpm_column, source, line_number) if rpm_column else float(rpm_default)
        advance_ratio = _number(row, j_column, source, line_number) if regime == "forward" else 0.0
        ct = _number(row, ct_column, source, line_number)
        cp = _number(row, cp_column, source, line_number)
        eta = _number(row, eta_column, source, line_number) if eta_column and row[eta_column].strip() else None
        result.append(MeasurementRecord(
            propeller_id=propeller_id,
            regime=regime,
            rpm=rpm,
            advance_ratio=advance_ratio,
            ct=ct,
            cp=cp,
            eta=eta,
            provenance=provenance,
            source_description=source_description,
            source_uri=source_name,
            source_sha256=digest,
            source_row=line_number,
        ))
    return tuple(result)


def parse_radial_geometry(
    path: str | Path,
    *,
    propeller_id: str,
    diameter_m: float,
    hub_radius_m: float,
    blade_count: int,
    geometry_provenance: GeometryProvenance,
    polar_provenance: PolarProvenance,
    source_conventions: SourceConventions,
    source_uri: str | None = None,
) -> PropellerGeometry:
    """Convert r/R, c/R and beta tables into a predictive SI geometry record."""
    source = Path(path)
    header, rows = _table(source)
    radius_ratio = _find_column(header, ("r_over_R", "r_R", "r/R"))
    chord_ratio = _find_column(header, ("c_over_R", "c_R", "c/R"))
    radius_m = _find_column(header, ("radius_m",))
    chord_m = _find_column(header, ("chord_m",))
    twist_rad = _find_column(header, ("twist_rad",))
    twist_deg = _find_column(header, ("beta_deg", "beta", "twist_deg"))
    if radius_ratio is None and radius_m is None:
        raise ValueError(f"{source}: radial ratio r/R or radius_m column is required")
    if chord_ratio is None and chord_m is None:
        raise ValueError(f"{source}: chord ratio c/R or chord_m column is required")
    if twist_rad is None and twist_deg is None:
        raise ValueError(f"{source}: beta_deg or twist_rad column is required")

    radius = diameter_m / 2
    stations = []
    for line_number, row in rows:
        r_value = _number(row, radius_ratio or radius_m, source, line_number)
        c_value = _number(row, chord_ratio or chord_m, source, line_number)
        angle = _number(row, twist_rad or twist_deg, source, line_number)
        station_radius = r_value * radius if radius_ratio else r_value
        station_chord = c_value * radius if chord_ratio else c_value
        angle_rad = math.radians(angle) if twist_deg else angle
        if radius_ratio and not (0 < r_value <= 1):
            raise ValueError(f"{source}:{line_number}: r/R must be in (0, 1]")
        if chord_ratio and c_value <= 0:
            raise ValueError(f"{source}:{line_number}: c/R must be positive")
        stations.append(BladeStation(station_radius, station_chord, angle_rad))
    if len(stations) < 2:
        raise ValueError(f"{source}: predictive radial geometry needs at least two stations")

    return PropellerGeometry(
        id=propeller_id,
        source_uri=source_uri if source_uri is not None else source.as_posix(),
        sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        blade_count=blade_count,
        diameter_m=diameter_m,
        hub_radius_m=hub_radius_m,
        stations=tuple(stations),
        geometry_provenance=geometry_provenance,
        polar_provenance=polar_provenance,
        source_conventions=source_conventions,
    )
