"""Load checksum-pinned, local-only external family holdouts."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from propulsion.benchmark import BenchmarkCase
from propulsion.data import parse_coefficient_measurements, parse_radial_geometry
from propulsion.models import (
    DataProvenance,
    GeometryProvenance,
    PolarProvenance,
    SourceConventions,
)


HOLDOUT_MANIFEST = Path(__file__).with_name("holdout_sources.json")


def _verify_source_file(path: Path, expected: dict) -> None:
    if not path.is_file():
        raise FileNotFoundError(
            f"holdout source file is missing: {path}; see {HOLDOUT_MANIFEST} for official download URLs"
        )
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if path.stat().st_size != expected["size_bytes"] or digest != expected["sha256"]:
        raise ValueError(f"holdout source file failed size/SHA-256 verification: {path}")


def load_graupner_cam_9x4_holdout(
    source_root: str | Path,
    *,
    manifest_path: str | Path = HOLDOUT_MANIFEST,
) -> tuple[tuple[BenchmarkCase, ...], dict]:
    """Load the fixed UIUC Volume 1 Graupner CAM 9×4 family holdout.

    The experiment tables are treated as untouched measurements; no file is
    rewritten and all original source hashes and row numbers are retained.
    """
    source = Path(source_root).resolve()
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    metadata = manifest["holdouts"]["graupner_cam_9x4"]
    files = metadata["sources"]
    for item in files.values():
        _verify_source_file(source / item["filename"], item)

    geometry_meta = files["geometry"]
    geometry_path = source / geometry_meta["filename"]
    diameter = float(metadata["diameter_m"])
    geometry = parse_radial_geometry(
        geometry_path,
        propeller_id="graupner_cam_9x4",
        diameter_m=diameter,
        hub_radius_m=0.0,
        blade_count=int(metadata["blade_count"]),
        geometry_provenance=GeometryProvenance.DIGITIZED,
        polar_provenance=PolarProvenance.ASSUMED,
        source_conventions=SourceConventions(
            rotor_axis="+X",
            blade_hand="UNKNOWN",
            section_sign="positive_pitch",
            frame="propeller_axis",
            units="r/R,c/R,beta_deg",
        ),
        source_uri=geometry_meta["url"],
    )

    measurement_files = (
        (files["static"], "static", None),
        (files["forward_4015rpm"], "forward", float(files["forward_4015rpm"]["rpm"])),
        (files["forward_5009rpm"], "forward", float(files["forward_5009rpm"]["rpm"])),
    )
    cases: list[BenchmarkCase] = []
    for item, regime, rpm_default in measurement_files:
        path = source / item["filename"]
        measurements = parse_coefficient_measurements(
            path,
            propeller_id="graupner_cam_9x4",
            regime=regime,
            provenance=DataProvenance.MEASURED,
            source_description=(
                "UIUC Volume 1 Graupner CAM Prop wind-tunnel record; source values preserved as published"
            ),
            rpm_default=rpm_default,
            source_uri=item["url"],
        )
        cases.extend(
            BenchmarkCase(
                propeller_id="graupner_cam_9x4",
                label="Graupner CAM 9×4",
                family="Graupner CAM Prop",
                measurement=measurement,
                geometry=geometry,
                diameter_source=metadata["diameter_basis"],
            )
            for measurement in measurements
        )
    if not cases:
        raise ValueError("Graupner CAM 9×4 holdout contains no measurement rows")
    return tuple(cases), metadata
