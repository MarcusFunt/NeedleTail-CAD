"""Recompute qprop.c's four published validation categories from raw source rows."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys

from propulsion.benchmark import BenchmarkCase, run_benchmark
from propulsion.data import parse_coefficient_measurements, parse_radial_geometry
from propulsion.models import (
    BladeStation,
    DataProvenance,
    GeometryProvenance,
    PolarProvenance,
    PropellerGeometry,
    SourceConventions,
)
from propulsion.qpropc import QPROPC_REVISION, QPropCBackend


EXPECTED_SOURCE_COMMIT = QPROPC_REVISION.split("@", 1)[1]
APC_CASES = {
    "apc_10x7sf": {
        "folder": "apc_10x7sf",
        "geometry_file": "apcsf_10x7_geom.txt",
        "native_geometry": "10x7SF-PERF.PE0",
        "diameter_m": 10 * 0.0254,
        "label": "APC 10×7 SF",
        "polar": "naca4412_Ncrit=6",
        "airfoil_source": "APC 12 described by upstream as equivalent to NACA 4412",
    },
    "apc_16x8e": {
        "folder": "apc_16x8e",
        "geometry_file": "apce_16x8_geom.txt",
        "native_geometry": "16x8E-PERF.PE0",
        "diameter_m": 16 * 0.0254,
        "label": "APC 16×8E",
        "polar": "naca4412_Ncrit=6",
        "airfoil_source": "upstream's common NACA 4412 XFOIL polar collection",
    },
    "apc_4p2x4": {
        "folder": "apc_4.2x4",
        "geometry_file": "apcff_4.2x4_geom.txt",
        "native_geometry": "42x4-PERF.PE0",
        "diameter_m": 4.2 * 0.0254,
        "label": "APC Free Flight 4.2×4",
        "polar": "clark_y_Ncrit=7",
        "airfoil_source": "upstream Clark-Y XFOIL polar collection",
    },
}


def rpm_from_upstream_filename(filename: str) -> float:
    match = re.search(r"_(\d+)\.txt$", Path(filename).name)
    if match is None:
        raise ValueError(f"UIUC forward filename has no measured RPM suffix: {filename}")
    return float(match.group(1))


def parse_apc_pe0_geometry(
    path: str | Path,
    *,
    propeller_id: str,
    diameter_m: float,
    source_uri: str,
) -> PropellerGeometry:
    """Read manufacturer station rows from the APC PE0 text geometry file."""
    source = Path(path)
    lines = source.read_text(encoding="utf-8-sig", errors="replace").splitlines()
    radius_match = next((re.search(r"\bRADIUS:\s*([0-9.]+)", line, re.I) for line in lines
                         if re.search(r"\bRADIUS:\s*([0-9.]+)", line, re.I)), None)
    hub_match = next((re.search(r"\bHUBTRA:\s*([0-9.]+)", line, re.I) for line in lines
                      if re.search(r"\bHUBTRA:\s*([0-9.]+)", line, re.I)), None)
    blade_match = next((re.search(r"\bBLADES:\s*(\d+)", line, re.I) for line in lines
                        if re.search(r"\bBLADES:\s*(\d+)", line, re.I)), None)
    if radius_match is None or hub_match is None or blade_match is None:
        raise ValueError(f"{source}: APC PE0 must declare RADIUS, HUBTRA, and BLADES")
    rows: list[list[float]] = []
    for line in lines:
        fields = line.split()
        if len(fields) != 13:
            continue
        try:
            values = [float(field) for field in fields]
        except ValueError:
            continue
        rows.append(values)
    if len(rows) < 2:
        raise ValueError(f"{source}: APC PE0 must contain at least two 13-column station rows")
    inches_to_metres = 0.0254
    radius_in = float(radius_match.group(1))
    hub_in = float(hub_match.group(1))
    stations = tuple(
        BladeStation(
            radius_m=row[0] * inches_to_metres,
            chord_m=row[1] * inches_to_metres,
            twist_rad=math.radians(row[7]),
        )
        for row in rows
    )
    if not math.isclose(2 * radius_in * inches_to_metres, diameter_m, rel_tol=0.01):
        raise ValueError(f"{source}: declared PE0 diameter does not match benchmark nominal diameter")
    return PropellerGeometry(
        id=propeller_id,
        source_uri=source_uri,
        sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        blade_count=int(blade_match.group(1)),
        diameter_m=diameter_m,
        hub_radius_m=hub_in * inches_to_metres,
        stations=stations,
        geometry_provenance=GeometryProvenance.MANUFACTURER,
        polar_provenance=PolarProvenance.XFOIL,
        source_conventions=SourceConventions(
            rotor_axis="+X",
            blade_hand="UNKNOWN",
            section_sign="manufacturer PE0 twist convention",
            frame="propeller_axis",
            units="station/chord in, twist degrees",
        ),
    )


def _polar_dir(upstream_root: Path, category: dict) -> Path:
    validation_dir = upstream_root / "validation" / category["folder"]
    linked = validation_dir / f"airfoil_polar_{category['polar']}"
    if linked.is_dir():
        return linked
    # Git on Windows may check a repository symlink out as its target text.
    direct = upstream_root / "webgui" / "airfoil_polars" / category["polar"]
    if direct.is_dir():
        return direct
    raise FileNotFoundError(f"qprop.c polar collection is unavailable: {linked} or {direct}")


def load_upstream_apc_cases(upstream_root: str | Path, propeller_id: str) -> tuple[list[BenchmarkCase], dict]:
    root = Path(upstream_root).resolve()
    category = APC_CASES[propeller_id]
    folder = root / "validation" / category["folder"]
    uiuc = folder / "uiuc_data"
    native = folder / category["native_geometry"]
    geom_candidates = list(uiuc.glob("*geom.txt"))
    if not native.is_file():
        raise FileNotFoundError(f"upstream APC PE0 geometry is missing for {propeller_id}")
    if len(geom_candidates) == 1:
        geometry_path = geom_candidates[0]
        geometry = parse_radial_geometry(
            geometry_path,
            propeller_id=propeller_id,
            diameter_m=category["diameter_m"],
            hub_radius_m=0.0,
            blade_count=2,
            geometry_provenance=GeometryProvenance.DIGITIZED,
            polar_provenance=PolarProvenance.XFOIL,
            source_conventions=SourceConventions(
                rotor_axis="+X",
                blade_hand="UNKNOWN",
                section_sign="positive_pitch",
                frame="propeller_axis",
                units="r/R,c/R,beta_deg",
            ),
            source_uri=geometry_path.relative_to(root).as_posix(),
        )
        geometry_description = "upstream UIUC radial chord/twist text table"
    elif not geom_candidates:
        geometry_path = native
        geometry = parse_apc_pe0_geometry(
            native,
            propeller_id=propeller_id,
            diameter_m=category["diameter_m"],
            source_uri=native.relative_to(root).as_posix(),
        )
        geometry_description = "manufacturer APC PE0 radial station table"
    else:
        raise ValueError(f"multiple UIUC radial geometry tables found for {propeller_id}: {geom_candidates}")
    case_rows = []
    for path in sorted(uiuc.glob("*_static_*.txt")):
        measurements = parse_coefficient_measurements(
            path,
            propeller_id=propeller_id,
            regime="static",
            provenance=DataProvenance.TRANSCRIBED,
            source_description="UIUC wind-tunnel measurement transcribed in the pinned qprop.c upstream validation source",
            source_uri=path.relative_to(root).as_posix(),
        )
        case_rows.extend(measurements)
    for path in sorted(uiuc.glob("*.txt")):
        if "geom" in path.name.lower() or "static" in path.name.lower():
            continue
        rpm = rpm_from_upstream_filename(path.name)
        measurements = parse_coefficient_measurements(
            path,
            propeller_id=propeller_id,
            regime="forward",
            rpm_default=rpm,
            provenance=DataProvenance.TRANSCRIBED,
            source_description="UIUC wind-tunnel measurement transcribed in the pinned qprop.c upstream validation source",
            source_uri=path.relative_to(root).as_posix(),
        )
        case_rows.extend(measurements)
    if not case_rows:
        raise ValueError(f"no UIUC records found for qprop.c validation category {propeller_id}")
    cases = [BenchmarkCase(
        propeller_id=propeller_id,
        label=category["label"],
        family="APC",
        measurement=measurement,
        geometry=geometry,
        diameter_source=f"APC PE0 manufacturer geometry: nominal {category['diameter_m'] / 0.0254:g} in; exact physical diameter not independently measured here",
    ) for measurement in case_rows]
    return cases, {
        "native_geometry": native.relative_to(root).as_posix(),
        "native_geometry_sha256": hashlib.sha256(native.read_bytes()).hexdigest(),
        "polar_source": category["airfoil_source"],
        "polar_path": _polar_dir(root, category).relative_to(root).as_posix(),
        "measurement_count": len(cases),
        "source_geometry_provenance": f"{geometry_description}; qprop prediction uses exact APC PE0 geometry",
    }


def _numeric_rows(path: Path) -> list[list[float]]:
    rows = []
    for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines()[24:]:
        try:
            values = [float(token) for token in line.split()]
        except ValueError:
            continue
        if len(values) >= 12:
            rows.append(values)
    if len(rows) < 2:
        raise ValueError(f"QPROP v1.22 radial result table is incomplete: {path}")
    return rows


def _original_qprop_loads(rows: list[list[float]], blade_count: int = 2) -> tuple[float, float, float]:
    radii = [row[0] for row in rows]
    dr = [0.0] * len(radii)
    dr[0] = radii[1] - radii[0]
    for i in range(1, len(radii) - 1):
        dr[i] = 0.5 * (radii[i + 1] - radii[i - 1])
    dr[-1] = radii[-1] - radii[-2]
    radius = radii[-1] + dr[-1] / 2
    thrust_distribution, torque_distribution = [], []
    for row in rows:
        wa = row[9]
        tangential = wa * row[0] / (row[11] * radius)
        speed = math.hypot(wa, tangential)
        inflow = math.atan(wa / tangential)
        cn = row[3] * math.cos(inflow) - row[4] * math.sin(inflow)
        ct = row[3] * math.sin(inflow) + row[4] * math.cos(inflow)
        dynamic = 0.5 * 1.225 * speed**2
        thrust_distribution.append(dynamic * cn * row[1])
        torque_distribution.append(dynamic * ct * row[1] * row[0])
    thrust = blade_count * sum(value * width for value, width in zip(thrust_distribution, dr))
    torque = blade_count * sum(value * width for value, width in zip(torque_distribution, dr))
    return thrust, torque, 2 * radius


def reproduce_graupner_validation(upstream_root: str | Path, portable_dir: str | Path, out_dir: str | Path) -> dict:
    root = Path(upstream_root).resolve()
    folder = root / "validation" / "graupner_6x3"
    module = QPropCBackend(portable_dir, None, polar_mode="common_analytic")
    qprop = module.module
    airfoil = qprop.analytic_polar_curves(0.50, 5.8, -0.3, 1.2, 0.028, 0.050, 0.020, 0.5, 70000.0, -0.7)
    records = []
    try:
        for condition, folder_name, freestream in (
            ("hover", "original_qprop1.22_data_hovering", 0.01),
            ("cruise", "original_qprop1.22_data_Uinf=5", 5.0),
        ):
            path = folder / folder_name / "cam6x3_qprop1.22_output.txt"
            original_rows = _numeric_rows(path)
            ref_thrust, ref_torque, diameter = _original_qprop_loads(original_rows)
            # Reconstruct the upstream script's section stations from QPROP v1.22
            # element-centre output and its forward/backward endpoint extrapolation.
            first, second = original_rows[0], original_rows[1]
            dr0, dc0, db0 = second[0] - first[0], second[1] - first[1], second[2] - first[2]
            sections = [qprop.create_section(first[1] - dc0, qprop.deg2rad(first[2] - db0), first[0] - dr0 / 2, airfoil)]
            for index in range(1, len(original_rows)):
                current, previous = original_rows[index], original_rows[index - 1]
                if index + 1 < len(original_rows):
                    next_row = original_rows[index + 1]
                    dr_i = next_row[0] - previous[0]
                    dc_i = next_row[1] - previous[1]
                    db_i = next_row[2] - previous[2]
                    sections.append(qprop.create_section(
                        current[1] + dc_i / 2, qprop.deg2rad(current[2] + db_i / 2),
                        current[0] + dr_i / 2, airfoil,
                    ))
                else:
                    dr_i = current[0] - previous[0]
                    dc_i = current[1] - previous[1]
                    db_i = current[2] - previous[2]
                    sections.append(qprop.create_section(
                        current[1] + dc_i / 2, qprop.deg2rad(current[2] + db_i / 2),
                        current[0] + dr_i / 2, airfoil,
                    ))
            rotor_diameter = 2 * sections[-1].r
            rotor = qprop.create_rotor(rotor_diameter, 2, len(sections), sections)
            omega = 14020 * math.pi / 30
            performance = None
            try:
                performance = qprop.qprop(rotor, freestream, omega, 1e-6, 200)
                residual_max = max(abs(float(performance.residuals[i])) for i in range(performance.nelems))
                record = {
                    "condition": condition,
                    "source_uri": path.relative_to(root).as_posix(),
                    "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "freestream_mps": freestream,
                    "rpm": 14020,
                    "qpropc_thrust_n": float(performance.T),
                    "qprop122_reconstructed_thrust_n": ref_thrust,
                    "thrust_error_n": float(performance.T) - ref_thrust,
                    "qpropc_torque_nm": float(performance.Q),
                    "qprop122_reconstructed_torque_nm": ref_torque,
                    "torque_error_nm": float(performance.Q) - ref_torque,
                    "max_abs_residual": residual_max,
                    "converged": residual_max <= 1e-6,
                    "comparison_type": "reconstruction against original QPROP v1.22 radial output; not an experiment",
                }
            finally:
                if performance is not None:
                    qprop.free_rotor_performance(performance)
                # create_rotor exposes a ctypes-owned section array; let
                # Python release it instead of passing it to C free_rotor.
            records.append(record)
    finally:
        qprop.free_airfoil(airfoil)
        module.close()
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "graupner_qprop122.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    return {"comparison": "original QPROP v1.22 outputs, not wind-tunnel validation", "conditions": records}


def run_upstream_validation(upstream_root: str | Path, portable_dir: str | Path, out_dir: str | Path) -> dict:
    root = Path(upstream_root).resolve()
    try:
        commit = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            check=True, capture_output=True, text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ValueError(f"upstream root must be a git checkout of qprop.c revision {EXPECTED_SOURCE_COMMIT}") from exc
    if commit != EXPECTED_SOURCE_COMMIT:
        raise ValueError(f"qprop.c source revision mismatch: expected {EXPECTED_SOURCE_COMMIT}, found {commit}")
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=False)
    results = {"qpropc_revision": QPROPC_REVISION, "upstream_commit": commit, "categories": {}}
    for propeller_id, category in APC_CASES.items():
        cases, provenance = load_upstream_apc_cases(root, propeller_id)
        backend = QPropCBackend(
            portable_dir,
            _polar_dir(root, category),
            polar_mode="xfoil",
            polar_source_description=category["airfoil_source"],
            native_geometry_paths={propeller_id: root / "validation" / category["folder"] / category["native_geometry"]},
            out_of_scope_propellers=(propeller_id,) if propeller_id == "apc_4p2x4" else (),
            scenario_name=f"qpropc_upstream_{propeller_id}",
        )
        try:
            run = run_benchmark(
                cases,
                [backend],
                output / propeller_id,
                suite=f"qpropc-upstream-{propeller_id}",
            )
        finally:
            backend.close()
        results["categories"][propeller_id] = {
            **provenance,
            "polar_sha256": backend.polar_sha256,
            "polar_coverage": backend.polar_coverage,
            "metrics": run["metrics"]["backends"][backend.name],
        }
    graupner = reproduce_graupner_validation(root, portable_dir, output)
    results["categories"]["graupner_6x3"] = graupner
    (output / "summary.json").write_text(json.dumps(results, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream-root", type=Path, required=True)
    parser.add_argument("--portable-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run_upstream_validation(args.upstream_root, args.portable_dir, args.out), indent=2))


if __name__ == "__main__":
    main()
