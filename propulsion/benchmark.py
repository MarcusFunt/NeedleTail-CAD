"""Backend-neutral, provenance-carrying benchmark execution and reporting."""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import platform
from pathlib import Path
import statistics
import subprocess
import sys
import time
from typing import Protocol, Sequence

from propulsion.data import parse_coefficient_measurements, parse_radial_geometry
from propulsion.models import (
    DataProvenance,
    GeometryProvenance,
    OperatingCondition,
    PolarProvenance,
    PropellerGeometry,
    PropellerResult,
    SourceConventions,
)
from propulsion.coefficients import loads_from_coefficients


@dataclass(frozen=True)
class BenchmarkCase:
    propeller_id: str
    label: str
    family: str
    measurement: object
    geometry: PropellerGeometry
    diameter_source: str

    @property
    def rpm(self) -> float:
        return self.measurement.rpm

    @property
    def advance_ratio(self) -> float:
        return self.measurement.advance_ratio

    @property
    def regime(self) -> str:
        return self.measurement.regime

    @property
    def input_sha256(self) -> str:
        payload = {
            "propeller_id": self.propeller_id,
            "source_sha256": self.measurement.source_sha256,
            "source_row": self.measurement.source_row,
            "rpm": self.rpm,
            "J": self.advance_ratio,
            "diameter_m": self.geometry.diameter_m,
            "geometry_sha256": self.geometry.sha256,
        }
        return hashlib.sha256(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()


class PropellerBackend(Protocol):
    name: str
    revision: str
    initialization_ms: float

    def predict(self, case: BenchmarkCase) -> PropellerResult: ...


def _load_frozen_module(source_root: Path):
    script = source_root / "benchmark.py"
    if not script.is_file():
        raise FileNotFoundError(f"frozen benchmark.py not found under {source_root}")
    try:
        spec = importlib.util.spec_from_file_location("_needletail_frozen_bemt", script)
        if spec is None or spec.loader is None:
            raise ImportError("Python could not create a loader")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except ModuleNotFoundError as exc:
        if exc.name == "numpy":
            raise RuntimeError("frozen BEMT requires the project's optional aero extra (numpy)") from exc
        raise
    return module


def load_legacy195_cases(source_root: str | Path) -> tuple[BenchmarkCase, ...]:
    """Load the exact six-prop, 195-row archive into normalized records."""
    root = Path(source_root).resolve()
    frozen = _load_frozen_module(root)
    conventions = SourceConventions(
        rotor_axis="+X",
        blade_hand="UNKNOWN",
        section_sign="positive_pitch",
        frame="propeller_axis",
        units="r/R,c/R,beta_deg",
    )
    cases = []
    for propeller_id, metadata in frozen.PROPS.items():
        diameter = metadata["diameter_m"]
        geometry_path = root / metadata["geometry"]
        geometry = parse_radial_geometry(
            geometry_path,
            propeller_id=propeller_id,
            diameter_m=diameter,
            hub_radius_m=0.0,
            blade_count=int(frozen.B),
            geometry_provenance=GeometryProvenance.DIGITIZED,
            polar_provenance=PolarProvenance.ASSUMED,
            source_conventions=conventions,
            source_uri=geometry_path.relative_to(root).as_posix(),
        )
        for regime in ("static", "forward"):
            data_path = root / metadata[regime]
            if not data_path.is_file():
                raise FileNotFoundError(f"measurement file missing: {data_path}")
            measurements = parse_coefficient_measurements(
                data_path,
                propeller_id=propeller_id,
                regime=regime,
                rpm_default=metadata["rpm_forward"] if regime == "forward" else None,
                provenance=DataProvenance.TRANSCRIBED,
                source_description="UIUC Volume 2 wind-tunnel record transcribed into the supplied frozen archive",
                source_uri=data_path.relative_to(root).as_posix(),
            )
            cases.extend(BenchmarkCase(
                propeller_id=propeller_id,
                label=metadata["label"],
                family=metadata["family"],
                measurement=measurement,
                geometry=geometry,
                diameter_source=metadata["diameter_source"],
            ) for measurement in measurements)
    if len(cases) != 195:
        raise ValueError(f"legacy195 suite expected 195 source records, found {len(cases)}")
    return tuple(cases)


class FrozenBEMTBackend:
    """Thin adapter over the exact archived BEMT implementation."""

    name = "frozen_bemt"

    def __init__(self, source_root: str | Path):
        start = time.perf_counter()
        self.source_root = Path(source_root).resolve()
        self.module = _load_frozen_module(self.source_root)
        source = self.source_root / "benchmark.py"
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        adapter_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.revision = f"archive-sha256:{source_hash};adapter-sha256:{adapter_hash}"
        self.initialization_ms = (time.perf_counter() - start) * 1000

    def predict(self, case: BenchmarkCase) -> PropellerResult:
        start = time.perf_counter()
        if case.propeller_id in getattr(self.module, "PROPS", {}):
            point = self.module.bemt(case.propeller_id, case.rpm, case.advance_ratio)
        else:
            point = _frozen_bemt_from_geometry(case, self.module)
        ct = float(point["CT_pred"])
        cp = float(point["CP_pred"])
        condition = OperatingCondition(
            rpm=case.rpm,
            axial_airspeed_mps=case.advance_ratio * (case.rpm / 60) * case.geometry.diameter_m,
            rho_kg_m3=float(self.module.RHO),
            dynamic_viscosity_pa_s=1.81e-5,
            temperature_k=288.15,
            pressure_pa=101325.0,
            rotation_sign=1,
        )
        loads = loads_from_coefficients(ct, cp, case.geometry, condition)
        eligible = ct > 0 and cp > 0 and case.advance_ratio <= 0.5
        warnings = ["frozen_parameters", "digitized_chord_twist", "airfoil_sections_unavailable"]
        if ct <= 0:
            warnings.append("nonpositive_thrust")
        if cp <= 0:
            warnings.append("nonpositive_shaft_power")
        if case.advance_ratio > 0.5:
            warnings.append("high_advance_ratio_not_validated")
        elapsed_ms = (time.perf_counter() - start) * 1000
        return PropellerResult(
            thrust_n=loads.thrust_n,
            torque_nm=loads.torque_nm,
            shaft_power_w=loads.shaft_power_w,
            ct=ct,
            cp=cp,
            advance_ratio=case.advance_ratio,
            efficiency=(case.advance_ratio * ct / cp if case.advance_ratio > 0 and ct > 0 and cp > 0 else None),
            converged=bool(point["solver_converged"]),
            within_validity_range=eligible,
            warnings=tuple(warnings),
            solver_name=self.name,
            solver_revision=self.revision,
            input_sha256=case.input_sha256,
            elapsed_ms=elapsed_ms,
            geometry_valid=True,
            convergence_evidence=(f"iterations={int(point['solver_iterations'])}",),
            model_scope="frozen low-order BEMT; positive-thrust propulsive use only",
        )


def _frozen_bemt_from_geometry(case: BenchmarkCase, frozen_model) -> dict[str, float | int | bool]:
    """Apply the archived frozen coefficients/iteration logic to new geometry."""
    np = frozen_model.np
    blades = int(frozen_model.B)
    if case.geometry.blade_count != blades:
        raise ValueError(f"frozen BEMT supports {blades}-blade geometry, found {case.geometry.blade_count}")
    diameter = case.geometry.diameter_m
    radius = diameter / 2
    radial_ratio = np.array([station.radius_m / radius for station in case.geometry.stations])
    chord_ratio = np.array([station.chord_m / radius for station in case.geometry.stations])
    twist_degrees = np.array([math.degrees(station.twist_rad) for station in case.geometry.stations])
    n_elements = int(frozen_model.NELEM)
    element_ratio = 0.15 + (np.arange(n_elements) + 0.5) * 0.85 / n_elements
    element_radius = element_ratio * radius
    element_width = 0.85 * radius / n_elements
    chord = np.interp(element_ratio, radial_ratio, chord_ratio) * radius
    beta = np.deg2rad(np.interp(element_ratio, radial_ratio, twist_degrees))
    omega = case.rpm / 60 * 2 * math.pi
    freestream = case.advance_ratio * (case.rpm / 60) * diameter
    induced = np.full_like(element_radius, omega * radius * 0.08)
    swirl = np.zeros_like(element_radius)
    converged = False
    residual = math.inf
    for iteration in range(int(frozen_model.NMAX)):
        axial = freestream + induced
        tangential = omega * element_radius - swirl
        phi = np.arctan2(axial, tangential)
        speed_squared = axial**2 + tangential**2
        lift = np.clip(frozen_model.LIFT_SLOPE * (beta - phi), -frozen_model.CL_MAX, frozen_model.CL_MAX)
        drag = frozen_model.CD0 + frozen_model.DRAG_K * lift**2
        tip_loss = 2 / math.pi * np.arccos(np.exp(
            -blades * (radius - element_radius)
            / (2 * element_radius * np.maximum(np.sin(phi), 0.01))
        ))
        tip_loss = np.clip(tip_loss, frozen_model.F_FLOOR, 1.0)
        dthrust_dr = blades * 0.5 * frozen_model.RHO * speed_squared * chord * (
            lift * np.cos(phi) - drag * np.sin(phi)
        )
        dtorque_dr = blades * 0.5 * frozen_model.RHO * speed_squared * chord * element_radius * (
            lift * np.sin(phi) + drag * np.cos(phi)
        )
        momentum = np.maximum(dthrust_dr, 0) / (4 * math.pi * frozen_model.RHO * element_radius * tip_loss)
        induced_next = (-freestream + np.sqrt(freestream**2 + 4 * momentum)) / 2
        swirl_next = np.maximum(dtorque_dr, 0) / (
            4 * math.pi * frozen_model.RHO * element_radius**2 * tip_loss
            * np.maximum(freestream + induced, 0.05)
        )
        induced_next = np.clip(induced_next, 0, omega * element_radius * 0.9)
        swirl_next = np.clip(swirl_next, 0, omega * element_radius * 0.7)
        residual = max(
            float(np.max(np.abs(induced_next - induced))),
            float(np.max(np.abs(swirl_next - swirl))),
        )
        induced = 0.92 * induced + 0.08 * induced_next
        swirl = 0.92 * swirl + 0.08 * swirl_next
        if residual < 1e-8:
            converged = True
            break
    if not converged:
        raise RuntimeError(f"frozen BEMT induction did not converge for {case.propeller_id}: residual={residual:g}")
    thrust = float(np.sum(dthrust_dr) * element_width)
    torque = float(np.sum(dtorque_dr) * element_width)
    revolutions_per_second = case.rpm / 60
    shaft_power = omega * torque
    ct = thrust / (frozen_model.RHO * revolutions_per_second**2 * diameter**4)
    cp = shaft_power / (frozen_model.RHO * revolutions_per_second**3 * diameter**5)
    return {
        "CT_pred": float(ct),
        "CP_pred": float(cp),
        "thrust_N_pred": thrust,
        "torque_Nm_pred": torque,
        "shaft_power_W_pred": shaft_power,
        "eta_pred": float(case.advance_ratio * ct / cp) if cp > 1e-12 else math.nan,
        "solver_iterations": iteration + 1,
        "solver_converged": converged,
        "V_mps": freestream,
    }


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _p95(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]


def _crossing(records: list[dict], coefficient_key: str) -> float | None:
    points = sorted(
        ((float(row["advance_ratio"]), float(row[coefficient_key])) for row in records
         if row.get("status") == "ok" and row.get(coefficient_key) is not None),
        key=lambda point: point[0],
    )
    for (j0, y0), (j1, y1) in zip(points, points[1:]):
        if y0 == 0:
            return j0
        if y1 == 0:
            return j1
        if y0 * y1 < 0:
            return j0 + (j1 - j0) * (-y0) / (y1 - y0)
    return None


def zero_thrust_crossing(records: Sequence[dict]) -> tuple[float | None, float | None]:
    """Return a single-RPM measured/predicted crossing; mixed RPM is ambiguous."""
    rows = list(records)
    if len(zero_thrust_crossings_by_rpm(rows)) != 1:
        return None, None
    return _crossing(rows, "measured_ct"), _crossing(rows, "predicted_ct")


def zero_thrust_crossings_by_rpm(records: Sequence[dict]) -> tuple[dict[str, object], ...]:
    """Return independently interpolated zero-CT crossings for each RPM sweep."""
    groups: dict[float | None, list[dict]] = {}
    for row in records:
        raw_rpm = row.get("rpm")
        rpm = None if raw_rpm is None else float(raw_rpm)
        groups.setdefault(rpm, []).append(row)
    crossings = []
    for rpm in sorted(groups, key=lambda value: (value is None, value if value is not None else 0.0)):
        rows = groups[rpm]
        measured = _crossing(rows, "measured_ct")
        predicted = _crossing(rows, "predicted_ct")
        crossings.append({
            "rpm": rpm,
            "measured_zero_thrust_j": measured,
            "predicted_zero_thrust_j": predicted,
            "zero_thrust_j_error": (
                predicted - measured if measured is not None and predicted is not None else None
            ),
        })
    return tuple(crossings)


def _metric_group(records: list[dict], *, include_zero_crossing: bool = False) -> dict[str, object]:
    evaluated = [row for row in records if row.get("status") == "ok"]
    ct_errors = [float(row["predicted_ct"]) - float(row["measured_ct"]) for row in evaluated]
    cp_errors = [float(row["predicted_cp"]) - float(row["measured_cp"]) for row in evaluated]
    ct_mape = [abs(error / float(row["measured_ct"])) * 100
               for row, error in zip(evaluated, ct_errors) if float(row["measured_ct"]) > 1e-10]
    cp_mape = [abs(error / float(row["measured_cp"])) * 100
               for row, error in zip(evaluated, cp_errors) if float(row["measured_cp"]) > 1e-10]
    propulsive = [row for row in evaluated
                  if float(row["measured_ct"]) > 1e-10 and float(row["measured_cp"]) > 1e-10]
    cp_propulsive_mape = [
        abs((float(row["predicted_cp"]) - float(row["measured_cp"])) / float(row["measured_cp"])) * 100
        for row in propulsive
    ]
    ct_propulsive_mape = [
        abs((float(row["predicted_ct"]) - float(row["measured_ct"])) / float(row["measured_ct"])) * 100
        for row in propulsive
    ]
    # Normalize within each propeller first, then average propellers so the
    # largest coefficient scale cannot dominate the multi-prop summary.
    normalized: dict[str, dict[str, list[tuple[float, float]]]] = {}
    for row in evaluated:
        pair = normalized.setdefault(row["propeller_id"], {"ct": [], "cp": []})
        pair["ct"].append((float(row["measured_ct"]), float(row["predicted_ct"])))
        pair["cp"].append((float(row["measured_cp"]), float(row["predicted_cp"])))

    def nmae(key: str) -> float | None:
        per_propeller = []
        for values in normalized.values():
            reference = max((abs(measured) for measured, _ in values[key]), default=0.0)
            if reference > 0 and values[key]:
                per_propeller.append(
                    100 * statistics.fmean(abs(predicted - measured) for measured, predicted in values[key]) / reference
                )
        return _mean(per_propeller)

    elapsed = [float(row["elapsed_ms"]) for row in evaluated if row.get("elapsed_ms") is not None]
    positive_thrust = [row for row in records if float(row["measured_ct"]) > 0]
    validity_reported = [row for row in records if row.get("within_validity_range") is not None]
    in_validity = [
        row for row in evaluated
        if row.get("within_validity_range") is True
    ]
    in_validity_ct_errors = [float(row["predicted_ct"]) - float(row["measured_ct"]) for row in in_validity]
    in_validity_cp_errors = [float(row["predicted_cp"]) - float(row["measured_cp"]) for row in in_validity]
    in_validity_ct_mape = [
        abs(error / float(row["measured_ct"])) * 100
        for row, error in zip(in_validity, in_validity_ct_errors)
        if float(row["measured_ct"]) > 1e-10
    ]
    in_validity_propulsive = [
        row for row in in_validity
        if float(row["measured_ct"]) > 1e-10 and float(row["measured_cp"]) > 1e-10
    ]
    in_validity_cp_propulsive_mape = [
        abs((float(row["predicted_cp"]) - float(row["measured_cp"])) / float(row["measured_cp"])) * 100
        for row in in_validity_propulsive
    ]
    converged_count = sum(row.get("status") == "ok" for row in records)
    result: dict[str, object] = {
        "total_measurements": len(records),
        "evaluated": len(evaluated),
        "solver_failures": len(records) - len(evaluated),
        "converged_count": converged_count,
        "nonconverged_count": len(records) - converged_count,
        "within_validity_range_count": sum(bool(row["within_validity_range"]) for row in validity_reported),
        "outside_validity_range_count": sum(not bool(row["within_validity_range"]) for row in validity_reported),
        "validity_unreported_count": len(records) - len(validity_reported),
        "in_validity_evaluated_count": len(in_validity),
        "in_validity_ct_mae": _mean([abs(value) for value in in_validity_ct_errors]),
        "in_validity_ct_positive_mape_pct": _mean(in_validity_ct_mape),
        "in_validity_ct_positive_mape_count": len(in_validity_ct_mape),
        "in_validity_cp_mae": _mean([abs(value) for value in in_validity_cp_errors]),
        "in_validity_cp_propulsive_mape_pct": _mean(in_validity_cp_propulsive_mape),
        "in_validity_cp_propulsive_mape_count": len(in_validity_cp_propulsive_mape),
        "nonpositive_thrust_measurements": len(records) - len(positive_thrust),
        "ct_mae": _mean([abs(value) for value in ct_errors]),
        "ct_signed_bias": _mean(ct_errors),
        "ct_positive_mape_pct": _mean(ct_mape),
        "ct_positive_mape_count": len(ct_mape),
        "ct_nmae_pct_to_per_propeller_max_reference": nmae("ct"),
        "cp_mae": _mean([abs(value) for value in cp_errors]),
        "cp_signed_bias": _mean(cp_errors),
        "cp_positive_mape_pct": _mean(cp_mape),
        "cp_positive_mape_count": len(cp_mape),
        "cp_propulsive_mape_pct": _mean(cp_propulsive_mape),
        "cp_propulsive_mape_count": len(cp_propulsive_mape),
        "ct_propulsive_mape_pct": _mean(ct_propulsive_mape),
        "ct_propulsive_mape_count": len(ct_propulsive_mape),
        "cp_nmae_pct_to_per_propeller_max_reference": nmae("cp"),
        "elapsed_ms_median": statistics.median(elapsed) if elapsed else None,
        "elapsed_ms_p95": _p95(elapsed),
        "advance_ratio_min": min((float(row["advance_ratio"]) for row in records), default=None),
        "advance_ratio_max": max((float(row["advance_ratio"]) for row in records), default=None),
    }
    crossings = zero_thrust_crossings_by_rpm(records) if include_zero_crossing else ()
    measured_crossing, predicted_crossing = (
        (crossings[0]["measured_zero_thrust_j"], crossings[0]["predicted_zero_thrust_j"])
        if len(crossings) == 1 else (None, None)
    )
    result["zero_thrust_crossings_by_rpm"] = list(crossings)
    result["measured_zero_thrust_j"] = measured_crossing
    result["predicted_zero_thrust_j"] = predicted_crossing
    result["zero_thrust_j_error"] = (
        predicted_crossing - measured_crossing
        if measured_crossing is not None and predicted_crossing is not None else None
    )
    return result


def summarize_records(records: Sequence[dict]) -> dict[str, object]:
    """Compute explicit per-regime and per-propeller error, cost and coverage."""
    rows = list(records)
    by_regime = {
        regime: _metric_group([row for row in rows if row["regime"] == regime])
        for regime in sorted({row["regime"] for row in rows})
    }
    by_propeller = {}
    for propeller in sorted({row["propeller_id"] for row in rows}):
        by_propeller[propeller] = {
            regime: _metric_group([
                row for row in rows
                if row["propeller_id"] == propeller and row["regime"] == regime
            ], include_zero_crossing=(regime == "forward"))
            for regime in sorted({row["regime"] for row in rows if row["propeller_id"] == propeller})
        }
    by_j_band = {}
    bands = {
        "low_j_le_0p3": lambda j: j <= 0.3,
        "mid_j_0p3_to_0p6": lambda j: 0.3 < j <= 0.6,
        "high_j_gt_0p6": lambda j: j > 0.6,
    }
    forward = [row for row in rows if row["regime"] == "forward"]
    for propeller in sorted({row["propeller_id"] for row in forward}):
        by_j_band[propeller] = {}
        for band, predicate in bands.items():
            band_rows = [
                row for row in forward
                if row["propeller_id"] == propeller and predicate(float(row["advance_ratio"]))
            ]
            by_j_band[propeller][band] = _metric_group(band_rows)
    return {
        "record_count": len(rows),
        "solver_failures": sum(row.get("status") != "ok" for row in rows),
        "by_regime": by_regime,
        "by_propeller": by_propeller,
        "forward_by_j_band": by_j_band,
    }


def _record(case: BenchmarkCase, backend: PropellerBackend) -> dict[str, object]:
    base = {
        "propeller_id": case.propeller_id,
        "label": case.label,
        "family": case.family,
        "regime": case.regime,
        "rpm": case.rpm,
        "advance_ratio": case.advance_ratio,
        "diameter_m": case.geometry.diameter_m,
        "diameter_source": case.diameter_source,
        "measured_ct": case.measurement.ct,
        "measured_cp": case.measurement.cp,
        "measured_eta": case.measurement.eta,
        "provenance": case.measurement.provenance.value,
        "source_uri": case.measurement.source_uri,
        "source_sha256": case.measurement.source_sha256,
        "source_row": case.measurement.source_row,
        "geometry_sha256": case.geometry.sha256,
        "case_input_sha256": case.input_sha256,
        "input_sha256": None,
        "backend": backend.name,
        "backend_revision": backend.revision,
    }
    try:
        result = backend.predict(case)
    except Exception as exc:
        return {
            **base,
            "status": "failed",
            "failure_type": type(exc).__name__,
            "failure_reason": str(exc),
            "predicted_ct": None,
            "predicted_cp": None,
            "predicted_thrust_n": None,
            "predicted_torque_nm": None,
            "predicted_shaft_power_w": None,
            "predicted_eta": None,
            "elapsed_ms": None,
            "within_validity_range": False,
            "geometry_valid": False,
            "warnings": "",
        }
    return {
        **base,
        "status": "ok" if result.converged else "failed",
        "failure_type": "" if result.converged else "nonconverged",
        "failure_reason": "" if result.converged else "; ".join(result.warnings),
        "predicted_ct": result.ct,
        "predicted_cp": result.cp,
        "predicted_thrust_n": result.thrust_n,
        "predicted_torque_nm": result.torque_nm,
        "predicted_shaft_power_w": result.shaft_power_w,
        "predicted_eta": result.efficiency,
        "convergence_evidence": ";".join(result.convergence_evidence),
        "elapsed_ms": result.elapsed_ms,
        "within_validity_range": result.within_validity_range,
        "geometry_valid": result.geometry_valid,
        "warnings": ";".join(result.warnings),
        "input_sha256": result.input_sha256,
    }


def _git_revision() -> str | None:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _write_svg(path: Path, records: list[dict], *, x_field: str, x_label: str) -> None:
    width, height = 820, 420
    rpm_groups: dict[float | None, list[dict]] = {}
    if x_field == "advance_ratio":
        for row in records:
            raw_rpm = row.get("rpm")
            rpm = None if raw_rpm is None else float(raw_rpm)
            rpm_groups.setdefault(rpm, []).append(row)
        if not rpm_groups:
            rpm_groups[None] = []
    else:
        rpm_groups[None] = records
    rpm_order = sorted(rpm_groups, key=lambda value: (value is None, value if value is not None else 0.0))
    if x_field == "advance_ratio" and len(rpm_order) > 1:
        height = 438
    panels = ((70, 75, 330, 270, "C_T"), (460, 75, 330, 270, "C_P"))
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<style>text{font:14px sans-serif}.axis{stroke:#333;stroke-width:1}.measured{fill:#2563eb}.predicted{fill:#dc2626}.prediction-line{stroke-width:1.8;fill:none}</style>',
             f'<text x="{width/2}" y="28" text-anchor="middle">{path.stem}: measured and backend predictions</text>']
    for x, y, w, h, coeff in panels:
        key = "ct" if coeff == "C_T" else "cp"
        measured_key, predicted_key = f"measured_{key}", f"predicted_{key}"
        values = [float(row[name]) for row in records for name in (measured_key, predicted_key)
                  if row.get("status") == "ok" and row.get(name) is not None]
        if not values:
            continue
        ymin, ymax = min(values), max(values)
        if ymin == ymax:
            ymin -= .01
            ymax += .01
        padding = (ymax - ymin) * .08
        ymin -= padding
        ymax += padding
        xvalues = [float(row[x_field]) for row in records]
        xmin, xmax = min(xvalues, default=0.0), max(xvalues, default=1.0)
        if xmin == xmax:
            xmin -= 1.0
            xmax += 1.0
        parts.extend((f'<text x="{x+w/2}" y="{y-22}" text-anchor="middle">{coeff} vs {x_label}</text>',
                      f'<line class="axis" x1="{x}" y1="{y+h}" x2="{x+w}" y2="{y+h}"/>',
                      f'<line class="axis" x1="{x}" y1="{y}" x2="{x}" y2="{y+h}"/>',
                      f'<text x="{x+w/2}" y="{y+h+38}" text-anchor="middle">{x_label}</text>',
                      f'<text x="{x-14}" y="{y-6}" text-anchor="end">{ymax:.3g}</text>',
                      f'<text x="{x-14}" y="{y+h}" text-anchor="end">{ymin:.3g}</text>'))
        for row in records:
            if row.get("status") != "ok":
                continue
            xvalue = float(row[x_field])
            for field, klass, marker in ((measured_key, "measured", "circle"),
                                         (predicted_key, "predicted", "rect")):
                value = row.get(field)
                if value is None:
                    continue
                px = x + ((xvalue - xmin) / (xmax - xmin)) * w
                py = y + h - ((float(value) - ymin) / (ymax - ymin)) * h
                if marker == "circle":
                    parts.append(f'<circle class="{klass}" cx="{px:.2f}" cy="{py:.2f}" r="3.2"/>')
                else:
                    parts.append(f'<rect class="{klass}" x="{px-2.8:.2f}" y="{py-2.8:.2f}" width="5.6" height="5.6"/>')
        palette = ("#dc2626", "#7c3aed", "#059669", "#d97706", "#2563eb", "#db2777", "#0891b2", "#4d7c0f")
        for series_index, rpm in enumerate(rpm_order):
            predictions = sorted(
                ((float(row[x_field]), float(row[predicted_key])) for row in rpm_groups[rpm]
                 if row.get("status") == "ok" and row.get(predicted_key) is not None),
                key=lambda item: item[0],
            )
            if len(predictions) > 1:
                points = " ".join(
                    f"{x + ((xvalue - xmin) / (xmax - xmin)) * w:.2f},"
                    f"{y + h - ((value - ymin) / (ymax - ymin)) * h:.2f}"
                    for xvalue, value in predictions
                )
                rpm_attribute = f' data-rpm="{rpm:g}"' if rpm is not None and x_field == "advance_ratio" else ""
                rpm_title = f"Predicted, {rpm:g} RPM" if rpm is not None and x_field == "advance_ratio" else "Predicted static sweep"
                color = palette[series_index % len(palette)]
                parts.append(
                    f'<polyline class="prediction-line"{rpm_attribute} style="stroke:{color}" points="{points}">'
                    f"<title>{rpm_title}</title></polyline>"
                )
    parts.extend(('<circle class="measured" cx="330" cy="392" r="4"/>',
                  '<text x="340" y="397">measurement</text>',
                  '<rect class="predicted" x="470" y="388" width="8" height="8"/>',
                  '<text x="484" y="397">prediction</text>',
                  '</svg>'))
    if x_field == "advance_ratio" and len(rpm_order) > 1:
        parts.insert(-1, '<text x="70" y="426">Predicted lines are split by RPM; hover a line for its RPM.</text>')
    path.write_text("\n".join(parts), encoding="utf-8")


def run_benchmark(
    cases: Sequence[BenchmarkCase],
    backends: Sequence[PropellerBackend],
    out_dir: str | Path,
    *,
    suite: str,
    input_manifest: str | Path | None = None,
) -> dict[str, object]:
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=False)
    all_records = []
    summaries = {}
    for backend in backends:
        records = [_record(case, backend) for case in cases]
        all_records.extend(records)
        summaries[backend.name] = summarize_records(records)
        plot_dir = output / "plots"
        plot_dir.mkdir(exist_ok=True)
        for propeller_id in sorted({row["propeller_id"] for row in records}):
            for regime, x_field, x_label in (
                ("static", "rpm", "rotor speed (RPM)"),
                ("forward", "advance_ratio", "advance ratio J"),
            ):
                selected = [row for row in records
                            if row["propeller_id"] == propeller_id and row["regime"] == regime]
                if selected:
                    _write_svg(
                        plot_dir / f"{propeller_id}_{backend.name}_{regime}.svg",
                        selected,
                        x_field=x_field,
                        x_label=x_label,
                    )
    fields = list(dict.fromkeys(key for row in all_records for key in row))
    with (output / "cases.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(all_records)
    summary = {"suite": suite, "backends": summaries}
    (output / "metrics.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "suite": suite,
        "source_case_count": len(cases),
        "backend_count": len(backends),
        "backend_revisions": {backend.name: backend.revision for backend in backends},
        "backend_initialization_ms": {
            backend.name: backend.initialization_ms for backend in backends
        },
        "backend_polar_inputs": {
            backend.name: {
                "sha256": getattr(backend, "polar_sha256", None),
                "description": getattr(backend, "polar_source_description", None),
                "coverage": getattr(backend, "polar_coverage", None),
            }
            for backend in backends
        },
        "backend_configuration": {
            backend.name: getattr(backend, "configuration", None)
            for backend in backends
        },
        "cpu": platform.processor() or None,
        "python": sys.version,
        "platform": platform.platform(),
        "git_head": _git_revision(),
        "local_implementation_sha256": {
            relative: hashlib.sha256((Path(__file__).resolve().parents[1] / relative).read_bytes()).hexdigest()
            for relative in (
                "propulsion/benchmark.py",
                "propulsion/coefficients.py",
                "propulsion/data.py",
                "propulsion/models.py",
                "propulsion/qpropc.py",
                "validation/holdouts.py",
                "validation/run_holdout.py",
                "validation/qpropc/validate_upstream.py",
            )
            if (Path(__file__).resolve().parents[1] / relative).is_file()
        },
        "input_manifest_sha256": (
            hashlib.sha256(Path(input_manifest).read_bytes()).hexdigest()
            if input_manifest is not None else None
        ),
        "raw_per_point_data": "cases.csv",
        "cache_state": "No backend result cache used by the runner.",
    }
    (output / "run_manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    report = [f"# Propeller benchmark: {suite}", "", f"Source rows: {len(cases)}", ""]
    for backend_name, backend_summary in summaries.items():
        report.extend((f"## {backend_name}", "", "| Regime | N | Evaluated | Failures | In validity | Out of validity | Unreported | CT MAE | CT propulsive MAPE | CP MAE | CP propulsive MAPE |",
                       "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"))
        for regime, metric in backend_summary["by_regime"].items():
            report.append(
                f"| {regime} | {metric['total_measurements']} | {metric['evaluated']} | "
                f"{metric['solver_failures']} | {metric['within_validity_range_count']} | "
                f"{metric['outside_validity_range_count']} | {metric['validity_unreported_count']} | "
                f"{_fmt(metric['ct_mae'])} | {_fmt(metric['ct_propulsive_mape_pct'])}% | "
                f"{_fmt(metric['cp_mae'])} | {_fmt(metric['cp_propulsive_mape_pct'])}% |"
            )
        report.extend((
            "",
            "Within-validity accuracy (successful rows only):",
            "",
            "| Regime | In-range N | CT MAE | CT positive MAPE | CP MAE | CP propulsive MAPE |",
            "|---|---:|---:|---:|---:|---:|",
        ))
        for regime, metric in backend_summary["by_regime"].items():
            report.append(
                f"| {regime} | {metric['in_validity_evaluated_count']} | "
                f"{_fmt(metric['in_validity_ct_mae'])} | {_fmt(metric['in_validity_ct_positive_mape_pct'])}% | "
                f"{_fmt(metric['in_validity_cp_mae'])} | {_fmt(metric['in_validity_cp_propulsive_mape_pct'])}% |"
            )
        report.append("")
    report.extend(("Per-condition raw data and source hashes are in `cases.csv`; plots are in `plots/`.",
                   "The CSV and JSON outputs are generated locally and can contain transcribed source measurements.", ""))
    (output / "REPORT.md").write_text("\n".join(report), encoding="utf-8")
    return {"metrics": summary, "manifest": manifest, "records": all_records}


def _fmt(value: object) -> str:
    return "—" if value is None else f"{float(value):.6g}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("legacy195",), required=True)
    parser.add_argument("--backend", choices=("frozen", "qpropc"), nargs="+", required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--source-manifest", type=Path)
    parser.add_argument("--qpropc-portable", type=Path)
    parser.add_argument("--polar-dir", type=Path)
    parser.add_argument("--polar-mode", choices=("common_analytic", "xfoil"), nargs="+",
                        default=("common_analytic",))
    parser.add_argument("--polar-description")
    args = parser.parse_args()
    cases = load_legacy195_cases(args.source_root)
    backends: list[PropellerBackend] = []
    qprop_backends = []
    for backend_name in args.backend:
        if backend_name == "frozen":
            backends.append(FrozenBEMTBackend(args.source_root))
        else:
            if args.qpropc_portable is None:
                parser.error("--qpropc-portable is required when --backend includes qpropc")
            if "xfoil" in args.polar_mode and args.polar_dir is None:
                parser.error("--polar-dir is required for --polar-mode xfoil")
            from propulsion.qpropc import QPropCBackend

            for mode in args.polar_mode:
                instance = QPropCBackend(
                    args.qpropc_portable,
                    args.polar_dir if mode == "xfoil" else None,
                    polar_mode=mode,
                    polar_source_description=args.polar_description,
                )
                backends.append(instance)
                qprop_backends.append(instance)
    try:
        result = run_benchmark(cases, backends, args.out, suite=args.suite, input_manifest=args.source_manifest)
    finally:
        for backend in qprop_backends:
            backend.close()
    compact = {
        "record_count": len(cases),
        "backends": {
            name: {
                "solver_failures": summary["solver_failures"],
                "by_regime": summary["by_regime"],
            }
            for name, summary in result["metrics"]["backends"].items()
        },
    }
    print(json.dumps(compact, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
