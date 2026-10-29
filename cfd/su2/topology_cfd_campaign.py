"""Run a matched, reduced-fidelity powered Euler screen across rotor topologies."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CFD_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(CFD_DIR))

from airframe_geometry import _one_solid, airframe_parameters_for_geometry
from configure_rotor_case import write_configs
from cadgen import build123d as bd
from design_cad import build_variant_components
from design_cad import clearance_report, validate_variant
from design_configuration import Design, moment_about_shaft_axis
from detailed_rotor_cfd import available_memory_bytes, geometry_values, run_solver
from generate_rotor_mesh import (
    RotorZoneOverlapError,
    StationaryZoneIntersectionError,
    _rotors,
    generate_mesh,
    validate_multizone_mesh,
    validate_rotor_zone_geometry,
)

TOPOLOGIES = ("tractor4", "pusher4", "coaxial8")
RHO = 101325.0 / (287.058 * 288.15)
SOUND = math.sqrt(1.4 * 287.058 * 288.15)
MIN_RUNTIME_MEMORY_BYTES = int(0.8 * 1024**3)
MIN_RUNTIME_DISK_BYTES = int(1.5 * 1024**3)


class CADPreflightError(ValueError):
    """A CAD clearance gate failed before meshing or solving."""


class DiagnosticOnlyStop(RuntimeError):
    """The explicit CAD diagnostic mode stopped before mesh generation."""
MAX_FINAL_LOG10_RESIDUAL = -5.0
MAX_FINAL_REVOLUTION_FORCE_CHANGE = 0.05


def replace_config_keys(text: str, updates: dict[str, str]) -> str:
    rows: list[str] = []
    remaining = dict(updates)
    for row in text.splitlines():
        key = row.split("=", 1)[0].strip()
        if "=" in row and not row.lstrip().startswith("%") and key in updates:
            rows.append(f"{key}= {updates[key]}")
            remaining.pop(key, None)
        else:
            rows.append(row)
    rows.extend(f"{key}= {value}" for key, value in remaining.items())
    return "\n".join(rows) + "\n"


def configure_histories(case_dir: Path, rotor_count: int, physical_steps: int) -> None:
    """Keep per-zone force histories while suppressing large flow snapshots."""
    zone_count = rotor_count + 1
    zone_groups = [
        f"{field}[{zone}]"
        for zone in range(zone_count)
        for field in ("BGS_RES", "RMS_RES", "AERO_COEFF", "NONPHYSICAL_POINTS")
    ]
    bgs_fields = ", ".join(f"AVG_BGS_RES[{zone}]" for zone in range(zone_count))
    driver_updates = {
        "MULTIZONE_RESIDUAL": "YES",
        "WRT_ZONE_HIST": "YES",
        "HISTORY_OUTPUT": "( ITER, TIME_DOMAIN, " + ", ".join(zone_groups) + " )",
        "HISTORY_WRT_FREQ_INNER": "5",
        "HISTORY_WRT_FREQ_OUTER": "1",
        "HISTORY_WRT_FREQ_TIME": "1",
        "SCREEN_OUTPUT": f"( TIME_ITER, OUTER_ITER, {bgs_fields} )",
        "CONV_FIELD": f"( {bgs_fields} )",
        "CONV_RESIDUAL_MINVAL": "-5",
        "OUTPUT_FILES": "( RESTART )",
        "OUTPUT_WRT_FREQ": str(physical_steps + 1),
        "WRT_RESTART_OVERWRITE": "YES",
    }
    driver = case_dir / "rotor_case.cfg"
    driver.write_text(
        replace_config_keys(driver.read_text(encoding="utf-8"), driver_updates),
        encoding="utf-8",
    )
    for zone in range(zone_count):
        path = case_dir / ("zone_static.cfg" if zone == 0 else f"zone_rotor_{zone}.cfg")
        marker = "AIRFRAME_MOTOR" if zone == 0 else f"ROTOR_PROPELLER_{zone}"
        zone_updates = {
            "MARKER_MONITORING": f"( {marker} )",
            "MARKER_PLOTTING": f"( {marker} )",
            "OUTPUT_FILES": "( RESTART )",
            "OUTPUT_WRT_FREQ": str(physical_steps + 1),
            "WRT_RESTART_OVERWRITE": "YES",
        }
        zone_text = replace_config_keys(path.read_text(encoding="utf-8"), zone_updates)
        zone_text += (
            "HISTORY_OUTPUT= ( ITER, TIME_DOMAIN, RMS_RES, AERO_COEFF, NONPHYSICAL_POINTS )\n"
            "HISTORY_WRT_FREQ_INNER= 5\n"
            "HISTORY_WRT_FREQ_OUTER= 1\n"
            "HISTORY_WRT_FREQ_TIME= 1\n"
            "SCREEN_OUTPUT= ( TIME_ITER, OUTER_ITER, RMS_RES, DRAG, LIFT )\n"
        )
        path.write_text(zone_text, encoding="utf-8")


def find_key(headers: list[str], target: str) -> str:
    normalized_target = target.lower().replace("_", "")
    for header in headers:
        # SU2 8.5 quotes and space-pads AERO_COEFF headers in its CSV files.
        base = header.split("[", 1)[0].strip().strip('"').strip()
        if base.lower().replace("_", "") == normalized_target:
            return header
    raise KeyError(f"Missing {target} in SU2 history columns: {headers}")


def _read_history_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream, skipinitialspace=True)
        headers = reader.fieldnames or []
        normalized = [header.strip().strip('"').casefold() for header in headers]
        if len(set(normalized)) != len(normalized):
            raise ValueError(f"History contains duplicate CSV columns: {path.name}")
        return headers, list(reader)


def rows_by_zone(case_dir: Path, zone: int) -> list[dict[str, str]]:
    paths = [case_dir / f"rotor_history_{zone}.csv", case_dir / f"history_{zone}.csv"]
    for path in paths:
        if path.is_file():
            return _read_history_csv(path)[1]
    driver = multizone_history_path(case_dir)
    if driver is None:
        raise FileNotFoundError(f"No history file found for zone {zone} under {case_dir}")
    _, all_rows = _read_history_csv(driver)
    suffix = f"[{zone}]"
    selected: list[dict[str, str]] = []
    for row in all_rows:
        new = {}
        for key, value in row.items():
            clean = key.strip().strip('"').strip()
            if clean.endswith(suffix):
                new[clean[:-len(suffix)].strip()] = value
            elif clean.casefold() in ("time_iter", "outer_iter"):
                new[clean] = value
        selected.append(new)
    return selected


def multizone_history_path(case_dir: Path) -> Path | None:
    for name in ("rotor_history_multizone.csv", "rotor_history.csv"):
        path = case_dir / name
        if path.is_file():
            return path
    return None


def _physical_time_step(row: dict[str, str], key: str, source: str) -> int:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{source} history has an invalid physical time index") from exc
    if not math.isfinite(value) or not value.is_integer():
        raise ValueError(f"{source} history has a non-integer physical time index")
    return int(value)


def summarize_coupling_residuals(
    case_dir: Path,
    zone_count: int,
    expected_time_indices: list[int],
) -> list[dict[str, Any]]:
    """Require complete, declining BGS/coupling histories for every zone."""
    driver = multizone_history_path(case_dir)
    if driver is None:
        return [{"zone": zone, "status": "missing", "passed": False,
                 "initial_log_residuals": {}, "final_log_residuals": {}}
                for zone in range(zone_count)]
    headers, rows = _read_history_csv(driver)
    if not rows:
        return [{"zone": zone, "status": "empty", "passed": False,
                 "initial_log_residuals": {}, "final_log_residuals": {}}
                for zone in range(zone_count)]
    time_key = find_key(headers, "Time_Iter")
    final_by_step: dict[int, dict[str, str]] = {}
    for row in rows:
        step = _physical_time_step(row, time_key, driver.name)
        final_by_step[step] = row
    observed_steps = sorted(final_by_step)
    reports = []
    for zone in range(zone_count):
        suffix = f"[{zone}]"
        residual_keys = []
        for header in headers:
            normalized = header.strip().strip('"').strip().lower().replace("_", "")
            if not normalized.endswith(suffix):
                continue
            base = normalized[:-len(suffix)]
            if "bgsres" in base or "couplingres" in base:
                residual_keys.append(header)
        if not residual_keys or not observed_steps:
            reports.append({"zone": zone, "status": "missing", "passed": False,
                            "complete_time_steps": observed_steps == expected_time_indices,
                            "initial_log_residuals": {}, "final_log_residuals": {}})
            continue
        initial_row = final_by_step[observed_steps[0]]
        final_row = final_by_step[observed_steps[-1]]
        initial, final = {}, {}
        for key in residual_keys:
            try:
                initial[key] = float(initial_row[key])
                final[key] = float(final_row[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("Multizone history has missing or nonnumeric coupling residuals") from exc
        if not all(math.isfinite(value) for value in (*initial.values(), *final.values())):
            raise ValueError("Multizone history contains nonfinite coupling residual")
        complete = observed_steps == expected_time_indices
        declined = bool(final) and initial.keys() == final.keys() and all(
            final[key] < initial[key] for key in final
        )
        threshold_passed = bool(final) and all(
            value <= MAX_FINAL_LOG10_RESIDUAL for value in final.values()
        )
        passed = complete and declined and threshold_passed
        reports.append({
            "zone": zone,
            "status": "passed" if passed else "failed",
            "passed": passed,
            "complete_time_steps": complete,
            "initial_log_residuals": initial,
            "final_log_residuals": final,
            "declined": declined,
            "final_threshold_passed": threshold_passed,
            "missing_time_step_indices_sample": [
                step for step in expected_time_indices if step not in final_by_step
            ][:20],
        })
    return reports


def geometry_preflight(
    params,
    design: Design,
    min_clearance_mm: float | None = None,
) -> dict[str, Any]:
    """Return the CAD clearance gate without requiring Gmsh or SU2."""
    required = (
        params.propeller_clearance_margin
        if min_clearance_mm is None
        else min_clearance_mm
    )
    if not math.isfinite(required) or required < 0:
        raise ValueError("Minimum CAD clearance must be finite and nonnegative")
    validation_error = None
    try:
        clearances = validate_variant(params, design, min_clearance_mm=required)
    except ValueError as exc:
        validation_error = str(exc)
        clearances = clearance_report(params, design)
    failures = {
        name: distance
        for name, distance in clearances.items()
        if not math.isfinite(distance) or distance < required
    }
    if not clearances:
        failures["clearance_report"] = "empty clearance report"
    if validation_error and not failures:
        failures["variant_validation"] = validation_error
    return {
        "valid": not failures,
        "required_minimum_clearance_mm": required,
        "minimum_clearance_mm": min(clearances.values()) if clearances else None,
        "clearance_count": len(clearances),
        "failures": failures,
        "coaxial_separation_mm": (
            design.coaxial_separation_mm if design.topology == "coaxial8" else None
        ),
    }


def validate_coaxial_separation(
    topologies: list[str] | tuple[str, ...],
    coaxial_separation_mm: float | None,
) -> None:
    if "coaxial8" in topologies and coaxial_separation_mm is None:
        raise ValueError("Set --coaxial-separation-mm when comparing the coaxial8 topology")
    if coaxial_separation_mm is not None and (
        not math.isfinite(coaxial_separation_mm) or coaxial_separation_mm <= 0
    ):
        raise ValueError("Coaxial rotor-plane separation must be finite and positive")


def shaft_power_regime(power_w: float) -> str:
    """Label signed shaft power without silently treating zero as motor input."""
    if not math.isfinite(power_w):
        raise ValueError("Shaft power must be finite")
    if power_w > 1e-9:
        return "motor_input_required"
    if power_w < -1e-9:
        return "windmilling_or_power_turbine"
    return "zero_shaft_power"


def summarize_history(
    case_dir: Path,
    topology: str,
    params,
    revolutions: int,
    steps_per_revolution: int,
    rpm: float,
    speed_mps: float,
    alpha_deg: float,
    coaxial_separation_mm: float | None = None,
) -> dict[str, Any]:
    rotors = _rotors(params, topology, coaxial_separation_mm)
    rotor_count = len(rotors)
    expected_steps = revolutions * steps_per_revolution
    expected_time_indices = list(range(expected_steps + 1))
    expected_index_set = set(expected_time_indices)
    q = 0.5 * RHO * speed_mps**2
    area = params.wing_planform_area * 1e-6
    taper = params.wing_tip_chord / params.wing_root_chord
    mac = (2 / 3 * params.wing_root_chord * 1e-3
           * (1 + taper + taper**2) / (1 + taper))
    zone_records: list[dict[str, Any]] = []

    for zone in range(rotor_count + 1):
        try:
            rows = rows_by_zone(case_dir, zone)
        except FileNotFoundError:
            zone_records.append({
                "zone": zone,
                "kind": "static_airframe_and_motor" if zone == 0 else "propeller_only_rotor_zone",
                "complete_time_steps": False,
                "status": "missing_history_file",
                "missing_time_step_count": expected_steps + 1,
                "missing_time_step_indices_sample": expected_time_indices[:20],
                "unexpected_time_steps": [],
                "sample_count_final_2_revolutions": 0,
            })
            continue
        if not rows:
            zone_records.append({
                "zone": zone,
                "kind": "static_airframe_and_motor" if zone == 0 else "propeller_only_rotor_zone",
                "complete_time_steps": False,
                "status": "empty_history",
                "missing_time_step_count": expected_steps + 1,
                "missing_time_step_indices_sample": expected_time_indices[:20],
                "unexpected_time_steps": [],
                "sample_count_final_2_revolutions": 0,
            })
            continue
        headers = list(rows[0])
        time_key = find_key(headers, "Time_Iter")
        force_keys = {axis: find_key(headers, f"CF{axis}") for axis in "xyz"}
        moment_keys = {axis: find_key(headers, f"CM{axis}") for axis in "xyz"}
        # SU2 can write several pseudo-time rows at one physical time; retain the final one.
        final_by_step: dict[int, dict[str, str]] = {}
        for row in rows:
            step = _physical_time_step(row, time_key, f"zone {zone}")
            final_by_step[step] = row
        steps = sorted(final_by_step)
        if not steps:
            raise RuntimeError(f"Zone {zone} history has no physical time indices")
        for step in steps:
            for key in (*force_keys.values(), *moment_keys.values()):
                try:
                    value = float(final_by_step[step][key])
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(f"Zone {zone} has a missing physical history value") from exc
                if not math.isfinite(value):
                    raise ValueError("History contains nonfinite physical force or moment")
        if steps != expected_time_indices:
            observed = set(steps)
            missing = [step for step in expected_time_indices if step not in observed]
            unexpected = [step for step in steps if step not in expected_index_set]
            zone_records.append({
                "zone": zone,
                "kind": "static_airframe_and_motor" if zone == 0 else "propeller_only_rotor_zone",
                "complete_time_steps": False,
                "last_time_iteration": steps[-1],
                "missing_time_step_count": len(missing),
                "missing_time_step_indices_sample": missing[:20],
                "unexpected_time_steps": unexpected[:20],
                "sample_count_final_2_revolutions": 0,
            })
            continue
        selected_start = max(0, expected_steps - 2 * steps_per_revolution)
        sample_steps = [step for step in steps if step >= selected_start]
        if not sample_steps:
            sample_steps = steps[-1:]

        def mean_force_moment(sample: list[int]) -> tuple[dict[str, float], dict[str, float]]:
            forces = {axis: 0.0 for axis in "xyz"}
            moments = {axis: 0.0 for axis in "xyz"}
            for step in sample:
                force_row = {axis: float(final_by_step[step][force_keys[axis]])
                             for axis in "xyz"}
                moment_row = {axis: float(final_by_step[step][moment_keys[axis]])
                              for axis in "xyz"}
                if not all(math.isfinite(value) for value in (*force_row.values(), *moment_row.values())):
                    raise ValueError("History contains nonfinite physical force or moment")
                for axis in "xyz":
                    forces[axis] += force_row[axis] * q * area / len(sample)
                    moments[axis] += moment_row[axis] * q * area * mac / len(sample)
            return forces, moments

        forces, moments = mean_force_moment(sample_steps)
        last_window = [step for step in steps if step > expected_steps - steps_per_revolution]
        previous_window = [step for step in steps
                           if expected_steps - 2 * steps_per_revolution < step
                           <= expected_steps - steps_per_revolution]
        drift: dict[str, float | None] = {}
        if last_window and previous_window:
            last_force, last_moment = mean_force_moment(last_window)
            previous_force, previous_moment = mean_force_moment(previous_window)
            for axis in "xyz":
                drift[axis] = abs(last_force[axis] - previous_force[axis]) / max(abs(forces[axis]), 0.02)
        else:
            last_force = last_moment = previous_force = previous_moment = None
            drift = {axis: None for axis in "xyz"}

        residual_keys = [key for key in headers
                         if key.strip().strip('"').strip().lower().replace("_", "").startswith("rms")]
        residuals, initial_residuals = {}, {}
        if residual_keys:
            for key in residual_keys:
                try:
                    initial_residuals[key] = float(final_by_step[steps[0]][key])
                    residuals[key] = float(final_by_step[steps[-1]][key])
                except (KeyError, TypeError, ValueError):
                    raise ValueError(f"Zone {zone} has missing or nonnumeric residual values")
        if not all(math.isfinite(value) for value in (*initial_residuals.values(), *residuals.values())):
            raise ValueError(f"Zone {zone} history contains nonfinite residual values")
        residuals_declined = bool(residuals) and initial_residuals.keys() == residuals.keys() and all(
            residuals[key] < initial_residuals[key] for key in residuals
        )
        nonphysical_points = {
            "available": False,
            "passed": False,
            "maximum_count": None,
            "nonzero_time_indices_sample": [],
            "invalid_time_indices_sample": [],
        }
        try:
            nonphysical_key = find_key(headers, "NONPHYSICAL_POINTS")
        except KeyError:
            pass
        else:
            counts: dict[int, int] = {}
            invalid_steps: list[int] = []
            for step in steps:
                try:
                    count_value = float(final_by_step[step][nonphysical_key])
                    if (not math.isfinite(count_value) or count_value < 0
                            or not count_value.is_integer()):
                        raise ValueError
                    counts[step] = int(count_value)
                except (KeyError, TypeError, ValueError):
                    invalid_steps.append(step)
            nonzero_steps = [step for step, count in counts.items() if count > 0]
            nonphysical_points = {
                "available": True,
                "passed": len(counts) == len(steps) and not invalid_steps and not nonzero_steps,
                "maximum_count": max(counts.values()) if counts else None,
                "nonzero_time_indices_sample": nonzero_steps[:20],
                "invalid_time_indices_sample": invalid_steps[:20],
            }
        record: dict[str, Any] = {
            "zone": zone,
            "kind": "static_airframe_and_motor" if zone == 0 else "propeller_only_rotor_zone",
            "complete_time_steps": steps == expected_time_indices,
            "last_time_iteration": steps[-1],
            "sample_count_final_2_revolutions": len(sample_steps),
            "force_N": forces,
            "moment_Nm": moments,
            "last_revolution_force_change_fraction": drift,
            "initial_residual_logs": initial_residuals,
            "final_residual_logs": residuals,
            "pseudo_time_residuals_declined": residuals_declined,
            "nonphysical_points": nonphysical_points,
        }
        if zone:
            rotor = rotors[zone - 1]
            # Rotor-zone histories monitor only ROTOR_PROPELLER_<index>, so the
            # shifted moment excludes motor and stationary airframe loads.
            torque = moment_about_shaft_axis(
                (moments["x"], moments["y"], moments["z"]),
                (forces["x"], forces["y"], forces["z"]),
                (rotor.x_m, rotor.y_m, rotor.z_m),
                rotor.shaft_axis_xyz,
            )
            if last_force is None:
                torque_drift = None
            else:
                last_torque = moment_about_shaft_axis(
                    tuple(last_moment[axis] for axis in "xyz"),
                    tuple(last_force[axis] for axis in "xyz"),
                    (rotor.x_m, rotor.y_m, rotor.z_m),
                    rotor.shaft_axis_xyz,
                )
                previous_torque = moment_about_shaft_axis(
                    tuple(previous_moment[axis] for axis in "xyz"),
                    tuple(previous_force[axis] for axis in "xyz"),
                    (rotor.x_m, rotor.y_m, rotor.z_m),
                    rotor.shaft_axis_xyz,
                )
                torque_drift = abs(last_torque - previous_torque) / max(abs(torque), 0.02)
            record["shaft_torque_change_fraction"] = torque_drift
            record["rotor_station"] = {
                "x_m": rotor.x_m, "y_m": rotor.y_m, "z_m": rotor.z_m,
                "side": rotor.side,
                "station": getattr(rotor, "station_name", None),
                "hand": rotor.prop_hand,
                "spin_sign": rotor.spin_sign,
                "shaft_axis_xyz": rotor.shaft_axis_xyz,
                "thrust_direction_xyz": rotor.thrust_direction_xyz,
            }
            record["shaft_torque_Nm"] = torque
            record["shaft_power_W"] = -torque * rotor.spin_sign * rpm * 2 * math.pi / 60
            record["shaft_power_regime"] = shaft_power_regime(record["shaft_power_W"])
            record["shaft_torque_source_marker"] = f"ROTOR_PROPELLER_{rotor.index}"
        zone_records.append(record)

    complete = all(row["complete_time_steps"] for row in zone_records)
    if not complete:
        return {
            "topology": topology,
            "rotor_count": rotor_count,
            "expected_physical_steps": expected_steps,
            "history_complete": False,
            "numerically_converged": False,
            "numerical_convergence": {
                "status": "not_assessed",
                "accepted": False,
                "residuals_passed": False,
                "pseudo_time_residuals_declined": False,
                "nonphysical_points_passed": False,
                "coupling_residuals_passed": False,
                "load_stability_passed": False,
                "reasons": ["Cannot assess numerical convergence with incomplete time histories."],
            },
            "zone_forces": zone_records,
            "model_validated": False,
            "physical_model_valid": None,
            "design_ready": False,
            "sensitivity_evidence": {
                "status": "not_assessed",
                "mesh_sensitivity_assessed": False,
                "time_step_sensitivity_assessed": False,
            },
            "numerical_acceptance": {
                "status": "not_assessed",
                "accepted": False,
                "reason": "History is incomplete.",
            },
            "design_readiness_blockers": [
                "Histories are incomplete; no force averages are available.",
                "No mesh-sensitivity study is attached to this campaign result.",
                "No time-step-sensitivity study is attached to this campaign result.",
            ],
            "limitations": [
                "One or more zone histories are missing expected physical time indices; force averages are withheld.",
                "Process completion and numerical convergence are separate from history completeness.",
            ],
        }

    residual_values = [
        value
        for zone in zone_records
        for value in zone["final_residual_logs"].values()
    ]
    residuals_passed = bool(residual_values) and all(
        math.isfinite(value) and value <= MAX_FINAL_LOG10_RESIDUAL
        for value in residual_values
    )
    pseudo_time_residuals_declined = all(
        row.get("pseudo_time_residuals_declined", False) for row in zone_records
    )
    nonphysical_points_passed = all(
        row.get("nonphysical_points", {}).get("passed", False) for row in zone_records
    )
    coupling_residuals = summarize_coupling_residuals(
        case_dir, rotor_count + 1, expected_time_indices
    )
    coupling_residuals_passed = (
        len(coupling_residuals) == rotor_count + 1
        and all(row["passed"] for row in coupling_residuals)
    )
    force_changes = [
        change
        for zone in zone_records
        for change in zone["last_revolution_force_change_fraction"].values()
    ] + [zone["shaft_torque_change_fraction"] for zone in zone_records
         if "shaft_torque_change_fraction" in zone]
    expected_stability_values = len(zone_records) * 3 + rotor_count
    load_stability_passed = (
        len(force_changes) == expected_stability_values
        and all(
            change is not None
            and math.isfinite(change)
            and change <= MAX_FINAL_REVOLUTION_FORCE_CHANGE
            for change in force_changes
        )
    )
    convergence_reasons = []
    if not residual_values:
        convergence_reasons.append("Final residual history is missing.")
    elif not residuals_passed:
        convergence_reasons.append(
            f"One or more final log residuals exceed {MAX_FINAL_LOG10_RESIDUAL:g}."
        )
    if not pseudo_time_residuals_declined:
        convergence_reasons.append("Pseudo-time residuals did not decline in every zone.")
    if not nonphysical_points_passed:
        convergence_reasons.append(
            "Nonphysical-point history is missing, malformed, or contains nonzero counts."
        )
    if not coupling_residuals_passed:
        convergence_reasons.append(
            "Coupling residual histories are missing, incomplete, above threshold, or did not decline."
        )
    if len(force_changes) != expected_stability_values or any(value is None for value in force_changes):
        convergence_reasons.append("Final-revolution load stability could not be computed for every zone and axis.")
    elif not load_stability_passed:
        convergence_reasons.append(
            f"One or more final-revolution force changes exceed {MAX_FINAL_REVOLUTION_FORCE_CHANGE:.1%}."
        )
    numerically_converged = (
        residuals_passed and pseudo_time_residuals_declined
        and nonphysical_points_passed and coupling_residuals_passed and load_stability_passed
    )
    numerical_convergence = {
        "status": "passed" if numerically_converged else "failed",
        "accepted": False,
        "residuals_passed": residuals_passed,
        "pseudo_time_residuals_declined": pseudo_time_residuals_declined,
        "nonphysical_points_passed": nonphysical_points_passed,
        "coupling_residuals_passed": coupling_residuals_passed,
        "coupling_residuals": coupling_residuals,
        "load_stability_passed": load_stability_passed,
        "criteria": {
            "maximum_final_log10_residual": MAX_FINAL_LOG10_RESIDUAL,
            "maximum_final_revolution_force_change_fraction": MAX_FINAL_REVOLUTION_FORCE_CHANGE,
            "final_residuals_decline": True,
            "coupling_residuals_decline": True,
            "nonphysical_points_must_be_zero": True,
            "shaft_torque_window_stability": True,
        },
        "reasons": convergence_reasons,
    }
    total = {axis: sum(row["force_N"][axis] for row in zone_records) for axis in "xyz"}
    total_moment = {axis: sum(row["moment_Nm"][axis] for row in zone_records) for axis in "xyz"}
    alpha = math.radians(alpha_deg)
    drag = -total["x"] * math.cos(alpha) + total["z"] * math.sin(alpha)
    lift = total["x"] * math.sin(alpha) + total["z"] * math.cos(alpha)
    return {
        "topology": topology,
        "rotor_count": rotor_count,
        "expected_physical_steps": expected_steps,
        "history_complete": complete,
        "numerically_converged": numerically_converged,
        "numerical_convergence": numerical_convergence,
        "zone_forces": zone_records,
        "total_force_N": total,
        "total_moment_Nm_about_geometry_origin": total_moment,
        "upward_lift_N": lift,
        "positive_drag_N_including_prop_thrust": drag,
        "shaft_power_sum_W": sum(row.get("shaft_power_W", 0.0) for row in zone_records),
        "shaft_power_regime": shaft_power_regime(
            sum(row.get("shaft_power_W", 0.0) for row in zone_records)
        ),
        "model_validated": False,
        "physical_model_valid": None,
        "design_ready": False,
        "numerical_acceptance": {
            "status": "preliminary_only" if numerically_converged else "failed",
            "accepted": False,
            "reason": "Mesh and time-step sensitivity evidence has not been assessed.",
        },
        "sensitivity_evidence": {
            "status": "not_assessed",
            "mesh_sensitivity_assessed": False,
            "time_step_sensitivity_assessed": False,
        },
        "design_readiness_blockers": [
            "No mesh-sensitivity study is attached to this campaign result.",
            "No time-step-sensitivity study is attached to this campaign result.",
            "The Euler model and propeller/airframe geometry remain exploratory.",
        ],
        "limitations": [
            "Compressible inviscid Euler; no viscous drag, transition, or separation model.",
            "Coarse tetrahedral sliding-zone mesh; no prism layers or mesh/time sensitivity evidence.",
            "Stationary airframe and motor surfaces are excluded from moving rotor zones; intersecting geometry is rejected.",
            "Net streamwise force includes propeller thrust and airframe/rotor drag; no installed propeller map or trim/power balance.",
            "Force averages remain exploratory even when the numerical screening criteria pass.",
        ],
    }


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def export_topology_oml(params, design: Design, path: Path, geometry: str) -> Path:
    """Export a fused one-solid OML so Gmsh sees clean rotor-interface faces."""
    if design.topology == "tractor4" and geometry == "pr3_baseline":
        source = ROOT / "airframe_oml.step"
        shutil.copy2(source, path)
        return path
    components = build_variant_components(params, design)
    solid = components[0]
    for component in components[1:]:
        solid = solid.fuse(component)
    solid = _one_solid(solid, f"needletail_{design.topology}_oml")
    bd.export_step(solid, str(path))
    return path


def write_campaign_report(out_dir: Path, results: list[dict[str, Any]], config: dict[str, Any]) -> None:
    diagnostic_mode = bool(config.get("interference_diagnostic"))
    if diagnostic_mode:
        rows = []
        for result in results:
            geometry = result.get("geometry_preflight", {})
            minimum = geometry.get("minimum_clearance_mm")
            required = geometry.get("required_minimum_clearance_mm")
            minimum_text = f"{minimum:.4f}" if isinstance(minimum, (float, int)) else "n/a"
            required_text = f"{required:.4f}" if isinstance(required, (float, int)) else "n/a"
            stationary_gate = result.get("stationary_wall_gate_passed")
            stationary_text = "pass" if stationary_gate is True else (
                "fail" if stationary_gate is False else "not checked"
            )
            rotating_gate = result.get("rotating_zone_gate_passed")
            rotating_text = "pass" if rotating_gate is True else (
                "fail" if rotating_gate is False else "not checked"
            )
            failures = (len(geometry.get("failures", {}))
                        + int(stationary_gate is False) + int(rotating_gate is False))
            rows.append(
                f"| {result['topology']} | {result.get('status', 'failed')} | "
                f"{str(bool(result.get('cad_geometry_valid'))).lower()} | "
                f"{stationary_text} | {rotating_text} | {minimum_text} | "
                f"{required_text} | {failures} |"
            )
        report_title = "CAD interference diagnostic"
        results_table = (
            "Clearance is informational; this mode computes no flow solution and cannot make a candidate eligible for ranking.\n\n"
            "| Topology | Status | CAD valid | Stationary surfaces | Moving zones | Minimum clearance (mm) | Required clearance (mm) | Failures |\n"
            "|---|---|---:|---:|---:|---:|---:|---:|\n"
            + "\n".join(rows)
        )
        output_description = (
            "Per-topology CAD and geometry-only Gmsh preflight details are in the topology subdirectories. "
            "See campaign_results.json for structured results."
        )
        diagnostic_note = (
            "This campaign used --interference-diagnostic. It performs CAD clearance and "
            "geometry-only Gmsh intersection checks; it never creates a mesh or starts SU2 "
            "and is excluded from feasible rankings.\n\n"
        )
    else:
        rows = []
        for result in results:
            flow = result.get("aerodynamics", {})
            numerical_status = flow.get("numerical_convergence", {}).get("status", "not_assessed")
            rows.append(
                f"| {result['topology']} | {result.get('status', 'failed')} | "
                f"{numerical_status} | "
                f"{flow.get('upward_lift_N', float('nan')):.4f} | "
                f"{flow.get('positive_drag_N_including_prop_thrust', float('nan')):.4f} | "
                f"{flow.get('total_moment_Nm_about_geometry_origin', {}).get('y', float('nan')):.6f} | "
                f"{flow.get('shaft_power_sum_W', float('nan')):.3f} |"
            )
        report_title = "Powered Euler CFD screen"
        results_table = (
            "Drag is positive aft; lift is positive upward. Net streamwise force includes propeller thrust as well as drag.\n\n"
            "| Topology | Status | Numerical gate | Lift (N) | Net drag (N) | My about geometry origin (N·m) | Propeller shaft power sum (W) |\n"
            "|---|---|---:|---:|---:|---:|---:|\n"
            + "\n".join(rows)
        )
        output_description = (
            "Per-case mesh statistics, time histories, forces, residuals, and solver logs "
            "are in the topology subdirectories. See campaign_results.json for structured results."
        )
        diagnostic_note = ""
    solver_description = (
        "not invoked (CAD diagnostic only)"
        if diagnostic_mode
        else (
            "SU2 8.5.0, compressible inviscid Euler, "
            f"{config['steps_per_revolution']} time steps per revolution, "
            f"{config['revolutions']} revolutions per candidate"
        )
    )
    if diagnostic_mode:
        campaign_detail_lines = (
            "- Rotor stations, hands, shaft axes, thrust directions and signed spin rates come from the shared design contract.\n"
            "- CAD propeller-envelope clearance and Gmsh rotor-cylinder intersections with fixed airframe/motor geometry are reported.\n"
            "- No volume mesh, flow history, force, torque or power is generated."
        )
    else:
        campaign_detail_lines = (
            "- Rotor stations, hands, shaft axes, thrust directions and signed spin rates come from the shared design contract.\n"
            "- Fixed airframe or motor surfaces intersecting a rigid rotor zone cause a geometry preflight failure.\n"
            "- Rotor shaft torque is integrated from the propeller-only wall marker; motor and airframe loads are not included in that value.\n"
            f"- Preliminary numerical screen: final log residuals <= {MAX_FINAL_LOG10_RESIDUAL:g}, declining pseudo-time/coupling histories, zero nonphysical-point counts, and final-revolution force/shaft-torque change <= {MAX_FINAL_REVOLUTION_FORCE_CHANGE:.1%}.\n"
            f"- Mesh sizes: static near/far {config['static_near_size_m']*1000:.1f}/{config['static_far_size_m']*1000:.0f} mm; rotor near {config['prop_near_size_m']*1000:.1f} mm; sliding interface {config['interface_size_m']*1000:.1f} mm.\n"
            "- Per-zone force and moment histories are averaged over the final two revolutions. Output contains no large transient volume snapshots."
        )
    report = f"""# {report_title}: {', '.join(config['topologies'])}

{diagnostic_note}## Campaign conditions

- Geometry: {config['geometry']}; topology candidates: {', '.join(config['topologies'])}.
- Requested coaxial rotor-plane separation: {config.get('coaxial_separation_mm') if config.get('coaxial_separation_mm') is not None else 'not configured'} mm.
- Requested operating point: {config['rpm']:.0f} RPM, {config['speed_mps']:.1f} m/s, aircraft alpha +{config['alpha_deg']:.1f}°, sea level.
- Solver: {solver_description}.
{campaign_detail_lines}

## Results

{results_table}

{'All flow results are exploratory and `feasible_for_ranking` remains false. Passing the numerical screen does not establish design readiness: mesh and time-step sensitivity evidence, viscous validation and propulsion-map integration are still absent. CAD clearance and stationary-zone intersection failures stop a case before the solver starts. Stationary-zone intersection is reported as `UNSUPPORTED_GEOMETRY`.' if not diagnostic_mode else ''}

{output_description}
"""
    (out_dir / "REPORT.md").write_text(report, encoding="utf-8")


def run_campaign(args: argparse.Namespace) -> int:
    coaxial_separation_mm = getattr(args, "coaxial_separation_mm", None)
    interference_diagnostic = bool(getattr(args, "interference_diagnostic", False))
    topologies = getattr(args, "topologies", [])
    validate_coaxial_separation(topologies, coaxial_separation_mm)
    out_dir = args.output_dir.resolve()
    if out_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing CFD campaign: {out_dir}")
    params = airframe_parameters_for_geometry(args.geometry)
    executable = None
    if not interference_diagnostic:
        executable = Path(os.environ.get("SU2_RUN", "")) / "SU2_CFD.exe"
        if not executable.is_file():
            executable = Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/SU2/8.5.0/bin/SU2_CFD.exe"
        if not executable.is_file():
            raise FileNotFoundError(f"SU2_CFD.exe not found: {executable}")
        os.environ["SU2_RUN"] = str(executable.parent)
        os.environ["PATH"] = str(executable.parent) + os.pathsep + os.environ.get("PATH", "")
    q = 0.5 * RHO * args.speed_mps**2
    _, area, mac = geometry_values(args.geometry)
    config_meta = {
        "geometry": args.geometry,
        "topologies": topologies,
        "coaxial_separation_mm": coaxial_separation_mm,
        "rpm": args.rpm,
        "speed_mps": args.speed_mps,
        "alpha_deg": args.alpha_deg,
        "revolutions": args.revolutions,
        "steps_per_revolution": args.steps_per_revolution,
        "static_near_size_m": args.static_near_size_m,
        "static_far_size_m": args.static_far_size_m,
        "prop_near_size_m": args.prop_near_size_m,
        "interface_size_m": args.interface_size_m,
        "interference_diagnostic": interference_diagnostic,
        "feasible_ranking_enabled": False,
        "threads": 4,
        "reference_dynamic_pressure_Pa": q,
        "reference_area_m2": area,
        "reference_length_m": mac,
        "solver_executable": str(executable) if executable else None,
        "solver_sha256": digest(executable) if executable else None,
        "airframe_geometry_sha256": digest(ROOT / "airframe_geometry.py"),
        "mesh_generator_sha256": digest(CFD_DIR / "generate_rotor_mesh.py"),
        "case_config_generator_sha256": digest(CFD_DIR / "configure_rotor_case.py"),
        "cad_variant_builder_sha256": digest(ROOT / "design_cad.py"),
        "design_station_contract_sha256": digest(ROOT / "design_configuration.py"),
        "campaign_source_sha256": digest(Path(__file__)),
    }
    out_dir.mkdir(parents=True)
    (out_dir / "campaign_configuration.json").write_text(
        json.dumps(config_meta, indent=2) + "\n", encoding="utf-8"
    )
    results: list[dict[str, Any]] = []
    for topology in topologies:
        case_dir = out_dir / topology
        case_dir.mkdir()
        result: dict[str, Any] = {
            "topology": topology,
            "case_dir": str(case_dir),
            "cad_geometry_valid": False,
            "geometry_valid": False,
            "model_validated": False,
            "physical_model_valid": None,
            "design_ready": False,
            "interference_diagnostic": interference_diagnostic,
            "feasible_for_ranking": False,
            "stationary_wall_gate_passed": None,
            "rotating_zone_gate_passed": None,
            "mesh_valid": False,
            "solver_exited_successfully": False,
            "solver_process_complete": False,
            "history_complete": False,
            "numerically_converged": False,
        }
        try:
            if not interference_diagnostic:
                if available_memory_bytes() < MIN_RUNTIME_MEMORY_BYTES:
                    raise RuntimeError("Less than 0.8 GiB RAM available before case setup")
                if shutil.disk_usage(case_dir).free < MIN_RUNTIME_DISK_BYTES:
                    raise RuntimeError("Less than 1.5 GiB disk free before case setup")
            design = Design(
                topology=topology,
                flaps="none",
                coaxial_separation_mm=coaxial_separation_mm,
            )
            rotors = _rotors(params, topology, coaxial_separation_mm)
            result["rotor_stations"] = [
                {"index": rotor.index, "x_m": rotor.x_m, "y_m": rotor.y_m,
                 "z_m": rotor.z_m, "side": rotor.side,
                 "station": rotor.station_name, "hand": rotor.prop_hand,
                 "spin_sign": rotor.spin_sign,
                 "shaft_axis_xyz": rotor.shaft_axis_xyz,
                 "thrust_direction_xyz": rotor.thrust_direction_xyz}
                for rotor in rotors
            ]
            geometry = geometry_preflight(params, design)
            result["geometry_preflight"] = geometry
            result["cad_geometry_valid"] = geometry["valid"]
            result["geometry_valid"] = geometry["valid"]
            if not geometry["valid"]:
                raise CADPreflightError(
                    "CAD geometry preflight failed minimum propeller clearance: "
                    + ", ".join(geometry["failures"])
                )
            airframe_step = case_dir / "airframe_variant.step"
            export_topology_oml(params, design, airframe_step, args.geometry)
            result["airframe_step_sha256"] = digest(airframe_step)
            if interference_diagnostic:
                result["rotor_zone_geometry_preflight"] = validate_rotor_zone_geometry(
                    params, airframe_step, topology, coaxial_separation_mm
                )
                result["stationary_wall_gate_passed"] = True
                result["rotating_zone_gate_passed"] = True
                raise DiagnosticOnlyStop(
                    "CAD and Gmsh geometry checks passed; diagnostic mode skipped meshing and SU2."
                )

            mesh_path = case_dir / "rotor_domain.su2"
            nodes, cells = generate_mesh(
                output_path=mesh_path,
                static_near_size_m=args.static_near_size_m,
                static_far_size_m=args.static_far_size_m,
                prop_near_size_m=args.prop_near_size_m,
                interface_size_m=args.interface_size_m,
                params=params,
                airframe_step=airframe_step,
                topology=topology,
                coaxial_separation_mm=coaxial_separation_mm,
            )
            result["stationary_wall_gate_passed"] = True
            result["rotating_zone_gate_passed"] = True
            rotor_count = len(rotors)
            mesh_zones = validate_multizone_mesh(mesh_path, rotor_count=rotor_count)
            result["mesh_valid"] = True
            result["mesh"] = {
                "path": str(mesh_path), "zone_count": rotor_count + 1,
                "zone_local_nodes": nodes, "tetrahedra": cells,
                "minimum_tetra_quality": min(
                    zone["minimum_tetra_quality"] for zone in mesh_zones
                ),
                "zones": mesh_zones,
                "sha256": digest(mesh_path),
            }
            config, _, time_levels = write_configs(
                case_dir,
                rpm=args.rpm,
                revolutions=args.revolutions,
                steps_per_revolution=args.steps_per_revolution,
                params=params,
                geometry=args.geometry,
                topology=topology,
                coaxial_separation_mm=coaxial_separation_mm,
                alpha_deg=args.alpha_deg,
                speed_mps=args.speed_mps,
            )
            configure_histories(case_dir, rotor_count, time_levels - 1)
            result["configuration_sha256"] = digest(config)
            run_solver(case_dir, "rotor_case.cfg", args.max_hours_per_case, dry=True)
            run_solver(case_dir, "rotor_case.cfg", args.max_hours_per_case)
            result["solver_exited_successfully"] = True
            result["solver_process_complete"] = True
            flow = summarize_history(
                case_dir, topology, params, args.revolutions,
                args.steps_per_revolution, args.rpm, args.speed_mps, args.alpha_deg,
                coaxial_separation_mm,
            )
            result["aerodynamics"] = flow
            result["history_complete"] = flow["history_complete"]
            result["numerically_converged"] = flow["numerically_converged"]
            result["status"] = "diagnostic_only" if interference_diagnostic else (
                "incomplete_history" if not flow["history_complete"] else
                "completed_exploratory" if flow["numerically_converged"] else
                "completed_unconverged"
            )
            result["solver_exit"] = "success"
        except CADPreflightError as exc:
            result["status"] = "interference_diagnostic" if interference_diagnostic else "invalid_cad_geometry"
            result["failure_type"] = "cad_interference"
            result["error"] = f"{type(exc).__name__}: {exc}"
            result["solver_skipped_reason"] = "CAD clearance gate failed before mesh generation."
        except DiagnosticOnlyStop as exc:
            result["status"] = "clearance_passed_diagnostic"
            result["solver_skipped_reason"] = str(exc)
        except (StationaryZoneIntersectionError, RotorZoneOverlapError) as exc:
            if isinstance(exc, StationaryZoneIntersectionError):
                result["stationary_wall_gate_passed"] = False
                result["rotating_zone_gate_passed"] = True
                result["failure_type"] = "stationary_zone_intersection"
            else:
                result["rotating_zone_gate_passed"] = False
                result["failure_type"] = "rotating_zone_overlap"
            result["geometry_valid"] = False
            result["rotor_zone_geometry_preflight"] = {"passed": False, "error": str(exc)}
            result["status"] = "interference_diagnostic" if interference_diagnostic else "UNSUPPORTED_GEOMETRY"
            result["error"] = f"{type(exc).__name__}: {exc}"
            result["solver_skipped_reason"] = str(exc)
        except Exception as exc:
            result["status"] = "failed"
            result["failure_type"] = type(exc).__name__
            result["error"] = f"{type(exc).__name__}: {exc}"
        (case_dir / "case_results.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        results.append(result)
        (out_dir / "campaign_results.json").write_text(
            json.dumps({"configuration": config_meta, "cases": results}, indent=2) + "\n",
            encoding="utf-8",
        )
        write_campaign_report(out_dir, results, config_meta)
        print(f"{topology}: {result['status']}", flush=True)
        if result.get("error"):
            print(result["error"], flush=True)
    success_status = "clearance_passed_diagnostic" if interference_diagnostic else "completed_exploratory"
    return 0 if all(row["status"] == success_status for row in results) else 1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", choices=("pr3_baseline", "airfoil_candidate"), default="pr3_baseline")
    parser.add_argument("--topologies", nargs="+", choices=TOPOLOGIES, default=list(TOPOLOGIES))
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "tmp" / "topology-cfd-screen-20261027")
    parser.add_argument("--rpm", type=float, default=40_000.0)
    parser.add_argument("--speed-mps", type=float, default=15.0)
    parser.add_argument("--alpha-deg", type=float, default=4.0)
    parser.add_argument("--revolutions", type=int, default=10)
    parser.add_argument("--steps-per-revolution", type=int, default=120)
    parser.add_argument("--coaxial-separation-mm", type=float)
    parser.add_argument(
        "--interference-diagnostic", action="store_true",
        help="Write CAD clearance diagnostics only; skip meshing/SU2 and exclude results from rankings.",
    )
    parser.add_argument("--static-near-size-m", type=float, default=0.004)
    parser.add_argument("--static-far-size-m", type=float, default=0.12)
    parser.add_argument("--prop-near-size-m", type=float, default=0.0018)
    parser.add_argument("--interface-size-m", type=float, default=0.003)
    parser.add_argument("--max-hours-per-case", type=float, default=4.0)
    args = parser.parse_args()
    if args.revolutions < 2 or args.steps_per_revolution < 12:
        parser.error("Use at least 2 revolutions and 12 steps per revolution for a force comparison")
    if len(set(args.topologies)) != len(args.topologies):
        parser.error("Each topology can appear only once")
    try:
        validate_coaxial_separation(args.topologies, args.coaxial_separation_mm)
    except ValueError as exc:
        parser.error(str(exc))
    raise SystemExit(run_campaign(args))


if __name__ == "__main__":
    main()
