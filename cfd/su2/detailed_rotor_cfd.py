"""Finite, resource-bounded powered URANS experiment and numerical-quality report.

This is an exploratory wall-modelled tetrahedral calculation, not a certified
wall-resolved calculation. No force result is labelled design-ready.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
from dataclasses import asdict
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CFD_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from airframe_geometry import (
    PARAMETERS,
    AirframeParameters,
    airframe_parameters_for_geometry,
)
import configure_rotor_case
from design_configuration import moment_about_shaft_axis

SPEED = 15.0
ALPHA = 4.0
RPM = 40_000.0
RHO = 101325.0 / (287.058 * 288.15)
SOUND = math.sqrt(1.4 * 287.058 * 288.15)
Q = 0.5 * RHO * SPEED**2
AREA = PARAMETERS.wing_planform_area * 1e-6
TAPER = PARAMETERS.wing_tip_chord / PARAMETERS.wing_root_chord
MAC = 2 / 3 * PARAMETERS.wing_root_chord * 1e-3 * (1 + TAPER + TAPER**2) / (1 + TAPER)
MIN_START_DISK_BYTES = 6 * 1024**3
MIN_RUNTIME_MEMORY_BYTES = int(0.8 * 1024**3)
MIN_RUNTIME_DISK_BYTES = int(1.5 * 1024**3)


def geometry_values(geometry: str) -> tuple[AirframeParameters, float, float]:
    params = airframe_parameters_for_geometry(geometry)
    taper = params.wing_tip_chord / params.wing_root_chord
    mac = (2 / 3 * params.wing_root_chord * 1e-3
           * (1 + taper + taper**2) / (1 + taper))
    return params, params.wing_planform_area * 1e-6, mac


def geometry_step(geometry: str) -> Path:
    filename = "airframe_oml.step" if geometry == "pr3_baseline" else "airframe_candidate_oml.step"
    return ROOT / filename


def source_paths(geometry: str) -> list[Path]:
    # Hash both source inputs and the exact geometry artifact consumed by Gmsh.
    mesher_path = CFD_DIR / "generate_rotor_mesh.py"
    motor_step = ROOT / "references" / "propeller-kit" / "STEP" / "iFlight_XING2_1404_4600KV_REFERENCE.step"
    prop_steps = [ROOT / "references" / "propeller-kit" / "STEP" / name for name in (
        "PROP_3x4_2B_CW.step", "PROP_3x4_2B_CCW.step")]
    return [ROOT / "airframe_geometry.py", ROOT / "profiles" / "airfoil_coordinates.py",
            geometry_step(geometry), motor_step, *prop_steps,
            Path(__file__), mesher_path, Path(configure_rotor_case.__file__)]


def dump(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def hashes(geometry: str) -> dict[str, str]:
    paths = source_paths(geometry)
    missing = [path for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("Missing campaign source inputs: " + ", ".join(map(str, missing)))
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


def campaign_configuration(geometry: str, revolutions: int,
                           hours_per_case: float) -> dict:
    params, area, mac = geometry_values(geometry)
    return {
        "geometry": geometry,
        "parameters": asdict(params),
        "operating_point": {"rpm": RPM, "speed_mps": SPEED, "alpha_deg": ALPHA,
                            "pressure_pa": 101325.0, "temperature_k": 288.15},
        "revolutions_per_case": revolutions,
        "hours_per_case": hours_per_case,
        "reference_area_m2": area,
        "reference_length_m": mac,
        "mesh_levels": [1.0, 0.75],
        "time_steps_per_revolution": [120, 120, 240],
        "moving_surface_contract": "propeller-only; stationary airframe and motor intersections fail preflight",
        "resources": {"omp_threads": 4, "max_zone_local_nodes": 700_000,
                      "max_cells": 4_000_000,
                      "minimum_start_disk_bytes": MIN_START_DISK_BYTES,
                      "minimum_runtime_memory_bytes": MIN_RUNTIME_MEMORY_BYTES,
                      "minimum_runtime_disk_bytes": MIN_RUNTIME_DISK_BYTES},
        "design_ready": False,
    }


def digest_json(value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         allow_nan=False).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def config_files_digest(directory: Path) -> str:
    paths = sorted([*directory.glob("*.cfg"), directory / "rotor_run_parameters.txt"])
    if len(paths) != 7 or any(not path.is_file() for path in paths):
        raise FileNotFoundError(f"Expected all seven prepared SU2 configuration files in {directory}")
    values = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    return digest_json(values)


def verify_prepared_campaign(plan: dict, geometry: str, revolutions: int,
                             hours_per_case: float, solver_digest: str,
                             preflight_config_digest: str) -> None:
    if plan.get("geometry") != geometry:
        raise RuntimeError("Selected geometry does not match the prepared campaign")
    if plan.get("source_sha256") != hashes(geometry):
        raise RuntimeError("Geometry or runner changed after preparation; prepare a new campaign")
    if plan.get("solver_sha256") != solver_digest:
        raise RuntimeError("SU2 executable changed after preparation; prepare a new campaign")
    if revolutions != plan.get("revolutions") or hours_per_case != plan.get("hours_per_case"):
        raise RuntimeError("Command does not match the prepared run budget")
    configuration = campaign_configuration(geometry, revolutions, hours_per_case)
    expected_configuration_hash = digest_json(configuration)
    if (plan.get("configuration_sha256") != expected_configuration_hash
            or digest_json(plan.get("configuration")) != expected_configuration_hash):
        raise RuntimeError("Campaign configuration changed after preparation")
    if plan.get("preflight_config_sha256") != preflight_config_digest:
        raise RuntimeError("Prepared SU2 configuration files changed after preparation")


def assert_fresh_campaign_outputs(directory: Path) -> None:
    output_names = ("medium", "fine", "fine_half_dt", "tests.log", "progress.json",
                    "campaign_results.json", "REPORT.md")
    existing = [name for name in output_names if (directory / name).exists()]
    if existing:
        raise FileExistsError(
            "Refusing to overwrite existing campaign outputs: " + ", ".join(existing)
        )


def solver() -> Path:
    location = os.environ.get("SU2_RUN") or str(
        Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/SU2/8.5.0/bin")
    executable = Path(location) / "SU2_CFD.exe"
    if not executable.is_file():
        raise FileNotFoundError(executable)
    return executable


def set_keys(text: str, updates: dict[str, str]) -> str:
    rows = []
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


def write_case(directory: Path, steps: int, revolutions: int,
               mesh: str = "rotor_domain.su2",
               geometry: str = "pr3_baseline") -> Path:
    params, area, mac = geometry_values(geometry)
    config, _, _ = configure_rotor_case.write_configs(
        directory, RPM, revolutions, steps, params=params, geometry=geometry
    )
    # SU2 uses vx=U*cos(AOA), vz=U*sin(AOA). With nose +X and z up,
    # aircraft alpha +4 degrees means wind velocity (-cos(4), 0, +sin(4)).
    # Thus AOA=176, and SU2's reported LIFT points downward. Use XYZ forces.
    updates = {
        "SOLVER": "RANS", "KIND_TURB_MODEL": "SST", "SST_OPTIONS": "V2003m",
        "MACH_NUMBER": f"{SPEED / SOUND:.12g}",
        # Moving zones otherwise normalize forces with the default MACH_MOTION=.8.
        "MACH_MOTION": f"{SPEED / SOUND:.12g}", "AOA": str(180 - ALPHA),
        "MESH_FILENAME": mesh,
        "VISCOSITY_MODEL": "CONSTANT_VISCOSITY", "MU_CONSTANT": "1.7894e-5",
        "FREESTREAM_TURBULENCEINTENSITY": "0.01",
        "FREESTREAM_TURB2LAMVISCRATIO": "10.0",
        "REF_AREA": f"{area:.12g}", "REF_LENGTH": f"{mac:.12g}",
        "REF_ORIGIN_MOMENT_X": "0.0", "REF_ORIGIN_MOMENT_Y": "0.0",
        "REF_ORIGIN_MOMENT_Z": "0.0",
        "OUTER_ITER": "10", "INNER_ITER": "5", "CFL_NUMBER": "0.5",
        "CFL_ADAPT": "NO", "MUSCL_FLOW": "YES", "MUSCL_TURB": "YES",
        "CONV_NUM_METHOD_TURB": "SCALAR_UPWIND", "TIME_DISCRE_TURB": "EULER_IMPLICIT",
        "CONV_FIELD": "RMS_DENSITY", "CONV_RESIDUAL_MINVAL": "-7",
        "CONV_STARTITER": "5", "LINEAR_SOLVER_ITER": "10",
        "HISTORY_OUTPUT": "( ITER, TIME_DOMAIN, RMS_RES, AERO_COEFF, NONPHYSICAL_POINTS )",
        "HISTORY_WRT_FREQ_INNER": "5", "HISTORY_WRT_FREQ_OUTER": "1",
        "HISTORY_WRT_FREQ_TIME": "1",
        "SCREEN_OUTPUT": "( TIME_ITER, OUTER_ITER, RMS_DENSITY, DRAG, LIFT )",
        "SCREEN_WRT_FREQ_INNER": "5", "SCREEN_WRT_FREQ_TIME": "10",
        "VOLUME_OUTPUT": "( COORDINATES, SOLUTION, PRIMITIVE )",
        "OUTPUT_WRT_FREQ": f"( {steps * 20}, {steps * 20}, {steps * 5} )",
        "WRT_RESTART_OVERWRITE": "YES", "WRT_VOLUME_OVERWRITE": "YES",
        "WRT_SURFACE_OVERWRITE": "YES", "WRT_PERFORMANCE": "YES",
    }
    config.write_text(set_keys(config.read_text(encoding="utf-8"), updates), encoding="utf-8")
    for index in range(5):
        path = directory / ("zone_static.cfg" if index == 0 else f"zone_rotor_{index}.cfg")
        marker = "AIRFRAME_MOTOR" if index == 0 else f"ROTOR_PROPELLER_{index}"
        text = path.read_text(encoding="utf-8").replace(
            f"MARKER_EULER= ( {marker} )", f"MARKER_HEATFLUX= ( {marker}, 0.0 )")
        text += (f"MARKER_WALL_FUNCTIONS= ( {marker}, STANDARD_WALL_FUNCTION )\n"
                 "WALLMODEL_MINYPLUS= 5.0\n")
        # Per-zone output selections are not inherited by SU2's multizone driver.
        for key in ("HISTORY_OUTPUT", "VOLUME_OUTPUT", "SCREEN_OUTPUT"):
            text += f"{key}= {updates[key]}\n"
        text += "CONV_FIELD= RMS_DENSITY\nCONV_RESIDUAL_MINVAL= -7\n"
        path.write_text(text, encoding="utf-8")
    zone_groups = [f"{group}[{zone}]" for zone in range(5)
                   for group in ("BGS_RES", "RMS_RES", "AERO_COEFF")]
    driver_updates = {
        "MULTIZONE_RESIDUAL": "YES", "WRT_ZONE_HIST": "YES",
        "HISTORY_OUTPUT": "( ITER, TIME_DOMAIN, " + ", ".join(zone_groups) + " )",
        "SCREEN_OUTPUT": "( TIME_ITER, OUTER_ITER, AVG_BGS_RES[0], AVG_BGS_RES[1] )",
        "CONV_FIELD": "( " + ", ".join(f"AVG_BGS_RES[{i}]" for i in range(5)) + " )",
        "CONV_RESIDUAL_MINVAL": "-5",
    }
    config.write_text(set_keys(config.read_text(encoding="utf-8"), driver_updates), encoding="utf-8")
    (directory / "rotor_run_parameters.txt").write_text(
        f"Exploratory compressible URANS, SST V2003m, standard wall functions.\n"
        f"Geometry: {geometry}; four real propeller STEP blades: {RPM:g} RPM, {revolutions} revolutions.\n"
        f"{SPEED:g} m/s, aircraft alpha={ALPHA:g} deg, SU2 AOA={180-ALPHA:g} deg.\n"
        f"{steps} steps/revolution; dt={60/RPM/steps:.12g} s.\n"
        "Adiabatic smooth walls; fully turbulent, no transition model.\n"
        "Only propeller walls move; airframe and motor geometry remain stationary. "
        "Any rotor-zone intersection with stationary geometry is rejected before meshing.\n"
        "Coefficients use the same freestream dynamic pressure in all zones.\n"
        "Use physical XYZ forces; SU2 LIFT has reversed sign in this orientation.\n"
        "Tetrahedral wall-function mesh; no prism layers, no design-ready claim.\n",
        encoding="utf-8")
    return config


def available_memory_bytes() -> int:
    if os.name == "nt":
        class MemoryStatus(ctypes.Structure):
            _fields_ = [
                ("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong),
                ("ullTotalPhys", ctypes.c_ulonglong), ("ullAvailPhys", ctypes.c_ulonglong),
                ("ullTotalPageFile", ctypes.c_ulonglong), ("ullAvailPageFile", ctypes.c_ulonglong),
                ("ullTotalVirtual", ctypes.c_ulonglong), ("ullAvailVirtual", ctypes.c_ulonglong),
                ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
            ]
        status = MemoryStatus()
        status.dwLength = ctypes.sizeof(status)
        if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            raise OSError("Windows could not report available physical memory")
        return int(status.ullAvailPhys)
    return int(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_AVPHYS_PAGES"))


def run_solver(directory: Path, config: str, limit_hours: float, dry: bool = False) -> None:
    log = directory / ("config_check.log" if dry else "solver.log")
    if log.exists():
        raise FileExistsError(f"Refusing to overwrite solver log: {log}")
    if available_memory_bytes() < MIN_RUNTIME_MEMORY_BYTES:
        raise RuntimeError("Less than 0.8 GiB RAM available; stopped before starting SU2")
    if shutil.disk_usage(directory).free < MIN_RUNTIME_DISK_BYTES:
        raise RuntimeError("Less than 1.5 GiB disk free; stopped before starting SU2")
    executable = solver()
    env = os.environ.copy()
    env["PATH"] = str(executable.parent) + os.pathsep + env["PATH"]
    env["OMP_NUM_THREADS"] = "4"
    argv = [str(executable), *(["-d"] if dry else []), config]
    started = time.monotonic()
    with log.open("w", encoding="utf-8") as stream:
        child = subprocess.Popen(argv, cwd=directory, env=env, stdout=stream,
                                 stderr=subprocess.STDOUT)
        try:
            while child.poll() is None:
                if time.monotonic() - started > limit_hours * 3600:
                    raise RuntimeError(f"Solver wall-time budget reached; see {log}")
                if available_memory_bytes() < MIN_RUNTIME_MEMORY_BYTES:
                    raise RuntimeError("Less than 0.8 GiB RAM available; stopped to preserve desktop")
                if shutil.disk_usage(directory).free < MIN_RUNTIME_DISK_BYTES:
                    raise RuntimeError("Less than 1.5 GiB disk free; stopped before filling disk")
                time.sleep(1)
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait()
    if child.returncode:
        raise RuntimeError(f"SU2 exited {child.returncode}; see {log}")
    if "Exit Success" not in log.read_text(encoding="utf-8", errors="replace")[-6000:]:
        raise RuntimeError(f"SU2 did not report normal completion; see {log}")


def make_mesh(directory: Path, scale: float, geometry: str = "pr3_baseline") -> dict:
    """Use the existing robust CAD booleans; add quality gates and wake sizing."""
    if str(CFD_DIR) not in sys.path:
        sys.path.insert(0, str(CFD_DIR))
    mesher = importlib.import_module("generate_rotor_mesh")
    params, _, _ = geometry_values(geometry)
    original_mesh_zone = mesher._mesh_one_zone
    original_fields = mesher._set_distance_field
    metrics = []

    def fields(groups, minimum_size, maximum_size):
        original_fields(groups, minimum_size, maximum_size)
        # IDs of all current threshold fields belong to this fresh zone model.
        current = list(mesher.gmsh.model.mesh.field.list())
        wake = mesher.gmsh.model.mesh.field.add("Box")
        for key, value in {"VIn": 0.010 * scale, "VOut": maximum_size,
                           "XMin": -0.55, "XMax": 0.08,
                           "YMin": -0.145, "YMax": 0.145,
                           "ZMin": -0.13, "ZMax": 0.13,
                           "Thickness": 0.04}.items():
            mesher.gmsh.model.mesh.field.setNumber(wake, key, value)
        combined = mesher.gmsh.model.mesh.field.add("Min")
        mesher.gmsh.model.mesh.field.setNumbers(combined, "FieldsList", [current[-1], wake])
        mesher.gmsh.model.mesh.field.setAsBackgroundMesh(combined)

    def mesh_zone(*args, **kwargs):
        stats = original_mesh_zone(*args, **kwargs)
        tags = mesher.gmsh.model.mesh.getElements(3)[1]
        flat = np.concatenate(tags)
        quality = np.asarray(mesher.gmsh.model.mesh.getElementQualities(flat, "minSICN"))
        record = {"zone": args[0], "nodes": stats[0], "tetrahedra": stats[1],
                  "min_SICN": float(np.min(quality)),
                  "p01_SICN": float(np.quantile(quality, .01)),
                  "nonpositive_quality_cells": int(np.count_nonzero(quality <= 0)),
                  "nonfinite_quality_cells": int(np.count_nonzero(~np.isfinite(quality)))}
        metrics.append(record)
        dump(directory / "mesh_quality.json", metrics)
        if record["nonpositive_quality_cells"] or record["nonfinite_quality_cells"]:
            raise RuntimeError(f"Invalid cell Jacobian/quality: {record}")
        if sum(row["nodes"] for row in metrics) > 700_000:
            raise RuntimeError("Mesh exceeds this machine's 700,000-node budget")
        if sum(row["tetrahedra"] for row in metrics) > 4_000_000:
            raise RuntimeError("Mesh exceeds this machine's 4-million-cell budget")
        return stats

    mesher._mesh_one_zone = mesh_zone
    mesher._set_distance_field = fields
    try:
        nodes, cells = mesher.generate_mesh(
            directory / "rotor_domain.su2", domain_padding_m=1.0,
            static_near_size_m=.0018 * scale, static_far_size_m=.12,
            prop_near_size_m=.0008 * scale, interface_size_m=.002 * scale,
            params=params, airframe_step=geometry_step(geometry))
    finally:
        mesher._mesh_one_zone = original_mesh_zone
        mesher._set_distance_field = original_fields
    result = {"nodes": nodes, "tetrahedra": cells, "zones": metrics,
              "near_sizes_m": {"airframe": .0018 * scale, "prop": .0008 * scale,
                               "interface": .002 * scale, "wake": .010 * scale},
              "prism_layers": 0, "wall_treatment": "standard wall functions"}
    dump(directory / "mesh_summary.json", result)
    return result


def read_table(path: Path) -> dict[str, np.ndarray]:
    with path.open(encoding="utf-8-sig") as stream:
        rows = csv.DictReader(stream, skipinitialspace=True)
        data = list(rows)
    if not data:
        return {}
    result = {}
    for key in data[0]:
        try:
            result[key.strip().strip('"')] = np.asarray([float(row[key]) for row in data])
        except (ValueError, TypeError):
            continue
    return result


def find_column(table: dict, name: str) -> np.ndarray:
    for key, column in table.items():
        if key == name or key.lower().replace("_", "") == name.lower().replace("_", ""):
            return column
    raise KeyError(f"Missing {name}; columns={list(table)}")


def multizone_history_path(directory: Path) -> Path | None:
    """Return the configured SU2 multizone history, allowing older output names."""
    candidates = (
        directory / "rotor_history_multizone.csv",
        directory / "rotor_history.csv",
        directory / "rotor_case.csv",
    )
    return next((path for path in candidates if path.is_file()), None)


def is_bgs_residual_column(name: str) -> bool:
    normalized = name.lower().replace("_", "").replace(" ", "")
    return normalized.startswith((
        "bgs[", "bgsres[", "bgsresidual[",
        "avgbgs[", "avgbgsres[", "avgbgsresidual[",
    ))


def summarize_case(directory: Path, steps: int, revolutions: int,
                   geometry: str = "pr3_baseline") -> dict:
    # Multizone writes separate zone histories as well as a global history.
    params, area, mac = geometry_values(geometry)
    rotors = configure_rotor_case._rotors(params)
    expected_steps = steps * revolutions
    all_zones_complete = True
    result = {"status": "incomplete", "run_complete": False, "geometry": geometry,
              "expected_physical_steps": expected_steps, "zone_forces": [], "issues": [],
              "design_ready": False}
    for zone in range(5):
        candidates = [directory / f"rotor_history_{zone}.csv",
                      directory / f"history_{zone}.csv"]
        path = next((p for p in candidates if p.is_file()), None)
        if path is None:
            # Driver history suffixes every zone-specific column with [zone].
            driver = multizone_history_path(directory)
            if driver is None:
                result["issues"].append(f"Zone {zone} history missing; force extraction unavailable")
                all_zones_complete = False
                continue
            suffix = f"[{zone}]"
            table = {k[:-len(suffix)] if k.endswith(suffix) else k: v
                     for k, v in read_table(driver).items()
                     if k.endswith(suffix) or k in ("Time_Iter", "Outer_Iter")}
        else:
            table = read_table(path)
        t = find_column(table, "Time_Iter").astype(int)
        if not len(t):
            result["issues"].append(f"Zone {zone}: history contains no iterations")
            all_zones_complete = False
            continue
        # Keep the last pseudo-time/coupling iterate for each physical step.
        last = np.r_[np.flatnonzero(t[1:] != t[:-1]), len(t)-1]
        t = t[last]
        data = {key: val[last] for key, val in table.items()}
        if t[-1] != expected_steps:
            result["issues"].append(
                f"Zone {zone}: physical run incomplete ({t[-1]} of {expected_steps} steps)"
            )
            all_zones_complete = False
            result["zone_forces"].append({"zone": zone, "history_complete": False,
                                          "last_time_iteration": int(t[-1])})
            continue
        mask = t > (revolutions - 10) * steps
        if np.count_nonzero(mask) < steps * 8:
            result["issues"].append(f"Zone {zone}: fewer than eight sampled revolutions")
        record = {"zone": zone, "history_complete": True,
                  "last_time_iteration": int(t[-1]), "force_N": {},
                  "moment_Nm": {}, "last_5rev_vs_previous_5rev_relative_change": {}}
        for axis in "xyz":
            forces = find_column(data, f"CF{axis}") * Q * area
            moments = find_column(data, f"CM{axis}") * Q * area * mac
            if not np.isfinite(forces).all() or not np.isfinite(moments).all():
                raise RuntimeError("Nonfinite force/moment history")
            record["force_N"][axis] = float(np.mean(forces[mask]))
            record["moment_Nm"][axis] = float(np.mean(moments[mask]))
            late = forces[t > (revolutions-5) * steps]
            early = forces[(t > (revolutions-10)*steps) & (t <= (revolutions-5)*steps)]
            drift = abs(float(np.mean(late) - np.mean(early))) / max(
                abs(record["force_N"][axis]), .02)
            record["last_5rev_vs_previous_5rev_relative_change"][axis] = drift
            if drift > .02:
                result["issues"].append(f"Zone {zone}: {axis}-force window drift >2%")
        residuals = {key: float(np.max(val[mask])) for key, val in data.items()
                     if key.lower().startswith("rms") and np.any(mask)}
        record["worst_final_step_log_residual_in_sample"] = residuals
        if not residuals or max(residuals.values()) > -4:
            result["issues"].append(f"Zone {zone}: pseudo-time residuals have not all reached -4")
        record["y_plus"] = yplus(directory, zone)
        if record["y_plus"] is None:
            result["issues"].append(f"Zone {zone}: y+ unavailable")
        elif record["y_plus"]["p95"] > 100:
            result["issues"].append(f"Zone {zone}: y+ p95 exceeds 100; refine wall-normal mesh")
        if record["y_plus"] is not None and record["y_plus"]["fraction_below_5"] > .2:
            result["issues"].append(f"Zone {zone}: >20% of wall nodes fall below wall-function activation; near-wall layer resolution needs review")
        if zone:
            rotor = rotors[zone - 1]
            record["shaft_torque_Nm"] = moment_about_shaft_axis(
                tuple(record["moment_Nm"][axis] for axis in "xyz"),
                tuple(record["force_N"][axis] for axis in "xyz"),
                (rotor.x_m, rotor.y_m, rotor.z_m),
                rotor.shaft_axis_xyz,
            )
            record["rotor_station"] = {
                "x_m": rotor.x_m, "y_m": rotor.y_m, "z_m": rotor.z_m,
                "station": rotor.station_name, "side": rotor.side,
                "hand": rotor.prop_hand, "spin_sign": rotor.spin_sign,
                "shaft_axis_xyz": rotor.shaft_axis_xyz,
                "thrust_direction_xyz": rotor.thrust_direction_xyz,
            }
            record["shaft_torque_source_marker"] = rotor.propeller_wall
        result["zone_forces"].append(record)
    driver = multizone_history_path(directory)
    if driver is not None:
        coupled = read_table(driver)
        time_index = find_column(coupled, "Time_Iter").astype(int)
        last = np.r_[np.flatnonzero(time_index[1:] != time_index[:-1]), len(time_index)-1]
        selected = last[time_index[last] > (revolutions-10)*steps]
        bgs = {key: float(np.max(column[selected])) for key, column in coupled.items()
               if is_bgs_residual_column(key) and len(selected)}
        result["worst_final_coupling_log_residual_in_sample"] = bgs
        if not bgs or max(bgs.values()) > -4:
            result["issues"].append("Coupled-zone residuals have not all reached -4")
    if len(result["zone_forces"]) == 5 and all(
        row.get("history_complete") for row in result["zone_forces"]
    ):
        total = {axis: sum(row["force_N"][axis] for row in result["zone_forces"]) for axis in "xyz"}
        angle = math.radians(ALPHA)
        result["total_force_N"] = total
        result["net_streamwise_force_N_positive_drag"] = -total["x"]*math.cos(angle) + total["z"]*math.sin(angle)
        result["upward_lift_N"] = total["x"]*math.sin(angle) + total["z"]*math.cos(angle)
        result["shaft_power_W"] = sum(
            -row["shaft_torque_Nm"] * row["rotor_station"]["spin_sign"] * RPM * 2 * math.pi / 60
            for row in result["zone_forces"][1:]
        )
    result["run_complete"] = all_zones_complete and len(result["zone_forces"]) == 5
    result["status"] = "completed_exploratory" if result["run_complete"] else "incomplete"
    result["issues"].append("No prism boundary layers or transition model; wall shear and separation need independent validation")
    dump(directory / "results.json", result)
    return result


def yplus(directory: Path, zone: int) -> dict | None:
    candidates = sorted(directory.glob(f"rotor_surface_{zone}_*.csv"))
    if not candidates:
        return None
    table = read_table(candidates[-1])
    column = next((v for k, v in table.items() if k.lower().replace("_", "").replace("+", "plus") in ("yplus", "ypluswall")), None)
    if column is None or not len(column) or not np.isfinite(column).all():
        return None
    return {"min": float(np.min(column)), "median": float(np.median(column)),
            "p95": float(np.quantile(column, .95)), "max": float(np.max(column)),
            "fraction_below_5": float(np.mean(column < 5)),
            "fraction_above_100": float(np.mean(column > 100)),
            "weighting": "surface nodes, not surface area"}


def report(directory: Path, cases: list[dict], failure: str | None,
           geometry: str = "pr3_baseline") -> None:
    comparisons = []
    for left, right, label in ((0, 1, "spatial"), (1, 2, "time-step")):
        if len(cases) <= right or "total_force_N" not in cases[left]["results"] or "total_force_N" not in cases[right]["results"]:
            continue
        a, b = cases[left]["results"]["total_force_N"], cases[right]["results"]["total_force_N"]
        comparisons.append({"comparison": label, "relative_total_force_change": {
            k: abs(b[k]-a[k])/max(abs(b[k]), .02) for k in "xyz"}})
    dump(directory / "campaign_results.json", {"geometry": geometry, "cases": cases,
          "comparisons": comparisons, "failure": failure, "design_ready": False})
    lines = ["# Powered CFD experiment", "", f"Geometry: {geometry}",
             "Status: " + ("stopped: " + failure if failure else "finite campaign completed"),
             "", "Four blade-resolved sliding zones; 40,000 RPM;",
             "15 m/s; alpha 4 degrees; sea level; smooth adiabatic walls; SST URANS.", "",
             "Moving zones contain propeller surfaces only. The mesher rejects cases where "
             "a rotating zone intersects stationary airframe or motor geometry.", "",
             "This is exploratory wall-modelled CFD. There are no prism layers and no transition",
             "model. Completion is not evidence of converged or design-ready aerodynamics.", ""]
    for case in cases:
        lines.extend([f"## {case['name']}", "", f"Mesh: {case['mesh']['nodes']:,} nodes, {case['mesh']['tetrahedra']:,} tetrahedra.", "",
                      "```json", json.dumps(case["results"], indent=2), "```", ""])
    lines += ["## Sensitivity", "", "```json", json.dumps(comparisons, indent=2), "```", "",
              "Two spatial levels are a sensitivity check, not a formal grid-convergence-index study.",
              "Forces include propeller thrust: net streamwise force is not airframe drag.",
              "No SD8020 comparison is run in this powered campaign."]
    (directory / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-dir", type=Path)
    parser.add_argument("--geometry", choices=("pr3_baseline", "airfoil_candidate"),
                        default="pr3_baseline")
    parser.add_argument("--preflight-mesh", type=Path,
                        help="Existing one-revolution rotor mesh used only for SU2 config parsing")
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--revolutions", type=int, default=100)
    parser.add_argument("--hours-per-case", type=float, default=18)
    args = parser.parse_args()
    if args.revolutions < 20 or args.hours_per_case <= 0:
        parser.error("Use at least 20 revolutions and a positive wall-time budget")
    directory = (args.case_dir or ROOT / "tmp" / f"su2-campaign-{args.geometry}").resolve()
    if args.prepare:
        if directory.exists() and any(directory.iterdir()):
            raise FileExistsError(f"Choose an empty campaign directory: {directory}")
        directory.mkdir(parents=True, exist_ok=True)
    elif not directory.is_dir():
        raise FileNotFoundError(f"Campaign directory not found: {directory}")
    plan_path = directory / "plan.json"
    if args.prepare:
        default_mesh_dir = ("su2-rotor" if args.geometry == "pr3_baseline"
                            else f"su2-rotor-{args.geometry}")
        preflight_mesh = (args.preflight_mesh or
                          ROOT / "tmp" / default_mesh_dir / "rotor_domain.su2").resolve()
        if not preflight_mesh.is_file():
            raise FileNotFoundError(
                f"Generate the selected geometry's one-revolution rotor mesh first: {preflight_mesh}"
            )
        configuration = campaign_configuration(args.geometry, args.revolutions,
                                               args.hours_per_case)
        plan = {"geometry": args.geometry, "source_sha256": hashes(args.geometry),
                "configuration": configuration,
                "configuration_sha256": digest_json(configuration),
                "revolutions": args.revolutions, "hours_per_case": args.hours_per_case,
                "design_ready": False,
                "solver_sha256": hashlib.sha256(solver().read_bytes()).hexdigest()}
        preflight = directory / "preflight"
        preflight.mkdir()
        mesh_arg = Path(os.path.relpath(preflight_mesh, preflight)).as_posix()
        config = write_case(preflight, 120, 20, mesh_arg, args.geometry)
        run_solver(preflight, config.name, .02, dry=True)
        plan["preflight_config_sha256"] = config_files_digest(preflight)
        dump(plan_path, plan)
        print(f"Prepared and parsed with SU2: {plan_path}", flush=True)
        return
    if not plan_path.is_file():
        raise FileNotFoundError(f"No prepared plan found at {plan_path}; run with --prepare first")
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    current_solver_hash = hashlib.sha256(solver().read_bytes()).hexdigest()
    preflight_config_hash = config_files_digest(directory / "preflight")
    verify_prepared_campaign(plan, args.geometry, args.revolutions,
                             args.hours_per_case, current_solver_hash,
                             preflight_config_hash)
    assert_fresh_campaign_outputs(directory)
    cases = []
    failure = None
    try:
        if shutil.disk_usage(directory).free < MIN_START_DISK_BYTES:
            raise RuntimeError("Need at least 6 GiB free disk for mesh/flow artifacts")
        with (directory / "tests.log").open("w", encoding="utf-8") as log:
            subprocess.run([sys.executable, "-m", "unittest", "discover", "-v"],
                           cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        for name, scale, steps in (("medium", 1., 120), ("fine", .75, 120), ("fine_half_dt", .75, 240)):
            work = directory / name
            if work.exists():
                raise FileExistsError(f"Refusing to overwrite {work}")
            work.mkdir()
            print(f"START {name}: {steps} steps/revolution", flush=True)
            dump(directory / "progress.json", {"stage": name, "operation": "meshing", "completed_cases": len(cases)})
            if name == "fine_half_dt":
                # Identical spatial mesh, fresh solution: isolate time-step change.
                shutil.copy2(directory / "fine/rotor_domain.su2", work / "rotor_domain.su2")
                mesh = json.loads((directory / "fine/mesh_summary.json").read_text())
                dump(work / "mesh_summary.json", mesh)
            else:
                mesh = make_mesh(work, scale, args.geometry)
            config = write_case(work, steps, args.revolutions, geometry=args.geometry)
            run_solver(work, config.name, .02, dry=True)
            dump(directory / "progress.json", {"stage": name, "operation": "solving", "completed_cases": len(cases)})
            run_solver(work, config.name, args.hours_per_case)
            results = summarize_case(work, steps, args.revolutions, args.geometry)
            cases.append({"name": name, "mesh": mesh, "results": results})
            report(directory, cases, None, args.geometry)
    except Exception as exc:
        failure = str(exc)
        raise
    finally:
        report(directory, cases, failure, args.geometry)
        dump(directory / "progress.json", {"stage": "finished" if failure is None else "stopped",
                                          "failure": failure, "completed_cases": len(cases)})


if __name__ == "__main__":
    main()
