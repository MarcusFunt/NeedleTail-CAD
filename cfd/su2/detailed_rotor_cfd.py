"""Finite, resource-bounded powered URANS experiment and numerical-quality report.

This is an exploratory wall-modelled tetrahedral calculation, not a certified
wall-resolved calculation. No force result is labelled design-ready.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
import psutil

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from airframe_geometry import PARAMETERS
import configure_rotor_case
import generate_rotor_mesh as mesher

SOURCES = [ROOT / "airframe_geometry.py", mesher.AIRFRAME_STEP,
           mesher.MOTOR_STEP, *mesher.PROP_STEPS.values(),
           Path(__file__), Path(mesher.__file__), Path(configure_rotor_case.__file__)]
SPEED = 15.0
ALPHA = 4.0
RPM = 40_000.0
RHO = 101325.0 / (287.058 * 288.15)
SOUND = math.sqrt(1.4 * 287.058 * 288.15)
Q = 0.5 * RHO * SPEED**2
AREA = PARAMETERS.wing_planform_area * 1e-6
TAPER = PARAMETERS.wing_tip_chord / PARAMETERS.wing_root_chord
MAC = 2 / 3 * PARAMETERS.wing_root_chord * 1e-3 * (1 + TAPER + TAPER**2) / (1 + TAPER)


def dump(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def hashes() -> dict[str, str]:
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in SOURCES}


def solver() -> Path:
    location = os.environ.get("SU2_RUN") or str(
        Path(os.environ["LOCALAPPDATA"]) / "Programs/SU2/8.5.0/bin")
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
               mesh: str = "rotor_domain.su2") -> Path:
    config, _, _ = configure_rotor_case.write_configs(directory, RPM, revolutions, steps)
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
        "REF_AREA": f"{AREA:.12g}", "REF_LENGTH": f"{MAC:.12g}",
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
        marker = "AIRFRAME_MOTOR" if index == 0 else f"ROTOR_WALL_{index}"
        text = path.read_text(encoding="utf-8").replace(
            f"MARKER_EULER= ( {marker} )", f"MARKER_HEATFLUX= ( {marker}, 0.0 )")
        text += (f"MARKER_MONITORING= ( {marker} )\nMARKER_PLOTTING= ( {marker} )\n"
                 f"MARKER_WALL_FUNCTIONS= ( {marker}, STANDARD_WALL_FUNCTION )\n"
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
        f"Four real propeller STEP blades: {RPM:g} RPM, {revolutions} revolutions.\n"
        f"{SPEED:g} m/s, aircraft alpha={ALPHA:g} deg, SU2 AOA={180-ALPHA:g} deg.\n"
        f"{steps} steps/revolution; dt={60/RPM/steps:.12g} s.\n"
        "Adiabatic smooth walls; fully turbulent, no transition model.\n"
        "Motor can follows rotor as in the existing sliding-zone model.\n"
        "Coefficients use the same freestream dynamic pressure in all zones.\n"
        "Use physical XYZ forces; SU2 LIFT has reversed sign in this orientation.\n"
        "Tetrahedral wall-function mesh; no prism layers, no design-ready claim.\n",
        encoding="utf-8")
    return config


def run_solver(directory: Path, config: str, limit_hours: float, dry: bool = False) -> None:
    executable = solver()
    env = os.environ.copy()
    env["PATH"] = str(executable.parent) + os.pathsep + env["PATH"]
    env["OMP_NUM_THREADS"] = "4"
    argv = [str(executable), *(["-d"] if dry else []), config]
    started = time.monotonic()
    log = directory / ("config_check.log" if dry else "solver.log")
    with log.open("w", encoding="utf-8") as stream:
        child = subprocess.Popen(argv, cwd=directory, env=env, stdout=stream,
                                 stderr=subprocess.STDOUT)
        try:
            while child.poll() is None:
                if time.monotonic() - started > limit_hours * 3600:
                    raise RuntimeError(f"Solver wall-time budget reached; see {log}")
                if psutil.virtual_memory().available < 0.8 * 1024**3:
                    raise RuntimeError("Less than 0.8 GiB RAM available; stopped to preserve desktop")
                if shutil.disk_usage(directory).free < 1.5 * 1024**3:
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


def make_mesh(directory: Path, scale: float) -> dict:
    """Use the existing robust CAD booleans; add quality gates and wake sizing."""
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
                  "nonpositive_quality_cells": int(np.count_nonzero(quality <= 0))}
        metrics.append(record)
        dump(directory / "mesh_quality.json", metrics)
        if record["nonpositive_quality_cells"]:
            raise RuntimeError(f"Nonpositive cell Jacobian/quality: {record}")
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
            prop_near_size_m=.0008 * scale, interface_size_m=.002 * scale)
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


def summarize_case(directory: Path, steps: int, revolutions: int) -> dict:
    # Multizone writes separate zone histories as well as a global history.
    result = {"status": "exploratory", "zone_forces": [], "issues": [],
              "design_ready": False}
    for zone in range(5):
        candidates = [directory / f"rotor_history_{zone}.csv",
                      directory / f"history_{zone}.csv"]
        path = next((p for p in candidates if p.is_file()), None)
        if path is None:
            # Driver history suffixes every zone-specific column with [zone].
            driver = directory / "rotor_case.csv"
            if not driver.is_file():
                result["issues"].append(f"Zone {zone} history missing; force extraction unavailable")
                continue
            suffix = f"[{zone}]"
            table = {k[:-len(suffix)] if k.endswith(suffix) else k: v
                     for k, v in read_table(driver).items()
                     if k.endswith(suffix) or k in ("Time_Iter", "Outer_Iter")}
        else:
            table = read_table(path)
        t = find_column(table, "Time_Iter").astype(int)
        # Keep the last pseudo-time/coupling iterate for each physical step.
        last = np.r_[np.flatnonzero(t[1:] != t[:-1]), len(t)-1]
        t = t[last]
        data = {key: val[last] for key, val in table.items()}
        if t[-1] < steps * revolutions:
            result["issues"].append(f"Zone {zone}: physical run incomplete ({t[-1]} steps)")
        mask = t > (revolutions - 10) * steps
        if np.count_nonzero(mask) < steps * 8:
            result["issues"].append(f"Zone {zone}: fewer than eight sampled revolutions")
        record = {"zone": zone, "last_time_iteration": int(t[-1]), "force_N": {},
                  "moment_Nm": {}, "last_5rev_vs_previous_5rev_relative_change": {}}
        for axis in "xyz":
            forces = find_column(data, f"CF{axis}") * Q * AREA
            moments = find_column(data, f"CM{axis}") * Q * AREA * MAC
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
            y_mm, z_mm = PARAMETERS.propeller_mount_stations[zone - 1]
            record["shaft_torque_Nm"] = record["moment_Nm"]["x"] - (
                y_mm * 1e-3 * record["force_N"]["z"] - z_mm * 1e-3 * record["force_N"]["y"])
        result["zone_forces"].append(record)
    driver = directory / "rotor_case.csv"
    if driver.is_file():
        coupled = read_table(driver)
        time_index = find_column(coupled, "Time_Iter").astype(int)
        last = np.r_[np.flatnonzero(time_index[1:] != time_index[:-1]), len(time_index)-1]
        selected = last[time_index[last] > (revolutions-10)*steps]
        bgs = {key: float(np.max(column[selected])) for key, column in coupled.items()
               if key.lower().startswith("bgs[") and len(selected)}
        result["worst_final_coupling_log_residual_in_sample"] = bgs
        if not bgs or max(bgs.values()) > -4:
            result["issues"].append("Coupled-zone residuals have not all reached -4")
    if len(result["zone_forces"]) == 5:
        total = {axis: sum(row["force_N"][axis] for row in result["zone_forces"]) for axis in "xyz"}
        angle = math.radians(ALPHA)
        result["total_force_N"] = total
        result["net_streamwise_force_N_positive_drag"] = -total["x"]*math.cos(angle) + total["z"]*math.sin(angle)
        result["upward_lift_N"] = total["x"]*math.sin(angle) + total["z"]*math.cos(angle)
        result["shaft_power_W"] = sum(-row["shaft_torque_Nm"] * spin * RPM*2*math.pi/60
                                        for row, spin in zip(result["zone_forces"][1:], (-1, 1, -1, 1)))
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


def report(directory: Path, cases: list[dict], failure: str | None) -> None:
    comparisons = []
    for left, right, label in ((0, 1, "spatial"), (1, 2, "time-step")):
        if len(cases) <= right or "total_force_N" not in cases[left]["results"] or "total_force_N" not in cases[right]["results"]:
            continue
        a, b = cases[left]["results"]["total_force_N"], cases[right]["results"]["total_force_N"]
        comparisons.append({"comparison": label, "relative_total_force_change": {
            k: abs(b[k]-a[k])/max(abs(b[k]), .02) for k in "xyz"}})
    dump(directory / "campaign_results.json", {"cases": cases, "comparisons": comparisons,
          "failure": failure, "design_ready": False})
    lines = ["# Powered CFD experiment", "", "Status: " + ("stopped: " + failure if failure else "finite campaign completed"),
             "", "Current NACA-derived airframe; four blade-resolved sliding zones; 40,000 RPM;",
             "15 m/s; alpha 4 degrees; sea level; smooth adiabatic walls; SST URANS.", "",
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
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--revolutions", type=int, default=100)
    parser.add_argument("--hours-per-case", type=float, default=18)
    args = parser.parse_args()
    if args.revolutions < 20 or args.hours_per_case <= 0:
        parser.error("Use at least 20 revolutions and a positive wall-time budget")
    directory = args.case_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    plan_path = directory / "plan.json"
    if args.prepare:
        if any((directory / name).exists() for name in ("medium", "fine", "fine_half_dt")):
            raise FileExistsError("Choose a new campaign directory; a calculation already exists")
        plan = {"source_sha256": hashes(), "parameters": asdict(PARAMETERS),
                "rpm": RPM, "speed_mps": SPEED, "alpha_deg": ALPHA,
                "revolutions": args.revolutions, "hours_per_case": args.hours_per_case,
                "reference_area_m2": AREA, "reference_length_m": MAC,
                "mesh_levels": [1.0, .75], "time_steps_per_revolution": [120, 120, 240],
                "design_ready": False, "solver_sha256": hashlib.sha256(solver().read_bytes()).hexdigest()}
        dump(plan_path, plan)
        preflight = directory / "preflight"
        config = write_case(preflight, 120, 20, "../../su2-rotor/rotor_domain.su2")
        run_solver(preflight, config.name, .02, dry=True)
        print(f"Prepared and parsed with SU2: {plan_path}", flush=True)
        return
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan["source_sha256"] != hashes():
        raise RuntimeError("Geometry or runner changed after preparation; prepare a new campaign")
    if args.revolutions != plan["revolutions"] or args.hours_per_case != plan["hours_per_case"]:
        raise RuntimeError("Command does not match the prepared run budget")
    cases = []
    failure = None
    try:
        if shutil.disk_usage(directory).free < 6 * 1024**3:
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
                mesh = make_mesh(work, scale)
            config = write_case(work, steps, args.revolutions)
            run_solver(work, config.name, .02, dry=True)
            dump(directory / "progress.json", {"stage": name, "operation": "solving", "completed_cases": len(cases)})
            run_solver(work, config.name, args.hours_per_case)
            results = summarize_case(work, steps, args.revolutions)
            cases.append({"name": name, "mesh": mesh, "results": results})
            report(directory, cases, None)
    except Exception as exc:
        failure = str(exc)
        raise
    finally:
        report(directory, cases, failure)
        dump(directory / "progress.json", {"stage": "finished" if failure is None else "stopped",
                                          "failure": failure, "completed_cases": len(cases)})


if __name__ == "__main__":
    main()
