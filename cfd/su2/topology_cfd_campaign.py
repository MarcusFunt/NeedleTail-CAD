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
from design_configuration import Design
from detailed_rotor_cfd import available_memory_bytes, geometry_values, run_solver
from generate_rotor_mesh import _rotors, generate_mesh

TOPOLOGIES = ("tractor4", "pusher4", "coaxial8")
RHO = 101325.0 / (287.058 * 288.15)
SOUND = math.sqrt(1.4 * 287.058 * 288.15)
MIN_RUNTIME_MEMORY_BYTES = int(0.8 * 1024**3)
MIN_RUNTIME_DISK_BYTES = int(1.5 * 1024**3)


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
        marker = "AIRFRAME_MOTOR" if zone == 0 else f"ROTOR_WALL_{zone}"
        zone_updates = {
            "OUTPUT_FILES": "( RESTART )",
            "OUTPUT_WRT_FREQ": str(physical_steps + 1),
            "WRT_RESTART_OVERWRITE": "YES",
        }
        zone_text = replace_config_keys(path.read_text(encoding="utf-8"), zone_updates)
        zone_text += (
            f"MARKER_MONITORING= ( {marker} )\n"
            f"MARKER_PLOTTING= ( {marker} )\n"
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


def rows_by_zone(case_dir: Path, zone: int) -> list[dict[str, str]]:
    paths = [case_dir / f"rotor_history_{zone}.csv", case_dir / f"history_{zone}.csv"]
    for path in paths:
        if path.is_file():
            with path.open(newline="", encoding="utf-8-sig") as stream:
                return list(csv.DictReader(stream))
    driver = case_dir / "rotor_history_multizone.csv"
    if not driver.is_file():
        driver = case_dir / "rotor_history.csv"
    if not driver.is_file():
        raise FileNotFoundError(f"No history file found for zone {zone} under {case_dir}")
    with driver.open(newline="", encoding="utf-8-sig") as stream:
        all_rows = list(csv.DictReader(stream))
    suffix = f"[{zone}]"
    selected: list[dict[str, str]] = []
    for row in all_rows:
        new = {key[:-len(suffix)] if key.endswith(suffix) else key: value
               for key, value in row.items()
               if key.endswith(suffix) or key in ("Time_Iter", "Outer_Iter")}
        selected.append(new)
    return selected


def summarize_history(
    case_dir: Path,
    topology: str,
    params,
    revolutions: int,
    steps_per_revolution: int,
    rpm: float,
    speed_mps: float,
    alpha_deg: float,
) -> dict[str, Any]:
    rotors = _rotors(params, topology)
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
        rows = rows_by_zone(case_dir, zone)
        if not rows:
            raise RuntimeError(f"Zone {zone} history is empty")
        headers = list(rows[0])
        time_key = find_key(headers, "Time_Iter")
        force_keys = {axis: find_key(headers, f"CF{axis}") for axis in "xyz"}
        moment_keys = {axis: find_key(headers, f"CM{axis}") for axis in "xyz"}
        # SU2 can write several pseudo-time rows at one physical time; retain the final one.
        final_by_step: dict[int, dict[str, str]] = {}
        for row in rows:
            try:
                step = int(float(row[time_key]))
            except (TypeError, ValueError):
                continue
            final_by_step[step] = row
        steps = sorted(final_by_step)
        if not steps:
            raise RuntimeError(f"Zone {zone} history has no physical time indices")
        if steps != expected_time_indices:
            observed = set(steps)
            missing = [step for step in expected_time_indices if step not in observed]
            unexpected = [step for step in steps if step not in expected_index_set]
            zone_records.append({
                "zone": zone,
                "kind": "static_airframe_and_motor" if zone == 0 else "rotor_zone_and_rotating_airframe_patches",
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
            forces = {
                axis: sum(float(final_by_step[step][force_keys[axis]]) * q * area
                          for step in sample) / len(sample)
                for axis in "xyz"
            }
            moments = {
                axis: sum(float(final_by_step[step][moment_keys[axis]]) * q * area * mac
                          for step in sample) / len(sample)
                for axis in "xyz"
            }
            return forces, moments

        forces, moments = mean_force_moment(sample_steps)
        last_window = [step for step in steps if step > expected_steps - steps_per_revolution]
        previous_window = [step for step in steps
                           if expected_steps - 2 * steps_per_revolution < step
                           <= expected_steps - steps_per_revolution]
        drift: dict[str, float | None] = {}
        if last_window and previous_window:
            last_force, _ = mean_force_moment(last_window)
            previous_force, _ = mean_force_moment(previous_window)
            for axis in "xyz":
                drift[axis] = abs(last_force[axis] - previous_force[axis]) / max(abs(forces[axis]), 0.02)
        else:
            drift = {axis: None for axis in "xyz"}

        residual_keys = [key for key in headers
                         if key.strip().strip('"').strip().lower().replace("_", "").startswith("rms")]
        residuals = {}
        if residual_keys:
            row = final_by_step[steps[-1]]
            for key in residual_keys:
                try:
                    residuals[key] = float(row[key])
                except (TypeError, ValueError):
                    pass
        record: dict[str, Any] = {
            "zone": zone,
            "kind": "static_airframe_and_motor" if zone == 0 else "rotor_zone_and_rotating_airframe_patches",
            "complete_time_steps": steps == expected_time_indices,
            "last_time_iteration": steps[-1],
            "sample_count_final_2_revolutions": len(sample_steps),
            "force_N": forces,
            "moment_Nm": moments,
            "last_revolution_force_change_fraction": drift,
            "final_residual_logs": residuals,
        }
        if zone:
            rotor = rotors[zone - 1]
            torque = moments["x"] - (
                rotor.y_m * forces["z"] - rotor.z_m * forces["y"]
            )
            record["rotor_station"] = {
                "x_m": rotor.x_m, "y_m": rotor.y_m, "z_m": rotor.z_m,
                "side": rotor.side, "hand": rotor.prop_hand, "spin_sign": rotor.spin_sign,
            }
            record["shaft_torque_Nm"] = torque
            record["shaft_power_W"] = -torque * rotor.spin_sign * rpm * 2 * math.pi / 60
        zone_records.append(record)

    complete = all(row["complete_time_steps"] for row in zone_records)
    if not complete:
        return {
            "topology": topology,
            "rotor_count": rotor_count,
            "expected_physical_steps": expected_steps,
            "run_complete": False,
            "zone_forces": zone_records,
            "design_ready": False,
            "limitations": [
                "One or more zone histories are missing expected physical time indices; force averages are withheld.",
                "Exploratory CFD only; no viscous drag, transition, separation model, or formal mesh/time convergence.",
            ],
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
        "run_complete": complete,
        "zone_forces": zone_records,
        "total_force_N": total,
        "total_moment_Nm_about_geometry_origin": total_moment,
        "upward_lift_N": lift,
        "positive_drag_N_including_prop_thrust": drag,
        "shaft_power_sum_W": sum(row.get("shaft_power_W", 0.0) for row in zone_records),
        "design_ready": False,
        "limitations": [
            "Compressible inviscid Euler; no viscous drag, transition, or separation model.",
            "Coarse tetrahedral sliding-zone mesh; no prism layers or formal mesh/time convergence.",
            "Each airframe OML patch intersecting a rotor zone rotates rigidly with that rotor.",
            "Net streamwise force includes propeller thrust and airframe/rotor drag; no installed propeller map or trim/power balance.",
            "Forces are averaged over the final two revolutions; compare final-revolution drift before interpreting topology ranking.",
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
    rows = []
    for result in results:
        flow = result.get("aerodynamics", {})
        rows.append(
            f"| {result['topology']} | {result.get('status', 'failed')} | "
            f"{flow.get('upward_lift_N', float('nan')):.4f} | "
            f"{flow.get('positive_drag_N_including_prop_thrust', float('nan')):.4f} | "
            f"{flow.get('total_moment_Nm_about_geometry_origin', {}).get('y', float('nan')):.6f} | "
            f"{flow.get('shaft_power_sum_W', float('nan')):.3f} |"
        )
    report = f"""# Powered Euler CFD screen: {', '.join(config['topologies'])}

## Campaign conditions

- Geometry: {config['geometry']}; topology candidates: {', '.join(config['topologies'])}.
- Operating point: {config['rpm']:.0f} RPM, {config['speed_mps']:.1f} m/s, aircraft alpha +{config['alpha_deg']:.1f}°, sea level.
- Solver: SU2 8.5.0, compressible inviscid Euler, {config['steps_per_revolution']} time steps per revolution, {config['revolutions']} revolutions per candidate.
- Topology-specific propeller counts and stations; airframe OML patches inside each sliding rotor zone move with that rotor.
- Mesh sizes: static near/far {config['static_near_size_m']*1000:.1f}/{config['static_far_size_m']*1000:.0f} mm; rotor near {config['prop_near_size_m']*1000:.1f} mm; sliding interface {config['interface_size_m']*1000:.1f} mm.
- Per-zone force and moment histories are averaged over the final two revolutions. Output contains no large transient volume snapshots.

## Results

Drag is positive aft; lift is positive upward. Net streamwise force includes propeller thrust as well as drag.

| Topology | Status | Lift (N) | Net drag (N) | My about geometry origin (N·m) | Shaft power sum (W) |
|---|---|---:|---:|---:|---:|
{chr(10).join(rows)}

All results are exploratory. This Euler calculation omits viscous effects and propeller performance maps. Rotor zones also rotate any clipped airframe patches, an explicit geometry approximation. The coaxial8 concept has zero-clearance propeller-disk/nacelle intersections in the separate CAD screen; its CFD result must be treated as an interference diagnostic unless the nacelle geometry is revised.

Per-case mesh statistics, time histories, forces, residuals, and solver logs are in the topology subdirectories. See campaign_results.json for structured results.
"""
    (out_dir / "REPORT.md").write_text(report, encoding="utf-8")


def run_campaign(args: argparse.Namespace) -> int:
    out_dir = args.output_dir.resolve()
    if out_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing CFD campaign: {out_dir}")
    params = airframe_parameters_for_geometry(args.geometry)
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
        "topologies": args.topologies,
        "rpm": args.rpm,
        "speed_mps": args.speed_mps,
        "alpha_deg": args.alpha_deg,
        "revolutions": args.revolutions,
        "steps_per_revolution": args.steps_per_revolution,
        "static_near_size_m": args.static_near_size_m,
        "static_far_size_m": args.static_far_size_m,
        "prop_near_size_m": args.prop_near_size_m,
        "interface_size_m": args.interface_size_m,
        "threads": 4,
        "reference_dynamic_pressure_Pa": q,
        "reference_area_m2": area,
        "reference_length_m": mac,
        "solver_executable": str(executable),
        "solver_sha256": digest(executable),
        "airframe_geometry_sha256": digest(ROOT / "airframe_geometry.py"),
        "mesh_generator_sha256": digest(CFD_DIR / "generate_rotor_mesh.py"),
        "case_config_generator_sha256": digest(CFD_DIR / "configure_rotor_case.py"),
        "cad_variant_builder_sha256": digest(ROOT / "design_cad.py"),
        "campaign_source_sha256": digest(Path(__file__)),
    }
    out_dir.mkdir(parents=True)
    (out_dir / "campaign_configuration.json").write_text(
        json.dumps(config_meta, indent=2) + "\n", encoding="utf-8"
    )
    results: list[dict[str, Any]] = []
    for topology in args.topologies:
        case_dir = out_dir / topology
        case_dir.mkdir()
        result: dict[str, Any] = {"topology": topology, "case_dir": str(case_dir)}
        try:
            if available_memory_bytes() < MIN_RUNTIME_MEMORY_BYTES:
                raise RuntimeError("Less than 0.8 GiB RAM available before case setup")
            if shutil.disk_usage(case_dir).free < MIN_RUNTIME_DISK_BYTES:
                raise RuntimeError("Less than 1.5 GiB disk free before case setup")
            design = Design(topology=topology, flaps="none")
            rotors = _rotors(params, topology)
            result["rotor_stations"] = [
                {"index": rotor.index, "x_m": rotor.x_m, "y_m": rotor.y_m,
                 "z_m": rotor.z_m, "side": rotor.side, "hand": rotor.prop_hand,
                 "spin_sign": rotor.spin_sign}
                for rotor in rotors
            ]
            airframe_step = case_dir / "airframe_variant.step"
            export_topology_oml(params, design, airframe_step, args.geometry)
            result["airframe_step_sha256"] = digest(airframe_step)
            result["cad_precheck"] = {
                "topology": topology,
                "note": "The CFD mesh is generated even for CAD-invalid concepts so interference can be diagnosed.",
            }
            if topology == "coaxial8":
                result["cad_precheck"]["warning"] = (
                    "The 8-coaxial concept has zero-clearance disk/nacelle intersections in the topology CAD screen."
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
            )
            rotor_count = len(_rotors(params, topology))
            result["mesh"] = {
                "path": str(mesh_path), "zone_count": rotor_count + 1,
                "zone_local_nodes": nodes, "tetrahedra": cells,
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
                alpha_deg=args.alpha_deg,
                speed_mps=args.speed_mps,
            )
            configure_histories(case_dir, rotor_count, time_levels - 1)
            result["configuration_sha256"] = digest(config)
            run_solver(case_dir, "rotor_case.cfg", args.max_hours_per_case, dry=True)
            run_solver(case_dir, "rotor_case.cfg", args.max_hours_per_case)
            flow = summarize_history(
                case_dir, topology, params, args.revolutions,
                args.steps_per_revolution, args.rpm, args.speed_mps, args.alpha_deg,
            )
            result["aerodynamics"] = flow
            result["status"] = "completed_exploratory" if flow["run_complete"] else "incomplete_history"
            result["solver_exit"] = "success"
        except Exception as exc:
            result["status"] = "failed"
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
    return 0 if all(row["status"] == "completed_exploratory" for row in results) else 1


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
    raise SystemExit(run_campaign(args))


if __name__ == "__main__":
    main()
