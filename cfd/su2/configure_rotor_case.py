"""Write per-zone SU2 configs for the four rotating propeller domains."""
from __future__ import annotations

import argparse
from math import pi, sqrt
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from airframe_geometry import (
    PARAMETERS,
    AirframeParameters,
    airframe_parameters_for_geometry,
)
from generate_rotor_mesh import _rotors


MM_TO_M = 0.001


def write_configs(
    case_dir: Path,
    rpm: float = 40_000.0,
    revolutions: float = 1.0,
    steps_per_revolution: int = 120,
    params: AirframeParameters = PARAMETERS,
    geometry: str = "pr3_baseline",
    topology: str = "tractor4",
    coaxial_separation_mm: float | None = None,
    alpha_deg: float = 0.0,
    speed_mps: float = 15.0,
) -> tuple[Path, float, int]:
    if rpm <= 0 or revolutions <= 0 or steps_per_revolution < 12:
        raise ValueError("RPM and revolutions must be positive; use at least 12 steps per revolution")
    if topology == "coaxial8" and coaxial_separation_mm is None:
        raise ValueError("coaxial8 configuration requires an explicit coaxial separation")

    rotors = _rotors(params, topology, coaxial_separation_mm)
    generated_names = [
        "zone_static.cfg",
        *(f"zone_rotor_{rotor.index}.cfg" for rotor in rotors),
        "rotor_case.cfg",
        "rotor_run_parameters.txt",
    ]
    existing = [name for name in generated_names if (case_dir / name).exists()]
    if existing:
        raise FileExistsError(
            f"Refusing to overwrite existing SU2 configuration files in {case_dir}: "
            + ", ".join(existing)
        )
    case_dir.mkdir(parents=True, exist_ok=True)
    angular_speed = rpm * 2 * pi / 60
    time_step = 60 / (rpm * steps_per_revolution)
    physical_steps = round(revolutions * steps_per_revolution)
    if physical_steps < 1:
        raise ValueError("Requested run is shorter than one time step")
    # Iteration 0 is the initial t=0 state, so include one level after all
    # requested time advances in SU2's TIME_ITER count.
    time_steps = physical_steps + 1

    zone_files = ["zone_static.cfg"]
    interface_pairs: list[str] = []

    static = """% Stationary aircraft and outer-flow region.
GRID_MOVEMENT= NONE
MARKER_MONITORING= ( AIRFRAME_MOTOR )
MARKER_PLOTTING= ( AIRFRAME_MOTOR )
MARKER_EULER= ( AIRFRAME_MOTOR )
MARKER_FAR= ( FARFIELD )
"""
    (case_dir / zone_files[0]).write_text(static, encoding="utf-8")

    for rotor in rotors:
        index = rotor.index
        hand = rotor.prop_hand
        spin_sign = rotor.spin_sign
        zone_name = f"zone_rotor_{index}.cfg"
        zone_files.append(zone_name)
        y_m = rotor.y_m
        z_m = rotor.z_m
        motion_origin_x = rotor.x_m
        marker = f"ROTOR_PROPELLER_{index}"
        rotor_interface = f"ROTOR_IF_{index}"
        stationary = f"STATIC_IF_{index}"
        interface_pairs.extend((stationary, rotor_interface))
        rate = spin_sign * angular_speed
        text = f"""% Rotor {index}: {hand} blade geometry, {spin_sign:+d} X-axis rotation.
GRID_MOVEMENT= RIGID_MOTION
MOTION_ORIGIN= {motion_origin_x:.9f} {y_m:.9f} {z_m:.9f}
ROTATION_RATE= {rate:.9f} 0.0 0.0
MARKER_MONITORING= ( {marker} )
MARKER_PLOTTING= ( {marker} )
MARKER_EULER= ( {marker} )
"""
        (case_dir / zone_name).write_text(text, encoding="utf-8")

    config = f"""% Blade-resolved topology sliding-mesh case.
% Geometry: {geometry}.
% Provisional motor speed: {rpm:.1f} RPM; change with run_rotor_case.ps1 -Rpm.
% Rotor zones and aircraft coordinates are generated from airframe_geometry.py.
MULTIZONE= YES
MULTIZONE_MESH= YES
CONFIG_LIST= ( {', '.join(zone_files)} )
MARKER_ZONE_INTERFACE= ( {', '.join(interface_pairs)} )
MARKER_FLUID_INTERFACE= ( {', '.join(interface_pairs)} )

SOLVER= EULER
MATH_PROBLEM= DIRECT
RESTART_SOL= NO

TIME_DOMAIN= YES
TIME_MARCHING= DUAL_TIME_STEPPING-2ND_ORDER
TIME_ITER= {time_steps}
TIME_STEP= {time_step:.12g}
OUTER_ITER= 5
INNER_ITER= 1

% 15 m/s cruise speed from +X toward -X; compressible because rotor tip Mach is high.
MACH_NUMBER= {speed_mps / sqrt(1.4 * 287.058 * 288.15):.12g}
MACH_MOTION= {speed_mps / sqrt(1.4 * 287.058 * 288.15):.12g}
AOA= {180.0 - alpha_deg:.9g}
FREESTREAM_PRESSURE= 101325.0
FREESTREAM_TEMPERATURE= 288.15
FREESTREAM_OPTION= TEMPERATURE_FS
INIT_OPTION= TD_CONDITIONS
REF_AREA= {params.wing_planform_area * 1e-6:.12g}
REF_LENGTH= {2 / 3 * params.wing_root_chord * 1e-3 * (1 + params.wing_tip_chord / params.wing_root_chord + (params.wing_tip_chord / params.wing_root_chord) ** 2) / (1 + params.wing_tip_chord / params.wing_root_chord):.12g}
REF_ORIGIN_MOMENT_X= 0.0
REF_ORIGIN_MOMENT_Y= 0.0
REF_ORIGIN_MOMENT_Z= 0.0

NUM_METHOD_GRAD= WEIGHTED_LEAST_SQUARES
CONV_NUM_METHOD_FLOW= ROE
MUSCL_FLOW= NO
ENTROPY_FIX_COEFF= 0.001
SLOPE_LIMITER_FLOW= VENKATAKRISHNAN
LIMITER_ITER= 100
TIME_DISCRE_FLOW= EULER_IMPLICIT
CFL_NUMBER= 1.0

CONV_FILENAME= rotor_history
MESH_FILENAME= rotor_domain.su2
MESH_FORMAT= SU2
MESH_OUT_FILENAME= rotor_mesh_out
RESTART_FILENAME= rotor_restart
VOLUME_FILENAME= rotor_flow
SURFACE_FILENAME= rotor_surface
TABULAR_FORMAT= CSV
OUTPUT_FILES= ( RESTART, PARAVIEW, SURFACE_CSV )
OUTPUT_WRT_FREQ= {max(1, min(time_steps, steps_per_revolution // 4))}
SCREEN_OUTPUT= ( OUTER_ITER, WALL_TIME, RMS_RES )
"""
    config_path = case_dir / "rotor_case.cfg"
    config_path.write_text(config, encoding="utf-8")
    run_info = (
        f"Geometry: {geometry}\nRPM: {rpm:.3f}\nAngular speed: {angular_speed:.9f} rad/s\n"
        f"Time step: {time_step:.12g} s\nTime steps: {time_steps}\n"
        f"Simulated revolutions: {(time_steps - 1) / steps_per_revolution:.6f}\n"
        f"Topology: {topology}; rotor count: {len(rotors)}; alpha: {alpha_deg:.3f} deg; "
        f"speed: {speed_mps:.3f} m/s\n"
        f"Time levels including t=0: {time_steps}\n"
        "Only propeller surfaces are assigned to rigidly rotating zones; stationary "
        "airframe and motor intersections are rejected before meshing.\n"
        f"Model: compressible inviscid Euler; {len(rotors)} actual propeller STEP solids in rotating zones.\n"
    )
    (case_dir / "rotor_run_parameters.txt").write_text(run_info, encoding="utf-8")
    print(
        f"Wrote {config_path}: {rpm:.1f} RPM, {angular_speed:.3f} rad/s, "
        f"{time_steps} time levels over {physical_steps / steps_per_revolution:.3f} revolutions."
    )
    return config_path, time_step, time_steps


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-dir", type=Path)
    parser.add_argument(
        "--geometry",
        choices=("pr3_baseline", "airfoil_candidate"),
        default="pr3_baseline",
    )
    parser.add_argument("--rpm", type=float, default=40_000.0)
    parser.add_argument("--revolutions", type=float, default=1.0)
    parser.add_argument("--steps-per-revolution", type=int, default=120)
    parser.add_argument("--topology", choices=("tractor4", "pusher4", "coaxial8"),
                        default="tractor4")
    parser.add_argument("--coaxial-separation-mm", type=float)
    parser.add_argument("--alpha-deg", type=float, default=0.0)
    parser.add_argument("--speed-mps", type=float, default=15.0)
    args = parser.parse_args()
    params = airframe_parameters_for_geometry(args.geometry)
    default_case_dir = (
        ROOT / "tmp" / "su2-rotor"
        if args.geometry == "pr3_baseline"
        else ROOT / "tmp" / f"su2-rotor-{args.geometry}"
    )
    write_configs(
        (args.case_dir or default_case_dir).resolve(),
        args.rpm,
        args.revolutions,
        args.steps_per_revolution,
        params,
        args.geometry,
        args.topology,
        args.coaxial_separation_mm,
        args.alpha_deg,
        args.speed_mps,
    )


if __name__ == "__main__":
    main()
