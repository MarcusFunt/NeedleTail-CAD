"""Early whole-aircraft aerodynamic trends for the NeedleTail airframe.

The model is assembled from the same AirframeParameters and CAD-matched
airfoil sections used by the geometry generator. Its outputs are conceptual
estimates: VLM is inviscid and assumes attached flow, while AeroBuildup is an
empirical model whose low-Reynolds-number behavior is uncertain.

Run from the repository root with::

    uv run --locked --extra aero python .\\whole_aircraft_aero.py

Results default to ``tmp/whole-aircraft-aero``.
"""
from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path

import aerosandbox as asb
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from airframe_geometry import (
    PARAMETERS,
    AirframeParameters,
    _sample_spline_radius_profile,
    _section_coordinates,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = ROOT / "tmp" / "whole-aircraft-aero"
ALPHAS_DEG = np.arange(-4.0, 8.01, 1.0)
BETAS_DEG = np.arange(-8.0, 8.01, 2.0)
REFERENCE_SPEEDS_MPS = (10.0, 15.0, 20.0)
REFERENCE_SPEED_FOR_VLM_MPS = 15.0
ALTITUDE_M = 0.0
N_CRIT = 9.0


def _airfoil_for_section(name: str, airfoil_name: str, chord_mm: float, te_mm: float):
    upper, lower = _section_coordinates(airfoil_name, chord_mm, te_mm)
    # AeroSandbox expects an airfoil contour ordered upper TE -> LE -> lower TE.
    coordinates = np.asarray(list(reversed(upper)) + lower[1:], dtype=float)
    return asb.Airfoil(name=f"{name}_{airfoil_name}_CAD_TE", coordinates=coordinates)


def _vertical_airfoil(airfoil: asb.Airfoil, name: str) -> asb.Airfoil:
    """Orient the CAD +Y section thickness onto ASB's vertical-wing +Y side."""
    # A Wing spanning upward along +Z has its local airfoil thickness axis toward
    # -Y. Reflect and reverse the Selig contour to preserve the project geometry.
    coordinates = np.asarray(airfoil.coordinates[::-1], dtype=float).copy()
    coordinates[:, 1] *= -1
    return asb.Airfoil(name=name, coordinates=coordinates)


def _fuselage_profile(params: AirframeParameters) -> tuple[tuple[float, float], ...]:
    half_length = params.fuselage_length / 2
    aft_x = -half_length
    nose_x = half_length
    max_x = nose_x - params.fuselage_max_station_from_nose * params.fuselage_length
    aft = [
        (aft_x + fraction * (max_x - aft_x), ratio * params.fuselage_max_radius)
        for fraction, ratio in params.fuselage_aft_radius_profile
    ]
    fore = [
        (max_x + fraction * (nose_x - max_x), ratio * params.fuselage_max_radius)
        for fraction, ratio in params.fuselage_fore_radius_profile[1:]
    ]
    return tuple(aft + fore)


def _pod_profile(params: AirframeParameters) -> tuple[tuple[float, float], ...]:
    half_length = params.motor_pod_length / 2
    max_radius = params.motor_pod_diameter / 2
    return tuple(
        (fraction * params.motor_pod_length, ratio * max_radius)
        for fraction, ratio in params.motor_pod_radius_profile
    )


def _sample_profile(
    stations: tuple[tuple[float, float], ...],
    count: int,
) -> list[tuple[float, float]]:
    """Resample the CAD spline at evenly spaced axial stations."""
    cad_spline_samples = _sample_spline_radius_profile(stations)
    x_sampled = np.linspace(stations[0][0], stations[-1][0], count)
    xs, radii = np.asarray(cad_spline_samples, dtype=float).T
    sampled_radii = np.interp(x_sampled, xs, radii)
    return list(zip(x_sampled.tolist(), sampled_radii.tolist()))


def _asb_x(project_x_mm: float) -> float:
    """Convert project +X-forward millimeters to ASB +X-aft meters."""
    return -project_x_mm / 1000


def _make_fuselage(
    name: str,
    profile: tuple[tuple[float, float], ...],
    center_x_mm: float = 0.0,
    center_y_m: float = 0.0,
    center_z_m: float = 0.0,
) -> asb.Fuselage:
    # The project profile runs aft -> nose in +X; reverse it so ASB sections run
    # nose -> tail in +X_aft, preserving monotonically increasing ASB x.
    sections = _sample_profile(profile, count=25)
    return asb.Fuselage(
        name=name,
        xsecs=[
            asb.FuselageXSec(
                xyz_c=[_asb_x(x_mm + center_x_mm), center_y_m, center_z_m],
                radius=radius_mm / 1000,
            )
            for x_mm, radius_mm in reversed(sections)
        ],
    )


def build_airplane(
    params: AirframeParameters = PARAMETERS,
    *,
    wing_span_mm: float | None = None,
    support_span_mm: float | None = None,
) -> asb.Airplane:
    """Build the AeroSandbox proxy; optional spans enable historic A/B sweeps.

    Overrides affect lifting-surface spans only. They do not move the motors,
    fuselage, pods, or change the airfoil/source-parameter definitions.
    """
    if wing_span_mm is None:
        wing_span_mm = params.wing_span
    if support_span_mm is None:
        support_span_mm = params.support_span
    if not (np.isfinite(wing_span_mm) and np.isfinite(support_span_mm)):
        raise ValueError("Span overrides must be finite")
    if wing_span_mm <= 0 or support_span_mm <= 0:
        raise ValueError("Span overrides must be positive")
    wing_root_airfoil = _airfoil_for_section(
        "wing_root", params.wing_airfoil, params.wing_root_chord, params.wing_trailing_edge
    )
    wing_tip_airfoil = _airfoil_for_section(
        "wing_tip", params.wing_airfoil, params.wing_tip_chord, params.wing_trailing_edge
    )
    support_section = _airfoil_for_section(
        "vertical_support",
        params.support_airfoil,
        params.support_chord,
        params.support_trailing_edge,
    )
    support_airfoil = _vertical_airfoil(
        support_section, f"vertical_support_{params.support_airfoil}_CAD_TE_oriented"
    )

    half_wing_span_m = wing_span_mm / 2000
    half_support_span_m = support_span_mm / 2000
    wing_root_chord_m = params.wing_root_chord / 1000
    wing_tip_chord_m = params.wing_tip_chord / 1000
    support_chord_m = params.support_chord / 1000

    horizontal_wing = asb.Wing(
        name="Horizontal wing (CAD-matched SD7003)",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[_asb_x(wing_root_chord_m * 500), 0, 0],
                chord=wing_root_chord_m,
                airfoil=wing_root_airfoil,
            ),
            asb.WingXSec(
                xyz_le=[_asb_x(wing_tip_chord_m * 500), half_wing_span_m, 0],
                chord=wing_tip_chord_m,
                airfoil=wing_tip_airfoil,
            ),
        ],
    )

    vertical_support = asb.Wing(
        name="Vertical support (CAD-matched SD8020)",
        symmetric=False,
        # The single continuous span runs from the lower to upper tip. The
        # profile is reflected above so its physical thickness remains +Y.
        xsecs=[
            asb.WingXSec(
                xyz_le=[_asb_x(support_chord_m * 500), 0, -half_support_span_m],
                chord=support_chord_m,
                airfoil=support_airfoil,
            ),
            asb.WingXSec(
                xyz_le=[_asb_x(support_chord_m * 500), 0, half_support_span_m],
                chord=support_chord_m,
                airfoil=support_airfoil,
            ),
        ],
    )

    pod_profile = _pod_profile(params)
    pod_centers = (
        (params.motor_station_x, -params.horizontal_motor_offset, 0),
        (params.motor_station_x, params.horizontal_motor_offset, 0),
        (params.motor_station_x, 0, -params.vertical_motor_offset),
        (params.motor_station_x, 0, params.vertical_motor_offset),
    )
    fuselages = [
        _make_fuselage("Central fuselage", _fuselage_profile(params)),
        *[
            _make_fuselage(
                f"Motor pod {index + 1}",
                pod_profile,
                center_x_mm=params.motor_station_x,
                center_y_m=y_mm / 1000,
                center_z_m=z_mm / 1000,
            )
            for index, (_, y_mm, z_mm) in enumerate(pod_centers)
        ],
    ]

    # The horizontal wing is the reference surface for coefficients. The CG
    # is left at the shared geometry origin because no mass properties are in
    # the parameter set; moments are therefore geometry-origin moments.
    horizontal_area_m2 = wing_span_mm * (params.wing_root_chord + params.wing_tip_chord) / 2 / 1_000_000
    taper = params.wing_tip_chord / params.wing_root_chord
    mean_aerodynamic_chord_m = (
        2
        / 3
        * wing_root_chord_m
        * (1 + taper + taper**2)
        / (1 + taper)
    )
    return asb.Airplane(
        name="NeedleTail cruciform tailsitter, early aerodynamic model",
        xyz_ref=[0, 0, 0],
        wings=[horizontal_wing, vertical_support],
        fuselages=fuselages,
        s_ref=horizontal_area_m2,
        c_ref=mean_aerodynamic_chord_m,
        b_ref=wing_span_mm / 1000,
    )


def _run_analysis(
    airplane: asb.Airplane,
    method: str,
    speed_mps: float,
    alpha_deg: np.ndarray,
    beta_deg: np.ndarray,
) -> dict:
    atmosphere = asb.Atmosphere(altitude=ALTITUDE_M)
    if method == "AeroBuildup":
        op_point = asb.OperatingPoint(
            atmosphere=atmosphere,
            velocity=np.full_like(alpha_deg, speed_mps, dtype=float),
            alpha=alpha_deg,
            beta=beta_deg,
        )
        return asb.AeroBuildup(
            airplane=airplane,
            op_point=op_point,
            xyz_ref=[0, 0, 0],
            model_size="large",
        ).run()
    if method == "VLM":
        # AeroSandbox's VLM currently expects a scalar OperatingPoint; run the
        # small sweep point-by-point and combine only the coefficients needed.
        point_results = []
        for alpha, beta in zip(alpha_deg, beta_deg):
            op_point = asb.OperatingPoint(
                atmosphere=atmosphere,
                velocity=speed_mps,
                alpha=float(alpha),
                beta=float(beta),
            )
            point_results.append(
                asb.VortexLatticeMethod(
                    airplane=airplane,
                    op_point=op_point,
                    xyz_ref=[0, 0, 0],
                    spanwise_resolution=12,
                    chordwise_resolution=8,
                    run_symmetric_if_possible=False,
                ).run()
            )
        return {
            field: np.asarray([point[field] for point in point_results], dtype=float)
            for field in COEFFICIENT_FIELDS
        }
    raise ValueError(f"Unknown analysis method: {method}")


COEFFICIENT_FIELDS = ("CL", "CY", "CD", "Cl", "Cm", "Cn")


def _write_rows(path: Path, rows: list[dict]) -> None:
    fields = [
        "method", "sweep", "speed_mps", "altitude_m", "alpha_deg", "beta_deg",
        "Re_wing_root", "Re_wing_tip", *COEFFICIENT_FIELDS,
    ]
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _scalar_at(values, index: int) -> float:
    return float(np.asarray(values, dtype=float)[index])


def generate_trends(airplane: asb.Airplane, output_dir: Path) -> list[dict]:
    rows: list[dict] = []
    alpha_sweep = ALPHAS_DEG
    alpha_zero = np.zeros_like(alpha_sweep)
    beta_sweep = BETAS_DEG
    beta_zero = np.zeros_like(beta_sweep)

    for speed in REFERENCE_SPEEDS_MPS:
        result = _run_analysis(airplane, "AeroBuildup", speed, alpha_sweep, alpha_zero)
        atmosphere = asb.Atmosphere(altitude=ALTITUDE_M)
        re_root = speed * (airplane.wings[0].xsecs[0].chord) / atmosphere.kinematic_viscosity()
        re_tip = speed * (airplane.wings[0].xsecs[1].chord) / atmosphere.kinematic_viscosity()
        for i, alpha in enumerate(alpha_sweep):
            row = {
                "method": "AeroBuildup",
                "sweep": "alpha",
                "speed_mps": speed,
                "altitude_m": ALTITUDE_M,
                "alpha_deg": float(alpha),
                "beta_deg": 0.0,
                "Re_wing_root": float(re_root),
                "Re_wing_tip": float(re_tip),
            }
            row.update({field: _scalar_at(result[field], i) for field in COEFFICIENT_FIELDS})
            rows.append(row)

    vlm_alpha = _run_analysis(
        airplane, "VLM", REFERENCE_SPEED_FOR_VLM_MPS, alpha_sweep, alpha_zero
    )
    for i, alpha in enumerate(alpha_sweep):
        row = {
            "method": "VLM",
            "sweep": "alpha",
            "speed_mps": REFERENCE_SPEED_FOR_VLM_MPS,
            "altitude_m": ALTITUDE_M,
            "alpha_deg": float(alpha),
            "beta_deg": 0.0,
            "Re_wing_root": "",
            "Re_wing_tip": "",
        }
        row.update({field: _scalar_at(vlm_alpha[field], i) for field in COEFFICIENT_FIELDS})
        rows.append(row)

    for method, speed in (
        ("AeroBuildup", REFERENCE_SPEED_FOR_VLM_MPS),
        ("VLM", REFERENCE_SPEED_FOR_VLM_MPS),
    ):
        result = _run_analysis(airplane, method, speed, beta_zero, beta_sweep)
        if method == "AeroBuildup":
            atmosphere = asb.Atmosphere(altitude=ALTITUDE_M)
            re_root = speed * airplane.wings[0].xsecs[0].chord / atmosphere.kinematic_viscosity()
            re_tip = speed * airplane.wings[0].xsecs[1].chord / atmosphere.kinematic_viscosity()
        else:
            re_root = re_tip = ""
        for i, beta in enumerate(beta_sweep):
            row = {
                "method": method,
                "sweep": "beta",
                "speed_mps": speed,
                "altitude_m": ALTITUDE_M,
                "alpha_deg": 0.0,
                "beta_deg": float(beta),
                "Re_wing_root": re_root,
                "Re_wing_tip": re_tip,
            }
            row.update({field: _scalar_at(result[field], i) for field in COEFFICIENT_FIELDS})
            rows.append(row)

    _write_rows(output_dir / "whole_aircraft_coefficients.csv", rows)
    return rows


def _plot_trends(rows: list[dict], output_dir: Path) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(13, 8), constrained_layout=True)
    panels = (("CL", "Lift coefficient"), ("CD", "Drag coefficient"), ("Cm", "Pitching moment coefficient"))
    colors = {10.0: "#377eb8", 15.0: "#4daf4a", 20.0: "#e41a1c"}

    for axis, (field, label) in zip(axes[0], panels):
        for method in ("AeroBuildup", "VLM"):
            speeds = (10.0, 15.0, 20.0) if method == "AeroBuildup" else (REFERENCE_SPEED_FOR_VLM_MPS,)
            for speed in speeds:
                subset = [
                    row for row in rows
                    if row["method"] == method
                    and row["sweep"] == "alpha"
                    and row["speed_mps"] == speed
                ]
                subset.sort(key=lambda row: row["alpha_deg"])
                style = "--" if method == "VLM" else "-"
                suffix = f"{speed:g} m/s" if method == "AeroBuildup" else "15 m/s"
                axis.plot(
                    [row["alpha_deg"] for row in subset],
                    [row[field] for row in subset],
                    linestyle=style,
                    color=colors[speed],
                    label=f"{method}, {suffix}",
                )
        axis.set_xlabel("Angle of attack α (deg)")
        axis.set_ylabel(field)
        axis.set_title(label)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=7)

    for axis, (field, label) in zip(axes[1], (("CY", "Side-force coefficient"), ("Cl", "Rolling moment coefficient"), ("Cn", "Yawing moment coefficient"))):
        for method in ("AeroBuildup", "VLM"):
            subset = [
                row for row in rows
                if row["method"] == method and row["sweep"] == "beta"
            ]
            subset.sort(key=lambda row: row["beta_deg"])
            axis.plot(
                [row["beta_deg"] for row in subset],
                [row[field] for row in subset],
                linestyle="-" if method == "AeroBuildup" else "--",
                label=method,
            )
        axis.set_xlabel("Sideslip β (deg)")
        axis.set_ylabel(field)
        axis.set_title(label)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)

    fig.suptitle(
        "NeedleTail whole-aircraft trends — conceptual, attached-flow range",
        fontsize=13,
    )
    fig.savefig(output_dir / "whole_aircraft_trends.png", dpi=180)
    plt.close(fig)


def _write_geometry(airplane: asb.Airplane, output_dir: Path) -> None:
    axes = airplane.draw_three_view(style="wireframe", show=False)
    figure = axes[0, 0].get_figure()
    figure.suptitle("AeroSandbox geometry adapter — ASB coordinates in meters")
    figure.savefig(output_dir / "whole_aircraft_geometry.png", dpi=160, bbox_inches="tight")
    plt.close(figure)

    figure = plt.figure(figsize=(9, 7))
    axis = figure.add_subplot(111, projection="3d")
    airplane.draw(
        backend="matplotlib",
        thin_wings=False,
        ax=axis,
        use_preset_view_angle="right_isometric",
        set_background_pane_color="#f3f6f9",
        set_background_pane_alpha=1.0,
        set_axis_visibility=False,
        show=False,
    )
    figure.set_size_inches(9, 7)
    figure.patch.set_facecolor("#f3f6f9")
    if axis.collections:
        axis.collections[0].set_facecolor("#8bb8d6")
        axis.collections[0].set_edgecolor("#3c5364")
        axis.collections[0].set_linewidth(0.25)
    axis.set_title("NeedleTail — AeroSandbox aircraft model", pad=18, fontsize=15)
    figure.savefig(
        output_dir / "needletail_aircraft_render.png",
        dpi=220,
        bbox_inches="tight",
        facecolor=figure.get_facecolor(),
    )
    plt.close(figure)


def run(output_dir: Path = DEFAULT_OUTPUT_DIR) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    airplane = build_airplane()
    airplane.save(output_dir / "needletail_airframe.asb", verbose=False)
    rows = generate_trends(airplane, output_dir)
    _plot_trends(rows, output_dir)
    _write_geometry(airplane, output_dir)

    params = PARAMETERS
    metadata = {
        "project": "NeedleTail",
        "aerosandbox_version": asb.__version__,
        "airframe_parameters": asdict(params),
        "coordinate_mapping": {
            "project_axes": "+X nose, +Y right, +Z up; geometry dimensions in mm",
            "aerosandbox_axes": "+X aft, +Y right, +Z up; geometry dimensions in m",
            "mapping": "x_asb = -x_project / 1000; y_asb = y_project / 1000; z_asb = z_project / 1000",
        },
        "geometry_matches": {
            "horizontal_wing": f"{params.wing_airfoil}, root {params.wing_root_chord:g} mm / tip {params.wing_tip_chord:g} mm, physical TE {params.wing_trailing_edge:g} mm",
            "vertical_support": f"{params.support_airfoil}, {params.support_chord:g} mm, physical TE {params.support_trailing_edge:g} mm",
            "fuselage": "25 axial samples from the CAD profile spline",
            "motor_pods": "four CAD-profile bodies, each with 25 axial samples",
        },
        "reference_values": {
            "area_m2": params.wing_planform_area / 1_000_000,
            "span_m": params.wing_span / 1000,
            "mean_aerodynamic_chord_m": airplane.c_ref,
            "moment_reference": "shared geometry origin; no CG or mass properties supplied",
        },
        "conditions": {
            "altitude_m": ALTITUDE_M,
            "aerobuildup_speeds_mps": list(REFERENCE_SPEEDS_MPS),
            "vlm_reference_speed_mps": REFERENCE_SPEED_FOR_VLM_MPS,
            "alpha_deg": ALPHAS_DEG.tolist(),
            "beta_deg": BETAS_DEG.tolist(),
            "n_crit": N_CRIT,
        },
        "assumptions_and_limits": [
            "VLM is inviscid and represents attached-flow lifting surfaces; it does not predict stall.",
            "AeroBuildup uses empirical component models and NeuralFoil section data; low-Reynolds-number transition and roughness remain uncertain.",
            "The propeller blades, thrust, and powered propwash are not modeled.",
            "The central body and four pods are represented as axisymmetric bodies; the model follows the project CAD profile splines with 25 axial samples each.",
            "Coefficients use horizontal-wing planform area as S_ref; moments use the geometry origin because mass properties and CG are not supplied.",
        ],
        "outputs": [
            "needletail_airframe.asb",
            "whole_aircraft_coefficients.csv",
            "whole_aircraft_trends.png",
            "whole_aircraft_geometry.png",
            "needletail_aircraft_render.png",
        ],
    }
    (output_dir / "whole_aircraft_model.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    return {"output_dir": str(output_dir), "rows": len(rows), "metadata": metadata}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    result = run(args.output_dir.resolve())
    print(f"Wrote {result['rows']} coefficient rows to {result['output_dir']}")
    print(
        "Wing reference area: "
        f"{result['metadata']['reference_values']['area_m2']:.6f} m^2; "
        f"span: {result['metadata']['reference_values']['span_m']:.3f} m"
    )


if __name__ == "__main__":
    main()
