"""Compare catalog-derived NACA 0015 / SD8020 bodies with the custom reference.

Run with ``uv run --locked --extra aero python compare_body_airfoils.py``.
Geometry checks use native CAD; drag comparisons are untrimmed AeroBuildup
estimates at equal lift. The airfoil names describe meridian provenance,
not published 2D airfoil performance for these revolved bodies.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, replace
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from airframe_geometry import (
    PARAMETERS, _fuselage, _sample_spline_radius_profile, build_airframe_oml,
    build_motor_pod, fuselage_radius_stations, motor_pod_radius_stations,
)
from compare_airframe_aero import compare
from profiles.airfoil_coordinates import AIRFOIL_CATALOG
from whole_aircraft_aero import _pod_profile


def run(output_dir: Path) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    designs = {
        "custom": replace(PARAMETERS, fuselage_body_airfoil=None, motor_pod_body_airfoil=None),
        "NACA0015": replace(PARAMETERS, fuselage_body_airfoil="NACA0015", motor_pod_body_airfoil="NACA0015"),
        "SD8020": replace(PARAMETERS, fuselage_body_airfoil="SD8020", motor_pod_body_airfoil="SD8020"),
    }
    report = {
        "active_baseline": "NACA0015",
        "sources": {name: AIRFOIL_CATALOG[name] for name in ("NACA0015", "SD8020")},
        "designs": {},
        "comparisons": {},
        "limits": "Scaled/truncated revolved profiles; unpowered, untrimmed empirical estimates; no validated viscous CFD or loaded CG.",
    }
    figure, axes = plt.subplots(2, 1, figsize=(11, 6), constrained_layout=True)
    for name, params in designs.items():
        print(f"Checking {name} geometry", flush=True)
        params.validate_motor_prop_interface()
        params.validate_propeller_clearance()
        body, pod, oml = _fuselage(params), build_motor_pod(params), build_airframe_oml(params)
        if not (body.is_valid and pod.is_valid and oml.is_valid and len(oml.solids()) == 1):
            raise ValueError(f"Invalid CAD geometry: {name}")
        report["designs"][name] = {
            "parameters": asdict(params),
            "fuselage_volume_cm3": body.volume / 1000,
            "fuselage_surface_mm2": body.area,
            "fuselage_max_diameter_mm": body.bounding_box().size.Y,
            "single_pod_volume_cm3": pod.volume / 1000,
            "single_pod_surface_mm2": pod.area,
            "single_pod_max_diameter_mm": pod.bounding_box().size.Y,
            "oml_valid_single_solid": True,
            "clearances_mm": params.propeller_clearance_report,
        }
        for axis, stations, dx in (
            (axes[0], fuselage_radius_stations(params), 0),
            (axes[1], _pod_profile(params), params.motor_pod_center_x),
        ):
            xy = np.asarray(_sample_spline_radius_profile(stations))
            axis.plot(xy[:, 0] + dx, xy[:, 1], label=name)
            axis.plot(xy[:, 0] + dx, -xy[:, 1], color=axis.lines[-1].get_color())
    for axis, title in zip(axes, ("Fuselage meridian", "Motor-pod meridian, including rounded aft cap")):
        axis.set(title=title, xlabel="Aircraft X (mm), aft ← → nose", ylabel="Radius (mm)")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(alpha=.25)
        axis.legend()
    figure.savefig(output_dir / "body_profiles.png", dpi=180)
    plt.close(figure)

    comparisons = (
        ("NACA0015_vs_custom", "custom", "NACA0015", 25),
        ("SD8020_vs_custom", "custom", "SD8020", 25),
        ("SD8020_vs_NACA0015", "NACA0015", "SD8020", 25),
        ("SD8020_vs_NACA0015_dense", "NACA0015", "SD8020", 101),
    )
    for key, original, candidate, count in comparisons:
        print(f"Comparing {key}, {count} body stations", flush=True)
        result = compare(designs[original], designs[candidate], body_sample_count=count)
        report["comparisons"][key] = result
        print(f"  Mean drag reduction {result['mean_drag_reduction_pct']:.3f}%; "
              f"minimum {result['worst_drag_reduction_pct']:.3f}%", flush=True)
        (output_dir / "comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "tmp" / "airfoil-bodies")
    args = parser.parse_args()
    run(args.output_dir.resolve())
