"""Repeat the equal-lift comparison against the pre-optimization wing.

Run with ``uv run --locked --extra aero python compare_airframe_aero.py``.
The baseline restores the original arms and pod layout, with 72/44 mm wing
chords and 200 mm span. The fuselage retains current parameters. This is an untrimmed, unpowered conceptual comparison,
not a fixed-weight mission optimization or a structural analysis.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

from airframe_geometry import PARAMETERS, AirframeParameters
from whole_aircraft_aero import _run_analysis, build_airplane


def compare(
    baseline: AirframeParameters | None = None,
    candidate_params: AirframeParameters = PARAMETERS,
    *, body_sample_count: int = 25,
) -> dict:
    """Compare supplied designs at equal lift; default to the original arm layout."""
    if baseline is None:
        baseline = replace(
            PARAMETERS, wing_root_chord=72.0, wing_tip_chord=44.0,
            wing_airfoil="NACA2415", support_airfoil="NACA0015",
            horizontal_motor_offset=100.0, vertical_motor_offset=85.0,
            wing_thickness_scale=1.0, wing_trailing_edge=0.30,
            support_chord=48.0, support_thickness_scale=1.0, support_trailing_edge=0.30,
            assembly_x_offset=0.0, motor_station_x=3.0, motor_pod_length=110.0,
            motor_mount_face_offset_from_pod_center=40.0,
        )
    for params in (baseline, candidate_params):
        params.validate_motor_prop_interface()
        params.validate_arm_pod_fit()
        params.validate_propeller_clearance()
    original = build_airplane(baseline, body_sample_count=body_sample_count)
    candidate = build_airplane(candidate_params, body_sample_count=body_sample_count)
    speeds = np.repeat([10.0, 15.0, 20.0], 3)
    anchors = np.tile([2.0, 4.0, 6.0], 3)
    zeros = np.zeros_like(anchors)
    reference = _run_analysis(original, "AeroBuildup", speeds, anchors, zeros)
    target = np.asarray(reference["CL"]) * original.s_ref
    reference_drag = np.asarray(reference["CD"]) * original.s_ref

    def analyze(alpha):
        return _run_analysis(candidate, "AeroBuildup", speeds, alpha, zeros)

    # Vectorized bisection solves each operating point at equal dimensional
    # lift (CL*S), avoiding misleading gains from changing reference area.
    lower, upper = np.full_like(anchors, -4.0), np.full_like(anchors, 8.0)
    lift_lower = np.asarray(analyze(lower)["CL"]) * candidate.s_ref
    lift_upper = np.asarray(analyze(upper)["CL"]) * candidate.s_ref
    if not (np.all(np.isfinite(target)) and np.all(reference_drag > 0)
            and np.all(lift_lower <= target) and np.all(target <= lift_upper)):
        raise ValueError("Equal-lift targets are invalid or outside the -4 to 8 degree bracket")
    for _ in range(30):
        middle = (lower + upper) / 2
        below_target = np.asarray(analyze(middle)["CL"]) * candidate.s_ref < target
        lower = np.where(below_target, middle, lower)
        upper = np.where(below_target, upper, middle)
    alpha = (lower + upper) / 2
    result = analyze(alpha)
    lift = np.asarray(result["CL"]) * candidate.s_ref
    drag = np.asarray(result["CD"]) * candidate.s_ref
    if not (np.all(np.isfinite(drag)) and np.all(drag > 0)
            and np.allclose(lift, target, rtol=1e-6, atol=1e-10)):
        raise ValueError("Equal-lift solution failed verification")
    reductions = 100 * (1 - drag / reference_drag)
    points = [
        {
            "speed_mps": float(speeds[i]),
            "baseline_alpha_deg": float(anchors[i]),
            "candidate_alpha_deg": float(alpha[i]),
            "lift_area_m2": float(target[i]),
            "baseline_drag_area_m2": float(reference_drag[i]),
            "candidate_drag_area_m2": float(drag[i]),
            "drag_reduction_pct": float(reductions[i]),
            "baseline_lift_to_drag": float(target[i] / reference_drag[i]),
            "candidate_lift_to_drag": float(target[i] / drag[i]),
        }
        for i in range(len(anchors))
    ]
    return {
        "method": "AeroBuildup, large NeuralFoil model; sea level, beta=0",
        "body_sample_count": body_sample_count,
        "baseline_parameters": asdict(baseline),
        "candidate_parameters": asdict(candidate_params),
        "mean_drag_reduction_pct": float(np.mean(reductions)),
        "worst_drag_reduction_pct": float(np.min(reductions)),
        "points": points,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path(__file__).resolve().parent / "tmp" / "airframe-comparison",
    )
    args = parser.parse_args()
    comparison = compare()
    output = args.output_dir.resolve() / "comparison.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(comparison, indent=2), encoding="utf-8")
    print(f"Equal-lift drag reduction: mean {comparison['mean_drag_reduction_pct']:.3f}%, "
          f"minimum {comparison['worst_drag_reduction_pct']:.3f}%")
    print(f"Wrote {output}")
