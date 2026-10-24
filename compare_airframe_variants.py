"""Controlled comparison of old span layout vs compact 178/148 mm layout.

Uses common current AeroSandbox component physics for both variants; this does
not replace native CAD/STEP RANS comparison or resolve powered slipstream.
"""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path

import aerosandbox as asb
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from airframe_geometry import PARAMETERS
from whole_aircraft_aero import build_airplane

DEFAULT_OUTPUT = Path(__file__).resolve().parent / "tmp" / "airframe-comparison"
SPEEDS_MPS = (10.0, 15.0, 20.0)
ALPHAS_DEG = (-2.0, 0.0, 2.0, 4.0, 6.0)
PREVIOUS_COMMIT = "3ea08b234742622e8806827458b4608dd3ce8321"
CURRENT_COMMIT = "d1ae7b8f927cd9823dd1de81d4a77ea2830d1d40"


def evaluate(
    speeds=SPEEDS_MPS,
    alphas=ALPHAS_DEG,
) -> list[dict]:
    variants = {
        "previous_240x170_proxy": (240.0, 170.0),
        "compact_178x148": (PARAMETERS.wing_span, PARAMETERS.support_span),
    }
    rows = []
    atmosphere = asb.Atmosphere(altitude=0)
    rho = float(atmosphere.density())
    for variant, (wing_span, support_span) in variants.items():
        airplane = build_airplane(
            wing_span_mm=wing_span, support_span_mm=support_span,
        )
        for speed in speeds:
            for alpha in alphas:
                op = asb.OperatingPoint(
                    atmosphere=atmosphere, velocity=float(speed),
                    alpha=float(alpha), beta=0.0,
                )
                coefficients = asb.AeroBuildup(
                    airplane=airplane, op_point=op, model_size="large",
                ).run()
                q = 0.5 * rho * speed ** 2
                cl = float(coefficients["CL"])
                cd = float(coefficients["CD"])
                if not (np.isfinite(cl) and np.isfinite(cd) and cd > 0):
                    raise ValueError(
                        f"Nonfinite/nonpositive AeroBuildup result: {variant}, "
                        f"{speed} m/s, alpha={alpha}, CL={cl}, CD={cd}"
                    )
                rows.append({
                    "variant": variant,
                    "speed_mps": float(speed),
                    "alpha_deg": float(alpha),
                    "wing_span_mm": wing_span,
                    "support_span_mm": support_span,
                    "s_ref_m2": float(airplane.s_ref),
                    "CL": cl,
                    "CD": cd,
                    "L_N": q * airplane.s_ref * cl,
                    "D_N": q * airplane.s_ref * cd,
                    "L_to_D": cl / cd,
                    "density_kg_m3": rho,
                })
    return rows


def generate(output_dir=DEFAULT_OUTPUT, speeds=SPEEDS_MPS, alphas=ALPHAS_DEG):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = evaluate(speeds=speeds, alphas=alphas)
    csv_path = output_dir / "variant_comparison.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metadata = {
        "previous_commit": PREVIOUS_COMMIT,
        "compact_commit": CURRENT_COMMIT,
        "method": "AeroSandbox AeroBuildup with NeuralFoil-large, unpowered",
        "common_inputs": {
            "motor_positions_mm": [-PARAMETERS.horizontal_motor_offset, PARAMETERS.horizontal_motor_offset, -PARAMETERS.vertical_motor_offset, PARAMETERS.vertical_motor_offset],
            "fuselage_pod_profiles": "current common profile interpolation",
            "horizontal_airfoil": PARAMETERS.wing_airfoil,
            "vertical_airfoil": PARAMETERS.support_airfoil,
            "alpha_deg": list(alphas),
            "speeds_mps": list(speeds),
            "altitude_m": 0,
        },
        "limitations": [
            "Historic configuration is reconstructed by overriding spans; it is not a mesh of the earlier native STEP.",
            "Both variants use current airfoil trailing-edge normalization and ruled-wing proxy.",
            "Physical force comparisons account for different reference wing areas; CL/CD alone cannot be used to compare aircraft drag.",
            "Neither variant includes propwash, actual mass, trim, actuator limits, airframe-junction interference or experimental validation.",
            "A higher-fidelity comparison requires each historical STEP solid, matched meshing, turbulence/transition settings and grid convergence.",
        ],
    }
    (output_dir / "comparison_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    fig, axs = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
    for variant in sorted({r["variant"] for r in rows}):
        subset = [r for r in rows if r["variant"] == variant and r["speed_mps"] == 15.0]
        if not subset:
            continue
        axs[0].plot([r["alpha_deg"] for r in subset], [r["D_N"] for r in subset], marker="o", label=variant)
        axs[1].plot([r["alpha_deg"] for r in subset], [r["L_N"] for r in subset], marker="o", label=variant)
    for axis, title, unit in ((axs[0], "Unpowered drag at 15 m/s", "Drag (N)"), (axs[1], "Lift at 15 m/s", "Lift (N)")):
        axis.set_title(title)
        axis.set_xlabel("Angle of attack (degrees)")
        axis.set_ylabel(unit)
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    fig.savefig(output_dir / "variant_force_comparison.png", dpi=175)
    plt.close(fig)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    rows = generate(args.output_dir)
    print(f"Wrote {len(rows)} common-conditions AeroBuildup comparison rows to {args.output_dir}")

if __name__ == "__main__":
    main()
