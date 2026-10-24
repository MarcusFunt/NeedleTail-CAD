"""Compare two native STEP airframe OMLs under the same CAD kernel.

Outputs explicit geometry metrics, rather than attributing aerodynamic performance
to surface area or volume alone. Both STEP inputs must contain one valid solid.
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path

from cadgen import build123d as bd

OUTPUT = Path(__file__).resolve().parent / "tmp" / "native-geometry-comparison"


def measure(filename):
    file = Path(filename).resolve()
    if not file.is_file() or not file.stat().st_size:
        raise FileNotFoundError(f"Missing/nonempty STEP required: {file}")
    shape = bd.import_step(str(file))
    solids = shape.solids()
    if not shape.is_valid or len(solids) != 1:
        raise ValueError(f"{file} must reimport as exactly one valid solid; got {len(solids)}")
    b = solids[0].bounding_box()
    return {
        "volume_mm3": float(solids[0].volume),
        "surface_area_mm2": float(solids[0].area),
        "length_x_mm": float(b.max.X - b.min.X),
        "span_y_mm": float(b.max.Y - b.min.Y),
        "height_z_mm": float(b.max.Z - b.min.Z),
        "min_x_mm": float(b.min.X),
        "max_x_mm": float(b.max.X),
        "min_y_mm": float(b.min.Y),
        "max_y_mm": float(b.max.Y),
        "min_z_mm": float(b.min.Z),
        "max_z_mm": float(b.max.Z),
    }


def compare(old_step, new_step, output_dir=OUTPUT):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    old = measure(old_step)
    new = measure(new_step)
    keys = list(old)
    rows = [
        {
            "metric": name,
            "previous": old[name],
            "compact": new[name],
            "delta_absolute": new[name] - old[name],
            "delta_percent": 100 * (new[name] / old[name] - 1) if abs(old[name]) > 1e-12 else "",
        }
        for name in keys
    ]
    with (output_dir / "native_step_geometry_metrics.csv").open("w", encoding="utf8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for source, output_name in (
        (old_step, "previous_airframe_oml.step"),
        (new_step, "compact_airframe_oml.step"),
    ):
        shutil.copyfile(Path(source).resolve(), output_dir / output_name)
    (output_dir / "native_step_geometry_metadata.json").write_text(
        json.dumps({
            "previous_commit": "3ea08b234742622e8806827458b4608dd3ce8321",
            "compact_commit": "d1ae7b8f927cd9823dd1de81d4a77ea2830d1d40",
            "method": "direct OCCT import of individual valid BREP solids with one identical pinned kernel",
            "metrics": {"previous": old, "compact": new},
            "limits": [
                "No CFD, flow, drag, or propulsive quantities can be inferred from these metrics.",
                "Matched SU2 transitional CFD and mesh convergence are needed to compare viscous drag.",
            ],
        }, indent=2) + "\n", encoding="utf8"
    )
    for row in rows:
        print(
            f"STEP_COMPARISON {row['metric']} "
            f"previous={row['previous']:.6f} compact={row['compact']:.6f} "
            f"delta_pct={row['delta_percent'] if row['delta_percent'] != '' else 'n/a'}"
        )
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--old-step", type=Path, required=True)
    p.add_argument("--new-step", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, default=OUTPUT)
    a = p.parse_args()
    compare(a.old_step, a.new_step, a.output_dir)
