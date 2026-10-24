"""Generate geometry-matched SD7003 and SD8020 airfoil polars.

Requires AeroSandbox (which includes NeuralFoil). XFOIL is optional and must
be installed separately; pass its executable with ``--xfoil`` to generate the
cross-check files.

By default, generated outputs go under ``tmp/airfoil-polars`` so the CAD
dependency set and committed project configuration remain untouched.
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path

import aerosandbox as asb
import aerosandbox.numpy as np
import neuralfoil

from airframe_geometry import PARAMETERS, _section_coordinates


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = ROOT / "tmp" / "airfoil-polars"
REYNOLDS_NUMBERS = (20_000, 50_000, 100_000, 200_000, 500_000)
ALPHAS = np.arange(-10.0, 18.01, 0.5)
XFOIL_REYNOLDS_NUMBERS = (50_000, 100_000, 200_000)
XFOIL_ALPHAS = np.array([-2.0, 0.0, 2.0, 4.0, 6.0, 8.0])
N_CRIT = 9.0

SECTIONS = (
    ("wing_root", PARAMETERS.wing_airfoil, PARAMETERS.wing_root_chord),
    ("wing_tip", PARAMETERS.wing_airfoil, PARAMETERS.wing_tip_chord),
    ("vertical_support", PARAMETERS.support_airfoil, PARAMETERS.support_chord),
)

POLAR_FIELDS = (
    "CL",
    "CD",
    "CM",
    "Top_Xtr",
    "Bot_Xtr",
    "analysis_confidence",
)


def _geometry_matched_airfoil(section_name: str, airfoil_name: str, chord_mm: float):
    """Create an AeroSandbox airfoil from the exact CAD section coordinates."""
    upper, lower = _section_coordinates(
        airfoil_name,
        chord_mm,
        PARAMETERS.wing_trailing_edge
        if section_name.startswith("wing_")
        else PARAMETERS.support_trailing_edge,
    )
    # _section_coordinates returns LE-to-TE upper and lower surfaces. Store a
    # standard airfoil contour from upper TE, over the leading edge, to lower
    # TE so XFOIL reads the intended upper and lower surfaces.
    coordinates = np.asarray(list(reversed(upper)) + lower[1:], dtype=float)
    airfoil = asb.Airfoil(name=f"{section_name}_{airfoil_name}_CAD_TE", coordinates=coordinates)
    te_thickness_mm = (
        PARAMETERS.wing_trailing_edge
        if section_name.startswith("wing_")
        else PARAMETERS.support_trailing_edge
    )
    return airfoil, coordinates, te_thickness_mm


def _write_airfoil_dat(path: Path, name: str, coordinates) -> None:
    with path.open("w", encoding="ascii", newline="\n") as file:
        file.write(f"{name}\n")
        for x, y in coordinates:
            file.write(f"{float(x):.10f} {float(y):.10f}\n")


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def generate_neuralfoil(output_dir: Path):
    rows: list[dict] = []
    section_data: dict[str, dict] = {}

    for section_name, airfoil_name, chord_mm in SECTIONS:
        airfoil, coordinates, te_thickness_mm = _geometry_matched_airfoil(
            section_name, airfoil_name, chord_mm
        )
        section_data[section_name] = {
            "airfoil": airfoil,
            "coordinates": coordinates,
            "airfoil_name": airfoil_name,
            "chord_mm": chord_mm,
            "te_thickness_mm": te_thickness_mm,
        }
        _write_airfoil_dat(
            output_dir / f"{section_name}_{airfoil_name}_cad_profile.dat",
            f"{section_name} {airfoil_name} CAD-matched TE",
            coordinates,
        )

        for reynolds in REYNOLDS_NUMBERS:
            aero = airfoil.get_aero_from_neuralfoil(
                alpha=ALPHAS,
                Re=reynolds,
                mach=0.0,
                n_crit=N_CRIT,
                model_size="large",
            )
            for index, alpha in enumerate(ALPHAS):
                row = {
                    "section": section_name,
                    "airfoil": airfoil_name,
                    "chord_mm": chord_mm,
                    "trailing_edge_mm": te_thickness_mm,
                    "trailing_edge_ratio": te_thickness_mm / chord_mm,
                    "Re": reynolds,
                    "alpha_deg": float(alpha),
                    "model": "NeuralFoil-large",
                }
                for field in POLAR_FIELDS:
                    row[field] = float(np.asarray(aero[field])[index])
                rows.append(row)

    fields = [
        "section", "airfoil", "chord_mm", "trailing_edge_mm",
        "trailing_edge_ratio", "Re", "alpha_deg", "model", *POLAR_FIELDS,
    ]
    _write_csv(output_dir / "airfoil_section_polars_neuralfoil.csv", fields, rows)
    return section_data, rows


def generate_xfoil(
    output_dir: Path,
    xfoil_command: str,
    neuralfoil_rows: list[dict],
) -> tuple[list[dict], list[dict]]:
    xfoil_rows: list[dict] = []
    comparison_rows: list[dict] = []
    nf_lookup = {
        (row["section"], row["Re"], row["alpha_deg"]): row
        for row in neuralfoil_rows
    }

    for section_name, airfoil_name, chord_mm in SECTIONS:
        airfoil, _, te_thickness_mm = _geometry_matched_airfoil(
            section_name, airfoil_name, chord_mm
        )
        for reynolds in XFOIL_REYNOLDS_NUMBERS:
            try:
                working_directory = (
                    output_dir / "xfoil_work" / f"{section_name}_Re{reynolds}"
                )
                working_directory.mkdir(parents=True, exist_ok=True)
                solver = asb.XFoil(
                    airfoil=airfoil,
                    Re=reynolds,
                    n_crit=N_CRIT,
                    max_iter=200,
                    xfoil_command=xfoil_command,
                    xfoil_repanel=True,
                    timeout=30,
                    working_directory=working_directory,
                )
                result = solver.alpha(XFOIL_ALPHAS, start_at=0)
            except Exception as error:  # Keep failures explicit in the generated report.
                xfoil_rows.append({
                    "section": section_name,
                    "airfoil": airfoil_name,
                    "chord_mm": chord_mm,
                    "trailing_edge_mm": te_thickness_mm,
                    "Re": reynolds,
                    "alpha_deg": "",
                    "status": f"error: {type(error).__name__}: {error}",
                    "CL": "",
                    "CD": "",
                    "CM": "",
                    "Top_Xtr": "",
                    "Bot_Xtr": "",
                })
                continue

            returned_alphas = np.asarray(result.get("alpha", []), dtype=float)
            for index, alpha in enumerate(returned_alphas):
                values = {
                    field: float(np.asarray(result[field])[index])
                    for field in POLAR_FIELDS
                    if field in result
                }
                row = {
                    "section": section_name,
                    "airfoil": airfoil_name,
                    "chord_mm": chord_mm,
                    "trailing_edge_mm": te_thickness_mm,
                    "Re": reynolds,
                    "alpha_deg": float(alpha),
                    "status": "converged",
                    **{field: values.get(field, "") for field in POLAR_FIELDS if field != "analysis_confidence"},
                }
                xfoil_rows.append(row)

                nf = nf_lookup.get((section_name, reynolds, float(alpha)))
                if nf is None:
                    continue
                comparison_rows.append({
                    "section": section_name,
                    "airfoil": airfoil_name,
                    "chord_mm": chord_mm,
                    "trailing_edge_mm": te_thickness_mm,
                    "Re": reynolds,
                    "alpha_deg": float(alpha),
                    "NeuralFoil_confidence": nf["analysis_confidence"],
                    **{
                        f"XFOIL_{field}": values.get(field, "")
                        for field in ("CL", "CD", "CM", "Top_Xtr", "Bot_Xtr")
                    },
                    **{
                        f"NeuralFoil_{field}": nf[field]
                        for field in ("CL", "CD", "CM", "Top_Xtr", "Bot_Xtr")
                    },
                    **{
                        f"delta_{field}_XFOIL_minus_NeuralFoil": (
                            values[field] - nf[field] if field in values else ""
                        )
                        for field in ("CL", "CD", "CM", "Top_Xtr", "Bot_Xtr")
                    },
                })

    xfoil_fields = [
        "section", "airfoil", "chord_mm", "trailing_edge_mm", "Re",
        "alpha_deg", "status", "CL", "CD", "CM", "Top_Xtr", "Bot_Xtr",
    ]
    comparison_fields = list(comparison_rows[0]) if comparison_rows else [
        "section", "airfoil", "chord_mm", "trailing_edge_mm", "Re",
        "alpha_deg", "NeuralFoil_confidence",
    ]
    _write_csv(output_dir / "airfoil_section_polars_xfoil.csv", xfoil_fields, xfoil_rows)
    _write_csv(output_dir / "airfoil_polars_xfoil_comparison.csv", comparison_fields, comparison_rows)
    return xfoil_rows, comparison_rows


def plot_neuralfoil(output_dir: Path, rows: list[dict]) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, len(SECTIONS), figsize=(15, 8))
    for column, (section_name, airfoil_name, chord_mm) in enumerate(SECTIONS):
        section_rows = [row for row in rows if row["section"] == section_name]
        for reynolds in REYNOLDS_NUMBERS:
            polar = [row for row in section_rows if row["Re"] == reynolds]
            alpha = [row["alpha_deg"] for row in polar]
            cl = [row["CL"] for row in polar]
            cd = [row["CD"] for row in polar]
            confidence = [row["analysis_confidence"] for row in polar]
            label = f"Re={reynolds:,}"
            axes[0, column].plot(alpha, cl, label=label)
            axes[1, column].plot(cd, cl, label=label)
            low_confidence = [value <= 0.9 for value in confidence]
            if any(low_confidence):
                axes[0, column].scatter(
                    [a for a, low in zip(alpha, low_confidence) if low],
                    [v for v, low in zip(cl, low_confidence) if low],
                    marker="x", s=12, color="black", alpha=0.65,
                )
        axes[0, column].set_title(f"{section_name}: {airfoil_name}, c={chord_mm:g} mm")
        axes[0, column].set_xlabel("Angle of attack [deg]")
        axes[0, column].set_ylabel("CL [-]")
        axes[1, column].set_xlabel("CD [-] (plot window to 0.08)")
        axes[1, column].set_ylabel("CL [-]")
        # Keep the attached-flow drag bucket legible; the full alpha sweep and
        # all larger post-stall drag values remain in the CSV.
        axes[1, column].set_xlim(0, 0.08)
        axes[0, column].grid(True, alpha=0.25)
        axes[1, column].grid(True, alpha=0.25)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Geometry-matched NeedleTail section polars (NeuralFoil large; x = confidence ≤ 0.9)")
    fig.tight_layout()
    fig.savefig(output_dir / "airfoil_section_polars_neuralfoil.png", dpi=180)
    plt.close(fig)


def summarize_comparison(rows: list[dict]) -> list[dict]:
    groups: dict[tuple[str, int], list[dict]] = {}
    for row in rows:
        groups.setdefault((row["section"], int(row["Re"])), []).append(row)

    summary = []
    for (section_name, reynolds), group in sorted(groups.items()):
        deltas = {
            field: [float(row[f"delta_{field}_XFOIL_minus_NeuralFoil"]) for row in group]
            for field in ("CL", "CD", "CM")
        }
        relative_cd = [
            abs(float(row["delta_CD_XFOIL_minus_NeuralFoil"]))
            / max(abs(float(row["XFOIL_CD"])), 1e-12)
            * 100
            for row in group
        ]
        section = next(item for item in SECTIONS if item[0] == section_name)
        summary.append({
            "section": section_name,
            "airfoil": section[1],
            "chord_mm": section[2],
            "Re": reynolds,
            "converged_points": len(group),
            "mean_abs_delta_CL": sum(map(abs, deltas["CL"])) / len(group),
            "max_abs_delta_CL": max(map(abs, deltas["CL"])),
            "mean_relative_delta_CD_pct": sum(relative_cd) / len(relative_cd),
            "mean_abs_delta_CM": sum(map(abs, deltas["CM"])) / len(group),
            "mean_NeuralFoil_confidence": sum(
                float(row["NeuralFoil_confidence"]) for row in group
            ) / len(group),
        })
    return summary


def plot_xfoil_comparison(
    output_dir: Path,
    neuralfoil_rows: list[dict],
    comparison_rows: list[dict],
) -> None:
    if not comparison_rows:
        return
    import matplotlib.pyplot as plt

    colors = plt.cm.viridis(np.linspace(0.1, 0.9, len(XFOIL_REYNOLDS_NUMBERS)))
    fig, axes = plt.subplots(2, len(SECTIONS), figsize=(15, 8))
    for column, (section_name, airfoil_name, chord_mm) in enumerate(SECTIONS):
        for color, reynolds in zip(colors, XFOIL_REYNOLDS_NUMBERS):
            nf = [
                row for row in neuralfoil_rows
                if row["section"] == section_name
                and row["Re"] == reynolds
                and XFOIL_ALPHAS[0] <= row["alpha_deg"] <= XFOIL_ALPHAS[-1]
            ]
            xf = [
                row for row in comparison_rows
                if row["section"] == section_name and row["Re"] == reynolds
            ]
            if not nf or not xf:
                continue
            label = f"Re={reynolds:,}"
            axes[0, column].plot(
                [row["alpha_deg"] for row in nf],
                [row["CL"] for row in nf],
                color=color,
                label=label,
            )
            axes[0, column].scatter(
                [row["alpha_deg"] for row in xf],
                [row["XFOIL_CL"] for row in xf],
                marker="o", facecolors="none", edgecolors=[color], zorder=3,
            )
            axes[1, column].plot(
                [row["CD"] for row in nf],
                [row["CL"] for row in nf],
                color=color,
                label=label,
            )
            axes[1, column].scatter(
                [row["XFOIL_CD"] for row in xf],
                [row["XFOIL_CL"] for row in xf],
                marker="o", facecolors="none", edgecolors=[color], zorder=3,
            )
        axes[0, column].set_title(f"{section_name}: {airfoil_name}, c={chord_mm:g} mm")
        axes[0, column].set_xlabel("Angle of attack [deg]")
        axes[0, column].set_ylabel("CL [-]")
        axes[1, column].set_xlabel("CD [-]")
        axes[1, column].set_ylabel("CL [-]")
        axes[1, column].set_xlim(left=0)
        axes[0, column].grid(True, alpha=0.25)
        axes[1, column].grid(True, alpha=0.25)
    axes[0, 0].legend(fontsize=8, title="Lines: NeuralFoil\nCircles: XFOIL")
    fig.suptitle("NeuralFoil vs XFOIL at selected section-polar points")
    fig.tight_layout()
    fig.savefig(output_dir / "airfoil_polars_xfoil_comparison.png", dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--xfoil",
        type=str,
        default=shutil.which("xfoil"),
        help="Path to an XFOIL executable; omit to generate NeuralFoil results only.",
    )
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    section_data, neuralfoil_rows = generate_neuralfoil(output_dir)
    plot_neuralfoil(output_dir, neuralfoil_rows)

    xfoil_rows: list[dict] = []
    comparison_rows: list[dict] = []
    if args.xfoil:
        xfoil_rows, comparison_rows = generate_xfoil(
            output_dir, args.xfoil, neuralfoil_rows
        )
        plot_xfoil_comparison(output_dir, neuralfoil_rows, comparison_rows)
    comparison_summary = summarize_comparison(comparison_rows)
    if comparison_summary:
        _write_csv(
            output_dir / "airfoil_polars_xfoil_summary.csv",
            list(comparison_summary[0]),
            comparison_summary,
        )

    metadata = {
        "aerosandbox_version": asb.__version__,
        "neuralfoil_version": neuralfoil.__version__,
        "xfoil_command": args.xfoil,
        "sections": {
            name: {
                "airfoil": data["airfoil_name"],
                "chord_mm": data["chord_mm"],
                "physical_trailing_edge_mm": data["te_thickness_mm"],
                "trailing_edge_ratio": data["te_thickness_mm"] / data["chord_mm"],
                "coordinate_points": len(data["coordinates"]),
            }
            for name, data in section_data.items()
        },
        "neuralfoil": {
            "model_size": "large",
            "n_crit": N_CRIT,
            "reynolds_numbers": REYNOLDS_NUMBERS,
            "alpha_start_deg": float(ALPHAS[0]),
            "alpha_end_deg": float(ALPHAS[-1]),
            "alpha_step_deg": 0.5,
        },
        "xfoil": {
            "reynolds_numbers": XFOIL_REYNOLDS_NUMBERS if args.xfoil else [],
            "requested_alpha_deg": XFOIL_ALPHAS.tolist() if args.xfoil else [],
            "returned_points": sum(row["status"] == "converged" for row in xfoil_rows),
            "matched_comparison_points": len(comparison_rows),
        },
        "caveat": (
            "Reynolds numbers are exploratory because NeedleTail flight speed and altitude are not specified. "
            "NeuralFoil is a surrogate for XFOIL, so this comparison checks implementation agreement, "
            "not independent experimental accuracy."
        ),
    }
    (output_dir / "airfoil_section_polars_metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    print(f"NeuralFoil points: {len(neuralfoil_rows)}")
    print(f"XFOIL returned points: {metadata['xfoil']['returned_points']}")
    print(f"Matched XFOIL comparisons: {metadata['xfoil']['matched_comparison_points']}")
    print(f"Output directory: {output_dir}")


if __name__ == "__main__":
    main()
