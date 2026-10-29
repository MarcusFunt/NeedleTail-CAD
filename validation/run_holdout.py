"""Run the checksum-pinned Graupner family holdout against isolated solvers."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from propulsion.benchmark import FrozenBEMTBackend, run_benchmark
from propulsion.qpropc import QPropCBackend
from validation.holdouts import HOLDOUT_MANIFEST, load_graupner_cam_9x4_holdout


def run_graupner_holdout(
    source_root: str | Path,
    frozen_source_root: str | Path,
    qpropc_portable: str | Path,
    polar_dir: str | Path,
    output_dir: str | Path,
) -> dict[str, object]:
    cases, metadata = load_graupner_cam_9x4_holdout(source_root)
    frozen = FrozenBEMTBackend(frozen_source_root)
    common = QPropCBackend(
        qpropc_portable,
        None,
        polar_mode="common_analytic",
        scenario_name="qpropc_common_analytic",
        polar_source_description="controlled common analytic polar matching the frozen BEMT CL/CD curves",
    )
    xfoil = QPropCBackend(
        qpropc_portable,
        polar_dir,
        polar_mode="xfoil",
        scenario_name="qpropc_naca4412_xfoil",
        polar_source_description="common NACA 4412 XFOIL Ncrit=6 set; section fidelity for Graupner is unverified",
    )
    backends = (frozen, common, xfoil)
    try:
        result = run_benchmark(
            cases,
            backends,
            output_dir,
            suite="graupner-cam-9x4-procedural-family-holdout",
            input_manifest=HOLDOUT_MANIFEST,
        )
    finally:
        common.close()
        xfoil.close()
    report = Path(output_dir) / "REPORT.md"
    with report.open("a", encoding="utf-8") as stream:
        stream.write(
            "\n## Holdout scope and limitations\n\n"
            f"- Family: {metadata['manufacturer_family']} ({metadata['propeller']}); "
            f"{len(cases)} UIUC wind-tunnel rows.\n"
            f"- Diameter: {metadata['diameter_basis']}\n"
            f"- Selection: {metadata['validation_role']}.\n"
            f"- Blinding: {metadata['blinding']}.\n"
            f"- Geometry: UIUC digitized radial geometry; source and measured rows are SHA-256 verified.\n"
            f"- Polar: common analytic case controls the section model; the XFOIL case uses a shared NACA 4412 "
            f"polar set whose suitability for this Graupner section is unverified.\n"
            f"- Redistribution: {metadata['redistribution_status']}\n"
            "- No backend is selected for production from this single family. Use results as a procedural "
            "family-level comparison and retain per-point failures, including windmill rows.\n"
        )
    result["holdout"] = {
        "id": "graupner_cam_9x4",
        "family": metadata["manufacturer_family"],
        "measurement_count": len(cases),
        "selection_status": metadata["validation_role"],
        "blinding": metadata["blinding"],
        "redistribution_status": metadata["redistribution_status"],
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True,
                        help="directory with the four verified Graupner files from holdout_sources.json")
    parser.add_argument("--frozen-source-root", type=Path, required=True,
                        help="extracted frozen-195 source root containing benchmark.py")
    parser.add_argument("--qpropc-portable", type=Path, required=True,
                        help="installed, checksum-verified qprop.c 2603 portable binding")
    parser.add_argument("--polar-dir", type=Path, required=True,
                        help="shared NACA4412 XFOIL Ncrit=6 polar collection")
    parser.add_argument("--out", type=Path, required=True,
                        help="new output directory; existing runs are never overwritten")
    args = parser.parse_args()
    result = run_graupner_holdout(
        args.source_root,
        args.frozen_source_root,
        args.qpropc_portable,
        args.polar_dir,
        args.out,
    )
    backend_summaries = {}
    for name, backend in result["metrics"]["backends"].items():
        backend_summaries[name] = {}
        for regime, metric in backend["by_regime"].items():
            per_propeller = backend["by_propeller"]["graupner_cam_9x4"][regime]
            backend_summaries[name][regime] = {
                "n": metric["evaluated"],
                "failures": metric["solver_failures"],
                "within_validity": metric["within_validity_range_count"],
                "outside_validity": metric["outside_validity_range_count"],
                "ct_mae": metric["ct_mae"],
                "ct_propulsive_mape_pct": metric["ct_propulsive_mape_pct"],
                "cp_mae": metric["cp_mae"],
                "cp_propulsive_mape_pct": metric["cp_propulsive_mape_pct"],
                "measured_zero_thrust_j": per_propeller["measured_zero_thrust_j"],
                "predicted_zero_thrust_j": per_propeller["predicted_zero_thrust_j"],
                "zero_thrust_crossings_by_rpm": per_propeller["zero_thrust_crossings_by_rpm"],
            }
    print(json.dumps({
        "suite": result["metrics"]["suite"],
        "holdout": result["holdout"],
        "backends": backend_summaries,
        "output": str(args.out.resolve()),
    }, indent=2))


if __name__ == "__main__":
    main()
