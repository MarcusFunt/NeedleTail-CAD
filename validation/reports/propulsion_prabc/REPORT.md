# Propulsion PR-A–C validation summary

**Reference branch base:** `d6fcbcb3068e7a32d526970410d3a1810d9b8343`

**QPropC:** release `2603`, commit `caacc6623a3b3596c9d59ff361ed647db40508a8`

**Scope:** frozen 195-row regression, QPropC upstream categories, and Graupner CAM 9×4 procedural family holdout.

Raw source tables and per-point case files are not included here. The supplied archive and UIUC source rights do not establish redistribution permission. They remain in local-only `validation/datasets/` and `validation/runs/`. This file records aggregate evidence, methods, and checksums without reproducing the raw rows.

## Results

| Run | Dataset | Backend | Sample | Main result | In declared range |
|---|---|---|---:|---|---:|
| `legacy195_prabc7` | 195-row regression, static | Frozen BEMT | 103 | CT MAE 0.005969 / MAPE 6.19%; CP MAE 0.018602 / MAPE 18.19% | 103/103 |
| `legacy195_prabc7` | 195-row regression, forward | Frozen BEMT | 92 | Full set CT MAE/MAPE 0.016241 / 59.72%; CP MAE/propulsive MAPE 0.012967 / 16.31%. In-range 52: CT MAE/MAPE 0.009602 / 15.19%; CP MAE/propulsive MAPE 0.006159 / 8.86%. | 52/92 |
| `legacy195_prabc7` | 195-row regression, static/forward | QPropC common analytic | 103 / 92 | Static CT/CP MAPE 7.13% / 7.72%; forward positive CT / propulsive CP MAPE 44.28% / 12.05% | 0/195 empirically valid; control polar only |
| `legacy195_prabc7` | 195-row regression, static/forward | QPropC XFOIL | 103 / 92 | Static CT/CP MAPE 22.30% / 25.91%; forward positive CT / propulsive CP MAPE 60.23% / 23.75% | 0/195 |
| `qpropc_upstream_prabc6` | APC 10×7SF | QPropC native geometry + XFOIL | 134 | Static CT/CP MAPE 1.66% / 7.18%; forward CT / CP MAPE 19.17% / 36.78% | 0/134 |
| `qpropc_upstream_prabc6` | APC 16×8E | QPropC native geometry + XFOIL | 52 | Static CT/CP MAPE 10.44% / 5.26%; full 39-row forward CT/CP MAPE 60.60% / 9.23%. In-range 33-row CT MAPE/MAE 69.15% / 0.005988; propulsive CP MAPE 9.58%. | 33/39 forward in range; 0/13 static |
| `qpropc_upstream_prabc6` | APC Free Flight 4.2×4 | QPropC native geometry + XFOIL | 54 | Static CT/CP MAPE 24.83% / 23.87%; forward CT / CP MAPE 31.40% / 36.57% | 0/54; known exception/out of scope |
| `graupner_cam9x4_prabc5` | CAM 9×4 holdout, static | Frozen BEMT | 16 | CT/CP MAPE 10.36% / 20.50% | 16/16 |
| `graupner_cam9x4_prabc5` | CAM 9×4 holdout, forward | Frozen BEMT | 27 | Full set CT/CP MAE 0.008694 / 0.008836; propulsive MAPE 100.39% / 40.04%. In-range 18: CT MAE/MAPE 0.005312 / 23.00%; CP MAE/propulsive MAPE 0.006365 / 30.87%. | 18/27 |
| `graupner_cam9x4_prabc5` | CAM 9×4 holdout, static/forward | QPropC common analytic | 16 / 27 | CT MAPE 9.57% / 74.18%; CP propulsive MAPE 13.62% / 36.45% | 0/43 empirically valid; control polar only |
| `graupner_cam9x4_prabc5` | CAM 9×4 holdout, static/forward | QPropC shared NACA 4412 XFOIL | 16 / 27 | CT MAPE 5.54% / 47.36%; CP propulsive MAPE 5.19% / 4.37% | 0/43 |

No QPropC solver failures occurred in the APC or Graupner runs. Reynolds uses `rho * W * chord / mu`. For APC 16×8E, 33 of 39 forward points are inside polar range; full-set CT MAPE is 60.60%, while the valid subset has CT MAPE 69.15%, CT MAE 0.005988, and propulsive CP MAPE 9.58%. The common analytic case is a controlled solver comparison, not empirical coverage. Graupner XFOIL errors are promising but cannot be treated as validated because section identity is unknown and no holdout point is in range. The holdout is procedural, not blind; its values were public during source discovery. Curves and zero-thrust crossings are split by RPM.

The upstream Graupner CAM 6×3 category compares QPropC to reconstructed original QPROP v1.22 radial outputs, not wind-tunnel measurements. Its two conditions converged with maximum residual below `5.95e-7`; thrust differences were -0.05361 N in hover and -0.03925 N at 5 m/s.

The frozen implementation produced no failures on the 195-row panel and retains all 12 nonpositive-thrust forward rows. Its generic adapter matched the archived implementation over all 195 inputs to maximum absolute CT/CP difference `8.33e-17`. APC 4.2×4 shows the upstream-known static discrepancy and remains out of scope for selection.

## Decision

**No production backend selected.** Keep Frozen BEMT as a regression reference and QPropC as an experimental option. Neither has adequate in-range high-advance-ratio and independent-family evidence to gate optimizer or motor use. See the [selection ADR](../../../docs/adr/propulsion_backend_selection.md) for the weighted rubric and hard-gate assessment.

## Reproduction and provenance

- Verify the local source bundle using the command in [G0_BASELINE_REPRODUCTION.md](../../G0_BASELINE_REPRODUCTION.md).
- Reproduce the 195-row comparison using the command in [propulsion/README.md](../../../propulsion/README.md); final output path `validation/runs/legacy195_prabc7`.
- Reproduce QPropC upstream categories with `python -m validation.qpropc.validate_upstream`; final output path `validation/runs/qpropc_upstream_prabc6`.
- Reproduce the family holdout with `python -m validation.run_holdout`; final output path `validation/runs/graupner_cam9x4_prabc5`.
- Original archive scripts and reports were rerun in disposable copies; see [G0_BASELINE_REPRODUCTION.md](../../G0_BASELINE_REPRODUCTION.md) for row identity and floating-point tolerance results.
- The frozen archive SHA and member hashes are tracked in [source_bundle_manifest.json](../../source_bundle_manifest.json). Holdout file hashes and caveats are tracked in [holdout_sources.json](../../holdout_sources.json). QPropC release/source pin is tracked in [qpropc_lock.json](../../qpropc/qpropc_lock.json).
- Baseline environment and original seven-test reproduction are documented in [G0_BASELINE_REPRODUCTION.md](../../G0_BASELINE_REPRODUCTION.md).

Generated metrics include local implementation hashes, machine/runtime details, effective QPropC configuration, separate case/backend input hashes, convergence, validity status, RPM-specific zero-thrust crossings, and raw per-point records. Generated records remain local because of data redistribution uncertainty.
