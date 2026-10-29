# ADR: isolated-propeller backend selection

**Status:** No winner; keep all backends in evaluation only

**Scope:** PR-A through PR-C; isolated propeller predictions only

**Baseline:** `d6fcbcb3068e7a32d526970410d3a1810d9b8343`

**Decision date:** 2026-10-29

## Context

NeedleTail needs traceable thrust, torque, and shaft-power maps before a motor model or optimizer can consume them. The 195-point archive is a development/regression panel, not a blind test set. The separate Graupner CAM 9×4 family set is a procedural holdout: its public values were visible during source discovery, and it must not be used for fitting. The NACA 4412 XFOIL polar used for that Graupner comparison is not verified for the Graupner section.

The candidate rubric from the integration plan weights accuracy and held-out coverage 45%, motor-relevant high-advance-ratio fidelity 20%, numerical robustness 15%, integration/runtime 10%, and reproducibility/license 10%. No aggregate score is assigned because a hard validity gate failed for each solver and the candidates were not evaluated under sufficiently supported conditions.

## Evidence

| Dataset and backend | Conditions | Result | Validity and interpretation |
|---|---:|---|---|
| Frozen BEMT, legacy regression, static | 103 | CT MAE 0.005969, MAPE 6.19%; CP MAE 0.018602, MAPE 18.19% | 103/103 within its declared baseline range; regression reference only |
| Frozen BEMT, legacy regression, forward | 92 | Full set: CT MAE 0.016241, positive-thrust MAPE 59.72%; CP MAE 0.012967, propulsive MAPE 16.31%. In-range 52: CT MAE/MAPE 0.009602 / 15.19%; CP MAE/propulsive MAPE 0.006159 / 8.86%. | 52/92 within range; all 12 nonpositive-thrust measurements retained |
| QPropC common analytic, legacy regression, static | 103 | CT MAPE 7.13%; CP MAPE 7.72% | 0/103 empirically valid; control polar only |
| QPropC common analytic, legacy regression, forward | 92 | positive-thrust CT MAPE 44.28%; propulsive CP MAPE 12.05% | 0/92 empirically valid; control polar only |
| QPropC XFOIL, legacy regression, static | 103 | CT MAPE 22.30%; CP MAPE 25.91% | 0/103 within polar/geometry validity |
| QPropC XFOIL, legacy regression, forward | 92 | positive-thrust CT MAPE 60.23%; propulsive CP MAPE 23.75% | 0/92 within validity |
| QPropC XFOIL, APC 10×7SF upstream rows | 134 | Static CT/CP MAPE 1.66% / 7.18%; forward CT/CP MAPE 19.17% / 36.78% | 0/134 within Re/alpha range |
| QPropC XFOIL, APC 16×8E upstream rows | 52 | Static CT/CP MAPE 10.44% / 5.26%; full 39-row forward CT/CP MAPE 60.60% / 9.23%. In-range 33-row CT MAPE/MAE 69.15% / 0.005988; propulsive CP MAPE 9.58%. | 33/39 forward in range; 0/13 static |
| QPropC XFOIL, APC Free Flight 4.2×4 upstream rows | 54 | Static CT/CP MAPE 24.83% / 23.87%; forward CT/CP MAPE 31.40% / 36.57% | 0/54 in range; known static exception/out of scope |
| Frozen BEMT, Graupner 9×4 holdout, static | 16 | CT MAPE 10.36%; CP MAPE 20.50% | 16/16 within declared baseline range; digitized geometry |
| Frozen BEMT, Graupner 9×4 holdout, forward | 27 | Full set: CT MAE 0.008694, propulsive MAPE 100.39%; CP MAE 0.008836, propulsive MAPE 40.04%. In-range 18: CT MAE/MAPE 0.005312 / 23.00%; CP MAE/propulsive MAPE 0.006365 / 30.87%. | 18/27 within range; 5 negative-thrust rows preserved |
| QPropC common analytic, Graupner 9×4 holdout, static/forward | 16 / 27 | CT MAPE 9.57% / 74.18%; CP MAPE 13.62% / 36.45% | 0/43 within polar/geometry validity |
| QPropC shared NACA 4412 XFOIL, Graupner 9×4 holdout, static/forward | 16 / 27 | CT MAPE 5.54% / 47.36%; CP MAPE 5.19% / 4.37% | 0/43 within validity; section match is unverified |

All reported predictions converged numerically in these runs. The frozen legacy implementation also passed its seven archived tests. The generic frozen adapter reproduced all 195 archived inputs against the original implementation with maximum absolute CT/CP difference `8.33e-17`. The original archive scripts were also rerun in disposable copies; all 195 stored condition identities matched, with maximum CT and CP prediction deltas `1.39e-17` and `2.78e-17`. QPropC was evaluated from the pinned MIT release `2603` at commit `caacc6623a3b3596c9d59ff361ed647db40508a8`. Correcting the Reynolds calculation places 33 APC 16×8E forward points inside polar coverage. The all-39-row CT MAPE is 60.60%; within the valid 33-row subset, CT MAPE is 69.15% (MAE 0.005988) and propulsive CP MAPE is 9.58%. APC 10×7SF and APC 4.2×4 have no in-range points. APC 4.2×4 remains a known exception and is excluded from eligibility claims.

The upstream Graupner CAM 6×3 comparison uses a reconstruction of original QPROP v1.22 radial outputs, not measured data. Its two predictions converged with maximum residual below `5.95e-7`; thrust differences from the reconstructed outputs were -0.05361 N in hover and -0.03925 N at 5 m/s. This is an implementation comparison only.

The Graupner XFOIL case has lower aggregate errors than the other holdout cases, but every point is outside the adapter's declared validity range and the airfoil identity is unverified. Its zero-thrust crossings are reported separately by RPM: predicted-minus-measured ΔJ is 0.00642 at 4015 RPM and 0.01586 at 5009 RPM. Frozen BEMT gives -0.05767 and -0.06430 at those RPMs. The XFOIL result therefore cannot support a solver selection. The common-polar and frozen comparisons likewise do not meet the plan's accuracy and coverage gates across the intended high-advance-ratio region.

## Decision by weighted criterion

| Criterion | Weight | Assessment |
|---|---:|---|
| Accuracy and independent coverage | 45% | No candidate passes. Frozen BEMT is a compatible regression reference; its high-advance-ratio thrust error is large. QPropC's APC 16×8E forward CT MAPE is 60.60% across all 39 rows and 69.15% within the valid 33-row subset; the Graupner holdout remains out of geometry/polar range. |
| Motor-relevant high-J fidelity | 20% | No production-valid high-J range is established. Motor equilibrium is not in PR-A–C. |
| Numerical robustness | 15% | All measured rows converged in these runs, but numerical convergence does not repair geometry or polar validity gaps. |
| Integration and runtime | 10% | Frozen BEMT is already local; QPropC 2603 ran through its verified Windows portable binding. This does not establish a supported production deployment matrix. |
| Reproducibility and license | 10% | The QPropC commit, release, archive and installed file hashes are pinned. Source datasets without established redistribution rights remain local. |

PropellerToolbox was not integrated or run. Its CC BY-NC-SA terms require a separate distribution/license decision; it is not added to the project dependency set. PR-C therefore compares Frozen BEMT and QPropC only. BladeAD, PyThrust, OpenVSP, and other optional research tools are outside PR-A–C and were not installed as project dependencies.

## Consequences

- Keep frozen BEMT as a regression baseline and QPropC as an experimental adapter. Do not feed either backend into optimizer ranking or claim a production winner.
- Keep polar range, geometry provenance, nonpropulsive regimes, and failure state attached to each prediction. Do not turn converged-but-out-of-range predictions into valid results.
- Acquire or establish section geometry/polar coverage and additional untouched propeller-family holdouts before a new selection decision. Preserve the current Graupner results as subsequent development evidence, not a blind score.
- Keep motor/electrical feasibility and installed-flow corrections as later phases. This ADR does not validate attainable RPM, topology ranking, or airframe performance.

## Reproduction

Final generated runs are local-only under `validation/runs/` because they contain transcribed measurement rows (`legacy195_prabc7`, `qpropc_upstream_prabc6`, and `graupner_cam9x4_prabc5`). The compact summary is in [validation/reports/propulsion_prabc/REPORT.md](../../validation/reports/propulsion_prabc/REPORT.md). Commands and source hashes are recorded in [G0_BASELINE_REPRODUCTION.md](../../validation/G0_BASELINE_REPRODUCTION.md), [source_bundle_manifest.json](../../validation/source_bundle_manifest.json), and [holdout_sources.json](../../validation/holdout_sources.json).
