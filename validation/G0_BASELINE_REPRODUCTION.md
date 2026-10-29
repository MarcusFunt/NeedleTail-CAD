# G0 baseline and provenance record

**Status:** G0 passed for reproduction of the supplied regression baseline. This does not qualify a production propulsion model.

## Supplied source bundle

The exact user-supplied archive is `NeedleTail_Integration_Plan_and_Validation_Bundles_2026-10-29.zip`.

- Local archive: `C:\Users\marcu\Downloads\NeedleTail_Integration_Plan_and_Validation_Bundles_2026-10-29.zip`
- Size: 1,472,070 bytes
- SHA-256: `b11fcbcf33b3897e68fa4789361713aa0bc3b93228e8fce50f3dda117f1b5f40`
- Source roots: `NeedleTail_Isolated_Propeller_Validation_2026-10-27` and `NeedleTail_Frozen_BEMT_MultiProp_ForwardFlight_Validation_2026-10-28`
- Verified source members: 62; no nested archives
- Tracked manifest: [source_bundle_manifest.json](source_bundle_manifest.json)
- Local-only extraction: `validation/datasets/source_bundle/`

Recheck the original ZIP and the extracted bytes with:

```powershell
uv run --locked python -m validation.verify_source_bundle `
  --archive C:\Users\marcu\Downloads\NeedleTail_Integration_Plan_and_Validation_Bundles_2026-10-29.zip `
  --manifest validation\source_bundle_manifest.json `
  --extracted-root validation\datasets\source_bundle
```

The measurements and original generated reports remain outside Git because redistribution terms were not established. The tracked manifest records each original member’s path, byte length and SHA-256; it does not copy the measurement rows.

## Reproduction environment

- Base commit: `d6fcbcb3068e7a32d526970410d3a1810d9b8343`
- Working branch: `codex/propulsion-benchmark-foundation`
- OS: Windows 11, build 26200
- Python: 3.13.5
- uv: 0.11.21
- NumPy: 2.5.4
- Matplotlib: 3.11.2
- Git LFS: 3.4.1
- `uv.lock` SHA-256: `6a4437ed542435f92a363e35aa1a701d690a6228655aa4c25f40a901461b4d7e`
- Dependency check: `uv sync --locked --all-extras` completed; 82 locked packages resolved and 78 installed packages checked, with no lockfile change.

## Original supplied checks

Both supplied source roots were copied to disposable temporary directories before running their original scripts; the extracted archive was left unchanged. On Windows, set `PYTHONUTF8=1` before running the frozen report generation because the original `produce_outputs.py` writes a π character to a text file using the platform default encoding.

In the disposable frozen-suite copy, the original `benchmark.py` produced 195 rows with zero failures, the seven original tests passed, and `produce_outputs.py` generated its report and plots. Comparing its rerun `results/all_cases.csv` with the archived generated CSV gave the same 195 propeller/regime/RPM/J identities. Maximum absolute prediction deltas were `1.39e-17` for CT and `2.78e-17` for CP; the remaining prediction, load, error, and percentage-error deltas were floating-point roundoff (maximum absolute value `1.99e-13` percentage points). The generated report and raw files stayed in the temporary copy.

The original isolated 3×2 `reproduce.py` also ran in a disposable copy and retained the archived 16-point coefficient results (CT MAPE 3.01035%, CP MAPE 4.81294%); all four `test_physics.py` tests passed. The original archive outputs were not overwritten.

The archived frozen implementation’s test command passed all seven tests:

```powershell
uv run --project C:\Users\marcu\OneDrive\Dokumenter\GitHub\NeedleTail-CAD `
  --locked --extra aero python -m unittest -v test_benchmark.py
```

Run with the working directory set to `validation/datasets/source_bundle/NeedleTail_Frozen_BEMT_MultiProp_ForwardFlight_Validation_2026-10-28`.

For an isolated reproduction, run `python reproduce.py` and `python -m unittest -v test_physics.py` from the 2026-10-27 source root. Run `python benchmark.py`, `python -m unittest -v test_benchmark.py`, and `python produce_outputs.py` from the 2026-10-28 root. On Windows, set `PYTHONUTF8=1` for the latter sequence so report text is written as UTF-8.

## Frozen BEMT baseline

The exact archived implementation was run through the new backend-neutral runner against all 195 original observations: 103 static and 92 forward-flight records. Every source row, including all 12 nonpositive-thrust forward rows, was retained. There were zero solver failures.

| Regime | Rows | CT MAE | CT MAPE on positive thrust | CP MAE | CP MAPE on propulsive points |
|---|---:|---:|---:|---:|---:|
| Static | 103 | 0.00596868 | 6.1882% | 0.0186018 | 18.1921% |
| Forward | 92 | 0.0162412 | 59.7176% (80 rows) | 0.0129674 | 16.3083% (80 rows) |

The frozen forward model is poor at predicting thrust over this broad range. MAPE uses positive-measured-thrust rows only; coefficient MAE and all negative/nonpropulsive rows remain reported separately. `CP MAPE on propulsive points` requires both measured `CT > 0` and `CP > 0`.

The regenerated, local-only run and its raw per-condition file are under `validation/runs/legacy195_frozen_g0/`. It contains `cases.csv`, `metrics.json`, `run_manifest.json`, `REPORT.md`, and per-propeller SVG plots. Its manifest hashes the source bundle manifest and local implementation files.

## 2026-10-27 isolated GWS 3×2 record

The supplied historical `reproduce.py` and its committed-in-archive summary use the filename-inferred diameter **0.0784 m**. That assumption gives the historical coefficient comparison of CT MAPE 3.01035% and CP MAPE 4.81294% across 16 static points, but its dimensional thrust, torque and power are correspondingly low. The later source in the supplied 195-row archive uses the UIUC figure-confirmed **3.25 in = 0.08255 m** diameter. Coefficient MAPE is dimensionless and unchanged by that diameter correction; reconstructed dimensional loads are not.

The original archive files and result images were left byte-for-byte unchanged. Do not use the older 0.0784 m dimensional outputs as the corrected physical reference.

## Reproduction limits

- This is a frozen-reference regression set, not a blind final validation split. Its data had already been used for prior model exploration.
- Geometry is predominantly digitized `r/R`, `c/R`, and pitch/twist; exact section airfoils and experimental polars are not supplied for all props.
- Coefficient agreement does not establish accurate dimensional loads where true propeller diameter remains nominal or assumed.
- Passing G0 authorizes the separately scoped PR-B and PR-C evaluations only. It does not establish qprop.c superiority, motor feasibility, full-aircraft performance, or installed-flow validity.
