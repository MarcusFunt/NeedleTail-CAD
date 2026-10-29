# Propulsion contracts and validation tools

This package adds a replaceable, provenance-carrying isolated-propeller layer. It does not change optimizer ranking or replace the existing topology CFD workflow.

## Contracts and sign conventions

All lengths, forces, moments, speeds, densities, and viscosities use SI units. Public operating speed is RPM; the solver conversion uses `n = rpm / 60` and `omega = 2*pi*n`.

- `J = V / (n D)`
- `T = CT * rho * n^2 * D^4`
- `Pshaft = CP * rho * n^3 * D^5`
- `Q = Pshaft / (2*pi*n)`

Positive thrust follows the commanded thrust axis. Positive torque and shaft power are the load required to drive the propeller. Negative thrust, torque, and power remain signed. Efficiency is undefined for static operation, windmilling, nonpositive thrust, or nonpositive shaft power. Coefficient normalization at zero RPM is explicitly undefined.

`PropellerGeometry` requires station geometry, source SHA-256, provenance, blade count, and source coordinate conventions. `GeometryProvenance.PACKAGING_ONLY` is rejected by predictive models. `MeasurementRecord` preserves each source row, its one-based source line, measurement provenance, original URL/path, and file hash. `PropellerResult` carries signed loads, convergence evidence, model scope, validity status, warnings, and input hash.

## Install the project environments

The core CAD environment remains unchanged:

```powershell
uv sync --locked
```

Install every dependency and optional extra declared by this project:

```powershell
uv sync --locked --all-extras
```

The optional extras are `aero` (AeroSandbox and NeuralFoil) and `cfd-meshing` (Gmsh). The qprop.c portable binding is a separate checksum-pinned MIT release and is not a Python package dependency or a change to `pyproject.toml`/`uv.lock`.

Install qprop.c to a new local directory (the installer refuses to overwrite it):

```powershell
uv run --locked python validation/qpropc/install_qpropc.py `
  --dest "$env:TEMP\NeedleTail-qpropc-installed-2603"
```

The installer verifies release 2603, source commit `caacc6623a3b3596c9d59ff361ed647db40508a8`, the MIT license and the portable archive SHA-256 from [qpropc_lock.json](../validation/qpropc/qpropc_lock.json). The adapter rechecks every installed file hash before loading the native library. Tests do not download or load a native binary.

## Frozen 195-row comparison

The source measurements are provided separately by the user and extracted locally under `validation/datasets/source_bundle/`. Verify the archive and extracted files first; see [G0_BASELINE_REPRODUCTION.md](../validation/G0_BASELINE_REPRODUCTION.md). The extracted measurements and generated raw output are local-only because data redistribution rights were not established.

The runner preserves all 195 original conditions, including negative-thrust rows and failures. It writes per-condition CSV, metrics JSON, a Markdown summary, SVG curves, and a run manifest with source-bundle and local implementation hashes. Existing output directories are never overwritten.

```powershell
uv run --locked --extra aero python -m propulsion.benchmark `
  --suite legacy195 `
  --backend frozen qpropc `
  --source-root validation/datasets/source_bundle/NeedleTail_Frozen_BEMT_MultiProp_ForwardFlight_Validation_2026-10-28 `
  --source-manifest validation/source_bundle_manifest.json `
  --qpropc-portable "$env:TEMP\NeedleTail-qpropc-installed-2603" `
  --polar-mode common_analytic xfoil `
  --polar-dir "$env:TEMP\NeedleTail-qprop-c-2603-source\webgui\airfoil_polars\naca4412_Ncrit=6" `
  --polar-description "Common NACA 4412 XFOIL Ncrit=6 polars from qprop.c release 2603" `
  --out validation/runs/legacy195-comparison
```

The `frozen` backend calls the original archived implementation for its six original propellers. For a new two-blade geometry it applies the archived lift, drag, tip-loss, and induction settings through a generic geometry adapter. On the six original propellers, that generic adapter was checked against the original implementation across all 195 rows; the maximum absolute CT/CP difference was `8.33e-17`.

The qprop.c adapter provides controlled `common_analytic` polar runs and XFOIL polar runs. The common analytic polar is a solver-control case matched to frozen BEMT; it never counts as empirical validity coverage. XFOIL validity requires the local radial operating Reynolds numbers (`rho * W * chord / mu`) and angles of attack to be bracketed by the supplied polar files. The adapter marks digitized geometry, nonpropulsive conditions, known exception cases, and polar-range extrapolation out of validity. A converged numerical solution by itself is not evidence that the input is inside the validated physical scope.

Per-point `case_input_sha256` identifies the measured row, geometry, RPM, and advance ratio. `input_sha256` identifies the effective backend inputs; for QPropC it includes the pinned release files, polar hash and mode, density, viscosity, speed of sound, and the native APC geometry hash when used. The run manifest records the corresponding backend configuration. Forward SVG prediction curves and zero-thrust crossings are kept separate by RPM.

## qprop.c upstream reproduction

Clone qprop.c at the locked source commit and install the portable release as shown above. The validation command reproduces its APC 10×7SF, APC 16×8E, APC 4.2×4 UIUC wind-tunnel categories and its two Graupner 6×3 comparisons to original QPROP v1.22 radial-output reconstructions:

```powershell
git clone https://github.com/pavanandrea/qprop.c "$env:TEMP\NeedleTail-qprop-c-2603-source"
git -C "$env:TEMP\NeedleTail-qprop-c-2603-source" checkout --detach caacc6623a3b3596c9d59ff361ed647db40508a8
uv run --locked --extra aero python -m validation.qpropc.validate_upstream `
  --upstream-root "$env:TEMP\NeedleTail-qprop-c-2603-source" `
  --portable-dir "$env:TEMP\NeedleTail-qpropc-installed-2603" `
  --out validation/runs/qpropc-upstream
```

The output labels the Graupner comparisons as reconstructions against QPROP v1.22, not experiments. APC 4.2×4 is excluded from qprop.c validity claims because upstream reports excessive static discrepancy. Negative-thrust source rows are retained. The source’s known missing Windows polar symlinks are handled by locating the corresponding checked-in upstream polar directory.

## Independent family holdout

The Graupner CAM 9×4 holdout is checksum-pinned by [holdout_sources.json](../validation/holdout_sources.json). Download the four original files from the URLs there into `validation/datasets/holdouts/graupner_9x4/`; the loader rejects any size or SHA mismatch. Keep the downloaded rows and generated per-point output local. The source page does not establish an explicit data redistribution license.

```powershell
uv run --locked --extra aero python -m validation.run_holdout `
  --source-root validation/datasets/holdouts/graupner_9x4 `
  --frozen-source-root validation/datasets/source_bundle/NeedleTail_Frozen_BEMT_MultiProp_ForwardFlight_Validation_2026-10-28 `
  --qpropc-portable "$env:TEMP\NeedleTail-qpropc-installed-2603" `
  --polar-dir "$env:TEMP\NeedleTail-qprop-c-2603-source\webgui\airfoil_polars\naca4412_Ncrit=6" `
  --out validation/runs/graupner-cam-9x4
```

This is a procedural family-level holdout, not a blinded experiment: public values were visible during source discovery, and they must not be used for coefficient fitting. The shared NACA 4412 XFOIL polar is not verified for Graupner’s section airfoil. See the generated `REPORT.md` for validity counts and limits.

## Scope boundary

The comparison does not determine motor electrical equilibrium or safe RPM, installed pusher/tractor/coaxial interference, or a maximum-speed configuration. PyThrust/motor integration and installed-flow evaluation are separate plan phases. The supplied propeller-kit STEP remains clearance-only. The existing topology CFD campaign was not modified here and is not considered physically qualified by passing these propulsion tests.
