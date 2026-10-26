# NeedleTail first-pass airframe CAD

## Coordinate system

- Units: millimeters.
- +X points from the aircraft center toward the nose; freestream enters from
  +X and travels toward -X.
- +Y is the right wing and +Z is up in the aircraft frame.
- The origin is the cruciform intersection on the fuselage centerline.
- UIUC airfoil coordinates are transformed so the airfoil nose points toward
  +X, independent of the profile library's proposed alternate convention.

## Profile-based geometry

`airframe_geometry.py` is the shared parametric source. `AirframeParameters`
contains the dimensions and body, pod, and finite trailing-edge profiles. The
horizontal wing uses the UIUC SD7003 coordinate profile; the vertical support
uses UIUC SD8020. The original source files remain unchanged in
`profiles/original/`, and `profiles/airfoil_coordinates.py` records their source
URLs and SHA-256 hashes.

The STEP derivative blends the last 6% of each airfoil chord to a 0.30 mm
physical trailing edge while retaining the imported profile shape forward of
that transition. The fuselage is a revolved, packaging-constrained radius
profile with its maximum radius at 38% of body length aft of the nose. Four
42 mm by 24 mm motor pods use a revolved profile with a rounded +X forebody and
an adjustable aft taper. The forebody radius now carries the XING2 1404 motor
case envelope to the motor-bell nose, with a 1.5 mm radial allowance around
the approximate motor profile before tapering to the propeller hub. This is
outer-envelope allowance; it does not model pod wall thickness or an internal
cavity. The compact layout uses a 178 mm horizontal wing and a 148 mm vertical
support, reduced from 260 mm and 190 mm. Both arm tips stop at the pods' inner
radial envelope with a 1 mm overlap for a robust fused OML: the tip stations are
`(0, ±89, 0)` and `(0, 0, ±74)`, while pod centers remain at
`(0, ±100, 0)` and `(0, 0, ±85)`. The arms therefore do not continue through the
pods to their centerlines or outboard sides. Span is derived from motor
stations, pod diameter, and overlap, so changing a motor offset also updates
the matching arm tip. The lifting surfaces use ruled lofts for a linear
spanwise taper. Their gross planform areas are 8,455 mm² for the horizontal
wing and 5,624 mm² for the vertical support; these exclude fuselage overlap and
are not aerodynamic effective-area estimates. Relative to 260 mm and 190 mm
baseline spans, the changes reduce planform area by 31.5% and 22.1%.

## Preliminary airfoil geometry candidate

The merged PR #3 geometry remains the default (`PARAMETERS`). The revised
airfoil-driven layout is available separately as
`AIRFOIL_CANDIDATE_PARAMETERS`; its supplied dimensions are intentionally kept
as a preliminary candidate and have not been tuned. It uses a 230 × 60 mm
NACA 0015-derived fuselage, 80/56 mm NACA 2415 wing chords, a 64 mm NACA 0015
support, 95/80 mm motor offsets, 124 × 19.9 mm NACA 0015 pods, and the shared
−5 mm assembly offset with a +1 mm local pod station. CAD and aerodynamic
builders accept an explicit `AirframeParameters` value, so callers can use
`build_components(AIRFOIL_CANDIDATE_PARAMETERS)` or
`whole_aircraft_aero.build_airplane(AIRFOIL_CANDIDATE_PARAMETERS)` without
changing the baseline. To export candidate STEP files alongside the baseline
exports, run `uv run --locked python .\airframe_candidate.py --force`; this
creates `airframe_candidate.step` and `airframe_candidate_oml.step`.

The optimizer and airfoil-polar tool offer named selection. For example:

```powershell
uv run --locked --extra aero python design_optimizer.py --geometry airfoil_candidate
uv run --locked --extra aero python airfoil_polars.py --geometry airfoil_candidate --output-dir .\tmp\airfoil-candidate-polars
uv run --locked --extra aero python compare_airframe_aero.py --geometry airfoil_candidate
```

Optimizer rows record `geometry_configuration` alongside the existing mass
scenario. The comparison scripts and the five files in `design_notes/` retain
the source branch's geometry studies and assumptions. They are exploratory
records: their panel/VLM or AeroBuildup estimates are unpowered and do not
resolve transition, propeller slipstream, structural margins, or the eventual
center of gravity. They do not promote the candidate or select a topology.

The 76.2 mm swept-disk envelope is tied to the shared airframe parameters and
checked against pod noses, both lifting surfaces, the fuselage, and adjacent
disks. Current modeled clearances are 1.5 mm to the pod noses, 8.5 mm to the
horizontal wing, and 3.5 mm to the vertical support; adjacent disks have
55.0 mm edge clearance. The minimum permitted modeled clearance is 1.0 mm.

## Motor and propeller interface

The reference motor mount face is at `X=+4.5 mm` from each pod center. The
motor body extends 13.5 mm forward from that plane. The propeller hub center
is 18.5 mm forward of the mount face, at `X=+23.0 mm`; the 4 mm reference hub
therefore starts at `X=+21.0 mm`, flush with the pod nose. The motor CAD's
modeled shaft tip is at `X=+22.9 mm`, giving 1.9 mm of nominal shaft entry
into the hub. The shaft extension remains an unverified packaging assumption.

`propeller_envelope.step` now places its 1 mm reference disks at the hub
centerline, 2 mm ahead of the pod nose. `motor_prop_installation.py` keeps the
motor and propeller as separate solids in a `Compound`, and rotates their local
+Z shaft axis onto aircraft +X. This preserves the hub bore and shaft
clearance for inspection. The pod-fit validator samples the same spline used
to generate the pod, and the profile coordinates are normalized at their
interpolated leading edge instead of being clipped. The selected reference
STEP files and their limits are documented in `references/propeller-kit/README.md`.

Elevons, propeller blades, internal structure, openings, and manufacturing
features are excluded. Pod X station and the control-surface design remain
provisional; this is aerodynamic concept geometry, not a flight-ready or
manufacturing definition.

## STEP outputs

- `airframe_oml.step` is the single-solid, watertight Boolean-fused outer mold
  line intended for CFD import.
- `airframe.step` is the labeled engineering assembly compound with the
  fuselage, lifting surfaces, and four motor pods retained as separate parts.
- `propeller_envelope.step` is separate construction geometry containing four
  76.2 mm diameter, 1 mm thick disks, centered on the propeller hubs.
- `motor_prop_installation.step` is a one-motor, one-propeller packaging
  reference assembly, with the components kept separate for interface review.

## Build and inspect

The repository pins Python 3.13.5 in `.python-version`, uv 0.11.21 in CI,
build123d 0.12.0, cadgen 0.7.26, and the full transitive Python dependency
set in `uv.lock`. Gmsh 4.15.2 is also locked as the optional `cfd-meshing`
extra for the CFD phase. Install uv 0.11.21, then from this directory run:

```powershell
uv sync --locked
uv run --locked python .\airframe_oml.py --force
uv run --locked python .\airframe.py --force
uv run --locked python .\propeller_envelope.py --force
uv run --locked python .\motor_prop_installation.py --force
uv run --locked python -m unittest discover --verbose
```

GitHub Actions runs these four builds, the geometry regression tests, and
STEP re-import and solid-validity checks on pushes and pull requests. The
build123d 0.12.0 target currently requires an explicit uv override because
cadgen 0.7.26's package metadata caps build123d below 0.12; the locked CI
environment exercises that combination directly.

## Airfoil polars

Install AeroSandbox and NeuralFoil into the project environment with:

```powershell
uv sync --locked --extra aero
```

XFOIL is a separate executable. Download the Windows build from the
[official XFOIL page](https://web.mit.edu/drela/Public/web/xfoil/) and place
`xfoil.exe` at `.venv\Scripts\xfoil.exe`. Generate geometry-matched SD7003
and SD8020 section polars and compare selected points with XFOIL using:

```powershell
uv run --locked --extra aero python .\airfoil_polars.py --xfoil .\.venv\Scripts\xfoil.exe
```

Results are written under `tmp\airfoil-polars\`. Set flight-speed and altitude
conditions before using the exploratory Reynolds-number sweep as an operating
envelope.

## Whole-aircraft aerodynamic trends

Build an AeroSandbox model from the shared airframe parameters and compare
attached-flow VLM trends with empirical AeroBuildup trends:

```powershell
uv run --locked --extra aero python .\whole_aircraft_aero.py
```

The model includes the CAD-matched SD7003 wing, SD8020 vertical support, the
central fuselage, and four motor pods. The airfoil sections preserve the
project's 0.30 mm trailing-edge closure, and the fuselage and pod bodies follow
the CAD profile splines. It uses sea-level conditions at 10, 15, and 20 m/s to
show the sensitivity of AeroBuildup to the low section Reynolds numbers. The
horizontal wing sets the coefficient reference area; moments are referenced to
the geometry origin because mass properties and center of gravity are not yet
defined.

Outputs are written under `tmp\whole-aircraft-aero\`: a reloadable `.asb`
aircraft object, coefficient sweeps, model/assumption metadata, a trend plot,
and both three-view and shaded 3D geometry renders. VLM is inviscid and assumes
attached flow. AeroBuildup uses empirical component models and NeuralFoil
section data. Treat both as conceptual estimates at this scale, where
transition, surface finish, propeller effects, and the actual center of gravity
can matter substantially. Propeller blades, thrust, and powered propwash are
not included.

## CAD-clearance validation and controlled baseline comparison

All propeller-envelope clearances are now measured as true 3D minimum distances
between build123d/OCCT solids. The report measures the 1 mm-thick swept-disk
**envelope** against wing, vertical support, fuselage, and own motor pod solids,
plus the other swept disks. Zero distance means contact or intersection.
A nominal 1 mm clearance remains a geometry-only criterion; rotor blade flex,
mount tolerance, bearing play and vibration require a larger engineering margin.
The report is cached per immutable AirframeParameters instance.

The `aero` CI job imports AeroSandbox and NeuralFoil, tests the shared section
contours, component dimensions, 3D coordinate mapping, physical clearance
invariants and finite low-Re aerodynamic outputs. It also runs:

```powershell
uv run --locked --extra aero python .\compare_airframe_variants.py
```

This writes `tmp/airframe-comparison/variant_comparison.csv`, metadata and
force/drag comparison plots. The historic *24 Oct pre-change* proxy restores
**240 mm horizontal** and **170 mm vertical** lifting spans, with the same
motor stations, pod/fuselage profiles, airfoil definitions and analytic model
as the compact 178/148 mm variant. It is a **controlled aerodynamic surrogate
comparison, not byte-identical execution of the former build123d STEP file**:
the former CAD used a smooth loft and historical normalization of SD7003.
Reported coefficients use each wing's reference area, so cross-variant
comparisons should emphasize dimensional lift and drag (N), and compute flight
trim/power once mass and propeller data are available. The first-pass models
are unpowered and exclude propeller slipstream and junction interference.
Results require eventual verification against mesh-converged, transitional
viscous CFD (e.g. SU2) on each revision's exact STEP surface.

The GitHub Actions CAD job independently regenerates all four STEP files under
the locked `build123d==0.12.0` / `cadgen==0.7.26` override environment,
then tests their solid counts and reimport validity. Until a successful run,
the metadata override must be regarded as provisionally supported.

## Exact previous-versus-compact native STEP comparison

CI additionally checks out historical commit `3ea08b2` into a detached worktree,
rebuilds its unfused assembly-independent OML STEP using **the same current
locked Python/build123d/cadgen environment**, and compares that imported solid
against current `airframe_oml.step`. Run locally with the two STEP paths:

```powershell
uv run --locked python .\compare_step_geometries.py --old-step .\old_airframe_oml.step --new-step .\airframe_oml.step
```

The job uploads both validated OML STEP files, surface/volume and envelope
statistics, and the numeric comparison report. Unlike the AeroSandbox comparison
these are **actual old and new BREP solids**; the comparison is purely geometric,
not a higher-fidelity drag calculation. Meaningful viscous-CFD evaluation still
requires matched Gmsh meshing, consistent reference quantities and farfield,
grid-convergence sweeps, and appropriate low-Reynolds transitional-flow
validation. Do not infer improved cruise performance from volume or surface
area alone.


## Experimental unified propulsion and control-surface design study

A new **opt-in** design space now includes `tractor4`, `pusher4`, `coaxial8` crossed with no flaps, 2 horizontal flaps, or all 4 flaps. All variants use the existing `AirframeParameters`. The original CAD/STEP build remains `tractor4 + none` by default.

Start with [DESIGN_OPTIMIZATION.md](DESIGN_OPTIMIZATION.md), then:

```powershell
uv run --locked --extra aero python -m unittest -v test_design_optimizer
uv run --locked --extra aero python design_optimizer.py --settings design_optimizer_settings.json
uv run --locked --extra aero python design_optimizer.py --check-best-cad
```

The exploratory optimizer evaluates constrained control-moment allocation, actuator mass and incremental flap drag. **Its score is not an attainable-speed result.** New rear-motor packages and neutral-surface assembly STEP previews require explicit OCCT validation; they are not manufacturing-ready. The powered propeller/wake and full-moment trim models remain separate validation milestones.
