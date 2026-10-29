# SU2 external-flow setup

## Topology campaign status

`topology_cfd_campaign.py` is an experimental powered Euler screen. It uses
the shared rotor station, shaft-axis, thrust-direction, hand and spin contract, requires an explicit coaxial separation,
checks CAD clearance, rejects fixed airframe or motor geometry inside a rotating
zone, rejects overlapping independent rotor zones, isolates propeller-only wall loads, and validates mesh orientation,
quality, connectivity, interfaces, and history acceptance. Numerical completion
does not establish design readiness; mesh and time-step sensitivity are still
required. See [`docs/cfd_topology_validation.md`](../../docs/cfd_topology_validation.md)
for the gates and the opt-in bounded smoke command.

The current baseline's geometry-only probe finds that rotor 1's rotating zone
intersects both the airframe OML and motor case, so the campaign correctly stops
before meshing. Revise the geometry or sliding-zone layout before running SU2.
`--interference-diagnostic` records both swept-disk CAD clearance and geometry-only
Gmsh checks for fixed-surface intersections. It creates no mesh, does not start
SU2, and never enables rankings.

This is the original end-to-end SU2 check against either `airframe_oml.step` or the
separate `airframe_candidate_oml.step`. The PR #3 baseline remains the default;
pass `-Geometry airfoil_candidate` to either PowerShell runner to select the
candidate. It creates an
external fluid box around the OML, scales the imported CAD from millimeters to
meters, writes a native `.su2` mesh with `AIRFRAME` and `FARFIELD` markers, and
runs a steady incompressible Euler case at 15 m/s and sea-level density. The
freestream travels along -X, matching the project's cruise coordinate convention.
The mesh generator removes exact duplicate triangles on imported pod skins,
then verifies the exported boundary markers against the tetrahedral boundary.
It stops if any exterior tetrahedron face is missing a marker.

## Install and run

SU2 8.5.0's Windows OpenMP binary is installed under
`%LOCALAPPDATA%\Programs\SU2\8.5.0\bin`. The user-level `SU2_RUN` and `PATH`
environment variables point to that folder. The OpenMP package runs without an
MPI installation.

From the repository root, install the project's declared meshing extra and run
the case:

```powershell
uv sync --locked --extra aero --extra cfd-meshing
.\cfd\su2\run_case.ps1
```

Mesh, config copy, solver history, restart, and flow outputs are written under
`tmp\su2-15mps\` for the baseline and `tmp\su2-15mps-airfoil_candidate\` for
the candidate. Existing mesh outputs are not overwritten.

If its OML STEP is not present, generate the baseline with
`uv run --locked python airframe_oml.py` or the candidate with
`uv run --locked python airframe_candidate.py` before meshing.

## First run status

On 2026-10-24, mesh generation produced 29,294 nodes and 158,374 tetrahedra.
SU2 completed its 300-iteration run, but the pressure residual did not meet the
convergence criterion. A separate 1,000-iteration run at CFL 0.2 also reached
its iteration limit without converging. Treat both as solver/setup checks; do
not use their force coefficients for design decisions.

## Blade-resolved rotating-propeller case

`run_rotor_case.ps1` builds one stationary outer-flow zone and four blade-resolved
sliding rotor zones from the selected geometry's `propeller_mount_stations` and the
CW/CCW propeller STEP files. SU2 moves each rotor mesh at the requested RPM. The
stationary zone uses the detailed motor STEP; moving zones contain propeller
surfaces only. The current strict clearance preflight can reject the supplied
airframe/motor arrangement before meshing; do not bypass that gate to obtain a
solver run. This is an external-flow check, not a motor cooling or
electromagnetic model.

Run from the repository root with a provisional 40,000 RPM and one revolution:

```powershell
.\cfd\su2\run_rotor_case.ps1 -Rpm 40000 -Revolutions 1 -StepsPerRevolution 120
```

Use `-Geometry airfoil_candidate` to select the separate geometry candidate.
Candidate outputs use `tmp\su2-rotor-airfoil_candidate\`; choose another
`-CaseDir` when you need a fresh run. Existing outputs are not overwritten.

The historical 2026-10-24 solver run used an earlier rotating-zone model and
completed 121 time levels (including t=0) through 0.0015 s, exactly one
revolution. It predates the current stationary-component clearance gate and is
not evidence that the geometry passes the updated physical checks. Its final
residuals had not converged, and its inviscid model and provisional RPM are not
suitable for design decisions.

## Scope

The original steady case is an inviscid startup check that does not model viscous
drag, transition, or propeller rotation. Both cases use coarse meshes and are not
suitable for design decisions. The OML must be refined with a wall-resolved or
validated wall-function mesh before using a viscous solver.

## Powered viscous experiment

`detailed_rotor_cfd.py` prepares and can run a finite three-case compressible SST
URANS campaign for either geometry candidate and the actual propeller STEP
blades. The user-selected exploratory operating point is 40,000 RPM; the assumed
forward-flight condition is 15 m/s, sea level, aircraft alpha +4 degrees. It uses
four counter-rotating sliding zones, second-order spatial/time discretization,
smooth adiabatic walls, 1% freestream turbulence intensity and viscosity ratio 10.

The medium and fine spatial meshes use respective airframe/propeller surface
sizes of 1.8/0.8 mm and 1.35/0.6 mm, with interface and downstream wake refinement.
The fine mesh is also run with half the physical time step: 120, 120 and 240
steps/revolution. Each starts from freestream and covers 100 revolutions (0.15 s,
approximately ten fuselage flow-through times); the final ten revolutions are
sampled. Force drift is compared between the final two five-revolution windows.
Two spatial levels provide sensitivity evidence, not a formal GCI study.

This mesh has **tetrahedra and standard wall functions, without prism layers**.
The report explicitly flags high y+, low y+ where wall functions deactivate,
unconverged pseudo-time/coupling residuals, and unsteady force drift. SST is fully
turbulent; transition and FDM surface roughness are unmodelled. These are material
limitations at this scale. No run is automatically labelled design-ready, even
when all numerical checks pass. Airframe drag and combined thrust/drag must be
interpreted separately.

All zones use the same 15 m/s reference speed for force coefficients, including
`MACH_MOTION`, the selected geometry's wing reference area, and mean aerodynamic chord.
The aircraft has nose +X and Z up; aircraft alpha +4 degrees corresponds to SU2
AOA 176 degrees. Reports use physical XYZ forces and explicitly project upward
lift and net streamwise force; SU2's default `LIFT` has reversed sign here.
The current multi-zone setup moves propeller surfaces only. Airframe and motor
surfaces remain stationary; the mesher rejects cases where a rotating zone
intersects either. The historical campaign approximation described in older
results is not used by the current setup and does not establish a valid stationary
wall model.

First generate and run the one-revolution Euler setup case for the selected
geometry with `run_rotor_case.ps1`. This supplies a mesh for the bounded SU2
configuration preflight below; the preflight does not launch the long URANS
campaign. No 100-revolution campaign has been run as part of integrating this
workflow.

Prepare with the project interpreter (no new dependencies):

```powershell
.venv\Scripts\python.exe cfd\su2\detailed_rotor_cfd.py --prepare
```

The baseline is the default. Add `--geometry airfoil_candidate` to select the
candidate. Campaign outputs default to `tmp/su2-campaign-pr3_baseline/` or
`tmp/su2-campaign-airfoil_candidate/`; `--case-dir` selects another directory.
Use `--preflight-mesh` only when the one-revolution mesh is stored outside the
selected geometry's default `tmp/su2-rotor*/` folder.

Preparation hashes the selected STEP geometry, propeller/motor inputs, geometry
profile/model sources, and mesher/config/runner sources; it records the SU2 binary
hash and checks that the configuration parses with SU2 8.5.0. Execution verifies
those hashes again. The actual finite payload, suitable for Codex Process Jobs,
is the same command without `--prepare`.
The payload runs existing unit tests first, generates and checks each mesh,
solves the cases sequentially, and writes `REPORT.md`, `campaign_results.json`,
`progress.json`, per-case quality/force/y+ summaries, solver logs and flow outputs.
It refuses to overwrite an existing calculation.

Resource limits: four OpenMP threads; at most 700,000 zone-local mesh nodes and
four million cells; stop the solver if free RAM falls below 0.8 GiB, free disk
below 1.5 GiB, or one solve exceeds 18 hours. The three-case campaign can therefore
take up to 54 solver hours plus meshing. A resource stop or solver failure preserves
available artifacts and produces a report stating the failure, without inventing
force results. Flow artifacts and case configurations remain under ignored `tmp/`.

References: [SU2 wall treatment](https://su2code.github.io/docs_v7/Markers-and-BC/),
[turbulence-model scope](https://su2code.github.io/docs_v7/Theory/),
and the [SU2 8.5.0 configuration template](https://github.com/su2code/SU2/blob/v8.5.0/config_template.cfg).
