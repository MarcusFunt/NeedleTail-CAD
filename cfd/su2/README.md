# SU2 external-flow setup

This is a first end-to-end SU2 check against `airframe_oml.step`. It creates an
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
`tmp\su2-15mps\`.

## First run status

On 2026-10-24, mesh generation produced 29,294 nodes and 158,374 tetrahedra.
SU2 completed its 300-iteration run, but the pressure residual did not meet the
convergence criterion. A separate 1,000-iteration run at CFL 0.2 also reached
its iteration limit without converging. Treat both as solver/setup checks; do
not use their force coefficients for design decisions.

## Blade-resolved rotating-propeller case

`run_rotor_case.ps1` builds one stationary outer-flow zone and four blade-resolved
sliding rotor zones from the current `PARAMETERS.propeller_mount_stations` and the
CW/CCW propeller STEP files. SU2 moves each rotor mesh at the requested RPM. The
rotor zones use a parametric cylindrical motor-can envelope, fused with the actual
prop STEP solids to remove overlap in the imported motor/prop assembly. The
stationary zone uses the detailed motor STEP; the rotating zones omit its shaft
and internal motor details. This is an external-flow check, not a motor cooling or
electromagnetic model.

Run from the repository root with a provisional 40,000 RPM and one revolution:

```powershell
.\cfd\su2\run_rotor_case.ps1 -Rpm 40000 -Revolutions 1 -StepsPerRevolution 120
```

The solver uses compressible inviscid Euler flow at 15 m/s cruise, with 120 time
steps per revolution. The generated mesh has five zones and about 369,000
tetrahedra. SU2 outputs per-zone `.vtu` snapshots, surface CSVs, restarts, and the
multizone history under `tmp\su2-rotor\`. The 2026-10-24 run completed 121 time
levels (including t=0) through 0.0015 s, exactly one revolution, with no solver or
interface errors. It was a transient setup check: the final residuals had not
converged, and its inviscid model and provisional RPM are not suitable for design
decisions.

## Scope

The original steady case is an inviscid startup check that does not model viscous
drag, transition, or propeller rotation. Both cases use coarse meshes and are not
suitable for design decisions. The OML must be refined with a wall-resolved or
validated wall-function mesh before using a viscous solver.

## Powered viscous experiment

`detailed_rotor_cfd.py` prepares and runs a finite three-case compressible SST
URANS campaign for the current NACA-derived airframe and actual propeller STEP
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
`MACH_MOTION`, the current wing reference area, and current mean aerodynamic chord.
The aircraft has nose +X and Z up; aircraft alpha +4 degrees corresponds to SU2
AOA 176 degrees. Reports use physical XYZ forces and explicitly project upward
lift and net streamwise force; SU2's default `LIFT` has reversed sign here.
Motor-can external surfaces rotate with their rotor zone, as in the previous
blade-resolved setup; this is not a motor-internals model.

Prepare with the project interpreter (no new dependencies):

```powershell
.venv\Scripts\python.exe cfd\su2\detailed_rotor_cfd.py --case-dir tmp\su2-powered-20261025 --prepare
```

Preparation hashes all CAD inputs and relevant model/mesher/runner sources and
checks the configuration with the installed SU2 8.5.0 binary. The actual finite
payload, suitable for Codex Process Jobs, is the same command without `--prepare`.
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
