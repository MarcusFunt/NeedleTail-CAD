# Integration Plan: Carry Airfoil and CFD Work onto Main

## Objective

Collect the work from `codex/airfoil-geometry-and-notes` onto current `main`, while preserving the revised airframe as an explicit geometry candidate. This plan is for integration and verification so the project is collected in one place for the next design decision. It does not select a topology, promote the candidate to the default geometry, or run the full CFD campaign.

## Starting Point

- PR #3 is merged on `main` at `41f0fb6b98ac547c43445f7e0afadcb51be1c1d8`.
- The source branch is 21 commits behind main and contains two feature commits: `347b2bb` (airfoil-driven airframe design) and `dc799ca` (exploratory SU2 rotor CFD workflow).
- Relative to the merged main baseline, the source branch changes 33 files. Its geometry and aero edits overlap PR #3, so do not merge the stale branch wholesale or replay its commits without adapting them to current interfaces.
- Port the work onto fresh branches from current main, keep the source branch as a reference, and merge in dependency order.

## Required End State

- The merged PR #3 geometry remains the unchanged default and can still reproduce the current baseline outputs.
- The revised airfoil-driven geometry is available through a clearly named candidate configuration and can be explicitly selected in CAD, AeroSandbox, and existing optimizer workflows.
- Existing optimizer defaults and named mass scenarios remain intact. Geometry choice is recorded with each generated result.
- The airfoil data, comparison utilities, engineering notes, and CFD workflow from the two source commits are either ported or explicitly accounted for in the integration review.
- CFD tooling is integrated and has bounded verification; the 54-hour campaign is not run as part of this integration effort.
- No topology or hardware decision is made during this work.

## Task 1: Integrate the Revised Geometry as a Separate Candidate (PR 1)

Create a fresh branch from current `main`. Port and adapt the geometry commit `347b2bb` as one cohesive geometry-and-aero change.

### Candidate/baseline API

- Keep the existing merged-main `AirframeParameters` values as the baseline configuration.
- Add a separate named candidate configuration using the source branch's defaults: fuselage 230 mm long with 30 mm maximum radius; wing chords 80/56 mm using NACA 2415, 1.2 thickness scale, and 1.2 mm trailing edge; 64 mm NACA 0015 support with 1.2 thickness scale and 1.2 mm trailing edge; 95/80 mm motor offsets; 124 × 19.9 mm NACA 0015 pods; assembly X offset −5 mm and local motor station +1 mm.
- Require callers to select the candidate explicitly. Do not replace the default `PARAMETERS` or silently change the tractor/no-flap legacy output.
- Keep the candidate labeled preliminary. Preserve its dimensions as supplied in the source branch; do not tune them during integration.

### Files and feature groups to account for

- Shared geometry and profiles: `airframe_geometry.py`, `profiles/airfoil_coordinates.py`, `profiles/original/*.dat`, and `airframe_sd8020.py`.
- Geometry consumers: `motor_prop_installation.py`, `propeller_envelope.py`, `whole_aircraft_aero.py`, and `airfoil_polars.py`.
- Comparison utilities and research notes: `compare_airframe_aero.py`, `compare_body_airfoils.py`, and all five documents under `design_notes/`.
- Project docs and checks: `README.md` and `test_airframe_geometry.py`, plus focused tests for candidate selection and adapter alignment.

### Acceptance and merge gate

- Baseline geometry/CAD output remains reproducible with no implicit switch to the candidate.
- Candidate profile provenance and coordinate validity are recorded; candidate CAD solids, caps, thicknesses, physical trailing edges, envelopes, and propeller clearances pass the project checks.
- CAD, AeroSandbox, propeller envelopes, and motor/pod placements share one explicit local/global coordinate contract; the assembly offset is applied once.
- Existing PR #3 topology and mass-scenario behavior remains available. Geometry and scenario names appear in optimizer outputs.
- The documentation and comparison scripts explain that the candidate is preliminary and that their aero estimates retain the documented model limitations.
- Locked default and AeroSandbox checks pass; the candidate is explicitly exercised without changing the baseline defaults.
- Merge PR 1 before beginning the CFD port.

## Task 2: Port the Exploratory SU2 Workflow (PR 2)

Create a new branch from the updated main after PR 1 merges. Port and adapt `dc799ca`'s ten CFD files into a separate CFD-focused PR. Keep CFD optional so a standard checkout and test discovery do not require Gmsh or SU2.

### Integration work

- Declare CFD-specific dependencies in the existing optional environment and isolate CFD tests from the default test suite.
- Bind prepared cases to solver, source, geometry, and configuration hashes; verify those hashes at execution.
- Prevent output overwrites and preserve incomplete-run artifacts with a clear failure status.
- Add bounded checks for the five-zone mesh, boundary markers/interfaces, cell validity, force normalization, torque/power signs, and run completeness.
- Keep reports explicit about the exploratory model and its limits.

### Defaults to retain for later campaign use

- Operating point: 40,000 RPM, 15 m/s, +4° aircraft angle of attack, sea level.
- Three sequential cases, 100 revolutions each; mesh scales 1.0 and 0.75; 120/120/240 steps per revolution.
- Four OpenMP threads; 18 hours per case (54 solver-hours total); maximum 700,000 zone-local nodes and 4 million cells.
- Start with at least 6 GiB free disk; stop below 0.8 GiB available RAM or 1.5 GiB free disk.

These are the source branch's configured assumptions and caps. Retain them for the future run, but do not execute the multi-case campaign as part of getting the code onto main.

### Acceptance and merge gate

- Default test discovery works without CFD extras; CFD unit/config tests run in the CFD environment.
- Hash verification, safe output handling, resource-stop reporting, mesh checks, and sign checks have bounded tests.
- A configuration/preflight check and one short solver case validate compatibility; no 100-revolution campaign is required for this integration PR.
- Merge PR 2 after CI and code review pass.

## Task 3: Final Mainline Audit

- Compare the merged files against `git diff main...codex/airfoil-geometry-and-notes` and account for all 33 changed paths from both commits.
- Confirm the baseline geometry remains the default, the candidate is explicitly selectable, the optimizer/mass defaults are unchanged, and CFD tools are optional.
- Confirm the two follow-up PRs are merged to current `main` and no source-branch work needed for the stated scope remains stranded.
- Leave topology selection, hardware selection, candidate retuning, and the full CFD campaign as later decisions for the user.
