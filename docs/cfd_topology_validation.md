# Topology CFD validation status

**Status: experimental infrastructure; G-CFD has not passed.** PR-CFD-R adds
fail-closed geometry, mesh, and numerical gates to
`cfd/su2/topology_cfd_campaign.py`. It does not qualify any topology for ranking
or installed-propulsion correction.

## Gate status

| Gate | Status | Evidence and remaining issue |
|---|---|---|
| Bounded topology regression | Passed | Fixtures cover missing history steps, invalid clearance, signed tetrahedra, cell quality, connectivity, interface area, canonical spin, and torque-marker configuration without launching Gmsh or SU2. |
| Canonical rotor position, hand, axis, thrust direction, and spin | Software contract added | CAD placement, mesh placement, and SU2 angular rate use one `RotorStation`. Shaft axis and thrust direction are +X; coaxial aft rotors counterrotate. Positive spin maps to the reference CW STEP label when viewed aft-to-nose. The generic reference STEP hand and real motor rotation still need physical hardware verification. |
| Stationary airframe/motor and moving-zone geometry | Gates implemented; current geometry rejected | A Gmsh geometry-only probe of the baseline tractor rotor found that rotor 1's moving cylinder intersects both the OML and motor case. Pairwise overlap of independently moving cylinders is also rejected. The campaign stops before mesh creation; no fixed surface is assigned to a rotating zone. Revise the physical geometry or zone layout before another run. |
| Blade/hub shaft torque isolation | Marker path implemented; SU2 output not verified | Rotating zones monitor only `ROTOR_PROPELLER_n`; shaft torque is shifted to the rotor axis from that propeller-only force/moment. A real SU2 history is still needed to confirm the installed SU2 8.5 output columns and sign. |
| CAD clearance and coaxial separation | Gate implemented | CAD preflight rejects any swept-disk clearance below the configured margin. The CFD CLI and mesher require an explicit positive `--coaxial-separation-mm` for `coaxial8`, and pass it to station/config/mesh generation. |
| Mesh orientation, quality, connectivity, and interface coverage | Fixture checks passed; thresholds provisional | Inverted, degenerate, low-quality, disconnected, unmarked, area-mismatched, or spatially displaced interface faces are rejected. Per-zone minimum and 1st-percentile quality, marker areas and stationary/propeller/interface area splits are recorded. The current screen uses normalized tetra quality >= 0.01 and <= 2% interface-area difference; these limits have not been calibrated against an accepted production mesh. |
| Process completion, history completeness, and convergence | Software gates implemented | Results separately report solver exit, history completeness, numerical convergence, physical-model validity, and design readiness. Every expected physical step and finite load is required. Numerical screening also requires available zero nonphysical-point counts, declining final pseudo-time and multizone coupling residuals, final logs <= -5, and force/shaft-torque window change <= 5%. No solver run was made to validate these criteria. |
| Mesh/time-step sensitivity and design readiness | Not passed | The topology campaign runs one mesh/time-step setting. `design_ready` remains false even if its numerical screen passes. |
| Gmsh/SU2 solver execution | Not run | Keep the smoke sequence opt-in and run it only after the geometry gate passes. The current baseline fails the stationary-component intersection check. |

## Interpretation and limits

CAD swept-disk clearance alone does not prove that a full rigidly rotating zone
is clear of stationary parts. The Gmsh probe also checks airframe and motor
intersection with each rotor cylinder. Cases that fail either gate stop before
meshing or solving and cannot appear as completed comparisons. The same
geometry-only probe checks that independently moving rotor cylinders do not
overlap one another.

Use `--interference-diagnostic` to report swept-disk CAD clearance and geometry-
only Gmsh checks for fixed airframe/motor intersections. It creates no mesh and
never starts SU2, even when both gates pass; every output remains ineligible for
feasible rankings. This mode requires the optional Gmsh meshing extra. The normal
physical-comparison mode rejects any candidate that fails either geometry gate.

The SU2 shaft-torque history is configured on the propeller marker only; motor
and airframe loads are not called shaft torque. Physical CW/CCW identity, output
column semantics, torque sign, mesh quality limits, and numerical thresholds
remain provisional until checked against a real solver run and hardware data.

The four-rotor viscous workflow remains a separate campaign with its original
SST settings and reporting path. Its shared mesher/configuration now use the
propeller-only moving-wall contract and reject fixed-component intersections;
the revised viscous path has not been run or validated. Do not pass topology
CFD loads to the motor model or use any output to rank layouts until geometry,
mesh, numerical convergence, and mesh/time-step sensitivity gates are supported
by evidence.

## Opt-in smoke sequence

Use a fresh output path. This bounded case checks two revolutions at 12 steps per
revolution; it is a solver setup check, not a design run. Install SU2 8.5.0 and
the project's `cfd-meshing` extra first. The campaign preserves configuration,
mesh, history, and log files under the output directory and refuses to overwrite
an existing run.

To run only the CAD/Gmsh interference report without requiring SU2:

```powershell
.venv\Scripts\python.exe cfd\su2\topology_cfd_campaign.py `
  --topologies tractor4 `
  --interference-diagnostic `
  --output-dir tmp\topology-cfd-geometry-diagnostic
```

This still requires the optional Gmsh meshing extra but does not generate a mesh.

The following solver setup check is only appropriate after geometry passes:

```powershell
.venv\Scripts\python.exe cfd\su2\topology_cfd_campaign.py `
  --topologies tractor4 `
  --revolutions 2 `
  --steps-per-revolution 12 `
  --output-dir tmp\topology-cfd-smoke-tractor4
```

The baseline currently fails the stationary airframe/motor intersection gate,
so this command records the failure and does not start Gmsh meshing or SU2. The
topology runner does not currently provide a single-rotor case; an isolated
rotor compatibility smoke remains outstanding. After geometry and zone layout
are revised, validate tractor4 and pusher4, then only CAD-valid coaxial8, using
fresh output paths, and retain `campaign_configuration.json`,
`campaign_results.json`, `REPORT.md`, each case's mesh/configuration/history/logs,
and the exact solver build hash. A coaxial smoke case must add, for example,
`--topologies coaxial8 --coaxial-separation-mm 60` and still pass all preflights.
