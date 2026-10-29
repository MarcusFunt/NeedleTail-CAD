# Topology CFD validation status

**Status: experimental infrastructure; G-CFD has not passed.** This PR-A–C work does not repair or execute `cfd/su2/topology_cfd_campaign.py`. The workflow remains unsuitable as evidence for topology ranking, blade shaft power, or installed propulsion corrections until the physical and numerical gates below are satisfied.

## Gate status

| Gate | Status | Evidence and remaining issue |
|---|---|---|
| Bounded topology regression | Passed as part of the 114-test repository suite | These tests do not prove mesh validity or a real solver run. |
| Canonical rotor position, hand, axis, and spin | Not passed | The integration plan records inconsistent spin conventions between configuration and mesh generation. Establish one tested physical contract across optimizer, CAD, Gmsh, and SU2. |
| Stationary airframe wall motion | Not passed | The current rotating-zone construction can include clipped airframe patches. Demonstrate that fixed wing, pod, and fuselage surfaces remain fixed, or mark the relevant geometry unsupported. |
| Blade/hub shaft torque isolation | Not passed | Aggregate rotating-zone moments include surfaces beyond the driven blade/hub. Separate physical surface loads and verify torque-axis origin and sign before interpreting motor power. |
| Coaxial/pusher geometry preflight | Not passed | Reject invalid clearance/overlap in physical-comparison mode; diagnostic outputs must not enter rankings. |
| Mesh quality and interface checks | Not passed | Signed orientation, cell quality, interface coverage, and disconnected-volume checks need validated acceptance limits and fixtures. |
| Numerical convergence and sensitivity | Not passed | A complete history is not convergence. Require residual and load stability plus mesh/time-step sensitivity evidence. |
| Gmsh/SU2 execution | Not run in this phase | No expensive CFD was launched. Keep it opt-in and run only after the geometry and mesh gates are addressed. |

## Allowed interpretation

The bounded tests check software behavior without requiring Gmsh or SU2. Passing them supports regression confidence in the current utilities only. It does not certify any CAD geometry, moving interface, load decomposition, mesh, physical model, or solver convergence. Keep topology results labeled experimental and do not pass aggregate CFD torque to the motor model.

The separate four-rotor viscous campaign remains a distinct workflow; this work neither changes nor validates it. The topology campaign remains topology-only with no flap matrix. The generic propeller-kit STEP remains packaging-only.

## Next actions

Address the existing campaign in a separate PR-CFD-R: unify rotor station conventions, remove moving fixed surfaces, isolate blade/hub torque, reject infeasible CAD, expose and validate coaxial separation, and separate process completion, history completeness, numerical convergence, geometry validity, and design readiness. Add dependency-free invalid geometry and history fixtures, then document a small opt-in solver smoke sequence. Preserve mesh, configurations, and logs for any later CFD run.
