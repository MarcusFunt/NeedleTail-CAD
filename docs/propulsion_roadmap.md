# Propulsion integration roadmap

This roadmap tracks the propulsion work completed by PR-A, PR-B, and PR-C and the remaining integration gates. PR-CFD-R adds safety checks to the existing topology CFD path; it does not qualify a design or alter optimizer ranking.

## Completed in PR-A–C

- Added SI geometry, operating-condition, result, provenance, and coefficient contracts with validation tests.
- Added checksum-aware source parsers, a backend-neutral benchmark runner, per-point records, metric summaries, and local run manifests/plots.
- Reproduced the supplied frozen 195-row reference suite and documented the corrected GWS 3×2 diameter as 3.25 in (82.55 mm).
- Added a frozen-BEMT adapter and verified it against the original implementation over all 195 reference rows.
- Added an adapter for checksum-pinned QPropC 2603 and reproduced its APC and Graupner upstream categories.
- Added a checksum-pinned Graupner CAM 9×4 procedural family holdout, retaining negative-thrust records and its data-rights and blinding limitations.
- Added a backend comparison ADR with a no-winner decision. No candidate currently has sufficient in-range experimental coverage for production use.

## Next gates

| Work | Status | Gate before use |
|---|---|---|
| PR-D: motor and battery operating points | Not started | Verify motor/ESC/battery parameters with sources and uncertainty; enforce current, voltage, thermal, RPM, and power balance; prove feasible map coverage before reporting achievable speed. |
| PR-CFD-R: repair the existing topology CFD campaign | Software gates implemented; G-CFD not passed | The baseline Gmsh geometry probe finds airframe and motor intersections with rotor 1's moving zone and stops before meshing. Physical CW/CCW identity, real SU2 history, calibrated mesh acceptance thresholds, and mesh/time-step sensitivity remain unverified. SU2 results cannot rank layouts or correct isolated maps. |
| PR-E: installed-flow study | Deferred | Requires valid motor points and G-CFD; reuse the existing topology campaign and keep installed/coaxial uncertainty explicit. |
| PR-F: physics-constrained optimizer | Deferred | Requires validated propulsion maps, motor feasibility, complete trim/force balance, and preserved compatibility with the existing concept-screen mode. |

## Open evidence needs

- Exact section/airfoil identity and Reynolds/angle coverage for each tested propeller, especially the Graupner holdout.
- Additional independent propeller families, ideally multiple pitch/diameter and blade-count regimes. The current holdout is public and procedural, not blind.
- Better forward-flight thrust and torque evidence near zero thrust and at high advance ratio; preserve windmill measurements and report appropriate absolute and normalized errors.
- Experimental motor resistance, no-load current, thermal/sustained current limits, ESC losses, battery sag, and structural RPM limits for the selected hardware.
- Valid coaxial spacing, installed flow, mass, CG, control, and flap-dependent aerodynamic evidence before comparing topologies.
- Clear redistribution terms for raw source tables. Unlicensed or unclear-rights source values and run rows remain local; only hashes, methods, and aggregate summaries are versioned.

## Operating boundaries

The generic 3×4 STEP model is packaging geometry only. The fixed-RPM exploratory SU2 campaign is not a motor model, and its aggregate zone loads are not motor shaft torque. Do not infer attainable maximum speed, topology preference, or installed-flow correction from the current PR-A–C results.
