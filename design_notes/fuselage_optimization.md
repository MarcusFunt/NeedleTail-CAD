# Fuselage profile refinement — 2026-10-24

**Historical custom-body study:** the active fuselage and pods now use
catalog-derived meridians. See [body_airfoil_comparison.md](body_airfoil_comparison.md).
Select `fuselage_body_airfoil=None` and `motor_pod_body_airfoil=None` to restore
the custom bodies. The CAD values and aerodynamic results below describe
that earlier custom-body stage, not the active NACA baseline.

The pods were subsequently given fuller, rounded aft profiles. The fuselage
CAD measurements below remain applicable; aerodynamic comparisons describe
the pod geometry used at the time of this fuselage study.

This is a bounded shape-refinement trial. The body keeps its reference
position, 230 mm length, nominal 60 mm diameter, and 3.6 mm finite end-cap
diameters. The wing, vertical arm, all four pods, motor interfaces, and the
shared -5 mm assembly offset are retained.

The refinement changes the nose/shoulder radius distribution and aft taper.
It preserves outer-envelope volume while reducing measured surface area.
It is not a demonstrated drag reduction or a global aerodynamic optimum.

## Saved CAD comparison

The following values are measured from the fuselage solids re-imported from
the before/after `airframe.step` exports, rather than inferred from renders.

| Quantity | Before | Refined |
| --- | ---: | ---: |
| Length, mm | 230.00 | 230.00 |
| Actual maximum diameter, mm | 60.22 | 60.07 |
| Outer-envelope volume, cm³ | 326.52 | 326.71 |
| External surface area, mm² | 28,764.17 | 28,701.70 |

Volume increases about **0.058%** and external area decreases about **0.217%**.
The source spline interpolates its radius stations, so actual maximum
diameter differs slightly from the nominal 60 mm station diameter.

The central region at X=-35 to +35 mm was protected with radius samples
every 5 mm: each radius may fall by at most approximately 0.1 mm relative
to the starting profile. These are exterior packaging proxies. The actual
battery bay and usable internal volume have not been modeled; unchanged
total envelope volume alone cannot guarantee component fit.

The saved engineering assembly retains seven valid solids. A before/after
check verifies that the other six solids keep their X/Y/Z bounds and volumes.
The Boolean-fused OML remains one valid solid. The four standard STEP exports
are regenerated, with a fresh one-off STL under
`tmp/fuselage-optimization/airframe_oml.stl`.

## Search and checks

A first sweep screened 343 combinations of maximum-radius station and
fore/aft profile scaling. Its feasible candidates did not improve the
objective. A subsequent local SLSQP refinement varied seven controls: the
nominal maximum-radius station and three radius ordinates on each side.
The objective was lower geometric surface area. Constraints retained
length/nominal diameter/end caps, volume within the baseline to +1%, the
central radius envelope, and limited nose and tail slopes.

The final refinement used 1,025 spline samples, converged in 22 iterations,
and was independently checked at 2,048 samples and against native CAD and
saved STEP geometry. The maximum aft slope is essentially retained within
sampling tolerance (about 0.25486 versus 0.25484 mm/mm); maximum forebody
slope decreases from about 0.7323 to 0.6774 mm/mm. An earlier, slightly
lower-area candidate was rejected because its tail slope was steeper.

The final normalized station and radius values are stored directly in
`AirframeParameters.fuselage_max_station_from_nose`,
`fuselage_aft_radius_profile`, and `fuselage_fore_radius_profile`.

## Aerodynamic trend

An isolated-body AeroBuildup comparison at 10, 15, and 20 m/s and 0°, 2°, 4°,
and 6° angle of attack predicts about **0.49% lower fuselage drag** with 201
axial stations. The corresponding 25-station estimate is about 0.50%, so the
sign of this small trend persists with finer sampling.

Using the existing whole-aircraft adapter at equal lift across the nine
2°/4°/6° baseline load points predicts **0.070% lower whole-aircraft drag on
average**, with a minimum reduction of about **0.061%**. The wing, supports,
pods, atmospheric conditions, and coefficient reference values are the same
in the two models. Candidate angle of attack is solved at each baseline
dimensional lift target; these are exploratory loads, not a supplied aircraft
weight or trimmed mission.

The body model uses an empirical fully turbulent skin-friction/form-factor
estimate and approximate geometric properties. Its fuselage form-factor
documentation explicitly limits the treatment of separation; see the
[official AeroSandbox model documentation](https://aerosandbox.readthedocs.io/en/master/autoapi/aerosandbox/aerodynamics/aero_3D/aero_buildup_submodels/fuselage_aerodynamics_utilities/index.html).
The CAD surface-area change and the model's approximate wetted-area change
are different quantities. No validated viscous CFD, powered propwash,
transition/roughness study, trim, loaded CG, or structural/printing validation
was performed. These gains are far smaller than the likely model uncertainty.

## Local artifacts and reproduction

The baseline parameter snapshot and STEP exports are retained under
`tmp/fuselage-optimization/`. `study.py` records the initial screen,
`final_refine.py` the constrained refinement, and `check_candidate.py` compares
the current design with the saved baseline using `compare_airframe_aero.compare`.
Run the latter from the repository root with the existing aero environment to
repeat the reported body and equal-lift comparisons:

```powershell
uv run --locked --extra aero python tmp/fuselage-optimization/check_candidate.py
```

Measurements are in `export_check.json`; aerodynamic comparisons in
`comparison.json`; refreshed sweeps/renders in `aero/`; and saved CAD snapshots
in the same directory. These experiment files are ignored local artifacts.
