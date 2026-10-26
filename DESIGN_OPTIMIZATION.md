# Unified NeedleTail propulsion, CAD and control-surface design space

**Status: engineering preview, awaiting locked-environment verification and physical calibration.** This implementation is intentionally a feature branch; the current tractor STEP generators and validated default CAD geometry are unchanged.

## Supported configurations

The common `Design` object in `design_configuration.py` holds:
- `topology`: `tractor4`, `pusher4`, or `coaxial8`.
- `flaps`: `none`, `horizontal` (two panels), or `all_four`.
- `hinge_fraction`, exposed-span start/end fractions, maximum flap deflection.
- Optional `coaxial_separation_mm`; default mirrored hub centers are +23 and -23 mm for the current compact model (46 mm / 0.60D).

The base CAD dimensions and section airfoils **always** come from `AirframeParameters`; there is no unrelated copy of wing span or pod geometry. `tractor4 + none` preserves the existing output.

## Usage

```powershell
uv sync --locked --extra aero
uv run --locked --extra aero python -m unittest -v test_design_optimizer
uv run --locked --extra aero python design_optimizer.py --settings design_optimizer_settings.json
uv run --locked --extra aero python design_optimizer.py --check-best-cad
```

Results: `tmp/design-optimizer/designs.json` and `designs.csv`.

Programmatic CAD variant preview:

```python
from airframe_geometry import PARAMETERS
from design_configuration import Design
from design_cad import validate_variant, export_concept_step

design = Design(topology="pusher4", flaps="horizontal")
clearance = validate_variant(PARAMETERS, design)
export_concept_step(PARAMETERS, design, "tmp/pusher4-horizontal.step")
```

If validation fails, **do not export**. In particular, a rear propeller can collide with the existing trailing-edge geometry. The pusher layout is not automatically buildable just because a mirrored axial location exists.

Programmatic unpowered AeroSandbox:

```python
from airframe_geometry import PARAMETERS
from design_configuration import Design
from design_aero import build_variant_airplane, flap_derivatives

d = Design(topology="tractor4", flaps="all_four")
airplane = build_variant_airplane(PARAMETERS, d)
names, dmoment_dflap = flap_derivatives(PARAMETERS, d, speed_mps=30)
```

Check the physical coordinate convention/sign and AeroSandbox release before feeding derivatives into the optimizer. The adapter currently produces *unpowered attached-flow* effects. It does **not** model a rotor slipstream, structural hinge torque, aeroelastic deflection, servo packaging or large-angle transition flow.

## Joint geometry and optional aerodynamic optimization

The optimizer uses `AirframeParameters` as the **only** source of physical dimensions.
`design_geometry_grid_example.json` demonstrates an opt-in sweep of horizontal
motor offset and wing chords, jointly crossed with the 9 propulsion/flap families:

```powershell
uv run --locked --extra aero python design_optimizer.py --settings design_geometry_grid_example.json
```

You may vary `wing_root_chord`, `wing_tip_chord`,
`horizontal_motor_offset`, `vertical_motor_offset`, and
`fuselage_length` in millimeters. Invalid CAD parameters are rejected by
the baseline geometric checks; exact 3D geometry validation is available for
shortlisted candidates using `--check-best-cad`. This initial score still
does **not** include full geometry-dependent lift, trim or propulsion, so
its ordering must not be used as a speed-optimum claim.

To explore geometry-dependent (but still unpowered) AeroSandbox flap moments,
add `--experimental-aero-derivatives`. Regression tests finite-difference plus
and minus deflections at two operating points and check the ASB-to-CAD moment
signs. This verifies the model's axis and polarity convention; actual servo
polarity and installed-flow derivatives still require physical validation
before control hardware or a flight controller uses these estimates.

## What the optimizer actually does

For each of the 9 mode combinations, the script tests hinge fractions 0.65/0.72/0.80, exposed-span start 0.18/0.35 and deflection limit 15°/25° (no-flap modes are each evaluated once). At the selected trial airspeed it assembles a bounded **roll/pitch/yaw** moment-allocation matrix from rotor station lever arms and estimated flap pressure/area. It solves an LP for maximum collective rotor thrust while satisfying prescribed moment demands. Then it subtracts a notional flap drag and mass penalty. It reports:

- Feasible/infeasible moment allocation (given assumed demands and rotor limits)
- Additional motors and actuators mass
- Flap count/size and incremental drag
- Collective thrust sacrificed to moments
- Estimated surface moment capacity for roll, pitch, yaw
- Assumed deflections, residual and per-trial provenance flags

It is **not** a flight-speed optimizer yet. The objective does not solve total longitudinal force balance, battery electrical power, motor–prop torque/RPM, real CG or transition dynamics. Do not interpret its score as max speed, validated control margins or aircraft stability.

### Mass assumptions and sensitivity scenarios

The optimizer treats hardware mass as a replaceable estimate. `design_optimizer_settings.json` exposes the assumed mass of an added motor, ESC, propeller, rear mount, and flap servo/actuator. The values at the top level are the baseline assumptions. `mass_scenarios` names alternate runs and overrides only the components that differ; omitted components inherit the baseline values.

The sample settings compare `baseline` with light and heavy sensitivity cases set 20% below and above those baseline guesses. These are example brackets, not selected hardware specifications. Replace them as the candidate hardware becomes clearer, or add scenarios for different motor/prop/servo combinations. Each output row records the scenario, the assumptions used, the added-mass component breakdown, and its rank within that scenario. `--check-best-cad` checks the top nine candidates in each named scenario.

The added-mass estimate is relative to the existing 4-tractor/no-flap baseline. It does not include a complete aircraft weight or solve the effect of total weight on trim; the optimizer uses it only as the documented score penalty.

### Why two versus four surfaces is worth evaluating

The horizontal pair can directly contribute roll and pitch moments, and the vertical pair can add roll and yaw authority. At zero freestream speed, the current unpowered approximation gives flaps zero effectiveness. No propwash benefit is invented. Full transition analysis requires a coupled actuator-disk or rotor wake and six-degree-of-freedom model.

## CAD limitations

`design_cad.py` uses the existing fuselage/wing/support/pod solids, mirrors the axisymmetric pod for rear propulsion, splits neutral deflection flap panels into conceptual solids with a 0.35-mm gap, and calculates exact OCCT swept-disk distances. The rear motor **cavity and shaft** are not validated in those mirrored pods; actuator/hinge/linkage geometry is absent. Coaxial pairs currently use overlapping fore/aft pod geometry as a concept, not finished internal packaging. The separate CAD pieces are exported as an assembly STEP; this is not a validated watertight CFD OML.

`design_aero.py` uses the CAD-matched section contours, independently addressable control surfaces in AeroSandbox, and a mirrored pusher/combined coaxial nacelle **proxy**. No propeller slipstream is included.

## Required milestones before claiming a high-speed advantage

1. Execute the project's locked build123d/cadgen and AeroSandbox environments on the new files. Verify all 9 variants and expected *rejection* of colliding rotor envelopes.
2. Validate actual CW/CCW prop placement, the sign of all independent aerodynamic control derivatives and CG-adjusted moments. Finite-difference at multiple angles, Reynolds numbers and speeds.
3. Make the rear pod structurally and mechanically plausible and CAD-clearance-test the flap range of motion, motor bell/hub, cooling path, hinges and wiring.
4. Feed geometry-dependent AeroSandbox/CFD forces and moments and measured propeller `CT(J)`/`CQ(J)` into the **same** nonlinear trim + electrical model.
5. Make the objective real **feasible maximum speed** (with roll/pitch/yaw authority, battery, thermal and structure constraints), then run a robust 9-family/geometry optimization.
6. Validate the best candidates with mesh-converged viscous CFD, wind-tunnel/bench prop data and measured masses.

The simple analysis is deliberately useful for identifying where improvements *might* occur, without presenting invented full-aircraft performance results as facts.
