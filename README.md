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
cavity. The compact layout uses a 240 mm horizontal wing and a 170 mm vertical
support, reduced from 260 mm and 190 mm. Motor stations are `(0, ±100, 0)` and
`(0, 0, ±85)`, 10 mm closer to the center on each arm. Their nearest nominal
76.2 mm propeller disks have 55.0 mm edge clearance. The span changes reduce
planform area by about 7.7% for the horizontal wing and 10.5% for the vertical
support.

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
clearance for inspection. The selected reference STEP files and their limits
are documented in `references/propeller-kit/README.md`.

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

From this directory, run:

```powershell
python .\airframe_oml.py --force
python .\airframe.py --force
python .\propeller_envelope.py --force
python .\motor_prop_installation.py --force
cadgen step inspect refs .\airframe_oml.step --facts --planes --positioning
cadgen step inspect validate .\airframe_oml.step
cadgen step inspect refs .\airframe.step --facts --planes --positioning
cadgen step inspect validate .\airframe.step
```

This workspace currently reports cadgen 0.6.3, Python 3.13.5, and build123d
0.11.1. The companion CFD plan specifies build123d 0.12.0; regenerate the STEP
under that pinned environment before treating it as the controlled CFD
baseline.
