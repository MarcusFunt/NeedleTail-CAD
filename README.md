# NeedleTail first-pass airframe CAD

## Coordinate system

- Units: millimeters.
- +X points from the aircraft center toward the nose; freestream enters from
  +X and travels toward -X.
- +Y is the right wing and +Z is up in the aircraft frame.
- The origin is the fixed fuselage datum at the original cruciform intersection.
  `assembly_x_offset=-5.0` moves both lifting surfaces, all pods, and their
  motor/propeller references 5 mm aft; the fuselage stays fixed. Negative values
  move aft and positive values move forward. `motor_station_x=1.0` is a local
  pod station; `motor_pod_center_x=-4.0` is its current aircraft coordinate.
- UIUC airfoil coordinates are transformed so the airfoil nose points toward
  +X, independent of the profile library's proposed alternate convention.

## Profile-based geometry

`airframe_geometry.py` is the shared parametric source. `AirframeParameters`
contains the dimensions and body, pod, and finite trailing-edge profiles. The
horizontal wing and vertical support each select their profile by name from
the local airfoil library in `profiles/original/`. The current library includes
SD7003, SD7037, SD8020, E423, NACA 0010, NACA 0015, NACA 2412, NACA 2415,
and NACA 4412. The vendored
Selig-format coordinates and source URLs and SHA-256 hashes are exposed by
`profiles/airfoil_coordinates.py`; sources are from the [UIUC Airfoil
Coordinates Database](https://m-selig.ae.illinois.edu/ads/coord_database.html).
Set `wing_airfoil` or `support_airfoil` to any catalog key to change sections.
These names are discrete optimization variables; dimensions and profile
stations remain numeric parameters on the same frozen dataclass.

An optimizer can iterate over `AIRFOILS` for the discrete section choice and
vary numeric `AirframeParameters` such as chord, span, pod length, and motor
leading-edge clearance. The CAD and AeroSandbox builders both accept a
parameter instance, so candidates can be generated from the same design state.

The default horizontal section derives from NACA 2415 and the vertical support
from symmetric NACA 0015; both source profiles remain replaceable from the
catalog. Chords are 80 mm root / 56 mm tip horizontally and 64 mm vertically.
`wing_thickness_scale` and `support_thickness_scale` are both 1.2: source
thickness is enlarged about the unchanged camber line from 15% to approximately
18%. These are modified sections, not the original NACA shapes. The last 6%
of each chord blends to a 1.2 mm physical trailing edge. Saved STEP section
depths are about 14.47 mm at the wing root, 10.13 mm at its tip, and 11.52 mm
through the vertical arm. These depths describe solid outer envelopes, not
shell-wall thickness or verified structural capacity.

The fuselage and motor pods now derive their revolved meridians from the
vendored NACA 0015 coordinates. `fuselage_body_airfoil` and
`motor_pod_body_airfoil` select those sources independently; SD8020 is the
comparison alternative. Set either selector to `None` to use that body's
historical custom profile. The custom fuselage station/fore/aft controls and
custom pod-radius tuple are inactive when their named-airfoil selector is set.

The NACA-derived fuselage is 230 mm long, about 60.00 mm across, and encloses
approximately 365.1 cm³. It uses the full symmetric half-thickness profile,
truncated to 1.8 mm-radius end caps and scaled independently in length and
radius. Its widest section is around X=+45.2 mm, about 30.3% aft of the nose.
These are airfoil-derived 3D bodies, not unmodified 15%-thick wing sections.
The preceding custom refinement is retained as a historical reference in
[fuselage_optimization.md](design_notes/fuselage_optimization.md); the new comparison is in
[body_airfoil_comparison.md](design_notes/body_airfoil_comparison.md).
The fuselage remains a solid outer-envelope part; the battery bay, hatch, and
component clearance need actual payload dimensions before they can be laid out.
The pod uses the source airfoil's afterbody, from its maximum-radius station
toward the trailing edge. The profile is truncated and scaled to the motor
interface, then blended into a rounded 3.0 mm aft end cap with a 1.0 mm cap
rounding parameter. `motor_pod_length=124` defines its local aft reference at
X=-62 mm; its forward motor face is X=+44 mm locally, leaving 106 mm of
plastic. Its nominal maximum diameter remains 19.9 mm. That is 1.325
times the 80 mm root chord and 1.89 times the 56 mm tip chord. The aft profile
has a fuller, rounded taper terminating in an approximately 3.0 mm flat end
cap, replacing the previous 0.30 mm needle-like tip. The overall length,
placements, and forward motor interface are retained. The pod itself
ends at the motor-sized mounting face, with no separate end disk or pad. For the
default station the face is at `X=+40.0 mm`, 17 mm ahead of the horizontal
wing's tip leading edge and 13 mm ahead of the vertical support's leading edge.
Motor and pod centerlines are coaxial, and the motor case is fully forward of
both surfaces. Pod length, shell diameter, and position remain parameters;
minimum forward-clearance and pod-to-wing-chord constraints keep optimization
candidates in the intended layout. The pod is a mounting body, not a motor
enclosure, and this first-pass geometry does not include a bracket or bolt
holes. The compact
layout uses a 190 mm
horizontal wing and a 160 mm vertical
support, shortened from the previous 208 mm and 170 mm. Both lifting surfaces
extend to the pod centerlines at Y=±95 mm and Z=±80 mm. The surface tip stations
are `(-5, ±95, 0)` and `(-5, 0, ±80)`; pod centers have X=-4 mm after the
shared assembly translation. Span is derived directly from motor stations, so
changing an offset also moves the matching wing tip. The lifting surfaces use
ruled lofts for a linear spanwise taper. Their gross planform areas are
12,920 mm² for the horizontal wing and 10,240 mm² for the vertical support; these
exclude fuselage overlap and are not aerodynamic effective-area estimates.
Relative to the preceding 208/170 mm spans and 73/40/48 mm chords, the current
horizontal planform is 9.9% larger and the vertical-support planform is 25.5%
larger, despite the shorter spans.

The 76.2 mm swept-disk envelope is tied to the shared airframe parameters and
checked against the motor mount faces, both lifting surfaces, the fuselage,
and adjacent disks. Current modeled clearances are 18.0 mm from the pod mount
face, 23.0 mm from the horizontal wing, and 31.0 mm from the vertical support;
adjacent disks have 48.0 mm edge clearance. The minimum permitted modeled
clearance is 1.0 mm.

## Motor and propeller interface

The reference motor mount face is at aircraft `X=+40.0 mm`, 44 mm ahead of each
pod center. The motor body extends 13.5 mm forward to `X=+53.5 mm`, ahead of
both lifting surfaces. Its shaft centerline is coaxial with the pod's +X
centerline. The propeller hub center is 18.5 mm forward of the mount face, at
`X=+58.5 mm`; the 4 mm reference hub starts at `X=+56.5 mm`. The motor CAD's
modeled shaft tip is at `X=+58.4 mm`, giving 1.9 mm of nominal shaft entry
into the hub. The shaft extension remains an unverified packaging assumption.

`propeller_envelope.step` places its 1 mm reference disks on the pod axes.
`motor_prop_installation.py` shows one pod, motor, and propeller as separate
solids. It aligns the pod's flat +X face with the motor mount face and rotates
the motor's local +Z shaft axis onto aircraft +X. This preserves the hub bore
and shaft clearance for inspection. The pod-fit validator checks the flat
mounting-face diameter against the motor case. Profile contours are split at
their minimum-X leading-edge point and normalized to unit chord. Both branches
share the actual nose, including cambered profiles with a nonzero nose height.
The selected reference STEP files and
their limits are documented in `references/propeller-kit/README.md`.

An initial 5 mm pod-only shift set the local pod-to-arm layout. The current
`assembly_x_offset` then translates the wings, arms, pods, and motor/prop
interfaces together while fixing the fuselage. Leading-edge layout targets
remain 17/13 mm, and the independent 1 mm swept-disk clearance requirement is
retained. See [pod_placement.md](design_notes/pod_placement.md) for placements and checks.

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
  76.2 mm diameter, 1 mm thick disks, centered on the pod axes.
- `motor_prop_installation.step` is a one-motor, one-propeller packaging
  reference assembly, with the components kept separate for review.
- `airframe_sd8020.step` is the separate SD8020-derived fuselage/pod
  alternative; its wing, arms, spans, and installation positions match the
  NACA baseline. Generate it with `python airframe_sd8020.py --force`.

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
`xfoil.exe` at `.venv\Scripts\xfoil.exe`. Generate geometry-matched section
polars for the airfoils selected in `PARAMETERS` and compare selected points
with XFOIL using:

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

The model includes the selected CAD-matched wing and support airfoils, the
central fuselage, and four motor pods. The airfoil sections preserve the
project's thickness scaling and 1.2 mm trailing-edge closure, and the fuselage and pod bodies follow
the CAD profile splines, including each pod's forward flat mount boundary. It
uses sea-level conditions at 10, 15, and 20 m/s to
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

## Shorter, thicker arm revision

The current geometry prioritizes deeper arms and less exposed span for the
user's 0.4 mm-nozzle FDM process. See [arm_revision.md](design_notes/arm_revision.md) for the
dimensions, saved-STEP measurements, checks, and print-planning limitations.
The 1.2 mm trailing edges replace the earlier 0.30 mm edges; slicing, material,
internal structure, and load validation are still required.

The earlier aerodynamic-only pass used a 208 mm span and 73/40 mm wing chords
and predicted a small 0.91% drag reduction. That historical result in
[aerodynamic_optimization.md](design_notes/aerodynamic_optimization.md) does not apply to
this revision. `compare_airframe_aero.py` compares the current model with the
original 200/170 mm, 72/44/48 mm arm geometry and original pod layout.
Historical CFD meshes and aerodynamic results must be regenerated for the
current geometry. Refreshed unpowered trends for the current shared-offset
placement and NACA-derived bodies are under `tmp/airfoil-bodies/naca-aero/`. Section polars remain under
`tmp/sturdier-arms/polars/`; the sections did not change in this translation.

## Compare the body templates

Run `uv run --locked --extra aero python compare_body_airfoils.py` to compare
the NACA 0015-derived bodies, SD8020-derived bodies, and historical custom
bodies. The report and meridian plot are written to `tmp/airfoil-bodies/`.
NACA 0015 is the active baseline. SD8020 predicts only about 0.06–0.07% less
whole-aircraft drag than NACA in the equal-lift comparison; this is not a
meaningful performance distinction at the model's uncertainty. Both templates
are fuller than the prior custom body and predict about 1.5–1.6% more drag.
