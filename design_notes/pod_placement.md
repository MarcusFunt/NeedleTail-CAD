# Aft pod placement — 2026-10-24

## Current shared assembly offset

The fuselage remains fixed. `AirframeParameters.assembly_x_offset=-5.0` now
translates both lifting surfaces, all four pods, the motor/prop reference,
and all four propeller disks 5 mm aft relative to the preceding pod-only
revision. The local pod station remains `motor_station_x=1.0`, preserving
that revision's pod-to-arm spacing. Use this one shared offset for future
common translations; `motor_pod_center_x` exposes the resulting aircraft X.

Current global X stations are -5 mm for the arm intersection, -4 mm for pod
centers, +40 mm for motor mounts, and +58.5 mm for propeller centers. The
fuselage keeps its prior position and shape. Local motor leading-edge gaps
remain 17/13 mm; conservative disk-to-arm clearances remain 23/31 mm. The
1 mm disk clearance requirement and all prior layout bounds are unchanged.

The AeroSandbox adapter applies the same placement, including the reversed
X sign and millimeter-to-meter conversion. Its central fuselage and moment
reference remain fixed; its lifting surfaces and four pod bodies move.
The historical comparison baseline explicitly restores a zero shared offset.
No stability improvement is claimed without a loaded CG and trim analysis.

Before-shift STEP exports are in `tmp/assembly-aft/before/`. Saved STEP
translation checks and snapshot reviews are under `tmp/assembly-aft/`.
Older CFD and aerodynamic results need regeneration for this placement.

All 22 tests pass, including arbitrary-offset translation tests for CAD and
the AeroSandbox adapter, preserved clearance reports, and saved STEP validity
and solid counts. The saved before/after check confirms zero motion of the
fuselage and -5 mm X motion of both arms, all four pods, all disks, and the
motor/prop reference, with individual volumes and Y/Z bounds retained.
The OML remains one fused solid. Current aerodynamic sweeps and geometry
renders are under `tmp/assembly-aft/aero/`.

## Earlier pod-only translation

All four pods are translated 5 mm along aircraft -X. Their source center
station changes from X=+6 to +1 mm. Pod dimensions, spans, arm sections,
fuselage, motor interfaces, and shaft/hub engagement are retained. The
motor/prop reference assembly and the four swept disks translate with the pods.

| Aircraft X station, mm | Before | After |
| --- | ---: | ---: |
| Pod center | 6.0 | 1.0 |
| Motor mounting face | 50.0 | 45.0 |
| Motor front | 63.5 | 58.5 |
| Propeller hub/disk center | 68.5 | 63.5 |
| Shaft tip | 68.4 | 63.4 |

The previous 20/18 mm minimum forward mounting-face layout targets would
reject this translation. They have explicitly been revised to 17/13 mm,
the new gaps ahead of the horizontal tip and vertical-arm leading edges.
These are provisional layout bounds, separate from the 1 mm swept-disk
collision clearance requirement, which is retained. Moving farther aft
requires another layout review; the validator still rejects it.

Conservative modeled disk clearances are now 23 mm to the horizontal wing,
31 mm to the vertical support, 11.9 mm radially to the fuselage envelope,
18 mm to the mounting face, and about 48 mm between adjacent disk edges.
The motor face remains forward of both arms; nominal shaft/hub engagement
remains 1.9 mm. These are packaging checks, not a powered-flow validation.

The four STEP exports are rebuilt from the shared parameters. Before-shift
STEP files and source are retained in `tmp/pods-aft/before/`. Older CFD and
aerodynamic outputs describe earlier placements and need regeneration for
this layout.

All 19 tests pass, including re-imported STEP validity and expected solid
counts. A separate before/after check of the saved exports verifies exactly
-5 mm X translation for all four pod solids, all four disks, and all nine
motor/prop reference solids, with Y/Z bounds and individual volumes retained.
It also verifies the three stationary airframe components retain their bounds
and volumes. Results are saved in `tmp/pods-aft/translation_check.json`.

## Should the wing-and-arm assembly also move aft?

It is a reasonable configuration to investigate, but fuselage appearance or
similarity to another design does not establish a useful wing station.
For longitudinal static stability, the relationship between the loaded
center of gravity and the aircraft neutral point matters. Trim and control
authority must also be checked. NASA discusses the CG/aerodynamic-center
relationship in [Introduction to the Aerodynamics of Flight](https://appel.nasa.gov/wp-content/uploads/2018/08/NASA-SP-367-Intro-to-Aero-of-Flight.pdf)
and the moment balance in [Trimmed Aircraft](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/trimmed-aircraft/).

If the CG were held fixed, moving the principal lifting assembly aft could
move the neutral point aft and increase static margin. That is a design
hypothesis, not a stability result for this airframe. Moving the arms and
motors also moves their mass aft, shifting the CG and changing the net effect.
The battery and payload placement therefore matter. This cruciform has no
separate conventional horizontal tail or defined control surfaces, so a
conventional aircraft's tail-placement logic cannot simply be copied.

Moving the fuselage +X or translating the wings, arms, and pods -X by the
same amount produces equivalent relative exterior geometry, up to a global
translation. Moments and CG must use a consistent reference whichever
coordinate choice is used. A separate common assembly offset would make a
later design study easier to compare while preserving local motor clearances.

The subsequent shared shift above is a provisional 5 mm design iteration.
Establish component masses and loaded positions (especially the battery),
estimate CG, then assess neutral point, trim, and control authority before
finalizing the assembly station.
