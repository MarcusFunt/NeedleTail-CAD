# NACA 0015-derived bodies versus SD8020

NACA 0015 is the body reference for the preliminary airfoil geometry
candidate. The PR #3 geometry remains the repository default.
`airframe_sd8020.py` produces the candidate comparison engineering assembly.
Both use the existing vendored UIUC
coordinate files, with source URLs and SHA-256 hashes recorded by
`profiles/airfoil_coordinates.py` and the comparison JSON.

## How the source sections become bodies

The generator reads the original symmetric half-thickness distribution,
normalizes its maximum radius, and uses it as the meridian revolved around X.
It does not use the wing's 1.2 thickness multiplier or blunt wing trailing edge.
Cambered source sections are rejected for these body-template selectors.

The fuselage uses both forebody and afterbody. Its nose and tail are truncated
at a specified radius, then the retained source interval is mapped to the
230 mm body length and nominal 30 mm radius. End caps are 3.6 mm across.

The pod uses the maximum-thickness-to-trailing-edge portion. It is truncated
where it can meet the aft rounding, mapped between the existing mounting
face and the tail, and joined to a sampled quarter-round cap profile. The
aft cap remains 3.0 mm across. Each pod remains nominally 106 mm long and
19.9 mm across, with its flat mounting face at aircraft X=+40 mm. A global
interpolating spline can bulge aft by less than 0.001 mm at the round cap.

Length/radius scaling, truncation, and the rounded cap change the geometry
from the original 15% NACA or 10.1% SD8020 wing section. The airfoil names
describe traceable shape provenance, not the aerodynamic behavior of a stock
wing section. Bodies of revolution based on airfoil sections are also
described in [NASA's OpenVSP guidance](https://www.nasa.gov/reference/openvsp-bodies-of-revolution/).

CAD and the aerodynamic adapter consume the same meridian station generators,
`fuselage_radius_stations` and `motor_pod_radius_stations`. Body geometry is
therefore selected from one source rather than duplicated in the two models.

## Geometry comparison

The length and nominal diameter envelopes, wing/arm geometry, -5 mm shared
assembly offset, motor/shaft/hub interfaces, and propeller positions match.
Volume is not constrained equal: the two catalog-derived bodies are fuller.

| Quantity | Custom reference | NACA 0015-derived | SD8020-derived |
| --- | ---: | ---: | ---: |
| Fuselage volume, cm³ | 326.71 | 365.08 | 361.67 |
| Fuselage surface area, mm² | 28,701.70 | 31,262.64 | 31,027.22 |
| One pod volume, cm³ | 17.56 | 18.63 | 18.83 |
| One pod surface area, mm² | 4,902.33 | 5,073.45 | 5,094.66 |
| One pod maximum diameter, mm | 19.896 | 19.896 | 19.907 |

The NACA fuselage maximum diameter is about 60.000 mm; the SD8020 version is
about 60.046 mm because of interpolation overshoot. Both are nominal 60 mm
designs within the existing envelope tolerance. The mounting faces remain
about 19.896 mm across in both designs, within the motor-fit allowance.

NACA adds about 11.7% fuselage volume over the custom reference; SD8020 adds
about 10.7%. Neither figure establishes battery fit or usable internal volume:
these remain solid outer-envelope models without a modeled shell or bay.

## Equal-lift aerodynamic comparison

`compare_body_airfoils.py` uses the existing AeroBuildup/NeuralFoil whole-aircraft
model at sea level, zero sideslip, and 10, 15, and 20 m/s. At each speed,
baseline lift at 2°, 4°, and 6° defines three targets. Candidate angle of attack
is solved to match each dimensional lift target, giving nine comparisons.
The mean is the arithmetic mean of their percentage drag changes, not an
endurance estimate. Negative drag reduction means increased predicted drag.

| Comparison, 25 body stations | Mean predicted drag change |
| --- | ---: |
| NACA versus custom | +1.596% |
| SD8020 versus custom | +1.540% |
| SD8020 versus NACA | -0.056% |

Repeating the direct SD8020/NACA comparison with 101 stations per body gives
about **0.070% lower predicted drag** for SD8020, with a minimum reduction of
about 0.061% across the nine points. The coarse result's minimum is 0.049%.
The small preference persists, but it is far below model uncertainty and
comparable to small geometry/sampling differences. It does not establish an
aerodynamic winner. NACA remains the requested baseline, with SD8020 available
as an alternative. Neither is a drag improvement over the leaner custom design
in this model; they provide a named shape basis and additional envelope volume.

The body estimate is empirical and excludes validated viscous separation,
powered propwash, actual surface finish, loaded CG, and trim/control drag.
Published 2D airfoil polars are not used as performance data for the revolved
bodies. Printing and structural adequacy have not been established.

## Files, controls, and verification

`fuselage_body_airfoil` and `motor_pod_body_airfoil` independently select the
body source. Either can be `None` to restore its custom spline. The old custom
fuselage station/fore/aft controls and pod-radius tuple only apply in that mode.
`fuselage_end_cap_radius`, `motor_pod_aft_cap_radius`, and
`motor_pod_aft_rounding` control the retained finite end features.

```powershell
uv run --locked python airframe_oml.py --force
uv run --locked python airframe.py --force
uv run --locked python motor_prop_installation.py --force
uv run --locked python propeller_envelope.py --force
uv run --locked python airframe_sd8020.py --force
uv run --locked --extra aero python compare_body_airfoils.py
uv run --locked python -m unittest discover --verbose
```

All 26 tests pass in the installed aero environment. Tests include symmetric
source validation, template envelopes, cap dimensions, motor fit, shared
CAD/aerodynamic meridians, and existing layout/solid checks. Each of the three
configurations also builds a valid single-solid OML in the comparison script.
Saved STEP checks confirm valid seven-solid NACA and SD8020 assemblies,
unchanged lifting surfaces and propeller positions, 3 mm pod end caps, and the
same mounting-plane positions. Snapshot reviews cover both alternatives and
the NACA OML/motor installation.

The JSON report, raw source identities, meridian plot, and saved STEP checks
are under `tmp/airfoil-bodies/`. The pre-change exports and source are under
`tmp/airfoil-bodies/before/`; NACA sweeps and renders are under `naca-aero/`.
The comparison sources are maintained as `compare_body_airfoils.py` and
`airframe_sd8020.py`; generated STEP files and local reports remain ignored.
