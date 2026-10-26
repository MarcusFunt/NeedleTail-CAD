# First aerodynamic optimization pass — 2026-10-24

**Historical revision:** this 208 mm, 73/40 mm candidate has been superseded by
the shorter, thicker arms described in [arm_revision.md](arm_revision.md).
The 0.911% result below applies only to the earlier candidate. Running the
comparison script now analyzes the current geometry against the original
arm/pod layout, not the superseded candidate.

The selected change retains NACA 2415 on the horizontal wing, NACA 0015 on the
vertical support, the 230 × 60 mm fuselage envelope, and all motor/propeller
interfaces. Dimensions are millimeters.

| Quantity | Baseline | Selected |
| --- | ---: | ---: |
| Horizontal span | 200 | 208 |
| Horizontal pod center Y | ±100 | ±104 |
| Wing root chord | 72 | 73 |
| Wing tip chord | 44 | 40 |
| Gross horizontal area, mm² | 11,600 | 11,752 |
| Nominal root section depth | 10.8 | 10.95 |
| Nominal tip section depth | 6.6 | 6.0 |

## Comparison method

`compare_airframe_aero.py` compares the current geometry to the original wing
dimensions while retaining the same non-wing parameters. It uses the existing
CAD-matched AeroBuildup adapter and its large NeuralFoil model at sea level,
zero sideslip, and 10, 15, and 20 m/s. At each speed, the original wing's lift
at 2°, 4°, and 6° defines three separate targets. The script solves candidate
angle of attack within -4° to 8° at each target and compares CD × reference
area. Equal CL alone would be misleading because the wing area changes.

These targets are exploratory loads, not a supplied aircraft weight or mission.
The summary is the equally weighted arithmetic mean of nine percentage drag
reductions, not a mission energy or endurance estimate.

| Speed | Drag reduction at baseline 2° lift | At baseline 4° lift | At baseline 6° lift |
| --- | ---: | ---: | ---: |
| 10 m/s | 0.287% | 0.741% | 1.361% |
| 15 m/s | 0.289% | 0.805% | 1.098% |
| 20 m/s | 0.647% | 1.128% | 1.841% |

Mean predicted reduction is **0.911%**, with a minimum of **0.287%** across
the nine points. At 15 m/s and the original 4° lift, L/D changes from about
4.356 to 4.392. This is a small predicted trend within the uncertainty of the
model, not evidence of an experimentally measurable improvement.

## Selection and limits

The exploratory study examined root/tip taper changes, 108–118 mm pod lengths,
horizontal motor offsets up to 115 mm, a symmetric NACA 0015 wing, and a UIUC
NACA 4415 section. The symmetric wing could not match all targets within the
angle bracket, and the more cambered section increased mean predicted drag.
Pod-length changes produced negligible gains. The selected span/taper
combination improved all nine modeled points while making a smaller span
change than the broader candidates. It is a bounded local candidate, not a
global optimum. The individual screening scripts and results remain under
the ignored `tmp/aero-optimization/` directory and `tmp/screen_*.py`.

The 15% thickness ratio remains, but absolute tip depth falls by about 9%.
Neither bending strength nor stiffness has been verified. Modeled propeller
clearances remain above the existing 1 mm margin; the horizontal-wing clearance
is 24.5 mm and nearest disk edge clearance is about 58.1 mm. The pod/root-chord
ratio is about 1.301, close to the existing 1.3 lower limit.

The study excludes powered propwash, motor/prop drag beyond the existing pod
representation, CG, trim/control drag, weight, and manufacturing effects. It
does not establish stability or flight readiness. Historical SU2 startup runs
are unconverged and use earlier geometry; they do not validate this candidate.
Validated viscous CFD or measurements and actual operating loads are needed
before claiming improved performance.

## Geometry handling and outputs

The airfoil importer now splits Selig contours at minimum X, so a cambered
profile's upper and lower branches share the real nose even when its height
is nonzero. This fixes pinch points in supported profiles such as SD7037 and
E423. The active NACA 2415/0015 sections have zero-height noses, so the fix
does not change the baseline comparison. Regression tests cover the shared
nose and positive interior thickness across the local catalog.

All four STEP exports are rebuilt: `airframe.step`, `airframe_oml.step`,
`propeller_envelope.step`, and `motor_prop_installation.step`. The comparison
JSON is under `tmp/aero-optimization/`; refreshed aerodynamic sweeps and
renders are under `tmp/aero-optimization/selected-trends/`. Original STEP
exports are retained under `tmp/aero-optimization/baseline/`.
