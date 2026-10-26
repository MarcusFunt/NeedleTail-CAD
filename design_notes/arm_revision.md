# Shorter, thicker arms — 2026-10-24

**Subsequent pod placement:** the arm dimensions below remain current, but
all four pods and their motor/propeller interfaces have since moved 5 mm aft.
Current X stations and clearance targets are in [pod_placement.md](pod_placement.md).
The X=6/50/68.5 positions and 28/36 mm disk clearances below describe the
initial thicker-arm revision before that translation.

This revision prioritizes deeper arm sections and shorter spans over the
previous aerodynamic-only candidate. The requested process is FDM with a
0.4 mm nozzle; the material, layer height, extrusion widths, and print
orientation remain unspecified. All dimensions below are millimeters.

| Dimension | Previous | Current |
| --- | ---: | ---: |
| Horizontal span between pod axes | 208 | 190 |
| Vertical span between pod axes | 170 | 160 |
| Wing root chord | 73 | 80 |
| Wing tip chord | 40 | 56 |
| Vertical-arm chord | 48 | 64 |
| Nominal airfoil thickness/chord | 15% | 18% |
| Physical trailing-edge thickness | 0.30 | 1.20 |

Both source airfoils retain their camber lines. Thickness is multiplied by
1.2 about those lines, then the aft 6% of the chord blends to the physical
trailing edge. The resulting sections are derivatives of NACA 2415 and
NACA 0015, not the unmodified catalog profiles. The same transformed
coordinates are used by CAD, section polars, and the whole-aircraft adapter.

## Geometry evidence

Sections taken through the saved `airframe.step` give these bounding depths
and trailing-edge line lengths. Bounding depth is the total envelope normal
to the section plane; it is not a minimum wall-thickness measurement.

| Section | Measured chord | Measured depth | Trailing edge |
| --- | ---: | ---: | ---: |
| Wing root, Y=0 | 80.00 | 14.47 | 1.20 |
| Wing tip, Y=95 | 56.00 | 10.13 | 1.20 |
| Vertical arm, Z=80 | 64.00 | 11.52 | 1.20 |

The horizontal pod axes move from Y=±104 to ±95; vertical axes move from
Z=±85 to ±80. All pod centers move from X=3 to X=6. To accommodate the wider
sections while retaining the existing clearance requirements, the untrimmed
pod profile length becomes 124 and its motor face offset becomes 44. The
remaining plastic pod is 106 long, for a pod/root-chord ratio of 1.325
(existing minimum 1.3). Motor case diameter and the shaft/hub interface
dimensions are retained.

The motor mount is now at aircraft X=50, giving 22 clearance ahead of the
horizontal tip leading edge and 18 ahead of the vertical leading edge.
Propeller disks are at X=68.5. Conservative modeled clearances are 28 to the
horizontal wing, 36 to the vertical arm, 11.9 to the fuselage envelope, and
about 48 between adjacent disk edges. These exceed the existing 1 mm disk
clearance margin. The motor face remains sized for the reference motor.

All four STEP exports rebuild. Re-import checks confirm a valid single-solid
OML, seven-solid engineering assembly, four disks, and nine-solid motor/prop
reference. The expanded suite has 18 passing tests, including thickness
scaling, preserved camber, exact trailing-edge thickness, and matching
CAD/aerodynamic section coordinates. Saved snapshots cover all four exports.

The previous STEP files and source are retained in `tmp/sturdier-arms/before/`.
Saved-STEP section measurements are in `tmp/sturdier-arms/cad_sections.json`.
A one-off STL export is available at `tmp/sturdier-arms/airframe_oml.stl`.

## Print and structural limits

The 1.2 mm trailing edge removes the previous 0.30 mm edge from the active
arms. It is a design target, not a claim of three printed perimeters: nozzle
diameter and extrusion width differ, and the slicer accounts for overlapping
extrusions. See [Prusa's layer/perimeter guidance](https://help.prusa3d.com/article/layers-and-perimeters_1748).

These are still solid outer-envelope CAD bodies. No hollow shell thickness,
spar, internal ribbing, build joints, or motor attachment holes have been
designed. The deeper profiles and shorter exposed arms do not establish
strength or stiffness without material, print orientation, infill/perimeter
settings, thrust/landing loads, and an appropriate structural check.

No slicer or full mesh DfAM audit was run. The optional mesh-audit runtime
lacks `trimesh` and `rtree`; no dependencies were added. The CAD sections above
measure envelope depth and edge geometry directly, and do not replace a
wall-thickness, overhang, or support audit. A one-piece cruciform print may
need supports or a split-part design, depending on orientation.

No aerodynamic improvement is claimed for this revision. The earlier 0.911%
drag result applies to the previous geometry. New unpowered trends/polars
are generated from the current thicker sections; existing CFD meshes and
powered-flow results must be regenerated before use.

Running the equal-lift comparison against the original 200/170 mm span,
72/44/48 mm chord, 15%-thick, 0.30 mm-edge arm/pod geometry predicts a mean
**14.34% drag increase** across the nine exploratory load points (individual
increases about 8.37–28.67%). This is a model estimate, not validated measured
performance. The output is `tmp/sturdier-arms/comparison/comparison.json`.
The comparison uses the original slimmer baseline, not the immediately
preceding 208 mm aerodynamic candidate.
