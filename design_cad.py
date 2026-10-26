"""CAD-facing 4-tractor / 4-pusher / 8-coaxial preview assemblies.

Uses the existing NeedleTail CAD solids and exact OCCT distances. Movable
trailing-edge panels are split from their parent lifting surfaces at neutral
deflection, with a small nominal clearance. They are *concept envelopes*, not
designed hinges, linkages, shafts, internal motor cavities, or flight hardware.
"""
from __future__ import annotations
from pathlib import Path

from design_configuration import Design, flap_segments, rotor_stations


def _pod_shapes(p, d):
    from cadgen import build123d as bd
    from airframe_geometry import _motor_pod
    from design_configuration import motor_positions
    pod = _motor_pod(p)
    # Rotating this axisymmetric pod 180 deg reverses its front/aft taper.
    rear_pod = bd.Rot(0, 180, 0) * pod
    centers = tuple(
        (p.motor_pod_center_x, y, z)
        for _, y, z in motor_positions(p)
    )
    out = []
    for i, pos in enumerate(centers):
        if d.topology != "pusher4":
            shape = bd.Pos(*pos) * pod
            shape.label = f"front_nacelle_{i + 1}"
            out.append(shape)
        if d.topology != "tractor4":
            shape = bd.Pos(*pos) * rear_pod
            shape.label = f"rear_nacelle_{i + 1}"
            out.append(shape)
    return out


def _split_flap_panels(p, d, wing, support, gap_mm=.35):
    from cadgen import build123d as bd
    remaining = {"horizontal":wing, "vertical":support}
    panels = []
    for name, axis, low, high in flap_segments(p, d):
        chord = (p.support_chord if axis == "vertical" else
                 p.wing_root_chord + (p.wing_tip_chord-p.wing_root_chord) *
                 abs(.5*(low+high))/(p.wing_span/2))
        # Trailing edge has negative CAD X. This plane approximates an
        # equal-fraction hinge line; tapered-wing hinge sweep is not modeled.
        x_hinge = (.5-d.hinge_fraction)*chord
        width = p.wing_root_chord + 2
        x_lo = -p.wing_root_chord
        x_mid = (x_hinge+x_lo)/2
        x_size = x_hinge-x_lo
        mid = (low+high)/2
        if axis == "horizontal":
            position = (x_mid, mid, 0.)
            boxsize = (x_size, high-low, 2*p.fuselage_max_radius)
        else:
            position = (x_mid, 0., mid)
            boxsize = (x_size, 2*p.fuselage_max_radius, high-low)
        select = bd.Pos(*position) * bd.Box(*boxsize)
        panel = remaining[axis] & select
        if not panel.solids():
            raise ValueError(f"No flap solid generated for {name}")
        # A slightly wider removal window leaves finite clearance around
        # the panel, including at the hinge and the outboard/inboard ends.
        cutter = bd.Pos(*position) * bd.Box(
            boxsize[0] + 2*gap_mm,
            boxsize[1] + (2*gap_mm if axis=="horizontal" else 0),
            boxsize[2] + (2*gap_mm if axis=="vertical" else 0),
        )
        remaining[axis] = remaining[axis] - cutter
        panel.label = f"{name}_movable_panel_neutral"
        panels.append(panel)
    return list(remaining.values()),panels


def build_variant_components(params, design: Design):
    from cadgen import build123d as bd
    from airframe_geometry import _fuselage, _wing, _vertical_support
    params.validate_motor_prop_interface()
    params.validate_arm_pod_fit()
    core = [_fuselage(params)]
    wings, panels = _split_flap_panels(params, design, _wing(params),
                                       _vertical_support(params))
    translated_surfaces = [
        bd.Pos(params.assembly_x_offset, 0, 0) * shape
        for shape in wings + panels
    ]
    return core + translated_surfaces + _pod_shapes(params, design)


def build_variant_assembly(params, design: Design):
    from cadgen import build123d as bd
    pieces=build_variant_components(params, design)
    result=bd.Compound(children=pieces)
    result.label=f"needletail_{design.topology}_{design.flaps}_concept_assembly"
    return result


def build_variant_propeller_envelopes(params, design: Design):
    from cadgen import build123d as bd
    disks=[]
    for rotor in rotor_stations(params, design):
        shape=(bd.Pos(*rotor.xyz_mm) * bd.Rot(0, 90, 0) *
               bd.Cylinder(radius=params.propeller_diameter/2,
                           height=params.propeller_envelope_thickness))
        shape.label=f"disk_{rotor.side}_{rotor.name}"
        disks.append(shape)
    return disks


def clearance_report(params, design: Design):
    """Exact OCCT minimum distances. 0 means contact/interference.

    Intentionally does not call the tractor-only clearance validator for
    pusher/coax configurations. Only designs passing ALL checks can proceed.
    """
    pieces=build_variant_components(params,design)
    disks=build_variant_propeller_envelopes(params,design)
    pod_count=4 if design.topology != "coaxial8" else 8
    protected=pieces[:-pod_count]
    pods=pieces[-pod_count:]
    # Each disk is compared with *all* pods, because an adjacent component
    # or opposite-side pod can collide even if its own pod is clear.
    report={}
    for disk in disks:
        for shape in protected+pods:
            report[disk.label+"__"+shape.label]=float(disk.distance_to(shape))
    for i in range(len(disks)):
        for j in range(i+1,len(disks)):
            report[disks[i].label+"__"+disks[j].label]=float(disks[i].distance_to(disks[j]))
    from math import isfinite
    if not all(isfinite(v) and v>=0 for v in report.values()):
        raise ValueError("Nonfinite or negative OCCT clearance")
    return report


def validate_variant(params, design: Design, min_clearance_mm=1.):
    report=clearance_report(params,design)
    failures={k:distance for k,distance in report.items() if distance<min_clearance_mm}
    if failures:
        raise ValueError(f"Invalid {design.topology}/{design.flaps} clearance: {failures}")
    return report


def export_concept_step(params, design: Design, path, *, require_clearance=True):
    from cadgen import build123d as bd
    if require_clearance:
        validate_variant(params,design,min_clearance_mm=params.propeller_clearance_margin)
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    assembly=build_variant_assembly(params,design)
    bd.export_step(assembly,str(path))
    return path
