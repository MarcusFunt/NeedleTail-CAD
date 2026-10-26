"""Optional AeroSandbox geometry adapter for 0/2/4 independent trailing-edge flaps.

Pusher nacelles are mirrored and coaxial nacelles are represented as a single
symmetrized fairing, avoiding double-counting overlapping fuselages. This is an
UNPOWERED aero model: no rotor wake, thrust or transition extrapolation.
"""
from __future__ import annotations

from design_configuration import Design, flap_segments, motor_positions


def _section_wing(params, axis, side, d, asb, aero):
    halfspan = params.wing_span / 2 if axis == "horizontal" else params.support_span / 2
    radius = params.fuselage_max_radius
    a = radius + (halfspan - radius) * d.flap_start_fraction
    b = radius + (halfspan - radius) * d.flap_end_fraction
    if axis == "horizontal":
        chord_at = lambda v: params.wing_root_chord + (params.wing_tip_chord - params.wing_root_chord) * abs(v) / halfspan
        airfoil_name, te = params.wing_airfoil, params.wing_trailing_edge
    else:
        chord_at = lambda v: params.support_chord
        airfoil_name, te = params.support_airfoil, params.support_trailing_edge
    label = {"horizontal": ("left", "right"), "vertical": ("bottom", "top")}[axis][side > 0]
    tag = label
    if side < 0:
        positions = [-halfspan, -b, -a, 0.]
        active_start_index = 1
    else:
        positions = [0., a, b, halfspan]
        active_start_index = 1
    sections = []
    for index, s in enumerate(positions):
        chord = chord_at(s)
        foil = aero._airfoil_for_section(f"{tag}_{index}", airfoil_name, chord, te)
        if axis == "vertical":
            foil = aero._vertical_airfoil(foil, f"{tag}_{index}_vertical")
        cs = ([asb.ControlSurface(
            name=tag, hinge_point=d.hinge_fraction, trailing_edge=True,
            symmetric=True, deflection=0.0,
        )] if index == active_start_index else [])
        xyz = [aero._asb_x(chord / 2), s / 1000., 0.] if axis == "horizontal" else [
            aero._asb_x(chord / 2), 0., s / 1000.]
        sections.append(asb.WingXSec(
            xyz_le=xyz, chord=chord / 1000., airfoil=foil, control_surfaces=cs,
        ))
    return asb.Wing(name=f"NeedleTail_{tag}_flap", symmetric=False, xsecs=sections)


def _nacelle_profile(params, topology):
    import numpy as np
    from whole_aircraft_aero import _pod_profile
    original = list(_pod_profile(params))
    if topology == "tractor4":
        return original
    if topology == "pusher4":
        return [(-x, r) for x, r in reversed(original)]
    # Two pods share one coaxial axis. Represent their *union-like envelope*
    # as ONE fairing, not two overlapping aerodynamic fuselages.
    xx = np.linspace(original[0][0], original[-1][0], 35)
    ox, rr = np.asarray(original).T
    return [(float(x), float(max(np.interp(x, ox, rr),
                                 np.interp(-x, ox, rr)))) for x in xx]


def build_variant_airplane(params, design: Design):
    """Return an AeroSandbox Airplane using the same CAD parameter source.

    Its control deflections may be applied with airplane.with_control_deflections
    or by rebuilding/evaluating as appropriate for the installed ASB release.
    """
    import aerosandbox as asb
    import whole_aircraft_aero as aero
    original = aero.build_airplane(params)
    wings = []
    if design.flaps == "none":
        wings = original.wings
    else:
        wings = [_section_wing(params, "horizontal", sign, design, asb, aero)
                 for sign in (-1, 1)]
        if design.flaps == "all_four":
            wings += [_section_wing(params, "vertical", sign, design, asb, aero)
                      for sign in (-1, 1)]
        else:
            wings += [original.wings[1]]
    if design.topology == "tractor4":
        nacelles = original.fuselages
    else:
        profile = tuple(_nacelle_profile(params, design.topology))
        nacelles = [original.fuselages[0]]
        for name, y, z in motor_positions(params):
            nacelles.append(aero._make_fuselage(
                f"{design.topology}_{name}_nacelle", profile,
                center_x_mm=params.motor_station_x,
                center_y_m=y / 1000, center_z_m=z / 1000,
            ))
    return asb.Airplane(
        name=f"NeedleTail_{design.topology}_{design.flaps}",
        wings=wings, fuselages=nacelles, xyz_ref=original.xyz_ref,
        s_ref=original.s_ref, c_ref=original.c_ref, b_ref=original.b_ref,
    )


def flap_derivatives(params, design: Design, speed_mps=30., alpha_deg=3.,
                     delta_deg=2., cg_xyz_asb_m=(0., 0., 0.)):
    """Finite-difference unpowered roll/pitch/yaw moments in Nm/rad, ASB axes.

    This is attached-flow screening only. No wake, hinge moment, post-stall
    capability, or motor thrust map is represented; do not use at hover.
    Convert the returned 3xN matrix before using it in CAD body axes.
    """
    import aerosandbox as asb
    import numpy as np
    if speed_mps <= 0 or delta_deg <= 0:
        raise ValueError("Positive operating speed and perturbation required")
    aircraft = build_variant_airplane(params, design)
    names = list(dict.fromkeys(seg[0] for seg in flap_segments(params, design)))
    op = asb.OperatingPoint(velocity=speed_mps, alpha=alpha_deg, beta=0.,
                           atmosphere=asb.Atmosphere(altitude=0))
    def evaluate(deflections):
        model = aircraft.with_control_deflections(deflections)
        return asb.AeroBuildup(model, op, xyz_ref=cg_xyz_asb_m).run()
    def coefficient(result, name):
        values = np.asarray(result[name], dtype=float).reshape(-1)
        if values.size != 1:
            raise ValueError(f"Expected one coefficient for scalar operating point; got {values.size}")
        return float(values[0])
    out = np.zeros((3, len(names)))
    q = .5 * float(op.atmosphere.density()) * speed_mps**2
    for k, name in enumerate(names):
        plus = evaluate({name:delta_deg})
        minus = evaluate({name:-delta_deg})
        grad = 2 * np.deg2rad(delta_deg)
        coefficient_delta = np.asarray([
            coefficient(plus, "Cl") - coefficient(minus, "Cl"),
            coefficient(plus, "Cm") - coefficient(minus, "Cm"),
            coefficient(plus, "Cn") - coefficient(minus, "Cn"),
        ])
        moment_scales = np.asarray([
            aircraft.s_ref * aircraft.b_ref,
            aircraft.s_ref * aircraft.c_ref,
            aircraft.s_ref * aircraft.b_ref,
        ])
        out[:, k] = q * moment_scales * coefficient_delta / grad
    return names, out
