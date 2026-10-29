"""Shared, SI-explicit design space for NeedleTail CAD / aerodynamics / optimizer.

All modes retain the current AirframeParameters as their single dimensional
source. New rotor placements and flap geometry are explicit *variants*; they
do not silently mutate the existing, validated tractor baseline.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import isfinite, sqrt

TOPOLOGIES = ("tractor4", "pusher4", "coaxial8")
FLAP_MODES = ("none", "horizontal", "all_four")
STATIONS = ("left", "right", "bottom", "top")
DEFAULT_MASS_ASSUMPTIONS_G = {
    "motor_mass_g": 9.1,
    "esc_mass_g": 2.0,
    "prop_mass_g": 1.2,
    "rear_mount_mass_g": 3.0,
    "servo_mass_g": 2.5,
}


@dataclass(frozen=True)
class Design:
    topology: str = "tractor4"
    flaps: str = "none"
    hinge_fraction: float = 0.72
    flap_start_fraction: float = 0.25
    flap_end_fraction: float = 0.90
    flap_limit_deg: float = 20.0
    coaxial_separation_mm: float | None = None

    def __post_init__(self):
        if self.topology not in TOPOLOGIES or self.flaps not in FLAP_MODES:
            raise ValueError("Invalid topology or flap mode")
        vals = (self.hinge_fraction, self.flap_start_fraction,
                self.flap_end_fraction, self.flap_limit_deg)
        if not all(isfinite(x) for x in vals):
            raise ValueError("Nonfinite control surface parameter")
        if not (0.55 <= self.hinge_fraction <= 0.90 and
                0 <= self.flap_start_fraction < self.flap_end_fraction <= 1 and
                0 < self.flap_limit_deg <= 35):
            raise ValueError("Invalid flap geometry or deflection")
        if self.coaxial_separation_mm is not None:
            if not isfinite(self.coaxial_separation_mm) or self.coaxial_separation_mm <= 0:
                raise ValueError("Invalid coaxial rotor separation")


@dataclass(frozen=True)
class RotorStation:
    name: str
    side: str
    xyz_mm: tuple[float, float, float]
    spin_sign: int
    shaft_axis_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)
    thrust_direction_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)

    def __post_init__(self):
        if isinstance(self.spin_sign, bool) or self.spin_sign not in (-1, 1):
            raise ValueError("Rotor spin sign must be -1 or +1")
        if len(self.xyz_mm) != 3 or not all(isfinite(value) for value in self.xyz_mm):
            raise ValueError("Rotor station position must contain three finite coordinates")
        for label, vector in (("shaft axis", self.shaft_axis_xyz),
                              ("thrust direction", self.thrust_direction_xyz)):
            if len(vector) != 3 or not all(isfinite(value) for value in vector):
                raise ValueError(f"Rotor {label} must contain three finite components")
            if abs(sqrt(sum(value * value for value in vector)) - 1.0) > 1e-9:
                raise ValueError(f"Rotor {label} must be a unit vector")

    @property
    def propeller_hand(self) -> str:
        """Reference STEP hand matching this +X spin sign; hardware needs verification."""
        return propeller_hand_for_spin_sign(self.spin_sign)


def propeller_hand_for_spin_sign(spin_sign: int) -> str:
    """Map signed +X rotation to the reference STEP hand label.

    Positive rotation appears clockwise from aft (-X) looking toward the nose
    (+X); this label mapping is a packaging convention pending hardware check.
    """
    if isinstance(spin_sign, bool) or spin_sign not in (-1, 1):
        raise ValueError("Rotor spin sign must be -1 or +1")
    return "CW" if spin_sign > 0 else "CCW"


def moment_about_shaft_axis(
    moment_xyz_nm: tuple[float, float, float],
    force_xyz_n: tuple[float, float, float],
    shaft_origin_xyz_m: tuple[float, float, float],
    shaft_axis_xyz: tuple[float, float, float],
) -> float:
    """Shift an origin moment to the shaft point and project onto its unit axis."""
    vectors = (moment_xyz_nm, force_xyz_n, shaft_origin_xyz_m, shaft_axis_xyz)
    if any(len(vector) != 3 or not all(isfinite(value) for value in vector)
           for vector in vectors):
        raise ValueError("Shaft torque inputs must be finite 3-vectors")
    if abs(sqrt(sum(value * value for value in shaft_axis_xyz)) - 1.0) > 1e-9:
        raise ValueError("Shaft axis must be a unit vector")
    rx, ry, rz = shaft_origin_xyz_m
    fx, fy, fz = force_xyz_n
    shifted = (
        moment_xyz_nm[0] - (ry * fz - rz * fy),
        moment_xyz_nm[1] - (rz * fx - rx * fz),
        moment_xyz_nm[2] - (rx * fy - ry * fx),
    )
    return sum(axis * moment for axis, moment in zip(shaft_axis_xyz, shifted))


def motor_positions(p):
    motor_z = getattr(p, "motor_axis_z_offset_from_pod_center", 0.0)
    return (("left", -p.horizontal_motor_offset, motor_z),
            ("right", p.horizontal_motor_offset, motor_z),
            ("bottom", 0., -p.vertical_motor_offset + motor_z),
            ("top", 0., p.vertical_motor_offset + motor_z))


def rotor_stations(p, d: Design):
    """Return rotor poses with signed angular rates about aircraft +X.

    ``spin_sign`` is the sign of the physical angular velocity around +X.
    Viewed from aircraft aft (-X) toward the nose (+X), positive is clockwise.
    The available CW/CCW STEP labels follow that view convention; the actual
    hardware rotation still needs confirmation before flight use.
    """
    pod_center_x = getattr(p, "motor_pod_center_x", p.motor_station_x)
    front_x = p.prop_hub_center_x
    rear_x = (front_x - d.coaxial_separation_mm if d.coaxial_separation_mm
              is not None and d.topology == "coaxial8"
              else 2 * pod_center_x - front_x)
    # Equal-throttle torque balance in the shared +X angular-rate convention.
    spin = {"left": 1, "right": 1, "bottom": -1, "top": -1}
    out = []
    for name, y, z in motor_positions(p):
        if d.topology != "pusher4":
            out.append(RotorStation(name, "front", (front_x, y, z), spin[name]))
        if d.topology != "tractor4":
            out.append(RotorStation(name, "rear", (rear_x, y, z),
                                    -spin[name] if d.topology == "coaxial8" else spin[name]))
    return tuple(out)


def flap_segments(p, d: Design):
    """Each panel: (name, axis, signed spanwise limits in mm).

    The physical hinge x coordinate depends on the local wing chord.
    Panel ranges are *exposed* spans measured outboard of the fuselage.
    """
    if d.flaps == "none":
        return ()
    out = []
    for axis, halfspan, names in (
        ("horizontal", p.wing_span / 2, ("left", "right")),
        ("vertical", p.support_span / 2, ("bottom", "top")),
    ):
        if axis == "vertical" and d.flaps != "all_four":
            continue
        r = p.fuselage_max_radius
        if halfspan <= r:
            raise ValueError("No exposed arm span for a flap")
        a = r + (halfspan - r) * d.flap_start_fraction
        b = r + (halfspan - r) * d.flap_end_fraction
        out += [(names[0], axis, -b, -a), (names[1], axis, a, b)]
    return tuple(out)


def mass_breakdown_g(
    d: Design,
    *,
    motor_g=DEFAULT_MASS_ASSUMPTIONS_G["motor_mass_g"],
    esc_g=DEFAULT_MASS_ASSUMPTIONS_G["esc_mass_g"],
    prop_g=DEFAULT_MASS_ASSUMPTIONS_G["prop_mass_g"],
    rear_mount_g=DEFAULT_MASS_ASSUMPTIONS_G["rear_mount_mass_g"],
    flap_actuator_g=DEFAULT_MASS_ASSUMPTIONS_G["servo_mass_g"],
):
    """Return assumed added hardware masses vs the 4-tractor/no-flap baseline."""
    extra_rotors = 4 if d.topology == "coaxial8" else 0
    n_flaps = len(("left", "right")) if d.flaps == "horizontal" else (
        4 if d.flaps == "all_four" else 0)
    breakdown = {
        "extra_motors_g": extra_rotors * motor_g,
        "extra_escs_g": extra_rotors * esc_g,
        "extra_props_g": extra_rotors * prop_g,
        "rear_mounts_g": extra_rotors * rear_mount_g,
        "flap_actuators_g": n_flaps * flap_actuator_g,
    }
    breakdown["total_added_g"] = sum(breakdown.values())
    return breakdown


def mass_penalty_g(
    d: Design,
    *,
    motor_g=DEFAULT_MASS_ASSUMPTIONS_G["motor_mass_g"],
    esc_g=DEFAULT_MASS_ASSUMPTIONS_G["esc_mass_g"],
    prop_g=DEFAULT_MASS_ASSUMPTIONS_G["prop_mass_g"],
    rear_mount_g=DEFAULT_MASS_ASSUMPTIONS_G["rear_mount_mass_g"],
    flap_actuator_g=DEFAULT_MASS_ASSUMPTIONS_G["servo_mass_g"],
):
    """Added mass vs 4-tractor/no flaps, not a complete weight estimate."""
    return mass_breakdown_g(
        d,
        motor_g=motor_g,
        esc_g=esc_g,
        prop_g=prop_g,
        rear_mount_g=rear_mount_g,
        flap_actuator_g=flap_actuator_g,
    )["total_added_g"]


def configuration_name(d: Design):
    gap = "" if d.topology != "coaxial8" else (
        f"-gap{d.coaxial_separation_mm:g}" if d.coaxial_separation_mm is not None else "")
    return f"{d.topology}-{d.flaps}{gap}"
