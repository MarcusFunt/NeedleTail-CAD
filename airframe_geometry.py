from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from functools import lru_cache
from math import cos, isfinite, pi, sin, sqrt

from cadgen import build123d as bd

from profiles.airfoil_coordinates import AIRFOIL_COORDINATES


@dataclass(frozen=True)
class AirframeParameters:
    """First-pass geometry parameters in millimeters."""

    fuselage_length: float = 170.0
    fuselage_max_radius: float = 21.0
    # Named symmetric airfoil meridians; None selects the historical custom spline.
    fuselage_body_airfoil: str | None = None
    fuselage_end_cap_radius: float = 1.8
    fuselage_max_station_from_nose: float = 0.38
    fuselage_aft_radius_profile: tuple[tuple[float, float], ...] = (
        (0.00, 0.06), (0.05, 0.12), (0.18, 0.27), (0.40, 0.50),
        (0.63, 0.76), (0.84, 0.93), (1.00, 1.00),
    )
    fuselage_fore_radius_profile: tuple[tuple[float, float], ...] = (
        (0.00, 1.00), (0.20, 0.99), (0.42, 0.90), (0.64, 0.69),
        (0.82, 0.39), (0.95, 0.13), (1.00, 0.06),
    )
    wing_root_chord: float = 60.0
    wing_tip_chord: float = 35.0
    wing_airfoil: str = "SD7003"
    wing_thickness_scale: float = 1.0
    wing_trailing_edge: float = 0.30
    support_chord: float = 38.0
    support_airfoil: str = "SD8020"
    support_thickness_scale: float = 1.0
    support_trailing_edge: float = 0.30
    # Shared translation of both lifting surfaces and all pod/motor/prop axes.
    # The fuselage and the aircraft reference origin stay fixed.
    assembly_x_offset: float = 0.0
    # Local pod station relative to the translated arm intersection.
    motor_station_x: float = 0.0
    horizontal_motor_offset: float = 100.0
    vertical_motor_offset: float = 85.0
    # Preserve PR #3's arm-to-pod overlap span by default. Candidate layouts may
    # explicitly terminate lifting surfaces on pod centerlines instead.
    arm_pod_overlap: float = 1.0
    span_to_pod_centerlines: bool = False
    propeller_diameter: float = 76.2
    propeller_envelope_thickness: float = 1.0
    propeller_clearance_margin: float = 1.0
    motor_pod_length: float = 42.0
    motor_pod_diameter: float = 24.0
    motor_pod_body_airfoil: str | None = None
    motor_pod_aft_cap_radius: float = 1.5
    motor_pod_aft_rounding: float = 1.0
    # Small radial allowance at the motor mounting face.
    motor_pod_profile_radius_scale: float = 1.0
    motor_body_diameter: float = 19.9
    motor_body_length: float = 13.5
    motor_body_radius_profile: tuple[tuple[float, float], ...] = (
        (0.0, 8.75), (3.8, 9.95), (11.4, 9.95), (13.5, 8.4),
    )
    motor_pod_radial_allowance: float = 1.5
    motor_axis_z_offset_from_pod_center: float = 0.0
    motor_mount_face_offset_from_pod_center: float = 4.5
    # Provisional layout bounds for the requested 5 mm aft pod translation.
    # Swept-disk collision clearance remains an independent 1 mm constraint.
    minimum_motor_mount_forward_of_wing_leading_edge: float | None = None
    minimum_motor_mount_forward_of_support_leading_edge: float | None = None
    minimum_motor_pod_to_root_chord_ratio: float | None = None
    motor_shaft_tip_from_mount_face: float = 18.4
    prop_hub_center_from_mount_face: float = 18.5
    prop_hub_thickness: float = 4.0
    motor_pod_radius_profile: tuple[tuple[float, float], ...] = (
        (-0.50, 0.05), (-0.42, 0.24), (-0.26, 0.58), (-0.05, 0.84),
        (0.12, 1.00), (0.26, 1.00), (0.38, 0.96), (0.43, 0.83),
        (0.46, 0.58), (0.49, 0.16), (0.50, 0.05),
    )

    @property
    def wing_span(self) -> float:
        """Full horizontal span [mm], to the pod center or its overlap inset."""
        if self.span_to_pod_centerlines:
            return 2 * self.horizontal_motor_offset
        return 2 * (
            self.horizontal_motor_offset
            - self.motor_pod_diameter / 2
            + self.arm_pod_overlap
        )

    @property
    def support_span(self) -> float:
        """Full support span [mm], to the pod center or its overlap inset."""
        if self.span_to_pod_centerlines:
            return 2 * self.vertical_motor_offset
        return 2 * (
            self.vertical_motor_offset
            - self.motor_pod_diameter / 2
            + self.arm_pod_overlap
        )

    @property
    def wing_planform_area(self) -> float:
        """Gross trapezoidal planform area in mm², assuming linear loft taper."""
        return self.wing_span * (self.wing_root_chord + self.wing_tip_chord) / 2

    @property
    def support_planform_area(self) -> float:
        """Gross rectangular support planform area in mm²."""
        return self.support_span * self.support_chord

    def validate_arm_pod_fit(self):
        if self.motor_pod_diameter <= 0:
            raise ValueError("Motor-pod diameter must be positive")
        if not self.span_to_pod_centerlines and not (
            0 < self.arm_pod_overlap < self.motor_pod_diameter / 2
        ):
            raise ValueError("Arm-to-pod overlap must be positive and less than the pod radius")
        if self.wing_span <= 0 or self.support_span <= 0:
            raise ValueError("Motor offsets must leave positive lifting-surface spans")

    @property
    def propeller_clearance_report(self) -> dict[str, float]:
        """Exact OCCT solid-to-solid clearances for the true swept disks [mm]."""
        return dict(_cached_exact_propeller_clearances(self))

    def validate_propeller_clearance(self):
        if self.propeller_diameter <= 0 or self.propeller_envelope_thickness <= 0:
            raise ValueError("Propeller diameter and envelope thickness must be positive")
        if self.propeller_clearance_margin < 0:
            raise ValueError("Propeller clearance margin must be nonnegative")
        if self.wing_root_chord <= self.wing_tip_chord:
            raise ValueError("Horizontal wing root chord must exceed its tip chord")
        clearances = self.propeller_clearance_report
        inadequate = {
            feature: clearance
            for feature, clearance in clearances.items()
            if clearance < self.propeller_clearance_margin
        }
        if inadequate:
            detail = ", ".join(
                f"{feature}={clearance:.2f} mm"
                for feature, clearance in inadequate.items()
            )
            raise ValueError(
                f"Propeller clearance is below the {self.propeller_clearance_margin:.2f} mm "
                f"minimum: {detail}"
            )

    @property
    def motor_pod_center_x(self) -> float:
        """Aircraft X coordinate of every pod axis origin [mm]."""
        return self.assembly_x_offset + self.motor_station_x

    @property
    def pod_nose_x(self) -> float:
        """Historical +X end station of the pod profile [mm]."""
        if self.motor_pod_body_airfoil is not None:
            return self.motor_mount_face_x
        return self.motor_pod_center_x + self.motor_pod_length / 2

    @property
    def motor_mount_face_x(self) -> float:
        return self.motor_pod_center_x + self.motor_mount_face_offset_from_pod_center

    @property
    def wing_tip_leading_edge_x(self) -> float:
        """Forward X coordinate of the horizontal wing leading edge at pod span."""
        return self.assembly_x_offset + self.wing_tip_chord / 2

    @property
    def motor_mount_forward_of_wing_leading_edge(self) -> float:
        """How far the flat motor mount face sits ahead of the wing leading edge [mm]."""
        return self.motor_mount_face_x - self.wing_tip_leading_edge_x

    @property
    def motor_mount_forward_of_support_leading_edge(self) -> float:
        """How far the flat motor mount face sits ahead of the support leading edge [mm]."""
        return self.motor_mount_face_x - (self.assembly_x_offset + self.support_chord / 2)

    @property
    def motor_pod_plastic_length(self) -> float:
        """Pod length from its aft profile end to the forward motor mount face [mm]."""
        return self.motor_pod_length / 2 + self.motor_mount_face_offset_from_pod_center

    @property
    def motor_pod_to_root_chord_ratio(self) -> float:
        return self.motor_pod_plastic_length / self.wing_root_chord

    @property
    def prop_hub_center_x(self) -> float:
        return self.motor_mount_face_x + self.prop_hub_center_from_mount_face

    @property
    def propeller_mount_stations(self) -> tuple[tuple[float, float], ...]:
        """Propeller centers in the aircraft YZ plane, following the pod centerlines."""
        motor_z = self.motor_axis_z_offset_from_pod_center
        return (
            (-self.horizontal_motor_offset, motor_z),
            (self.horizontal_motor_offset, motor_z),
            (0.0, -self.vertical_motor_offset + motor_z),
            (0.0, self.vertical_motor_offset + motor_z),
        )

    @property
    def prop_hub_rear_face_x(self) -> float:
        return self.prop_hub_center_x - self.prop_hub_thickness / 2

    @property
    def prop_hub_front_face_x(self) -> float:
        return self.prop_hub_center_x + self.prop_hub_thickness / 2

    @property
    def motor_shaft_tip_x(self) -> float:
        return self.motor_mount_face_x + self.motor_shaft_tip_from_mount_face

    @property
    def motor_shaft_hub_engagement(self) -> float:
        return min(
            self.motor_shaft_tip_x - self.prop_hub_rear_face_x,
            self.prop_hub_thickness,
        )

    def validate_motor_prop_interface(self):
        if not isfinite(self.assembly_x_offset):
            raise ValueError("Assembly X offset must be finite")
        if self.motor_pod_profile_radius_scale <= 0:
            raise ValueError("Motor-pod profile radius scale must be positive")
        if not -self.motor_pod_length / 2 < self.motor_mount_face_offset_from_pod_center < self.motor_pod_length / 2:
            raise ValueError("The motor mount face must lie within the motor-pod profile")
        pod_stations = motor_pod_radius_stations(self)
        pod_radius_profile = _sample_spline_radius_profile(pod_stations)
        if self.motor_pod_body_airfoil is None:
            motor_aft_x = self.motor_mount_face_x
            motor_forward_x = motor_aft_x + self.motor_body_length
            pod_aft_x = self.motor_pod_center_x - self.motor_pod_length / 2
            if motor_aft_x < pod_aft_x or motor_forward_x > self.pod_nose_x:
                raise ValueError("The motor body must fit within the motor-pod axial envelope")
            if self.motor_body_diameter > self.motor_pod_diameter:
                raise ValueError("The motor body diameter exceeds the motor-pod outside diameter")
            motor_sample_xs = set()
            for (x0, _), (x1, _) in zip(
                self.motor_body_radius_profile,
                self.motor_body_radius_profile[1:],
            ):
                motor_sample_xs.update(
                    x0 + (x1 - x0) * index / 16 for index in range(17)
                )
            for motor_x in motor_sample_xs:
                pod_x = self.motor_mount_face_offset_from_pod_center + motor_x
                pod_radius = _interpolate(pod_radius_profile, pod_x)
                motor_radius = _interpolate(self.motor_body_radius_profile, motor_x)
                if pod_radius < motor_radius + self.motor_pod_radial_allowance - 0.01:
                    raise ValueError(
                        "The motor-pod profile does not leave the required radial allowance "
                        f"at X={pod_x:.3f} mm"
                    )
        else:
            for bound, actual, description in (
                (
                    self.minimum_motor_mount_forward_of_wing_leading_edge,
                    self.motor_mount_forward_of_wing_leading_edge,
                    "horizontal wing leading edge",
                ),
                (
                    self.minimum_motor_mount_forward_of_support_leading_edge,
                    self.motor_mount_forward_of_support_leading_edge,
                    "vertical support leading edge",
                ),
            ):
                if bound is not None and actual < bound:
                    raise ValueError(
                        f"The motor mount must be at least {bound:.2f} mm ahead of "
                        f"the {description}"
                    )
            if (
                self.minimum_motor_pod_to_root_chord_ratio is not None
                and self.motor_pod_to_root_chord_ratio
                < self.minimum_motor_pod_to_root_chord_ratio
            ):
                raise ValueError(
                    "The plastic motor pod must be at least "
                    f"{self.minimum_motor_pod_to_root_chord_ratio:.2f} times the wing root chord"
                )
            mount_radius = _interpolate(
                pod_radius_profile,
                self.motor_mount_face_offset_from_pod_center,
            )
            if mount_radius * 2 < self.motor_body_diameter - 0.02:
                raise ValueError(
                    "The flat motor mount face is smaller than the motor case diameter "
                    f"({mount_radius * 2:.2f} mm < {self.motor_body_diameter:.2f} mm)"
                )
        if self.prop_hub_rear_face_x < self.pod_nose_x - 1e-9:
            raise ValueError("The propeller hub overlaps the motor-pod nose")
        if not (
            self.prop_hub_rear_face_x
            <= self.motor_shaft_tip_x
            <= self.prop_hub_front_face_x
        ):
            raise ValueError("The modeled motor shaft tip must enter the propeller hub")


AIRFOILS = AIRFOIL_COORDINATES
PARAMETERS = AirframeParameters()
AIRFOIL_CANDIDATE_PARAMETERS = AirframeParameters(
    fuselage_length=230.0,
    fuselage_max_radius=30.0,
    fuselage_body_airfoil="NACA0015",
    fuselage_end_cap_radius=1.8,
    fuselage_max_station_from_nose=0.37759005,
    fuselage_aft_radius_profile=(
        (0.00, 0.06), (0.05, 0.12), (0.18, 0.26823659), (0.40, 0.49320856),
        (0.63, 0.76128161), (0.84, 0.93), (1.00, 1.00),
    ),
    fuselage_fore_radius_profile=(
        (0.00, 1.00), (0.20, 0.99), (0.42, 0.93767258), (0.64, 0.66977980),
        (0.82, 0.33), (0.95, 0.13), (1.00, 0.06),
    ),
    wing_root_chord=80.0,
    wing_tip_chord=56.0,
    wing_airfoil="NACA2415",
    wing_thickness_scale=1.2,
    wing_trailing_edge=1.2,
    support_chord=64.0,
    support_airfoil="NACA0015",
    support_thickness_scale=1.2,
    support_trailing_edge=1.2,
    assembly_x_offset=-5.0,
    motor_station_x=1.0,
    horizontal_motor_offset=95.0,
    vertical_motor_offset=80.0,
    span_to_pod_centerlines=True,
    motor_pod_length=124.0,
    motor_pod_diameter=19.9,
    motor_pod_body_airfoil="NACA0015",
    motor_pod_aft_cap_radius=1.5,
    motor_pod_aft_rounding=1.0,
    motor_pod_profile_radius_scale=0.9998,
    motor_mount_face_offset_from_pod_center=44.0,
    minimum_motor_mount_forward_of_wing_leading_edge=17.0,
    minimum_motor_mount_forward_of_support_leading_edge=13.0,
    minimum_motor_pod_to_root_chord_ratio=1.3,
    motor_pod_radius_profile=(
        (-0.50, 0.15), (-0.49, 0.23), (-0.47, 0.29), (-0.44, 0.34),
        (-0.40, 0.38), (-0.30, 0.46), (-0.20, 0.56), (-0.10, 0.67),
        (0.00, 0.78), (0.10, 0.90), (0.20, 0.97), (0.28, 0.995),
        (0.34, 1.00), (0.40, 1.00), (0.44, 0.95), (0.48, 0.55),
        (0.50, 0.05),
    ),
)


GEOMETRY_CONFIGURATIONS = {
    "pr3_baseline": PARAMETERS,
    "airfoil_candidate": AIRFOIL_CANDIDATE_PARAMETERS,
}


def airframe_parameters_for_geometry(name: str) -> AirframeParameters:
    """Return a named airframe geometry without changing the PR #3 default."""
    try:
        return GEOMETRY_CONFIGURATIONS[name]
    except KeyError as ex:
        choices = ", ".join(GEOMETRY_CONFIGURATIONS)
        raise ValueError(
            f"Unknown geometry configuration {name!r}; choose from: {choices}"
        ) from ex


def _one_solid(shape, label: str):
    solids = shape.solids()
    if len(solids) != 1:
        raise ValueError(f"{label} must be one solid; got {len(solids)}")
    solid = solids[0]
    solid.label = label
    return solid


@lru_cache(maxsize=16)
def _sample_spline_radius_profile(
    stations: tuple[tuple[float, float], ...],
    sample_count: int = 2048,
) -> tuple[tuple[float, float], ...]:
    edge = _radius_profile_edge(stations)
    points = edge.positions([index / sample_count for index in range(sample_count + 1)])
    profile = tuple((float(point.X), float(point.Y)) for point in points)
    if any(x0 >= x1 for (x0, _), (x1, _) in zip(profile, profile[1:])):
        raise ValueError("Generated radius spline must progress monotonically along X")
    return profile


def _radius_profile_edge(stations):
    return bd.Edge.make_spline([(x, radius, 0) for x, radius in stations])


def _revolve_radius_profile(stations, label: str):
    if len(stations) < 4:
        raise ValueError(f"{label} needs at least four radius stations")
    if any(stations[i][0] >= stations[i + 1][0] for i in range(len(stations) - 1)):
        raise ValueError(f"{label} x stations must be strictly increasing")
    if min(radius for _, radius in stations) <= 0:
        raise ValueError(f"{label} must use finite-radius end caps, not cusps")

    outer_edge = _radius_profile_edge(stations)
    aft_x, aft_radius = stations[0]
    nose_x, nose_radius = stations[-1]
    nose_cap = bd.Edge.make_line((nose_x, nose_radius, 0), (nose_x, 0, 0))
    axis_edge = bd.Edge.make_line((nose_x, 0, 0), (aft_x, 0, 0))
    aft_cap = bd.Edge.make_line((aft_x, 0, 0), (aft_x, aft_radius, 0))
    wires = bd.Wire.combine([outer_edge, nose_cap, axis_edge, aft_cap])
    if len(wires) != 1:
        raise ValueError(f"Could not close the {label} revolve profile")
    body = bd.revolve(bd.Face(wires[0]), axis=bd.Axis.X, revolution_arc=360)
    return _one_solid(body, label)


def fuselage_radius_stations(params: AirframeParameters) -> tuple[tuple[float, float], ...]:
    """Canonical local meridian shared by CAD and aerodynamic geometry."""
    if params.fuselage_body_airfoil is not None:
        source = _body_airfoil_half_profile(params.fuselage_body_airfoil)
        radius, length = params.fuselage_max_radius, params.fuselage_length
        cap = params.fuselage_end_cap_radius
        if length <= 0 or not 0 < cap < radius:
            raise ValueError("Fuselage length and end-cap radius must fit the body envelope")
        peak = max(range(len(source)), key=lambda i: source[i][1])
        nose_cut = _radius_crossing(source[:peak + 1], cap / radius)
        tail_cut = _radius_crossing(source[peak:], cap / radius)
        points = [(nose_cut, cap / radius)]
        points.extend((x, r) for x, r in source if nose_cut < x < tail_cut)
        points.append((tail_cut, cap / radius))
        return tuple(reversed([
            (length / 2 - (x - nose_cut) / (tail_cut - nose_cut) * length, r * radius)
            for x, r in points
        ]))
    half_length = params.fuselage_length / 2
    aft_x = -half_length
    nose_x = half_length
    x_max = nose_x - params.fuselage_max_station_from_nose * params.fuselage_length
    radius = params.fuselage_max_radius
    aft = [
        (aft_x + fraction * (x_max - aft_x), ratio * radius)
        for fraction, ratio in params.fuselage_aft_radius_profile
    ]
    fore = [
        (x_max + fraction * (nose_x - x_max), ratio * radius)
        for fraction, ratio in params.fuselage_fore_radius_profile[1:]
    ]
    return tuple(aft + fore)


def _fuselage(params: AirframeParameters):
    return _revolve_radius_profile(fuselage_radius_stations(params), "fuselage")


def _motor_pod(params: AirframeParameters):
    stations = motor_pod_radius_stations(params)
    # The aft nacelle profile points toward -X; the +X end is cut at the
    # configured motor interface, ahead of the wing leading edge.
    pod = _revolve_radius_profile(stations, "motor_pod")
    if params.motor_pod_body_airfoil is not None:
        return pod  # This meridian already ends on the flat motor mounting face.
    return pod


def build_motor_pod(params: AirframeParameters = PARAMETERS):
    """Build one pod with its flat +X motor mounting face."""
    params.validate_motor_prop_interface()
    return _motor_pod(params)


def _split_airfoil(coordinates):
    raw_points = []
    for x, y in coordinates:
        point = (float(x), float(y))
        if not all(isfinite(value) for value in point):
            raise ValueError("Airfoil coordinates must be finite numbers")
        if point[0] < -1e-3 or point[0] > 1.0 + 1e-3:
            raise ValueError("Airfoil chord coordinates must lie between 0 and 1")
        if not raw_points or abs(point[0] - raw_points[-1][0]) + abs(point[1] - raw_points[-1][1]) > 1e-12:
            raw_points.append(point)

    if len(raw_points) < 4:
        raise ValueError("Airfoil data must contain at least four distinct points")
    # Selig contours turn at minimum X. Cambered profiles need not have y=0
    # there; splitting at a y crossing can introduce an interior pinch point.
    nose_index = min(range(len(raw_points)), key=lambda i: raw_points[i][0])
    if nose_index == 0 or nose_index == len(raw_points) - 1:
        raise ValueError("Airfoil data must run from upper trailing edge through the nose")
    upper = raw_points[:nose_index + 1]
    lower = raw_points[nose_index:]

    upper = sorted(upper, key=lambda point: point[0])
    lower = sorted(lower, key=lambda point: point[0])
    x_min = min(upper[0][0], lower[0][0])
    x_max = max(upper[-1][0], lower[-1][0])
    if x_max - x_min <= 1e-12:
        raise ValueError("Airfoil coordinates must cover a nonzero chord")
    chord = x_max - x_min
    upper = [((x - x_min) / chord, y) for x, y in upper]
    lower = [((x - x_min) / chord, y) for x, y in lower]
    return upper, lower


def _interpolate(points, x: float) -> float:
    xs = [point[0] for point in points]
    i = bisect_right(xs, x)
    if i <= 0:
        return points[0][1]
    if i >= len(points):
        return points[-1][1]
    x0, y0 = points[i - 1]
    x1, y1 = points[i]
    if abs(x1 - x0) < 1e-14:
        return 0.5 * (y0 + y1)
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


@lru_cache(maxsize=16)
def _body_airfoil_half_profile(name: str) -> tuple[tuple[float, float], ...]:
    """Return the symmetric catalog half-thickness, normalized to unit radius."""
    if name not in AIRFOILS:
        raise ValueError(f"Unknown body airfoil {name!r}")
    upper, lower = _split_airfoil(AIRFOILS[name])
    xs = sorted({x for x, _ in upper + lower})
    radii = []
    for x in xs:
        u, l = _interpolate(upper, x), _interpolate(lower, x)
        if abs(u + l) > 1e-4:
            raise ValueError("Body airfoil must be symmetric about its chord line")
        radii.append((u - l) / 2)
    maximum = max(radii)
    if maximum <= 0:
        raise ValueError("Body airfoil must have positive thickness")
    return tuple((x, r / maximum) for x, r in zip(xs, radii))


def _radius_crossing(profile, radius: float) -> float:
    for (x0, r0), (x1, r1) in zip(profile, profile[1:]):
        if r0 != r1 and min(r0, r1) <= radius <= max(r0, r1):
            return x0 + (radius - r0) * (x1 - x0) / (r1 - r0)
    raise ValueError("Body end radius does not intersect the selected airfoil branch")


def motor_pod_radius_stations(params: AirframeParameters) -> tuple[tuple[float, float], ...]:
    """Airfoil afterbody plus rounded tail, or the legacy untrimmed meridian."""
    radius = params.motor_pod_diameter / 2 * params.motor_pod_profile_radius_scale
    if params.motor_pod_body_airfoil is None:
        return tuple((s * params.motor_pod_length, r * radius)
                     for s, r in params.motor_pod_radius_profile)
    cap, rounding = params.motor_pod_aft_cap_radius, params.motor_pod_aft_rounding
    if not (0 < cap and 0 < rounding < params.motor_pod_plastic_length
            and cap + rounding < radius):
        raise ValueError("Pod end cap and rounding must fit the pod envelope")
    source = _body_airfoil_half_profile(params.motor_pod_body_airfoil)
    peak = max(range(len(source)), key=lambda i: source[i][1])
    tail_cut = _radius_crossing(source[peak:], (cap + rounding) / radius)
    points = [(x, r) for x, r in source[peak:] if x < tail_cut]
    points.append((tail_cut, (cap + rounding) / radius))
    mount_x = params.motor_mount_face_offset_from_pod_center
    aft_x = -params.motor_pod_length / 2
    airfoil_length = params.motor_pod_plastic_length - rounding
    body = list(reversed([
        (mount_x - (x - source[peak][0]) / (tail_cut - source[peak][0]) * airfoil_length,
         r * radius) for x, r in points
    ]))
    cap_arc = [(aft_x + rounding * (1 - cos(theta)), cap + rounding * sin(theta))
               for theta in (0, pi / 8, pi / 4, 3 * pi / 8, pi / 2)]
    return tuple(cap_arc + body[1:])


def _section_coordinates(
    airfoil: str, chord: float, trailing_edge: float, thickness_scale: float = 1.0,
):
    if airfoil not in AIRFOILS:
        raise ValueError(f"Unknown airfoil {airfoil!r}; available: {sorted(AIRFOILS)}")
    if chord <= trailing_edge:
        raise ValueError("Chord must be larger than the physical trailing-edge thickness")
    if not isfinite(thickness_scale) or thickness_scale <= 0:
        raise ValueError("Airfoil thickness scale must be finite and positive")

    upper, lower = _split_airfoil(AIRFOILS[airfoil])
    blend_x = 0.94
    target_thickness = trailing_edge / chord

    def raw_thickness(x):
        return (_interpolate(upper, x) - _interpolate(lower, x)) * thickness_scale

    start_thickness = raw_thickness(blend_x)
    slope_step = 1e-4
    start_slope = (
        raw_thickness(blend_x + slope_step) - raw_thickness(blend_x - slope_step)
    ) / (2 * slope_step)
    blend_length = 1.0 - blend_x

    xs = {x for x, _ in upper if x < blend_x}
    xs.update(x for x, _ in lower if x < blend_x)
    xs.update((blend_x, 0.955, 0.97, 0.985, 0.995, 1.0))
    upper_out = []
    lower_out = []
    for x in sorted(xs):
        upper_raw = _interpolate(upper, x)
        lower_raw = _interpolate(lower, x)
        camber = 0.5 * (upper_raw + lower_raw)
        if x <= blend_x:
            thickness = (upper_raw - lower_raw) * thickness_scale
        else:
            t = (x - blend_x) / blend_length
            h00 = 2 * t**3 - 3 * t**2 + 1
            h10 = t**3 - 2 * t**2 + t
            h01 = -2 * t**3 + 3 * t**2
            thickness = (
                h00 * start_thickness
                + h10 * blend_length * start_slope
                + h01 * target_thickness
            )
        leading_edge_x = min(upper[0][0], lower[0][0])
        if thickness < -1e-12 or (
            thickness <= 1e-12 and abs(x - leading_edge_x) > 1e-12
        ):
            raise ValueError(f"{airfoil} trailing-edge transition produced nonpositive thickness")
        upper_out.append((x, camber + thickness / 2))
        lower_out.append((x, camber - thickness / 2))
    return upper_out, lower_out


def _airfoil_face(
    airfoil: str, chord: float, trailing_edge: float, plane: str,
    thickness_scale: float = 1.0,
):
    upper, lower = _section_coordinates(airfoil, chord, trailing_edge, thickness_scale)

    def world_point(point):
        x, ordinate = point
        x_world = (0.5 - x) * chord
        offset = ordinate * chord
        if plane == "xz":
            return (x_world, 0, offset)
        if plane == "xy":
            return (x_world, offset, 0)
        raise ValueError("Airfoil section plane must be 'xz' or 'xy'")

    upper_edge = bd.Edge.make_spline([world_point(point) for point in upper])
    lower_edge = bd.Edge.make_spline([world_point(point) for point in reversed(lower)])
    trailing_edge = bd.Edge.make_line(
        world_point(upper[-1]), world_point(lower[-1])
    )
    wires = bd.Wire.combine([upper_edge, trailing_edge, lower_edge])
    if len(wires) != 1:
        raise ValueError(f"Could not close the {airfoil} section in the {plane.upper()} plane")
    return bd.Face(wires[0])


def _loft_surface(
    sections, airfoil: str, trailing_edge: float, plane: str, label: str,
    thickness_scale: float = 1.0,
):
    faces = [
        bd.Pos(*position) * _airfoil_face(airfoil, chord, trailing_edge, plane, thickness_scale)
        for position, chord in sections
    ]
    lofted = bd.loft(sections=faces, ruled=True, clean=True)
    return _one_solid(lofted, label)


def _wing(params: AirframeParameters):
    half_span = params.wing_span / 2
    return _loft_surface(
        [
            ((0, -half_span, 0), params.wing_tip_chord),
            ((0, 0, 0), params.wing_root_chord),
            ((0, half_span, 0), params.wing_tip_chord),
        ],
        params.wing_airfoil,
        params.wing_trailing_edge,
        "xz",
        f"horizontal_wing_{params.wing_airfoil.lower()}",
        thickness_scale=params.wing_thickness_scale,
    )


def _vertical_support(params: AirframeParameters):
    half_span = params.support_span / 2
    return _loft_surface(
        [
            ((0, 0, -half_span), params.support_chord),
            ((0, 0, 0), params.support_chord),
            ((0, 0, half_span), params.support_chord),
        ],
        params.support_airfoil,
        params.support_trailing_edge,
        "xy",
        f"vertical_support_{params.support_airfoil.lower()}",
        thickness_scale=params.support_thickness_scale,
    )


def build_components(params: AirframeParameters = PARAMETERS):
    params.validate_motor_prop_interface()
    params.validate_arm_pod_fit()
    params.validate_propeller_clearance()
    fuselage = _fuselage(params)
    wing = _wing(params)
    support = _vertical_support(params)
    pod = _motor_pod(params)
    stations = (
        (params.motor_station_x, -params.horizontal_motor_offset, 0),
        (params.motor_station_x, params.horizontal_motor_offset, 0),
        (params.motor_station_x, 0, -params.vertical_motor_offset),
        (params.motor_station_x, 0, params.vertical_motor_offset),
    )
    pods = [bd.Pos(*station) * pod for station in stations]
    arm_assembly = [wing, support, *pods]
    translation = bd.Pos(params.assembly_x_offset, 0, 0)
    return [fuselage, *[translation * part for part in arm_assembly]]


def build_engineering_assembly(params: AirframeParameters = PARAMETERS):
    assembly = bd.Compound(children=build_components(params))
    assembly.label = "cruciform_tailsitter_engineering_assembly"
    return assembly


def build_airframe_oml(params: AirframeParameters = PARAMETERS):
    components = build_components(params)
    solid = components[0]
    for component in components[1:]:
        solid = solid.fuse(component)
    solid = _one_solid(solid, "cruciform_tailsitter_oml")
    solid.label = "cruciform_tailsitter_oml"
    return solid


@lru_cache(maxsize=64)
def _cached_exact_propeller_clearances(
    params: AirframeParameters,
) -> tuple[tuple[str, float], ...]:
    """Measure actual OCCT distances without calling validated public builders."""
    fuselage = _fuselage(params)
    translation = bd.Pos(params.assembly_x_offset, 0, 0)
    wing = translation * _wing(params)
    support = translation * _vertical_support(params)
    pod = _motor_pod(params)
    propeller_stations = params.propeller_mount_stations
    pod_centers = tuple(
        (params.motor_pod_center_x, y, z) for y, z in propeller_stations
    )
    pods = [bd.Pos(*center) * pod for center in pod_centers]
    disks = [
        bd.Pos(params.prop_hub_center_x, y, z)
        * bd.Rot(0, 90, 0)
        * bd.Cylinder(
            radius=params.propeller_diameter / 2,
            height=params.propeller_envelope_thickness,
        )
        for y, z in propeller_stations
    ]

    def closest(shape):
        return min(float(disk.distance_to(shape)) for disk in disks)

    result = {
        "pod_noses": min(
            float(disk.distance_to(pod_shape))
            for disk, pod_shape in zip(disks, pods)
        ),
        "horizontal_wing": closest(wing),
        "vertical_support": closest(support),
        "fuselage": closest(fuselage),
        "neighboring_propellers": min(
            float(disks[i].distance_to(disks[j]))
            for i in range(len(disks))
            for j in range(i + 1, len(disks))
        ),
    }
    if any(not isfinite(value) or value < 0 for value in result.values()):
        raise ValueError("CAD propeller-distance calculation was not finite/nonnegative")
    return tuple(result.items())
