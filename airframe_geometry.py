from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from functools import lru_cache
from math import isfinite, sqrt

from cadgen import build123d as bd

from profiles.airfoil_coordinates import SD7003_COORDINATES, SD8020_COORDINATES


@dataclass(frozen=True)
class AirframeParameters:
    """First-pass geometry parameters in millimeters."""

    fuselage_length: float = 170.0
    fuselage_max_radius: float = 21.0
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
    wing_trailing_edge: float = 0.30
    support_chord: float = 38.0
    support_airfoil: str = "SD8020"
    support_trailing_edge: float = 0.30
    motor_station_x: float = 0.0
    horizontal_motor_offset: float = 100.0
    vertical_motor_offset: float = 85.0
    propeller_diameter: float = 76.2
    propeller_envelope_thickness: float = 1.0
    propeller_clearance_margin: float = 1.0
    motor_pod_length: float = 42.0
    motor_pod_diameter: float = 24.0
    arm_pod_overlap: float = 1.0
    motor_body_diameter: float = 19.9
    motor_body_length: float = 13.5
    motor_body_radius_profile: tuple[tuple[float, float], ...] = (
        (0.0, 8.75), (3.8, 9.95), (11.4, 9.95), (13.5, 8.4),
    )
    motor_pod_radial_allowance: float = 1.5
    motor_mount_face_offset_from_pod_center: float = 4.5
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
        return 2 * (
            self.horizontal_motor_offset
            - self.motor_pod_diameter / 2
            + self.arm_pod_overlap
        )

    @property
    def support_span(self) -> float:
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
        pod_radius = self.motor_pod_diameter / 2
        if pod_radius <= 0:
            raise ValueError("Motor-pod diameter must be positive")
        if not 0 < self.arm_pod_overlap < pod_radius:
            raise ValueError("Arm-to-pod overlap must be positive and less than the pod radius")
        if self.wing_span <= 0 or self.support_span <= 0:
            raise ValueError("Motor offsets must leave positive arm spans inside the pods")

    @property
    def propeller_clearance_report(self) -> dict[str, float]:
        """Conservative clearances from swept disks to modeled airframe features."""
        disk_radius = self.propeller_diameter / 2
        disk_x_min = self.prop_hub_center_x - self.propeller_envelope_thickness / 2
        nearest_x = max(0.0, abs(self.prop_hub_center_x) - self.propeller_envelope_thickness / 2)

        wing_root_half_chord = self.wing_root_chord / 2
        wing_tip_half_chord = self.wing_tip_chord / 2
        if nearest_x >= wing_root_half_chord:
            horizontal_wing_clearance = nearest_x - wing_root_half_chord
        else:
            if wing_tip_half_chord >= nearest_x:
                wing_span_at_disk_x = self.wing_span / 2
            else:
                chord_fraction = (wing_root_half_chord - nearest_x) / (
                    wing_root_half_chord - wing_tip_half_chord
                )
                wing_span_at_disk_x = self.wing_span / 2 * chord_fraction
            radial_gap = max(0.0, self.horizontal_motor_offset - wing_span_at_disk_x)
            horizontal_wing_clearance = radial_gap - disk_radius

        support_x_gap = nearest_x - self.support_chord / 2
        if support_x_gap > 0:
            vertical_support_clearance = support_x_gap
        else:
            support_radial_gap = max(
                0.0, self.vertical_motor_offset - self.support_span / 2
            )
            vertical_support_clearance = support_radial_gap - disk_radius

        propeller_stations = (
            (-self.horizontal_motor_offset, 0.0),
            (self.horizontal_motor_offset, 0.0),
            (0.0, -self.vertical_motor_offset),
            (0.0, self.vertical_motor_offset),
        )
        neighbor_spacing = min(
            sqrt((y0 - y1) ** 2 + (z0 - z1) ** 2)
            for index, (y0, z0) in enumerate(propeller_stations)
            for y1, z1 in propeller_stations[index + 1:]
        )
        return {
            "pod_noses": disk_x_min - self.pod_nose_x,
            "horizontal_wing": horizontal_wing_clearance,
            "vertical_support": vertical_support_clearance,
            "fuselage": min(
                self.horizontal_motor_offset,
                self.vertical_motor_offset,
            ) - self.fuselage_max_radius - disk_radius,
            "neighboring_propellers": neighbor_spacing - self.propeller_diameter,
        }

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
    def motor_mount_face_x(self) -> float:
        return self.motor_station_x + self.motor_mount_face_offset_from_pod_center

    @property
    def pod_nose_x(self) -> float:
        return self.motor_station_x + self.motor_pod_length / 2

    @property
    def prop_hub_center_x(self) -> float:
        return self.motor_mount_face_x + self.prop_hub_center_from_mount_face

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
        motor_aft_x = self.motor_mount_face_x
        motor_forward_x = motor_aft_x + self.motor_body_length
        pod_aft_x = self.motor_station_x - self.motor_pod_length / 2
        if motor_aft_x < pod_aft_x or motor_forward_x > self.pod_nose_x:
            raise ValueError("The motor body must fit within the motor-pod axial envelope")
        if self.motor_body_diameter > self.motor_pod_diameter:
            raise ValueError("The motor body diameter exceeds the motor-pod outside diameter")
        pod_stations = tuple(
            (fraction * self.motor_pod_length, ratio * self.motor_pod_diameter / 2)
            for fraction, ratio in self.motor_pod_radius_profile
        )
        # Sample the same interpolating spline used by _motor_pod, rather than
        # validating a piecewise-linear approximation of its control stations.
        pod_radius_profile = _sample_spline_radius_profile(pod_stations)
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
        if self.prop_hub_rear_face_x < self.pod_nose_x - 1e-9:
            raise ValueError("The propeller hub overlaps the motor-pod nose")
        if not (
            self.prop_hub_rear_face_x
            <= self.motor_shaft_tip_x
            <= self.prop_hub_front_face_x
        ):
            raise ValueError("The modeled motor shaft tip must enter the propeller hub")


PARAMETERS = AirframeParameters()
AIRFOILS = {
    "SD7003": SD7003_COORDINATES,
    "SD8020": SD8020_COORDINATES,
}


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


def _fuselage(params: AirframeParameters):
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
    return _revolve_radius_profile(aft + fore, "fuselage")


def _motor_pod(params: AirframeParameters):
    half_length = params.motor_pod_length / 2
    max_radius = params.motor_pod_diameter / 2
    stations = [
        (fraction * params.motor_pod_length, ratio * max_radius)
        for fraction, ratio in params.motor_pod_radius_profile
    ]
    # The local pod is centered on X=0. Its rounded forebody points toward +X.
    return _revolve_radius_profile(stations, "motor_pod")


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
    for i in range(1, len(raw_points)):
        x0, y0 = raw_points[i - 1]
        x1, y1 = raw_points[i]
        if y0 > 0 and y1 <= 0:
            if abs(y1) < 1e-14:
                nose = (x1, 0.0)
                upper = raw_points[:i] + [nose]
                lower = [nose] + raw_points[i + 1:]
            else:
                fraction = y0 / (y0 - y1)
                nose = (x0 + fraction * (x1 - x0), 0.0)
                upper = raw_points[:i] + [nose]
                # The crossing segment's second point has already passed the
                # leading edge in source traversal order; retaining it here
                # makes the lower branch non-monotonic after sorting.
                lower = [nose] + raw_points[i + 1:]
            break
    else:
        raise ValueError("Airfoil data must run from upper trailing edge through the nose")

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


def _section_coordinates(airfoil: str, chord: float, trailing_edge: float):
    if airfoil not in AIRFOILS:
        raise ValueError(f"Unknown airfoil {airfoil!r}; available: {sorted(AIRFOILS)}")
    if chord <= trailing_edge:
        raise ValueError("Chord must be larger than the physical trailing-edge thickness")

    upper, lower = _split_airfoil(AIRFOILS[airfoil])
    blend_x = 0.94
    target_thickness = trailing_edge / chord

    def raw_thickness(x):
        return _interpolate(upper, x) - _interpolate(lower, x)

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
            thickness = upper_raw - lower_raw
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


def _airfoil_face(airfoil: str, chord: float, trailing_edge: float, plane: str):
    upper, lower = _section_coordinates(airfoil, chord, trailing_edge)

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


def _loft_surface(sections, airfoil: str, trailing_edge: float, plane: str, label: str):
    faces = [
        bd.Pos(*position) * _airfoil_face(airfoil, chord, trailing_edge, plane)
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
        "horizontal_wing_sd7003",
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
        "vertical_support_sd8020",
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
    return [fuselage, wing, support, *pods]


def build_engineering_assembly(params: AirframeParameters = PARAMETERS):
    assembly = bd.Compound(children=build_components(params))
    assembly.label = "cruciform_tailsitter_engineering_assembly"
    return assembly


def build_airframe_oml(params: AirframeParameters = PARAMETERS):
    components = build_components(params)
    with bd.BuildPart() as airframe:
        for component in components:
            bd.insert(component)
    solid = _one_solid(airframe.part, "cruciform_tailsitter_oml")
    solid.label = "cruciform_tailsitter_oml"
    return solid
