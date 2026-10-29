"""SI conversions for standard propeller coefficients."""
from __future__ import annotations

import math

from propulsion.models import CoefficientState, OperatingCondition, PropellerGeometry, RotorLoads


def _finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def loads_from_coefficients(
    ct: float,
    cp: float,
    geometry: PropellerGeometry,
    condition: OperatingCondition,
) -> RotorLoads:
    """Convert CT/CP to thrust, required torque, and shaft power."""
    _finite("ct", ct)
    _finite("cp", cp)
    if condition.rpm <= 0:
        raise ValueError("Coefficient normalization requires rpm > 0")
    revolutions_per_second = condition.rpm / 60.0
    diameter = geometry.diameter_m
    thrust = ct * condition.rho_kg_m3 * revolutions_per_second**2 * diameter**4
    power = cp * condition.rho_kg_m3 * revolutions_per_second**3 * diameter**5
    torque = power / (2.0 * math.pi * revolutions_per_second)
    return RotorLoads(thrust_n=thrust, torque_nm=torque, shaft_power_w=power)


def coefficients_from_loads(
    loads: RotorLoads,
    geometry: PropellerGeometry,
    condition: OperatingCondition,
) -> CoefficientState:
    """Convert signed dimensional loads to CT/CP/J without inventing zero-RPM values."""
    if condition.rpm == 0:
        return CoefficientState(
            ct=None,
            cp=None,
            advance_ratio=None,
            efficiency=None,
            normalization_defined=False,
            warnings=("zero_rpm",),
        )
    revolutions_per_second = condition.rpm / 60.0
    diameter = geometry.diameter_m
    ct = loads.thrust_n / (
        condition.rho_kg_m3 * revolutions_per_second**2 * diameter**4
    )
    cp = loads.shaft_power_w / (
        condition.rho_kg_m3 * revolutions_per_second**3 * diameter**5
    )
    advance_ratio = condition.axial_airspeed_mps / (revolutions_per_second * diameter)
    efficiency = (
        advance_ratio * ct / cp
        if advance_ratio > 0 and ct > 0 and cp > 0
        else None
    )
    warnings: list[str] = []
    if ct <= 0:
        warnings.append("nonpositive_thrust")
    if cp <= 0:
        warnings.append("nonpositive_shaft_power")
    if advance_ratio <= 0:
        warnings.append("nonpositive_advance_ratio")
    return CoefficientState(
        ct=ct,
        cp=cp,
        advance_ratio=advance_ratio,
        efficiency=efficiency,
        normalization_defined=True,
        warnings=tuple(warnings),
    )
