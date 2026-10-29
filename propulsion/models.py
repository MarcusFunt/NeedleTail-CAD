"""Typed, SI-based contracts shared by propulsion solvers and benchmarks."""
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
import math
import re


class GeometryProvenance(StrEnum):
    MEASURED = "measured"
    MANUFACTURER = "manufacturer"
    DIGITIZED = "digitized"
    INFERRED = "inferred"
    PACKAGING_ONLY = "packaging_only"


class PolarProvenance(StrEnum):
    EXPERIMENTAL = "experimental"
    XFOIL = "xfoil"
    NEURALFOIL = "neuralfoil"
    ASSUMED = "assumed"


class DataProvenance(StrEnum):
    """How a numeric record entered the validation data model."""

    MEASURED = "measured"
    MANUFACTURER_PREDICTED = "manufacturer_predicted"
    TRANSCRIBED = "transcribed"
    DIGITIZED = "digitized"
    INFERRED = "inferred"


def _finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


@dataclass(frozen=True)
class BladeStation:
    """A radial blade section, normalized to metres and radians."""

    radius_m: float
    chord_m: float
    twist_rad: float
    airfoil_id: str | None = None

    def __post_init__(self) -> None:
        for name, value in (
            ("radius_m", self.radius_m),
            ("chord_m", self.chord_m),
            ("twist_rad", self.twist_rad),
        ):
            _finite(name, value)
        if self.radius_m <= 0 or self.chord_m <= 0:
            raise ValueError("Blade station radius and chord must be positive")
        if self.airfoil_id is not None and not self.airfoil_id.strip():
            raise ValueError("airfoil_id must be nonempty when provided")


@dataclass(frozen=True)
class SourceConventions:
    """Source coordinate and section conventions before SI normalization."""

    rotor_axis: str
    blade_hand: str
    section_sign: str
    frame: str
    units: str

    def __post_init__(self) -> None:
        for name, value in (
            ("rotor_axis", self.rotor_axis),
            ("blade_hand", self.blade_hand),
            ("section_sign", self.section_sign),
            ("frame", self.frame),
            ("units", self.units),
        ):
            if not value.strip():
                raise ValueError(f"{name} must be documented")
        if self.blade_hand.upper() not in {"CW", "CCW", "UNKNOWN"}:
            raise ValueError("blade_hand must be CW, CCW, or unknown")


@dataclass(frozen=True)
class PropellerGeometry:
    """Propeller source geometry and traceable fidelity labels."""

    id: str
    source_uri: str
    sha256: str
    blade_count: int
    diameter_m: float
    hub_radius_m: float
    stations: tuple[BladeStation, ...]
    geometry_provenance: GeometryProvenance
    polar_provenance: PolarProvenance
    source_conventions: SourceConventions

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.source_uri.strip():
            raise ValueError("Propeller id and source_uri must be nonempty")
        if not re.fullmatch(r"[0-9a-fA-F]{64}", self.sha256):
            raise ValueError("sha256 must contain exactly 64 hexadecimal characters")
        if (isinstance(self.blade_count, bool)
                or not isinstance(self.blade_count, int)
                or self.blade_count < 1):
            raise ValueError("blade_count must be a positive integer")
        for name, value in (("diameter_m", self.diameter_m), ("hub_radius_m", self.hub_radius_m)):
            _finite(name, value)
        if self.diameter_m <= 0 or self.hub_radius_m < 0 or self.hub_radius_m >= self.diameter_m / 2:
            raise ValueError("Propeller diameter must be positive and hub radius below tip radius")
        if not isinstance(self.stations, tuple):
            raise ValueError("stations must be an immutable tuple")
        previous_radius = self.hub_radius_m
        for index, station in enumerate(self.stations):
            if (station.radius_m < previous_radius
                    or (index > 0 and station.radius_m <= previous_radius)
                    or station.radius_m > self.diameter_m / 2):
                raise ValueError("Blade stations must increase from the hub and stay within the tip radius")
            previous_radius = station.radius_m

    def validate_for_prediction(self) -> None:
        """Reject clearance-only solids and incomplete geometry as solver inputs."""
        if self.geometry_provenance is GeometryProvenance.PACKAGING_ONLY:
            raise ValueError("packaging-only geometry cannot be used for prediction")
        if len(self.stations) < 2:
            raise ValueError("predictive blade geometry requires at least two radial stations")


@dataclass(frozen=True)
class OperatingCondition:
    """One operating point using SI units and RPM for the public interface."""

    rpm: float
    axial_airspeed_mps: float
    rho_kg_m3: float
    dynamic_viscosity_pa_s: float
    temperature_k: float
    pressure_pa: float
    rotation_sign: int

    def __post_init__(self) -> None:
        for name, value in (
            ("rpm", self.rpm),
            ("axial_airspeed_mps", self.axial_airspeed_mps),
            ("rho_kg_m3", self.rho_kg_m3),
            ("dynamic_viscosity_pa_s", self.dynamic_viscosity_pa_s),
            ("temperature_k", self.temperature_k),
            ("pressure_pa", self.pressure_pa),
        ):
            _finite(name, value)
        if self.rpm < 0:
            raise ValueError("rpm must be nonnegative")
        for name, value in (
            ("rho_kg_m3", self.rho_kg_m3),
            ("dynamic_viscosity_pa_s", self.dynamic_viscosity_pa_s),
            ("temperature_k", self.temperature_k),
            ("pressure_pa", self.pressure_pa),
        ):
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if isinstance(self.rotation_sign, bool) or self.rotation_sign not in (-1, 1):
            raise ValueError("rotation_sign must be -1 or 1")


@dataclass(frozen=True)
class RotorLoads:
    """Signed dimensional loads; torque is positive when required to drive."""

    thrust_n: float
    torque_nm: float
    shaft_power_w: float

    def __post_init__(self) -> None:
        for name, value in (
            ("thrust_n", self.thrust_n),
            ("torque_nm", self.torque_nm),
            ("shaft_power_w", self.shaft_power_w),
        ):
            _finite(name, value)


@dataclass(frozen=True)
class CoefficientState:
    """Propeller coefficients, or an explicit undefined-normalization state."""

    ct: float | None
    cp: float | None
    advance_ratio: float | None
    efficiency: float | None
    normalization_defined: bool
    warnings: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        values = (self.ct, self.cp, self.advance_ratio, self.efficiency)
        if self.normalization_defined:
            if any(value is None or not math.isfinite(value) for value in values[:3]):
                raise ValueError("Defined coefficient state requires finite CT, CP and advance ratio")
            if self.efficiency is not None and not math.isfinite(self.efficiency):
                raise ValueError("Efficiency must be finite when defined")
        elif any(value is not None for value in values):
            raise ValueError("Undefined normalization must not contain nominal coefficients")


@dataclass(frozen=True)
class MeasurementRecord:
    """One immutable observed or transcribed coefficient record with provenance."""

    propeller_id: str
    regime: str
    rpm: float
    advance_ratio: float
    ct: float
    cp: float
    eta: float | None
    provenance: DataProvenance
    source_description: str
    source_uri: str
    source_sha256: str
    source_row: int

    def __post_init__(self) -> None:
        if not self.propeller_id.strip():
            raise ValueError("propeller_id must be nonempty")
        if self.regime not in {"static", "forward"}:
            raise ValueError("regime must be 'static' or 'forward'")
        for name, value in (
            ("rpm", self.rpm),
            ("advance_ratio", self.advance_ratio),
            ("ct", self.ct),
            ("cp", self.cp),
        ):
            _finite(name, value)
        if self.eta is not None:
            _finite("eta", self.eta)
        if self.rpm <= 0:
            raise ValueError("measurement rpm must be positive")
        if self.regime == "static" and self.advance_ratio != 0:
            raise ValueError("static measurements must have advance_ratio=0")
        if not isinstance(self.provenance, DataProvenance):
            raise ValueError("provenance must be a DataProvenance value")
        if not self.source_description.strip() or not self.source_uri.strip():
            raise ValueError("measurement source description and URI must be nonempty")
        if not re.fullmatch(r"[0-9a-fA-F]{64}", self.source_sha256):
            raise ValueError("source_sha256 must contain exactly 64 hexadecimal characters")
        if isinstance(self.source_row, bool) or not isinstance(self.source_row, int) or self.source_row < 1:
            raise ValueError("source_row must be a positive one-based line number")


@dataclass(frozen=True)
class PropellerResult:
    """Backend-neutral result with solver, validity and convergence evidence."""

    thrust_n: float
    torque_nm: float
    shaft_power_w: float
    ct: float | None
    cp: float | None
    advance_ratio: float | None
    efficiency: float | None
    converged: bool
    within_validity_range: bool
    warnings: tuple[str, ...]
    solver_name: str
    solver_revision: str
    input_sha256: str
    elapsed_ms: float
    geometry_valid: bool
    convergence_evidence: tuple[str, ...]
    model_scope: str

    def __post_init__(self) -> None:
        for name, value in (
            ("thrust_n", self.thrust_n),
            ("torque_nm", self.torque_nm),
            ("shaft_power_w", self.shaft_power_w),
            ("elapsed_ms", self.elapsed_ms),
        ):
            _finite(name, value)
        if self.elapsed_ms < 0:
            raise ValueError("elapsed_ms must be nonnegative")
        rotating_coefficients = (self.ct, self.cp, self.advance_ratio)
        if any(value is None for value in rotating_coefficients):
            if any(value is not None for value in rotating_coefficients):
                raise ValueError("ct, cp and advance_ratio must be all defined or all undefined")
        else:
            for name, value in zip(("ct", "cp", "advance_ratio"), rotating_coefficients):
                assert value is not None
                _finite(name, value)
        if self.efficiency is not None:
            _finite("efficiency", self.efficiency)
        if not self.solver_name.strip() or not self.solver_revision.strip() or not self.model_scope.strip():
            raise ValueError("solver name, revision and model scope must be documented")
        if not re.fullmatch(r"[0-9a-fA-F]{64}", self.input_sha256):
            raise ValueError("input_sha256 must contain exactly 64 hexadecimal characters")
        if not isinstance(self.warnings, tuple) or not isinstance(self.convergence_evidence, tuple):
            raise ValueError("warnings and convergence_evidence must be immutable tuples")
