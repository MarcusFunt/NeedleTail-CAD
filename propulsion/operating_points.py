"""Constrained motor/battery operating points using a pinned PyThrust API.

PyThrust's propeller interpolator clamps out-of-range RPM and low advance ratio.
This adapter validates map coverage first, solves only inside supported map
intervals, and records the assumptions needed for XING2 electrical and thermal
inputs that its datasheet does not publish.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import hashlib
import io
import json
import math
from pathlib import Path
from typing import Iterable, Literal, Sequence

import pythrust
from scipy.optimize import brentq

from pythrust.battery import BatteryState, RateMapBattery
from pythrust.propellers.database import (
    PropellerDataPoint,
    PropellerEntry,
    PropellerMetadata,
)
from pythrust.propulsion import MotorSpec, PropellerSpec, SystemSpec
from pythrust.propulsion.solver import evaluate_propulsion_state


PYTHRUST_REVISION = "c7a9f597d13ab70f1d4b15d6a77c2c424be42cd6"
PROP_MAP_COLUMNS = ("rpm", "advance_ratio", "thrust_coeff", "power_coeff")
DEFAULT_THROTTLE_VALUES = tuple(index / 40.0 for index in range(1, 41))
@dataclass(frozen=True)
class Xing2ManufacturerData:
    """Values transcribed from iFlight's XING2 1404 4600KV datasheet.

    Missing values are ``None`` rather than guessed defaults. The two
    effective-resistance values are sensitivity cases for the unresolved
    interphase-to-PyThrust winding-resistance convention, not measurements.
    """

    kv_rpm_per_v: float
    interphase_resistance_ohm: float
    effective_resistance_sensitivity_ohm: tuple[float, float]
    peak_current_a: float
    max_test_power_w: float
    tested_voltage_v: float
    test_temperature_c: float
    input_cells_min: int
    input_cells_max: int
    no_load_current_a: float | None
    sustained_current_limit_a: float | None
    winding_temperature_limit_c: float | None
    esc_specification: str | None
    battery_resistance_or_model: str | None
    source_pdf_url: str
    source_product_url: str
    source_pdf_sha256: str
    source_scope: str


def _load_xing2_record() -> Xing2ManufacturerData:
    record_path = Path(__file__).parent / "data" / "xing2_1404_4600kv.json"
    record = json.loads(record_path.read_text(encoding="utf-8"))
    motor = record["motor"]
    source = record["source"]
    cells = motor["input_cells"]
    return Xing2ManufacturerData(
        kv_rpm_per_v=float(motor["kv_rpm_per_v"]),
        interphase_resistance_ohm=float(motor["interphase_resistance_ohm"]),
        effective_resistance_sensitivity_ohm=tuple(
            float(value) for value in motor["effective_resistance_sensitivity_ohm"]
        ),
        peak_current_a=float(motor["peak_current_a"]),
        max_test_power_w=float(motor["max_reported_test_power_w"]),
        tested_voltage_v=float(motor["test_voltage_v"]),
        test_temperature_c=float(motor["test_temperature_c"]),
        input_cells_min=int(cells[0]),
        input_cells_max=int(cells[1]),
        no_load_current_a=motor["no_load_current_a"],
        sustained_current_limit_a=motor["sustained_current_limit_a"],
        winding_temperature_limit_c=motor["winding_temperature_limit_c"],
        esc_specification=motor["esc_specification"],
        battery_resistance_or_model=motor["battery_model_or_resistance"],
        source_pdf_url=str(source["datasheet_url"]),
        source_product_url=str(source["manufacturer_product_url"]),
        source_pdf_sha256=str(source["datasheet_sha256"]),
        source_scope=str(source["scope"]),
    )


XING2_1404_4600KV = _load_xing2_record()


class OutOfMapError(ValueError):
    """Requested RPM/advance ratio lies outside the measured map domain."""


@dataclass(frozen=True)
class PropellerMapMetadata:
    id: str
    manufacturer: str
    model: str
    diameter_m: float
    pitch_m: float
    blade_count: int
    source_uri: str
    source_description: str
    source_sha256: str | None = None

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.manufacturer.strip() or not self.model.strip():
            raise ValueError("propeller id, manufacturer, and model must be nonempty")
        if not self.source_uri.strip() or not self.source_description.strip():
            raise ValueError("map source URI and description must be documented")
        if not math.isfinite(self.diameter_m) or self.diameter_m <= 0:
            raise ValueError("diameter_m must be finite and positive")
        if not math.isfinite(self.pitch_m) or self.pitch_m <= 0:
            raise ValueError("pitch_m must be finite and positive")
        if isinstance(self.blade_count, bool) or not isinstance(self.blade_count, int) or self.blade_count < 1:
            raise ValueError("blade_count must be a positive integer")
        if self.source_sha256 is not None and not _is_sha256(self.source_sha256):
            raise ValueError("source_sha256 must be a 64-character hexadecimal hash")


@dataclass(frozen=True)
class PropellerMap:
    """A strict, provenance-carrying coefficient map accepted by PyThrust."""

    metadata: PropellerMapMetadata
    rows: tuple[tuple[float, float, float, float], ...]

    def __post_init__(self) -> None:
        normalized = tuple(
            sorted(
                (
                    _finite_float("rpm", rpm),
                    _finite_float("advance_ratio", advance_ratio),
                    _finite_float("thrust_coeff", ct),
                    _finite_float("power_coeff", cp),
                )
                for rpm, advance_ratio, ct, cp in self.rows
            )
        )
        if len(normalized) < 4:
            raise ValueError("a propeller map needs at least two RPM bands and two J rows")
        seen: set[tuple[float, float]] = set()
        grouped: dict[float, list[float]] = {}
        for rpm, advance_ratio, ct, cp in normalized:
            if rpm <= 0 or advance_ratio < 0 or ct < 0 or cp < 0:
                raise ValueError("map rows require rpm>0, J>=0, CT>=0, and CP>=0")
            pair = (rpm, advance_ratio)
            if pair in seen:
                raise ValueError(f"duplicate map row for rpm={rpm:g}, J={advance_ratio:g}")
            seen.add(pair)
            grouped.setdefault(rpm, []).append(advance_ratio)
        if len(grouped) < 2:
            raise ValueError("a propeller map needs at least two distinct RPM bands")
        if any(len(j_values) < 2 for j_values in grouped.values()):
            raise ValueError("each RPM band needs at least two advance-ratio rows")
        object.__setattr__(self, "rows", normalized)

    @classmethod
    def from_rows(
        cls,
        metadata: PropellerMapMetadata,
        rows: Iterable[Sequence[float]],
    ) -> "PropellerMap":
        return cls(metadata, tuple(tuple(row) for row in rows))  # type: ignore[arg-type]

    @classmethod
    def from_files(cls, csv_path: str | Path, metadata_path: str | Path) -> "PropellerMap":
        csv_path = Path(csv_path)
        raw_metadata = json.loads(Path(metadata_path).read_text(encoding="utf-8"))
        source = raw_metadata.get("source", {})
        source = source if isinstance(source, dict) else {}
        actual_sha256 = _sha256_file(csv_path)
        expected_data_sha256 = raw_metadata.get("data_sha256")
        if expected_data_sha256 and actual_sha256.lower() != str(expected_data_sha256).lower():
            raise ValueError(f"coefficient CSV hash does not match metadata: {csv_path}")
        metadata = PropellerMapMetadata(
            id=str(raw_metadata["id"]),
            manufacturer=str(raw_metadata["manufacturer"]),
            model=str(raw_metadata["model"]),
            diameter_m=float(raw_metadata["diameter_m"]),
            pitch_m=float(raw_metadata["pitch_m"]),
            blade_count=_positive_integer("blade_count", raw_metadata["blade_count"]),
            source_uri=str(raw_metadata.get("source_uri", source.get("uri", ""))),
            source_description=str(
                raw_metadata.get("source_description", source.get("description", ""))
            ),
            source_sha256=str(source.get("sha256", actual_sha256)),
        )
        with csv_path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None or not set(PROP_MAP_COLUMNS).issubset(reader.fieldnames):
                raise ValueError(
                    "coefficient CSV must include: " + ", ".join(PROP_MAP_COLUMNS)
                )
            rows = []
            for line, row in enumerate(reader, start=2):
                try:
                    rows.append(tuple(float(row[column]) for column in PROP_MAP_COLUMNS))
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"invalid coefficient row at {csv_path}:{line}") from exc
        return cls.from_rows(metadata, rows)

    @property
    def rpm_levels(self) -> tuple[float, ...]:
        return tuple(sorted({row[0] for row in self.rows}))

    @property
    def data_sha256(self) -> str:
        return hashlib.sha256(_canonical_map_csv_bytes(self.rows)).hexdigest()

    def coefficients(self, rpm: float, advance_ratio: float) -> tuple[float, float]:
        """Interpolate only when both requested coordinates are supported."""
        rpm = _finite_float("rpm", rpm)
        advance_ratio = _finite_float("advance_ratio", advance_ratio)
        levels = self.rpm_levels
        if rpm < levels[0] or rpm > levels[-1]:
            raise OutOfMapError(f"rpm={rpm:g} is out of map [{levels[0]:g}, {levels[-1]:g}]")
        brackets = _bracketing_levels(levels, rpm)
        by_rpm = {level: [row for row in self.rows if row[0] == level] for level in brackets}
        for level, rpm_rows in by_rpm.items():
            j_min = rpm_rows[0][1]
            j_max = rpm_rows[-1][1]
            if advance_ratio < j_min or advance_ratio > j_max:
                raise OutOfMapError(
                    f"J={advance_ratio:g} is out of map at rpm={level:g} [{j_min:g}, {j_max:g}]"
                )
        entry = self.to_pythrust_entry()
        return entry.get_coefficients(rpm, advance_ratio)

    def to_pythrust_entry(self) -> "_StrictPropellerEntry":
        metadata = PropellerMetadata(
            id=self.metadata.id,
            manufacturer=self.metadata.manufacturer,
            model=self.metadata.model,
            diameter_in=self.metadata.diameter_m / 0.0254,
            pitch_in=self.metadata.pitch_m / 0.0254,
            blade_count=self.metadata.blade_count,
            data_csv="in-memory-provenance-map.csv",
        )
        grouped: dict[float, list[PropellerDataPoint]] = {}
        for rpm, advance_ratio, ct, cp in self.rows:
            grouped.setdefault(rpm, []).append(
                PropellerDataPoint(j=advance_ratio, ct=ct, cp=cp)
            )
        return _StrictPropellerEntry(metadata=metadata, data_by_rpm=grouped)

    def export(self, directory: str | Path) -> tuple[Path, Path]:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        csv_path = directory / "propeller_map.csv"
        metadata_path = directory / "propeller_map.json"
        if csv_path.exists() or metadata_path.exists():
            raise FileExistsError(f"refusing to overwrite propeller-map export in {directory}")
        csv_path.write_bytes(_canonical_map_csv_bytes(self.rows))
        data_sha256 = _sha256_file(csv_path)
        manifest = {
            "schema_version": 1,
            "id": self.metadata.id,
            "manufacturer": self.metadata.manufacturer,
            "model": self.metadata.model,
            "diameter_m": self.metadata.diameter_m,
            "pitch_m": self.metadata.pitch_m,
            "blade_count": self.metadata.blade_count,
            "source": {
                "uri": self.metadata.source_uri,
                "description": self.metadata.source_description,
                "sha256": self.metadata.source_sha256 or data_sha256,
            },
            "data_csv": csv_path.name,
            "data_sha256": data_sha256,
            "row_count": len(self.rows),
            "columns": list(PROP_MAP_COLUMNS),
            "solver": {
                "package": "setuav-pythrust",
                "version": pythrust.__version__,
                "revision": PYTHRUST_REVISION,
            },
            "domain": {
                "rpm_min": self.rpm_levels[0],
                "rpm_max": self.rpm_levels[-1],
                "advance_ratio_by_rpm": {
                    str(rpm): [
                        min(row[1] for row in self.rows if row[0] == rpm),
                        max(row[1] for row in self.rows if row[0] == rpm),
                    ]
                    for rpm in self.rpm_levels
                },
            },
        }
        metadata_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return csv_path, metadata_path


class _StrictPropellerEntry(PropellerEntry):
    """PyThrust database entry that rejects its upstream clamp behavior."""

    def get_coefficients(self, rpm: float, advance_ratio: float) -> tuple[float, float]:
        levels = self.rpm_levels
        if not levels or rpm < levels[0] or rpm > levels[-1]:
            raise OutOfMapError(f"rpm={rpm:g} is out of propeller-map bounds")
        brackets = _bracketing_levels(levels, rpm)
        for level in brackets:
            points = self.data_by_rpm[level]
            if advance_ratio < points[0].j or advance_ratio > points[-1].j:
                raise OutOfMapError(f"J={advance_ratio:g} is out of map at rpm={level:g}")
        return super().get_coefficients(rpm, advance_ratio)


@dataclass(frozen=True)
class MotorElectricalScenario:
    """Explicit values for XING2 fields that need calibration or convention choice."""

    label: str
    effective_resistance_ohm: float
    no_load_current_a: float
    assumption_note: str

    def __post_init__(self) -> None:
        if not self.label.strip() or not self.assumption_note.strip():
            raise ValueError("motor scenario label and assumption note must be documented")
        if not math.isfinite(self.effective_resistance_ohm) or self.effective_resistance_ohm <= 0:
            raise ValueError("effective_resistance_ohm must be finite and positive")
        if not math.isfinite(self.no_load_current_a) or self.no_load_current_a < 0:
            raise ValueError("no_load_current_a must be finite and non-negative")

    def to_pythrust_motor(self) -> MotorSpec:
        return MotorSpec(
            kv_rpm_per_v=XING2_1404_4600KV.kv_rpm_per_v,
            resistance_ohm=self.effective_resistance_ohm,
            no_load_current_a=self.no_load_current_a,
            current_max_a=XING2_1404_4600KV.peak_current_a,
        )


@dataclass(frozen=True)
class OperatingConstraints:
    """Caller-supplied ESC, motor thermal, and structural bounds.

    Thermal screening assumes the modeled motor input-minus-shaft power is heat
    and applies a caller-provided steady-state thermal resistance. These are
    explicit screening constraints, not datasheet-rated values.
    """

    motor_scenario: MotorElectricalScenario
    battery_model: RateMapBattery
    system_resistance_ohm: float
    esc_efficiency: float
    max_motor_current_a: float
    max_battery_current_a: float
    max_rpm: float
    max_winding_temperature_c: float
    motor_thermal_resistance_c_per_w: float
    max_esc_temperature_c: float
    esc_thermal_resistance_c_per_w: float
    ambient_temperature_c: float
    mission_duration_s: float
    throttle_values: tuple[float, ...] = DEFAULT_THROTTLE_VALUES
    rho_kg_m3: float = 1.225
    max_shaft_power_w: float | None = None
    max_battery_power_w: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.battery_model, RateMapBattery):
            raise ValueError("battery_model must be a PyThrust RateMapBattery so sag and energy can be checked")
        if not (
            XING2_1404_4600KV.input_cells_min
            <= self.battery_model.series
            <= XING2_1404_4600KV.input_cells_max
        ):
            raise ValueError("XING2 operating points require a 3S or 4S battery model")
        for name, value in (
            ("system_resistance_ohm", self.system_resistance_ohm),
            ("esc_efficiency", self.esc_efficiency),
            ("max_motor_current_a", self.max_motor_current_a),
            ("max_battery_current_a", self.max_battery_current_a),
            ("max_rpm", self.max_rpm),
            ("max_winding_temperature_c", self.max_winding_temperature_c),
            ("motor_thermal_resistance_c_per_w", self.motor_thermal_resistance_c_per_w),
            ("max_esc_temperature_c", self.max_esc_temperature_c),
            ("esc_thermal_resistance_c_per_w", self.esc_thermal_resistance_c_per_w),
            ("mission_duration_s", self.mission_duration_s),
            ("rho_kg_m3", self.rho_kg_m3),
            ("ambient_temperature_c", self.ambient_temperature_c),
        ):
            _finite_float(name, value)
        if self.system_resistance_ohm < 0:
            raise ValueError("system_resistance_ohm must be non-negative")
        if not (0.0 < self.esc_efficiency <= 1.0):
            raise ValueError("esc_efficiency must be in (0, 1]")
        if min(
            self.max_motor_current_a,
            self.max_battery_current_a,
            self.max_rpm,
            self.motor_thermal_resistance_c_per_w,
            self.esc_thermal_resistance_c_per_w,
            self.mission_duration_s,
            self.rho_kg_m3,
        ) <= 0:
            raise ValueError("current, RPM, thermal resistance, mission duration, and density must be positive")
        if self.max_motor_current_a > XING2_1404_4600KV.peak_current_a:
            raise ValueError("max_motor_current_a cannot exceed the manufacturer-reported peak current")
        if self.max_battery_current_a > self.battery_model.max_current_a * self.battery_model.parallel:
            raise ValueError("max_battery_current_a cannot exceed the battery model's pack current limit")
        for name, value in (
            ("max_shaft_power_w", self.max_shaft_power_w),
            ("max_battery_power_w", self.max_battery_power_w),
        ):
            if value is not None and _finite_float(name, value) <= 0:
                raise ValueError(f"{name} must be finite and positive when provided")
        throttles = tuple(float(value) for value in self.throttle_values)
        if not throttles or any(not math.isfinite(value) or value <= 0 or value > 1 for value in throttles):
            raise ValueError("throttle_values must contain values in (0, 1]")
        if tuple(sorted(set(throttles))) != throttles:
            raise ValueError("throttle_values must be unique and ascending")
        object.__setattr__(self, "throttle_values", throttles)


OperatingStatus = Literal["feasible", "infeasible", "solver_failed", "out_of_map"]


@dataclass(frozen=True)
class MotorOperatingPoint:
    status: OperatingStatus
    reason: str | None
    throttle: float
    speed_mps: float
    battery_soc: float
    battery_series: int
    map_id: str
    motor_scenario: str
    rpm: float | None = None
    advance_ratio: float | None = None
    ct: float | None = None
    cp: float | None = None
    thrust_n: float | None = None
    torque_nm: float | None = None
    shaft_power_w: float | None = None
    motor_power_w: float | None = None
    battery_power_w: float | None = None
    motor_current_a: float | None = None
    battery_current_a: float | None = None
    motor_voltage_v: float | None = None
    battery_voltage_v: float | None = None
    propeller_efficiency: float | None = None
    motor_efficiency: float | None = None
    system_efficiency: float | None = None
    motor_loss_w: float | None = None
    system_loss_w: float | None = None
    esc_loss_w: float | None = None
    battery_c_rate: float | None = None
    battery_open_circuit_voltage_v: float | None = None
    battery_internal_loss_w: float | None = None
    motor_temperature_estimate_c: float | None = None
    esc_temperature_estimate_c: float | None = None
    energy_consumed_wh: float | None = None
    final_soc: float | None = None


@dataclass(frozen=True)
class OperatingEnvelope:
    scenario_count: int
    feasible_scenario_count: int
    thrust_min_n: float | None
    thrust_max_n: float | None
    shaft_power_min_w: float | None
    shaft_power_max_w: float | None
    battery_power_min_w: float | None
    battery_power_max_w: float | None
    statuses: tuple[str, ...]


def solve_operating_point(
    speed_mps: float,
    battery_state: BatteryState,
    propeller: PropellerMap,
    constraints: OperatingConstraints,
    throttle: float,
) -> MotorOperatingPoint:
    """Solve a single throttle point, restricting every evaluation to map data."""
    speed_mps = _finite_float("speed_mps", speed_mps)
    throttle = _finite_float("throttle", throttle)
    if speed_mps < 0 or not 0 < throttle <= 1:
        raise ValueError("speed_mps must be non-negative and throttle must be in (0, 1]")

    motor = constraints.motor_scenario.to_pythrust_motor()
    prop_spec = PropellerSpec(
        diameter_m=propeller.metadata.diameter_m,
        blade_count=propeller.metadata.blade_count,
        pitch_m=propeller.metadata.pitch_m,
    )
    entry = propeller.to_pythrust_entry()
    system = SystemSpec(resistance_ohm=constraints.system_resistance_ohm)

    def state_at_rpm(rpm: float):
        ct, cp, j, torque_nm, motor_current_a, v_back = evaluate_propulsion_state(
            motor,
            prop_spec,
            entry,
            constraints.rho_kg_m3,
            speed_mps,
            rpm,
        )
        motor_voltage_v = v_back + motor_current_a * motor.get_winding_resistance(motor_current_a)
        system_voltage_drop = motor_current_a * system.resistance_ohm
        battery_current_a = throttle * motor_current_a / constraints.esc_efficiency
        battery_point = constraints.battery_model.state_at_current(
            state=battery_state,
            current_a=battery_current_a,
        )
        required_voltage_v = motor_voltage_v + system_voltage_drop
        residual_v = required_voltage_v - throttle * battery_point.terminal_voltage_v
        return {
            "ct": ct,
            "cp": cp,
            "j": j,
            "torque_nm": torque_nm,
            "motor_current_a": motor_current_a,
            "battery_current_a": battery_current_a,
            "v_back": v_back,
            "motor_voltage_v": motor_voltage_v,
            "battery_point": battery_point,
            "residual_v": residual_v,
        }

    # Sampling identifies contiguous valid map intervals before root finding;
    # PyThrust's own coefficient clamp can therefore never fabricate coverage.
    rpm_min, rpm_max = propeller.rpm_levels[0], propeller.rpm_levels[-1]
    samples = max(401, (len(propeller.rpm_levels) - 1) * 100 + 1)
    rpm_grid = [rpm_min + (rpm_max - rpm_min) * index / (samples - 1) for index in range(samples)]
    evaluated: list[tuple[float, dict[str, object]] | None] = []
    for rpm in rpm_grid:
        try:
            evaluated.append((rpm, state_at_rpm(rpm)))
        except OutOfMapError:
            evaluated.append(None)
        except Exception as exc:
            return _empty_result(
                "solver_failed",
                f"state evaluation failed: {type(exc).__name__}: {exc}",
                speed_mps,
                battery_state,
                propeller,
                constraints,
                throttle,
            )

    valid = [item for item in evaluated if item is not None]
    if not valid:
        return _empty_result(
            "out_of_map",
            "no RPM/J samples are covered by the supplied propeller map at this speed",
            speed_mps,
            battery_state,
            propeller,
            constraints,
            throttle,
        )

    bracket: tuple[float, float] | None = None
    previous: tuple[float, dict[str, object]] | None = None
    for item in evaluated:
        if item is None:
            previous = None
            continue
        residual = float(item[1]["residual_v"])
        if residual == 0.0:
            bracket = (item[0], item[0])
            break
        if previous is not None:
            previous_residual = float(previous[1]["residual_v"])
            if previous_residual * residual < 0:
                bracket = (previous[0], item[0])
                break
        previous = item

    if bracket is None:
        return _empty_result(
            "out_of_map",
            "the electrical equilibrium lies outside the supported RPM/J map intervals",
            speed_mps,
            battery_state,
            propeller,
            constraints,
            throttle,
        )

    try:
        if bracket[0] == bracket[1]:
            rpm = bracket[0]
        else:
            rpm = brentq(
                lambda value: float(state_at_rpm(value)["residual_v"]),
                *bracket,
                xtol=1e-7,
                rtol=1e-12,
                maxiter=100,
            )
        state = state_at_rpm(rpm)
    except OutOfMapError as exc:
        return _empty_result(
            "out_of_map", str(exc), speed_mps, battery_state, propeller, constraints, throttle
        )
    except Exception as exc:
        return _empty_result(
            "solver_failed",
            f"equilibrium solver failed: {type(exc).__name__}: {exc}",
            speed_mps,
            battery_state,
            propeller,
            constraints,
            throttle,
        )

    ct = float(state["ct"])
    cp = float(state["cp"])
    j = float(state["j"])
    torque_nm = float(state["torque_nm"])
    motor_current_a = float(state["motor_current_a"])
    battery_current_a = float(state["battery_current_a"])
    motor_voltage_v = float(state["motor_voltage_v"])
    battery_point = state["battery_point"]
    assert hasattr(battery_point, "terminal_voltage_v")
    battery_voltage_v = float(battery_point.terminal_voltage_v)
    battery_power_w = float(battery_point.power_w)
    n = rpm / 60.0
    thrust_n = ct * constraints.rho_kg_m3 * n**2 * prop_spec.diameter_m**4
    shaft_power_w = cp * constraints.rho_kg_m3 * n**3 * prop_spec.diameter_m**5
    motor_power_w = motor_voltage_v * motor_current_a
    motor_loss_w = motor_power_w - shaft_power_w
    system_loss_w = motor_current_a**2 * system.resistance_ohm
    esc_loss_w = battery_power_w - motor_power_w - system_loss_w
    battery_open_circuit_voltage_v = float(battery_point.ocv_v) * constraints.battery_model.series
    battery_internal_loss_w = (battery_open_circuit_voltage_v - battery_voltage_v) * battery_current_a
    temperature_estimate_c = (
        constraints.ambient_temperature_c
        + max(0.0, motor_loss_w) * constraints.motor_thermal_resistance_c_per_w
    )
    esc_temperature_estimate_c = (
        constraints.ambient_temperature_c
        + max(0.0, esc_loss_w) * constraints.esc_thermal_resistance_c_per_w
    )
    prop_efficiency = (thrust_n * speed_mps / shaft_power_w) if shaft_power_w > 0 else 0.0
    motor_efficiency = shaft_power_w / motor_power_w if motor_power_w > 0 else 0.0
    system_efficiency = thrust_n * speed_mps / battery_power_w if battery_power_w > 0 else 0.0

    # Verify the coupled voltage, torque, and power identities before assessing
    # user limits. These tolerances only cover floating-point/root residual.
    omega = 2.0 * math.pi * rpm / 60.0
    shaft_from_torque_w = torque_nm * omega
    voltage_residual = float(state["residual_v"])
    expected_battery_power_w = (motor_power_w + system_loss_w) / constraints.esc_efficiency
    balance_tolerance = max(1e-5, abs(battery_power_w) * 1e-6)
    if abs(shaft_power_w - shaft_from_torque_w) > max(1e-7, abs(shaft_power_w) * 1e-7):
        return _empty_result(
            "solver_failed",
            "PyThrust torque and shaft-power identities do not balance",
            speed_mps,
            battery_state,
            propeller,
            constraints,
            throttle,
        )
    if abs(voltage_residual) > 1e-5 or abs(battery_power_w - expected_battery_power_w) > balance_tolerance:
        return _empty_result(
            "solver_failed",
            "motor, ESC, system, and battery power balance did not close",
            speed_mps,
            battery_state,
            propeller,
            constraints,
            throttle,
        )

    reasons: list[str] = []
    if min(motor_loss_w, esc_loss_w, battery_internal_loss_w) < -balance_tolerance:
        reasons.append("invalid_power_balance")
    efficiencies = (prop_efficiency, motor_efficiency, system_efficiency)
    if any(
        not math.isfinite(value) or value < -1e-10 or value > 1.0001
        for value in efficiencies
    ):
        reasons.append("invalid_efficiency")
    if motor_current_a > min(constraints.max_motor_current_a, XING2_1404_4600KV.peak_current_a):
        reasons.append("motor_current_limit")
    if battery_current_a > constraints.max_battery_current_a or not battery_point.is_feasible:
        reasons.append("battery_current_or_voltage_limit")
    if rpm > constraints.max_rpm:
        reasons.append("rpm_limit")
    if temperature_estimate_c > constraints.max_winding_temperature_c:
        reasons.append("thermal_limit")
    if esc_temperature_estimate_c > constraints.max_esc_temperature_c:
        reasons.append("esc_thermal_limit")
    if constraints.max_shaft_power_w is not None and shaft_power_w > constraints.max_shaft_power_w:
        reasons.append("shaft_power_limit")
    if constraints.max_battery_power_w is not None and battery_power_w > constraints.max_battery_power_w:
        reasons.append("battery_power_limit")

    energy_consumed_wh: float | None = None
    final_soc: float | None = None
    if not reasons:
        try:
            mission = constraints.battery_model.integrate_power(
                state=battery_state,
                power_w=battery_power_w,
                dt_s=constraints.mission_duration_s,
                max_step_s=max(0.1, constraints.mission_duration_s / 1000.0),
            )
            energy_consumed_wh = mission.delivered_energy_wh
            final_soc = mission.final_state.soc
            if not mission.is_feasible:
                reasons.append(f"battery_energy:{mission.stop_reason}")
        except Exception as exc:
            return _empty_result(
                "solver_failed",
                f"battery mission integration failed: {type(exc).__name__}: {exc}",
                speed_mps,
                battery_state,
                propeller,
                constraints,
                throttle,
            )

    return MotorOperatingPoint(
        status="infeasible" if reasons else "feasible",
        reason=", ".join(reasons) if reasons else None,
        throttle=throttle,
        speed_mps=speed_mps,
        battery_soc=battery_state.soc,
        battery_series=constraints.battery_model.series,
        map_id=propeller.metadata.id,
        motor_scenario=constraints.motor_scenario.label,
        rpm=rpm,
        advance_ratio=j,
        ct=ct,
        cp=cp,
        thrust_n=thrust_n,
        torque_nm=torque_nm,
        shaft_power_w=shaft_power_w,
        motor_power_w=motor_power_w,
        battery_power_w=battery_power_w,
        motor_current_a=motor_current_a,
        battery_current_a=battery_current_a,
        motor_voltage_v=motor_voltage_v,
        battery_voltage_v=battery_voltage_v,
        propeller_efficiency=prop_efficiency,
        motor_efficiency=motor_efficiency,
        system_efficiency=system_efficiency,
        motor_loss_w=motor_loss_w,
        system_loss_w=system_loss_w,
        esc_loss_w=esc_loss_w,
        battery_c_rate=float(battery_point.c_rate),
        battery_open_circuit_voltage_v=battery_open_circuit_voltage_v,
        battery_internal_loss_w=battery_internal_loss_w,
        motor_temperature_estimate_c=temperature_estimate_c,
        esc_temperature_estimate_c=esc_temperature_estimate_c,
        energy_consumed_wh=energy_consumed_wh,
        final_soc=final_soc,
    )


def max_feasible_operating_point(
    speed: float,
    battery_state: BatteryState,
    propeller: PropellerMap,
    constraints: OperatingConstraints,
) -> MotorOperatingPoint:
    """Return the highest-thrust feasible point found by a sampled throttle sweep.

    The configured throttle grid is scanned, and every sampled feasible/infeasible
    boundary is bisected. This is not a global optimizer for arbitrary
    non-monotonic maps; the returned result is a repeatable constrained screen.
    """
    points = [
        solve_operating_point(speed, battery_state, propeller, constraints, throttle)
        for throttle in constraints.throttle_values
    ]
    # Refine every sampled feasible/infeasible transition. This closes the
    # boundary gap without pretending the finite sweep found a continuous
    # global optimum across arbitrary non-monotonic user maps.
    boundary_samples = tuple(points)
    for lower_point, upper_point in zip(boundary_samples, boundary_samples[1:]):
        lower_is_feasible = lower_point.status == "feasible"
        upper_is_feasible = upper_point.status == "feasible"
        if lower_is_feasible == upper_is_feasible:
            continue
        low, high = lower_point.throttle, upper_point.throttle
        for _ in range(20):
            midpoint = (low + high) / 2.0
            midpoint_point = solve_operating_point(
                speed, battery_state, propeller, constraints, midpoint
            )
            points.append(midpoint_point)
            midpoint_is_feasible = midpoint_point.status == "feasible"
            if midpoint_is_feasible == lower_is_feasible:
                low = midpoint
            else:
                high = midpoint
    feasible = [point for point in points if point.status == "feasible"]
    if feasible:
        return max(feasible, key=lambda point: (point.thrust_n or 0.0, point.throttle))
    solved = [point for point in points if point.thrust_n is not None]
    if solved:
        return max(solved, key=lambda point: (point.thrust_n or 0.0, point.throttle))
    # A sweep can contain multiple diagnostic classes; prefer a physical
    # coverage result, then a solver failure, and preserve the reason text.
    return next((p for p in points if p.status == "out_of_map"), points[0])


def sweep_uncertainty_envelope(
    speed_mps: float,
    battery_state: BatteryState,
    propeller: PropellerMap,
    constraints: OperatingConstraints,
    scenarios: Sequence[MotorElectricalScenario],
) -> OperatingEnvelope:
    """Evaluate per-scenario maximum-feasible points and report their envelope."""
    if not scenarios:
        raise ValueError("at least one explicit motor sensitivity scenario is required")
    points = [
        max_feasible_operating_point(
            speed_mps,
            battery_state,
            propeller,
            replace(constraints, motor_scenario=scenario),
        )
        for scenario in scenarios
    ]
    feasible = [point for point in points if point.status == "feasible"]
    return OperatingEnvelope(
        scenario_count=len(points),
        feasible_scenario_count=len(feasible),
        thrust_min_n=_minimum(point.thrust_n for point in feasible),
        thrust_max_n=_maximum(point.thrust_n for point in feasible),
        shaft_power_min_w=_minimum(point.shaft_power_w for point in feasible),
        shaft_power_max_w=_maximum(point.shaft_power_w for point in feasible),
        battery_power_min_w=_minimum(point.battery_power_w for point in feasible),
        battery_power_max_w=_maximum(point.battery_power_w for point in feasible),
        statuses=tuple(point.status for point in points),
    )


def export_propeller_map_cli(argv: Sequence[str] | None = None) -> tuple[Path, Path]:
    """Validate a coefficient CSV and export its canonical CSV/metadata pair."""
    parser = argparse.ArgumentParser(description="Validate and export a PyThrust propeller coefficient map")
    parser.add_argument("input_csv")
    parser.add_argument("metadata_json")
    parser.add_argument("output_directory")
    args = parser.parse_args(argv)
    source = Path(args.input_csv)
    metadata_raw = json.loads(Path(args.metadata_json).read_text(encoding="utf-8"))
    metadata_path = Path(args.metadata_json)
    # The supplied source hash is carried into the exported map manifest.
    metadata = PropellerMapMetadata(
        id=str(metadata_raw["id"]),
        manufacturer=str(metadata_raw["manufacturer"]),
        model=str(metadata_raw["model"]),
        diameter_m=float(metadata_raw["diameter_m"]),
        pitch_m=float(metadata_raw["pitch_m"]),
        blade_count=_positive_integer("blade_count", metadata_raw["blade_count"]),
        source_uri=str(metadata_raw["source_uri"]),
        source_description=str(metadata_raw["source_description"]),
        source_sha256=_sha256_file(source),
    )
    raw = PropellerMap.from_files(source, metadata_path)
    return replace(raw, metadata=metadata).export(args.output_directory)


def main(argv: Sequence[str] | None = None) -> int:
    """Command-line map exporter and constrained operating-point sweep."""
    parser = argparse.ArgumentParser(description="Propeller map and XING2 operating-point tools")
    commands = parser.add_subparsers(dest="command", required=True)
    export_parser = commands.add_parser("export-map", help="validate and normalize a coefficient CSV")
    export_parser.add_argument("input_csv")
    export_parser.add_argument("metadata_json")
    export_parser.add_argument("output_directory")
    sweep_parser = commands.add_parser("sweep", help="run a constrained throttle and uncertainty sweep")
    sweep_parser.add_argument("--map-csv", required=True)
    sweep_parser.add_argument("--map-metadata", required=True)
    sweep_parser.add_argument("--battery-cell-json", required=True)
    sweep_parser.add_argument("--series", type=int, choices=(3, 4), required=True)
    sweep_parser.add_argument("--parallel", type=int, default=1)
    sweep_parser.add_argument("--soc", type=float, required=True)
    sweep_parser.add_argument("--speed-mps", type=float, required=True)
    sweep_parser.add_argument("--constraints-json", required=True)
    sweep_parser.add_argument("--output-directory", required=True)
    args = parser.parse_args(argv)
    if args.command == "export-map":
        csv_path, metadata_path = export_propeller_map_cli(
            [args.input_csv, args.metadata_json, args.output_directory]
        )
        print(f"wrote {csv_path}")
        print(f"wrote {metadata_path}")
        return 0

    propeller = PropellerMap.from_files(args.map_csv, args.map_metadata)
    battery = RateMapBattery.from_json(args.battery_cell_json, series=args.series, parallel=args.parallel)
    state = BatteryState(soc=args.soc)
    raw_constraints = json.loads(Path(args.constraints_json).read_text(encoding="utf-8"))
    scenarios_raw = raw_constraints.pop("motor_scenarios", None)
    constraints = _constraints_from_json(raw_constraints, battery)
    scenarios = (
        tuple(_scenario_from_json(value) for value in scenarios_raw)
        if scenarios_raw
        else (constraints.motor_scenario,)
    )
    points: list[MotorOperatingPoint] = []
    for scenario in scenarios:
        scenario_constraints = replace(constraints, motor_scenario=scenario)
        scenario_points = [
            solve_operating_point(args.speed_mps, state, propeller, scenario_constraints, throttle)
            for throttle in constraints.throttle_values
        ]
        points.extend(scenario_points)
        maximum = max_feasible_operating_point(
            args.speed_mps, state, propeller, scenario_constraints
        )
        if not any(
            point.status == maximum.status
            and math.isclose(point.throttle, maximum.throttle, rel_tol=0.0, abs_tol=1e-12)
            for point in scenario_points
        ):
            points.append(maximum)
    envelopes = {
        "all_motor_scenarios": _envelope_dict(
            sweep_uncertainty_envelope(
                args.speed_mps,
                state,
                propeller,
                constraints,
                scenarios,
            )
        ),
        "by_scenario": {
            scenario.label: _envelope_dict(
                sweep_uncertainty_envelope(
                    args.speed_mps,
                    state,
                    propeller,
                    replace(constraints, motor_scenario=scenario),
                    (scenario,),
                )
            )
            for scenario in scenarios
        },
    }
    _write_sweep_outputs(
        Path(args.output_directory), propeller, battery, state, args.speed_mps,
        points, envelopes, constraints, scenarios, Path(args.battery_cell_json),
    )
    return 0


def _write_sweep_outputs(
    directory: Path,
    propeller: PropellerMap,
    battery: RateMapBattery,
    state: BatteryState,
    speed_mps: float,
    points: Sequence[MotorOperatingPoint],
    envelopes: dict[str, dict[str, object]],
    constraints: OperatingConstraints,
    scenarios: Sequence[MotorElectricalScenario],
    battery_json_path: Path,
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    csv_path = directory / "operating_points.csv"
    metadata_path = directory / "operating_points.json"
    if csv_path.exists() or metadata_path.exists():
        raise FileExistsError(f"refusing to overwrite operating-point outputs in {directory}")
    fields = list(MotorOperatingPoint.__dataclass_fields__)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for point in points:
            writer.writerow(point.__dict__)
    manifest = {
        "schema_version": 1,
        "propeller_map_id": propeller.metadata.id,
        "propeller_map_source_sha256": propeller.metadata.source_sha256,
        "propeller_map_data_sha256": propeller.data_sha256,
        "battery_name": battery.name,
        "battery_source": battery.source,
        "battery_input_sha256": _sha256_file(battery_json_path),
        "series": battery.series,
        "parallel": battery.parallel,
        "initial_soc": state.soc,
        "speed_mps": speed_mps,
        "pythrust": {"version": pythrust.__version__, "revision": PYTHRUST_REVISION},
        "motor_manufacturer_data": {
            "kv_rpm_per_v": XING2_1404_4600KV.kv_rpm_per_v,
            "interphase_resistance_ohm": XING2_1404_4600KV.interphase_resistance_ohm,
            "no_load_current_a": XING2_1404_4600KV.no_load_current_a,
            "peak_current_a": XING2_1404_4600KV.peak_current_a,
            "sustained_current_limit_a": XING2_1404_4600KV.sustained_current_limit_a,
            "winding_temperature_limit_c": XING2_1404_4600KV.winding_temperature_limit_c,
            "max_reported_test_power_w": XING2_1404_4600KV.max_test_power_w,
            "test_voltage_v": XING2_1404_4600KV.tested_voltage_v,
            "test_temperature_c": XING2_1404_4600KV.test_temperature_c,
            "source_pdf_url": XING2_1404_4600KV.source_pdf_url,
            "source_pdf_sha256": XING2_1404_4600KV.source_pdf_sha256,
            "scope": XING2_1404_4600KV.source_scope,
        },
        "motor_scenarios": [
            {
                "label": scenario.label,
                "effective_resistance_ohm": scenario.effective_resistance_ohm,
                "no_load_current_a": scenario.no_load_current_a,
                "assumption_note": scenario.assumption_note,
            }
            for scenario in scenarios
        ],
        "constraints": {
            "system_resistance_ohm": constraints.system_resistance_ohm,
            "esc_efficiency": constraints.esc_efficiency,
            "max_motor_current_a": constraints.max_motor_current_a,
            "max_battery_current_a": constraints.max_battery_current_a,
            "max_rpm": constraints.max_rpm,
            "max_winding_temperature_c": constraints.max_winding_temperature_c,
            "motor_thermal_resistance_c_per_w": constraints.motor_thermal_resistance_c_per_w,
            "max_esc_temperature_c": constraints.max_esc_temperature_c,
            "esc_thermal_resistance_c_per_w": constraints.esc_thermal_resistance_c_per_w,
            "ambient_temperature_c": constraints.ambient_temperature_c,
            "mission_duration_s": constraints.mission_duration_s,
            "throttle_values": list(constraints.throttle_values),
            "rho_kg_m3": constraints.rho_kg_m3,
            "max_shaft_power_w": constraints.max_shaft_power_w,
            "max_battery_power_w": constraints.max_battery_power_w,
        },
        "source_notes": {
            "motor": XING2_1404_4600KV.source_scope,
            "thermal_model": "caller-specified steady-state screening; not manufacturer-calibrated",
            "flight_readiness": "not established by this analysis",
        },
        "uncertainty_envelopes": envelopes,
    }
    metadata_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _constraints_from_json(raw: dict[str, object], battery: RateMapBattery) -> OperatingConstraints:
    scenario_raw = raw.pop("motor_scenario")
    if not isinstance(scenario_raw, dict):
        raise ValueError("constraints JSON requires a motor_scenario object")
    return OperatingConstraints(
        motor_scenario=_scenario_from_json(scenario_raw),
        battery_model=battery,
        system_resistance_ohm=float(raw["system_resistance_ohm"]),
        esc_efficiency=float(raw["esc_efficiency"]),
        max_motor_current_a=float(raw["max_motor_current_a"]),
        max_battery_current_a=float(raw["max_battery_current_a"]),
        max_rpm=float(raw["max_rpm"]),
        max_winding_temperature_c=float(raw["max_winding_temperature_c"]),
        motor_thermal_resistance_c_per_w=float(raw["motor_thermal_resistance_c_per_w"]),
        max_esc_temperature_c=float(raw["max_esc_temperature_c"]),
        esc_thermal_resistance_c_per_w=float(raw["esc_thermal_resistance_c_per_w"]),
        ambient_temperature_c=float(raw["ambient_temperature_c"]),
        mission_duration_s=float(raw["mission_duration_s"]),
        throttle_values=tuple(
            float(value) for value in raw.get("throttle_values", DEFAULT_THROTTLE_VALUES)
        ),
        rho_kg_m3=float(raw.get("rho_kg_m3", 1.225)),
        max_shaft_power_w=_optional_float(raw.get("max_shaft_power_w")),
        max_battery_power_w=_optional_float(raw.get("max_battery_power_w")),
    )


def _scenario_from_json(raw: dict[str, object]) -> MotorElectricalScenario:
    return MotorElectricalScenario(
        label=str(raw["label"]),
        effective_resistance_ohm=float(raw["effective_resistance_ohm"]),
        no_load_current_a=float(raw["no_load_current_a"]),
        assumption_note=str(raw["assumption_note"]),
    )


def _empty_result(
    status: OperatingStatus,
    reason: str,
    speed_mps: float,
    battery_state: BatteryState,
    propeller: PropellerMap,
    constraints: OperatingConstraints,
    throttle: float,
) -> MotorOperatingPoint:
    return MotorOperatingPoint(
        status=status,
        reason=reason,
        throttle=throttle,
        speed_mps=speed_mps,
        battery_soc=battery_state.soc,
        battery_series=constraints.battery_model.series,
        map_id=propeller.metadata.id,
        motor_scenario=constraints.motor_scenario.label,
    )


def _bracketing_levels(levels: Sequence[float], rpm: float) -> tuple[float, ...]:
    for index, level in enumerate(levels):
        if rpm == level:
            return (level,)
        if rpm < level:
            return (levels[index - 1], level)
    return (levels[-1],)


def _finite_float(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _optional_float(value: object) -> float | None:
    return None if value is None else float(value)


def _positive_integer(name: str, value: object) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a positive integer")
    try:
        numeric = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a positive integer") from exc
    if not math.isfinite(numeric) or numeric < 1 or not numeric.is_integer():
        raise ValueError(f"{name} must be a positive integer")
    return int(numeric)


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(character in "0123456789abcdefABCDEF" for character in value)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_map_csv_bytes(rows: Sequence[tuple[float, float, float, float]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(PROP_MAP_COLUMNS)
    for row in rows:
        writer.writerow([format(value, ".12g") for value in row])
    return stream.getvalue().encode("utf-8")


def _minimum(values: Iterable[float | None]) -> float | None:
    numbers = [float(value) for value in values if value is not None]
    return min(numbers) if numbers else None


def _maximum(values: Iterable[float | None]) -> float | None:
    numbers = [float(value) for value in values if value is not None]
    return max(numbers) if numbers else None


def _envelope_dict(envelope: OperatingEnvelope) -> dict[str, object]:
    return {
        "scenario_count": envelope.scenario_count,
        "feasible_scenario_count": envelope.feasible_scenario_count,
        "thrust_n": [envelope.thrust_min_n, envelope.thrust_max_n],
        "shaft_power_w": [envelope.shaft_power_min_w, envelope.shaft_power_max_w],
        "battery_power_w": [envelope.battery_power_min_w, envelope.battery_power_max_w],
        "statuses": list(envelope.statuses),
    }


if __name__ == "__main__":
    raise SystemExit(main())
