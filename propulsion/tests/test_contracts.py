from __future__ import annotations

import unittest

from propulsion.coefficients import coefficients_from_loads, loads_from_coefficients
from propulsion.models import (
    BladeStation,
    GeometryProvenance,
    OperatingCondition,
    PolarProvenance,
    PropellerGeometry,
    RotorLoads,
    SourceConventions,
)


def reference_geometry(**overrides) -> PropellerGeometry:
    values = {
        "id": "fixture-3x2",
        "source_uri": "fixture://needletail/fixture-3x2",
        "sha256": "a" * 64,
        "blade_count": 2,
        "diameter_m": 0.0762,
        "hub_radius_m": 0.006,
        "stations": (
            BladeStation(0.012, 0.010, 0.4, "synthetic-section"),
            BladeStation(0.035, 0.007, 0.1, "synthetic-section"),
        ),
        "geometry_provenance": GeometryProvenance.MEASURED,
        "polar_provenance": PolarProvenance.EXPERIMENTAL,
        "source_conventions": SourceConventions(
            rotor_axis="+X", blade_hand="CW", section_sign="positive_pitch",
            frame="aircraft_body", units="SI",
        ),
    }
    values.update(overrides)
    return PropellerGeometry(**values)


def reference_condition(**overrides) -> OperatingCondition:
    values = {
        "rpm": 12_000.0,
        "axial_airspeed_mps": 10.0,
        "rho_kg_m3": 1.225,
        "dynamic_viscosity_pa_s": 1.81e-5,
        "temperature_k": 288.15,
        "pressure_pa": 101_325.0,
        "rotation_sign": 1,
    }
    values.update(overrides)
    return OperatingCondition(**values)


class PropulsionContractTests(unittest.TestCase):
    def test_coefficient_conversion_round_trips_dimensional_loads(self):
        geometry = reference_geometry()
        condition = reference_condition()
        loads = loads_from_coefficients(0.10, 0.05, geometry, condition)

        coefficients = coefficients_from_loads(loads, geometry, condition)

        self.assertAlmostEqual(coefficients.ct, 0.10)
        self.assertAlmostEqual(coefficients.cp, 0.05)
        self.assertAlmostEqual(coefficients.advance_ratio, 10.0 / (200.0 * 0.0762))
        self.assertAlmostEqual(loads.shaft_power_w, loads.torque_nm * 2.0 * 3.141592653589793 * 200.0)
        self.assertIsNotNone(coefficients.efficiency)

    def test_windmilling_signs_are_preserved_and_efficiency_is_undefined(self):
        geometry = reference_geometry()
        condition = reference_condition()
        loads = loads_from_coefficients(-0.02, -0.01, geometry, condition)

        coefficients = coefficients_from_loads(loads, geometry, condition)

        self.assertLess(loads.thrust_n, 0.0)
        self.assertLess(loads.shaft_power_w, 0.0)
        self.assertLess(coefficients.ct, 0.0)
        self.assertLess(coefficients.cp, 0.0)
        self.assertIsNone(coefficients.efficiency)

    def test_static_zero_advance_does_not_claim_propulsive_efficiency(self):
        geometry = reference_geometry()
        condition = reference_condition(axial_airspeed_mps=0.0)
        loads = loads_from_coefficients(0.10, 0.05, geometry, condition)

        coefficients = coefficients_from_loads(loads, geometry, condition)

        self.assertEqual(coefficients.advance_ratio, 0.0)
        self.assertIsNone(coefficients.efficiency)

    def test_zero_rpm_returns_typed_undefined_normalization(self):
        result = coefficients_from_loads(
            RotorLoads(thrust_n=1.0, torque_nm=0.1, shaft_power_w=1.0),
            reference_geometry(),
            reference_condition(rpm=0.0),
        )

        self.assertIsNone(result.ct)
        self.assertIsNone(result.cp)
        self.assertIsNone(result.advance_ratio)
        self.assertIsNone(result.efficiency)
        self.assertFalse(result.normalization_defined)
        self.assertIn("zero_rpm", result.warnings)

    def test_packaging_only_geometry_cannot_enter_predictive_models(self):
        geometry = reference_geometry(
            geometry_provenance=GeometryProvenance.PACKAGING_ONLY,
            stations=(),
        )

        with self.assertRaisesRegex(ValueError, "packaging-only"):
            geometry.validate_for_prediction()

    def test_geometry_rejects_noninteger_blade_count(self):
        with self.assertRaisesRegex(ValueError, "positive integer"):
            reference_geometry(blade_count=2.5)

    def test_operating_condition_rejects_invalid_speed_or_rotation_sign(self):
        with self.assertRaises(ValueError):
            reference_condition(rpm=-1.0)
        with self.assertRaises(ValueError):
            reference_condition(rotation_sign=0)


if __name__ == "__main__":
    unittest.main()
