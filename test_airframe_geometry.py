from dataclasses import replace
from math import sqrt
from pathlib import Path
import unittest

from cadgen import build123d as bd

from airframe_geometry import (
    PARAMETERS,
    _section_coordinates,
    _split_airfoil,
    build_airframe_oml,
    build_components,
)
from profiles.airfoil_coordinates import SD7003_COORDINATES
from propeller_envelope import build_propeller_envelopes


class AirframeParameterTests(unittest.TestCase):
    def test_arm_spans_follow_motor_offsets_and_pod_radius(self):
        params = replace(
            PARAMETERS,
            horizontal_motor_offset=95.0,
            vertical_motor_offset=80.0,
        )

        self.assertEqual(params.wing_span, 168.0)
        self.assertEqual(params.support_span, 138.0)
        params.validate_arm_pod_fit()

    def test_current_propeller_clearances_have_a_positive_margin(self):
        clearances = PARAMETERS.propeller_clearance_report

        self.assertAlmostEqual(clearances["pod_noses"], 1.5, delta=0.05)
        # True three-dimensional distance is much smaller than a spanwise estimate.
        self.assertGreater(clearances["horizontal_wing"], 0.0)
        self.assertLess(clearances["horizontal_wing"], 1.3)
        self.assertGreaterEqual(clearances["horizontal_wing"], PARAMETERS.propeller_clearance_margin)
        self.assertAlmostEqual(clearances["vertical_support"], 3.5, delta=0.05)
        self.assertAlmostEqual(
            clearances["neighboring_propellers"],
            sqrt(100.0**2 + 85.0**2) - PARAMETERS.propeller_diameter,
        )
        PARAMETERS.validate_propeller_clearance()

    def test_larger_propeller_that_intersects_wing_is_rejected(self):
        params = replace(PARAMETERS, propeller_diameter=100.0)

        with self.assertRaisesRegex(ValueError, "horizontal_wing"):
            params.validate_propeller_clearance()

    def test_cad_min_distance_detects_actual_disk_penetration(self):
        params = replace(PARAMETERS, propeller_diameter=100.0)
        clearances = params.propeller_clearance_report
        self.assertAlmostEqual(clearances["horizontal_wing"], 0.0, delta=1e-5)
        self.assertRaises(ValueError, params.validate_propeller_clearance)

    def test_larger_clearance_requirement_is_rejected(self):
        params = replace(PARAMETERS, propeller_clearance_margin=2.0)
        with self.assertRaisesRegex(ValueError, "horizontal_wing|pod_noses"):
            params.validate_propeller_clearance()

    def test_propeller_envelopes_use_the_shared_diameter_parameter(self):
        params = replace(PARAMETERS, propeller_diameter=70.0)
        bounds = build_propeller_envelopes(params).bounding_box()

        self.assertAlmostEqual(bounds.max.Y, 135.0)
        self.assertAlmostEqual(bounds.min.Y, -135.0)
        self.assertAlmostEqual(bounds.max.Z, 120.0)
        self.assertAlmostEqual(bounds.min.Z, -120.0)

    def test_aft_propeller_plane_that_intersects_lifting_surfaces_is_rejected(self):
        params = replace(PARAMETERS, motor_station_x=-5.0)

        with self.assertRaisesRegex(ValueError, "horizontal_wing|vertical_support"):
            params.validate_propeller_clearance()

    def test_planform_areas_follow_the_linear_ruled_loft(self):
        self.assertAlmostEqual(PARAMETERS.wing_planform_area, 8455.0)
        self.assertAlmostEqual(PARAMETERS.support_planform_area, 5624.0)

    def test_motor_pod_validator_accepts_reference_fit_and_rejects_small_profile(self):
        PARAMETERS.validate_motor_prop_interface()
        reduced_profile = tuple(
            (station, ratio * 0.7)
            for station, ratio in PARAMETERS.motor_pod_radius_profile
        )
        params = replace(PARAMETERS, motor_pod_radius_profile=reduced_profile)

        with self.assertRaisesRegex(ValueError, "radial allowance"):
            params.validate_motor_prop_interface()


class AirfoilGeometryTests(unittest.TestCase):
    def test_leading_edge_is_normalized_without_clamping_source_points(self):
        upper, lower = _split_airfoil(SD7003_COORDINATES)

        self.assertAlmostEqual(upper[0][0], 0.0)
        self.assertAlmostEqual(lower[0][0], 0.0)
        self.assertAlmostEqual(upper[-1][0], 1.0, places=4)
        self.assertAlmostEqual(lower[-1][0], 1.0, places=4)

    def test_physical_trailing_edge_thickness_and_leading_edge_are_preserved(self):
        upper, lower = _section_coordinates("SD7003", 60.0, 0.30)

        self.assertAlmostEqual(upper[0][0], 0.0)
        self.assertAlmostEqual(lower[0][0], 0.0)
        self.assertAlmostEqual((upper[-1][1] - lower[-1][1]) * 60.0, 0.30)

    def test_nonfinite_or_materially_out_of_range_airfoil_data_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            _split_airfoil(((1.0, 0.0), (0.5, float("nan")), (0.0, 0.0), (1.0, 0.0)))
        with self.assertRaisesRegex(ValueError, "between 0 and 1"):
            _split_airfoil(((1.01, 0.0), (0.5, 0.1), (0.0, 0.0), (1.0, -0.1)))


class GeneratedGeometryTests(unittest.TestCase):
    def test_arm_tip_stations_match_derived_spans(self):
        components = build_components()
        wing_bounds = components[1].bounding_box()
        support_bounds = components[2].bounding_box()

        self.assertAlmostEqual(wing_bounds.min.Y, -PARAMETERS.wing_span / 2, places=5)
        self.assertAlmostEqual(wing_bounds.max.Y, PARAMETERS.wing_span / 2, places=5)
        self.assertAlmostEqual(support_bounds.min.Z, -PARAMETERS.support_span / 2, places=5)
        self.assertAlmostEqual(support_bounds.max.Z, PARAMETERS.support_span / 2, places=5)

    def test_oml_remains_one_solid(self):
        self.assertEqual(len(build_airframe_oml().solids()), 1)


class GeneratedStepExportTests(unittest.TestCase):
    EXPECTED_SOLID_COUNTS = {
        "airframe_oml.step": 1,
        "airframe.step": 7,
        "propeller_envelope.step": 4,
        "motor_prop_installation.step": 9,
    }

    def test_step_exports_reimport_as_valid_shapes_with_expected_solids(self):
        root = Path(__file__).resolve().parent
        for filename, expected_solid_count in self.EXPECTED_SOLID_COUNTS.items():
            with self.subTest(filename=filename):
                step_path = root / filename
                self.assertTrue(step_path.is_file(), f"Missing STEP export: {filename}")
                self.assertGreater(step_path.stat().st_size, 0)
                imported_shape = bd.import_step(str(step_path))
                self.assertTrue(imported_shape.is_valid, f"Invalid STEP shape: {filename}")
                self.assertEqual(
                    len(imported_shape.solids()),
                    expected_solid_count,
                    f"Unexpected solid count in {filename}",
                )


if __name__ == "__main__":
    unittest.main()
