from dataclasses import replace
from importlib.util import find_spec
from math import sqrt
from pathlib import Path
import unittest

from cadgen import build123d as bd

from airframe_geometry import (
    AIRFOILS,
    PARAMETERS,
    _fuselage,
    build_motor_pod,
    fuselage_radius_stations,
    motor_pod_radius_stations,
    _section_coordinates,
    _split_airfoil,
    build_airframe_oml,
    build_components,
)
from profiles.airfoil_coordinates import SD7003_COORDINATES
from propeller_envelope import build_propeller_envelopes


class AirframeParameterTests(unittest.TestCase):
    def test_shared_offset_preserves_local_clearances_and_moves_interfaces(self):
        reference = replace(PARAMETERS, assembly_x_offset=0.0)
        shifted = replace(reference, assembly_x_offset=-12.0)
        self.assertEqual(reference.propeller_clearance_report, shifted.propeller_clearance_report)
        for property_name in (
            "motor_pod_center_x", "motor_mount_face_x", "wing_tip_leading_edge_x",
            "prop_hub_center_x", "motor_shaft_tip_x",
        ):
            self.assertAlmostEqual(getattr(shifted, property_name) - getattr(reference, property_name), -12.0)
        shifted.validate_motor_prop_interface()
        shifted.validate_propeller_clearance()

    def test_arm_spans_reach_motor_pod_centerlines(self):
        params = replace(
            PARAMETERS,
            horizontal_motor_offset=95.0,
            vertical_motor_offset=80.0,
        )

        self.assertEqual(params.wing_span, 190.0)
        self.assertEqual(params.support_span, 160.0)
        self.assertEqual(params.wing_span / 2, params.horizontal_motor_offset)
        self.assertEqual(params.support_span / 2, params.vertical_motor_offset)
        params.validate_arm_pod_fit()

    def test_current_propeller_clearances_have_a_positive_margin(self):
        clearances = PARAMETERS.propeller_clearance_report

        self.assertAlmostEqual(clearances["pod_mount_face"], 18.0)
        self.assertAlmostEqual(clearances["horizontal_wing"], 23.0)
        self.assertAlmostEqual(clearances["vertical_support"], 31.0)
        self.assertAlmostEqual(
            clearances["neighboring_propellers"],
            sqrt(95.0**2 + 80.0**2) - PARAMETERS.propeller_diameter,
        )
        PARAMETERS.validate_propeller_clearance()

    def test_oversized_propeller_that_intersects_neighboring_disks_is_rejected(self):
        params = replace(PARAMETERS, propeller_diameter=140.0)

        with self.assertRaisesRegex(ValueError, "neighboring_propellers"):
            params.validate_propeller_clearance()

    def test_propeller_envelopes_use_the_shared_diameter_parameter(self):
        params = replace(PARAMETERS, propeller_diameter=70.0)
        bounds = build_propeller_envelopes(params).bounding_box()

        self.assertAlmostEqual(bounds.max.Y, 130.0)
        self.assertAlmostEqual(bounds.min.Y, -130.0)
        self.assertAlmostEqual(bounds.max.Z, 115.0)
        self.assertAlmostEqual(bounds.min.Z, -115.0)

    def test_aft_propeller_plane_that_intersects_lifting_surfaces_is_rejected(self):
        params = replace(PARAMETERS, motor_station_x=-40.0)

        with self.assertRaisesRegex(ValueError, "horizontal_wing|vertical_support"):
            params.validate_propeller_clearance()

    def test_planform_areas_follow_the_linear_ruled_loft(self):
        self.assertAlmostEqual(PARAMETERS.wing_planform_area, 12920.0)
        self.assertAlmostEqual(PARAMETERS.support_planform_area, 10240.0)

    def test_motor_pod_validator_accepts_reference_fit_and_rejects_small_pod_face(self):
        PARAMETERS.validate_motor_prop_interface()
        params = replace(PARAMETERS, motor_pod_diameter=18.0)

        with self.assertRaisesRegex(ValueError, "flat motor mount face"):
            params.validate_motor_prop_interface()

    def test_further_aft_pod_motion_beyond_layout_target_is_rejected(self):
        PARAMETERS.validate_motor_prop_interface()
        params = replace(PARAMETERS, motor_station_x=0.0)
        with self.assertRaisesRegex(ValueError, "motor mount must be at least"):
            params.validate_motor_prop_interface()


class AirfoilGeometryTests(unittest.TestCase):
    def test_thickness_scaling_preserves_camber_and_physical_trailing_edge(self):
        upper, lower = _section_coordinates("NACA2415", 56.0, 1.2)
        thick_upper, thick_lower = _section_coordinates(
            "NACA2415", 56.0, 1.2, thickness_scale=1.2,
        )
        for u, l, tu, tl in zip(upper, lower, thick_upper, thick_lower):
            self.assertAlmostEqual(u[0], tu[0])
            self.assertAlmostEqual((u[1] + l[1]) / 2, (tu[1] + tl[1]) / 2)
            if 0 < u[0] <= 0.94:
                self.assertAlmostEqual(tu[1] - tl[1], 1.2 * (u[1] - l[1]))
        self.assertAlmostEqual((thick_upper[-1][1] - thick_lower[-1][1]) * 56.0, 1.2)

    def test_invalid_thickness_scales_are_rejected(self):
        for scale in (0.0, -1.0, float("nan"), float("inf")):
            with self.subTest(scale=scale), self.assertRaisesRegex(ValueError, "thickness scale"):
                _section_coordinates("NACA2415", 56.0, 1.2, thickness_scale=scale)

    def test_catalog_sections_have_shared_noses_and_positive_interior_thickness(self):
        for name in AIRFOILS:
            with self.subTest(airfoil=name):
                upper, lower = _section_coordinates(name, 44.0, 0.30)
                self.assertEqual(upper[0], lower[0])
                self.assertTrue(all(u[1] > l[1] for u, l in zip(upper[1:], lower[1:])))

    def test_positive_y_leading_edge_is_shared_by_both_surfaces(self):
        # A cambered Selig contour can turn around before crossing y=0.
        contour = ((1.0, 0.0), (0.3, 0.12), (0.0, 0.001),
                   (0.01, -0.01), (0.3, -0.04), (1.0, 0.0))
        upper, lower = _split_airfoil(contour)
        self.assertEqual(upper[0], lower[0])
        self.assertEqual(upper[0], (0.0, 0.001))

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
    def test_refined_fuselage_retains_envelope_volume_with_less_surface_area(self):
        # Retain the historical custom-profile refinement as a selectable reference.
        body = _fuselage(replace(PARAMETERS, fuselage_body_airfoil=None))
        bounds = body.bounding_box()
        self.assertAlmostEqual(bounds.size.X, 230.0, places=5)
        self.assertLessEqual(bounds.size.Y, 60.25)
        self.assertLessEqual(bounds.size.Z, 60.25)
        self.assertGreaterEqual(body.volume, 326523.0)
        self.assertLess(body.area, 28764.0)

    def test_airfoil_body_templates_preserve_interfaces_and_envelopes(self):
        for name in ("NACA0015", "SD8020"):
            with self.subTest(airfoil=name):
                params = replace(PARAMETERS, fuselage_body_airfoil=name, motor_pod_body_airfoil=name)
                params.validate_motor_prop_interface()
                body, pod = _fuselage(params), build_motor_pod(params)
                self.assertTrue(body.is_valid and pod.is_valid)
                self.assertAlmostEqual(body.bounding_box().size.X, 230.0, places=5)
                self.assertLessEqual(body.bounding_box().size.Y, 60.25)
                self.assertGreaterEqual(body.volume, 326523.0)
                # The interpolated rounded cap can bulge aft by <0.001 mm.
                self.assertAlmostEqual(pod.bounding_box().min.X, -62.0, places=3)
                self.assertAlmostEqual(pod.bounding_box().max.X, 44.0, places=5)
                aft = next(f for f in pod.faces() if abs(f.bounding_box().min.X + 62) < 1e-5
                           and abs(f.bounding_box().max.X + 62) < 1e-5)
                self.assertAlmostEqual(aft.bounding_box().size.Y, 3.0, places=5)

    def test_cambered_body_template_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "symmetric"):
            _fuselage(replace(PARAMETERS, fuselage_body_airfoil="NACA2415"))

    def test_shared_offset_moves_all_arms_and_pods_but_keeps_fuselage_fixed(self):
        reference = replace(PARAMETERS, assembly_x_offset=0.0)
        before = build_components(reference)
        after = build_components(replace(reference, assembly_x_offset=-7.0))
        for index, (original, shifted) in enumerate(zip(before, after)):
            with self.subTest(component=index):
                expected_dx = 0.0 if index == 0 else -7.0
                a, b = original.bounding_box(), shifted.bounding_box()
                for endpoint in ("min", "max"):
                    av, bv = getattr(a, endpoint), getattr(b, endpoint)
                    self.assertAlmostEqual(bv.X - av.X, expected_dx, places=5)
                    self.assertAlmostEqual(bv.Y, av.Y, places=5)
                    self.assertAlmostEqual(bv.Z, av.Z, places=5)
                self.assertAlmostEqual(original.volume, shifted.volume, places=4)

    def test_arm_tip_stations_match_derived_spans(self):
        components = build_components()
        wing_bounds = components[1].bounding_box()
        support_bounds = components[2].bounding_box()

        self.assertAlmostEqual(wing_bounds.min.Y, -PARAMETERS.horizontal_motor_offset, places=5)
        self.assertAlmostEqual(wing_bounds.max.Y, PARAMETERS.horizontal_motor_offset, places=5)
        self.assertAlmostEqual(support_bounds.min.Z, -PARAMETERS.vertical_motor_offset, places=5)
        self.assertAlmostEqual(support_bounds.max.Z, PARAMETERS.vertical_motor_offset, places=5)

    def test_oml_remains_one_solid(self):
        self.assertEqual(len(build_airframe_oml().solids()), 1)


@unittest.skipUnless(find_spec("aerosandbox"), "Optional aero dependencies are not installed")
class AeroGeometryAdapterTests(unittest.TestCase):
    def test_body_airfoil_meridians_are_shared_between_cad_and_aero(self):
        from whole_aircraft_aero import _fuselage_profile, _pod_profile, build_airplane

        for name in ("NACA0015", "SD8020"):
            with self.subTest(airfoil=name):
                params = replace(PARAMETERS, fuselage_body_airfoil=name, motor_pod_body_airfoil=name)
                self.assertEqual(_fuselage_profile(params), fuselage_radius_stations(params))
                self.assertEqual(_pod_profile(params), motor_pod_radius_stations(params))
                plane = build_airplane(params, body_sample_count=101)
                self.assertEqual(len(plane.fuselages[0].xsecs), 101)
                for section in (plane.fuselages[0].xsecs[0], plane.fuselages[0].xsecs[-1]):
                    self.assertAlmostEqual(section.width / 2, params.fuselage_end_cap_radius / 1000)
                for pod in plane.fuselages[1:]:
                    self.assertAlmostEqual(pod.xsecs[0].xyz_c[0], -params.motor_mount_face_x / 1000)
                    self.assertAlmostEqual(pod.xsecs[-1].width / 2, params.motor_pod_aft_cap_radius / 1000)

    def test_shared_offset_matches_cad_translation_in_aero_coordinates(self):
        import numpy as np
        from whole_aircraft_aero import build_airplane

        reference = replace(PARAMETERS, assembly_x_offset=0.0)
        original = build_airplane(reference)
        shifted = build_airplane(replace(reference, assembly_x_offset=-7.0))
        for old_wing, new_wing in zip(original.wings, shifted.wings):
            for old_section, new_section in zip(old_wing.xsecs, new_wing.xsecs):
                np.testing.assert_allclose(np.asarray(new_section.xyz_le) - old_section.xyz_le, [0.007, 0, 0], atol=1e-12)
        for i, (old_body, new_body) in enumerate(zip(original.fuselages, shifted.fuselages)):
            for old_section, new_section in zip(old_body.xsecs, new_body.xsecs):
                np.testing.assert_allclose(np.asarray(new_section.xyz_c) - old_section.xyz_c, [0 if i == 0 else 0.007, 0, 0], atol=1e-12)

    def test_aero_adapters_preserve_thickened_sections_and_blunt_edges(self):
        import numpy as np
        from airfoil_polars import _geometry_matched_airfoil
        from whole_aircraft_aero import build_airplane

        airplane = build_airplane()
        for name, chord, aero_section in (
            ("wing_root", PARAMETERS.wing_root_chord, airplane.wings[0].xsecs[0]),
            ("wing_tip", PARAMETERS.wing_tip_chord, airplane.wings[0].xsecs[1]),
        ):
            with self.subTest(section=name):
                upper, lower = _section_coordinates(
                    PARAMETERS.wing_airfoil, chord, PARAMETERS.wing_trailing_edge,
                    PARAMETERS.wing_thickness_scale,
                )
                expected = list(reversed(upper)) + lower[1:]
                np.testing.assert_allclose(aero_section.airfoil.coordinates, expected)
                polar_section, _, _ = _geometry_matched_airfoil(name, PARAMETERS.wing_airfoil, chord)
                np.testing.assert_allclose(polar_section.coordinates, expected)

        upper, lower = _section_coordinates(
            PARAMETERS.support_airfoil, PARAMETERS.support_chord,
            PARAMETERS.support_trailing_edge, PARAMETERS.support_thickness_scale,
        )
        # The vertical adapter reflects and reverses the CAD contour.
        expected = np.asarray(list(reversed(upper)) + lower[1:])[::-1].copy()
        expected[:, 1] *= -1
        np.testing.assert_allclose(airplane.wings[1].xsecs[0].airfoil.coordinates, expected)
        polar_section, _, _ = _geometry_matched_airfoil(
            "vertical_support", PARAMETERS.support_airfoil, PARAMETERS.support_chord,
        )
        np.testing.assert_allclose(polar_section.coordinates, list(reversed(upper)) + lower[1:])


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
