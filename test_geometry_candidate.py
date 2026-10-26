"""Regression tests for explicit baseline and revised airframe configurations."""

import unittest
from importlib.util import find_spec

from airframe_geometry import (
    AIRFOIL_CANDIDATE_PARAMETERS,
    PARAMETERS,
    airframe_parameters_for_geometry,
    build_components,
)


class GeometryCandidateConfigurationTests(unittest.TestCase):
    def test_named_geometry_lookup_keeps_baseline_and_candidate_distinct(self):
        self.assertIs(airframe_parameters_for_geometry("pr3_baseline"), PARAMETERS)
        self.assertIs(
            airframe_parameters_for_geometry("airfoil_candidate"),
            AIRFOIL_CANDIDATE_PARAMETERS,
        )
        with self.assertRaisesRegex(ValueError, "Unknown geometry configuration"):
            airframe_parameters_for_geometry("unknown")

    def test_pr3_geometry_remains_the_default_configuration(self):
        self.assertEqual(PARAMETERS.fuselage_length, 170.0)
        self.assertEqual(PARAMETERS.fuselage_max_radius, 21.0)
        self.assertEqual(PARAMETERS.wing_root_chord, 60.0)
        self.assertEqual(PARAMETERS.wing_tip_chord, 35.0)
        self.assertEqual(PARAMETERS.wing_airfoil, "SD7003")
        self.assertEqual(PARAMETERS.support_chord, 38.0)
        self.assertEqual(PARAMETERS.support_airfoil, "SD8020")

    def test_revised_geometry_is_available_as_an_explicit_candidate(self):
        candidate = AIRFOIL_CANDIDATE_PARAMETERS

        self.assertIsNot(candidate, PARAMETERS)
        self.assertEqual(candidate.fuselage_length, 230.0)
        self.assertEqual(candidate.fuselage_max_radius, 30.0)
        self.assertEqual(candidate.fuselage_body_airfoil, "NACA0015")
        self.assertEqual(candidate.wing_root_chord, 80.0)
        self.assertEqual(candidate.wing_tip_chord, 56.0)
        self.assertEqual(candidate.wing_airfoil, "NACA2415")
        self.assertEqual(candidate.wing_thickness_scale, 1.2)
        self.assertEqual(candidate.wing_trailing_edge, 1.2)
        self.assertEqual(candidate.support_chord, 64.0)
        self.assertEqual(candidate.support_airfoil, "NACA0015")
        self.assertEqual(candidate.support_thickness_scale, 1.2)
        self.assertEqual(candidate.support_trailing_edge, 1.2)
        self.assertEqual(candidate.assembly_x_offset, -5.0)
        self.assertEqual(candidate.motor_station_x, 1.0)
        self.assertEqual(candidate.horizontal_motor_offset, 95.0)
        self.assertEqual(candidate.vertical_motor_offset, 80.0)
        self.assertEqual(candidate.motor_pod_length, 124.0)
        self.assertEqual(candidate.motor_pod_diameter, 19.9)
        self.assertEqual(candidate.motor_pod_body_airfoil, "NACA0015")
        self.assertEqual(candidate.motor_mount_face_offset_from_pod_center, 44.0)

    def test_candidate_cad_uses_one_shared_global_assembly_translation(self):
        candidate = AIRFOIL_CANDIDATE_PARAMETERS
        components = build_components(candidate)

        fuselage, wing, support, *pods = components
        body_bounds = fuselage.bounding_box()
        wing_bounds = wing.bounding_box()
        support_bounds = support.bounding_box()

        self.assertAlmostEqual(body_bounds.size.X, 230.0, places=5)
        self.assertAlmostEqual(wing_bounds.min.X, -45.0, places=5)
        self.assertAlmostEqual(wing_bounds.max.X, 35.0, places=5)
        self.assertAlmostEqual(wing_bounds.min.Y, -95.0, places=5)
        self.assertAlmostEqual(wing_bounds.max.Y, 95.0, places=5)
        self.assertAlmostEqual(support_bounds.min.X, -37.0, places=5)
        self.assertAlmostEqual(support_bounds.max.X, 27.0, places=5)
        self.assertAlmostEqual(support_bounds.min.Z, -80.0, places=5)
        self.assertAlmostEqual(support_bounds.max.Z, 80.0, places=5)
        self.assertEqual(len(pods), 4)
        for pod in pods:
            bounds = pod.bounding_box()
            self.assertAlmostEqual(bounds.min.X, -66.0, places=3)
            self.assertAlmostEqual(bounds.max.X, 40.0, places=4)
        candidate.validate_propeller_clearance()

        self.assertEqual(
            set(candidate.propeller_clearance_report),
            {"pod_noses", "horizontal_wing", "vertical_support", "fuselage", "neighboring_propellers"},
        )

    def test_candidate_oml_body_and_pod_are_valid_single_solids(self):
        from airframe_geometry import (
            AIRFOIL_CANDIDATE_PARAMETERS,
            _fuselage,
            build_airframe_oml,
            build_motor_pod,
        )

        params = AIRFOIL_CANDIDATE_PARAMETERS
        body = _fuselage(params)
        pod = build_motor_pod(params)
        oml = build_airframe_oml(params)

        self.assertTrue(body.is_valid)
        self.assertTrue(pod.is_valid)
        self.assertTrue(oml.is_valid)
        self.assertEqual(len(oml.solids()), 1)
        self.assertAlmostEqual(body.bounding_box().size.X, 230.0, places=4)
        self.assertAlmostEqual(pod.bounding_box().min.X, -62.0, delta=0.001)
        self.assertAlmostEqual(pod.bounding_box().max.X, 44.0, places=4)

    def test_candidate_thickness_scale_preserves_camber_and_physical_edge(self):
        from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS, _section_coordinates

        params = AIRFOIL_CANDIDATE_PARAMETERS
        base_upper, base_lower = _section_coordinates(
            params.wing_airfoil,
            params.wing_tip_chord,
            params.wing_trailing_edge,
        )
        upper, lower = _section_coordinates(
            params.wing_airfoil,
            params.wing_tip_chord,
            params.wing_trailing_edge,
            thickness_scale=params.wing_thickness_scale,
        )
        self.assertEqual([point[0] for point in base_upper], [point[0] for point in upper])
        for base_u, base_l, thick_u, thick_l in zip(
            base_upper, base_lower, upper, lower
        ):
            self.assertAlmostEqual(
                (base_u[1] + base_l[1]) / 2,
                (thick_u[1] + thick_l[1]) / 2,
                places=10,
            )
            if 0 < base_u[0] <= 0.94:
                self.assertAlmostEqual(
                    thick_u[1] - thick_l[1],
                    params.wing_thickness_scale * (base_u[1] - base_l[1]),
                    places=10,
                )
        self.assertAlmostEqual(
            (upper[-1][1] - lower[-1][1]) * params.wing_tip_chord,
            params.wing_trailing_edge,
            places=8,
        )


@unittest.skipUnless(find_spec("aerosandbox"), "Optional aero dependencies are not installed")
class CandidateAeroAdapterTests(unittest.TestCase):
    def test_candidate_airplane_matches_cad_sections_profiles_and_translation(self):
        import numpy as np

        from airframe_geometry import (
            AIRFOIL_CANDIDATE_PARAMETERS,
            _section_coordinates,
            fuselage_radius_stations,
            motor_pod_radius_stations,
        )
        from whole_aircraft_aero import (
            _fuselage_profile,
            _pod_profile,
            build_airplane,
        )

        params = AIRFOIL_CANDIDATE_PARAMETERS
        airplane = build_airplane(params)
        wing, support = airplane.wings

        self.assertEqual(_fuselage_profile(params), fuselage_radius_stations(params))
        self.assertEqual(_pod_profile(params), motor_pod_radius_stations(params))
        self.assertAlmostEqual(wing.xsecs[0].xyz_le[0], -0.035)
        self.assertAlmostEqual(wing.xsecs[1].xyz_le[0], -0.023)
        self.assertAlmostEqual(wing.xsecs[1].xyz_le[1], 0.095)
        self.assertAlmostEqual(support.xsecs[0].xyz_le[0], -0.027)
        self.assertAlmostEqual(support.xsecs[1].xyz_le[2], 0.080)
        self.assertAlmostEqual(airplane.fuselages[1].xsecs[0].xyz_c[0], -0.040)
        self.assertAlmostEqual(airplane.fuselages[1].xsecs[-1].xyz_c[0], 0.066)

        upper, lower = _section_coordinates(
            params.wing_airfoil,
            params.wing_root_chord,
            params.wing_trailing_edge,
            params.wing_thickness_scale,
        )
        expected = np.asarray(list(reversed(upper)) + lower[1:])
        np.testing.assert_allclose(wing.xsecs[0].airfoil.coordinates, expected)

    def test_flap_variant_preserves_candidate_thickness_and_shared_offset(self):
        import numpy as np

        from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS, _section_coordinates
        from design_aero import build_variant_airplane
        from design_configuration import Design

        params = AIRFOIL_CANDIDATE_PARAMETERS
        airplane = build_variant_airplane(params, Design(flaps="horizontal"))
        root_sections = [
            section
            for wing in airplane.wings[:2]
            for section in wing.xsecs
            if abs(section.xyz_le[1]) < 1e-12
        ]
        self.assertEqual(len(root_sections), 2)
        for section in root_sections:
            self.assertAlmostEqual(section.xyz_le[0], -0.035)

        upper, lower = _section_coordinates(
            params.wing_airfoil,
            params.wing_root_chord,
            params.wing_trailing_edge,
            params.wing_thickness_scale,
        )
        expected = np.asarray(list(reversed(upper)) + lower[1:])
        np.testing.assert_allclose(root_sections[0].airfoil.coordinates, expected)

    def test_candidate_polar_adapter_uses_candidate_thickness_and_trailing_edge(self):
        import numpy as np

        from airfoil_polars import _geometry_matched_airfoil
        from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS, _section_coordinates

        params = AIRFOIL_CANDIDATE_PARAMETERS
        airfoil, coordinates, trailing_edge = _geometry_matched_airfoil(
            "wing_root", params.wing_airfoil, params.wing_root_chord, params
        )
        upper, lower = _section_coordinates(
            params.wing_airfoil,
            params.wing_root_chord,
            params.wing_trailing_edge,
            thickness_scale=params.wing_thickness_scale,
        )
        expected = np.asarray(list(reversed(upper)) + lower[1:])
        np.testing.assert_allclose(coordinates, expected)
        np.testing.assert_allclose(airfoil.coordinates, expected)
        self.assertAlmostEqual(trailing_edge, params.wing_trailing_edge)


class CandidateTopologyPlacementTests(unittest.TestCase):
    def test_rotor_pair_is_mirrored_about_global_candidate_pod_center(self):
        from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS
        from design_configuration import Design, rotor_stations

        params = AIRFOIL_CANDIDATE_PARAMETERS
        stations = rotor_stations(params, Design(topology="coaxial8"))
        left_pair = [station for station in stations if station.name == "left"]
        front = next(station for station in left_pair if station.side == "front")
        rear = next(station for station in left_pair if station.side == "rear")

        self.assertAlmostEqual(front.xyz_mm[0], params.prop_hub_center_x)
        self.assertAlmostEqual(
            (front.xyz_mm[0] + rear.xyz_mm[0]) / 2,
            params.motor_pod_center_x,
        )

    def test_candidate_variant_cad_applies_assembly_offset_once(self):
        from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS
        from design_cad import build_variant_components
        from design_configuration import Design

        params = AIRFOIL_CANDIDATE_PARAMETERS
        _, wing, support, *pods = build_variant_components(
            params, Design(topology="tractor4")
        )
        wing_bounds = wing.bounding_box()
        support_bounds = support.bounding_box()

        self.assertAlmostEqual(wing_bounds.min.X, -45.0, places=5)
        self.assertAlmostEqual(wing_bounds.max.X, 35.0, places=5)
        self.assertAlmostEqual(support_bounds.min.X, -37.0, places=5)
        self.assertAlmostEqual(support_bounds.max.X, 27.0, places=5)
        self.assertEqual(len(pods), 4)
        for pod in pods:
            bounds = pod.bounding_box()
            self.assertAlmostEqual(bounds.min.X, -66.0, places=3)
            self.assertAlmostEqual(bounds.max.X, 40.0, places=5)

    def test_propeller_envelopes_follow_candidate_motor_axis_z_offset(self):
        from dataclasses import replace

        from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS
        from propeller_envelope import build_propeller_envelopes

        params = replace(
            AIRFOIL_CANDIDATE_PARAMETERS,
            motor_axis_z_offset_from_pod_center=2.0,
        )
        bounds = build_propeller_envelopes(params).bounding_box()

        self.assertAlmostEqual(bounds.max.Z, 120.1, places=5)
        self.assertAlmostEqual(bounds.min.Z, -116.1, places=5)

    def test_motor_prop_reference_can_be_built_for_the_candidate(self):
        from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS, PARAMETERS
        from motor_prop_installation import build_motor_prop_installation

        baseline = build_motor_prop_installation(PARAMETERS)
        candidate = build_motor_prop_installation(AIRFOIL_CANDIDATE_PARAMETERS)
        baseline_parts = baseline.children
        candidate_parts = candidate.children

        self.assertEqual(len(candidate_parts), 3)
        for baseline_part, candidate_part in zip(baseline_parts[1:], candidate_parts[1:]):
            before = baseline_part.bounding_box()
            after = candidate_part.bounding_box()
            self.assertAlmostEqual(after.center().X - before.center().X, 35.5, places=5)
            self.assertAlmostEqual(after.center().Y, 95.0, places=5)
            self.assertAlmostEqual(after.center().Z, before.center().Z, places=5)


if __name__ == "__main__":
    unittest.main()
