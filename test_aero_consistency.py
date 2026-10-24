"""Independent checks for the CAD-to-AeroSandbox adapter and solver health."""
from __future__ import annotations

import math
import unittest

import aerosandbox as asb
import numpy as np

from airframe_geometry import (
    PARAMETERS, _section_coordinates, build_components,
)
from whole_aircraft_aero import build_airplane, _sample_profile, _fuselage_profile, _pod_profile


class AircraftGeometryConsistency(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.airplane = build_airplane()
        cls.params = PARAMETERS

    def test_spans_chords_leading_edges_and_reference_area(self):
        params, airplane = self.params, self.airplane
        wing, support = airplane.wings
        self.assertAlmostEqual(wing.xsecs[0].chord, params.wing_root_chord / 1000)
        self.assertAlmostEqual(wing.xsecs[1].chord, params.wing_tip_chord / 1000)
        self.assertAlmostEqual(wing.xsecs[0].xyz_le[0], -params.wing_root_chord / 2000)
        self.assertAlmostEqual(wing.xsecs[1].xyz_le[0], -params.wing_tip_chord / 2000)
        self.assertAlmostEqual(wing.xsecs[1].xyz_le[1], params.wing_span / 2000)
        self.assertAlmostEqual(support.xsecs[0].xyz_le[2], -params.support_span / 2000)
        self.assertAlmostEqual(support.xsecs[1].xyz_le[2], params.support_span / 2000)
        self.assertAlmostEqual(airplane.s_ref, params.wing_planform_area / 1e6)
        self.assertAlmostEqual(airplane.b_ref, params.wing_span / 1000)
        cad_parts = build_components(params)
        wing_box, support_box = cad_parts[1].bounding_box(), cad_parts[2].bounding_box()
        self.assertAlmostEqual(wing_box.min.Y / 1000, -wing.xsecs[1].xyz_le[1], delta=1e-5)
        self.assertAlmostEqual(wing_box.max.Y / 1000, wing.xsecs[1].xyz_le[1], delta=1e-5)
        self.assertAlmostEqual(support_box.min.Z / 1000, support.xsecs[0].xyz_le[2], delta=1e-5)
        self.assertAlmostEqual(support_box.max.Z / 1000, support.xsecs[1].xyz_le[2], delta=1e-5)

    def test_airfoil_coordinates_are_the_exact_shared_profiles(self):
        params = self.params
        for wing, foil_name, chord_mm, te_mm in (
            (self.airplane.wings[0].xsecs[0], params.wing_airfoil, params.wing_root_chord, params.wing_trailing_edge),
            (self.airplane.wings[0].xsecs[1], params.wing_airfoil, params.wing_tip_chord, params.wing_trailing_edge),
        ):
            upper, lower = _section_coordinates(foil_name, chord_mm, te_mm)
            expected = np.asarray(list(reversed(upper)) + lower[1:])
            np.testing.assert_allclose(np.asarray(wing.airfoil.coordinates), expected, atol=1e-10, rtol=0)
            self.assertAlmostEqual((expected[0, 1] - expected[-1, 1]) * chord_mm, te_mm, delta=1e-6)

    def test_axisymmetric_profiles_match_cad_splines(self):
        params = self.params
        for body, profile, center_x_mm, y_mm, z_mm in [
            (self.airplane.fuselages[0], _fuselage_profile(params), 0, 0, 0),
            (self.airplane.fuselages[1], _pod_profile(params), params.motor_station_x, -params.horizontal_motor_offset, 0),
            (self.airplane.fuselages[2], _pod_profile(params), params.motor_station_x, params.horizontal_motor_offset, 0),
            (self.airplane.fuselages[3], _pod_profile(params), params.motor_station_x, 0, -params.vertical_motor_offset),
            (self.airplane.fuselages[4], _pod_profile(params), params.motor_station_x, 0, params.vertical_motor_offset),
        ]:
            sampled = list(reversed(_sample_profile(profile, 25)))
            self.assertEqual(len(body.xsecs), len(sampled))
            for xsec, (x_mm, r_mm) in zip(body.xsecs, sampled):
                np.testing.assert_allclose(
                    np.asarray(xsec.xyz_c, dtype=float),
                    np.array((-(x_mm + center_x_mm)/1000, y_mm/1000, z_mm/1000)),
                    atol=1e-9, rtol=0,
                )
                self.assertAlmostEqual(xsec.width, 2 * r_mm / 1000, delta=1e-9)
                self.assertAlmostEqual(xsec.height, 2 * r_mm / 1000, delta=1e-9)

    def test_variant_override_changes_only_lifting_spans(self):
        old = build_airplane(wing_span_mm=240.0, support_span_mm=170.0)
        self.assertAlmostEqual(old.wings[0].xsecs[1].xyz_le[1], 0.12)
        self.assertAlmostEqual(old.wings[1].xsecs[1].xyz_le[2], 0.085)
        self.assertAlmostEqual(old.s_ref, 0.0114)
        self.assertAlmostEqual(old.fuselages[1].xsecs[0].xyz_c[1], self.airplane.fuselages[1].xsecs[0].xyz_c[1])


class AeroSmokeTests(unittest.TestCase):
    def test_neuralfoil_low_re_is_finite_and_positive_drag(self):
        foil = build_airplane().wings[0].xsecs[0].airfoil
        result = foil.get_aero_from_neuralfoil(alpha=np.array([0.0, 3.0]), Re=100_000, model_size="large")
        for field in ("CL", "CD", "CM", "analysis_confidence"):
            values = np.asarray(result[field], dtype=float)
            self.assertEqual(values.size, 2)
            self.assertTrue(np.all(np.isfinite(values)), field)
        self.assertTrue(np.all(np.asarray(result["CD"]) > 0))

    def test_aerobuildup_and_vlm_coefficients_are_finite(self):
        airplane = build_airplane()
        op = asb.OperatingPoint(velocity=15.0, alpha=2.0, beta=0.0)
        methods = (
            asb.AeroBuildup(airplane=airplane, op_point=op, model_size="large"),
            asb.VortexLatticeMethod(airplane=airplane, op_point=op, spanwise_resolution=8, chordwise_resolution=6, run_symmetric_if_possible=False),
        )
        for solver in methods:
            with self.subTest(solver=type(solver).__name__):
                result = solver.run()
                for key in ("CL", "CD", "CY", "Cl", "Cm", "Cn"):
                    values = np.asarray(result[key], dtype=float)
                    self.assertEqual(values.size, 1, (key, values.shape))
                    value = float(values.reshape(-1)[0])
                    self.assertTrue(math.isfinite(value), (key, value))
                    self.assertLess(abs(value), 10.0, (key, value))
                self.assertGreaterEqual(float(np.asarray(result["CD"]).reshape(-1)[0]), -1e-4)

if __name__ == "__main__":
    unittest.main()
