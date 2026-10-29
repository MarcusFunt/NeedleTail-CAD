from __future__ import annotations

import unittest

from validation.qpropc.validate_upstream import (
    _original_qprop_loads,
    parse_apc_pe0_geometry,
    rpm_from_upstream_filename,
)
from propulsion.qpropc import propeller_is_out_of_scope


class QPropCUpstreamTests(unittest.TestCase):
    def test_rpm_is_parsed_from_all_upstream_uiuc_forward_filename_shapes(self):
        self.assertEqual(rpm_from_upstream_filename("apcsf_10x7_kt0828_3008.txt"), 3008.0)
        self.assertEqual(rpm_from_upstream_filename("apce_16x8_2154od_4968.txt"), 4968.0)
        self.assertEqual(rpm_from_upstream_filename("apcff_4.2x4_0620rd_10042.txt"), 10042.0)

    def test_rpm_parser_rejects_file_without_measured_rpm_suffix(self):
        with self.assertRaisesRegex(ValueError, "RPM suffix"):
            rpm_from_upstream_filename("forward_data.txt")

    def test_apc_pe0_geometry_is_parsed_in_si_with_manufacturer_provenance(self):
        from pathlib import Path
        from tempfile import TemporaryDirectory

        source = """APC test
STATION CHORD PITCH PITCH PITCH SWEEP THICKNESS TWIST MAX-THICK AREA ZHIGH CGY CGZ
1.4000 1.0256 8.0000 7.9978 6.8949 0.5018 0.2111 42.2773 0.2165 0.1366 0.5899 0.1557 0.3265
8.0000 0.0157 8.0000 8.0201 8.0200 -0.3582 0.1000 9.0654 0.0016 0.0000 0.0685 0.0000 0.0000
RADIUS: 8.00 PROPELLER RADIUS (IN)
HUBTRA: 1.40 HUB TRANSITION (IN)
BLADES: 2 NUMBER OF BLADES
"""
        with TemporaryDirectory() as temporary:
            path = Path(temporary) / "test.PE0"
            path.write_text(source, encoding="utf-8")
            geometry = parse_apc_pe0_geometry(
                path,
                propeller_id="apc_16x8e",
                diameter_m=16 * 0.0254,
                source_uri="validation/test.PE0",
            )

        self.assertEqual(geometry.blade_count, 2)
        self.assertAlmostEqual(geometry.hub_radius_m, 1.4 * 0.0254)
        self.assertAlmostEqual(geometry.stations[0].radius_m, 1.4 * 0.0254)
        self.assertAlmostEqual(geometry.stations[0].chord_m, 1.0256 * 0.0254)
        self.assertAlmostEqual(geometry.stations[0].twist_rad, 42.2773 * 3.141592653589793 / 180)
        self.assertEqual(geometry.geometry_provenance.value, "manufacturer")

    def test_known_validation_exception_is_outside_solver_validity_claim(self):
        exceptions = ("apc_4p2x4",)
        self.assertTrue(propeller_is_out_of_scope("apc_4p2x4", exceptions))
        self.assertFalse(propeller_is_out_of_scope("apc_10x7sf", exceptions))

    def test_qprop122_reconstruction_uses_extrapolated_tip_section_diameter(self):
        rows = [
            [0.02, 0.01, 0.2, 0.1, 0.01, 0, 0, 0, 0, 1, 0, 1],
            [0.03, 0.01, 0.2, 0.1, 0.01, 0, 0, 0, 0, 1, 0, 1],
            [0.04, 0.01, 0.2, 0.1, 0.01, 0, 0, 0, 0, 1, 0, 1],
        ]
        _thrust, _torque, diameter = _original_qprop_loads(rows)
        self.assertAlmostEqual(diameter, 0.09)


if __name__ == "__main__":
    unittest.main()
