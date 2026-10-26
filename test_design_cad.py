"""Exact CAD smoke tests for the unified NeedleTail variants.

Unlike aerodynamic proxy screens, these require the repo's locked build123d
and cadgen override. Collision-prone arrangements are reported, not silently
declared buildable.
"""
import math
import unittest
from dataclasses import replace

from airframe_geometry import PARAMETERS
from design_configuration import Design
from design_cad import (build_variant_components, build_variant_propeller_envelopes,
                        validate_variant)


class VariantCADSmoke(unittest.TestCase):
    def test_legacy_tractor_clearance_reproduced(self):
        d=Design()
        self.assertEqual(len(build_variant_components(PARAMETERS,d)),7)
        self.assertEqual(len(build_variant_propeller_envelopes(PARAMETERS,d)),4)
        report=validate_variant(PARAMETERS,d,PARAMETERS.propeller_clearance_margin)
        self.assertTrue(report)
        self.assertGreaterEqual(min(report.values()),PARAMETERS.propeller_clearance_margin)

    def test_pusher_and_coax_pod_envelopes_generate(self):
        for topology,n in (("pusher4",4),("coaxial8",8)):
            d=Design(topology=topology)
            self.assertEqual(len(build_variant_propeller_envelopes(PARAMETERS,d)),n)
            self.assertEqual(len(build_variant_components(PARAMETERS,d)),3+n)
            report=validate_variant(PARAMETERS,d,PARAMETERS.propeller_clearance_margin)
            self.assertTrue(report)
            self.assertTrue(all(math.isfinite(v) and
                                v>=PARAMETERS.propeller_clearance_margin
                                for v in report.values()))

    def test_pusher_clearance_rejects_rotor_disk_intersection_with_wing(self):
        larger_prop = replace(PARAMETERS,propeller_diameter=80.)
        with self.assertRaisesRegex(ValueError,"horizontal_wing"):
            validate_variant(
                larger_prop,
                Design(topology="pusher4"),
                larger_prop.propeller_clearance_margin,
            )

    def test_neutral_flap_cad_split_generates_solid_panels(self):
        for mode,n in (("horizontal",2),("all_four",4)):
            parts=build_variant_components(PARAMETERS,Design(flaps=mode))
            self.assertEqual(len(parts),7+n)
            self.assertEqual(sum("movable_panel" in part.label for part in parts),n)
            self.assertTrue(all(part.solids() for part in parts))


if __name__=="__main__": unittest.main()
