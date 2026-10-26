"""AeroSandbox control-geometry smoke tests using the locked 'aero' environment."""
import unittest
import numpy as np

from airframe_geometry import PARAMETERS
from design_aero import build_variant_airplane, _nacelle_profile, flap_derivatives
from design_configuration import Design
from design_optimizer import _asb_moments_to_cad


class ControlAeroSmoke(unittest.TestCase):
    def test_surface_names_and_independent_deflection(self):
        for mode,expected in (
            ("none", set()),
            ("horizontal", {"left","right"}),
            ("all_four", {"left","right","bottom","top"}),
        ):
            airplane=build_variant_airplane(
                PARAMETERS,Design(flaps=mode,topology="tractor4"))
            names=set()
            for wing in airplane.wings:
                names.update(wing.get_control_surface_names())
            self.assertEqual(names,expected)
            if names:
                moved=airplane.with_control_deflections({sorted(names)[0]:5.})
                deflections={}
                for wing in moved.wings:
                    for sec in wing.xsecs:
                        for surf in sec.control_surfaces:
                            deflections[surf.name]=surf.deflection
                self.assertAlmostEqual(deflections[sorted(names)[0]],5.)
                for name in sorted(names)[1:]:
                    self.assertAlmostEqual(deflections[name],0.)

    def test_all_topologies_have_same_four_nacelle_stations(self):
        for top in ("tractor4","pusher4","coaxial8"):
            airplane=build_variant_airplane(PARAMETERS,Design(topology=top))
            self.assertEqual(len(airplane.fuselages),5)
            self.assertEqual(len(airplane.wings),2)

    def test_pusher_nacelle_profile_is_reversed(self):
        tractor=_nacelle_profile(PARAMETERS,"tractor4")
        pusher=_nacelle_profile(PARAMETERS,"pusher4")
        self.assertAlmostEqual(tractor[0][0],-pusher[-1][0])
        self.assertAlmostEqual(tractor[0][1],pusher[-1][1])

    def test_flap_derivative_signs_follow_cad_body_axes(self):
        def assert_signs_at(speed_mps, alpha_deg):
            names, asb_derivatives = flap_derivatives(
                PARAMETERS,
                Design(flaps="all_four"),
                speed_mps=speed_mps,
                alpha_deg=alpha_deg,
                delta_deg=2,
            )
            derivatives = _asb_moments_to_cad(asb_derivatives)

            self.assertEqual(names, ["left", "right", "bottom", "top"])
            self.assertEqual(derivatives.shape, (3, 4))
            self.assertTrue(np.isfinite(derivatives).all())

            # A positive horizontal flap deflection gives opposite roll on the
            # two wings and the same pitch direction in CAD body axes.
            self.assertGreater(derivatives[0, 0], 0)
            self.assertLess(derivatives[0, 1], 0)
            self.assertLess(derivatives[1, 0], 0)
            self.assertLess(derivatives[1, 1], 0)

            # Vertical flaps produce opposite roll and common yaw.
            self.assertGreater(derivatives[0, 2], 0)
            self.assertLess(derivatives[0, 3], 0)
            self.assertGreater(derivatives[2, 2], 0)
            self.assertGreater(derivatives[2, 3], 0)

            # Symmetry cancels cross-axis terms at zero angle of attack. At a
            # nonzero angle the lifting flow can create legitimate coupling.
            if alpha_deg == 0:
                self.assertLess(abs(derivatives[2, 0]), 0.1 * abs(derivatives[1, 0]))
                self.assertLess(abs(derivatives[2, 1]), 0.1 * abs(derivatives[1, 1]))
                self.assertLess(abs(derivatives[1, 2]), 0.1 * abs(derivatives[0, 2]))
                self.assertLess(abs(derivatives[1, 3]), 0.1 * abs(derivatives[0, 3]))

        for speed_mps, alpha_deg in ((30, 0), (15, 3)):
            with self.subTest(speed_mps=speed_mps, alpha_deg=alpha_deg):
                assert_signs_at(speed_mps, alpha_deg)


if __name__=="__main__":unittest.main()
