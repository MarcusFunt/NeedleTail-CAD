"""AeroSandbox control-geometry smoke tests using the locked 'aero' environment."""
import unittest

from airframe_geometry import PARAMETERS
from design_aero import build_variant_airplane, _nacelle_profile
from design_configuration import Design


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


if __name__=="__main__":unittest.main()
