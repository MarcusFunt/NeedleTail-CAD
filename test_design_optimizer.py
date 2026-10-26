"""Fast regression tests for the unified NeedleTail design space.

No build123d/SU2 required. Run with: python -m unittest -v test_design_optimizer
"""
import unittest
from dataclasses import replace
import numpy as np

from design_configuration import (Design, FLAP_MODES, TOPOLOGIES,
                                  flap_segments, rotor_stations, mass_penalty_g)
from design_optimizer import (Settings, candidate_designs, evaluate, optimize)


class FakeAirframe:
    horizontal_motor_offset=100.
    vertical_motor_offset=85.
    motor_station_x=0.
    prop_hub_center_x=23.
    fuselage_max_radius=21.
    wing_span=178.
    support_span=148.
    wing_root_chord=60.
    wing_tip_chord=35.
    support_chord=38.


class DesignTest(unittest.TestCase):
    def setUp(self):
        self.p=FakeAirframe()
        self.s=Settings()

    def test_default_retains_original_four_tractor_and_no_flaps(self):
        d=Design()
        self.assertEqual(d.topology,"tractor4")
        self.assertEqual(d.flaps,"none")
        self.assertEqual(len(rotor_stations(self.p,d)),4)
        self.assertEqual(flap_segments(self.p,d),())

    def test_all_9_mode_pairs_exist(self):
        pairs={(d.topology,d.flaps) for d in candidate_designs()}
        self.assertEqual(pairs,set((a,b) for a in TOPOLOGIES for b in FLAP_MODES))

    def test_rotor_locations_and_coaxial_spin(self):
        for mode,n in (("tractor4",4),("pusher4",4),("coaxial8",8)):
            stations=rotor_stations(self.p,Design(topology=mode))
            self.assertEqual(len(stations),n)
            for r in stations:
                self.assertIn(r.xyz_mm[0],(23.,-23.))
        pairs=rotor_stations(self.p,Design(topology="coaxial8"))
        for side in ("left","right","bottom","top"):
            rs=[p for p in pairs if p.name==side]
            self.assertEqual(len(rs),2)
            self.assertEqual(sum(p.spin_sign for p in rs),0)
        pair=rotor_stations(self.p,Design(topology="coaxial8",coaxial_separation_mm=60))
        self.assertEqual(pair[1].xyz_mm[0],-37.)

    def test_flap_counts_and_exposure(self):
        self.assertEqual(len(flap_segments(self.p,Design(flaps="horizontal"))),2)
        ss=flap_segments(self.p,Design(flaps="all_four"))
        self.assertEqual(len(ss),4)
        self.assertTrue(all(abs(v)>=self.p.fuselage_max_radius
                            for _,axis,a,b in ss for v in (a,b)))

    def test_added_mass(self):
        self.assertEqual(mass_penalty_g(Design()),0.)
        self.assertAlmostEqual(mass_penalty_g(Design(topology="coaxial8")),61.2)
        self.assertAlmostEqual(mass_penalty_g(Design(flaps="horizontal")),5.)
        self.assertAlmostEqual(mass_penalty_g(Design(flaps="all_four")),10.)

    def test_invalid_parameters(self):
        for kwargs in (dict(topology="wrong"),dict(flaps="bad"),
                       dict(hinge_fraction=.99),
                       dict(flap_start_fraction=.9,flap_end_fraction=.8),
                       dict(coaxial_separation_mm=-5)):
            with self.assertRaises(ValueError):Design(**kwargs)
        with self.assertRaises(ValueError):Settings(per_rotor_thrust_N=0.)

    def test_equilibrium_moment_residuals_and_bounds(self):
        for top in TOPOLOGIES:
            for flap in FLAP_MODES:
                r=evaluate(self.p,Design(topology=top,flaps=flap),self.s)
                self.assertTrue(r["feasible"],msg=r)
                self.assertLess(r["moment_residual_Nm"],1e-6)
                self.assertLessEqual(r["thrust_sacrificed_N"],8+1e-8)
                self.assertGreaterEqual(r["flap_incremental_drag_N"],0)

    def test_flaps_cannot_work_in_dead_air_without_wake(self):
        s=replace(self.s,speed_mps=0.)
        zero=evaluate(self.p,Design(),s)
        with_flaps=evaluate(self.p,Design(flaps="all_four"),s)
        self.assertAlmostEqual(zero["available_collective_N"],
                               with_flaps["available_collective_N"],places=7)

    def test_derivative_shape_validation_and_zero_model(self):
        with self.assertRaises(ValueError):
            evaluate(self.p,Design(flaps="horizontal"),self.s,
                     derivatives=np.zeros((2,2)))
        d=Design(flaps="all_four")
        a=evaluate(self.p,d,replace(self.s,require_modelled_control=False))
        b=evaluate(self.p,Design(),self.s)
        self.assertAlmostEqual(a["available_collective_N"],b["available_collective_N"])

    def test_optimizer_output_and_all_modes(self):
        results=optimize(self.p,self.s)
        self.assertEqual(len(results),75)
        self.assertEqual({r["topology"] for r in results},set(TOPOLOGIES))
        self.assertEqual({r["flaps"] for r in results},set(FLAP_MODES))
        self.assertTrue(all(r["cad_validated"] is False for r in results))
        scores=[x["score_N"] for x in results if x["feasible"]]
        self.assertEqual(scores,sorted(scores,reverse=True))

    def test_parametric_airframe_geometry_grid(self):
        from airframe_geometry import PARAMETERS
        from design_optimizer import airframes_from_grid
        scenario=Settings(geometry_grid_mm={
            "wing_root_chord":[55.,60.],
            "horizontal_motor_offset":[100.,110.],
        })
        items=list(airframes_from_grid(PARAMETERS,scenario))
        self.assertEqual(len(items),4)
        self.assertEqual({p.wing_span for p,_ in items},{178.,198.})
        self.assertEqual({p.wing_root_chord for p,_ in items},{55.,60.})
        self.assertTrue(all(p.horizontal_motor_offset==mm["horizontal_motor_offset"]
                            for p,mm in items))
        with self.assertRaises(ValueError):
            Settings(geometry_grid_mm={"unknown_dimension":[10.]})


if __name__=="__main__":unittest.main()
