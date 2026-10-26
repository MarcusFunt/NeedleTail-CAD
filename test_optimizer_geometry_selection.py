"""Tests that optimizer results identify the explicitly selected geometry."""

import unittest

from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS, PARAMETERS
from design_optimizer import Settings, optimize


class OptimizerGeometrySelectionTests(unittest.TestCase):
    def test_candidate_selection_labels_every_optimizer_result(self):
        results = optimize(
            AIRFOIL_CANDIDATE_PARAMETERS,
            Settings(mass_scenarios={"baseline": {}}),
            geometry_configuration="airfoil_candidate",
        )

        self.assertTrue(results)
        self.assertEqual({row["geometry_configuration"] for row in results}, {"airfoil_candidate"})
        self.assertEqual({row["mass_scenario"] for row in results}, {"baseline"})

    def test_baseline_optimization_defaults_to_pr3_geometry_label(self):
        results = optimize(PARAMETERS, Settings())

        self.assertTrue(results)
        self.assertEqual({row["geometry_configuration"] for row in results}, {"pr3_baseline"})


if __name__ == "__main__":
    unittest.main()
