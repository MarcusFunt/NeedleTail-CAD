from __future__ import annotations

import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from airframe_geometry import PARAMETERS
from cfd.su2 import topology_cfd_campaign as campaign
from cfd.su2.generate_rotor_mesh import _rotors
from design_configuration import Design, rotor_stations


class TopologyStationContractTests(unittest.TestCase):
    def test_mesher_uses_canonical_rotor_station_positions_for_all_topologies(self):
        for topology, expected_count in (
            ("tractor4", 4), ("pusher4", 4), ("coaxial8", 8),
        ):
            with self.subTest(topology=topology):
                canonical = rotor_stations(PARAMETERS, Design(topology=topology))
                mesher = _rotors(PARAMETERS, topology)

                self.assertEqual(len(canonical), expected_count)
                self.assertEqual(len(mesher), expected_count)
                self.assertEqual(
                    [(r.x_m, r.y_m, r.z_m, r.side) for r in mesher],
                    [(*[value * 0.001 for value in s.xyz_mm], s.side) for s in canonical],
                )


class TopologyCampaignRegressionTests(unittest.TestCase):
    def test_missing_intermediate_history_step_marks_run_incomplete_and_withholds_averages(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="cw", spin_sign=1,
        )
        rows = [
            {
                "Time_Iter": str(step),
                "CFx": "1", "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6",
            }
            for step in range(25) if step != 13
        ]
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", return_value=rows
        ):
            result = campaign.summarize_history(
                Path("unused"), "tractor4", params,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0,
            )

        self.assertFalse(result["run_complete"])
        self.assertEqual(len(result["zone_forces"]), 2)
        for zone in result["zone_forces"]:
            self.assertFalse(zone["complete_time_steps"])
            self.assertEqual(zone["missing_time_step_indices_sample"], [13])
            self.assertNotIn("force_N", zone)

    def test_failed_solver_preflight_leaves_output_path_retryable(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            output_dir = root / "campaign"
            args = SimpleNamespace(geometry="pr3_baseline", output_dir=output_dir)
            env = {
                "SU2_RUN": str(root / "missing-su2-run"),
                "LOCALAPPDATA": str(root / "missing-local-app-data"),
            }
            with patch.dict(os.environ, env), patch.object(
                campaign, "airframe_parameters_for_geometry", return_value=object()
            ):
                for _ in range(2):
                    with self.assertRaises(FileNotFoundError):
                        campaign.run_campaign(args)
                    self.assertFalse(output_dir.exists())

    def test_report_uses_selected_topologies_and_step_count(self):
        config = {
            "topologies": ["pusher4"],
            "geometry": "pr3_baseline",
            "rpm": 30_000,
            "speed_mps": 12.5,
            "alpha_deg": 3.0,
            "revolutions": 2,
            "steps_per_revolution": 24,
            "static_near_size_m": 0.004,
            "static_far_size_m": 0.12,
            "prop_near_size_m": 0.0018,
            "interface_size_m": 0.003,
        }
        with TemporaryDirectory() as temp:
            campaign.write_campaign_report(Path(temp), [], config)
            report = (Path(temp) / "REPORT.md").read_text(encoding="utf-8")

        self.assertTrue(report.startswith("# Powered Euler CFD screen: pusher4\n"))
        self.assertIn("24 time steps per revolution, 2 revolutions per candidate", report)
        self.assertNotIn("Three-topology", report)


if __name__ == "__main__":
    unittest.main()
