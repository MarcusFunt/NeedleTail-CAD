from __future__ import annotations

import csv
import os
import json
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from airframe_geometry import PARAMETERS
from cfd.su2 import topology_cfd_campaign as campaign
from cfd.su2 import generate_rotor_mesh as mesh
from cfd.su2.generate_rotor_mesh import _rotors
from design_configuration import Design, rotor_stations


def _two_zone_tetra_mesh(*, inverted_zone=None, flat_zone=None,
                         disconnected_zone=None, mismatched_interface=False,
                         shifted_interface=False):
    """Write a dependency-free one-rotor mesh fixture with closed markers."""
    points = [
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    ]
    face_rows = ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3))
    zone_text = []
    for zone in (1, 2):
        zone_points = list(points)
        if shifted_interface and zone == 2:
            zone_points = [(x + 0.25, y, z) for x, y, z in zone_points]
        elements = [(0, 1, 2, 3)]
        if disconnected_zone == zone:
            zone_points.extend((
                (3.0, 0.0, 0.0), (4.0, 0.0, 0.0),
                (3.0, 1.0, 0.0), (3.0, 0.0, 1.0),
            ))
            elements.append((4, 5, 6, 7))
        if inverted_zone == zone:
            elements[0] = (0, 1, 3, 2)
        if flat_zone == zone:
            zone_points[3] = (0.5, 0.5, 1e-9)

        if zone == 1:
            markers = {
                "AIRFRAME_MOTOR": [face_rows[0]],
                "FARFIELD": [face_rows[3]],
                "STATIC_IF_1": [face_rows[1], face_rows[2]],
            }
        else:
            interface_faces = [face_rows[1], face_rows[2]]
            if mismatched_interface:
                interface_faces = interface_faces[:1]
            propeller_faces = [face for face in face_rows if face not in interface_faces]
            markers = {
                "ROTOR_PROPELLER_1": propeller_faces,
                "ROTOR_IF_1": interface_faces,
            }
        if disconnected_zone == zone:
            markers["AIRFRAME_MOTOR" if zone == 1 else "ROTOR_PROPELLER_1"].extend(
                ((4, 5, 6), (4, 5, 7), (4, 6, 7), (5, 6, 7))
            )

        rows = [f"IZONE= {zone}", "NDIME= 3", f"NELEM= {len(elements)}"]
        rows.extend("10 " + " ".join(str(node) for node in tetra) for tetra in elements)
        rows.append(f"NPOIN= {len(zone_points)}")
        rows.extend(f"{x} {y} {z} {index}" for index, (x, y, z) in enumerate(zone_points))
        rows.append(f"NMARK= {len(markers)}")
        for name, faces in markers.items():
            rows.extend((f"MARKER_TAG= {name}", f"MARKER_ELEMS= {len(faces)}"))
            rows.extend("5 " + " ".join(str(node) for node in face) for face in faces)
        zone_text.extend(rows)
    return "NZONE= 2\n" + "\n".join(zone_text) + "\n"


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

    def test_mesher_uses_canonical_spin_and_hand_for_all_topologies(self):
        for topology in ("tractor4", "pusher4", "coaxial8"):
            stations = rotor_stations(PARAMETERS, Design(topology=topology))
            mesher = _rotors(PARAMETERS, topology)
            with self.subTest(topology=topology):
                self.assertEqual(
                    [rotor.spin_sign for rotor in mesher],
                    [station.spin_sign for station in stations],
                )
                self.assertEqual(
                    [rotor.prop_hand for rotor in mesher],
                    [getattr(station, "propeller_hand", None) for station in stations],
                )
                self.assertTrue(all(getattr(station, "shaft_axis_xyz", None) == (1.0, 0.0, 0.0)
                                    for station in stations))
                self.assertTrue(all(getattr(station, "thrust_direction_xyz", None) == (1.0, 0.0, 0.0)
                                    for station in stations))
                self.assertEqual([getattr(rotor, "shaft_axis_xyz", None) for rotor in mesher],
                                 [getattr(station, "shaft_axis_xyz", None) for station in stations])
                self.assertEqual([getattr(rotor, "thrust_direction_xyz", None) for rotor in mesher],
                                 [getattr(station, "thrust_direction_xyz", None) for station in stations])

    def test_explicit_coaxial_separation_reaches_rotor_stations(self):
        rotors = _rotors(PARAMETERS, "coaxial8", coaxial_separation_mm=60.0)
        for rotor in rotors[:4]:
            station = [r for r in rotors if (r.y_m, r.z_m) == (rotor.y_m, rotor.z_m)]
            self.assertEqual(len(station), 2)
            self.assertAlmostEqual(abs(station[0].x_m - station[1].x_m) * 1000, 60.0)
        for front, aft in zip(rotors[::2], rotors[1::2]):
            self.assertEqual(aft.spin_sign, -front.spin_sign)
            self.assertEqual(aft.prop_hand, "CCW" if front.prop_hand == "CW" else "CW")
            self.assertEqual(aft.thrust_direction_xyz, front.thrust_direction_xyz)

    def test_coaxial_history_uses_the_cli_separation_for_reported_stations(self):
        rows = [
            {
                "Time_Iter": str(step), "CFx": "1", "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6",
            }
            for step in range(25)
        ]
        with patch.object(campaign, "rows_by_zone", return_value=rows):
            result = campaign.summarize_history(
                Path("unused"), "coaxial8", PARAMETERS,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0, coaxial_separation_mm=60.0,
            )
        front, aft = result["zone_forces"][1:3]
        self.assertAlmostEqual(
            abs(front["rotor_station"]["x_m"] - aft["rotor_station"]["x_m"]) * 1000,
            60.0,
        )

    def test_configured_rotation_rates_follow_canonical_station_signs(self):
        rotors = _rotors(PARAMETERS, "tractor4")
        with TemporaryDirectory() as temp:
            campaign.write_configs(
                Path(temp), rpm=30_000, revolutions=2,
                steps_per_revolution=12, params=PARAMETERS,
                topology="tractor4",
            )
            for rotor in rotors:
                text = (Path(temp) / f"zone_rotor_{rotor.index}.cfg").read_text(encoding="utf-8")
                rate = rotor.spin_sign * 30_000 * 2 * 3.141592653589793 / 60
                self.assertIn(f"ROTATION_RATE= {rate:.9f} 0.0 0.0", text)
                self.assertIn(f"MARKER_MONITORING= ( ROTOR_PROPELLER_{rotor.index} )", text)

    def test_coaxial_config_contains_static_plus_eight_rotor_zones(self):
        with TemporaryDirectory() as temp:
            case = Path(temp)
            campaign.write_configs(
                case, rpm=30_000, revolutions=2, steps_per_revolution=12,
                params=PARAMETERS, topology="coaxial8", coaxial_separation_mm=60.0,
            )
            names = [path.name for path in case.glob("zone_*.cfg")]
            self.assertEqual(len(names), 9)
            self.assertIn("zone_static.cfg", names)
            self.assertIn("zone_rotor_8.cfg", names)


class TopologyCampaignRegressionTests(unittest.TestCase):
    def test_shaft_power_reports_zero_and_negative_regimes_explicitly(self):
        self.assertEqual(campaign.shaft_power_regime(0.0), "zero_shaft_power")
        self.assertEqual(campaign.shaft_power_regime(-0.1), "windmilling_or_power_turbine")
        self.assertEqual(campaign.shaft_power_regime(0.1), "motor_input_required")

    def test_shaft_torque_projects_shifted_moment_onto_canonical_axis(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=1.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(0.0, 1.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        q = campaign.RHO * 0.5 * 15.0**2
        area = params.wing_planform_area * 1e-6
        mac = 2 / 3 * params.wing_root_chord * 1e-3 * (1 + 0.5 + 0.25) / 1.5
        rows = [
            {
                "Time_Iter": str(step), "CFx": "0", "CFy": "0",
                "CFz": str(2.0 / (q * area)),
                "CMx": "0", "CMy": str(3.0 / (q * area * mac)), "CMz": "0",
            }
            for step in range(25)
        ]
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", return_value=rows
        ):
            result = campaign.summarize_history(
                Path("unused"), "tractor4", params,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0,
            )
        self.assertAlmostEqual(result["zone_forces"][1]["shaft_torque_Nm"], 5.0)

    def test_nonfinite_physical_history_is_rejected(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        rows = [
            {
                "Time_Iter": str(step),
                "CFx": "nan" if step == 24 else "1", "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6", "RMS_RES": "-6",
            }
            for step in range(25)
        ]
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", return_value=rows
        ):
            with self.assertRaisesRegex(ValueError, "nonfinite physical force or moment"):
                campaign.summarize_history(
                    Path("unused"), "tractor4", params,
                    revolutions=2, steps_per_revolution=12, rpm=30_000,
                    speed_mps=15.0, alpha_deg=4.0,
                )

    def test_torque_window_drift_fails_load_stability_even_when_forces_are_stable(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        static_rows = [
            {"Time_Iter": str(step), "CFx": "1", "CFy": "2", "CFz": "3",
             "CMx": "4", "CMy": "5", "CMz": "6", "RMS_RES": "-6"}
            for step in range(25)
        ]
        rotor_rows = [
            {"Time_Iter": str(step), "CFx": "1", "CFy": "2", "CFz": "3",
             "CMx": "1" if step <= 12 else "2", "CMy": "5", "CMz": "6", "RMS_RES": "-6"}
            for step in range(25)
        ]
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", side_effect=lambda _path, zone: static_rows if zone == 0 else rotor_rows
        ):
            result = campaign.summarize_history(
                Path("unused"), "tractor4", params,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0,
            )
        self.assertGreater(
            result["zone_forces"][1]["shaft_torque_change_fraction"],
            campaign.MAX_FINAL_REVOLUTION_FORCE_CHANGE,
        )
        self.assertFalse(result["numerical_convergence"]["load_stability_passed"])

    def test_missing_zone_history_is_reported_incomplete_without_averages(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        rows = [
            {"Time_Iter": str(step), "CFx": "1", "CFy": "2", "CFz": "3",
             "CMx": "4", "CMy": "5", "CMz": "6"}
            for step in range(25)
        ]
        def zone_rows(_path, zone):
            if zone == 1:
                raise FileNotFoundError("missing rotor history")
            return rows
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", side_effect=zone_rows
        ):
            result = campaign.summarize_history(
                Path("unused"), "tractor4", params,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0,
            )
        self.assertFalse(result["history_complete"])
        self.assertEqual(result["zone_forces"][1]["status"], "missing_history_file")
        self.assertIsNone(result["physical_model_valid"])
        self.assertNotIn("total_force_N", result)

    def test_duplicate_su2_history_columns_are_rejected(self):
        with TemporaryDirectory() as temp:
            path = Path(temp) / "rotor_history_0.csv"
            path.write_text(
                'Time_Iter,CFx," CFx "\n0,1,2\n', encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "duplicate CSV columns"):
                campaign.rows_by_zone(Path(temp), 0)

    def test_fractional_history_time_indices_are_rejected(self):
        rows = [
            {"Time_Iter": "13.5" if step == 13 else str(step),
             "CFx": "1", "CFy": "2", "CFz": "3",
             "CMx": "4", "CMy": "5", "CMz": "6"}
            for step in range(25)
        ]
        with patch.object(campaign, "_rotors", return_value=[]), patch.object(
            campaign, "rows_by_zone", return_value=rows
        ):
            with self.assertRaisesRegex(ValueError, "non-integer physical time index"):
                campaign.summarize_history(
                    Path("unused"), "tractor4", PARAMETERS,
                    revolutions=2, steps_per_revolution=12, rpm=30_000,
                    speed_mps=15.0, alpha_deg=4.0,
                )

    def test_bad_coupling_residuals_fail_otherwise_excellent_history(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        rows = [
            {
                "Time_Iter": str(step), "CFx": "1", "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6",
                "RMS_RES": "-2" if step == 0 else "-6",
            }
            for step in range(25)
        ]
        with TemporaryDirectory() as temp:
            driver = Path(temp) / "rotor_history_multizone.csv"
            with driver.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("Time_Iter", "BGS_RES[0]", "BGS_RES[1]"))
                for step in range(25):
                    value = "-2" if step < 24 else "-3"
                    writer.writerow((step, value, value))
            with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
                campaign, "rows_by_zone", return_value=rows
            ):
                result = campaign.summarize_history(
                    Path(temp), "tractor4", params,
                    revolutions=2, steps_per_revolution=12, rpm=30_000,
                    speed_mps=15.0, alpha_deg=4.0,
                )
        self.assertTrue(result["history_complete"])
        self.assertFalse(result["numerically_converged"])
        self.assertFalse(result["numerical_convergence"]["coupling_residuals_passed"])
        self.assertTrue(any("coupling" in reason.lower()
                            for reason in result["numerical_convergence"]["reasons"]))

    def test_missing_intermediate_history_step_marks_run_incomplete_and_withholds_averages(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="cw", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
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

        self.assertFalse(result["history_complete"])
        self.assertEqual(len(result["zone_forces"]), 2)
        for zone in result["zone_forces"]:
            self.assertFalse(zone["complete_time_steps"])
            self.assertEqual(zone["missing_time_step_indices_sample"], [13])
            self.assertNotIn("force_N", zone)

    def test_complete_history_alone_does_not_establish_numerical_convergence(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        rows = [
            {
                "Time_Iter": str(step),
                "CFx": "1", "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6",
            }
            for step in range(25)
        ]
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", return_value=rows
        ):
            result = campaign.summarize_history(
                Path("unused"), "tractor4", params,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0,
            )

        self.assertTrue(result["history_complete"])
        self.assertFalse(result["numerically_converged"])
        self.assertFalse(result["zone_forces"][0]["nonphysical_points"]["available"])
        self.assertFalse(result["numerical_convergence"]["nonphysical_points_passed"])
        self.assertFalse(result["design_ready"])
        self.assertIn("residual", " ".join(result["numerical_convergence"]["reasons"]).lower())

    def test_high_final_residuals_and_force_drift_fail_numerical_gate(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        rows = []
        for step in range(25):
            force = 2 if step > 12 else 1
            rows.append({
                "Time_Iter": str(step),
                "CFx": str(force), "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6",
                "RMS_RES": "-3",
            })
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", return_value=rows
        ):
            result = campaign.summarize_history(
                Path("unused"), "tractor4", params,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0,
            )

        self.assertTrue(result["history_complete"])
        self.assertFalse(result["numerically_converged"])
        self.assertTrue(result["numerical_convergence"]["residuals_passed"] is False)
        self.assertTrue(result["numerical_convergence"]["load_stability_passed"] is False)

    def test_converged_history_is_still_not_design_ready_without_sensitivity(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        rows = [
            {
                "Time_Iter": str(step),
                "CFx": "1", "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6",
                "RMS_RES": "-2" if step == 0 else "-6",
                "NONPHYSICAL_POINTS": "0",
            }
            for step in range(25)
        ]
        with TemporaryDirectory() as temp:
            driver = Path(temp) / "rotor_history_multizone.csv"
            with driver.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("Time_Iter", "BGS_RES[0]", "BGS_RES[1]"))
                for step in range(25):
                    value = "-2" if step == 0 else "-6"
                    writer.writerow((step, value, value))
            with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
                campaign, "rows_by_zone", return_value=rows
            ):
                result = campaign.summarize_history(
                    Path(temp), "tractor4", params,
                    revolutions=2, steps_per_revolution=12, rpm=30_000,
                    speed_mps=15.0, alpha_deg=4.0,
                )
        self.assertTrue(result["history_complete"])
        self.assertTrue(result["numerically_converged"])
        self.assertTrue(result["numerical_convergence"]["nonphysical_points_passed"])
        self.assertIsNone(result["physical_model_valid"])
        self.assertFalse(result["design_ready"])
        self.assertTrue(any("mesh-sensitivity" in item for item in result["design_readiness_blockers"]))

    def test_nonzero_nonphysical_points_block_numerical_convergence(self):
        params = SimpleNamespace(
            wing_planform_area=120_000,
            wing_tip_chord=100,
            wing_root_chord=200,
        )
        rotor = SimpleNamespace(
            index=1, x_m=0.0, y_m=0.0, z_m=0.0,
            side="left", prop_hand="CW", spin_sign=1,
            shaft_axis_xyz=(1.0, 0.0, 0.0), thrust_direction_xyz=(1.0, 0.0, 0.0),
        )
        rows = [
            {
                "Time_Iter": str(step),
                "CFx": "1", "CFy": "2", "CFz": "3",
                "CMx": "4", "CMy": "5", "CMz": "6",
                "RMS_RES": "-2" if step == 0 else "-6",
                "NONPHYSICAL_POINTS": "2" if step == 13 else "0",
            }
            for step in range(25)
        ]
        coupling = [
            {"zone": zone, "passed": True}
            for zone in range(2)
        ]
        with patch.object(campaign, "_rotors", return_value=[rotor]), patch.object(
            campaign, "rows_by_zone", return_value=rows
        ), patch.object(
            campaign, "summarize_coupling_residuals", return_value=coupling
        ):
            result = campaign.summarize_history(
                Path("unused"), "tractor4", params,
                revolutions=2, steps_per_revolution=12, rpm=30_000,
                speed_mps=15.0, alpha_deg=4.0,
            )
        self.assertTrue(result["history_complete"])
        self.assertFalse(result["numerically_converged"])
        self.assertFalse(result["numerical_convergence"]["nonphysical_points_passed"])
        self.assertEqual(result["zone_forces"][0]["nonphysical_points"]["maximum_count"], 2)
        self.assertIn(
            "nonphysical-point",
            " ".join(result["numerical_convergence"]["reasons"]).lower(),
        )

    def test_geometry_preflight_rejects_clearance_failures(self):
        design = Design(topology="coaxial8", coaxial_separation_mm=60.0)
        clearances = {"disk_left__wing": 0.0, "disk_right__wing": 4.0}
        with patch.object(campaign, "validate_variant", side_effect=ValueError("invalid clearance")), \
             patch.object(campaign, "clearance_report", return_value=clearances):
            result = campaign.geometry_preflight(PARAMETERS, design, min_clearance_mm=1.0)
        self.assertFalse(result["valid"])
        self.assertEqual(result["failures"], {"disk_left__wing": 0.0})

    def test_campaign_does_not_mesh_a_cad_invalid_topology(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            output_dir = root / "campaign"
            su2_dir = root / "su2"
            su2_dir.mkdir()
            (su2_dir / "SU2_CFD.exe").write_bytes(b"preflight fixture")
            args = SimpleNamespace(
                geometry="pr3_baseline", output_dir=output_dir,
                topologies=["tractor4"], rpm=30_000, speed_mps=15.0,
                alpha_deg=4.0, revolutions=2, steps_per_revolution=12,
                static_near_size_m=0.004, static_far_size_m=0.12,
                prop_near_size_m=0.0018, interface_size_m=0.003,
                max_hours_per_case=0.01,
            )
            fake_params = PARAMETERS
            with patch.dict(os.environ, {"SU2_RUN": str(su2_dir)}), patch.object(
                campaign, "airframe_parameters_for_geometry", return_value=fake_params
            ), patch.object(campaign, "geometry_values", return_value=(fake_params, 1.0, 1.0)), patch.object(
                campaign, "validate_variant", side_effect=ValueError("invalid clearance")
            ), patch.object(
                campaign, "clearance_report", return_value={"disk__wing": 0.0}
            ), patch.object(campaign, "available_memory_bytes", return_value=2 * 1024**3), patch.object(
                campaign.shutil, "disk_usage", return_value=SimpleNamespace(free=2 * 1024**3)
            ), patch.object(campaign, "generate_mesh") as generate_mesh:
                with redirect_stdout(StringIO()):
                    exit_code = campaign.run_campaign(args)

            self.assertEqual(exit_code, 1)
            generate_mesh.assert_not_called()
            result = json.loads(
                (output_dir / "campaign_results.json").read_text(encoding="utf-8")
            )["cases"][0]
            self.assertFalse(result["cad_geometry_valid"])
            self.assertFalse(result["mesh_valid"])

    def test_interference_diagnostic_preserves_failure_and_never_runs_solver(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            output_dir = root / "campaign"
            su2_dir = root / "su2"
            su2_dir.mkdir()
            (su2_dir / "SU2_CFD.exe").write_bytes(b"preflight fixture")
            args = SimpleNamespace(
                geometry="pr3_baseline", output_dir=output_dir,
                topologies=["tractor4"], rpm=30_000, speed_mps=15.0,
                alpha_deg=4.0, revolutions=2, steps_per_revolution=12,
                static_near_size_m=0.004, static_far_size_m=0.12,
                prop_near_size_m=0.0018, interface_size_m=0.003,
                max_hours_per_case=0.01, interference_diagnostic=True,
            )
            with patch.dict(os.environ, {"SU2_RUN": str(su2_dir)}), patch.object(
                campaign, "airframe_parameters_for_geometry", return_value=PARAMETERS
            ), patch.object(campaign, "geometry_values", return_value=(PARAMETERS, 1.0, 1.0)), patch.object(
                campaign, "validate_variant", side_effect=ValueError("invalid disk clearance")
            ), patch.object(
                campaign, "clearance_report", return_value={"disk__wing": 0.0}
            ), patch.object(campaign, "available_memory_bytes", return_value=2 * 1024**3), patch.object(
                campaign.shutil, "disk_usage", return_value=SimpleNamespace(free=2 * 1024**3)
            ), patch.object(campaign, "generate_mesh") as generate_mesh, patch.object(
                campaign, "run_solver"
            ) as run_solver:
                with redirect_stdout(StringIO()):
                    exit_code = campaign.run_campaign(args)

            self.assertEqual(exit_code, 1)
            generate_mesh.assert_not_called()
            run_solver.assert_not_called()
            result = json.loads(
                (output_dir / "campaign_results.json").read_text(encoding="utf-8")
            )["cases"][0]
            self.assertEqual(result["status"], "interference_diagnostic", result)
            self.assertTrue(result["interference_diagnostic"])
            self.assertFalse(result["feasible_for_ranking"])
            self.assertFalse(result["solver_exited_successfully"])
            self.assertIsNone(result["stationary_wall_gate_passed"])

    def test_passing_interference_diagnostic_is_clearance_only_without_su2(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            output_dir = root / "campaign"
            args = SimpleNamespace(
                geometry="pr3_baseline", output_dir=output_dir,
                topologies=["tractor4"], rpm=30_000, speed_mps=15.0,
                alpha_deg=4.0, revolutions=2, steps_per_revolution=12,
                static_near_size_m=0.004, static_far_size_m=0.12,
                prop_near_size_m=0.0018, interface_size_m=0.003,
                max_hours_per_case=0.01, interference_diagnostic=True,
            )
            env = {
                "SU2_RUN": str(root / "missing-su2-run"),
                "LOCALAPPDATA": str(root / "missing-local-app-data"),
            }
            with patch.dict(os.environ, env), patch.object(
                campaign, "airframe_parameters_for_geometry", return_value=PARAMETERS
            ), patch.object(
                campaign, "geometry_values", return_value=(PARAMETERS, 1.0, 1.0)
            ), patch.object(
                campaign, "validate_variant", return_value={"disk__wing": 2.0}
            ), patch.object(
                campaign, "export_topology_oml",
                side_effect=lambda _params, _design, path, _geometry: (
                    (path.write_bytes(b"geometry fixture"), path)[1]
                ),
            ), patch.object(
                campaign, "validate_rotor_zone_geometry",
                return_value={
                    "passed": True, "rotor_count": 4,
                    "rotating_zones_disjoint": True,
                    "stationary_components_clear": True,
                },
            ), patch.object(
                campaign, "available_memory_bytes"
            ) as memory_check, patch.object(
                campaign, "generate_mesh"
            ) as generate_mesh, patch.object(
                campaign, "run_solver"
            ) as run_solver:
                with redirect_stdout(StringIO()):
                    exit_code = campaign.run_campaign(args)

            self.assertEqual(exit_code, 0)
            memory_check.assert_not_called()
            generate_mesh.assert_not_called()
            run_solver.assert_not_called()
            results_path = output_dir / "campaign_results.json"
            results = json.loads(results_path.read_text(encoding="utf-8"))
            result = results["cases"][0]
            self.assertEqual(result["status"], "clearance_passed_diagnostic")
            self.assertTrue(result["cad_geometry_valid"])
            self.assertTrue(result["stationary_wall_gate_passed"])
            self.assertTrue(result["rotating_zone_gate_passed"])
            self.assertTrue(result["rotor_zone_geometry_preflight"]["passed"])
            self.assertFalse(result["feasible_for_ranking"])
            self.assertIsNone(results["configuration"]["solver_executable"])
            report = (output_dir / "REPORT.md").read_text(encoding="utf-8")
            self.assertIn("# CAD interference diagnostic: tractor4", report)
            self.assertIn("not invoked (CAD diagnostic only)", report)
            self.assertIn("| true | pass | pass | 2.0000 |", report)
            self.assertIn("geometry-only Gmsh intersection checks", report)
            self.assertIn("No volume mesh, flow history", report)
            self.assertNotIn("NaN", report)

    def test_interference_diagnostic_reports_stationary_intersection_and_skips_mesh(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            output_dir = root / "campaign"
            args = SimpleNamespace(
                geometry="pr3_baseline", output_dir=output_dir,
                topologies=["tractor4"], rpm=30_000, speed_mps=15.0,
                alpha_deg=4.0, revolutions=2, steps_per_revolution=12,
                static_near_size_m=0.004, static_far_size_m=0.12,
                prop_near_size_m=0.0018, interface_size_m=0.003,
                max_hours_per_case=0.01, interference_diagnostic=True,
            )
            env = {
                "SU2_RUN": str(root / "missing-su2-run"),
                "LOCALAPPDATA": str(root / "missing-local-app-data"),
            }
            with patch.dict(os.environ, env), patch.object(
                campaign, "airframe_parameters_for_geometry", return_value=PARAMETERS
            ), patch.object(
                campaign, "geometry_values", return_value=(PARAMETERS, 1.0, 1.0)
            ), patch.object(
                campaign, "validate_variant", return_value={"disk__wing": 2.0}
            ), patch.object(
                campaign, "export_topology_oml",
                side_effect=lambda _params, _design, path, _geometry: (
                    (path.write_bytes(b"geometry fixture"), path)[1]
                ),
            ), patch.object(
                campaign, "validate_rotor_zone_geometry",
                side_effect=campaign.StationaryZoneIntersectionError(
                    "Rotor 1 rotating zone intersects stationary surfaces: motor case"
                ),
            ), patch.object(
                campaign, "generate_mesh"
            ) as generate_mesh, patch.object(
                campaign, "run_solver"
            ) as run_solver:
                with redirect_stdout(StringIO()):
                    exit_code = campaign.run_campaign(args)

            self.assertEqual(exit_code, 1)
            generate_mesh.assert_not_called()
            run_solver.assert_not_called()
            result = json.loads(
                (output_dir / "campaign_results.json").read_text(encoding="utf-8")
            )["cases"][0]
            self.assertEqual(result["status"], "interference_diagnostic", result)
            self.assertTrue(result["cad_geometry_valid"])
            self.assertFalse(result["geometry_valid"])
            self.assertFalse(result["stationary_wall_gate_passed"])
            self.assertTrue(result["rotating_zone_gate_passed"])
            self.assertEqual(result["failure_type"], "stationary_zone_intersection")
            report = (output_dir / "REPORT.md").read_text(encoding="utf-8")
            self.assertIn("| fail | pass | 2.0000 |", report)

    def test_coaxial_campaign_requires_explicit_positive_separation(self):
        with self.assertRaisesRegex(ValueError, "coaxial-separation-mm"):
            campaign.validate_coaxial_separation(["coaxial8"], None)
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            campaign.validate_coaxial_separation(["coaxial8"], 0.0)
        campaign.validate_coaxial_separation(["tractor4"], None)

    def test_rotating_zone_rejects_stationary_airframe_surface_patches(self):
        with self.assertRaisesRegex(ValueError, "stationary surfaces"):
            mesh.require_no_stationary_zone_intersections(
                intersecting_components={"airframe wing"},
                rotor_index=2,
            )

    def test_overlapping_independent_rotor_zones_are_rejected(self):
        rotors = (
            SimpleNamespace(index=1, y_m=0.0, z_m=0.0),
            SimpleNamespace(index=2, y_m=0.0, z_m=0.0),
        )
        with self.assertRaisesRegex(mesh.RotorZoneOverlapError, "zones 1 and 2 overlap"):
            mesh.require_nonoverlapping_rotor_zones(
                rotors,
                {1: (0.0, 0.05, 0.02), 2: (0.04, 0.08, 0.02)},
            )
        mesh.require_nonoverlapping_rotor_zones(
            rotors,
            {1: (0.0, 0.03, 0.02), 2: (0.04, 0.08, 0.02)},
        )

    def test_rotor_history_monitors_propeller_marker_only(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "rotor_case.cfg").write_text("MULTIZONE_RESIDUAL= NO\n", encoding="utf-8")
            (root / "zone_static.cfg").write_text("GRID_MOVEMENT= NONE\n", encoding="utf-8")
            (root / "zone_rotor_1.cfg").write_text("GRID_MOVEMENT= RIGID_MOTION\n", encoding="utf-8")
            campaign.configure_histories(root, rotor_count=1, physical_steps=24)
            rotor_config = (root / "zone_rotor_1.cfg").read_text(encoding="utf-8")
        self.assertIn("MARKER_MONITORING= ( ROTOR_PROPELLER_1 )", rotor_config)
        self.assertNotIn("ROTOR_WALL_1", rotor_config)
        for key in ("MARKER_MONITORING", "MARKER_PLOTTING", "HISTORY_OUTPUT"):
            self.assertEqual(sum(line.split("=", 1)[0] == key
                                 for line in rotor_config.splitlines() if "=" in line), 1)

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


class TopologyMeshValidationTests(unittest.TestCase):
    def test_boolean_multiple_fluid_components_are_not_discarded(self):
        with self.assertRaisesRegex(ValueError, "disconnected fluid volumes"):
            mesh._single_fluid_volume([(3, 1), (3, 2)], "fixture")

    def test_minimal_valid_mesh_has_signed_quality_and_interface_coverage(self):
        with TemporaryDirectory() as temp:
            path = Path(temp) / "valid.su2"
            path.write_text(_two_zone_tetra_mesh(), encoding="utf-8")
            summaries = mesh.validate_multizone_mesh(path, rotor_count=1)
        self.assertEqual(len(summaries), 2)
        self.assertGreater(summaries[0]["minimum_tetra_quality"], 0.01)
        self.assertGreater(summaries[0]["p01_tetra_quality"], 0.01)
        self.assertIn("AIRFRAME_MOTOR", summaries[0]["boundary_areas_m2"])
        self.assertIn("ROTOR_PROPELLER_1", summaries[1]["boundary_areas_m2"])
        self.assertIn("rotating_propeller_walls", summaries[1]["surface_area_split_m2"])
        self.assertAlmostEqual(
            summaries[0]["marker_areas_m2"]["STATIC_IF_1"],
            summaries[1]["marker_areas_m2"]["ROTOR_IF_1"],
        )

    def test_inverted_tetrahedron_is_rejected(self):
        with TemporaryDirectory() as temp:
            path = Path(temp) / "inverted.su2"
            path.write_text(_two_zone_tetra_mesh(inverted_zone=1), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "inverted"):
                mesh.validate_multizone_mesh(path, rotor_count=1)

    def test_poor_quality_tetrahedron_is_rejected(self):
        with TemporaryDirectory() as temp:
            path = Path(temp) / "sliver.su2"
            path.write_text(_two_zone_tetra_mesh(flat_zone=1), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "quality"):
                mesh.validate_multizone_mesh(path, rotor_count=1)

    def test_disconnected_fluid_components_are_rejected(self):
        with TemporaryDirectory() as temp:
            path = Path(temp) / "disconnected.su2"
            path.write_text(_two_zone_tetra_mesh(disconnected_zone=1), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "disconnected"):
                mesh.validate_multizone_mesh(path, rotor_count=1)

    def test_mismatched_static_and_rotor_interface_coverage_is_rejected(self):
        with TemporaryDirectory() as temp:
            path = Path(temp) / "mismatched.su2"
            path.write_text(_two_zone_tetra_mesh(mismatched_interface=True), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "interface area"):
                mesh.validate_multizone_mesh(path, rotor_count=1)

    def test_equal_area_but_displaced_interfaces_are_rejected(self):
        with TemporaryDirectory() as temp:
            path = Path(temp) / "displaced.su2"
            path.write_text(_two_zone_tetra_mesh(shifted_interface=True), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "interface surface triangles do not match"):
                mesh.validate_multizone_mesh(path, rotor_count=1)


if __name__ == "__main__":
    unittest.main()
