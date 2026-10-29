"""Check physical normalization and rotor-axis moments in the CFD report."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

CFD = Path(__file__).parent / "cfd/su2"
sys.path.insert(0, str(CFD))
import detailed_rotor_cfd as cfd
import generate_rotor_mesh as mesher


def write_mesh_fixture(path: Path, omit_marker: str | None = None,
                       tetra_row: str = "10 0 1 2 3") -> None:
    lines = ["NZONE= 5"]
    boundary_triangles = ["5 0 1 3", "5 0 2 3", "5 1 2 3",
                          "5 0 2 4", "5 0 1 4", "5 1 2 4"]
    static_markers = ["AIRFRAME_MOTOR", "FARFIELD", *(f"STATIC_IF_{i}" for i in range(1, 5))]
    coordinates = ["0 0 0 0", "1 0 0 1", "0.5 0.8660254037844386 0 2",
                   "0.5 0.2886751345948129 0.816496580927726 3",
                   "0.5 0.2886751345948129 -0.816496580927726 4"]
    tetrahedra = (tetra_row, "10 0 2 1 4")
    for zone in range(1, 6):
        lines.extend((f"IZONE= {zone}", "NDIME= 3", "NELEM= 2", *tetrahedra,
                      "NPOIN= 5", *coordinates))
        markers = ({name: [triangle] for name, triangle in zip(static_markers, boundary_triangles)}
                   if zone == 1 else {
                       f"ROTOR_PROPELLER_{zone - 1}": [
                           triangle for triangle in boundary_triangles
                           if triangle != boundary_triangles[zone]
                       ],
                       f"ROTOR_IF_{zone - 1}": [boundary_triangles[zone]],
                   })
        if omit_marker:
            markers.pop(omit_marker, None)
        lines.append(f"NMARK= {len(markers)}")
        for marker, triangles in markers.items():
            lines.extend((f"MARKER_TAG= {marker}", f"MARKER_ELEMS= {len(triangles)}", *triangles))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class DetailedRotorReportingTests(unittest.TestCase):
    def test_rotor_force_and_axis_torque_use_common_dimensional_reference(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for zone in range(5):
                with (directory / f"rotor_history_{zone}.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("Time_Iter", "CFx", "CFy", "CFz", "CMx", "CMy", "CMz", "rms[Rho]"))
                    for step in range(2401):
                        # Two coupling iterates: only the final iterate is physical output.
                        writer.writerow((step, 99, 99, 99, 99, 99, 99, -1))
                        writer.writerow((step, 1, 2, 3, 4, 5, 6, -6))
                with (directory / f"rotor_surface_{zone}_02400.csv").open("w") as stream:
                    stream.write('"Y_Plus"\n10\n20\n30\n')
            result = cfd.summarize_case(directory, 120, 20)
            force = cfd.Q * cfd.AREA
            self.assertAlmostEqual(result["total_force_N"]["x"], 5*force)
            first_rotor = result["zone_forces"][1]
            y, z = cfd.PARAMETERS.propeller_mount_stations[0]
            expected_torque = 4*force*cfd.MAC - (y*1e-3*3*force-z*1e-3*2*force)
            self.assertAlmostEqual(first_rotor["shaft_torque_Nm"], expected_torque)
            self.assertEqual(first_rotor["y_plus"]["median"], 20)
            self.assertEqual(first_rotor["last_5rev_vs_previous_5rev_relative_change"]["x"], 0)
            self.assertFalse(result["design_ready"])
            self.assertTrue(result["run_complete"])

    def test_zone_config_records_forces_and_consistent_moving_reference(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            config = cfd.write_case(directory, 240, 100).read_text()
            self.assertIn(f"MACH_MOTION= {cfd.SPEED/cfd.SOUND:.12g}", config)
            self.assertIn("AOA= 176.0", config)
            self.assertIn("TIME_ITER= 24001", config)
            self.assertIn("WRT_ZONE_HIST= YES", config)
            for index in range(1, 5):
                zone = (directory / f"zone_rotor_{index}.cfg").read_text()
                self.assertIn("MARKER_HEATFLUX", zone)
                self.assertNotIn("MARKER_EULER", zone)
                self.assertIn(f"MARKER_MONITORING= ( ROTOR_PROPELLER_{index} )", zone)
                self.assertNotIn(f"ROTOR_WALL_{index}", zone)
                self.assertIn("AERO_COEFF", zone)
                self.assertIn("STANDARD_WALL_FUNCTION", zone)

    def test_candidate_geometry_uses_candidate_reference_area_and_station(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            params, area, _ = cfd.geometry_values("airfoil_candidate")
            config = cfd.write_case(directory, 120, 20, geometry="airfoil_candidate")
            self.assertIn(f"REF_AREA= {area:.12g}", config.read_text())
            self.assertIn("Geometry: airfoil_candidate", (directory / "rotor_run_parameters.txt").read_text())
            zone = (directory / "zone_rotor_1.cfg").read_text()
            y, z = params.propeller_mount_stations[0]
            self.assertIn(f"{y * 0.001:.9f}", zone)
            self.assertIn(f"{z * 0.001:.9f}", zone)

    def test_config_writer_refuses_to_overwrite_existing_case(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            cfd.write_case(directory, 120, 20)
            with self.assertRaises(FileExistsError):
                cfd.write_case(directory, 120, 20)

    def test_prepared_campaign_binds_geometry_sources_budget_and_solver(self):
        configuration = cfd.campaign_configuration("pr3_baseline", 100, 18)
        plan = {"geometry": "pr3_baseline", "source_sha256": {"source.py": "abc"},
                "configuration": configuration,
                "configuration_sha256": cfd.digest_json(configuration),
                "preflight_config_sha256": "config-hash", "solver_sha256": "solver-hash",
                "revolutions": 100, "hours_per_case": 18}
        with patch.object(cfd, "hashes", return_value={"source.py": "abc"}):
            cfd.verify_prepared_campaign(plan, "pr3_baseline", 100, 18, "solver-hash", "config-hash")
            with self.assertRaisesRegex(RuntimeError, "Selected geometry"):
                cfd.verify_prepared_campaign(plan, "airfoil_candidate", 100, 18, "solver-hash", "config-hash")
            with self.assertRaisesRegex(RuntimeError, "changed after preparation"):
                with patch.object(cfd, "hashes", return_value={"source.py": "changed"}):
                    cfd.verify_prepared_campaign(plan, "pr3_baseline", 100, 18, "solver-hash", "config-hash")
            with self.assertRaisesRegex(RuntimeError, "SU2 executable changed"):
                cfd.verify_prepared_campaign(plan, "pr3_baseline", 100, 18, "different-hash", "config-hash")
            with self.assertRaisesRegex(RuntimeError, "configuration files changed"):
                cfd.verify_prepared_campaign(plan, "pr3_baseline", 100, 18, "solver-hash", "changed-config")
            with self.assertRaisesRegex(RuntimeError, "run budget"):
                cfd.verify_prepared_campaign(plan, "pr3_baseline", 99, 18, "solver-hash", "config-hash")

    def test_campaign_output_guard_refuses_existing_artifacts(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            cfd.assert_fresh_campaign_outputs(directory)
            (directory / "progress.json").write_text("{}")
            with self.assertRaisesRegex(FileExistsError, "progress.json"):
                cfd.assert_fresh_campaign_outputs(directory)

    def test_solver_stops_before_resource_floor_and_terminates_child(self):
        class Child:
            returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = -15

            def wait(self):
                return self.returncode

        with tempfile.TemporaryDirectory() as temporary:
            child = Child()
            with (patch.object(cfd, "solver", return_value=Path(temporary) / "SU2_CFD.exe"),
                  patch.object(cfd, "available_memory_bytes", return_value=0),
                  patch.object(cfd.subprocess, "Popen", return_value=child)):
                with self.assertRaisesRegex(RuntimeError, "Less than 0.8 GiB RAM"):
                    cfd.run_solver(Path(temporary), "case.cfg", 1)
            self.assertIsNone(child.returncode)

    def test_solver_terminates_child_when_memory_falls_below_floor(self):
        class Child:
            returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = -15

            def wait(self):
                return self.returncode

        with tempfile.TemporaryDirectory() as temporary:
            child = Child()
            with (patch.object(cfd, "solver", return_value=Path(temporary) / "SU2_CFD.exe"),
                  patch.object(cfd, "available_memory_bytes",
                               side_effect=[cfd.MIN_RUNTIME_MEMORY_BYTES + 1, 0]),
                  patch.object(cfd.subprocess, "Popen", return_value=child)):
                with self.assertRaisesRegex(RuntimeError, "stopped to preserve desktop"):
                    cfd.run_solver(Path(temporary), "case.cfg", 1)
            self.assertEqual(child.returncode, -15)

    def test_failure_report_preserves_resource_stop_reason(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            reason = "Less than 1.5 GiB disk free; stopped before filling disk"
            cfd.report(directory, [], reason, "airfoil_candidate")
            self.assertIn("stopped: " + reason, (directory / "REPORT.md").read_text())
            results = __import__("json").loads((directory / "campaign_results.json").read_text())
            self.assertEqual(results["failure"], reason)
            self.assertEqual(results["geometry"], "airfoil_candidate")

    def test_rotor_torque_and_shaft_power_signs_follow_rotation_direction(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            spins = tuple(rotor.spin_sign for rotor in mesher._rotors(cfd.PARAMETERS))
            for zone in range(5):
                with (directory / f"rotor_history_{zone}.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("Time_Iter", "CFx", "CFy", "CFz", "CMx", "CMy", "CMz", "rms[Rho]"))
                    spin = spins[zone - 1] if zone else 0
                    torque = -spin * 0.01
                    force_y, force_z = 0.2, 0.3
                    y, z = cfd.PARAMETERS.propeller_mount_stations[zone - 1] if zone else (0, 0)
                    origin_moment = torque + y * 1e-3 * force_z - z * 1e-3 * force_y
                    coeff_scale = cfd.Q * cfd.AREA
                    cmx = origin_moment / (coeff_scale * cfd.MAC)
                    for step in range(2401):
                        writer.writerow((step, 0, force_y / coeff_scale,
                                         force_z / coeff_scale, cmx, 0, 0, -6))
                (directory / f"rotor_surface_{zone}_02400.csv").write_text(
                    '"Y_Plus"\n10\n20\n30\n', encoding="utf-8")

            result = cfd.summarize_case(directory, 120, 20)
            expected_torques = tuple(-spin * 0.01 for spin in spins)
            for record, expected in zip(result["zone_forces"][1:], expected_torques):
                self.assertAlmostEqual(record["shaft_torque_Nm"], expected, places=10)
            expected_power = 4 * 0.01 * cfd.RPM * 2 * 3.141592653589793 / 60
            self.assertGreater(result["shaft_power_W"], 0)
            self.assertAlmostEqual(result["shaft_power_W"], expected_power, places=8)

    def test_multizone_history_file_and_bgs_residual_columns_are_reported(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for zone in range(5):
                with (directory / f"rotor_history_{zone}.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("Time_Iter", "CFx", "CFy", "CFz", "CMx", "CMy", "CMz", "rms[Rho]"))
                    for step in range(2401):
                        writer.writerow((step, 0, 0, 0, 0, 0, 0, -6))
                (directory / f"rotor_surface_{zone}_02400.csv").write_text(
                    '"Y_Plus"\n10\n20\n30\n', encoding="utf-8")

            with (directory / "rotor_history_multizone.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("Time_Iter", *(f"BGS_RES[{zone}]" for zone in range(5))))
                for step in range(2401):
                    writer.writerow((step, *([-6] * 5)))

            result = cfd.summarize_case(directory, 120, 20)

            self.assertEqual(result["worst_final_coupling_log_residual_in_sample"],
                             {f"BGS_RES[{zone}]": -6.0 for zone in range(5)})
            self.assertFalse(any("Coupled-zone residuals" in issue for issue in result["issues"]))

    def test_incomplete_history_is_not_reported_as_a_completed_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for zone in range(5):
                with (directory / f"rotor_history_{zone}.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("Time_Iter", "CFx", "CFy", "CFz", "CMx", "CMy", "CMz", "rms[Rho]"))
                    for step in range(2400):
                        writer.writerow((step, 1, 2, 3, 4, 5, 6, -6))
                (directory / f"rotor_surface_{zone}_02399.csv").write_text(
                    '"Y_Plus"\n10\n20\n30\n', encoding="utf-8")
            result = cfd.summarize_case(directory, 120, 20)
            self.assertFalse(result["run_complete"])
            self.assertEqual(result["status"], "incomplete")
            self.assertTrue(any("physical run incomplete" in issue for issue in result["issues"]))
            self.assertNotIn("total_force_N", result)

    def test_mesh_validator_accepts_well_formed_five_zone_fixture(self):
        with tempfile.TemporaryDirectory() as temporary:
            mesh = Path(temporary) / "valid.su2"
            write_mesh_fixture(mesh)
            self.assertEqual([row["zone"] for row in mesher.validate_multizone_mesh(mesh)],
                             [1, 2, 3, 4, 5])

    def test_mesh_validator_rejects_missing_marker_and_bad_tetrahedra(self):
        with tempfile.TemporaryDirectory() as temporary:
            mesh = Path(temporary) / "invalid.su2"
            write_mesh_fixture(mesh, omit_marker="FARFIELD")
            with self.assertRaisesRegex(ValueError, "missing required markers"):
                mesher.validate_multizone_mesh(mesh)
            write_mesh_fixture(mesh, tetra_row="10 0 0 2 3")
            with self.assertRaisesRegex(ValueError, "degenerate tetrahedron"):
                mesher.validate_multizone_mesh(mesh)
            write_mesh_fixture(mesh, tetra_row="10 0 1 2 5")
            with self.assertRaisesRegex(ValueError, "missing point"):
                mesher.validate_multizone_mesh(mesh)

    def test_nonfinite_yplus_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "rotor_surface_0_00001.csv").write_text("Y_Plus\nnan\n")
            self.assertIsNone(cfd.yplus(directory, 0))


if __name__ == "__main__":
    unittest.main()
