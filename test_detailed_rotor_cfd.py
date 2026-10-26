"""Check physical normalization and rotor-axis moments in the CFD report."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

CFD = Path(__file__).parent / "cfd/su2"
sys.path.insert(0, str(CFD))
import detailed_rotor_cfd as cfd


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
                self.assertIn("AERO_COEFF", zone)
                self.assertIn("STANDARD_WALL_FUNCTION", zone)

    def test_nonfinite_yplus_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "rotor_surface_0_00001.csv").write_text("Y_Plus\nnan\n")
            self.assertIsNone(cfd.yplus(directory, 0))


if __name__ == "__main__":
    unittest.main()
