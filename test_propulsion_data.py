from __future__ import annotations

import hashlib
import math
import tempfile
import unittest
from pathlib import Path

from propulsion.data import parse_coefficient_measurements, parse_radial_geometry
from propulsion.models import (
    DataProvenance,
    GeometryProvenance,
    PolarProvenance,
    PropellerResult,
    SourceConventions,
)


class PropulsionDataTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_csv_measurements_keep_exact_source_provenance_and_negative_coefficients(self):
        path = self.root / "forward.csv"
        content = "J,CT_measured,CP_measured,eta\n0.2,0.08,0.04,0.4\n0.8,-0.01,-0.02,0.4\n"
        path.write_text(content, encoding="utf-8", newline="")

        rows = parse_coefficient_measurements(
            path,
            propeller_id="fixture",
            regime="forward",
            rpm_default=12_000,
            provenance=DataProvenance.TRANSCRIBED,
            source_description="synthetic transcribed fixture",
        )

        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0].rpm, 12_000)
        self.assertEqual(rows[0].advance_ratio, 0.2)
        self.assertEqual(rows[0].provenance, DataProvenance.TRANSCRIBED)
        self.assertEqual(rows[0].source_description, "synthetic transcribed fixture")
        self.assertEqual(rows[0].source_row, 2)
        self.assertEqual(rows[0].source_sha256, hashlib.sha256(content.encode()).hexdigest())
        self.assertEqual(rows[1].ct, -0.01)
        self.assertEqual(rows[1].cp, -0.02)

    def test_whitespace_static_measurements_parse_rpm_and_leave_advance_zero(self):
        path = self.root / "static.txt"
        path.write_text("RPM CT CP\n6000 0.12 0.06\n", encoding="utf-8")

        row = parse_coefficient_measurements(
            path,
            propeller_id="fixture",
            regime="static",
            provenance=DataProvenance.MEASURED,
            source_description="wind tunnel",
        )[0]

        self.assertEqual((row.rpm, row.advance_ratio), (6000.0, 0.0))
        self.assertEqual((row.ct, row.cp), (0.12, 0.06))
        self.assertEqual(row.provenance, DataProvenance.MEASURED)

    def test_measurements_reject_missing_required_coefficients(self):
        path = self.root / "incomplete.csv"
        path.write_text("RPM,CT\n6000,0.1\n", encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "CP"):
            parse_coefficient_measurements(
                path,
                propeller_id="fixture",
                regime="static",
                provenance=DataProvenance.TRANSCRIBED,
                source_description="fixture",
            )

    def test_digitized_radial_geometry_converts_radii_chords_and_twist_to_si(self):
        path = self.root / "geometry.csv"
        path.write_text(
            "r_over_R,c_over_R,beta_deg\n0.2,0.3,30\n0.8,0.1,5\n",
            encoding="utf-8",
        )

        geometry = parse_radial_geometry(
            path,
            propeller_id="fixture",
            diameter_m=0.1,
            hub_radius_m=0.005,
            blade_count=2,
            geometry_provenance=GeometryProvenance.DIGITIZED,
            polar_provenance=PolarProvenance.ASSUMED,
            source_conventions=SourceConventions(
                rotor_axis="+X",
                blade_hand="CW",
                section_sign="positive_pitch",
                frame="aircraft_body",
                units="r/R,c/R,beta_deg",
            ),
        )

        self.assertEqual(geometry.geometry_provenance, GeometryProvenance.DIGITIZED)
        self.assertAlmostEqual(geometry.stations[0].radius_m, 0.01)
        self.assertAlmostEqual(geometry.stations[0].chord_m, 0.015)
        self.assertAlmostEqual(geometry.stations[0].twist_rad, math.radians(30))
        self.assertEqual(geometry.sha256, hashlib.sha256(path.read_bytes()).hexdigest())

    def test_result_contract_requires_traceable_finite_solver_output(self):
        result = PropellerResult(
            thrust_n=1.0,
            torque_nm=0.02,
            shaft_power_w=10.0,
            ct=0.1,
            cp=0.05,
            advance_ratio=0.2,
            efficiency=0.4,
            converged=True,
            within_validity_range=True,
            warnings=(),
            solver_name="fixture",
            solver_revision="test",
            input_sha256="a" * 64,
            elapsed_ms=1.5,
            geometry_valid=True,
            convergence_evidence=("residual=1e-8",),
            model_scope="synthetic fixture",
        )
        self.assertTrue(result.converged)
        with self.assertRaisesRegex(ValueError, "finite"):
            PropellerResult(
                **{**result.__dict__, "thrust_n": math.nan}
            )


if __name__ == "__main__":
    unittest.main()
