from __future__ import annotations

import unittest
import csv
import hashlib
import importlib.util
import json
import math
import tempfile
from pathlib import Path

from propulsion.benchmark import (
    BenchmarkCase,
    FrozenBEMTBackend,
    run_benchmark,
    summarize_records,
    _write_svg,
    zero_thrust_crossing,
    zero_thrust_crossings_by_rpm,
)
from propulsion.models import (
    BladeStation,
    DataProvenance,
    GeometryProvenance,
    MeasurementRecord,
    PolarProvenance,
    PropellerGeometry,
    PropellerResult,
    SourceConventions,
)


class _FixtureBackend:
    name = "fixture"
    revision = "fixture-1"
    initialization_ms = 0.0
    configuration = {"fixture_setting": "recorded"}

    def predict(self, case):
        return PropellerResult(
            thrust_n=0.5,
            torque_nm=0.01,
            shaft_power_w=2.0,
            ct=0.1,
            cp=0.05,
            advance_ratio=case.advance_ratio,
            efficiency=None,
            converged=True,
            within_validity_range=True,
            warnings=(),
            solver_name=self.name,
            solver_revision=self.revision,
            input_sha256="f" * 64,
            elapsed_ms=1.0,
            geometry_valid=True,
            convergence_evidence=("fixture",),
            model_scope="synthetic fixture",
        )


class PropulsionBenchmarkTests(unittest.TestCase):
    def test_runner_creates_nested_output_and_writes_auditable_artifacts(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "measurement.csv"
            source.write_text("rpm,CT,CP\n6000,0.1,0.05\n7000,0.11,0.06\n", encoding="utf-8")
            measurement = MeasurementRecord(
                propeller_id="fixture",
                regime="static",
                rpm=6000,
                advance_ratio=0,
                ct=0.1,
                cp=0.05,
                eta=None,
                provenance=DataProvenance.MEASURED,
                source_description="synthetic fixture",
                source_uri="fixture://measurement.csv",
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                source_row=2,
            )
            second_measurement = MeasurementRecord(
                propeller_id="fixture",
                regime="static",
                rpm=7000,
                advance_ratio=0,
                ct=0.11,
                cp=0.06,
                eta=None,
                provenance=DataProvenance.MEASURED,
                source_description="synthetic fixture",
                source_uri="fixture://measurement.csv",
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                source_row=3,
            )
            geometry = PropellerGeometry(
                id="fixture",
                source_uri="fixture://geometry",
                sha256="a" * 64,
                blade_count=2,
                diameter_m=0.1,
                hub_radius_m=0.005,
                stations=(BladeStation(0.01, 0.01, 0.2), BladeStation(0.04, 0.006, 0.1)),
                geometry_provenance=GeometryProvenance.MEASURED,
                polar_provenance=PolarProvenance.EXPERIMENTAL,
                source_conventions=SourceConventions("+X", "CW", "positive_pitch", "body", "SI"),
            )
            case = BenchmarkCase("fixture", "fixture propeller", "fixture", measurement, geometry, "measured")
            second_case = BenchmarkCase("fixture", "fixture propeller", "fixture", second_measurement, geometry, "measured")
            output = root / "reports" / "nested" / "run"

            run_benchmark([case, second_case], [_FixtureBackend()], output, suite="fixture")

            for artifact in (
                "cases.csv",
                "metrics.json",
                "run_manifest.json",
                "REPORT.md",
                "plots/fixture_fixture_static.svg",
            ):
                self.assertTrue((output / artifact).is_file(), artifact)
            manifest = json.loads((output / "run_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(len(manifest["local_implementation_sha256"]["propulsion/benchmark.py"]), 64)
            self.assertEqual(manifest["backend_configuration"]["fixture"], {"fixture_setting": "recorded"})
            with (output / "cases.csv").open(newline="", encoding="utf-8") as stream:
                case_row = next(csv.DictReader(stream))
            self.assertNotEqual(case_row["case_input_sha256"], case_row["input_sha256"])
            plot = (output / "plots/fixture_fixture_static.svg").read_text(encoding="utf-8")
            self.assertIn("rotor speed (RPM)", plot)
            self.assertIn("C_T vs rotor speed (RPM)", plot)
            self.assertIn("polyline", plot)

    def test_metrics_count_failures_and_keep_nonpositive_rows_out_of_mape(self):
        records = [
            {
                "propeller_id": "fixture",
                "regime": "forward",
                "advance_ratio": 0.2,
                "measured_ct": 0.1,
                "predicted_ct": 0.11,
                "measured_cp": 0.05,
                "predicted_cp": 0.04,
                "elapsed_ms": 2.0,
                "status": "ok",
            },
            {
                "propeller_id": "fixture",
                "regime": "forward",
                "advance_ratio": 0.8,
                "measured_ct": -0.01,
                "predicted_ct": -0.02,
                "measured_cp": -0.02,
                "predicted_cp": -0.01,
                "elapsed_ms": 4.0,
                "status": "ok",
            },
            {
                "propeller_id": "fixture",
                "regime": "forward",
                "advance_ratio": 0.4,
                "measured_ct": 0.04,
                "predicted_ct": None,
                "measured_cp": 0.03,
                "predicted_cp": None,
                "elapsed_ms": 1.0,
                "status": "failed",
            },
        ]

        result = summarize_records(records)["by_regime"]["forward"]

        self.assertEqual(result["total_measurements"], 3)
        self.assertEqual(result["evaluated"], 2)
        self.assertEqual(result["solver_failures"], 1)
        self.assertEqual(result["ct_positive_mape_count"], 1)
        self.assertAlmostEqual(result["ct_positive_mape_pct"], 10.0)
        self.assertAlmostEqual(result["cp_positive_mape_pct"], 20.0)
        self.assertEqual(result["cp_propulsive_mape_count"], 1)
        self.assertAlmostEqual(result["cp_propulsive_mape_pct"], 20.0)
        self.assertEqual(result["nonpositive_thrust_measurements"], 1)
        self.assertAlmostEqual(result["elapsed_ms_median"], 3.0)
        self.assertIsNone(result["measured_zero_thrust_j"])

    def test_metrics_report_validity_and_convergence_coverage(self):
        records = [
            {
                "propeller_id": "fixture",
                "regime": "forward",
                "advance_ratio": 0.2,
                "measured_ct": 0.1,
                "predicted_ct": 0.11,
                "measured_cp": 0.05,
                "predicted_cp": 0.04,
                "status": "ok",
                "within_validity_range": True,
            },
            {
                "propeller_id": "fixture",
                "regime": "forward",
                "advance_ratio": 0.4,
                "measured_ct": 0.06,
                "predicted_ct": 0.05,
                "measured_cp": 0.04,
                "predicted_cp": 0.03,
                "status": "ok",
                "within_validity_range": False,
            },
            {
                "propeller_id": "fixture",
                "regime": "forward",
                "advance_ratio": 0.6,
                "measured_ct": 0.01,
                "predicted_ct": None,
                "measured_cp": 0.02,
                "predicted_cp": None,
                "status": "failed",
                "within_validity_range": False,
            },
        ]

        result = summarize_records(records)["by_regime"]["forward"]

        self.assertEqual(result["within_validity_range_count"], 1)
        self.assertEqual(result["outside_validity_range_count"], 2)
        self.assertEqual(result["validity_unreported_count"], 0)
        self.assertEqual(result["in_validity_evaluated_count"], 1)
        self.assertAlmostEqual(result["in_validity_ct_mae"], 0.01)
        self.assertAlmostEqual(result["in_validity_ct_positive_mape_pct"], 10.0)
        self.assertAlmostEqual(result["in_validity_cp_mae"], 0.01)
        self.assertAlmostEqual(result["in_validity_cp_propulsive_mape_pct"], 20.0)
        self.assertEqual(result["converged_count"], 2)
        self.assertEqual(result["nonconverged_count"], 1)

    def test_zero_thrust_crossing_is_interpolated_only_when_sign_changes(self):
        records = [
            {"advance_ratio": 0.1, "measured_ct": 0.1, "predicted_ct": 0.05, "status": "ok"},
            {"advance_ratio": 0.3, "measured_ct": -0.1, "predicted_ct": -0.05, "status": "ok"},
        ]

        measured, predicted = zero_thrust_crossing(records)

        self.assertAlmostEqual(measured, 0.2)
        self.assertAlmostEqual(predicted, 0.2)
        self.assertEqual(zero_thrust_crossing(records[:1]), (None, None))

    def test_zero_thrust_crossings_are_kept_separate_for_each_rpm(self):
        records = []
        for rpm, start_j in ((5000.0, 0.1), (10000.0, 0.5)):
            records.extend((
                {"rpm": rpm, "advance_ratio": start_j, "measured_ct": 0.1,
                 "predicted_ct": 0.05, "status": "ok"},
                {"rpm": rpm, "advance_ratio": start_j + 0.2, "measured_ct": -0.1,
                 "predicted_ct": -0.05, "status": "ok"},
            ))

        self.assertEqual(zero_thrust_crossing(records), (None, None))
        crossings = zero_thrust_crossings_by_rpm(records)
        self.assertEqual([item["rpm"] for item in crossings], [5000.0, 10000.0])
        self.assertAlmostEqual(crossings[0]["measured_zero_thrust_j"], 0.2)
        self.assertAlmostEqual(crossings[0]["predicted_zero_thrust_j"], 0.2)
        self.assertAlmostEqual(crossings[1]["measured_zero_thrust_j"], 0.6)
        self.assertAlmostEqual(crossings[1]["predicted_zero_thrust_j"], 0.6)

        for row in records:
            row.update({
                "propeller_id": "fixture",
                "regime": "forward",
                "measured_cp": 0.04,
                "predicted_cp": 0.03,
            })
        group = summarize_records(records)["by_propeller"]["fixture"]["forward"]
        self.assertIsNone(group["measured_zero_thrust_j"])
        self.assertEqual(len(group["zero_thrust_crossings_by_rpm"]), 2)

    def test_forward_plot_draws_a_separate_prediction_line_for_each_rpm(self):
        records = []
        for rpm, start_j in ((5000.0, 0.1), (10000.0, 0.5)):
            for offset in (0.0, 0.2):
                records.append({
                    "rpm": rpm,
                    "advance_ratio": start_j + offset,
                    "measured_ct": 0.1 - offset,
                    "predicted_ct": 0.08 - offset,
                    "measured_cp": 0.04,
                    "predicted_cp": 0.03,
                    "status": "ok",
                })
        with tempfile.TemporaryDirectory() as temp:
            plot_path = Path(temp) / "rpm_sweeps.svg"
            _write_svg(plot_path, records, x_field="advance_ratio", x_label="advance ratio J")
            plot = plot_path.read_text(encoding="utf-8")

        self.assertEqual(plot.count('<polyline class="prediction-line"'), 4)
        self.assertEqual(plot.count('data-rpm="5000"'), 2)
        self.assertEqual(plot.count('data-rpm="10000"'), 2)
        self.assertIn("hover a line for its RPM", plot)

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "requires the optional aero extra")
    def test_frozen_bemt_adapter_accepts_a_new_propeller_family(self):
        from types import SimpleNamespace

        import numpy as np

        source = b"holdout-fixture"
        measurement = MeasurementRecord(
            propeller_id="graupner_9x4",
            regime="forward",
            rpm=4015,
            advance_ratio=0.317,
            ct=0.0272,
            cp=0.0206,
            eta=0.419,
            provenance=DataProvenance.MEASURED,
            source_description="synthetic parser fixture",
            source_uri="fixture://graupner/forward.txt",
            source_sha256=hashlib.sha256(source).hexdigest(),
            source_row=2,
        )
        stations = tuple(
            BladeStation(
                radius_m=ratio * 0.2286 / 2,
                chord_m=chord * 0.2286 / 2,
                twist_rad=math.radians(beta),
            )
            for ratio, chord, beta in (
                (0.15, 0.163, 36.4),
                (0.50, 0.144, 17.7),
                (1.00, 0.034, 6.39),
            )
        )
        geometry = PropellerGeometry(
            id="graupner_9x4",
            source_uri="fixture://graupner/geometry.txt",
            sha256="c" * 64,
            blade_count=2,
            diameter_m=0.2286,
            hub_radius_m=0.0,
            stations=stations,
            geometry_provenance=GeometryProvenance.DIGITIZED,
            polar_provenance=PolarProvenance.ASSUMED,
            source_conventions=SourceConventions("+X", "UNKNOWN", "positive_pitch", "axis", "r/R,c/R,beta_deg"),
        )
        case = BenchmarkCase("graupner_9x4", "Graupner CAM 9×4", "Graupner", measurement, geometry, "nominal 9 in")
        model = SimpleNamespace(
            np=np,
            RHO=1.225,
            B=2,
            NELEM=160,
            LIFT_SLOPE=2 * math.pi,
            CL_MAX=1.2,
            CD0=0.025,
            DRAG_K=0.055,
            F_FLOOR=0.15,
            NMAX=800,
        )
        backend = object.__new__(FrozenBEMTBackend)
        backend.module = model
        backend.name = "frozen_bemt"
        backend.revision = "fixture"
        backend.initialization_ms = 0.0

        result = backend.predict(case)

        self.assertTrue(math.isfinite(result.ct))
        self.assertTrue(math.isfinite(result.cp))
        self.assertEqual(result.solver_name, "frozen_bemt")


if __name__ == "__main__":
    unittest.main()
