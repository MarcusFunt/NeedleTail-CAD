from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from pythrust.battery import BatteryState, RateMapBattery

from propulsion.operating_points import (
    MotorElectricalScenario,
    OperatingConstraints,
    PropellerMap,
    PropellerMapMetadata,
    XING2_1404_4600KV,
    export_propeller_map_cli,
    main,
    max_feasible_operating_point,
    solve_operating_point,
    sweep_uncertainty_envelope,
)
from pythrust.propulsion.solver import evaluate_propulsion_state
from pythrust.propulsion import PropellerSpec


def fixture_map() -> PropellerMap:
    metadata = PropellerMapMetadata(
        id="fixture_3x4",
        manufacturer="fixture",
        model="3x4 two blade",
        diameter_m=0.0762,
        pitch_m=0.1016,
        blade_count=2,
        source_uri="fixture://propeller-map.csv",
        source_description="synthetic operating-point fixture",
    )
    rows = []
    for rpm in (20_000.0, 40_000.0, 60_000.0):
        for advance_ratio, ct, cp in (
            (0.0, 0.090, 0.045),
            (0.2, 0.075, 0.041),
            (0.4, 0.050, 0.036),
            (0.6, 0.020, 0.030),
        ):
            rows.append((rpm, advance_ratio, ct, cp))
    return PropellerMap.from_rows(metadata, rows)


def fixture_battery(series: int) -> RateMapBattery:
    return RateMapBattery(
        name=f"synthetic {series}S LiPo",
        capacity_ah=1.5,
        cutoff_voltage_v=3.0,
        charge_voltage_v=4.2,
        max_current_a=20.0,
        series=series,
        parallel=1,
        dod=(0.0, 0.5, 1.0),
        ocv_v=(4.2, 3.75, 3.35),
        resistance_ohm=(0.012, 0.016, 0.025),
        source="synthetic test fixture",
    )


def fixture_constraints(
    series: int,
    *,
    max_motor_current_a: float = 15.0,
        max_winding_temperature_c: float = 200.0,
    max_esc_temperature_c: float = 200.0,
    mission_duration_s: float = 10.0,
) -> OperatingConstraints:
    return OperatingConstraints(
        motor_scenario=MotorElectricalScenario(
            label="fixture midpoint resistance",
            effective_resistance_ohm=0.0795,
            no_load_current_a=0.5,
            assumption_note="Synthetic bounded-sensitivity test scenario.",
        ),
        battery_model=fixture_battery(series),
        system_resistance_ohm=0.015,
        esc_efficiency=0.95,
        max_motor_current_a=max_motor_current_a,
        max_battery_current_a=15.0,
        max_rpm=58_000.0,
        max_winding_temperature_c=max_winding_temperature_c,
        motor_thermal_resistance_c_per_w=8.0,
        max_esc_temperature_c=max_esc_temperature_c,
        esc_thermal_resistance_c_per_w=8.0,
        ambient_temperature_c=25.0,
        mission_duration_s=mission_duration_s,
        throttle_values=(0.35, 0.5, 0.65, 0.8, 1.0),
    )


class OperatingPointTests(unittest.TestCase):
    def test_xing2_source_record_keeps_unknowns_and_interphase_convention_explicit(self):
        self.assertEqual(XING2_1404_4600KV.kv_rpm_per_v, 4600.0)
        self.assertEqual(XING2_1404_4600KV.interphase_resistance_ohm, 0.159)
        self.assertEqual(XING2_1404_4600KV.peak_current_a, 17.99)
        self.assertIsNone(XING2_1404_4600KV.no_load_current_a)
        self.assertIsNone(XING2_1404_4600KV.sustained_current_limit_a)
        self.assertTrue(XING2_1404_4600KV.source_pdf_sha256)
        self.assertEqual(len(XING2_1404_4600KV.effective_resistance_sensitivity_ohm), 2)
        source_record = Path(__file__).parent / "propulsion" / "data" / "xing2_1404_4600kv.json"
        data = json.loads(source_record.read_text(encoding="utf-8"))
        self.assertIsNone(data["motor"]["no_load_current_a"])
        self.assertIsNone(data["motor"]["sustained_current_limit_a"])
        self.assertEqual(data["source"]["datasheet_sha256"], XING2_1404_4600KV.source_pdf_sha256)

    def test_map_export_records_schema_provenance_and_deterministic_hashes(self):
        propeller = fixture_map()
        with tempfile.TemporaryDirectory() as temp:
            csv_path, metadata_path = propeller.export(Path(temp))
            with csv_path.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(
                    reader.fieldnames,
                    ["rpm", "advance_ratio", "thrust_coeff", "power_coeff"],
                )
                rows = list(reader)
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

        self.assertEqual(len(rows), 12)
        self.assertEqual(metadata["schema_version"], 1)
        self.assertEqual(metadata["id"], "fixture_3x4")
        self.assertEqual(metadata["source"]["uri"], "fixture://propeller-map.csv")
        self.assertEqual(len(metadata["data_sha256"]), 64)
        self.assertEqual(metadata["solver"]["package"], "setuav-pythrust")

    def test_exported_map_round_trips_and_detects_modified_data(self):
        propeller = fixture_map()
        with tempfile.TemporaryDirectory() as temp:
            csv_path, metadata_path = propeller.export(Path(temp))
            loaded = PropellerMap.from_files(csv_path, metadata_path)
            self.assertEqual(loaded.metadata.id, propeller.metadata.id)
            self.assertEqual(loaded.coefficients(40_000.0, 0.2), (0.075, 0.041))
            csv_path.write_text(csv_path.read_text(encoding="utf-8") + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "hash does not match"):
                PropellerMap.from_files(csv_path, metadata_path)

    def test_propeller_map_refuses_rpm_and_advance_ratio_clamping(self):
        propeller = fixture_map()
        with self.assertRaisesRegex(ValueError, "out of map"):
            propeller.coefficients(19_999.0, 0.2)
        with self.assertRaisesRegex(ValueError, "out of map"):
            propeller.coefficients(40_000.0, -0.01)
        with self.assertRaisesRegex(ValueError, "out of map"):
            propeller.coefficients(40_000.0, 0.61)

    def test_3s_and_4s_battery_states_solve_with_sag_and_power_balance(self):
        propeller = fixture_map()
        for series in (3, 4):
            with self.subTest(series=series):
                result = solve_operating_point(
                    speed_mps=5.0,
                    battery_state=BatteryState(soc=0.9),
                    propeller=propeller,
                    constraints=fixture_constraints(series),
                    throttle=0.8,
                )
                self.assertEqual(result.status, "feasible", result.reason)
                self.assertGreater(result.rpm, 0.0)
                self.assertGreater(result.thrust_n, 0.0)
                self.assertGreater(result.propeller_efficiency, 0.0)
                self.assertAlmostEqual(
                    result.shaft_power_w,
                    result.torque_nm * (result.rpm * 2.0 * 3.141592653589793 / 60.0),
                    places=8,
                )
                self.assertGreater(result.battery_current_a, 0.0)
                self.assertLess(result.battery_voltage_v, result.battery_open_circuit_voltage_v)
                self.assertGreaterEqual(result.battery_power_w, result.motor_power_w)
                self.assertGreater(result.battery_c_rate, 0.0)
                self.assertGreaterEqual(result.esc_loss_w, 0.0)
                self.assertIsNotNone(result.energy_consumed_wh)
                self.assertLess(result.final_soc, 0.9)

    def test_current_thermal_and_rpm_limits_mark_a_solved_point_infeasible(self):
        propeller = fixture_map()
        state = BatteryState(soc=0.9)
        current_limited = solve_operating_point(
            0.0, state, propeller, fixture_constraints(4, max_motor_current_a=0.1), 0.8
        )
        self.assertEqual(current_limited.status, "infeasible")
        self.assertIn("motor_current_limit", current_limited.reason)

        battery_limited_constraints = OperatingConstraints(
            **{**fixture_constraints(4).__dict__, "max_battery_current_a": 0.1}
        )
        battery_limited = solve_operating_point(0.0, state, propeller, battery_limited_constraints, 0.8)
        self.assertEqual(battery_limited.status, "infeasible")
        self.assertIn("battery_current", battery_limited.reason)

        thermal_limited = solve_operating_point(
            0.0,
            state,
            propeller,
            fixture_constraints(4, max_winding_temperature_c=25.01),
            0.8,
        )
        self.assertEqual(thermal_limited.status, "infeasible")
        self.assertIn("thermal_limit", thermal_limited.reason)

        esc_thermal_limited = solve_operating_point(
            0.0,
            state,
            propeller,
            fixture_constraints(4, max_esc_temperature_c=25.01),
            0.8,
        )
        self.assertEqual(esc_thermal_limited.status, "infeasible")
        self.assertIn("esc_thermal_limit", esc_thermal_limited.reason)

        rpm_constraints = fixture_constraints(4)
        rpm_constraints = OperatingConstraints(
            **{**rpm_constraints.__dict__, "max_rpm": 1000.0}
        )
        rpm_limited = solve_operating_point(0.0, state, propeller, rpm_constraints, 0.8)
        self.assertEqual(rpm_limited.status, "infeasible")
        self.assertIn("rpm_limit", rpm_limited.reason)

    def test_long_mission_rejects_battery_energy_exhaustion(self):
        result = solve_operating_point(
            0.0,
            BatteryState(soc=0.15),
            fixture_map(),
            fixture_constraints(4, mission_duration_s=20_000.0),
            0.8,
        )
        self.assertEqual(result.status, "infeasible")
        self.assertIn("battery_energy", result.reason)

    def test_speed_outside_every_supported_map_interval_is_out_of_map(self):
        result = solve_operating_point(
            100.0,
            BatteryState(soc=0.9),
            fixture_map(),
            fixture_constraints(4),
            0.8,
        )
        self.assertEqual(result.status, "out_of_map")

    def test_map_with_impossible_efficiency_cannot_be_marked_feasible(self):
        source = fixture_map()
        low_power_rows = tuple((rpm, j, ct, cp / 10.0) for rpm, j, ct, cp in source.rows)
        propeller = PropellerMap.from_rows(source.metadata, low_power_rows)
        result = solve_operating_point(
            5.0,
            BatteryState(soc=0.9),
            propeller,
            fixture_constraints(3),
            0.8,
        )
        self.assertEqual(result.status, "infeasible")
        self.assertIn("invalid_efficiency", result.reason)

    def test_zero_propeller_load_matches_motor_no_load_current(self):
        metadata = PropellerMapMetadata(
            id="zero_load",
            manufacturer="fixture",
            model="zero load",
            diameter_m=0.0762,
            pitch_m=0.1016,
            blade_count=2,
            source_uri="fixture://zero-load",
            source_description="synthetic zero torque fixture",
        )
        rows = tuple(
            (rpm, advance_ratio, 0.0, 0.0)
            for rpm in (20_000.0, 40_000.0, 60_000.0)
            for advance_ratio in (0.0, 0.2)
        )
        propeller = PropellerMap.from_rows(metadata, rows)
        scenario = fixture_constraints(3).motor_scenario
        ct, cp, _j, torque_nm, current_a, _v_back = evaluate_propulsion_state(
            scenario.to_pythrust_motor(),
            PropellerSpec(diameter_m=metadata.diameter_m, blade_count=2, pitch_m=metadata.pitch_m),
            propeller.to_pythrust_entry(),
            1.225,
            0.0,
            40_000.0,
        )
        self.assertEqual((ct, cp, torque_nm), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(current_a, scenario.no_load_current_a)

    def test_equilibrium_exception_is_reported_as_solver_failed(self):
        with patch("propulsion.operating_points.brentq", side_effect=RuntimeError("fixture failure")):
            result = solve_operating_point(
                0.0, BatteryState(soc=0.9), fixture_map(), fixture_constraints(3), 0.8
            )
        self.assertEqual(result.status, "solver_failed")
        self.assertIn("fixture failure", result.reason)

    def test_max_feasible_point_and_uncertainty_envelope_are_repeatable(self):
        propeller = fixture_map()
        constraints = fixture_constraints(3)
        state = BatteryState(soc=0.9)
        point = max_feasible_operating_point(0.0, state, propeller, constraints)
        self.assertEqual(point.status, "feasible", point.reason)
        sampled_points = [
            solve_operating_point(0.0, state, propeller, constraints, value)
            for value in constraints.throttle_values
        ]
        sampled_feasible = [sample for sample in sampled_points if sample.status == "feasible"]
        self.assertGreaterEqual(point.thrust_n, max(sample.thrust_n for sample in sampled_feasible))
        self.assertEqual(point, max_feasible_operating_point(0.0, state, propeller, constraints))

        bounded_constraints = fixture_constraints(3, max_motor_current_a=7.5)
        bounded_point = max_feasible_operating_point(0.0, state, propeller, bounded_constraints)
        self.assertEqual(bounded_point.status, "feasible")
        self.assertGreater(bounded_point.throttle, 0.8)
        self.assertLess(bounded_point.throttle, 1.0)
        self.assertLessEqual(bounded_point.motor_current_a, 7.5)

        scenarios = (
            constraints.motor_scenario,
            MotorElectricalScenario(
                label="higher resistance / no-load sensitivity",
                effective_resistance_ohm=0.159,
                no_load_current_a=1.0,
                assumption_note="Synthetic upper sensitivity fixture.",
            ),
        )
        envelope = sweep_uncertainty_envelope(0.0, state, propeller, constraints, scenarios)
        self.assertEqual(envelope.scenario_count, 2)
        self.assertGreater(envelope.thrust_min_n, 0.0)
        self.assertGreaterEqual(envelope.thrust_max_n, envelope.thrust_min_n)
        self.assertGreaterEqual(envelope.battery_power_max_w, envelope.battery_power_min_w)

    def test_export_cli_normalizes_a_user_map_and_preserves_source_hash(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.csv"
            source.write_text(
                "rpm,advance_ratio,thrust_coeff,power_coeff\n"
                "20000,0,0.09,0.045\n20000,0.2,0.07,0.04\n"
                "40000,0,0.09,0.045\n40000,0.2,0.07,0.04\n",
                encoding="utf-8",
            )
            metadata = root / "source.json"
            metadata.write_text(json.dumps({
                "id": "cli_fixture",
                "manufacturer": "fixture",
                "model": "test",
                "diameter_m": 0.0762,
                "pitch_m": 0.1016,
                "blade_count": 2,
                "source_uri": "fixture://source.csv",
                "source_description": "synthetic CLI input",
            }), encoding="utf-8")
            output = root / "out"

            export_propeller_map_cli([str(source), str(metadata), str(output)])

            self.assertTrue((output / "propeller_map.csv").is_file())
            manifest = json.loads((output / "propeller_map.json").read_text(encoding="utf-8"))
            self.assertEqual(len(manifest["source"]["sha256"]), 64)

    def test_sweep_cli_writes_auditable_points_and_combined_uncertainty_envelope(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            map_dir = root / "map"
            fixture_map().export(map_dir)
            battery_path = root / "battery.json"
            battery_path.write_text(json.dumps({
                "name": "synthetic test cell",
                "source": "synthetic fixture; not a product specification",
                "cell": {
                    "capacity_ah": 1.5,
                    "cutoff_voltage_v": 3.0,
                    "charge_voltage_v": 4.2,
                    "max_current_a": 20.0,
                },
                "curves": {
                    "dod": [0.0, 0.5, 1.0],
                    "ocv_v": [4.2, 3.75, 3.35],
                    "resistance_ohm": [0.012, 0.016, 0.025],
                },
            }), encoding="utf-8")
            scenario = {
                "label": "fixture low resistance",
                "effective_resistance_ohm": 0.0795,
                "no_load_current_a": 0.5,
                "assumption_note": "Synthetic CLI fixture.",
            }
            constraints_path = root / "constraints.json"
            constraints_path.write_text(json.dumps({
                "motor_scenario": scenario,
                "motor_scenarios": [
                    scenario,
                    {
                        "label": "fixture high resistance",
                        "effective_resistance_ohm": 0.159,
                        "no_load_current_a": 1.0,
                        "assumption_note": "Synthetic upper sensitivity fixture.",
                    },
                ],
                "system_resistance_ohm": 0.015,
                "esc_efficiency": 0.95,
                "max_motor_current_a": 15.0,
                "max_battery_current_a": 15.0,
                "max_rpm": 58000.0,
                "max_winding_temperature_c": 200.0,
                "motor_thermal_resistance_c_per_w": 8.0,
                "max_esc_temperature_c": 200.0,
                "esc_thermal_resistance_c_per_w": 8.0,
                "ambient_temperature_c": 25.0,
                "mission_duration_s": 10.0,
            }), encoding="utf-8")
            output_dir = root / "run"

            code = main([
                "sweep",
                "--map-csv", str(map_dir / "propeller_map.csv"),
                "--map-metadata", str(map_dir / "propeller_map.json"),
                "--battery-cell-json", str(battery_path),
                "--series", "3",
                "--parallel", "1",
                "--soc", "0.9",
                "--speed-mps", "0",
                "--constraints-json", str(constraints_path),
                "--output-directory", str(output_dir),
            ])

            self.assertEqual(code, 0)
            with (output_dir / "operating_points.csv").open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            manifest = json.loads((output_dir / "operating_points.json").read_text(encoding="utf-8"))

        self.assertGreaterEqual(len(rows), 2 * len(range(1, 41)))
        self.assertEqual(manifest["uncertainty_envelopes"]["all_motor_scenarios"]["scenario_count"], 2)
        self.assertEqual(len(manifest["motor_scenarios"]), 2)
        self.assertEqual(len(manifest["battery_input_sha256"]), 64)
        self.assertEqual(manifest["propeller_map_data_sha256"], fixture_map().data_sha256)


if __name__ == "__main__":
    unittest.main()
