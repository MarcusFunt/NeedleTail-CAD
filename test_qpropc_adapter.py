from __future__ import annotations

import hashlib
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from propulsion.qpropc import (
    QPropCBackend,
    _polar_coverage,
    _operating_samples,
    frozen_bemt_common_polar_parameters,
    polar_sample_in_coverage,
)
from validation.qpropc.install_qpropc import verify_archive_sha256


class QPropCAdapterTests(unittest.TestCase):
    def test_controlled_common_polar_matches_frozen_bemt_coefficients(self):
        self.assertEqual(
            frozen_bemt_common_polar_parameters(),
            {
                "CL0": 0.0,
                "CL_a": 2 * 3.141592653589793,
                "CLmin": -1.2,
                "CLmax": 1.2,
                "CD0": 0.025,
                "CD2u": 0.055,
                "CD2l": 0.055,
                "CLCD0": 0.0,
                "REref": 70000.0,
                "REexp": 0.0,
            },
        )

    def test_predict_leaves_python_owned_rotor_for_python_gc_and_releases_performance(self):
        performance = SimpleNamespace(
            nelems=1,
            T=1.0,
            Q=0.1,
            CT=0.12,
            CP=0.08,
            J=0.3,
            residuals=[1e-8],
        )
        module = SimpleNamespace(
            create_section=Mock(return_value=object()),
            create_rotor=Mock(return_value=object()),
            qprop=Mock(return_value=performance),
            free_rotor_performance=Mock(),
            free_rotor=Mock(),
        )
        backend = object.__new__(QPropCBackend)
        backend._closed = False
        backend.native_geometry_paths = {}
        backend.native_geometry_sha256_by_propeller = {}
        backend.out_of_scope_propellers = ()
        backend.module = module
        backend.airfoil = object()
        backend.polar_sha256 = "a" * 64
        backend.rho_kg_m3 = 1.225
        backend.dynamic_viscosity_pa_s = 1.81e-5
        backend.speed_of_sound_mps = 0.0
        backend.polar_mode = "common_analytic"
        backend.input_configuration = {
            "qpropc_revision": "2603@test",
            "portable_archive_sha256": "d" * 64,
            "portable_file_sha256": {},
            "polar_mode": "common_analytic",
            "polar_sha256": "a" * 64,
            "rho_kg_m3": 1.225,
            "dynamic_viscosity_pa_s": 1.81e-5,
            "speed_of_sound_mps": 0.0,
        }
        backend.polar_source_description = "test polar"
        backend.name = "qpropc_test"
        backend.revision = "2603@test"
        station = SimpleNamespace(chord_m=0.01, twist_rad=0.2, radius_m=0.03)
        geometry = SimpleNamespace(
            validate_for_prediction=Mock(),
            diameter_m=0.1,
            blade_count=2,
            stations=(station, station),
            geometry_provenance=SimpleNamespace(value="measured"),
        )
        case = SimpleNamespace(
            propeller_id="synthetic",
            input_sha256="b" * 64,
            geometry=geometry,
            rpm=3000.0,
            advance_ratio=0.3,
        )

        first_result = backend.predict(case)

        module.free_rotor_performance.assert_called_once_with(performance)
        module.free_rotor.assert_not_called()
        self.assertFalse(first_result.within_validity_range)
        self.assertIn("common_analytic_control_polar_is_not_empirical_validity_evidence", first_result.warnings)
        self.assertNotEqual(first_result.input_sha256, case.input_sha256)

        backend.rho_kg_m3 = 1.0
        backend.input_configuration = {**backend.input_configuration, "rho_kg_m3": 1.0}
        second_result = backend.predict(case)
        self.assertNotEqual(first_result.input_sha256, second_result.input_sha256)
        self.assertEqual(module.free_rotor_performance.call_count, 2)
        module.free_rotor_performance.assert_called_with(performance)

    def test_operating_reynolds_uses_density_chord_and_section_speed(self):
        rotor = SimpleNamespace(
            nsections=2,
            sections=(
                SimpleNamespace(r=0.01, c=0.02, beta=math.radians(20.0)),
                SimpleNamespace(r=0.03, c=0.04, beta=math.radians(10.0)),
            ),
        )
        performance = SimpleNamespace(nelems=1, r=[0.02], W=[10.0], phi=[math.radians(5.0)])

        samples = _operating_samples(rotor, performance, 1.225, 1.0e-5)

        self.assertEqual(len(samples), 1)
        self.assertAlmostEqual(samples[0][0], 1.225 * 10.0 * 0.03 / 1.0e-5)
        self.assertAlmostEqual(samples[0][1], 10.0)

    def test_release_archive_hash_verifier_accepts_and_rejects_bytes(self):
        payload = b"pinned release fixture"
        expected = hashlib.sha256(payload).hexdigest()

        self.assertEqual(verify_archive_sha256(payload, expected), expected)
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            verify_archive_sha256(payload, "0" * 64)

    def test_xfoil_validity_requires_reynolds_and_alpha_bracket(self):
        limits = ((30000.0, -15.0, 15.0), (50000.0, -12.0, 12.0), (100000.0, -10.0, 10.0))

        self.assertTrue(polar_sample_in_coverage(limits, 40000.0, 12.0))
        self.assertFalse(polar_sample_in_coverage(limits, 20000.0, 0.0))
        self.assertFalse(polar_sample_in_coverage(limits, 40000.0, 12.01))
        self.assertFalse(polar_sample_in_coverage(limits, 75000.0, 11.0))

    def test_polar_report_uses_dimensionless_reynolds_number(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "polar.txt"
            path.write_text("Re = 0.030\n-15 -0.8 0.1\n15 0.8 0.1\n", encoding="utf-8")

            coverage = _polar_coverage((path,))

        self.assertEqual(coverage["re_min"], 30000.0)
        self.assertEqual(coverage["re_max"], 30000.0)

    def test_adapter_rejects_unmanifested_portable_library(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "qprop.py").write_text("", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "install_qpropc.py"):
                QPropCBackend(root, root)


if __name__ == "__main__":
    unittest.main()
