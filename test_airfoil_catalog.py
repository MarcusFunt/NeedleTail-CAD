"""Tests that vendored airfoil data stays traceable to its published source."""

import hashlib
from pathlib import Path
import unittest

from airframe_geometry import AIRFOILS, _section_coordinates
from profiles.airfoil_coordinates import AIRFOIL_CATALOG, AIRFOIL_COORDINATES


class AirfoilCatalogTests(unittest.TestCase):
    def test_revised_candidate_profiles_are_vendored_with_source_hashes(self):
        profile_directory = Path(__file__).parent / "profiles" / "original"

        for name in (
            "E423",
            "NACA0010",
            "NACA0015",
            "NACA2412",
            "NACA2415",
            "NACA4412",
            "SD7037",
        ):
            with self.subTest(profile=name):
                path = profile_directory / f"{name.lower()}.dat"
                self.assertTrue(path.is_file())
                self.assertGreaterEqual(len(AIRFOIL_COORDINATES[name]), 4)
                self.assertEqual(AIRFOIL_CATALOG[name]["file"], path.name)
                self.assertTrue(AIRFOIL_CATALOG[name]["source_url"].startswith("https://"))
                self.assertEqual(
                    AIRFOIL_CATALOG[name]["sha256"],
                    hashlib.sha256(path.read_bytes()).hexdigest(),
                )

    def test_every_catalog_profile_converts_to_a_valid_section(self):
        self.assertEqual(set(AIRFOILS), set(AIRFOIL_COORDINATES))
        for name in AIRFOILS:
            with self.subTest(airfoil=name):
                upper, lower = _section_coordinates(name, 56.0, 0.3)
                self.assertEqual(upper[0], lower[0])
                self.assertTrue(all(u[1] > l[1] for u, l in zip(upper[1:], lower[1:])))


if __name__ == "__main__":
    unittest.main()
