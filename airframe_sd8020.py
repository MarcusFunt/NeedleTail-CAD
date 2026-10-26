"""Preliminary candidate geometry with SD8020-derived fuselage/pod bodies."""
from __future__ import annotations

from dataclasses import replace

from cadgen import step

from airframe_geometry import AIRFOIL_CANDIDATE_PARAMETERS, build_engineering_assembly


@step
def airframe_sd8020():
    return build_engineering_assembly(replace(
        AIRFOIL_CANDIDATE_PARAMETERS,
        fuselage_body_airfoil="SD8020",
        motor_pod_body_airfoil="SD8020",
    ))


if __name__ == "__main__":
    airframe_sd8020()
