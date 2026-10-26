"""SD8020-derived fuselage/pod alternative; current wing and installation layout."""
from __future__ import annotations

from dataclasses import replace

from cadgen import step

from airframe_geometry import PARAMETERS, build_engineering_assembly


@step
def airframe_sd8020():
    return build_engineering_assembly(replace(
        PARAMETERS, fuselage_body_airfoil="SD8020", motor_pod_body_airfoil="SD8020",
    ))


if __name__ == "__main__":
    airframe_sd8020()
