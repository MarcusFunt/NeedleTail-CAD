from __future__ import annotations

from cadgen import step

from airframe_geometry import PARAMETERS, build_airframe_oml


@step
def airframe_oml():
    return build_airframe_oml(PARAMETERS)


if __name__ == "__main__":
    airframe_oml()
