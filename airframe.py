from __future__ import annotations

from cadgen import step

from airframe_geometry import PARAMETERS, build_engineering_assembly


@step
def airframe():
    """Build the labeled engineering assembly using library airfoils."""
    return build_engineering_assembly(PARAMETERS)


if __name__ == "__main__":
    airframe()
