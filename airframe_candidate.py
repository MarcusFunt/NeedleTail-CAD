"""Export the preliminary airfoil geometry candidate as separate STEP files."""
from __future__ import annotations

from cadgen import step

from airframe_geometry import (
    AIRFOIL_CANDIDATE_PARAMETERS,
    build_airframe_oml,
    build_engineering_assembly,
)


@step
def airframe_candidate():
    """Build the candidate engineering assembly without changing the default."""
    return build_engineering_assembly(AIRFOIL_CANDIDATE_PARAMETERS)


@step
def airframe_candidate_oml():
    """Build the candidate's single-solid outer mold line for evaluation."""
    return build_airframe_oml(AIRFOIL_CANDIDATE_PARAMETERS)


if __name__ == "__main__":
    airframe_candidate()
    airframe_candidate_oml()
