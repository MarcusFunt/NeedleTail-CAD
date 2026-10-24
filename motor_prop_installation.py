"""Build one XING2 1404 plus 3x4 prop reference at the +Y motor station.

The motor and prop remain separate solids in an assembly Compound so the
shaft-to-hub interface can be inspected. This is a packaging reference, not a
manufacturer fit check or flight-ready propeller model.
"""
from __future__ import annotations

from pathlib import Path

from cadgen import build123d as bd
from cadgen import step

from airframe_geometry import PARAMETERS


REFERENCE_DIR = Path(__file__).resolve().parent / "references" / "propeller-kit" / "STEP"
MOTOR_FILE = REFERENCE_DIR / "iFlight_XING2_1404_4600KV_REFERENCE.step"
PROP_FILE = REFERENCE_DIR / "PROP_3x4_2B_CW.step"


def build_motor_prop_installation():
    """Place a reference motor/prop pair at the +Y wing station."""
    params = PARAMETERS
    params.validate_motor_prop_interface()
    missing = [path.name for path in (MOTOR_FILE, PROP_FILE) if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Missing propeller-kit reference STEP file(s): " + ", ".join(missing)
        )

    motor = bd.import_step(str(MOTOR_FILE))
    prop = bd.import_step(str(PROP_FILE))
    shaft_to_aircraft_x = bd.Rot(0, 90, 0)
    y = params.horizontal_motor_offset

    # The reference motor STEP uses local +Z as the shaft axis. The hub center
    # is located 18.5 mm ahead of the mount face, keeping its rear face flush
    # with the rounded pod nose at +X.
    motor_placed = (
        bd.Pos(params.motor_mount_face_x, y, 0)
        * shaft_to_aircraft_x
        * motor
    )
    prop_placed = (
        bd.Pos(params.prop_hub_center_x, y, 0)
        * shaft_to_aircraft_x
        * prop
    )

    assembly = bd.Compound(children=[motor_placed, prop_placed])
    assembly.label = "xing2_1404_plus_prop_3x4_reference"
    return assembly


@step
def motor_prop_installation():
    return build_motor_prop_installation()


if __name__ == "__main__":
    motor_prop_installation()
