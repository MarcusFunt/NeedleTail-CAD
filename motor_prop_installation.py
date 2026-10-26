"""Build a centered XING2 1404 and 3x4 prop reference at the +Y station.

The pod, motor, and prop remain separate solids in an assembly Compound so the
mounting and shaft-to-hub interfaces can be inspected. This is a packaging
reference, not a manufacturer fit check or flight-ready propeller model.
"""
from __future__ import annotations

from pathlib import Path

from cadgen import build123d as bd
from cadgen import step

from airframe_geometry import PARAMETERS, AirframeParameters, build_motor_pod


REFERENCE_DIR = Path(__file__).resolve().parent / "references" / "propeller-kit" / "STEP"
MOTOR_FILE = REFERENCE_DIR / "iFlight_XING2_1404_4600KV_REFERENCE.step"
PROP_FILE = REFERENCE_DIR / "PROP_3x4_2B_CW.step"


def build_motor_prop_installation(params: AirframeParameters = PARAMETERS):
    """Place a reference motor/prop pair on the +Y pod's forward mount face."""
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
    motor_z = params.motor_axis_z_offset_from_pod_center
    pod_placed = bd.Pos(params.motor_pod_center_x, y, 0) * build_motor_pod(params)

    # The motor's local +Z shaft axis is mapped onto aircraft +X. Its mount face
    # sits on the pod's flat +X end, ahead of the wing leading edge.
    motor_placed = (
        bd.Pos(params.motor_mount_face_x, y, motor_z)
        * shaft_to_aircraft_x
        * motor
    )
    prop_placed = (
        bd.Pos(params.prop_hub_center_x, y, motor_z)
        * shaft_to_aircraft_x
        * prop
    )

    assembly = bd.Compound(children=[pod_placed, motor_placed, prop_placed])
    assembly.label = "motor_pod_with_xing2_1404_and_prop_3x4"
    return assembly


@step
def motor_prop_installation():
    return build_motor_prop_installation()


if __name__ == "__main__":
    motor_prop_installation()
