from __future__ import annotations

from dataclasses import dataclass

from cadgen import build123d as bd
from cadgen import step

from airframe_geometry import PARAMETERS


@dataclass(frozen=True)
class PropellerEnvelopeParameters:
    diameter: float = 76.2
    thickness: float = 1.0


ENVELOPE_PARAMETERS = PropellerEnvelopeParameters()


def build_propeller_envelopes(
    airframe=PARAMETERS,
    envelope=ENVELOPE_PARAMETERS,
):
    """Four provisional 3-inch swept-disk envelopes, normal to the X axes."""
    airframe.validate_motor_prop_interface()
    plane_x = airframe.prop_hub_center_x
    stations = (
        (plane_x, -airframe.horizontal_motor_offset, 0),
        (plane_x, airframe.horizontal_motor_offset, 0),
        (plane_x, 0, -airframe.vertical_motor_offset),
        (plane_x, 0, airframe.vertical_motor_offset),
    )
    disks = []
    for index, (x, y, z) in enumerate(stations, start=1):
        disk = bd.Cylinder(
            radius=envelope.diameter / 2,
            height=envelope.thickness,
        )
        disk = bd.Pos(x, y, z) * bd.Rot(0, 90, 0) * disk
        disk.label = f"propeller_clearance_disk_{index}"
        disks.append(disk)
    assembly = bd.Compound(children=disks)
    assembly.label = "four_propeller_clearance_envelopes"
    return assembly


@step
def propeller_envelope():
    return build_propeller_envelopes()


if __name__ == "__main__":
    propeller_envelope()
