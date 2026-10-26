from __future__ import annotations

from cadgen import build123d as bd
from cadgen import step

from airframe_geometry import PARAMETERS


def build_propeller_envelopes(
    airframe=PARAMETERS,
):
    """Four provisional 3-inch swept-disk envelopes, normal to the X axes."""
    airframe.validate_motor_prop_interface()
    airframe.validate_arm_pod_fit()
    airframe.validate_propeller_clearance()
    plane_x = airframe.prop_hub_center_x
    stations = tuple(
        (plane_x, y, z) for y, z in airframe.propeller_mount_stations
    )
    disks = []
    for index, (x, y, z) in enumerate(stations, start=1):
        disk = bd.Cylinder(
            radius=airframe.propeller_diameter / 2,
            height=airframe.propeller_envelope_thickness,
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
