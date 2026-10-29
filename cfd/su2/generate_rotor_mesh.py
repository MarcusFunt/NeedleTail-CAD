"""Build an SU2 multizone mesh with topology-specific blade-resolved rotors."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

try:
    import gmsh
except ImportError:  # Keep test discovery usable without the optional CFD extra.
    gmsh = None

from airframe_geometry import PARAMETERS, AirframeParameters, airframe_parameters_for_geometry
from design_configuration import Design, rotor_stations


REFERENCE_DIR = ROOT / "references" / "propeller-kit" / "STEP"
AIRFRAME_STEP = ROOT / "airframe_oml.step"
MOTOR_STEP = REFERENCE_DIR / "iFlight_XING2_1404_4600KV_REFERENCE.step"
PROP_STEPS = {
    "CW": REFERENCE_DIR / "PROP_3x4_2B_CW.step",
    "CCW": REFERENCE_DIR / "PROP_3x4_2B_CCW.step",
}
DEFAULT_OUTPUT = ROOT / "tmp" / "su2-rotor" / "rotor_domain.su2"
MM_TO_M = 0.001
# OCC can expand clipped cap bounds by up to 35 microns. This tolerance is for
# boundary-box classification; the rotor and static zones use the same OML cut.
BOUNDARY_TOLERANCE_M = 5e-5
ROTOR_CYLINDER_RADIAL_CLEARANCE_M = 0.003
ROTOR_CYLINDER_AXIAL_CLEARANCE_M = 0.002
MIN_TETRA_QUALITY = 0.01
INTERFACE_AREA_RELATIVE_TOLERANCE = 0.02
INTERFACE_COORDINATE_TOLERANCE_M = 2e-6


@dataclass(frozen=True)
class Rotor:
    index: int
    x_m: float
    y_m: float
    z_m: float
    prop_hand: str
    spin_sign: int
    side: str
    station_name: str
    shaft_axis_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)
    thrust_direction_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)

    @property
    def interface_static(self) -> str:
        return f"STATIC_IF_{self.index}"

    @property
    def interface_rotor(self) -> str:
        return f"ROTOR_IF_{self.index}"

    @property
    def rotating_wall(self) -> str:
        # Keep the historical property name as an alias for callers that only
        # need a rotating wall marker; this marker is now propeller-only.
        return self.propeller_wall

    @property
    def propeller_wall(self) -> str:
        return f"ROTOR_PROPELLER_{self.index}"


class StationaryZoneIntersectionError(ValueError):
    """A fixed airframe or motor component enters a rigidly rotating zone."""


class RotorZoneOverlapError(ValueError):
    """Two independently moving rotor zones occupy overlapping fluid volume."""


def require_no_stationary_zone_intersections(
    intersecting_components: set[str] | list[str] | tuple[str, ...],
    rotor_index: int,
) -> None:
    components = sorted(set(intersecting_components))
    if components:
        raise StationaryZoneIntersectionError(
            f"Rotor {rotor_index} rotating zone intersects stationary surfaces: "
            + ", ".join(components)
        )


def _solid_intersection_has_measure(
    left: tuple[int, int], right: list[tuple[int, int]],
) -> bool:
    intersections, _ = gmsh.model.occ.intersect(
        [left], right, removeObject=False, removeTool=False
    )
    gmsh.model.occ.synchronize()
    tolerances = {1: 1e-9, 2: 1e-12, 3: 1e-15}
    has_measure = any(
        dim in tolerances and gmsh.model.occ.getMass(dim, tag) > tolerances[dim]
        for dim, tag in intersections
    )
    if intersections:
        gmsh.model.occ.remove(intersections, recursive=True)
        gmsh.model.occ.synchronize()
    return has_measure


def _check_stationary_geometry_clearance(
    rotors: tuple[Rotor, ...],
    rotor_cylinders: list[tuple[int, int]],
    params: AirframeParameters,
    airframe_step: Path,
) -> None:
    airframe = _load_airframe_oml(airframe_step)
    for rotor, cylinder in zip(rotors, rotor_cylinders):
        motor = _place_motor_reference(rotor, params)
        intersecting: list[str] = []
        if _solid_intersection_has_measure(cylinder, airframe):
            intersecting.append("airframe OML")
        if _solid_intersection_has_measure(cylinder, motor):
            intersecting.append("motor case")
        require_no_stationary_zone_intersections(intersecting, rotor.index)
        gmsh.model.occ.remove(motor, recursive=True)
        gmsh.model.occ.remove([cylinder], recursive=True)
        gmsh.model.occ.synchronize()


def require_nonoverlapping_rotor_zones(
    rotors: tuple[Rotor, ...],
    rotor_bounds: dict[int, tuple[float, float, float]],
    tolerance_m: float = 1e-12,
) -> None:
    """Reject intersections between the current parallel-X rotor cylinders."""
    for left_index, left in enumerate(rotors):
        left_x0, left_x1, left_radius = rotor_bounds[left.index]
        for right in rotors[left_index + 1:]:
            right_x0, right_x1, right_radius = rotor_bounds[right.index]
            axial_overlap = min(left_x1, right_x1) - max(left_x0, right_x0)
            radial_distance = math.hypot(left.y_m - right.y_m, left.z_m - right.z_m)
            if (axial_overlap > tolerance_m
                    and radial_distance < left_radius + right_radius - tolerance_m):
                raise RotorZoneOverlapError(
                    f"Rotor zones {left.index} and {right.index} overlap: "
                    f"axial overlap={axial_overlap:.9g} m, "
                    f"radial center distance={radial_distance:.9g} m, "
                    f"radius sum={left_radius + right_radius:.9g} m"
                )


def _rotors(
    params: AirframeParameters = PARAMETERS,
    topology: str = "tractor4",
    coaxial_separation_mm: float | None = None,
) -> tuple[Rotor, ...]:
    design = Design(
        topology=topology,
        flaps="none",
        coaxial_separation_mm=coaxial_separation_mm,
    )
    stations = rotor_stations(params, design)
    result = []
    for index, station in enumerate(stations, start=1):
        result.append(Rotor(
            index,
            station.xyz_mm[0] * MM_TO_M,
            station.xyz_mm[1] * MM_TO_M,
            station.xyz_mm[2] * MM_TO_M,
            station.propeller_hand,
            station.spin_sign,
            station.side,
            station.name,
            station.shaft_axis_xyz,
            station.thrust_direction_xyz,
        ))
    return tuple(result)


def _bounds(dim: int, tag: int) -> tuple[float, float, float, float, float, float]:
    return tuple(gmsh.model.getBoundingBox(dim, tag))


def _bounds_for(entities: list[tuple[int, int]]) -> tuple[float, float, float, float, float, float]:
    boxes = [_bounds(dim, tag) for dim, tag in entities]
    if not boxes:
        raise ValueError("Cannot compute bounds for an empty entity set")
    return (
        min(box[0] for box in boxes),
        min(box[1] for box in boxes),
        min(box[2] for box in boxes),
        max(box[3] for box in boxes),
        max(box[4] for box in boxes),
        max(box[5] for box in boxes),
    )


def _close(left: float, right: float, tolerance: float | None = None) -> bool:
    if tolerance is None:
        tolerance = BOUNDARY_TOLERANCE_M
    return abs(left - right) <= tolerance


def _same_box(
    actual: tuple[float, float, float, float, float, float],
    expected: tuple[float, float, float, float, float, float],
) -> bool:
    return all(_close(a, b) for a, b in zip(actual, expected))


def _single_fluid_volume(volumes: list[tuple[int, int]], description: str) -> tuple[int, int]:
    if not volumes:
        raise ValueError(f"No 3D volume was created for {description}")
    if len(volumes) != 1:
        raise ValueError(
            f"{description} produced {len(volumes)} disconnected fluid volumes; "
            "refusing to discard any component"
        )
    mass = gmsh.model.occ.getMass(*volumes[0])
    if mass <= 0:
        raise ValueError(f"The main fluid volume for {description} has nonpositive volume")
    return volumes[0]


def _import_scaled_step(path: Path, heal: bool = False) -> list[tuple[int, int]]:
    if not path.is_file():
        raise FileNotFoundError(f"Reference STEP not found: {path}")
    entities = gmsh.model.occ.importShapes(str(path))
    if heal:
        entities = gmsh.model.occ.healShapes(
            entities,
            tolerance=1e-4,
            fixDegenerated=True,
            fixSmallEdges=True,
            fixSmallFaces=True,
            sewFaces=True,
            makeSolids=True,
        )
        entities = [entity for entity in entities if entity[0] == 3]
    volumes = [(dim, tag) for dim, tag in entities if dim == 3]
    if not volumes:
        raise ValueError(f"STEP has no closed solids: {path}")
    gmsh.model.occ.dilate(entities, 0, 0, 0, MM_TO_M, MM_TO_M, MM_TO_M)
    gmsh.model.occ.synchronize()
    return volumes


def _place_motor_reference(
    rotor: Rotor, params: AirframeParameters = PARAMETERS
) -> list[tuple[int, int]]:
    """Load the detailed motor only for the stationary airframe wall mesh."""
    motor = _import_scaled_step(MOTOR_STEP)
    gmsh.model.occ.rotate(motor, 0, 0, 0, 0, 1, 0, 0.5 * 3.141592653589793)
    gmsh.model.occ.translate(
        motor,
        params.motor_mount_face_x * MM_TO_M,
        rotor.y_m,
        rotor.z_m,
    )
    if rotor.side == "rear":
        center_x = params.motor_pod_center_x * MM_TO_M
        gmsh.model.occ.mirror(motor, 1, 0, 0, -center_x)
    return motor


def _place_propeller(
    rotor: Rotor, params: AirframeParameters = PARAMETERS
) -> list[tuple[int, int]]:
    prop = _import_scaled_step(PROP_STEPS[rotor.prop_hand])
    gmsh.model.occ.rotate(prop, 0, 0, 0, 0, 1, 0, 0.5 * 3.141592653589793)
    gmsh.model.occ.translate(
        prop,
        rotor.x_m,
        rotor.y_m,
        rotor.z_m,
    )
    return prop


def _load_airframe_oml(airframe_step: Path) -> list[tuple[int, int]]:
    airframe = _import_scaled_step(airframe_step)
    return [
        entity
        for entity in gmsh.model.occ.healShapes(airframe, tolerance=1e-6)
        if entity[0] == 3
    ]


def _rotor_extent(
    rotor: Rotor, params: AirframeParameters = PARAMETERS
) -> tuple[float, float, float]:
    """Read the imported prop bounds and return its axial and radial extents."""
    prop = _place_propeller(rotor, params)
    gmsh.model.occ.synchronize()
    x0, y0, z0, x1, y1, z1 = _bounds_for(prop)
    radial_extent = max(
        ((y - rotor.y_m) ** 2 + (z - rotor.z_m) ** 2) ** 0.5
        for y in (y0, y1)
        for z in (z0, z1)
    )
    gmsh.model.occ.remove(prop, recursive=True)
    return x0, x1, radial_extent


def _mesh_options(minimum_size: float, maximum_size: float) -> None:
    gmsh.option.setNumber("Mesh.MeshSizeMin", minimum_size)
    gmsh.option.setNumber("Mesh.MeshSizeMax", maximum_size)
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.ElementOrder", 1)
    gmsh.option.setNumber("Mesh.Binary", 0)


def _set_distance_field(
    surface_groups: list[tuple[list[int], float, float, float, float]],
    minimum_size: float,
    maximum_size: float,
) -> None:
    fields: list[int] = []
    for surfaces, size_min, size_max, distance_min, distance_max in surface_groups:
        if not surfaces:
            continue
        distance = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumbers(distance, "SurfacesList", surfaces)
        gmsh.model.mesh.field.setNumber(distance, "Sampling", 100)
        threshold = gmsh.model.mesh.field.add("Threshold")
        gmsh.model.mesh.field.setNumber(threshold, "InField", distance)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMin", size_min)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMax", size_max)
        gmsh.model.mesh.field.setNumber(threshold, "DistMin", distance_min)
        gmsh.model.mesh.field.setNumber(threshold, "DistMax", distance_max)
        fields.append(threshold)
    if len(fields) == 1:
        gmsh.model.mesh.field.setAsBackgroundMesh(fields[0])
    elif fields:
        minimum = gmsh.model.mesh.field.add("Min")
        gmsh.model.mesh.field.setNumbers(minimum, "FieldsList", fields)
        gmsh.model.mesh.field.setAsBackgroundMesh(minimum)
    gmsh.option.setNumber("Mesh.MeshSizeMin", minimum_size)
    gmsh.option.setNumber("Mesh.MeshSizeMax", maximum_size)


def _remove_duplicate_surface_triangles(surface_tags: list[int]) -> int:
    removed = 0
    for surface_tag in surface_tags:
        element_types, element_tags_by_type, node_tags_by_type = gmsh.model.mesh.getElements(
            2, surface_tag
        )
        for element_type, element_tags, node_tags in zip(
            element_types, element_tags_by_type, node_tags_by_type
        ):
            _, dimension, _, node_count, _, _ = gmsh.model.mesh.getElementProperties(
                element_type
            )
            if dimension != 2 or node_count != 3:
                continue
            seen: set[tuple[int, int, int]] = set()
            duplicate_tags: list[int] = []
            for index, element_tag in enumerate(element_tags):
                start = index * node_count
                triangle = tuple(
                    sorted(int(node) for node in node_tags[start : start + node_count])
                )
                if triangle in seen:
                    duplicate_tags.append(int(element_tag))
                else:
                    seen.add(triangle)
            if duplicate_tags:
                gmsh.model.mesh.removeElements(2, surface_tag, duplicate_tags)
                removed += len(duplicate_tags)
    return removed


def _conform_su2_boundary_markers(mesh_path: Path) -> int:
    """Keep only marker triangles matching exterior tetrahedron faces."""
    lines = mesh_path.read_text(encoding="utf-8").splitlines()
    cursor = 0
    if not lines or not lines[cursor].startswith("NDIME="):
        raise ValueError("Gmsh did not write a valid SU2 mesh header")
    if int(lines[cursor].split("=", 1)[1]) != 3:
        raise ValueError("The SU2 mesh must be three-dimensional")
    cursor += 1
    if not lines[cursor].startswith("NELEM="):
        raise ValueError("SU2 mesh is missing its volume element count")
    volume_count = int(lines[cursor].split("=", 1)[1])
    cursor += 1
    tetrahedra: list[tuple[int, int, int, int]] = []
    for _ in range(volume_count):
        row = [int(value) for value in lines[cursor].split()]
        cursor += 1
        if row[0] != 10 or len(row) < 5:
            raise ValueError("Boundary conformity check requires linear tetrahedra")
        tetrahedra.append(tuple(row[1:5]))

    if cursor >= len(lines) or not lines[cursor].startswith("NPOIN="):
        raise ValueError("SU2 mesh is missing its point count")
    point_count = int(lines[cursor].split("=", 1)[1])
    cursor += 1 + point_count
    if cursor >= len(lines) or not lines[cursor].startswith("NMARK="):
        raise ValueError("SU2 mesh is missing its boundary marker count")
    marker_header_index = cursor
    marker_count = int(lines[cursor].split("=", 1)[1])
    cursor += 1

    face_counts: Counter[tuple[int, int, int]] = Counter()
    for a, b, c, d in tetrahedra:
        for face in ((a, b, c), (a, b, d), (a, c, d), (b, c, d)):
            face_counts[tuple(sorted(face))] += 1
    if any(count > 2 for count in face_counts.values()):
        raise ValueError("Tetrahedral mesh contains non-manifold faces")
    boundary_faces = {face for face, count in face_counts.items() if count == 1}

    markers: list[tuple[str, list[str]]] = []
    seen_boundary_faces: set[tuple[int, int, int]] = set()
    removed = 0
    for _ in range(marker_count):
        marker_name = lines[cursor].split("=", 1)[1].strip()
        cursor += 1
        marker_element_count = int(lines[cursor].split("=", 1)[1])
        cursor += 1
        kept_rows: list[str] = []
        for _ in range(marker_element_count):
            row_text = lines[cursor]
            row = [int(value) for value in row_text.split()]
            cursor += 1
            if row[0] != 5 or len(row) < 4:
                raise ValueError("Boundary conformity check requires triangle markers")
            face = tuple(sorted(row[1:4]))
            if face not in boundary_faces or face in seen_boundary_faces:
                removed += 1
                continue
            seen_boundary_faces.add(face)
            kept_rows.append(row_text)
        if not kept_rows:
            raise ValueError(f"Boundary marker {marker_name} became empty")
        markers.append((marker_name, kept_rows))

    missing_faces = boundary_faces - seen_boundary_faces
    if missing_faces:
        raise ValueError(
            f"{len(missing_faces)} exterior tetrahedron faces have no SU2 boundary marker"
        )
    normalized = lines[: marker_header_index + 1]
    for marker_name, rows in markers:
        normalized.extend((f"MARKER_TAG= {marker_name}", f"MARKER_ELEMS= {len(rows)}", *rows))
    mesh_path.write_text("\n".join(normalized) + "\n", encoding="utf-8")
    return removed


def _interface_surface_sets(
    surfaces: list[int],
    rotors: tuple[Rotor, ...],
    axial_bounds: dict[int, tuple[float, float, float]],
) -> dict[int, list[int]]:
    groups = {rotor.index: [] for rotor in rotors}
    for surface in surfaces:
        box = _bounds(2, surface)
        for rotor in rotors:
            x0, x1, radius = axial_bounds[rotor.index]
            expected = (
                (x0, rotor.y_m - radius, rotor.z_m - radius,
                 x1, rotor.y_m + radius, rotor.z_m + radius),
                (x0, rotor.y_m - radius, rotor.z_m - radius,
                 x0, rotor.y_m + radius, rotor.z_m + radius),
                (x1, rotor.y_m - radius, rotor.z_m - radius,
                 x1, rotor.y_m + radius, rotor.z_m + radius),
            )
            if any(_same_box(box, candidate) for candidate in expected):
                groups[rotor.index].append(surface)
                break
    for rotor in rotors:
        if len(groups[rotor.index]) < 3:
            raise ValueError(
                f"Expected cylindrical side and two end faces for rotor {rotor.index}; "
                f"found {len(groups[rotor.index])} interface surfaces"
            )
    return groups


def _classify_outer_surfaces(
    fluid_volume: tuple[int, int],
    farfield_bounds: tuple[float, float, float, float, float, float],
    rotors: tuple[Rotor, ...],
    rotor_bounds: dict[int, tuple[float, float, float]],
) -> tuple[list[int], list[int], dict[int, list[int]]]:
    boundary = [
        tag
        for dim, tag in gmsh.model.getBoundary([fluid_volume], combined=False,
                                             oriented=False, recursive=False)
        if dim == 2
    ]
    farfield_surfaces: list[int] = []
    remaining: list[int] = []
    for surface in boundary:
        box = _bounds(2, surface)
        if any(
            _close(box[axis], farfield_bounds[axis])
            or _close(box[axis + 3], farfield_bounds[axis + 3])
            for axis in range(3)
        ):
            farfield_surfaces.append(surface)
        else:
            remaining.append(surface)
    interfaces = _interface_surface_sets(remaining, rotors, rotor_bounds)
    interface_tags = {surface for group in interfaces.values() for surface in group}
    wall_surfaces = [surface for surface in remaining if surface not in interface_tags]
    if len(farfield_surfaces) < 6 or not wall_surfaces:
        raise ValueError(
            f"Could not classify outer zone boundaries: farfield={len(farfield_surfaces)}, "
            f"wall={len(wall_surfaces)}"
        )
    return wall_surfaces, farfield_surfaces, interfaces


def _create_rotor_cylinders(
    rotors: tuple[Rotor, ...], params: AirframeParameters = PARAMETERS,
) -> tuple[list[tuple[int, int]], dict[int, tuple[float, float, float]]]:
    cylinders: list[tuple[int, int]] = []
    bounds: dict[int, tuple[float, float, float]] = {}
    for rotor in rotors:
        prop_x0, prop_x1, prop_radius = _rotor_extent(rotor, params)
        x0 = prop_x0 - ROTOR_CYLINDER_AXIAL_CLEARANCE_M
        x1 = prop_x1 + ROTOR_CYLINDER_AXIAL_CLEARANCE_M
        radius = prop_radius + ROTOR_CYLINDER_RADIAL_CLEARANCE_M
        cylinder = gmsh.model.occ.addCylinder(
            x0, rotor.y_m, rotor.z_m, x1 - x0, 0, 0, radius
        )
        cylinders.append((3, cylinder))
        bounds[rotor.index] = (x0, x1, radius)
        print(
            f"Rotor {rotor.index} ({rotor.prop_hand}, spin {rotor.spin_sign:+d}): "
            f"zone X={x0:.5f}..{x1:.5f} m, radius={radius:.5f} m"
        )
    return cylinders, bounds


def _probe_stationary_zone_clearance(
    rotors: tuple[Rotor, ...], params: AirframeParameters, airframe_step: Path,
) -> tuple[dict[int, tuple[float, float, float]], dict[str, Any]]:
    """Run only the OCC geometry checks; do not create a volume mesh."""
    gmsh.model.add("rotor_geometry_probe")
    cylinders, rotor_bounds = _create_rotor_cylinders(rotors, params)
    require_nonoverlapping_rotor_zones(rotors, rotor_bounds)
    _check_stationary_geometry_clearance(rotors, cylinders, params, airframe_step)
    gmsh.clear()
    return rotor_bounds, {
        "passed": True,
        "rotor_count": len(rotors),
        "rotating_zones_disjoint": True,
        "stationary_components_clear": True,
    }


def validate_rotor_zone_geometry(
    params: AirframeParameters,
    airframe_step: Path,
    topology: str = "tractor4",
    coaxial_separation_mm: float | None = None,
) -> dict[str, Any]:
    """Check zone overlap and fixed-component intersections without meshing/SU2."""
    if gmsh is None:
        raise RuntimeError("Gmsh is required for the geometry-only rotor-zone diagnostic")
    rotors = _rotors(params, topology, coaxial_separation_mm)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.option.setNumber("Geometry.OCCBooleanPreserveNumbering", 0)
        _, result = _probe_stationary_zone_clearance(rotors, params, airframe_step)
        return result
    finally:
        gmsh.finalize()


def _mesh_one_zone(
    name: str,
    fluid_volume: tuple[int, int],
    marker_surfaces: list[tuple[str, list[int]]],
    output_path: Path,
    sizing: tuple[float, float, float],
    farfield_surfaces: list[int] | None = None,
    interface_groups: list[tuple[list[int], float]] | None = None,
) -> tuple[int, int, int]:
    wall_surfaces = [
        surface
        for marker, surfaces in marker_surfaces
        if marker.startswith(("AIRFRAME", "ROTOR_PROPELLER", "STATIC_IF", "ROTOR_IF"))
        for surface in surfaces
    ]
    prop_or_motor_surfaces = [
        surface
        for marker, surfaces in marker_surfaces
        if marker.startswith("ROTOR_PROPELLER")
        for surface in surfaces
    ]
    near_size, core_size, far_size = sizing
    _mesh_options(near_size, far_size)

    for marker, surfaces in marker_surfaces:
        if not surfaces:
            raise ValueError(f"Zone {name} has empty marker {marker}")
        group = gmsh.model.addPhysicalGroup(2, surfaces)
        gmsh.model.setPhysicalName(2, group, marker)
    if farfield_surfaces is not None:
        farfield_group = gmsh.model.addPhysicalGroup(2, farfield_surfaces)
        gmsh.model.setPhysicalName(2, farfield_group, "FARFIELD")
    volume_group = gmsh.model.addPhysicalGroup(3, [fluid_volume[1]])
    gmsh.model.setPhysicalName(3, volume_group, f"FLUID_{name}")

    field_groups: list[tuple[list[int], float, float, float, float]] = []
    if farfield_surfaces is not None:
        field_groups.append((wall_surfaces, near_size, far_size, 0.01, 0.30))
    else:
        field_groups.append((prop_or_motor_surfaces, near_size, core_size, 0.001, 0.020))
        if interface_groups:
            for surfaces, interface_size in interface_groups:
                field_groups.append((surfaces, interface_size, core_size, 0.003, 0.025))
    _set_distance_field(field_groups, near_size, far_size)

    all_surfaces = [surface for _, surfaces in marker_surfaces for surface in surfaces]
    if farfield_surfaces:
        all_surfaces.extend(farfield_surfaces)
    gmsh.model.mesh.generate(2)
    duplicate_triangles = _remove_duplicate_surface_triangles(all_surfaces)
    if duplicate_triangles:
        print(f"Zone {name}: removed {duplicate_triangles:,} duplicate surface triangles")
    gmsh.model.mesh.generate(3)
    node_tags, _, _ = gmsh.model.mesh.getNodes()
    element_types, element_tags_by_type, _ = gmsh.model.mesh.getElements(3, fluid_volume[1])
    volume_elements = sum(len(tags) for tags in element_tags_by_type)
    gmsh.option.setNumber("Mesh.Format", 42)
    gmsh.write(str(output_path))
    filtered = _conform_su2_boundary_markers(output_path)
    print(
        f"Zone {name}: {len(node_tags):,} nodes, {volume_elements:,} tetrahedra, "
        f"filtered {filtered:,} nonconforming marker faces"
    )
    return len(node_tags), volume_elements, filtered


def _build_static_zone(
    output_path: Path,
    rotors: tuple[Rotor, ...],
    rotor_bounds: dict[int, tuple[float, float, float]],
    padding_m: float,
    mesh_sizes: tuple[float, float],
    params: AirframeParameters = PARAMETERS,
    airframe_step: Path = AIRFRAME_STEP,
) -> tuple[int, int, int]:
    gmsh.model.add("NeedleTail_stationary_external_flow")
    airframe = _load_airframe_oml(airframe_step)
    body_bounds = _bounds_for(airframe)
    cylinders = [
        (
            3,
            gmsh.model.occ.addCylinder(
                rotor_bounds[rotor.index][0],
                rotor.y_m,
                rotor.z_m,
                rotor_bounds[rotor.index][1] - rotor_bounds[rotor.index][0],
                0,
                0,
                rotor_bounds[rotor.index][2],
            ),
        )
        for rotor in rotors
    ]
    x_min, y_min, z_min, x_max, y_max, z_max = (
        body_bounds[0] - padding_m,
        body_bounds[1] - padding_m,
        body_bounds[2] - padding_m,
        body_bounds[3] + padding_m,
        body_bounds[4] + padding_m,
        body_bounds[5] + padding_m,
    )
    farfield = (3, gmsh.model.occ.addBox(
        x_min, y_min, z_min, x_max - x_min, y_max - y_min, z_max - z_min
    ))
    # Each motor intersects its rotor cylinder. Fuse those overlapping tools
    # first so OCC does not have to resolve many redundant motor/cylinder
    # intersections in one large farfield cut (especially for coaxial8).
    rotor_tools_by_side: dict[str, list[tuple[int, int]]] = {
        "front": [], "rear": []
    }
    for rotor, cylinder in zip(rotors, cylinders):
        motor = _place_motor_reference(rotor, params)
        fused, _ = gmsh.model.occ.fuse(
            [cylinder], motor, removeObject=True, removeTool=True
        )
        rotor_tools_by_side[rotor.side].extend(
            entity for entity in fused if entity[0] == 3
        )
    gmsh.model.occ.synchronize()
    # Keep the airframe boolean separate from the rotor/motor cuts. A single
    # 17-tool OCC subtraction becomes extremely slow for coaxial8 because the
    # rotor tools intersect the imported airframe at many locations.
    fluid, _ = gmsh.model.occ.cut(
        [farfield], airframe, removeObject=True, removeTool=True
    )
    gmsh.model.occ.synchronize()
    # The front and rear stacks are separated along X; cutting each stack
    # independently avoids one large eight-tool intersection graph.
    for side in ("front", "rear"):
        tools = rotor_tools_by_side[side]
        if tools:
            fluid, _ = gmsh.model.occ.cut(
                fluid, tools, removeObject=True, removeTool=True
            )
            gmsh.model.occ.synchronize()
    gmsh.model.occ.removeAllDuplicates()
    gmsh.model.occ.synchronize()
    volumes = [(dim, tag) for dim, tag in fluid if dim == 3]
    main_volume = _single_fluid_volume(volumes, "stationary zone")
    walls, farfield_surfaces, interfaces = _classify_outer_surfaces(
        main_volume, (x_min, y_min, z_min, x_max, y_max, z_max), rotors, rotor_bounds
    )
    markers = [("AIRFRAME_MOTOR", walls)] + [
        (rotor.interface_static, interfaces[rotor.index]) for rotor in rotors
    ]
    return _mesh_one_zone(
        "static",
        main_volume,
        markers,
        output_path,
        (mesh_sizes[0], 0.03, mesh_sizes[1]),
        farfield_surfaces,
    )


def _build_rotor_zone(
    output_path: Path,
    rotor: Rotor,
    rotor_bounds: tuple[float, float, float],
    prop_size_m: float,
    interface_size_m: float,
    params: AirframeParameters = PARAMETERS,
) -> tuple[int, int, int]:
    """Build a rotor-only fluid zone after stationary-component clearance passes."""
    gmsh.model.add(f"NeedleTail_rotor_{rotor.index}")
    x0, x1, radius = rotor_bounds
    cylinder = (3, gmsh.model.occ.addCylinder(
        x0, rotor.y_m, rotor.z_m, x1 - x0, 0, 0, radius
    ))
    gmsh.model.occ.synchronize()
    prop = _place_propeller(rotor, params)
    fluid, _ = gmsh.model.occ.cut(
        [cylinder], prop, removeObject=True, removeTool=True
    )
    gmsh.model.occ.synchronize()
    volumes = [(dim, tag) for dim, tag in fluid if dim == 3]
    main_volume = _single_fluid_volume(volumes, f"rotor {rotor.index}")
    boundary_surfaces = [
        tag
        for dim, tag in gmsh.model.getBoundary([main_volume], combined=False,
                                             oriented=False, recursive=False)
        if dim == 2
    ]
    interfaces = _interface_surface_sets(
        boundary_surfaces, (rotor,), {rotor.index: rotor_bounds}
    )[rotor.index]
    walls = [surface for surface in boundary_surfaces if surface not in set(interfaces)]
    if not walls:
        raise ValueError(f"Rotor {rotor.index} has no propeller wall surfaces")
    markers = [(rotor.propeller_wall, walls), (rotor.interface_rotor, interfaces)]
    return _mesh_one_zone(
        f"rotor_{rotor.index}",
        main_volume,
        markers,
        output_path,
        (prop_size_m, 0.006, 0.012),
        interface_groups=[(interfaces, interface_size_m)],
    )


def _combine_zones(zone_paths: list[Path], output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8", newline="\n") as combined:
        combined.write(f"NZONE= {len(zone_paths)}\n")
        for zone_index, zone_path in enumerate(zone_paths, start=1):
            lines = zone_path.read_text(encoding="utf-8").splitlines()
            if not lines or not lines[0].startswith("NDIME="):
                raise ValueError(f"Invalid SU2 zone mesh: {zone_path}")
            combined.write(f"IZONE= {zone_index}\n")
            combined.write("\n".join(lines))
            combined.write("\n")


def _tetra_quality(points: tuple[tuple[float, float, float], ...]) -> float:
    """Return normalized mean-ratio quality (1 for regular, 0 when degenerate)."""
    edge_squared_sum = sum(
        sum((points[i][axis] - points[j][axis]) ** 2 for axis in range(3))
        for i, j in ((0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3))
    )
    a, b, c, d = points
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    ad = tuple(d[i] - a[i] for i in range(3))
    determinant = (
        ab[0] * (ac[1] * ad[2] - ac[2] * ad[1])
        - ab[1] * (ac[0] * ad[2] - ac[2] * ad[0])
        + ab[2] * (ac[0] * ad[1] - ac[1] * ad[0])
    )
    if not math.isfinite(determinant):
        raise ValueError("Mesh contains a nonfinite signed tetrahedron volume")
    if determinant < 0:
        raise ValueError("Mesh contains an inverted tetrahedron (negative signed volume)")
    if determinant <= 1e-20 or edge_squared_sum <= 0:
        raise ValueError("Mesh contains a zero-volume or degenerate tetrahedron")
    quality = 12 * (determinant / 2) ** (2 / 3) / edge_squared_sum
    if not math.isfinite(quality):
        raise ValueError("Mesh contains a nonfinite tetrahedron quality")
    if quality < MIN_TETRA_QUALITY:
        raise ValueError(
            f"Mesh contains a poor-quality tetrahedron: normalized quality "
            f"{quality:.6g} < {MIN_TETRA_QUALITY:g}"
        )
    return quality


def _triangle_area(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> float:
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return 0.5 * math.sqrt(sum(value * value for value in cross))


def _validate_interface_area_coverage(
    summaries: list[dict[str, Any]], rotor_count: int,
) -> None:
    static_areas = summaries[0]["marker_areas_m2"]
    for rotor_index in range(1, rotor_count + 1):
        static_area = static_areas.get(f"STATIC_IF_{rotor_index}", 0.0)
        rotor_area = summaries[rotor_index]["marker_areas_m2"].get(
            f"ROTOR_IF_{rotor_index}", 0.0
        )
        relative_error = abs(static_area - rotor_area) / max(static_area, rotor_area, 1e-30)
        if static_area <= 0 or rotor_area <= 0 or relative_error > INTERFACE_AREA_RELATIVE_TOLERANCE:
            raise ValueError(
                f"Rotor {rotor_index} interface area coverage differs across zones: "
                f"static={static_area:.9g} m^2, rotor={rotor_area:.9g} m^2, "
                f"relative difference={relative_error:.3%}"
            )
        static_faces = summaries[0]["marker_face_coordinates"].get(
            f"STATIC_IF_{rotor_index}", []
        )
        rotor_faces = summaries[rotor_index]["marker_face_coordinates"].get(
            f"ROTOR_IF_{rotor_index}", []
        )
        if Counter(static_faces) != Counter(rotor_faces):
            raise ValueError(
                f"Rotor {rotor_index} interface surface triangles do not match across zones"
            )


def validate_multizone_mesh(mesh_path: Path, rotor_count: int = 4) -> list[dict[str, Any]]:
    """Validate zone connectivity, signed cell quality, markers and interfaces."""
    lines = mesh_path.read_text(encoding="utf-8").splitlines()
    zone_count = rotor_count + 1
    if not lines or lines[0].strip() != f"NZONE= {zone_count}":
        raise ValueError(f"Expected a {zone_count}-zone SU2 mesh")
    cursor = 1
    summaries: list[dict[str, Any]] = []
    for zone in range(1, zone_count + 1):
        if cursor >= len(lines) or lines[cursor].strip() != f"IZONE= {zone}":
            raise ValueError(f"Expected IZONE={zone} in {zone_count}-zone mesh")
        cursor += 1
        if cursor >= len(lines) or lines[cursor].strip() != "NDIME= 3":
            raise ValueError(f"Zone {zone} must have NDIME=3")
        cursor += 1
        if cursor >= len(lines) or not lines[cursor].startswith("NELEM="):
            raise ValueError(f"Zone {zone} is missing NELEM")
        element_count = int(lines[cursor].split("=", 1)[1])
        cursor += 1
        if element_count <= 0 or cursor + element_count > len(lines):
            raise ValueError(f"Zone {zone} has an invalid element count")
        tetrahedra: list[tuple[int, int, int, int]] = []
        for row in lines[cursor:cursor + element_count]:
            values = [int(value) for value in row.split()]
            if len(values) not in (5, 6) or values[0] != 10:
                raise ValueError(f"Zone {zone} must contain only linear tetrahedra")
            # Gmsh's SU2 writer appends an element ID after the four node IDs.
            tetra = tuple(values[1:5])
            if len(set(tetra)) != 4:
                raise ValueError(f"Zone {zone} has a degenerate tetrahedron connectivity")
            tetrahedra.append(tetra)
        volume_faces: Counter[tuple[int, int, int]] = Counter()
        face_owners: dict[tuple[int, int, int], list[int]] = {}
        for element_index, (a, b, c, d) in enumerate(tetrahedra):
            for face in ((a, b, c), (a, b, d), (a, c, d), (b, c, d)):
                key = tuple(sorted(face))
                volume_faces[key] += 1
                face_owners.setdefault(key, []).append(element_index)
        if any(count > 2 for count in volume_faces.values()):
            raise ValueError(f"Zone {zone} contains non-manifold tetrahedron faces")
        adjacency = [set() for _ in tetrahedra]
        for owners in face_owners.values():
            if len(owners) == 2:
                adjacency[owners[0]].add(owners[1])
                adjacency[owners[1]].add(owners[0])
        visited: set[int] = set()
        pending = [0]
        while pending:
            current = pending.pop()
            if current in visited:
                continue
            visited.add(current)
            pending.extend(adjacency[current] - visited)
        if len(visited) != len(tetrahedra):
            raise ValueError(f"Zone {zone} contains disconnected fluid volume components")
        boundary_faces = {face for face, count in volume_faces.items() if count == 1}
        cursor += element_count
        if cursor >= len(lines) or not lines[cursor].startswith("NPOIN="):
            raise ValueError(f"Zone {zone} is missing NPOIN")
        point_count = int(lines[cursor].split("=", 1)[1])
        cursor += 1
        if point_count < 4 or cursor + point_count > len(lines):
            raise ValueError(f"Zone {zone} has an invalid point count")
        coordinates: list[tuple[float, float, float]] = []
        for row in lines[cursor:cursor + point_count]:
            values = row.split()
            if len(values) < 3:
                raise ValueError(f"Zone {zone} has an invalid point row")
            point = tuple(float(value) for value in values[:3])
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f"Zone {zone} contains nonfinite point coordinates")
            coordinates.append(point)
        cursor += point_count
        tetra_qualities: list[float] = []
        for tetra in tetrahedra:
            if min(tetra) < 0 or max(tetra) >= point_count:
                raise ValueError(f"Zone {zone} tetrahedron references a missing point")
            tetra_qualities.append(_tetra_quality(tuple(coordinates[index] for index in tetra)))
        if cursor >= len(lines) or not lines[cursor].startswith("NMARK="):
            raise ValueError(f"Zone {zone} is missing NMARK")
        marker_count = int(lines[cursor].split("=", 1)[1])
        cursor += 1
        markers: dict[str, int] = {}
        marker_areas: dict[str, float] = {}
        marker_face_coordinates: dict[str, list[tuple[tuple[int, int, int], ...]]] = {}
        marked_boundary_faces: set[tuple[int, int, int]] = set()
        for _ in range(marker_count):
            if cursor >= len(lines) or not lines[cursor].startswith("MARKER_TAG="):
                raise ValueError(f"Zone {zone} has a malformed marker tag")
            name = lines[cursor].split("=", 1)[1].strip()
            cursor += 1
            if cursor >= len(lines) or not lines[cursor].startswith("MARKER_ELEMS="):
                raise ValueError(f"Zone {zone} marker {name} has no element count")
            count = int(lines[cursor].split("=", 1)[1])
            cursor += 1
            if count <= 0 or cursor + count > len(lines):
                raise ValueError(f"Zone {zone} marker {name} is empty or truncated")
            expected_type = 5
            marker_area = 0.0
            for row in lines[cursor:cursor + count]:
                values = [int(value) for value in row.split()]
                if len(values) not in (4, 5) or values[0] != expected_type:
                    raise ValueError(f"Zone {zone} marker {name} must contain triangles")
                # Marker rows can also carry a trailing Gmsh element ID.
                nodes = values[1:4]
                if min(nodes) < 0 or max(nodes) >= point_count:
                    raise ValueError(f"Zone {zone} marker {name} references a missing point")
                face = tuple(sorted(nodes))
                if face not in boundary_faces:
                    raise ValueError(f"Zone {zone} marker {name} includes a non-boundary face")
                if face in marked_boundary_faces:
                    raise ValueError(f"Zone {zone} repeats an exterior face across markers")
                marked_boundary_faces.add(face)
                marker_area += _triangle_area(*(coordinates[node] for node in nodes))
                marker_face_coordinates.setdefault(name, []).append(tuple(sorted(
                    tuple(round(value / INTERFACE_COORDINATE_TOLERANCE_M)
                          for value in coordinates[node])
                    for node in nodes
                )))
            cursor += count
            if name in markers:
                raise ValueError(f"Zone {zone} repeats marker {name}")
            markers[name] = count
            marker_areas[name] = marker_area
        required = (
            {"AIRFRAME_MOTOR", "FARFIELD", *(f"STATIC_IF_{i}" for i in range(1, rotor_count + 1))}
            if zone == 1
            else {f"ROTOR_PROPELLER_{zone - 1}", f"ROTOR_IF_{zone - 1}"}
        )
        missing = required - markers.keys()
        if missing:
            raise ValueError(f"Zone {zone} is missing required markers: {sorted(missing)}")
        uncovered_faces = boundary_faces - marked_boundary_faces
        if uncovered_faces:
            raise ValueError(
                f"Zone {zone} has {len(uncovered_faces)} exterior tetrahedron faces without markers"
            )
        ordered_qualities = sorted(tetra_qualities)
        p01_index = max(0, math.ceil(0.01 * len(ordered_qualities)) - 1)
        surface_area_split = {
            "stationary_walls": sum(value for name, value in marker_areas.items()
                                     if name == "AIRFRAME_MOTOR"),
            "rotating_propeller_walls": sum(value for name, value in marker_areas.items()
                                             if name.startswith("ROTOR_PROPELLER_")),
            "sliding_interfaces": sum(value for name, value in marker_areas.items()
                                       if name.startswith(("STATIC_IF_", "ROTOR_IF_"))),
            "farfield": sum(value for name, value in marker_areas.items()
                            if name == "FARFIELD"),
        }
        summaries.append({
            "zone": zone,
            "nodes": point_count,
            "tetrahedra": element_count,
            "minimum_tetra_quality": min(tetra_qualities),
            "p01_tetra_quality": ordered_qualities[p01_index],
            "boundary_areas_m2": marker_areas,
            "surface_area_split_m2": surface_area_split,
            "marker_areas_m2": marker_areas,
            "marker_face_coordinates": marker_face_coordinates,
        })
    if cursor != len(lines):
        raise ValueError(f"Unexpected trailing data after {zone_count}-zone mesh")
    _validate_interface_area_coverage(summaries, rotor_count)
    for summary in summaries:
        summary.pop("marker_face_coordinates", None)
    return summaries


def generate_mesh(
    output_path: Path = DEFAULT_OUTPUT,
    domain_padding_m: float = 1.0,
    static_near_size_m: float = 0.004,
    static_far_size_m: float = 0.12,
    prop_near_size_m: float = 0.0018,
    interface_size_m: float = 0.003,
    params: AirframeParameters = PARAMETERS,
    airframe_step: Path = AIRFRAME_STEP,
    topology: str = "tractor4",
    coaxial_separation_mm: float | None = None,
) -> tuple[int, int]:
    if topology == "coaxial8" and coaxial_separation_mm is None:
        raise ValueError("coaxial8 meshing requires an explicit coaxial separation")
    if gmsh is None:
        raise RuntimeError("Gmsh is required for meshing; install the optional cfd-meshing extra")
    for path in (airframe_step, MOTOR_STEP, *PROP_STEPS.values()):
        if not path.is_file():
            raise FileNotFoundError(f"Required STEP input not found: {path}")
    if domain_padding_m <= 0 or min(static_near_size_m, prop_near_size_m, interface_size_m) <= 0:
        raise ValueError("Domain padding and mesh sizes must be positive")
    if static_far_size_m < static_near_size_m:
        raise ValueError("Farfield mesh size must not be smaller than the static near-wall size")

    temporary_dir = output_path.parent / "rotor_zones"
    if output_path.exists() or temporary_dir.exists():
        raise FileExistsError(
            f"Refusing to overwrite existing rotor mesh outputs under {output_path.parent}"
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rotors = _rotors(params, topology, coaxial_separation_mm)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.option.setNumber("Geometry.OCCBooleanPreserveNumbering", 0)

        # Probe the actual blades and reject any fixed component that would be
        # clipped into a rigidly rotating zone. This check runs before mesh files
        # are created, so invalid geometry is retryable after a CAD correction.
        rotor_bounds, _ = _probe_stationary_zone_clearance(rotors, params, airframe_step)

        temporary_dir.mkdir(parents=True, exist_ok=True)
        zone_paths = [temporary_dir / "zone_1_static.su2"] + [
            temporary_dir / f"zone_{rotor.index + 1}_rotor.su2" for rotor in rotors
        ]
        zone_stats: list[tuple[int, int, int]] = []
        zone_stats.append(
            _build_static_zone(
                zone_paths[0], rotors, rotor_bounds, domain_padding_m,
                (static_near_size_m, static_far_size_m), params, airframe_step,
            )
        )
        gmsh.clear()
        for rotor, zone_path in zip(rotors, zone_paths[1:]):
            zone_stats.append(
                _build_rotor_zone(
                    zone_path, rotor, rotor_bounds[rotor.index], prop_near_size_m,
                    interface_size_m, params,
                )
            )
            gmsh.clear()

        _combine_zones(zone_paths, output_path)
        validate_multizone_mesh(output_path, rotor_count=len(rotors))
        total_nodes = sum(nodes for nodes, _, _ in zone_stats)
        total_elements = sum(elements for _, elements, _ in zone_stats)
        total_filtered = sum(filtered for _, _, filtered in zone_stats)
        print(
            f"Wrote {output_path}: {len(zone_paths)} zones, {total_nodes:,} zone-local nodes, "
            f"{total_elements:,} tetrahedra, filtered {total_filtered:,} nonconforming faces."
        )
        return total_nodes, total_elements
    finally:
        gmsh.finalize()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--domain-padding-m", type=float, default=1.0)
    parser.add_argument("--static-near-size-m", type=float, default=0.004)
    parser.add_argument("--static-far-size-m", type=float, default=0.12)
    parser.add_argument("--prop-near-size-m", type=float, default=0.0018)
    parser.add_argument("--interface-size-m", type=float, default=0.003)
    parser.add_argument("--geometry", choices=("pr3_baseline", "airfoil_candidate"),
                        default="pr3_baseline")
    parser.add_argument("--topology", choices=("tractor4", "pusher4", "coaxial8"),
                        default="tractor4")
    parser.add_argument("--coaxial-separation-mm", type=float)
    parser.add_argument("--airframe-step", type=Path)
    args = parser.parse_args()
    if args.topology == "coaxial8" and args.coaxial_separation_mm is None:
        parser.error("--coaxial-separation-mm is required for coaxial8")
    params = airframe_parameters_for_geometry(args.geometry)
    default_output = (DEFAULT_OUTPUT if args.geometry == "pr3_baseline" else
                      ROOT / "tmp" / f"su2-rotor-{args.geometry}" / "rotor_domain.su2")
    output = (args.output or default_output).resolve()
    airframe_step = ROOT / ("airframe_oml.step" if args.geometry == "pr3_baseline"
                            else "airframe_candidate_oml.step")
    if args.airframe_step:
        airframe_step = args.airframe_step.resolve()
    generate_mesh(output, args.domain_padding_m, args.static_near_size_m,
                  args.static_far_size_m, args.prop_near_size_m, args.interface_size_m,
                  params, airframe_step, args.topology, args.coaxial_separation_mm)


if __name__ == "__main__":
    main()
