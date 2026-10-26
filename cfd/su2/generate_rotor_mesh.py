"""Build a five-zone SU2 mesh with four blade-resolved sliding rotors."""
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


REFERENCE_DIR = ROOT / "references" / "propeller-kit" / "STEP"
AIRFRAME_STEP = ROOT / "airframe_oml.step"
MOTOR_STEP = REFERENCE_DIR / "iFlight_XING2_1404_4600KV_REFERENCE.step"
PROP_STEPS = {
    "CW": REFERENCE_DIR / "PROP_3x4_2B_CW.step",
    "CCW": REFERENCE_DIR / "PROP_3x4_2B_CCW.step",
}
DEFAULT_OUTPUT = ROOT / "tmp" / "su2-rotor" / "rotor_domain.su2"
MM_TO_M = 0.001
BOUNDARY_TOLERANCE_M = 1e-5
ROTOR_CYLINDER_RADIAL_CLEARANCE_M = 0.003
ROTOR_CYLINDER_AXIAL_CLEARANCE_M = 0.002
MOTOR_BODY_RADIUS_M = PARAMETERS.motor_body_diameter * MM_TO_M / 2


@dataclass(frozen=True)
class Rotor:
    index: int
    y_m: float
    z_m: float
    prop_hand: str
    spin_sign: int

    @property
    def interface_static(self) -> str:
        return f"STATIC_IF_{self.index}"

    @property
    def interface_rotor(self) -> str:
        return f"ROTOR_IF_{self.index}"

    @property
    def rotating_wall(self) -> str:
        return f"ROTOR_WALL_{self.index}"


def _rotors(params: AirframeParameters = PARAMETERS) -> tuple[Rotor, ...]:
    stations = params.propeller_mount_stations
    # Diagonal pairs counter-rotate. The +Y installation preserves the CW
    # orientation used by motor_prop_installation.py.
    setup = (("CCW", -1), ("CW", 1), ("CCW", -1), ("CW", 1))
    return tuple(
        Rotor(index, y * MM_TO_M, z * MM_TO_M, hand, sign)
        for index, ((y, z), (hand, sign)) in enumerate(zip(stations, setup), start=1)
    )


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


def _close(left: float, right: float, tolerance: float = BOUNDARY_TOLERANCE_M) -> bool:
    return abs(left - right) <= tolerance


def _same_box(
    actual: tuple[float, float, float, float, float, float],
    expected: tuple[float, float, float, float, float, float],
) -> bool:
    return all(_close(a, b) for a, b in zip(actual, expected))


def _largest_volume(volumes: list[tuple[int, int]], description: str) -> tuple[int, int]:
    if not volumes:
        raise ValueError(f"No 3D volume was created for {description}")
    ranked = sorted(
        ((gmsh.model.occ.getMass(dim, tag), (dim, tag)) for dim, tag in volumes),
        reverse=True,
    )
    largest_mass, largest = ranked[0]
    if largest_mass <= 0:
        raise ValueError(f"The main fluid volume for {description} has nonpositive volume")
    if len(ranked) > 1:
        print(
            f"{description}: selecting main connected fluid region ({largest_mass:.6g} m^3) "
            f"from {len(ranked)} boolean output volumes."
        )
    return largest


def _discard_nonfluid_volumes(
    volumes: list[tuple[int, int]], main_volume: tuple[int, int], description: str
) -> None:
    extras = [volume for volume in volumes if volume != main_volume]
    if extras:
        print(f"{description}: excluding {len(extras)} non-fluid boolean solids from volume meshing.")
        # Keep boundary faces because OCC can share them with the selected fluid shell.
        gmsh.model.occ.remove(extras, recursive=False)
        gmsh.model.occ.synchronize()


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


def _motor_body(rotor: Rotor, params: AirframeParameters = PARAMETERS) -> list[tuple[int, int]]:
    """Build the parametric axisymmetric motor can inside a rotating zone."""
    tag = gmsh.model.occ.addCylinder(
        params.motor_mount_face_x * MM_TO_M,
        rotor.y_m,
        rotor.z_m,
        params.motor_body_length * MM_TO_M,
        0,
        0,
        params.motor_body_diameter * MM_TO_M / 2,
    )
    return [(3, tag)]


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
    return motor


def _place_propeller(
    rotor: Rotor, params: AirframeParameters = PARAMETERS
) -> list[tuple[int, int]]:
    prop = _import_scaled_step(PROP_STEPS[rotor.prop_hand])
    gmsh.model.occ.rotate(prop, 0, 0, 0, 0, 1, 0, 0.5 * 3.141592653589793)
    gmsh.model.occ.translate(
        prop,
        params.prop_hub_center_x * MM_TO_M,
        rotor.y_m,
        rotor.z_m,
    )
    return prop


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
        if marker.startswith(("AIRFRAME", "ROTOR_WALL", "STATIC_IF", "ROTOR_IF"))
        for surface in surfaces
    ]
    prop_or_motor_surfaces = [
        surface
        for marker, surfaces in marker_surfaces
        if marker.startswith("ROTOR_WALL")
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
    airframe = _import_scaled_step(airframe_step)
    airframe = [
        entity
        for entity in gmsh.model.occ.healShapes(airframe, tolerance=1e-6)
        if entity[0] == 3
    ]
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
    motors = []
    for rotor in rotors:
        motors.extend(_place_motor_reference(rotor, params))
    cutters = [*airframe, *cylinders, *motors]
    fluid, _ = gmsh.model.occ.cut([farfield], cutters, removeObject=True, removeTool=True)
    gmsh.model.occ.removeAllDuplicates()
    gmsh.model.occ.synchronize()
    volumes = [(dim, tag) for dim, tag in fluid if dim == 3]
    main_volume = _largest_volume(volumes, "stationary zone")
    _discard_nonfluid_volumes(volumes, main_volume, "stationary zone")
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
    gmsh.model.add(f"NeedleTail_rotor_{rotor.index}")
    x0, x1, radius = rotor_bounds
    cylinder = (3, gmsh.model.occ.addCylinder(
        x0, rotor.y_m, rotor.z_m, x1 - x0, 0, 0, radius
    ))
    gmsh.model.occ.synchronize()
    motor = _motor_body(rotor, params)
    prop = _place_propeller(rotor, params)
    obstacle, _ = gmsh.model.occ.fuse(
        motor, prop, removeObject=True, removeTool=True
    )
    gmsh.model.occ.synchronize()
    fluid, _ = gmsh.model.occ.cut(
        [cylinder], obstacle, removeObject=True, removeTool=True
    )
    gmsh.model.occ.synchronize()
    volumes = [(dim, tag) for dim, tag in fluid if dim == 3]
    main_volume = _largest_volume(volumes, f"rotor {rotor.index}")
    _discard_nonfluid_volumes(volumes, main_volume, f"rotor {rotor.index}")
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
    markers = [(rotor.rotating_wall, walls), (rotor.interface_rotor, interfaces)]
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


def validate_multizone_mesh(mesh_path: Path) -> list[dict[str, int]]:
    """Validate five-zone SU2 structure, tetrahedra, points, and required markers."""
    lines = mesh_path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "NZONE= 5":
        raise ValueError("Expected a five-zone SU2 mesh")
    cursor = 1
    summaries: list[dict[str, int]] = []
    for zone in range(1, 6):
        if cursor >= len(lines) or lines[cursor].strip() != f"IZONE= {zone}":
            raise ValueError(f"Expected IZONE={zone} in five-zone mesh")
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
            if len(values) != 5 or values[0] != 10:
                raise ValueError(f"Zone {zone} must contain only linear tetrahedra")
            tetra = tuple(values[1:])
            if len(set(tetra)) != 4:
                raise ValueError(f"Zone {zone} has a degenerate tetrahedron connectivity")
            tetrahedra.append(tetra)
        volume_faces: Counter[tuple[int, int, int]] = Counter()
        for a, b, c, d in tetrahedra:
            for face in ((a, b, c), (a, b, d), (a, c, d), (b, c, d)):
                volume_faces[tuple(sorted(face))] += 1
        if any(count > 2 for count in volume_faces.values()):
            raise ValueError(f"Zone {zone} contains non-manifold tetrahedron faces")
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
        for tetra in tetrahedra:
            if min(tetra) < 0 or max(tetra) >= point_count:
                raise ValueError(f"Zone {zone} tetrahedron references a missing point")
            a, b, c, d = (coordinates[index] for index in tetra)
            ab = tuple(b[i] - a[i] for i in range(3))
            ac = tuple(c[i] - a[i] for i in range(3))
            ad = tuple(d[i] - a[i] for i in range(3))
            determinant = (
                ab[0] * (ac[1] * ad[2] - ac[2] * ad[1])
                - ab[1] * (ac[0] * ad[2] - ac[2] * ad[0])
                + ab[2] * (ac[0] * ad[1] - ac[1] * ad[0])
            )
            if not math.isfinite(determinant) or abs(determinant) <= 1e-20:
                raise ValueError(f"Zone {zone} contains a zero-volume tetrahedron")
        if cursor >= len(lines) or not lines[cursor].startswith("NMARK="):
            raise ValueError(f"Zone {zone} is missing NMARK")
        marker_count = int(lines[cursor].split("=", 1)[1])
        cursor += 1
        markers: dict[str, int] = {}
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
            for row in lines[cursor:cursor + count]:
                values = [int(value) for value in row.split()]
                if len(values) != 4 or values[0] != expected_type:
                    raise ValueError(f"Zone {zone} marker {name} must contain triangles")
                if min(values[1:]) < 0 or max(values[1:]) >= point_count:
                    raise ValueError(f"Zone {zone} marker {name} references a missing point")
                face = tuple(sorted(values[1:]))
                if face not in boundary_faces:
                    raise ValueError(f"Zone {zone} marker {name} includes a non-boundary face")
                if face in marked_boundary_faces:
                    raise ValueError(f"Zone {zone} repeats an exterior face across markers")
                marked_boundary_faces.add(face)
            cursor += count
            if name in markers:
                raise ValueError(f"Zone {zone} repeats marker {name}")
            markers[name] = count
        required = ({"AIRFRAME_MOTOR", "FARFIELD", *(f"STATIC_IF_{i}" for i in range(1, 5))}
                    if zone == 1 else {f"ROTOR_WALL_{zone - 1}", f"ROTOR_IF_{zone - 1}"})
        missing = required - markers.keys()
        if missing:
            raise ValueError(f"Zone {zone} is missing required markers: {sorted(missing)}")
        uncovered_faces = boundary_faces - marked_boundary_faces
        if uncovered_faces:
            raise ValueError(
                f"Zone {zone} has {len(uncovered_faces)} exterior tetrahedron faces without markers"
            )
        summaries.append({"zone": zone, "nodes": point_count, "tetrahedra": element_count})
    if cursor != len(lines):
        raise ValueError("Unexpected trailing data after five-zone mesh")
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
) -> tuple[int, int]:
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
    temporary_dir.mkdir(parents=True, exist_ok=True)
    rotors = _rotors(params)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.option.setNumber("Geometry.OCCBooleanPreserveNumbering", 0)

        # Probe each actual blade STEP before sizing the nested rotating zones.
        gmsh.model.add("rotor_geometry_probe")
        cylinders, rotor_bounds = _create_rotor_cylinders(rotors, params)
        gmsh.model.occ.remove(cylinders, recursive=True)
        gmsh.clear()

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
        validate_multizone_mesh(output_path)
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
    args = parser.parse_args()
    params = airframe_parameters_for_geometry(args.geometry)
    default_output = (DEFAULT_OUTPUT if args.geometry == "pr3_baseline" else
                      ROOT / "tmp" / f"su2-rotor-{args.geometry}" / "rotor_domain.su2")
    output = (args.output or default_output).resolve()
    airframe_step = ROOT / ("airframe_oml.step" if args.geometry == "pr3_baseline"
                            else "airframe_candidate_oml.step")
    generate_mesh(output, args.domain_padding_m, args.static_near_size_m,
                  args.static_far_size_m, args.prop_near_size_m, args.interface_size_m,
                  params, airframe_step)


if __name__ == "__main__":
    main()
