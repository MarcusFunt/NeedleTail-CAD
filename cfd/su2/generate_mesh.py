"""Create a coarse external-flow SU2 mesh around the current airframe OML."""
from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

try:
    import gmsh
except ImportError:  # Keep imports/test discovery usable without optional CFD tools.
    gmsh = None


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STEP = ROOT / "airframe_oml.step"
DEFAULT_OUTPUT = ROOT / "tmp" / "su2-15mps" / "airframe_domain.su2"
MM_TO_M = 0.001


def _bounds(dim: int, tag: int) -> tuple[float, float, float, float, float, float]:
    return tuple(gmsh.model.getBoundingBox(dim, tag))


def _remove_duplicate_surface_triangles(surface_tags: list[int]) -> int:
    """Remove exact duplicate linear triangles from imported CAD surface meshes."""
    removed = 0
    for surface_tag in surface_tags:
        element_types, element_tags_by_type, node_tags_by_type = (
            gmsh.model.mesh.getElements(2, surface_tag)
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
    """Keep only boundary triangles that exactly match an exterior tet face."""
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
        if row[0] != 10 or len(row) < 6:
            raise ValueError("Boundary conformity check currently requires linear tetrahedra")
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
    invalid_adjacency = [count for count in face_counts.values() if count > 2]
    if invalid_adjacency:
        raise ValueError("Tetrahedral mesh contains non-manifold faces")
    boundary_faces = {face for face, count in face_counts.items() if count == 1}

    markers: list[tuple[str, list[str]]] = []
    seen_boundary_faces: set[tuple[int, int, int]] = set()
    removed = 0
    for _ in range(marker_count):
        if not lines[cursor].startswith("MARKER_TAG="):
            raise ValueError("Malformed SU2 marker tag")
        marker_name = lines[cursor].split("=", 1)[1].strip()
        cursor += 1
        if not lines[cursor].startswith("MARKER_ELEMS="):
            raise ValueError(f"Marker {marker_name} is missing its element count")
        marker_element_count = int(lines[cursor].split("=", 1)[1])
        cursor += 1

        kept_rows: list[str] = []
        for _ in range(marker_element_count):
            row_text = lines[cursor]
            row = [int(value) for value in row_text.split()]
            cursor += 1
            if row[0] != 5 or len(row) < 4:
                raise ValueError("Boundary conformity check currently requires triangle markers")
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
        normalized.append(f"MARKER_TAG= {marker_name}")
        normalized.append(f"MARKER_ELEMS= {len(rows)}")
        normalized.extend(rows)
    mesh_path.write_text("\n".join(normalized) + "\n", encoding="utf-8")
    return removed


def generate_mesh(
    step_path: Path = DEFAULT_STEP,
    output_path: Path = DEFAULT_OUTPUT,
    domain_padding_m: float = 1.0,
    near_size_m: float = 0.004,
    far_size_m: float = 0.12,
) -> tuple[int, int]:
    if gmsh is None:
        raise RuntimeError("Gmsh is required for meshing; install the optional cfd-meshing extra")
    if not step_path.is_file():
        raise FileNotFoundError(f"Airframe STEP not found: {step_path}")
    if domain_padding_m <= 0 or near_size_m <= 0 or far_size_m < near_size_m:
        raise ValueError("Mesh sizes and domain padding must be positive and ordered")

    if output_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing mesh: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMin", near_size_m)
        gmsh.option.setNumber("Mesh.MeshSizeMax", far_size_m)
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
        gmsh.option.setNumber("Mesh.ElementOrder", 1)
        gmsh.option.setNumber("Mesh.Binary", 0)

        gmsh.model.add("NeedleTail_external_flow")
        imported = gmsh.model.occ.importShapes(str(step_path))
        body_volumes = [(dim, tag) for dim, tag in imported if dim == 3]
        if len(body_volumes) != 1:
            raise ValueError(
                f"Expected one watertight airframe OML solid; got {len(body_volumes)}"
            )

        # CadQuery/build123d STEP dimensions are in millimeters; SU2 expects meters.
        gmsh.model.occ.dilate(body_volumes, 0, 0, 0, MM_TO_M, MM_TO_M, MM_TO_M)
        gmsh.model.occ.synchronize()
        body_box = _bounds(*body_volumes[0])

        x_min = body_box[0] - domain_padding_m
        y_min = body_box[1] - domain_padding_m
        z_min = body_box[2] - domain_padding_m
        x_max = body_box[3] + domain_padding_m
        y_max = body_box[4] + domain_padding_m
        z_max = body_box[5] + domain_padding_m
        farfield_box = gmsh.model.occ.addBox(
            x_min,
            y_min,
            z_min,
            x_max - x_min,
            y_max - y_min,
            z_max - z_min,
        )
        fluid, _ = gmsh.model.occ.cut(
            [(3, farfield_box)], body_volumes, removeObject=True, removeTool=True
        )
        gmsh.model.occ.synchronize()
        fluid_volumes = [(dim, tag) for dim, tag in fluid if dim == 3]
        if len(fluid_volumes) != 1:
            raise ValueError(f"Expected one external fluid volume; got {len(fluid_volumes)}")

        wall_surfaces: list[int] = []
        farfield_surfaces: list[int] = []
        for dim, tag in gmsh.model.getBoundary(
            fluid_volumes, combined=False, oriented=False, recursive=False
        ):
            if dim != 2:
                continue
            box = _bounds(dim, tag)
            on_outer_box = (
                abs(box[0] - x_min) < 1e-6
                or abs(box[3] - x_max) < 1e-6
                or abs(box[1] - y_min) < 1e-6
                or abs(box[4] - y_max) < 1e-6
                or abs(box[2] - z_min) < 1e-6
                or abs(box[5] - z_max) < 1e-6
            )
            (farfield_surfaces if on_outer_box else wall_surfaces).append(tag)

        if not wall_surfaces or len(farfield_surfaces) != 6:
            raise ValueError(
                "Could not identify airframe and six outer-box boundary surfaces "
                f"(airframe={len(wall_surfaces)}, farfield={len(farfield_surfaces)})"
            )

        airframe_group = gmsh.model.addPhysicalGroup(2, wall_surfaces)
        gmsh.model.setPhysicalName(2, airframe_group, "AIRFRAME")
        farfield_group = gmsh.model.addPhysicalGroup(2, farfield_surfaces)
        gmsh.model.setPhysicalName(2, farfield_group, "FARFIELD")
        fluid_group = gmsh.model.addPhysicalGroup(3, [fluid_volumes[0][1]])
        gmsh.model.setPhysicalName(3, fluid_group, "FLUID")

        distance = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumbers(distance, "SurfacesList", wall_surfaces)
        gmsh.model.mesh.field.setNumber(distance, "Sampling", 100)
        threshold = gmsh.model.mesh.field.add("Threshold")
        gmsh.model.mesh.field.setNumber(threshold, "InField", distance)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMin", near_size_m)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMax", far_size_m)
        gmsh.model.mesh.field.setNumber(threshold, "DistMin", 0.01)
        gmsh.model.mesh.field.setNumber(threshold, "DistMax", 0.30)
        gmsh.model.mesh.field.setAsBackgroundMesh(threshold)

        gmsh.model.mesh.generate(2)
        duplicate_triangles = _remove_duplicate_surface_triangles(
            wall_surfaces + farfield_surfaces
        )
        if duplicate_triangles:
            print(
                "Removed "
                f"{duplicate_triangles:,} exact duplicate boundary triangles "
                "from the imported CAD mesh."
            )
        gmsh.model.mesh.generate(3)
        node_tags, _, _ = gmsh.model.mesh.getNodes()
        element_types, element_tags, _ = gmsh.model.mesh.getElements(
            3, fluid_volumes[0][1]
        )
        volume_elements = sum(len(tags) for tags in element_tags)
        gmsh.option.setNumber("Mesh.Format", 42)  # SU2 native mesh format.
        gmsh.write(str(output_path))
        nonconformal_markers = _conform_su2_boundary_markers(output_path)
        print(
            f"Wrote {output_path}: {len(node_tags):,} nodes, "
            f"{volume_elements:,} volume elements, "
            f"{len(wall_surfaces)} airframe surfaces, "
            f"{len(farfield_surfaces)} farfield surfaces; "
            f"filtered {nonconformal_markers:,} marker triangles "
            "without a matching tetrahedron boundary face."
        )
        return len(node_tags), volume_elements
    finally:
        gmsh.finalize()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", choices=("pr3_baseline", "airfoil_candidate"),
                        default="pr3_baseline")
    parser.add_argument("--step", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--domain-padding-m", type=float, default=1.0)
    parser.add_argument("--near-size-m", type=float, default=0.004)
    parser.add_argument("--far-size-m", type=float, default=0.12)
    args = parser.parse_args()
    default_step = ROOT / ("airframe_oml.step" if args.geometry == "pr3_baseline"
                           else "airframe_candidate_oml.step")
    default_dir = (ROOT / "tmp" / "su2-15mps" if args.geometry == "pr3_baseline"
                   else ROOT / "tmp" / f"su2-15mps-{args.geometry}")
    generate_mesh(
        (args.step or default_step).resolve(),
        (args.output or (default_dir / "airframe_domain.su2")).resolve(),
        args.domain_padding_m,
        args.near_size_m,
        args.far_size_m,
    )


if __name__ == "__main__":
    main()
