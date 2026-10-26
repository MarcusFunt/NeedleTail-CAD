"""Render the full SU2 drone solution with pressure surfaces and streamlines."""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import struct
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import colors, colormaps
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from airframe_geometry import PARAMETERS


CASE_DIR = ROOT / "tmp" / "su2-rotor"
DEFAULT_OUTPUT = CASE_DIR / "drone_cfd_streamlines.png"
MM_TO_M = 0.001
FREESTREAM_SPEED = 15.0
FREESTREAM_PRESSURE = 101325.0
FREESTREAM_DENSITY = 1.225


def _read_vtu(path: Path) -> dict[str, np.ndarray]:
    data = path.read_bytes()
    xml_end = data.index(b"<AppendedData")
    xml = data[:xml_end].decode("ascii")
    appended = data.index(b'<AppendedData encoding="raw">\n_') + len(
        b'<AppendedData encoding="raw">\n_'
    )
    arrays: dict[str, np.ndarray] = {}
    dtypes = {"Float32": "<f4", "Float64": "<f8", "Int32": "<i4", "UInt8": "u1"}
    for tag in re.findall(r"<DataArray[^>]+>", xml):
        match = re.search(r'Name="([^"]*)"', tag)
        name = match.group(1) if match else ""
        value_type = re.search(r'type="([^"]+)"', tag).group(1)
        offset = int(re.search(r'offset="(\d+)"', tag).group(1))
        components_match = re.search(r'NumberOfComponents\s*=\s*"?(\d+)', tag)
        components = int(components_match.group(1)) if components_match else 1
        dtype = dtypes[value_type]
        byte_count = struct.unpack_from("<Q", data, appended + offset)[0]
        values = np.frombuffer(
            data,
            dtype=dtype,
            count=byte_count // np.dtype(dtype).itemsize,
            offset=appended + offset + 8,
        )
        arrays[name] = values.reshape(-1, components) if components > 1 else values
    return arrays


def _read_surface_marker(path: Path, marker_name: str) -> tuple[np.ndarray, np.ndarray]:
    """Return SU2 node coordinates and triangle IDs for one named marker."""
    lines = path.read_text(encoding="utf-8").splitlines()
    cursor = 0
    if not lines[cursor].startswith("NDIME=") or int(lines[cursor].split("=", 1)[1]) != 3:
        raise ValueError(f"Expected a 3D SU2 zone mesh: {path}")
    cursor += 1
    if not lines[cursor].startswith("NELEM="):
        raise ValueError(f"Missing volume element count in {path}")
    volume_count = int(lines[cursor].split("=", 1)[1])
    cursor += 1 + volume_count
    if not lines[cursor].startswith("NPOIN="):
        raise ValueError(f"Missing point count in {path}")
    point_count = int(lines[cursor].split("=", 1)[1])
    cursor += 1
    points = np.asarray(
        [[float(value) for value in lines[row].split()[:3]]
         for row in range(cursor, cursor + point_count)],
        dtype=np.float64,
    )
    cursor += point_count
    if not lines[cursor].startswith("NMARK="):
        raise ValueError(f"Missing marker count in {path}")
    marker_count = int(lines[cursor].split("=", 1)[1])
    cursor += 1
    for _ in range(marker_count):
        name = lines[cursor].split("=", 1)[1].strip()
        count = int(lines[cursor + 1].split("=", 1)[1])
        cursor += 2
        rows = lines[cursor:cursor + count]
        cursor += count
        if name != marker_name:
            continue
        triangles = []
        for row in rows:
            element = [int(value) for value in row.split()]
            if element[0] == 5 and len(element) == 4:
                triangles.append(element[1:4])
        return points, np.asarray(triangles, dtype=np.int64)
    raise ValueError(f"Marker {marker_name!r} was not found in {path}")


class TetField:
    """Locate points in a linear-tetrahedron VTK field and interpolate velocity."""

    def __init__(self, arrays: dict[str, np.ndarray]):
        self.points = arrays[""].astype(np.float64, copy=False)
        self.cells = arrays["connectivity"].reshape(-1, 4).astype(np.int64, copy=False)
        self.velocity = arrays["Velocity"].astype(np.float64, copy=False)
        self.centroids = self.points[self.cells].mean(axis=1)
        self.tree = cKDTree(self.centroids, compact_nodes=True, balanced_tree=True)

    def velocity_at(self, position: np.ndarray) -> np.ndarray | None:
        for candidate_count in (16, 64):
            _, ids = self.tree.query(position, k=candidate_count)
            ids = np.atleast_1d(ids).astype(np.int64, copy=False)
            vertices = self.points[self.cells[ids]]
            matrix = np.stack(
                (vertices[:, 1] - vertices[:, 0],
                 vertices[:, 2] - vertices[:, 0],
                 vertices[:, 3] - vertices[:, 0]),
                axis=2,
            )
            determinant = np.linalg.det(matrix)
            valid_matrix = np.abs(determinant) > 1e-20
            if not np.any(valid_matrix):
                continue
            barycentric_tail = np.full((len(ids), 3), np.nan)
            barycentric_tail[valid_matrix] = np.linalg.solve(
                matrix[valid_matrix],
                (position - vertices[valid_matrix, 0])[..., None],
            )[..., 0]
            weights = np.column_stack(
                (1.0 - np.nansum(barycentric_tail, axis=1), barycentric_tail)
            )
            inside = (
                valid_matrix
                & (weights.min(axis=1) >= -2e-5)
                & (weights.max(axis=1) <= 1.00002)
            )
            if np.any(inside):
                index = int(np.flatnonzero(inside)[0])
                return weights[index] @ self.velocity[self.cells[ids[index]]]
        return None


def _load_zone(index: int, time_index: int) -> tuple[dict[str, np.ndarray], TetField, np.ndarray]:
    if index == 0:
        zone_path = CASE_DIR / "rotor_zones" / "zone_1_static.su2"
        marker = "AIRFRAME_MOTOR"
        vtk_path = CASE_DIR / f"rotor_flow_0_{time_index:05d}.vtu"
    else:
        zone_path = CASE_DIR / "rotor_zones" / f"zone_{index + 1}_rotor.su2"
        marker = f"ROTOR_WALL_{index}"
        vtk_path = CASE_DIR / f"rotor_flow_{index}_{time_index:05d}.vtu"
    arrays = _read_vtu(vtk_path)
    mesh_points, triangles = _read_surface_marker(zone_path, marker)
    vtk_points = arrays[""].astype(np.float64, copy=False)
    if mesh_points.shape != vtk_points.shape:
        raise ValueError(
            f"Node count differs between {zone_path.name} and {vtk_path.name}: "
            f"{mesh_points.shape[0]} vs {vtk_points.shape[0]}"
        )
    mismatch = np.linalg.norm(mesh_points - vtk_points, axis=1).max()
    if mismatch > 2e-6:
        raise ValueError(f"Marker points do not align with {vtk_path.name} (max error {mismatch:g} m)")
    return arrays, TetField(arrays), triangles


def _build_rotor_zones(
    arrays: list[dict[str, np.ndarray]], fields: list[TetField]
) -> list[tuple[float, float, float, float, float]]:
    zones = []
    for index, ((y_mm, z_mm), data) in enumerate(
        zip(PARAMETERS.propeller_mount_stations, arrays[1:]), start=1
    ):
        points = data[""].astype(np.float64, copy=False)
        center_y, center_z = y_mm * MM_TO_M, z_mm * MM_TO_M
        radius = np.hypot(points[:, 1] - center_y, points[:, 2] - center_z).max()
        zones.append((points[:, 0].min(), points[:, 0].max(), center_y, center_z, radius))
    return zones


def _velocity_at(
    position: np.ndarray,
    fields: list[TetField],
    rotor_zones: list[tuple[float, float, float, float, float]],
) -> np.ndarray | None:
    for index, (x0, x1, center_y, center_z, radius) in enumerate(rotor_zones, start=1):
        radial = np.hypot(position[1] - center_y, position[2] - center_z)
        if x0 - 1e-6 <= position[0] <= x1 + 1e-6 and radial <= radius + 1e-6:
            value = fields[index].velocity_at(position)
            if value is not None:
                return value
    return fields[0].velocity_at(position)


def _trace_streamline(
    seed: np.ndarray,
    fields: list[TetField],
    rotor_zones: list[tuple[float, float, float, float, float]],
    step_length: float,
    end_x: float,
) -> tuple[np.ndarray, np.ndarray] | None:
    positions = [seed.astype(np.float64)]
    speeds: list[float] = []
    position = positions[0]
    travelled = 0.0
    for _ in range(220):
        velocity = _velocity_at(position, fields, rotor_zones)
        if velocity is None:
            break
        speed = float(np.linalg.norm(velocity))
        if not np.isfinite(speed) or speed < 0.05:
            break
        tangent = velocity / speed
        midpoint = position + 0.5 * step_length * tangent
        midpoint_velocity = _velocity_at(midpoint, fields, rotor_zones)
        if midpoint_velocity is None:
            break
        midpoint_speed = float(np.linalg.norm(midpoint_velocity))
        if not np.isfinite(midpoint_speed) or midpoint_speed < 0.05:
            break
        position = position + step_length * midpoint_velocity / midpoint_speed
        positions.append(position.copy())
        speeds.append(midpoint_speed)
        travelled += step_length
        if position[0] <= end_x or travelled >= 0.62:
            break
    if len(positions) < 8:
        return None
    return np.asarray(positions), np.asarray(speeds)


def render_model_view(output: Path, time_index: int = 120) -> Path:
    """Render the solved wall surfaces as a clean, assembled drone view."""
    loaded = [_load_zone(index, time_index) for index in range(5)]
    figure = plt.figure(figsize=(12, 8), facecolor="white")
    axes = figure.add_subplot(111, projection="3d")
    axes.set_proj_type("ortho")
    light = np.asarray((0.35, -0.50, 0.79), dtype=np.float64)
    light /= np.linalg.norm(light)
    palettes = ("#8faab8", "#d1a65d", "#bd795d", "#d7b66b", "#bd795d")
    all_vertices = []
    for zone_index, (arrays, _field, triangles) in enumerate(loaded):
        points = arrays[""].astype(np.float64, copy=False)
        faces = points[triangles]
        all_vertices.append(faces.reshape(-1, 3))
        normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
        norm = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(norm[:, None], 1e-20)
        brightness = 0.52 + 0.48 * np.abs(normals @ light)
        base = np.asarray(colors.to_rgb(palettes[zone_index]))
        facecolors = np.clip(brightness[:, None] * base[None, :], 0, 1)
        axes.add_collection3d(
            Poly3DCollection(
                faces,
                facecolors=facecolors,
                edgecolors="none",
                linewidths=0,
                antialiased=True,
            )
        )

    bounds = np.concatenate(all_vertices, axis=0)
    minimum, maximum = bounds.min(axis=0), bounds.max(axis=0)
    margin = np.maximum((maximum - minimum) * 0.12, 0.012)
    axes.set_xlim(minimum[0] - margin[0], maximum[0] + margin[0])
    axes.set_ylim(minimum[1] - margin[1], maximum[1] + margin[1])
    axes.set_zlim(minimum[2] - margin[2], maximum[2] + margin[2])
    axes.set_box_aspect(maximum - minimum)
    axes.view_init(elev=25, azim=-43)
    axes.set_axis_off()
    figure.suptitle("NeedleTail · complete motor and propeller assembly", y=0.975, fontsize=17)
    figure.text(
        0.5,
        0.935,
        "Airframe with four outrunner motors and four propellers · final SU2 geometry",
        ha="center",
        color="#394452",
        fontsize=10,
    )
    figure.subplots_adjust(left=0.02, right=0.98, top=0.89, bottom=0.02)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=190, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    print(f"Saved {output}: four motor and propeller assemblies.")
    return output


def render(output: Path, time_index: int = 120, seed_count: int = 9) -> Path:
    loaded = [_load_zone(index, time_index) for index in range(5)]
    arrays = [item[0] for item in loaded]
    fields = [item[1] for item in loaded]
    surface_markers = [item[2] for item in loaded]
    rotor_zones = _build_rotor_zones(arrays, fields)

    streamlines: list[tuple[np.ndarray, np.ndarray]] = []
    seed_values = np.linspace(-0.155, 0.155, seed_count)
    for y in seed_values:
        for z in seed_values:
            streamline = _trace_streamline(
                np.asarray((0.255, y, z)), fields, rotor_zones, 0.008, -0.235
            )
            if streamline is not None:
                streamlines.append(streamline)
    if not streamlines:
        raise RuntimeError("No streamlines could be traced from the requested seed plane")

    vertices: list[np.ndarray] = []
    surface_pressures: list[np.ndarray] = []
    for data, triangles in zip(arrays, surface_markers):
        points = data[""].astype(np.float64, copy=False)
        pressure = data["Pressure"].astype(np.float64, copy=False)
        vertices.append(points[triangles])
        surface_pressures.append(pressure[triangles].mean(axis=1))
    surface_vertices = np.concatenate(vertices, axis=0)
    surface_pressure = np.concatenate(surface_pressures)

    line_segments = []
    line_speeds = []
    for points, speeds in streamlines:
        line_segments.extend(np.stack((points[:-1], points[1:]), axis=1))
        line_speeds.extend(speeds.tolist())
    line_segments_array = np.asarray(line_segments)
    line_speed_array = np.asarray(line_speeds)

    figure = plt.figure(figsize=(13, 8.5), facecolor="white")
    axes = figure.add_subplot(111, projection="3d")
    axes.set_proj_type("ortho")
    pressure_min, pressure_max = np.percentile(surface_pressure, (1, 99))
    pressure_norm = colors.Normalize(vmin=pressure_min, vmax=pressure_max, clip=True)
    pressure_map = colormaps["coolwarm"]
    wall_mesh = Poly3DCollection(
        surface_vertices,
        facecolors=pressure_map(pressure_norm(surface_pressure)),
        edgecolors="none",
        linewidths=0,
        antialiased=False,
    )
    axes.add_collection3d(wall_mesh)

    speed_norm = colors.Normalize(vmin=14.8, vmax=17.2, clip=True)
    speed_map = colormaps["RdYlGn_r"]
    flow_lines = Line3DCollection(
        line_segments_array,
        cmap=speed_map,
        norm=speed_norm,
        linewidths=1.2,
        alpha=0.84,
    )
    flow_lines.set_array(line_speed_array)
    axes.add_collection3d(flow_lines)

    axes.set_xlim(-0.235, 0.255)
    axes.set_ylim(-0.17, 0.17)
    axes.set_zlim(-0.15, 0.15)
    axes.set_box_aspect((0.49, 0.34, 0.30))
    axes.view_init(elev=25, azim=-43)
    axes.set_xlabel("X (m) · airflow −X", labelpad=10)
    axes.set_ylabel("Y (m)", labelpad=8)
    axes.set_zlabel("Z (m)", labelpad=8)
    axes.set_title("NeedleTail · SU2 airflow around the complete drone", pad=18, fontsize=17)
    axes.grid(True, color="#d8dde3", linewidth=0.55, alpha=0.7)
    axes.xaxis.pane.set_facecolor((0.98, 0.98, 0.98, 1))
    axes.yaxis.pane.set_facecolor((0.98, 0.98, 0.98, 1))
    axes.zaxis.pane.set_facecolor((0.98, 0.98, 0.98, 1))

    pressure_bar = figure.colorbar(
        plt.cm.ScalarMappable(norm=pressure_norm, cmap=pressure_map),
        ax=axes,
        shrink=0.60,
        pad=0.08,
        location="right",
    )
    pressure_bar.set_label("Body and rotor surface pressure (Pa)")
    speed_bar = figure.colorbar(
        flow_lines,
        ax=axes,
        shrink=0.60,
        pad=0.13,
        location="left",
    )
    speed_bar.set_label("Air speed along streamlines (m/s)")
    figure.text(
        0.5,
        0.91,
        "40,000 RPM · 15 m/s cruise · final SU2 time level, 1.5 ms · pressure on surfaces, speed on lines",
        ha="center",
        color="#394452",
        fontsize=10,
    )
    figure.subplots_adjust(left=0.02, right=0.93, top=0.90, bottom=0.03)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    print(
        f"Saved {output}: {len(streamlines)} streamlines, "
        f"{len(surface_pressure):,} pressure-colored surface triangles."
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--time-index", type=int, default=120)
    parser.add_argument("--seed-count", type=int, default=9)
    parser.add_argument("--model-view", action="store_true", help="Render the assembled drone without flow lines")
    args = parser.parse_args()
    if args.seed_count < 5:
        raise ValueError("Use at least five seed points on each side of the plane")
    if args.model_view:
        render_model_view(args.output.resolve(), args.time_index)
    else:
        render(args.output.resolve(), args.time_index, args.seed_count)


if __name__ == "__main__":
    main()
