"""Tests of tools/volume_mesh.py through its command line.

The tool's own dependencies (tetgen, rtree) come from its PEP 723 header, which
`uv run` resolves; the tests need only pytest, numpy, scipy and trimesh.

Run: uv run --with pytest --with numpy --with scipy --with trimesh pytest -q tools/tests
"""

import re
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np
import pytest
import trimesh

TOOL = Path(__file__).resolve().parents[1] / "volume_mesh.py"
BOX_HALF = np.array([0.1, 0.1, 0.3])  # 0.2 x 0.2 x 0.6 m


def box_surface() -> trimesh.Trimesh:
    return trimesh.creation.box(extents=2 * BOX_HALF)


def l_surface() -> trimesh.Trimesh:
    """L-shaped prism, 0.2 x 0.2 m footprint with a 0.1 m notch, 0.6 m high."""
    outline = np.array([[0, 0], [0.2, 0], [0.2, 0.1], [0.1, 0.1], [0.1, 0.2], [0, 0.2]])
    caps = [[0, 1, 2], [0, 2, 3], [0, 3, 5], [3, 4, 5]]  # counter-clockwise from +z
    vertices = np.vstack([np.c_[outline, np.zeros(6)], np.c_[outline, np.full(6, 0.6)]])
    faces = [[c[0], c[2], c[1]] for c in caps]  # bottom, seen from -z
    faces += [[c[0] + 6, c[1] + 6, c[2] + 6] for c in caps]  # top
    for i in range(6):
        j = (i + 1) % 6
        faces += [[i, j, j + 6], [i, j + 6, i + 6]]
    mesh = trimesh.Trimesh(vertices, np.array(faces), process=True)
    assert mesh.is_watertight and mesh.volume > 0
    return mesh


def annulus_surface() -> trimesh.Trimesh:
    """Faceted annulus (64 sections), inner radius 50 mm, outer 100 mm, 50 mm high."""
    return trimesh.creation.annulus(r_min=0.05, r_max=0.1, height=0.05, sections=64)


SOLIDS = {"box": box_surface, "l": l_surface, "annulus": annulus_surface}


def run_tool(*paths: Path) -> None:
    subprocess.run(["uv", "run", "--quiet", str(TOOL), *map(str, paths)], check=True)


def distance_to_surface(mesh: trimesh.Trimesh, points: np.ndarray) -> np.ndarray:
    """Exact point-triangle distance, brute force over the (small) test surfaces."""
    triangles = mesh.triangles
    out = np.empty(len(points))
    chunk = max(1, 200_000 // len(triangles))
    for start in range(0, len(points), chunk):
        p = points[start : start + chunk]
        pairs = np.repeat(p, len(triangles), axis=0)
        tris = np.tile(triangles, (len(p), 1, 1))
        closest = trimesh.triangles.closest_point(tris, pairs)
        out[start : start + len(p)] = (
            np.linalg.norm(closest - pairs, axis=1).reshape(len(p), -1).min(axis=1)
        )
    return out


def read_vtk(path: Path):
    lines = path.read_text().splitlines()
    title = lines[1]
    n_points = int(lines[4].split()[1])
    nodes = np.array(
        [[float(x) for x in lines[5 + i].split()] for i in range(n_points)]
    )
    start = 5 + n_points
    n_cells = int(lines[start].split()[1])
    cells = np.array(
        [[int(x) for x in lines[start + 1 + i].split()[1:]] for i in range(n_cells)]
    )
    assert lines[start + 1 + n_cells] == f"CELL_TYPES {n_cells}"
    assert set(lines[start + 2 + n_cells :]) == {"10"}
    return title, nodes, cells


def tet_volumes(nodes, cells):
    a, b, c, d = (nodes[cells[:, i]] for i in range(4))
    return np.einsum("ij,ij->i", b - a, np.cross(c - a, d - a)) / 6.0


def interpolate(nodes, cells, values, point):
    """Linear interpolation of per-vertex values at a point inside the mesh."""
    a = nodes[cells[:, 0]]
    edges = np.stack([nodes[cells[:, i]] - a for i in (1, 2, 3)], axis=2)
    rhs = np.broadcast_to((point - a)[:, :, None], (len(cells), 3, 1))
    weights = np.linalg.solve(edges, rhs)[:, :, 0]
    bary = np.c_[1 - weights.sum(axis=1), weights]
    inside = np.flatnonzero((bary > -1e-9).all(axis=1))
    assert inside.size, "point is outside the mesh"
    k = inside[0]
    return float(bary[k] @ values[cells[k]])


@pytest.fixture(params=SOLIDS)
def solid(request, tmp_path):
    surface = tmp_path / f"{request.param}.obj"
    SOLIDS[request.param]().export(surface)
    run_tool(surface)
    # Compare against the file as the tool reads it (OBJ export rounds coordinates).
    mesh = trimesh.load(surface, force="mesh", process=True)
    title, nodes, cells = read_vtk(surface.with_suffix(".vtk"))
    return request.param, mesh, title, nodes, cells


def test_title_line_is_depth_of_deepest_vertex(solid):
    name, mesh, title, nodes, _ = solid
    match = re.fullmatch(r"volcontact foundation_depth=(\S+)", title)
    assert match, title
    depth = float(match.group(1))
    if name == "box":
        expected = np.max(np.min(BOX_HALF - np.abs(nodes), axis=1))
    else:
        expected = distance_to_surface(mesh, nodes).max()
    assert depth == pytest.approx(expected, abs=1e-12)
    assert match.group(1) == f"{depth:.17g}"  # 17 significant digits


def test_positive_volumes_and_boundary_is_the_input_surface(solid):
    _, mesh, _, nodes, cells = solid
    volumes = tet_volumes(nodes, cells)
    assert (volumes > 0).all()
    assert volumes.sum() == pytest.approx(mesh.volume, rel=1e-9)

    # Faces used by exactly one tetrahedron form the boundary.
    faces = Counter(
        tuple(sorted(f))
        for cell in cells
        for f in [cell[[0, 1, 2]], cell[[0, 1, 3]], cell[[0, 2, 3]], cell[[1, 2, 3]]]
    )
    boundary = np.array([f for f, count in faces.items() if count == 1])
    assert len(boundary) > 0
    tris = nodes[boundary]
    area = (
        0.5
        * np.linalg.norm(
            np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1
        ).sum()
    )
    assert area == pytest.approx(mesh.area, rel=1e-9)
    assert distance_to_surface(mesh, nodes[np.unique(boundary)]).max() < 1e-12


def test_pressure_is_linear_in_depth_next_to_every_face(solid):
    _, mesh, _, nodes, cells = solid
    phi = distance_to_surface(mesh, nodes)
    # Triangle centroids lie on every face; pushing them inwards gives the depth.
    for centre, outward in zip(mesh.triangles_center, mesh.face_normals):
        for depth in (1e-4, 1e-3):
            point = centre - depth * outward
            assert interpolate(nodes, cells, phi, point) == pytest.approx(
                depth, rel=0.05
            )


def test_tet_count_within_budget(solid):
    *_, cells = solid
    assert len(cells) <= 200_000
