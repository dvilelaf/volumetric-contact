# /// script
# requires-python = ">=3.11"
# dependencies = ["tetgen>=0.6", "trimesh>=4.0", "numpy", "scipy", "rtree"]
# ///
"""Writes a tetrahedral volume mesh next to each closed surface mesh, for Drake's
compliant hydroelastic contact.

<stem>.vtk is a legacy ASCII VTK unstructured grid of tetrahedra (cell type 10)
whose boundary is the input surface. Line 2 is `volcontact foundation_depth=<H>`:
H is the largest distance from a vertex to the surface, the elastic foundation
depth Drake uses in p = E·φ/H. Tetrahedra are positively oriented, as Drake
expects: det[V1-V0, V2-V0, V3-V0] > 0.

Size control: tetgen's maximum tetrahedron volume is V / n_tets, so every mesh
has about n_tets tetrahedra at any scale. Drake's φ is affine on any tetrahedron
whose vertices project onto the same face, so depth is reproduced exactly next to
flat faces and well next to curved ones as long as the edges stay well below the
distance to the other faces.

usage: uv run tools/volume_mesh.py surface.obj [more.obj ...] [--tets 30000]
"""

import argparse
import time
from pathlib import Path

import numpy as np
import tetgen
import trimesh

MAX_TETS = 200_000


def tetrahedralize(
    surface: trimesh.Trimesh, n_tets: int
) -> tuple[np.ndarray, np.ndarray]:
    tgen = tetgen.TetGen(surface.vertices, surface.faces)
    # tetgen's maxvolume keyword is ignored by the python wrapper, so pass the switch.
    nodes, cells = tgen.tetrahedralize(switches=f"pqa{surface.volume / n_tets:.17g}")[
        :2
    ]
    nodes = np.asarray(nodes, dtype=np.float64)
    cells = np.asarray(cells, dtype=np.int64)
    # Drake wants positive orientation; tetgen does not guarantee it.
    volumes = signed_volumes(nodes, cells)
    flip = volumes < 0
    cells[flip] = cells[flip][:, [0, 2, 1, 3]]
    return nodes, cells


def signed_volumes(nodes: np.ndarray, cells: np.ndarray) -> np.ndarray:
    a, b, c, d = (nodes[cells[:, i]] for i in range(4))
    return np.einsum("ij,ij->i", b - a, np.cross(c - a, d - a)) / 6.0


def foundation_depth(surface: trimesh.Trimesh, nodes: np.ndarray) -> float:
    _, distance, _ = trimesh.proximity.closest_point(surface, nodes)
    return float(distance.max())


def write_vtk(path: Path, nodes: np.ndarray, cells: np.ndarray, depth: float) -> None:
    lines = [
        "# vtk DataFile Version 3.0",
        f"volcontact foundation_depth={depth:.17g}",
        "ASCII",
        "DATASET UNSTRUCTURED_GRID",
        f"POINTS {len(nodes)} double",
    ]
    lines += [" ".join(f"{x:.17g}" for x in p) for p in nodes]
    lines.append(f"CELLS {len(cells)} {5 * len(cells)}")
    lines += [f"4 {a} {b} {c} {d}" for a, b, c, d in cells]
    lines.append(f"CELL_TYPES {len(cells)}")
    lines += ["10"] * len(cells)
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("surfaces", type=Path, nargs="+")
    ap.add_argument(
        "--tets", type=int, default=30000, help="approximate tetrahedra per mesh"
    )
    args = ap.parse_args()

    for path in args.surfaces:
        start = time.perf_counter()
        surface = trimesh.load(path, force="mesh", process=True)
        if not surface.is_watertight:
            raise SystemExit(f"{path} is not closed")
        if surface.volume <= 0:
            raise SystemExit(f"{path} is not outward-facing")
        nodes, cells = tetrahedralize(surface, args.tets)
        if len(cells) > MAX_TETS:
            raise SystemExit(
                f"{path}: {len(cells)} tetrahedra, above the {MAX_TETS} limit; use --tets"
            )
        depth = foundation_depth(surface, nodes)
        out = path.with_name("volume.vtk") if path.name == "surface.obj" else path.with_suffix(".vtk")
        write_vtk(out, nodes, cells, depth)
        print(
            f"{out}: {len(nodes)} vertices, {len(cells)} tets, H={depth:.6g} m, {time.perf_counter() - start:.1f} s"
        )


if __name__ == "__main__":
    main()
