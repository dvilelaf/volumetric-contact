# /// script
# requires-python = ">=3.11"
# dependencies = ["coacd>=1.0", "trimesh>=4.0", "numpy", "scipy"]
# ///
"""Splits a closed mesh into convex pieces (CoACD) for the distributed model.

Writes <out_dir>/<stem>_<k>.obj; the pieces of asset <name> are the .obj files
in <data_dir>/<group>/<name>/pieces/. The pieces keep the input mesh's frame.

usage: uv run tools/decompose.py mesh.obj out_dir [--threshold 0.05]
"""

import argparse
from pathlib import Path

import coacd
import numpy as np
import trimesh


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--threshold", type=float, default=0.05, help="concavity tolerance (smaller: more pieces)")
    args = ap.parse_args()

    mesh = trimesh.load(args.mesh, force="mesh", process=True)
    if not mesh.is_watertight:
        raise SystemExit(f"{args.mesh} is not closed")
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=args.threshold)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    width = len(str(len(parts)))
    for k, (vertices, faces) in enumerate(parts):
        piece = trimesh.Trimesh(np.asarray(vertices), np.asarray(faces)).convex_hull
        piece.export(args.out_dir / f"{args.mesh.stem}_{k:0{width}d}.obj")
    print(f"{len(parts)} convex pieces in {args.out_dir}")


if __name__ == "__main__":
    main()
