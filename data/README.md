# data/

Mesh assets of the scenario battery (spec: `docs/scenarios.md` §1).

```
bodies/<asset>/    box, cylinder, sphere, frustum, ring
grounds/<asset>/   slab, table, vgroove, hole, corner, bowl, trough, bearing, seat
  surface.obj      whole closed surface
  volume.vtk       tetrahedral mesh (generated, gitignored; every asset but slabs)
  pieces/NNN.obj   convex pieces (non-convex assets only)
```

## Names

`<kind>_<params>`, parameters in millimetres, decimals allowed (`hole_s99.8_d100`).
Curved assets end in `_seg<N>`: N segments per full turn (32 coarse, 64 base, 128 fine).
`table_<L>x<W>x<H>` is a finite box with its top at z = 0; `_p<N>` cuts it into N equal tiles
(`pieces/`), the same solid in 2, 8 or 64 convex pieces.
`bowl_r105_t20_a60_seg64` belongs to no scenario: it is the conforming seat of `sphere_r100`
that `tests/test_invariants.cpp` needs (base resolution only).

## Frames and units

Files are in metres; each mesh is in its own body frame. `box_..._o50x0x0` carries an
origin offset (mm) in its name.

## Criteria in short

Closed, manifold, outward triangles, positive volume, 17 significant digits. Meshes
approximate the solid from inside (cavities enclose the true cavity). Pieces are convex,
disjoint and add up to the solid.

## Regenerate

```
uv run tools/meshgen.py            # everything, deterministic, verified
uv run tools/meshgen.py NAME ...   # selected assets
```

`surface.obj` and `pieces/` are committed; `volume.vtk` is rebuilt by `tools/volume_mesh.py`.
