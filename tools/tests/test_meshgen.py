"""Frame and quality checks of the meshgen assets, read back from the written files."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import meshgen

N = 64
D = 2 * np.pi / N
TOL = 1e-9


def load(path):
    """OBJ in metres -> (vertices in mm, faces)."""
    v, f = [], []
    for line in Path(path).read_text().splitlines():
        k, *rest = line.split()
        (v if k == "v" else f).append([float(x) if k == "v" else int(x) - 1 for x in rest])
    return np.array(v) * 1000, np.array(f)


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    return tmp_path_factory.mktemp("data")


def generate(out, name):
    asset = meshgen.build(name)
    meshgen.check(asset)
    meshgen.write(asset, out)
    P, T = load(meshgen.asset_dir(out, name) / "surface.obj")
    (P0, T0), _ = meshgen.meshes(asset)
    assert np.abs(P - P0).max() < 1e-12 and (T == T0).all()
    return asset, P, T


def pieces_of(out, name):
    return [load(p) for p in sorted((meshgen.asset_dir(out, name) / "pieces").glob("*.obj"))]


def test_box_with_origin_offset(out):
    _, P, T = generate(out, "box_400x400x50_o50x0x0")
    assert np.allclose(P.min(0), [-250, -200, -25], atol=TOL) and np.allclose(P.max(0), [150, 200, 25], atol=TOL)
    assert len(P) == 8 and len(T) == 12


def test_cylinder(out):
    _, P, T = generate(out, f"cylinder_r100_h20_seg{N}")
    assert np.allclose(P.min(0), [-100, -100, -10]) and np.allclose(P.max(0), [100, 100, 10])
    assert np.allclose(np.hypot(P[:, 0], P[:, 1])[P[:, 2] != 0].max(), 100, atol=TOL)
    assert abs(meshgen.volume(P, T) - np.pi * 100 ** 2 * 20) / (np.pi * 100 ** 2 * 20) < D * D / 5


def test_sphere(out):
    _, P, T = generate(out, f"sphere_r100_seg{N}")
    assert np.allclose(np.linalg.norm(P, axis=1), 100, atol=TOL)
    assert np.allclose(meshgen.centroid(P, T), 0, atol=TOL)
    assert len(P) == N * (N // 2 - 1) + 2  # N longitudes, N/2 latitude bands


def test_frustum(out):
    _, P, T = generate(out, f"frustum_r50_r100_h100_seg{N}")
    zbar = 100 * (2500 + 10000 + 30000) / (4 * (2500 + 5000 + 10000))
    assert np.isclose(P[:, 2].min(), -zbar, atol=TOL) and np.isclose(P[:, 2].max(), 100 - zbar, atol=TOL)
    assert np.isclose(np.hypot(P[:, 0], P[:, 1])[np.isclose(P[:, 2], -zbar)].max(), 50, atol=TOL)  # a at the bottom
    assert np.isclose(np.hypot(P[:, 0], P[:, 1]).max(), 100, atol=TOL)
    assert np.allclose(meshgen.centroid(P, T), 0, atol=TOL)


def test_ring(out):
    _, P, _T = generate(out, f"ring_ro250_ri150_h50_seg{N}")
    assert np.isclose(np.hypot(P[:, 0], P[:, 1]).max(), 250, atol=TOL)
    # the hole polygon is circumscribed: its facets are tangent to the true hole of radius 150
    rho = np.hypot(P[:, 0], P[:, 1])
    inner = rho < 200
    assert inner.sum() == 2 * N and np.allclose(rho[inner], 150 / np.cos(D / 2), atol=TOL)
    assert len(pieces_of(out, f"ring_ro250_ri150_h50_seg{N}")) == N


def test_vgroove(out):
    _, P, _T = generate(out, "vgroove_w300_l4000")
    assert np.allclose(P.min(0), [-2000, -150, -100]) and np.allclose(P.max(0), [2000, 150, 150])
    assert (np.abs(P[:, 2] - np.abs(P[:, 1])) < TOL)[P[:, 2] > -100].all()  # faces z = |y|
    assert len(pieces_of(out, "vgroove_w300_l4000")) == 2


def test_hole(out):
    _, P, _T = generate(out, "hole_s99.8_d100")
    assert np.isclose(P[:, 2].max(), 0) and np.isclose(P[:, 2].min(), -100)
    assert np.isclose(np.abs(P[:, :2]).max(), 149.9) and np.isclose(np.abs(P[:, :2]).min(), 49.9)
    assert len(pieces_of(out, "hole_s99.8_d100")) == 4


def test_corner(out):
    _, P, _T = generate(out, "corner_s300")
    assert np.allclose(P.min(0), [-50, -50, -50]) and np.allclose(P.max(0), [300, 300, 300])
    assert (np.abs(P - 0).sum(1) < TOL).any()  # the inner corner is a vertex at the origin
    pieces = pieces_of(out, "corner_s300")
    assert sorted(tuple(np.round(p.max(0) - p.min(0))) for p, _ in pieces) == [
        (50, 350, 300), (300, 50, 300), (350, 350, 50)]
    assert any(np.allclose(p.min(0), [-50, -50, -50]) for p, _ in pieces)  # floor
    assert any(np.allclose(p.min(0), [0, -50, 0]) and np.allclose(p.max(0), [300, 0, 300]) for p, _ in pieces)


def test_bowl(out):
    _, P, _T = generate(out, f"bowl_r300_t20_a60_seg{N}")
    ch = np.cos(D / 2)
    cav = (np.linalg.norm(P - (0, 0, 300), axis=1) < 300 / ch**2 + 1e-6)  # cavity vertices
    assert np.isclose(P[cav][:, 2].min(), 300 - 300 / ch, atol=TOL)  # pole, just under the true lowest point
    assert (np.linalg.norm(P - (0, 0, 300), axis=1) >= 300 - TOL).all()
    assert len(pieces_of(out, f"bowl_r300_t20_a60_seg{N}")) == 11 * N  # 60 deg rounded to 11 steps


def test_trough(out):
    name = f"trough_r300_t20_a60_l500_seg{N}"
    _, P, _T = generate(out, name)
    assert np.allclose([P[:, 1].min(), P[:, 1].max()], [-250, 250])
    cav = np.hypot(P[:, 0], P[:, 2] - 300) < 300 / np.cos(D / 2) + 1e-6
    assert np.isclose(P[cav][:, 2].min(), 300 - 300 / np.cos(D / 2), atol=TOL)  # bottom line, axis at z = R
    assert len(pieces_of(out, name)) == 22


def test_table(out):
    _, P, T = generate(out, "table_1000x600x200")
    assert np.allclose(P.min(0), [-500, -300, -200]) and np.allclose(P.max(0), [500, 300, 0])  # top at z = 0
    assert len(P) == 8 and len(T) == 12
    assert not (meshgen.asset_dir(out, "table_1000x600x200") / "pieces").exists()
    for n, tile in ((2, (800, 1600, 200)), (8, (400, 800, 200)), (64, (200, 200, 200))):
        name = f"table_1600x1600x200_p{n}"
        _, P, T = generate(out, name)
        assert np.allclose(P.min(0), [-800, -800, -200]) and np.allclose(P.max(0), [800, 800, 0])
        assert np.isclose(meshgen.volume(P, T), 1600 * 1600 * 200)
        pieces = pieces_of(out, name)
        assert len(pieces) == n
        assert all(np.allclose(p.max(0) - p.min(0), tile) and len(p) == 8 for p, _ in pieces)
    with pytest.raises(SystemExit):
        meshgen.build("table_1600x1600x200_p6")  # not a power of two


def test_bearing(out):
    name = f"bearing_r50_t20_l200_seg{N}"
    _, P, _T = generate(out, name)
    r = np.hypot(P[:, 0], P[:, 2])
    assert np.allclose([P[:, 1].min(), P[:, 1].max()], [-100, 100])
    assert np.isclose(r.min(), 50 / np.cos(D / 2), atol=TOL) and np.isclose(r.max(), 70 / np.cos(D / 2), atol=TOL)
    assert len(pieces_of(out, name)) == N


def test_seat(out):
    name = f"seat_r50_r100_h100_t20_seg{N}"
    _, P, _T = generate(out, name)
    zbar = 100 * 42500 / 70000
    rho = np.hypot(P[:, 0], P[:, 1])
    cone = 50 + 0.5 * (P[:, 2] + zbar)  # true cone radius at each height, a = 50 at the frustum bottom
    inner = np.abs(rho * np.cos(D / 2) - cone) < 1e-6  # facets tangent to the true cone: apothem = cone radius
    assert inner.sum() == 2 * N and np.isclose(cone[inner].min(), 50 - 5, atol=TOL)
    assert np.isclose(P[:, 2].min(), -zbar - 10) and np.isclose(P[:, 2].max(), 100 - zbar + 10)
    assert len(pieces_of(out, name)) == N


def test_broken_assets_fail():
    a = meshgen.build("box_100x100x100")
    a.pool.v[0] = (-60.0, -50.0, -50.0)
    with pytest.raises(SystemExit):
        meshgen.check(a)
    a = meshgen.build(f"bearing_r50_t20_l200_seg{N}")
    a.pieces[1] = a.pieces[0]  # a duplicated piece overlaps and breaks the volume sum
    with pytest.raises(SystemExit):
        meshgen.check(a)


def test_missing_volumes_only_asks_for_absent_ones(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(meshgen.subprocess, "run", lambda cmd, check: calls.append(cmd))
    names = ["box_100x100x100", f"sphere_r100_seg{N}", "slab_40000x40000x1000"]
    for n in names:
        meshgen.write(meshgen.build(n), tmp_path)
    (meshgen.asset_dir(tmp_path, names[0]) / "volume.vtk").write_text("")
    meshgen.missing_volumes(names, tmp_path)
    assert len(calls) == 1 and [Path(p).parent.name for p in calls[0][3:]] == [names[1]]
    calls.clear()
    (meshgen.asset_dir(tmp_path, names[1]) / "volume.vtk").write_text("")
    meshgen.missing_volumes(names, tmp_path)
    assert calls == []
