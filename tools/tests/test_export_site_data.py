"""The slim site dataset (tools/viewer/export_site_data.py) of a synthetic results/traj."""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))
import export_site_data as export


def run(n=1001, spike_at=337, **extra):
    t = [k * 0.01 for k in range(n)]
    v = [math.sin(k / 50) for k in range(n)]
    v[spike_at] = 7.0  # a one-sample spike that even spacing would miss
    poses = [[0.123456789 * k, 0.0, -0.0, 1.0, 1e-9, 0.0, 0.0] for k in range(n)]
    return {
        "scenario": "block", "description": "Block, mu = 0.5, 30.0, deg", "class": "A",
        "grounds": [{"surface": "data/grounds/slab/surface.obj", "pose": [0, 0, 0.1234567891, 1, 0, 0, 0]}],
        "t": t, "bodies": [{"name": "b", "surface": "data/bodies/box/surface.obj", "poses": poses}],
        "series": {"v": v, "v_theory": [x * 0.5 for x in v]},
        "error": 0.0123456789, "scalars": {"v_end": 2.64484123, "err_v": 0.0123456789}, **extra,
    }


def write(root, scenario, variant, name, data):
    path = root / scenario / variant / f"{name}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def make(tmp_path, **kw):
    traj = tmp_path / "results" / "traj"
    write(traj, "block", "base", "drake-gonthier", run())
    write(traj, "block", "base", "chrono-native", run(spike_at=500))
    write(traj, "block", "dt0.5", "drake-gonthier", run())
    return traj


def load(path):
    return json.loads(path.read_text())


def test_thins_to_the_frame_budget_keeping_ends_and_extremes(tmp_path):
    traj = make(tmp_path)
    out = tmp_path / "slim"
    export.export(traj, out, frames=300)
    d = load(out / "block/base/drake-gonthier.json")
    assert len(d["t"]) <= 300
    assert d["t"][0] == 0 and d["t"][-1] == 10
    assert len(d["bodies"][0]["poses"]) == len(d["series"]["v"]) == len(d["series"]["v_theory"]) == len(d["t"])
    assert max(d["series"]["v"]) == 7  # the spike survives
    assert d["t"] == sorted(d["t"])


def test_short_runs_are_kept_whole(tmp_path):
    traj = tmp_path / "traj"
    write(traj, "block", "base", "drake-gonthier", run(n=120, spike_at=7))
    export.export(traj, tmp_path / "slim", results=tmp_path)
    assert len(load(tmp_path / "slim/block/base/drake-gonthier.json")["t"]) == 120


def test_rounds_and_keeps_other_fields(tmp_path):
    traj = make(tmp_path)
    out = tmp_path / "slim"
    export.export(traj, out)
    d = load(out / "block/base/drake-gonthier.json")
    pose = d["bodies"][0]["poses"][1]
    assert pose[0] == round(pose[0], 6) and pose[2] == 0 and pose[3] == 1 and pose[4] == 0
    assert d["grounds"][0]["pose"][2] == 0.123457
    assert d["error"] == 0.012346 and d["scalars"]["v_end"] == 2.6448
    assert d["description"] == "Block, mu = 0.5, 30.0, deg" and d["class"] == "A"
    assert d["bodies"][0]["surface"] == "data/bodies/box/surface.obj"


def test_only_the_requested_variants_and_smaller_files(tmp_path):
    traj = make(tmp_path)
    out = tmp_path / "slim"
    export.export(traj, out)
    assert not (out / "block/dt0.5").exists()
    assert (out / "block/base/chrono-native.json").stat().st_size < (traj / "block/base/chrono-native.json").stat().st_size / 2
    export.export(traj, out, variants=("base", "dt0.5"))
    assert (out / "block/dt0.5/drake-gonthier.json").exists()


def test_deterministic_bytes(tmp_path):
    traj = make(tmp_path)
    export.export(traj, tmp_path / "a")
    export.export(traj, tmp_path / "b")
    files = sorted(p.relative_to(tmp_path / "a") for p in (tmp_path / "a").rglob("*.json"))
    assert files and all((tmp_path / "a" / f).read_bytes() == (tmp_path / "b" / f).read_bytes() for f in files)


def test_non_finite_runs_are_left_out_and_the_index_says_why(tmp_path):
    traj = make(tmp_path)
    bad = run()
    bad["bodies"][0]["poses"][5][0] = float("nan")
    write(traj, "block", "base", "mujoco-native", bad)
    (tmp_path / "results" / "battery.csv").write_text(
        "engine,contact,scenario,variant,status,error,realtime_ratio,wall_s\n"
        "drake,gonthier,block,base,ok,0.01,0.01,0.5\n"
        "chrono,native,block,base,ok,0.02,0.02,0.7\n"
        "mujoco,native,block,base,nan,nan,0.03,0.9\n"
        "drake,native,block,base,timeout,,,\n"
    )
    out = tmp_path / "slim"
    written, skipped = export.export(traj, out)
    assert written == 2 and skipped == ["block/base/mujoco-native"]
    assert not (out / "block/base/mujoco-native.json").exists()
    runs = {r["id"]: r for r in load(out / "index.json")["scenarios"]["block"]["variants"]["base"]["runs"]}
    assert runs["mujoco-native"]["status"] == "nan"
    assert runs["drake-native"]["status"] == "timeout"  # no file at all, still listed
    assert runs["drake-gonthier"]["status"] == "ok" and runs["drake-gonthier"]["wall_s"] == 0.5
    assert runs["drake-gonthier"]["speed"] == 100 and runs["drake-gonthier"]["speed_source"] == "battery"
    assert "speed" not in runs["drake-native"]
