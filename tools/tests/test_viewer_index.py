"""The viewer's index (tools/viewer/index.py) of a synthetic results/traj."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))
import index


def write_run(root, scenario, variant, run, **fields):
    path = root / scenario / variant / f"{run}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"scenario": scenario, "bodies": [], "grounds": [], **fields}))


def build(tmp_path, timing="", battery=""):
    root = tmp_path / "traj"
    header = "engine,contact,scenario,variant,error,realtime_ratio\n"
    if timing:
        (tmp_path / "timing.csv").write_text(header + timing)
    if battery:
        (tmp_path / "battery.csv").write_text(header + battery)
    index.main([str(root)])
    return json.loads((root / "index.json").read_text())["scenarios"]


def test_lists_runs_with_engine_and_hyphenated_contact(tmp_path):
    write_run(tmp_path / "traj", "disk", "base", "drake-gonthier")
    write_run(tmp_path / "traj", "disk", "base", "mujoco-native-point")
    runs = build(tmp_path)["disk"]["variants"]["base"]["runs"]
    assert [(r["id"], r["engine"], r["contact"]) for r in runs] == [
        ("drake-gonthier", "drake", "gonthier"),
        ("mujoco-native-point", "mujoco", "native-point"),
    ]


def test_speed_prefers_timing_and_falls_back_to_battery(tmp_path):
    for variant in ("base", "fine"):
        write_run(tmp_path / "traj", "disk", variant, "drake-gonthier")
    write_run(tmp_path / "traj", "disk", "base", "chrono-native")
    variants = build(
        tmp_path,
        timing="drake,gonthier,disk,base,0.1,0.005\n",
        battery="drake,gonthier,disk,base,0.1,0.5\ndrake,gonthier,disk,fine,0.1,4\nchrono,native,disk,base,0.1,nan\n",
    )["disk"]["variants"]
    by_id = {r["id"]: r for r in variants["base"]["runs"]}
    assert (by_id["drake-gonthier"]["speed"], by_id["drake-gonthier"]["speed_source"]) == (200, "timing")
    assert "speed" not in by_id["chrono-native"]
    fine = variants["fine"]["runs"][0]
    assert (fine["speed"], fine["speed_source"]) == (0.25, "battery")


def test_scenario_and_variant_fields_come_from_the_runs(tmp_path):
    body = [{"surface": "data/bodies/cylinder_r250_h50_seg128/surface.obj"}]
    ground = [{"surface": "data/grounds/slab_40000x40000x1000/surface.obj"}]
    write_run(tmp_path / "traj", "disk", "fine", "drake-gonthier", description="Disk", bodies=body, grounds=ground, dt=0.001, **{"class": "B"})
    write_run(tmp_path / "traj", "block", "base", "drake-gonthier", grounds=ground)
    scenarios = build(tmp_path)
    assert (scenarios["disk"]["description"], scenarios["disk"]["class"]) == ("Disk", "B")
    assert scenarios["disk"]["variants"]["fine"]["segments"] == 128
    assert scenarios["disk"]["variants"]["fine"]["dt"] == 0.001
    assert scenarios["block"]["variants"]["base"]["segments"] is None
    assert scenarios["block"]["class"] is None


def test_status_wall_time_and_runs_without_a_trajectory(tmp_path):
    write_run(tmp_path / "traj", "disk", "base", "drake-gonthier")
    (tmp_path / "battery.csv").write_text(
        "engine,contact,scenario,variant,status,error,realtime_ratio,wall_s\n"
        "drake,gonthier,disk,base,ok,0.1,0.02,1.5\n"
        "drake,native,disk,base,timeout,,,\n"
        "drake,gonthier,disk,mesh32,ok,0.1,0.02,1.5\n"  # a variant with no folder is not listed
    )
    scenarios = build(tmp_path)
    assert list(scenarios["disk"]["variants"]) == ["base"]
    runs = {r["id"]: r for r in scenarios["disk"]["variants"]["base"]["runs"]}
    assert (runs["drake-gonthier"]["status"], runs["drake-gonthier"]["wall_s"]) == ("ok", 1.5)
    assert runs["drake-native"] == {"id": "drake-native", "engine": "drake", "contact": "native", "status": "timeout", "file": False}


def test_speed_of_a_timing_pass_is_the_median_of_its_repeats(tmp_path):
    write_run(tmp_path / "traj", "disk", "base", "drake-gonthier")
    rows = "".join(f"drake,gonthier,disk,base,0.1,{r}\n" for r in (0.01, 0.02, 0.5))
    run = build(tmp_path, timing=rows)["disk"]["variants"]["base"]["runs"][0]
    assert (run["speed"], run["speed_source"]) == (50, "timing")
