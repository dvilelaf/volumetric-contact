# /// script
# requires-python = ">=3.10"
# ///
"""Index of the trajectories of a bench run, the only file the viewer needs to list them.

    uv run tools/viewer/index.py results/traj [--results results]

Writes <root>/index.json:

    {"version": 3, "scenarios": {<scenario>: {"description", "class", "variants": {<variant>:
        {"segments", "dt", "perturbed", "runs": [{"id", "engine", "contact", "status", "wall_s",
        "speed", "speed_source"}]}}}}}

A run is the file <scenario>/<variant>/<engine>-<contact>.json (the engine has no hyphen, the
contact method may: native-point). `speed` is simulated time over wall time: from timing.csv
(`speed_source` "timing", the idle serial pass, median of its repeats) or else battery.csv
("battery", the parallel accuracy pass, approximate). `status` (ok, nan, timeout, failed) and
`wall_s` come from battery.csv; a row of the CSV whose run left no trajectory (a timeout, a
failure) is listed too, without a file, so that the viewer can say what happened to it. Both
tables are looked for in --results, by default the folder above <root>. Description, class, time
step and perturbation come from the first run of each variant when the bench wrote them;
`segments` from the names of its curved meshes.
"""

import argparse
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path

KEY = ("scenario", "variant", "engine", "contact")


def read_rows(path):
    if not path.is_file():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def number(text):
    try:
        x = float(text)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def speeds(path):
    """{(scenario, variant, engine, contact): simulated time / wall time} of a battery or timing CSV
    (the median when a key has several rows, as the repeats of a timing pass)."""
    ratios = {}
    for row in read_rows(path):
        ratio = number(row.get("realtime_ratio"))
        if ratio and ratio > 0:  # also drops nan
            ratios.setdefault(tuple(row[k] for k in KEY), []).append(ratio)
    return {key: 1 / statistics.median(r) for key, r in ratios.items()}


def outcomes(path):
    """{key: {"status", "wall_s"}} of a battery CSV; a CSV without status columns is all ok."""
    table = {}
    for row in read_rows(path):
        entry = {"status": row.get("status") or "ok"}
        wall = number(row.get("wall_s"))
        if wall is not None:
            entry["wall_s"] = float(f"{wall:.4g}")
        table[tuple(row[k] for k in KEY)] = entry
    return table


def first_run(files):
    for f in files:
        try:
            return json.loads(f.read_text())
        except (OSError, ValueError):
            continue
    return {}


def segments(run):
    """Segments per turn of the curved meshes of a run; None without them or when they differ."""
    surfaces = [x.get("surface", "") for x in run.get("bodies", []) + run.get("grounds", [])]
    found = {int(n) for s in surfaces for n in re.findall(r"_seg(\d+)", s)}
    return found.pop() if len(found) == 1 else None


def build(root, results=None):
    results = Path(results) if results else root.parent
    timing = speeds(results / "timing.csv")
    battery = speeds(results / "battery.csv")
    done = outcomes(results / "battery.csv")
    scenarios = {}
    for scenario in sorted(p for p in root.iterdir() if p.is_dir()):
        entry = {"variants": {}}
        for variant in sorted(p for p in scenario.iterdir() if p.is_dir()):
            files = sorted(variant.glob("*.json"))
            if not files:
                continue
            sample = first_run(files)
            entry.setdefault("description", sample.get("description"))
            entry.setdefault("class", sample.get("class"))
            stems = {f.stem for f in files}
            ids = set(stems)
            # Runs the battery recorded without a trajectory (timeout, failure) are listed as well.
            for key in done:
                if key[:2] == (scenario.name, variant.name):
                    ids.add(f"{key[2]}-{key[3]}")
            runs = []
            for rid in sorted(ids):
                engine, _, contact = rid.partition("-")
                run = {"id": rid, "engine": engine, "contact": contact}
                key = (scenario.name, variant.name, engine, contact)
                if key in done:
                    run.update(done[key])
                if rid not in stems:
                    # No file: a timeout or failure, or a trajectory with non-finite numbers.
                    run["file"] = False
                    if run.get("status", "ok") == "ok":
                        run["status"] = "nan"
                for source, table in (("timing", timing), ("battery", battery)):
                    if key in table:
                        run["speed"] = float(f"{table[key]:.4g}")
                        run["speed_source"] = source
                        break
                runs.append(run)
            entry["variants"][variant.name] = {
                "segments": segments(sample),
                "dt": sample.get("dt"),
                "perturbed": sample.get("perturbed"),
                "runs": runs,
            }
        if entry["variants"]:
            scenarios[scenario.name] = entry
    return {"version": 3, "scenarios": scenarios}


def write(root, results=None, compact=False):
    index = build(root, results)
    text = json.dumps(index, separators=(",", ":")) if compact else json.dumps(index, indent=1) + "\n"
    (root / "index.json").write_text(text)
    return index


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("root", nargs="?", default="results/traj")
    ap.add_argument("--results", help="folder of battery.csv and timing.csv (default: above root)")
    args = ap.parse_args(argv)
    root = Path(args.root)
    index = write(root, args.results)
    print(f"{root}/index.json: {len(index['scenarios'])} scenarios")


if __name__ == "__main__":
    main(sys.argv[1:])
