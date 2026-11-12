# /// script
# requires-python = ">=3.10"
# ///
"""Slim, committable copy of the trajectories for the published site.

    uv run tools/viewer/export_site_data.py [--traj results/traj] [--results results]
                                            [--out site/data/traj] [--frames 300] [--variants base]

results/traj holds every run of the matrix (hundreds of MB, not versioned). The site needs far
less: the base variant of every scenario, every engine and method, thinned in time and rounded.
For each <scenario>/<variant>/<run>.json of the input the output has the same file with

- at most --frames samples (default 300): the first and last, the sample of the extreme (minimum
  and maximum) of every series, and the rest evenly spaced, so a spike of a plot survives;
- positions rounded to a micrometre, quaternions to 1e-5, series and scalars to 5 significant
  digits (the viewer interpolates between samples, so this is far below what can be seen);
- compact JSON, no whitespace (it compresses well: GitHub Pages serves it gzipped).

Runs whose trajectory holds a non-finite number are left out (the viewer shows the status the
battery recorded for them). An index.json (tools/viewer/index.py) is written next to the runs,
with the speeds and statuses of the CSVs in --results. The output is a pure function of the
input: running it twice gives identical bytes.
"""

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import index as index_tool

POSITION_DECIMALS = 6
QUATERNION_DECIMALS = 5
SIGNIFICANT = 5


def tidy(x):
    """An integral float as an int: "1" is shorter than "1.0" (and -0.0 becomes 0)."""
    return int(x) if isinstance(x, float) and x == int(x) and abs(x) < 1e15 else x


def sig(x, digits=SIGNIFICANT):
    """x rounded to `digits` significant digits; None (and anything that is not a float) stays."""
    if not isinstance(x, float):
        return x
    return tidy(0 if x == 0 else float(f"{x:.{digits - 1}e}"))


def fixed(x, decimals):
    return tidy(round(x, decimals))


def finite(values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def is_finite_run(run):
    if not run.get("t") or not finite(run["t"]):
        return False
    return all(finite(x for pose in b["poses"] for x in pose) for b in run.get("bodies", []))


def extremes(values):
    """Indices of the minimum and maximum of the finite entries (first of equals)."""
    pairs = [(v, i) for i, v in enumerate(values) if isinstance(v, (int, float)) and math.isfinite(v)]
    if not pairs:
        return set()
    return {min(pairs, key=lambda p: (p[0], p[1]))[1], max(pairs, key=lambda p: (p[0], -p[1]))[1]}


def keep_indices(run, frames):
    """Sorted sample indices to keep: at most `frames`, the ends and the series extremes included."""
    n = len(run["t"])
    if n <= frames:
        return list(range(n))
    must = {0, n - 1}
    for values in run.get("series", {}).values():
        if len(values) == n:
            must |= extremes(values)
    # The extremes are capped so that they cannot crowd out the even spacing.
    if len(must) > frames // 3:
        must = {0, n - 1}
    budget = frames - len(must)
    even = {round(k * (n - 1) / (budget - 1)) for k in range(budget)} if budget > 1 else set()
    return sorted(must | even)[:frames] if len(must | even) > frames else sorted(must | even)


def thin(run, frames):
    keep = keep_indices(run, frames)
    out = {k: v for k, v in run.items() if k not in ("t", "bodies", "series", "scalars", "error", "grounds")}
    out["grounds"] = [
        {**g, "pose": [fixed(x, POSITION_DECIMALS) for x in g["pose"]]} if "pose" in g else g
        for g in run.get("grounds", [])
    ]
    out["t"] = [sig(float(run["t"][i]), 7) for i in keep]
    out["bodies"] = [
        {
            **{k: v for k, v in b.items() if k != "poses"},
            "poses": [
                [fixed(p, POSITION_DECIMALS) for p in b["poses"][i][:3]]
                + [fixed(q, QUATERNION_DECIMALS) for q in b["poses"][i][3:]]
                for i in keep
            ],
        }
        for b in run["bodies"]
    ]
    n = len(run["t"])
    out["series"] = {
        name: [sig(values[i]) for i in keep] if len(values) == n else [sig(v) for v in values]
        for name, values in run.get("series", {}).items()
    }
    out["error"] = sig(run.get("error"))
    out["scalars"] = {k: sig(v) for k, v in run.get("scalars", {}).items()}
    return out


def dump(run):
    return json.dumps(run, separators=(",", ":"), allow_nan=False)


def export(traj, out, results=None, frames=300, variants=("base",)):
    """Writes the slim copy of the `variants` of traj into out; returns (runs written, skipped)."""
    traj, out = Path(traj), Path(out)
    if out.exists():
        shutil.rmtree(out)
    written, skipped = 0, []
    for scenario in sorted(p for p in traj.iterdir() if p.is_dir()):
        for variant in variants:
            for src in sorted((scenario / variant).glob("*.json")):
                run = json.loads(src.read_text())
                if not is_finite_run(run):
                    skipped.append(f"{scenario.name}/{variant}/{src.stem}")
                    continue
                dst = out / scenario.name / variant / src.name
                dst.parent.mkdir(parents=True, exist_ok=True)
                dst.write_text(dump(thin(run, frames)))
                written += 1
    index_tool.write(out, results or traj.parent, compact=True)
    return written, skipped


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--traj", default="results/traj")
    ap.add_argument("--results", default=None, help="folder of battery.csv and timing.csv (default: above --traj)")
    ap.add_argument("--out", default="site/data/traj")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--variants", default="base", help="comma list (default: base)")
    args = ap.parse_args(argv)
    written, skipped = export(args.traj, args.out, args.results, args.frames, tuple(args.variants.split(",")))
    size = sum(f.stat().st_size for f in Path(args.out).rglob("*") if f.is_file())
    print(f"{args.out}: {written} runs, {size / 1e6:.1f} MB, {len(skipped)} left out (non-finite)")
    for s in skipped:
        print("  skipped", s)


if __name__ == "__main__":
    main()
