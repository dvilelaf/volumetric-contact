# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "matplotlib"]
# ///
"""Numbers, tables and figures of the paper, generated from the battery (paper/README.md).

Reads results/battery.csv, results/terms.csv, results/stats.json (tools/stats.py: the bootstrap
intervals, the robust comparisons and the convergence flags), results/ablation_triangulation.csv
(tools/ablation_triangulation.py), paper/provenance.json and, when it exists, results/timing.csv
(the timing pass on an idle machine; without it the costs are the wall times of the accuracy pass,
measured under load and labelled as indicative). Writes

    paper/generated/numbers.tex          \\setresult{key}{value}, only the keys the paper uses
    paper/generated/tables/<name>.tex    complete tabular environments
    paper/generated/figures/<name>.pdf   vector figures

The output is a function of the inputs and of the \\result{} keys found in paper/main.tex,
paper/supplement.tex and paper/sections/*.tex: running it twice gives the same bytes. A key that the paper uses and this
script does not define is an error, so the paper cannot cite a number that does not exist.

Conventions. A law (gonthier, distributed, ...) gives the same error on the three engines, so
tables and figures show its Chrono runs; keys accept `<engine>-<law>` for the others. Native models
have short names: hydro, dpoint (Drake), smc, nsc (Chrono), soft, mjdefault (MuJoCo). Errors are
printed in percent; an error of 1000 % or more is printed as such (the run was lost), and a run that
did not finish prints its status, never a number.

usage: uv run tools/paper_results.py [--results DIR] [--paper DIR] [--no-figures] [--list-keys]
"""

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path

FLOOR = 1e-3  # as tools/stats.py: errors below it are a practical tie
LOST = 10.0  # an error of 1000 % or more: the run was lost
THRESHOLDS = (0.01, 0.05, 0.20)
ENGINES = ("chrono", "drake", "mujoco")
LAW_ENGINE = "chrono"
LAWS = {  # paper key -> contact label of the bench
    "gonthier": "gonthier",
    "thesis": "gonthier-thesis",
    "distributed": "distributed",
    "vertex": "distributed-vertex",
    "centroid": "distributed-centroid",
    "pcm": "pcm",
}
NATIVES = {  # paper key -> method of battery.csv
    "hydro": "drake-native",
    "dpoint": "drake-native-point",
    "smc": "chrono-native",
    "nsc": "chrono-native-nsc",
    "soft": "mujoco-native",
    "mjdefault": "mujoco-native-default",
}
NAMES = {
    "gonthier": "Gonthier",
    "thesis": "Gonthier, 2018 variant",
    "distributed": "Distributed",
    "vertex": "Distributed, vertex rule",
    "centroid": "Distributed, one-point rule",
    "pcm": "PCM",
    "hydro": "Drake hydroelastic",
    "dpoint": "Drake point",
    "smc": "Chrono SMC",
    "nsc": "Chrono NSC",
    "soft": "MuJoCo soft",
    "mjdefault": "MuJoCo default",
}
SHORT = {
    "gonthier": "Gon",
    "thesis": "G18",
    "distributed": "Dis",
    "vertex": "Dis-v",
    "centroid": "Dis-1",
    "pcm": "PCM",
    "hydro": "Dr-h",
    "dpoint": "Dr-p",
    "smc": "Ch-S",
    "nsc": "Ch-N",
    "soft": "Mj-s",
    "mjdefault": "Mj-d",
}
MAIN = ["gonthier", "thesis", "distributed", "pcm", *NATIVES]  # columns of the error tables
WINKLER = ["gonthier", "thesis", "distributed", "pcm", "hydro"]  # class C: laws and Drake hydroelastic
ALL = [*LAWS, *NATIVES]

# Order of docs/scenarios.md §4.2 and §4.3 (Appendix B of the paper).
DEV = {
    "A": ["block", "cylinder", "push", "billiard", "vgroove", "bowl", "trough", "twoblocks", "sphere", "journal"],
    "B": ["disk", "contensou", "ring", "bearing", "cone"],
    "C": ["tip", "drop", "hole", "corner", "offset", "stack"],
}
HELDOUT = [
    "pieces1", "pieces2", "pieces8", "pieces64", "wedge", "restitution", "oblique", "edge",
    "turntable", "energy", "tower", "massratio",
]  # fmt: skip
# Class B: band of admissible pressure distributions as factors on the nominal stop time
# (docs/scenarios.md §4.2; design parameters, also in Appendix B of the paper).
BANDS = {
    "disk": (0.85, 1.33),
    "contensou": (0.95, 1.12),
    "ring": (1.0, 1.02),
    "bearing": (0.81, 1.27),
    "cone": (1.0, 1.04),
}
# As tools/stats.py: class A scenarios whose rigid reference no damped compliant model can meet at
# the chosen h and K; "Ar" is class A without them.
PARAMETER_BOUND = ("bowl", "trough", "turntable", "edge")
PRIMARY_NATIVES = ("hydro", "smc", "soft")  # the default native model of each engine
PRIMARY = [("distributed", "gonthier"), *[(law, n) for n in PRIMARY_NATIVES for law in ("distributed", "gonthier")]]
DT_SERIES = (("dt2", 2.0), ("base", 1.0), ("dt0.5", 0.5), ("dt0.2", 0.2), ("dt0.1", 0.1))
MESH_SERIES = (("mesh32", 32), ("base", 64), ("mesh128", 128))
# Groups of scenarios that the text discusses together (Section 7.1 of the paper).
GROUPS = {"oneplane": ["block", "cylinder", "push", "twoblocks", "sphere", "vgroove", "billiard"]}
FLAGS = ("converging", "flat", "non-monotone", "diverging", "failed")


def method(key):
    """Paper key of a method -> method of battery.csv (engine-contact)."""
    if key in NATIVES:
        return NATIVES[key]
    if key in LAWS:
        return f"{LAW_ENGINE}-{LAWS[key]}"
    engine, _, law = key.partition("-")
    if engine in ENGINES and law in LAWS:
        return f"{engine}-{LAWS[law]}"
    raise KeyError(key)


# ---- formatting ----------------------------------------------------------------------------------


def sig(x, digits=2):
    """x with `digits` significant figures, without exponent."""
    if x == 0:
        return "0"
    decimals = max(0, digits - 1 - math.floor(math.log10(abs(x))))
    return f"{round(x, decimals):.{decimals}f}"


def pct(e):
    """An error (fraction of the reference) as a percentage, without the sign."""
    if e >= LOST:
        return "$>$1000"
    p = 100 * e
    if p < 0.01:
        return "$<$0.01"
    if p >= 9.95:
        return f"{p:.0f}"
    return f"{p:.1f}" if p >= 0.995 else f"{p:.2f}"


def tri_pct(x, signed=True):
    """An error of the triangulation experiment in percent, with its sign."""
    p = 100 * x
    if abs(p) < 0.005:
        return "$<$0.01\\,\\%"
    digits = 0 if abs(p) >= 9.95 else (1 if abs(p) >= 0.995 else 2)
    text = f"{abs(p):.{digits}f}\\,\\%"
    return ("$-$" if p < 0 else "$+$") + text if signed else text


def sci(x):
    """x as m x 10^e in TeX, two significant figures."""
    e = math.floor(math.log10(abs(x)))
    m = round(x / 10**e, 1)
    if m >= 10:
        m, e = m / 10, e + 1
    mant = f"{m:.1f}".rstrip("0").rstrip(".")
    return f"$10^{{{e}}}$" if mant == "1" else f"${mant}\\times10^{{{e}}}$"


def scn(name):
    return f"\\scn{{{name}}}"


# ---- data ----------------------------------------------------------------------------------------


class Battery:
    def __init__(self, results, paper):
        results, paper = Path(results), Path(paper)
        raw = (results / "battery.csv").read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        rows = {}
        for r in csv.DictReader(raw.decode().splitlines()):
            rows[(r["scenario"], f"{r['engine']}-{r['contact']}", r["variant"])] = r  # the last of duplicates wins
        self.status, self.err, self.cost, self.cls = {}, {}, {}, {}
        self.wall_s, self.duration = {}, {}
        for key, r in rows.items():
            ok = r["status"] == "ok" and math.isfinite(float(r["error"]))
            self.status[key] = r["status"] if not ok and r["status"] != "ok" else ("ok" if ok else "nan")
            self.err[key] = float(r["error"]) if ok else math.inf
            self.cost[key] = float(r["realtime_ratio"]) if r.get("realtime_ratio") else math.nan
            self.cls[r["scenario"]] = r["class"]
            self.wall_s[key] = float(r["wall_s"]) if r.get("wall_s") else math.nan
            if ok and key[2] == "base" and float(r["steps"]) > 0:
                self.duration[r["scenario"]] = float(r["steps"]) * float(r["dt"])
        self.n_rows = len(rows)
        self.terms = defaultdict(dict)
        terms = results / "terms.csv"
        if terms.exists():
            with open(terms, newline="") as f:
                for r in csv.DictReader(f):
                    try:
                        value = float(r["value"])
                    except ValueError:
                        value = math.nan
                    self.terms[(r["scenario"], f"{r['engine']}-{r['contact']}", r["variant"])][r["term"]] = value
        self.stats = json.loads((results / "stats.json").read_text()) if (results / "stats.json").exists() else {}
        self.triangulation = []
        if (results / "ablation_triangulation.csv").exists():
            with open(results / "ablation_triangulation.csv", newline="") as f:
                self.triangulation = list(csv.DictReader(f))
        self.provenance = json.loads((paper / "provenance.json").read_text())
        # Costs: the timing pass if it exists (median of its repeats), else the accuracy pass.
        self.timing = {}
        timing = results / "timing.csv"
        if timing.exists():
            repeats = defaultdict(list)
            with open(timing, newline="") as f:
                for r in csv.DictReader(f):
                    if r.get("realtime_ratio"):  # a run without an error value still has a cost
                        repeats[(r["scenario"], f"{r['engine']}-{r['contact']}", r["variant"])].append(
                            float(r["realtime_ratio"])
                        )
            self.timing = {k: statistics.median(v) for k, v in repeats.items()}
        dev = [s for c in "ABC" for s in DEV[c]]
        known = dev + HELDOUT
        self.scenarios = [s for s in known if s in self.cls] + sorted(s for s in self.cls if s not in known)
        self.heldout = [s for s in self.scenarios if s in HELDOUT]
        self.dev = [s for s in self.scenarios if s not in HELDOUT]

    def of_class(self, cls, pool=None):
        """Scenarios of the classes named in cls ("A", "AB", ...); "Ar" is class A without PARAMETER_BOUND."""
        if cls == "Ar":
            return [s for s in self.of_class("A", pool) if s not in PARAMETER_BOUND]
        return [s for s in (pool or self.scenarios) if self.cls[s] in cls]

    def perturbed(self, scenario, key):
        """The eight perturbed runs: (sorted errors of those that have one, runs without an error value)."""
        errs = [self.e(scenario, key, f"perturbed{k}") for k in range(1, 9)]
        errs = [e for e in errs if e is not None]
        finite = sorted(e for e in errs if math.isfinite(e))
        return finite, len(errs) - len(finite)

    def e(self, scenario, key, variant="base"):
        return self.err.get((scenario, method(key), variant))

    def st(self, scenario, key, variant="base"):
        return self.status.get((scenario, method(key), variant))

    def wall(self, scenario, key, variant="base"):
        """Wall time over simulated time."""
        k = (scenario, method(key), variant)
        return self.timing.get(k, math.nan) if self.timing else self.cost.get(k, math.nan)

    def wall_bound(self, scenario, key):
        """Lower bound of wall over simulated time of a base run stopped by its time limit in the
        accuracy pass: the wall time it used over the whole duration of the scenario."""
        k = (scenario, method(key), "base")
        if self.status.get(k) != "timeout" or scenario not in self.duration:
            return math.nan
        return self.wall_s[k] / self.duration[scenario]

    def cell(self, scenario, key, variant="base"):
        """Table cell: the error in percent, or the status of a run that did not finish."""
        e = self.e(scenario, key, variant)
        if e is None:
            return "--"
        if not math.isfinite(e):
            return f"\\textit{{{self.st(scenario, key, variant)}}}"
        return pct(e)

    def series(self, scenario, key, kind):
        pts = []
        for variant, h in DT_SERIES if kind == "dt" else MESH_SERIES:
            e = self.e(scenario, key, variant)
            if e is not None:
                pts.append((h, e))
        return pts


def eff(e):
    return max(e, FLOOR)


def pass_rate(b, key, cls, threshold, pool=None):
    errs = [b.e(s, key) for s in b.of_class(cls, pool)]
    errs = [e for e in errs if e is not None]
    return sum(e <= threshold for e in errs), len(errs)


def independence(b, law, variant="base"):
    """Per scenario: (absolute, relative) spread of a law's error over the three engines."""
    out = {}
    for s in b.scenarios:
        errs = [b.err.get((s, f"{eng}-{LAWS[law]}", variant)) for eng in ENGINES]
        if any(e is None for e in errs):
            continue
        if any(not math.isfinite(e) for e in errs):
            out[s] = (math.inf, math.inf, max(errs))
        else:
            spread = max(errs) - min(errs)
            out[s] = (spread, spread / max(max(errs), FLOOR), max(errs))
    return out


def paired(b, a, c, subset):
    """Row of stats.json for the ratio of the errors of a over c (paper keys; own engine for natives)."""
    ma, mc = method(a), method(c)
    if c in NATIVES and a in LAWS:
        ma = f"{mc.split('-')[0]}-{LAWS[a]}"
    for row in b.stats.get("paired", []):
        if row["a"] == ma and row["b"] == mc and row["subset"] == subset:
            return row
    return None


def robust(b, a, c, subset):
    """Row of stats.json for the robust ratio of the errors of a over c (capped, failures counted)."""
    ma, mc = method(a), method(c)
    if c in NATIVES and a in LAWS:
        ma = f"{mc.split('-')[0]}-{LAWS[a]}"
    for row in b.stats.get("robust", []):
        if row["a"] == ma and row["b"] == mc and row["subset"] == subset:
            return row
    return None


def pvalue(p):
    return "$<$0.001" if p < 0.001 else (f"{p:.3f}" if p < 0.1 else f"{p:.2f}")


def interval(row):
    return f"{sig(row['ratio'])} ({sig(10 ** row['ci'][0])}--{sig(10 ** row['ci'][1])})"


def band_status(b, scenario, key):
    """Where a class B run lies against the band of admissible distributions, as far as an error
    without sign can tell: inside, outside, undetermined, or none (no stop time)."""
    e = b.e(scenario, key)
    if e is None or not math.isfinite(e):
        return "none"
    lo, hi = BANDS[scenario]
    narrow, wide = sorted((1 - lo, hi - 1))
    return "outside" if e > wide else ("undetermined" if e > narrow else "inside")


def scattered(b, keys, classes="AB", factor=2.0, floor=0.01):
    """(scenario, key) whose perturbed runs spread by more than `factor` (largest over smallest
    error, both at least `floor`) or lose a run."""
    out = []
    for s in b.of_class(classes):
        for k in keys:
            finite, lost = b.perturbed(s, k)
            if finite and (lost or max(finite[-1], floor) / max(finite[0], floor) > factor):
                out.append((s, k))
    return out


def convergence(b, kind, scenario, key):
    return b.stats.get("convergence", {}).get(f"{kind}/{scenario}/{method(key)}")


def perturbation_spread(b, key):
    """Median over scenarios of (max - min) / median error of the eight perturbed runs."""
    spreads = []
    for s in b.scenarios:
        errs = [b.e(s, key, f"perturbed{k}") for k in range(1, 9)]
        errs = [e for e in errs if e is not None and math.isfinite(e)]
        if errs:
            spreads.append((max(errs) - min(errs)) / max(statistics.median(errs), FLOOR))
    return statistics.median(spreads) if spreads else math.nan


def profile(b, keys, classes=("A", "B")):
    """key -> sorted ratios of its floored error to the best of `keys`, per scenario (inf: failed)."""
    ratios = {k: [] for k in keys}
    for s in b.of_class(classes):
        col = [eff(b.e(s, k)) if b.e(s, k) is not None else math.inf for k in keys]
        best = min(col)
        if math.isfinite(best):
            for k, e in zip(keys, col):
                ratios[k].append(e / best)
    return {k: sorted(v) for k, v in ratios.items()}


# ---- numbers -------------------------------------------------------------------------------------


class Numbers:
    """Lazy dictionary of \\result keys: resolve(key) returns the TeX value or raises KeyError."""

    def __init__(self, b):
        self.b = b

    def resolve(self, key):
        head, _, rest = key.partition("/")
        handler = getattr(self, "_" + head.replace("-", "_"), None)
        if handler is None:
            raise KeyError(key)
        value = handler(*rest.split("/")) if rest else handler()
        if value is None:
            raise KeyError(key)
        return str(value)

    # battery/...
    def _battery(self, what, arg=None):
        b, p = self.b, self.b.provenance
        counts = Counter(b.status.values())
        simple = {
            "commit": lambda: f"\\code{{{p['commit']}}}",
            "commit-baselines": lambda: f"\\code{{{p['commit_baselines']}}}",
            "timing-commit": lambda: f"\\code{{{p['timing']['commit']}}}",
            "timing-date": lambda: p["timing"]["date"],
            "timing-machine": lambda: p["timing"]["machine"],
            "timing-repeats": lambda: p["timing"]["repeats"],
            "date": lambda: p["date"],
            "spot-runs": lambda: p["spot_runs"],
            "spot-diff": lambda: sci(p["spot_max_relative_difference"]),
            "sha256": lambda: f"\\code{{{b.sha256[:12]}}}",
            "runs": lambda: f"{b.n_rows:,}".replace(",", "\\,"),
            "ok": lambda: f"{counts['ok']:,}".replace(",", "\\,"),
            "failed": lambda: b.n_rows - counts["ok"],
            "nan": lambda: counts["nan"],
            "timeout": lambda: counts["timeout"],
            "crashed": lambda: counts["failed"],
            "scenarios": lambda: len(b.scenarios),
            "heldout": lambda: len(b.heldout),
            "dev": lambda: len(b.dev),
            "methods": lambda: len({m for _, m, _ in b.status}),
            "variants": lambda: len({v for _, _, v in b.status}),
            # run time of the accuracy pass: the wall-clock seconds of every run, added up
            "cpu-hours": lambda: f"{sum(w for w in b.wall_s.values() if math.isfinite(w)) / 3600:.0f}",
        }
        if what in simple:
            return simple[what]()
        if what == "n":  # battery/n/<class> or battery/n/<class>-dev, -heldout
            cls, _, part = arg.partition("-")
            pool = {"": None, "dev": b.dev, "heldout": b.heldout}[part]
            return len(b.of_class(cls, pool))
        return None

    # status/<method>/<status|notok>, status/laws/notok (the two laws, three engines)
    def _status(self, key, what):
        b = self.b
        if key == "laws":
            methods = {f"{e}-{LAWS[law]}" for e in ENGINES for law in ("gonthier", "distributed")}
        elif key == "natives":
            methods = set(NATIVES.values())
        elif key == "baselines":
            methods = {f"{e}-{LAWS[law]}" for e in ENGINES for law in ("thesis", "vertex", "centroid", "pcm")}
        else:
            methods = {method(key)}
        sts = [st for (_, m, _), st in b.status.items() if m in methods]
        return sum(st != "ok" for st in sts) if what == "notok" else sum(st == what for st in sts)

    # err/<scenario>/<method>[/<variant>]: with the unit
    def _err(self, scenario, key, variant="base"):
        e = self.b.e(scenario, key, variant)
        if e is None:
            return None
        if not math.isfinite(e):
            return f"\\textit{{{self.b.st(scenario, key, variant)}}}"
        return pct(e) + "\\,\\%"

    # term/<scenario>/<method>/<term>[/<variant>]
    def _term(self, scenario, key, term, variant="base"):
        value = self.b.terms.get((scenario, method(key), variant), {}).get(term)
        if value is None:
            return None
        return pct(value) + "\\,\\%" if math.isfinite(value) else "\\textit{nan}"

    # within/<group>/<method>[+<method>...]/<percent>: "5 of 7", the scenarios of the group in which every
    # method listed has an error at or below the threshold
    def _within(self, group, keys, percent):
        threshold = float(percent) / 100
        errs = [[self.b.e(s, k) for k in keys.split("+")] for s in GROUPS[group]]
        n = sum(all(e is not None and e <= threshold for e in row) for row in errs)
        return f"{n} of {len(errs)}"

    # pass/<class>/<method>/<1|5|20>[/dev|heldout]: "16 of 21"; passn/...: the count
    def _pass(self, cls, key, threshold, part=""):
        n, total = self._passes(cls, key, threshold, part)
        return f"{n} of {total}"

    def _passn(self, cls, key, threshold, part=""):
        return self._passes(cls, key, threshold, part)[0]

    def _passes(self, cls, key, threshold, part):
        pool = {"": None, "dev": self.b.dev, "heldout": self.b.heldout}[part]
        return pass_rate(self.b, key, cls.replace("+", ""), int(threshold) / 100, pool)

    # median/<class>/<method>: median error of the finished base runs
    def _median(self, cls, key):
        errs = [self.b.e(s, key) for s in self.b.of_class(cls.replace("+", ""))]
        errs = [e for e in errs if e is not None and math.isfinite(e)]
        return pct(statistics.median(errs)) + "\\,\\%" if errs else None

    # ratio/<a>-vs-<b>/<subset>/<gm|lo|hi|n|excl|wins|losses|full>
    def _ratio(self, pair, subset, what):
        a, _, c = pair.partition("-vs-")
        row = paired(self.b, a, c, subset)
        if row is None:
            return None
        gm, lo, hi = row["ratio"], 10 ** row["ci"][0], 10 ** row["ci"][1]
        return {
            "gm": sig(gm),
            "lo": sig(lo),
            "hi": sig(hi),
            "n": row["n"],
            "excl": row["excluded"],
            "wins": row["wins"],
            "losses": row["losses"],
            "full": f"{sig(gm)} ({sig(lo)}--{sig(hi)})",
        }.get(what)

    # robust/<a>-vs-<b>/<A|A+B|A-hK>/<gm|lo|hi|full|holm|p|wins|ties|losses|wtl|loo|loo-unit|n>
    def _robust(self, pair, subset, what):
        a, _, c = pair.partition("-vs-")
        row = robust(self.b, a, c, subset)
        if row is None:
            return None
        gm = row["ratio"]
        far = 0 if abs(math.log(row["loo"][0] / gm)) >= abs(math.log(row["loo"][1] / gm)) else 1
        return {
            "gm": sig(gm),
            "lo": sig(10 ** row["ci"][0]),
            "hi": sig(10 ** row["ci"][1]),
            "full": interval(row),
            "holm": pvalue(row["p_holm"]) if row["p_holm"] is not None else None,
            "p": pvalue(row["p_boot"]),
            "wins": row["wins"],
            "ties": row["ties"],
            "losses": row["losses"],
            "wtl": f"{row['wins']}/{row['ties']}/{row['losses']}",
            "loo": f"{sig(row['loo'][0])}--{sig(row['loo'][1])}",
            "loo-unit": scn(row["loo_units"][far]),
            "n": row["n"],
        }.get(what)

    # primary/<n|significant|excluding-one>[/<subset>]: the family of primary comparisons
    def _primary(self, what, subset=None):
        rows = [r for r in self.b.stats.get("robust", []) if r["primary"] and subset in (None, r["subset"])]
        if not rows:
            return None
        if what == "n":
            return len(rows)
        if what == "significant":  # Holm-adjusted p below 0.05
            return sum(r["p_holm"] < 0.05 for r in rows)
        if what == "excluding-one":  # the unadjusted 95 % interval excludes 1
            return sum(r["ci"][0] > 0 or r["ci"][1] < 0 for r in rows)
        return None

    # rules/<cap|tie|floor>: the rules of the robust comparisons
    def _rules(self, what):
        rules = self.b.stats.get("robust_rules")
        if not rules:
            return None
        return {
            "cap": f"{100 * rules['cap']:.0f}\\,\\%",
            "tie": f"{rules['tie']:g}",
            "floor": f"{100 * rules['floor']:g}\\,\\%",
        }.get(what)

    # pertrange/<scenario>/<method>/<min|median|max|range|lost>: the eight perturbed runs
    def _pertrange(self, scenario, key, what):
        finite, lost = self.b.perturbed(scenario, key)
        if what == "lost":
            return lost
        if not finite:
            return None
        return {
            "min": pct(finite[0]) + "\\,\\%",
            "median": pct(statistics.median(finite)) + "\\,\\%",
            "max": pct(finite[-1]) + "\\,\\%",
            "range": f"{pct(finite[0])}--{pct(finite[-1])}\\,\\%",
        }.get(what)

    # band/<method>/<inside|undetermined|outside|none>: class B scenarios by position against the band
    def _band(self, key, what):
        return sum(band_status(self.b, s, key) == what for s in self.b.of_class("B"))

    # tri/<shape>/<case>/<triangulation>/<parameter>/<rule>/<moment|force|triangles>, tri/closed-spread
    def _tri(self, shape, case=None, name=None, parameter=None, rule=None, what=None):
        rows = self.b.triangulation
        if not rows:
            return None
        if shape == "closed-spread":
            return sci(max(max(abs(float(r["moment_error"])), float(r["force_error"])) for r in rows if r["rule"] == "closed"))
        for r in rows:
            if (r["shape"], r["case"], r["triangulation"], r["parameter"], r["rule"]) == (shape, case, name, parameter, rule):
                if what == "triangles":
                    return r["triangles"]
                return tri_pct(float(r["moment_error" if what == "moment" else "force_error"]), signed=what == "moment")
        return None

    # ind/<law>/<maxabs|maxrel|median|worst-scenario>[/<variant>]: over the scenarios that every
    # engine holds (error below 100 %); independence/worst is the largest maxabs of the two laws.
    def _ind(self, law, what, variant="base"):
        held = {s: v for s, v in independence(self.b, law, variant).items() if v[2] < 1}
        if not held:
            return None
        if what == "maxabs":
            return pct(max(v[0] for v in held.values())) + " percentage points"
        if what == "maxrel":
            return pct(max(v[1] for v in held.values())) + "\\,\\%"
        if what == "median":
            return sci(max(statistics.median(v[1] for v in held.values()), 1e-12))
        if what == "worst-scenario":
            return scn(max(held, key=lambda s: held[s][0]))
        if what == "n":
            return len(held)
        return None

    def _independence(self, what):
        if what != "worst":
            return None
        worst = max(
            v[0] for law in ("gonthier", "distributed") for v in independence(self.b, law).values() if v[2] < 1
        )
        return pct(worst) + " percentage points"

    # conv/<dt|mesh>/<scenario>/<method>/<flag|slope>
    def _conv(self, kind, scenario, key, what):
        row = convergence(self.b, kind, scenario, key)
        if row is None:
            return None
        if what == "flag":
            return row["flag"]
        return f"{row['slope']:.1f}" if row["slope"] is not None and math.isfinite(row["slope"]) else "n/a"

    # convn/<dt|mesh>/<method>/<flag|n>: scenarios with that flag
    def _convn(self, kind, key, flag):
        rows = [convergence(self.b, kind, s, key) for s in self.b.scenarios]
        rows = [r for r in rows if r is not None]
        return len(rows) if flag == "n" else sum(r["flag"] == flag for r in rows)

    # pert/<method>: median relative spread over the eight perturbations
    def _pert(self, key):
        spread = perturbation_spread(self.b, key)
        return pct(spread) + "\\,\\%" if math.isfinite(spread) else None

    # stiffn/<method>/<better|worse|n>[/<classes>]: scenarios whose error at K x 10 is below / above
    # the one at K (both at 0.2 ms) by more than 10 %
    def _stiffn(self, key, what, classes="A"):
        better = worse = n = 0
        for s in self.b.of_class(classes):
            soft, stiff = self.b.e(s, key, "dt0.2"), self.b.e(s, key, "stiff")
            if soft is None or stiff is None or not (math.isfinite(soft) and math.isfinite(stiff)):
                continue
            n += 1
            better += eff(stiff) < 0.9 * eff(soft)
            worse += eff(stiff) > 1.1 * eff(soft)
        return {"better": better, "worse": worse, "n": n}.get(what)

    # cost/<method>/<median|min|max|slow>, cost/<scenario>/<method>, cost/source, cost/source-short
    def _cost(self, first, second=None):
        b = self.b
        if first == "source":
            return (
                "the timing pass (serial, on an idle machine)"
                if b.timing
                else "the accuracy pass, which ran several jobs at a time on a shared machine"
            )
        if first == "label":
            return "timing pass" if b.timing else "indicative, measured under load"
        if first == "bound":  # cost/bound/<scenario>-<method>
            scenario, _, key = (second or "").partition("-")
            w = b.wall_bound(scenario, key)
            return sig(w) if math.isfinite(w) else None
        if first in b.cls:
            w = b.wall(first, second)
            return sig(w) if math.isfinite(w) else None
        walls = [b.wall(s, first) for s in b.scenarios]
        walls = [w for w in walls if math.isfinite(w)]
        if not walls:
            return None
        return {
            "median": sig(statistics.median(walls)),
            "min": sig(min(walls)),
            "max": sig(max(walls)),
            "slow": sum(w > 1 for w in walls),
            "n": len(walls),
        }.get(second)

    # first/<scenario>/<gonthier|distributed>: first run of the held-out set (provenance.json)
    def _first(self, scenario, law):
        value = self.b.provenance["heldout_first_run"].get(scenario, {}).get(law)
        return None if value is None else pct(value) + "\\,\\%"


KEY = re.compile(r"\\result\{([^}]*)\}")


def used_keys(paper):
    paper = Path(paper)
    keys = set()
    for path in [paper / "main.tex", paper / "supplement.tex", *sorted((paper / "sections").glob("*.tex"))]:
        if path.exists():
            # comments may hold key patterns; a commented line is not a use
            text = "\n".join(re.sub(r"(?<!\\)%.*", "", line) for line in path.read_text().splitlines())
            keys.update(KEY.findall(text))
    return sorted(keys)


def numbers_tex(b, keys):
    numbers = Numbers(b)
    missing, lines = [], []
    for key in keys:
        try:
            lines.append(f"\\setresult{{{key}}}{{{numbers.resolve(key)}}}")
        except (KeyError, TypeError, ValueError):
            missing.append(key)
    if missing:
        raise SystemExit("paper_results: the paper uses keys that are not defined: " + ", ".join(missing))
    p = b.provenance
    head = (
        f"% Generated by tools/paper_results.py; do not edit. Battery of commit {p['commit']} "
        f"({p['date'].replace('--', '-')}), battery.csv sha256 {b.sha256}.\n"
    )
    return head + "\n".join(lines) + "\n"


# ---- tables --------------------------------------------------------------------------------------


def tabular(spec, head, rows, size="\\footnotesize"):
    lines = [f"{size}\\setlength{{\\tabcolsep}}{{3pt}}%", f"\\begin{{tabular}}{{{spec}}}", "\\toprule"]
    lines += [" & ".join(h) + " \\\\" for h in head]
    lines.append("\\midrule")
    for row in rows:
        lines.append(row if isinstance(row, str) else " & ".join(str(c) for c in row) + " \\\\")
    lines += ["\\botrule", "\\end{tabular}"]
    return "\n".join(lines) + "\n"


def error_table(b, scenarios, keys, extra=None, mark=None):
    """Scenario by method, errors in percent. extra: (header, function of the scenario);
    mark: function of the scenario and the error that returns a suffix for the cell."""
    head = ["Scenario", *([extra[0]] if extra else []), *[SHORT[k] for k in keys]]

    def cell(s, k):
        e = b.e(s, k)
        return b.cell(s, k) + (mark(s, e) if mark and e is not None and math.isfinite(e) else "")

    rows = [[scn(s), *([extra[1](s)] if extra else []), *[cell(s, k) for k in keys]] for s in scenarios]
    return tabular("@{}l" + ("l" if extra else "") + "r" * len(keys) + "@{}", [head], rows)


def band_text(s):
    lo, hi = BANDS[s]
    return f"$-${round((1 - lo) * 100)}/$+${round((hi - 1) * 100)}"


def band_mark(s, e):
    """The battery records the size of the error against the nominal value, not its sign. An error
    above the wider side of the band is outside it whatever its sign (dagger); one above the
    narrower side only may be outside (asterisk); one below both is inside."""
    lo, hi = BANDS[s]
    narrow, wide = sorted((1 - lo, hi - 1))
    return "$^\\dagger$" if e > wide else ("$^{*}$" if e > narrow else "")


def table_independence(b):
    rows = []
    for law in LAWS:
        ind = independence(b, law)
        held = {s: v for s, v in ind.items() if v[2] < 1}
        if not held:
            continue
        worst = max(held, key=lambda s: held[s][0])
        rows.append(
            [
                NAMES[law],
                len(held),
                len(ind) - len(held),
                sci(max(statistics.median(v[1] for v in held.values()), 1e-12)),
                pct(held[worst][0]),
                scn(worst),
            ]
        )
    head = ["Law", "Held", "Lost", "Median rel.\\ diff.", "Largest diff.\\ [pp]", "in"]
    return tabular("@{}lrrrrl@{}", [head], rows)


def table_status(b):
    """Every run that did not finish, by method, status and scenario (with the number of variants)."""
    groups = defaultdict(Counter)
    for (s, m, _), st in b.status.items():
        if st != "ok":
            groups[(m, st)][s] += 1
    names = {method(k): NAMES[k] for k in ALL}
    for engine in ENGINES:
        for key, law in LAWS.items():
            names[f"{engine}-{law}"] = f"{NAMES[key]} ({engine.capitalize() if engine != 'mujoco' else 'MuJoCo'})"
    rows = []
    for (m, st), scen in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        total = {s: sum(1 for (s2, m2, _) in b.status if s2 == s and m2 == m) for s in scen}
        where = ", ".join(f"{scn(s)} {n}/{total[s]}" for s, n in sorted(scen.items()))
        rows.append([names.get(m, m), st, sum(scen.values()), where])
    head = ["Method", "Status", "Runs", "Scenario (runs affected / runs of the scenario)"]
    return tabular("@{}llrp{0.52\\linewidth}@{}", [head], rows)


def table_pass(b):
    """Scenarios within 1, 5 and 20 %: class A, class A without the parameter-bound scenarios, class B."""
    head = ["Method"]
    for cls, name in (("A", "Class A"), ("Ar", "Class A, reduced"), ("B", "Class B")):
        n = len(b.of_class(cls))
        head.append(f"\\multicolumn{{3}}{{c}}{{{name} ($n={n}$)}}")
    sub = [""] + ["1\\,\\%", "5\\,\\%", "20\\,\\%"] * 3
    rows = []
    for key in ALL:
        row = [NAMES[key]]
        for cls in ("A", "Ar", "B"):
            for t in THRESHOLDS:
                row.append(pass_rate(b, key, cls, t)[0])
        rows.append(row)
        if key == "pcm":
            rows.append("\\midrule")
    return tabular("@{}l" + "rrr" * 3 + "@{}", [head, sub], rows)


RATIO_ROWS = [
    ("distributed", "gonthier"),
    ("thesis", "gonthier"),
    ("distributed", "vertex"),
    ("distributed", "centroid"),
    ("distributed", "pcm"),
    None,
    *[("distributed", n) for n in NATIVES],
    None,
    *[("gonthier", n) for n in NATIVES],
    None,
    *[("pcm", n) for n in NATIVES],
]


def table_ratios(b):
    def cells(a, c, subset):
        row = paired(b, a, c, subset)
        if row is None:
            return ["--", "--", "--"]
        lo, hi = 10 ** row["ci"][0], 10 ** row["ci"][1]
        gm = sig(row["ratio"])
        if lo > 1 or hi < 1:  # the interval excludes 1
            gm = f"\\textbf{{{gm}}}"
        return [gm, f"{sig(lo)}--{sig(hi)}", f"{row['wins']}/{row['losses']}" + (f" ({row['excluded']})" if row["excluded"] else "")]

    rows = []
    for pair in RATIO_ROWS:
        if pair is None:
            rows.append("\\midrule")
            continue
        a, c = pair
        rows.append([f"{NAMES[a]} / {NAMES[c]}", *cells(a, c, "A"), *cells(a, c, "A+B")])
    head = ["", "\\multicolumn{3}{c}{Class A}", "\\multicolumn{3}{c}{Classes A and B}"]
    sub = ["Errors of / over", "Ratio", "95\\,\\% interval", "W/L", "Ratio", "95\\,\\% interval", "W/L"]
    return tabular("@{}lrrrrrr@{}", [head, sub], rows)


def table_ratios_primary(b):
    """The primary comparisons: the ratio as first analysed (uncapped, failed pairs left out, every
    scenario one unit) next to the robust one, with Holm's p, the leave-one-out range and W/T/L."""
    rows = []
    for subset, name in (("A", "Class A"), ("A+B", "Classes A and B")):
        block = []
        for a, c in PRIMARY:
            old, new = paired(b, a, c, subset), robust(b, a, c, subset)
            if old is None or new is None:
                continue
            gm = sig(new["ratio"])
            if new["p_holm"] is not None and new["p_holm"] < 0.05:
                gm = f"\\textbf{{{gm}}}"
            block.append(
                [
                    f"{SHORT[a]} / {SHORT[c]}",
                    interval(old) + (f" [{old['excluded']}]" if old["excluded"] else ""),
                    f"{gm} ({sig(10 ** new['ci'][0])}--{sig(10 ** new['ci'][1])})",
                    pvalue(new["p_holm"]) if new["p_holm"] is not None else "--",
                    f"{sig(new['loo'][0])}--{sig(new['loo'][1])}",
                    f"{new['wins']}/{new['ties']}/{new['losses']}",
                    new["n"],
                ]
            )
        if not block:
            continue
        if rows:
            rows.append("\\midrule")
        rows.append(f"\\multicolumn{{6}}{{@{{}}l}}{{\\emph{{{name}}} ({block[0][-1]} units)}} \\\\")
        rows += [r[:-1] for r in block]
    head = ["", "Uncapped", "\\multicolumn{4}{c}{Capped, failures counted (reported)}"]
    sub = ["Errors of / over", "ratio (95\\,\\% int.)", "ratio (95\\,\\% int.)", "$p$ (Holm)", "leave-one-out", "W/T/L"]
    return tabular("@{}llllll@{}", [head, sub], rows)


def table_primary_compact(b):
    """The primary comparisons in one small table: robust ratio with its interval and Holm's p, on
    class A and on classes A and B, one row per pair."""
    rows = []
    for a, c in PRIMARY:
        cells = [f"{SHORT[a]} / {SHORT[c]}"]
        for subset in ("A", "A+B"):
            row = robust(b, a, c, subset)
            if row is None:
                cells += ["--", "--"]
                continue
            gm = sig(row["ratio"])
            if row["p_holm"] is not None and row["p_holm"] < 0.05:
                gm = f"\\textbf{{{gm}}}"
            cells += [
                f"{gm} ({sig(10 ** row['ci'][0])}--{sig(10 ** row['ci'][1])})",
                pvalue(row["p_holm"]) if row["p_holm"] is not None else "--",
            ]
        rows.append(cells)
    head = ["", "\\multicolumn{2}{c}{Class A}", "\\multicolumn{2}{c}{Classes A and B}"]
    sub = ["Errors of / over", "ratio (95\\,\\% int.)", "$p$", "ratio (95\\,\\% int.)", "$p$"]
    return tabular("@{}lllll@{}", [head, sub], rows)


def table_ratios_robust(b):
    """Every pair of the ratio table under the robust rules (exploratory: unadjusted intervals)."""

    def cells(a, c, subset):
        row = robust(b, a, c, subset)
        if row is None:
            return ["--", "--", "--"]
        return [sig(row["ratio"]), f"{sig(10 ** row['ci'][0])}--{sig(10 ** row['ci'][1])}", f"{row['wins']}/{row['ties']}/{row['losses']}"]

    rows = []
    for pair in RATIO_ROWS:
        if pair is None:
            rows.append("\\midrule")
            continue
        a, c = pair
        rows.append([f"{NAMES[a]} / {NAMES[c]}", *cells(a, c, "A"), *cells(a, c, "A+B")])
    head = ["", "\\multicolumn{3}{c}{Class A}", "\\multicolumn{3}{c}{Classes A and B}"]
    sub = ["Errors of / over", "Ratio", "95\\,\\% interval", "W/T/L", "Ratio", "95\\,\\% interval", "W/T/L"]
    return tabular("@{}lrrrrrr@{}", [head, sub], rows)


def table_reduced(b):
    """The primary comparisons on class A with and without the parameter-bound scenarios."""
    rows = []
    for a, c in PRIMARY:
        full, red = robust(b, a, c, "A"), robust(b, a, c, "A-hK")
        if full is None or red is None:
            continue
        wtl = lambda r: f"{r['wins']}/{r['ties']}/{r['losses']}"
        rows.append([f"{SHORT[a]} / {SHORT[c]}", interval(full), wtl(full), interval(red), wtl(red)])
    head = ["", "\\multicolumn{2}{c}{Class A}", "\\multicolumn{2}{c}{Class A, reduced}"]
    sub = ["Errors of / over", "ratio (95\\,\\% int.)", "W/T/L", "ratio (95\\,\\% int.)", "W/T/L"]
    return tabular("@{}lllll@{}", [head, sub], rows)


def table_band(b):
    """Class B: scenarios inside the band, undetermined, outside and without a stop time, by method."""
    rows = []
    for key in MAIN:
        counts = Counter(band_status(b, s, key) for s in b.of_class("B"))
        rows.append([NAMES[key], counts["inside"], counts["undetermined"], counts["outside"], counts["none"]])
        if key == "pcm":
            rows.append("\\midrule")
    head = ["Method", "Inside", "Undetermined", "Outside", "No stop time"]
    return tabular("@{}lrrrr@{}", [head], rows)


def table_perturbations(b):
    """Native cells of classes A and B whose perturbed runs scatter: base value, median and range."""
    rows = []
    for s, k in scattered(b, list(NATIVES)):
        finite, lost = b.perturbed(s, k)
        rows.append([scn(s), NAMES[k], b.cell(s, k), pct(statistics.median(finite)), f"{pct(finite[0])}--{pct(finite[-1])}", lost or ""])
    head = ["Scenario", "Model", "Base", "Median", "Range", "No value"]
    return tabular("@{}llrrrr@{}", [head], rows)


TRI_ROWS = [  # (shape, triangulation, parameter, label)
    ("disk", "rim-fan", "64", "Disk, rim fan (battery)"),
    ("disk", "zigzag", "64", "Disk, slivers"),
    ("disk", "centre-fan", "64", "Disk, centre fan"),
    ("disk", "hex", "4", "Disk, equilateral"),
    ("disk", "hex", "8", "Disk, equilateral"),
    ("disk", "hex", "16", "Disk, equilateral"),
    ("disk", "random", "300", "Disk, random"),
    ("disk", "random", "3000", "Disk, random"),
    None,
    ("ring", "quads", "64", "Ring, quads (battery)"),
    ("ring", "layers", "4", "Ring, four layers"),
]


def table_triangulation(b):
    def get(shape, case, name, parameter, rule, column, signed=True):
        for r in b.triangulation:
            if (r["shape"], r["case"], r["triangulation"], r["parameter"], r["rule"]) == (shape, case, name, parameter, rule):
                return r["triangles"] if column == "triangles" else tri_pct(float(r[column]), signed).replace("\\,\\%", "")
        return "--"

    rows = []
    for item in TRI_ROWS:
        if item is None:
            rows.append("\\midrule")
            continue
        shape, name, parameter, label = item
        rows.append(
            [
                label,
                get(shape, "spin", name, parameter, "closed", "triangles"),
                get(shape, "spin", name, parameter, "centroid", "moment_error"),
                get(shape, "spin", name, parameter, "vertex", "moment_error"),
                get(shape, "slide+spin", name, parameter, "centroid", "moment_error"),
                get(shape, "slide+spin", name, parameter, "centroid", "force_error", False),
                get(shape, "slide+spin", name, parameter, "vertex", "moment_error"),
                get(shape, "slide+spin", name, parameter, "vertex", "force_error", False),
            ]
        )
    head = ["", "", "\\multicolumn{2}{c}{Spin: torque}", "\\multicolumn{2}{c}{Slide+spin, one point}", "\\multicolumn{2}{c}{Slide+spin, vertex}"]
    sub = ["Triangulation", "Triangles", "One point", "Vertex", "torque", "force", "torque", "force"]
    return tabular("@{}lrrrrrrr@{}", [head, sub], rows)


def table_convergence(b):
    rows = []
    for key in ALL:
        row = [NAMES[key]]
        for kind in ("dt", "mesh"):
            flags = Counter()
            for s in b.scenarios:
                r = convergence(b, kind, s, key)
                if r is not None:
                    flags[r["flag"]] += 1
            row += [flags[f] for f in FLAGS]
        rows.append(row)
        if key == "pcm":
            rows.append("\\midrule")
    n_dt = sum(convergence(b, "dt", s, "distributed") is not None for s in b.scenarios)
    n_mesh = sum(convergence(b, "mesh", s, "distributed") is not None for s in b.scenarios)
    head = ["", f"\\multicolumn{{5}}{{c}}{{Step ($n={n_dt}$)}}", f"\\multicolumn{{5}}{{c}}{{Mesh ($n={n_mesh}$)}}"]
    sub = ["Method"] + ["conv.", "flat", "n-m.", "div.", "fail."] * 2
    return tabular("@{}l" + "r" * 10 + "@{}", [head, sub], rows)


def table_cost(b):
    rows = []
    for key in ALL:
        walls = sorted(w for w in (b.wall(s, key) for s in b.scenarios) if math.isfinite(w))
        if not walls:
            continue
        rows.append([NAMES[key], len(walls), sig(statistics.median(walls)), sig(walls[0]), sig(walls[-1]), sum(w > 1 for w in walls)])
        if key == "pcm":
            rows.append("\\midrule")
    head = ["Method", "Runs", "Median", "Fastest", "Slowest", "Slower than real time"]
    return tabular("@{}lrrrrr@{}", [head], rows)


def table_ablation(b):
    keys = ["distributed", "vertex", "centroid", "pcm"]
    rows = []
    for s in ("disk", "contensou", "ring"):
        for variant, seg in MESH_SERIES:
            rows.append([scn(s), f"{seg} segments", *[b.cell(s, k, variant) for k in keys]])
    rows.append("\\midrule")
    for s, pieces in (("pieces1", 1), ("pieces2", 2), ("pieces8", 8), ("pieces64", 64)):
        if s in b.cls:
            rows.append([scn(s), f"{pieces} piece" + ("s" if pieces > 1 else ""), *[b.cell(s, k) for k in keys]])
    head = ["Scenario", "Variant", "Closed form", "Vertex rule", "One point", "PCM"]
    return tabular("@{}llrrrr@{}", [head], rows)


def table_stiffness(b):
    keys = ["gonthier", "distributed", "pcm", "hydro", "nsc"]
    rows = []
    for cls in "AB":
        for s in b.of_class(cls):
            if b.e(s, "distributed", "stiff") is None:
                continue
            row = [scn(s)]
            for k in keys:
                row.append(f"{b.cell(s, k, 'dt0.2')}\\,$\\to$\\,{b.cell(s, k, 'stiff')}")
            rows.append(row)
        if cls == "A":
            rows.append("\\midrule")
    head = ["Scenario", *[SHORT[k] for k in keys]]
    return tabular("@{}l" + "r" * len(keys) + "@{}", [head], rows)


def table_heldout_first(b):
    first = b.provenance["heldout_first_run"]
    rows = []
    for s in b.heldout:
        f = first.get(s, {})
        rows.append(
            [
                scn(s),
                b.cls[s],
                pct(f["gonthier"]) if "gonthier" in f else "--",
                b.cell(s, "gonthier"),
                pct(f["distributed"]) if "distributed" in f else "--",
                b.cell(s, "distributed"),
            ]
        )
    head = ["", "", "\\multicolumn{2}{c}{Gonthier}", "\\multicolumn{2}{c}{Distributed}"]
    sub = ["Scenario", "Class", "First run", "Reported", "First run", "Reported"]
    return tabular("@{}llrrrr@{}", [head, sub], rows)


def tables(b):
    out = {
        "independence": table_independence(b),
        "status": table_status(b),
        "errors_A": error_table(b, b.of_class("A", b.dev), MAIN),
        "errors_B": error_table(b, b.of_class("B", b.dev), MAIN, ("Band", band_text), band_mark),
        "errors_C": error_table(b, b.of_class("C", b.dev), WINKLER),
        "errors_heldout": error_table(b, b.heldout, MAIN, ("Cl.", lambda s: b.cls[s])),
        # The same errors by class, both sets together (the tables of the main text).
        "errors_class_A": error_table(b, b.of_class("A"), MAIN),
        "errors_class_C": error_table(b, b.of_class("C"), WINKLER),
        "heldout_first": table_heldout_first(b),
        "pass_rates": table_pass(b),
        "ablation": table_ablation(b),
        "stiffness": table_stiffness(b),
        "cost": table_cost(b),
    }
    if b.stats:
        out["ratios"] = table_ratios(b)
        out["convergence"] = table_convergence(b)
    if b.stats.get("robust"):
        out["ratios_primary"] = table_ratios_primary(b)
        out["primary_compact"] = table_primary_compact(b)
        out["ratios_robust"] = table_ratios_robust(b)
        out["reduced"] = table_reduced(b)
    out["band"] = table_band(b)
    out["perturbations"] = table_perturbations(b)
    if b.triangulation:
        out["triangulation"] = table_triangulation(b)
    return out


# ---- figures -------------------------------------------------------------------------------------

WIDTH = 4.92  # inches: the 12.5 cm text width of the journal's template
# Okabe-Ito colours for the laws; the native models in black and greys. Every method also has its
# own marker and line style, so the figures can be read without colour.
STYLE = {
    "gonthier": ("#0072B2", "o", "-"),
    "thesis": ("#56B4E9", "o", "--"),
    "distributed": ("#D55E00", "s", "-"),
    "vertex": ("#E69F00", "^", "--"),
    "centroid": ("#CC79A7", "v", ":"),
    "pcm": ("#009E73", "D", "-."),
    "hydro": ("#000000", "P", "-"),
    "dpoint": ("#000000", "X", "--"),
    "smc": ("#6E6E6E", "<", "-"),
    "nsc": ("#6E6E6E", ">", "--"),
    "soft": ("#A8A8A8", "*", "-"),
    "mjdefault": ("#A8A8A8", "h", "--"),
}


def line_style(key, **more):
    colour, marker, ls = STYLE[key]
    open_marker = key in ("thesis", "vertex", "centroid", "dpoint", "nsc", "mjdefault")
    return {
        "color": colour, "marker": marker, "linestyle": ls, "markersize": 4, "linewidth": 1.0,
        "markerfacecolor": "white" if open_marker else colour, "markeredgewidth": 0.8, "label": NAMES[key],
    } | more  # fmt: skip


def figures(b, out_dir):
    import matplotlib

    matplotlib.use("pdf")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import LogNorm

    plt.rcParams.update(
        {
            "font.family": "STIXGeneral",
            "mathtext.fontset": "stix",
            "font.size": 8,
            "axes.titlesize": 8,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 6.5,
            "axes.linewidth": 0.6,
            "lines.linewidth": 1.0,
            "grid.linewidth": 0.3,
            "grid.color": "#CCCCCC",
            "axes.grid": True,
            "axes.axisbelow": True,
            "pdf.fonttype": 42,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.02,
            "hatch.linewidth": 0.4,
        }
    )
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    def save(fig, name):
        # No dates in the file: the same inputs give the same bytes.
        fig.savefig(out_dir / f"{name}.pdf", metadata={"CreationDate": None, "ModDate": None, "Producer": None, "Creator": None})
        plt.close(fig)

    def show(e):  # errors in percent on a log axis: floor at 0.001 %, cap lost runs at 3000 %
        return 100 * min(max(e, 1e-5), 30.0) if math.isfinite(e) else math.nan

    def heatmap(name, blocks, keys):
        """blocks: [(title, scenarios, keys shown in colour)]."""
        rows = sum(len(s) for _, s, _ in blocks)
        fig, axes = plt.subplots(
            len(blocks), 1, figsize=(WIDTH, 0.17 * rows + 0.55 * len(blocks) + 0.5), squeeze=False,
            gridspec_kw={"height_ratios": [len(s) + 0.01 for _, s, _ in blocks], "hspace": 0.12},
        )  # fmt: skip
        norm = LogNorm(1e-4, 1.0, clip=True)
        cmap = plt.get_cmap("cividis_r")
        image = None
        for ax, (title, scenarios, coloured) in zip(axes[:, 0], blocks):
            grid = np.full((len(scenarios), len(keys)), np.nan)
            for i, s in enumerate(scenarios):
                for j, k in enumerate(keys):
                    e = b.e(s, k)
                    if k in coloured and e is not None and math.isfinite(e):
                        grid[i, j] = max(e, 1e-4)
            image = ax.imshow(grid, norm=norm, cmap=cmap, aspect="auto")
            for i, s in enumerate(scenarios):
                for j, k in enumerate(keys):
                    e = b.e(s, k)
                    if k not in coloured or e is None:
                        continue
                    if not math.isfinite(e):
                        ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, fill=False, hatch="////", edgecolor="#888888", linewidth=0))
                        label, colour = {"timeout": "t/o", "failed": "fail"}.get(b.st(s, k), "nan"), "black"
                    else:
                        label = pct(e).replace("$>$", ">").replace("$<$", "<")
                        r, g, bl, _ = cmap(norm(max(e, 1e-4)))
                        colour = "white" if 0.299 * r + 0.587 * g + 0.114 * bl < 0.5 else "black"
                    ax.text(j, i, label, ha="center", va="center", fontsize=5.3, color=colour)
            if len(coloured) < len(keys):
                ax.text(
                    (len(coloured) + len(keys) - 1) / 2, (len(scenarios) - 1) / 2,
                    "models of another kind:\nnot compared on class C", ha="center", va="center", fontsize=7, color="#555555",
                )  # fmt: skip
            ax.set_yticks(range(len(scenarios)), scenarios)
            ax.set_xticks([])
            ax.set_ylabel(title)
            ax.grid(False)
            ax.tick_params(length=0)
        top = axes[0, 0]
        top.xaxis.tick_top()
        top.set_xticks(range(len(keys)), [NAMES[k].replace(", ", ",\n").replace(" ", "\n") for k in keys], fontsize=6)
        bar = fig.colorbar(image, ax=axes[:, 0].tolist(), orientation="horizontal", fraction=0.03, pad=0.02, aspect=50)
        bar.set_ticks([1e-4, 1e-3, 1e-2, 1e-1, 1], labels=["$\\leq$0.01 %", "0.1 %", "1 %", "10 %", "$\\geq$100 %"])
        bar.ax.tick_params(labelsize=6.5)
        save(fig, name)

    heatmap(
        "heatmap_A",
        [("Class A, development", b.of_class("A", b.dev), MAIN), ("Class A, second set", b.of_class("A", b.heldout), MAIN)],
        MAIN,
    )
    heatmap(
        "heatmap_BC",
        [("Class B", b.of_class("B"), MAIN), ("Class C", b.of_class("C"), WINKLER)],
        MAIN,
    )

    # Performance profiles, classes A and B.
    fig, ax = plt.subplots(figsize=(WIDTH, 2.6))
    prof = profile(b, MAIN)
    taus = np.logspace(0, 3, 241)
    for k in MAIN:
        rho = [sum(r <= t for r in prof[k]) / len(prof[k]) for t in taus]
        ax.step(taus, rho, where="post", **line_style(k, markevery=(MAIN.index(k) * 5, 60)))
    ax.set_xscale("log")
    ax.set_xlabel("Factor $\\tau$ of the smallest error of the scenario")
    ax.set_ylabel("Share of scenarios within $\\tau$")
    ax.set_ylim(0, 1.02)
    ax.set_xlim(1, 1000)
    ax.legend(ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.2), frameon=False)
    save(fig, "profile_AB")

    # Paired ratios with their bootstrap intervals.
    if b.stats:
        pairs = [p for p in RATIO_ROWS if p is not None and p[0] != "pcm"]
        fig, ax = plt.subplots(figsize=(WIDTH, 0.19 * len(pairs) + 0.7))
        for i, (a, c) in enumerate(pairs):
            y = len(pairs) - 1 - i
            for subset, dy, filled in (("A", 0.17, False), ("A+B", -0.17, True)):
                row = paired(b, a, c, subset)
                if row is None:
                    continue
                gm, lo, hi = row["ratio"], 10 ** row["ci"][0], 10 ** row["ci"][1]
                ax.plot([lo, hi], [y + dy] * 2, color="black", linewidth=0.8)
                ax.plot(
                    gm, y + dy, marker="o", markersize=3.5, color="black", markerfacecolor="black" if filled else "white",
                    markeredgewidth=0.8, linestyle="none", label=None if i else ("Classes A and B" if filled else "Class A"),
                )  # fmt: skip
        ax.axvline(1, color="#555555", linewidth=0.6, linestyle="--")
        ax.set_xscale("log")
        ax.set_yticks(range(len(pairs)), [f"{NAMES[a]} / {NAMES[c]}" for a, c in reversed(pairs)])
        ax.set_xlabel("Ratio of errors (below 1: the first method has the smaller error)")
        ax.set_ylim(-0.6, len(pairs) - 0.4)
        ax.grid(axis="y", visible=False)
        ax.legend(loc="lower right", frameon=False)
        save(fig, "forest_AB")

    # The primary comparisons: the uncapped estimate (open) and the reported, capped one (filled).
    if b.stats.get("robust"):
        rows = [(a, c, s) for s in ("A", "A+B") for a, c in PRIMARY if robust(b, a, c, s) is not None]
        fig, ax = plt.subplots(figsize=(WIDTH, 0.21 * len(rows) + 0.75))
        for i, (a, c, subset) in enumerate(rows):
            y = len(rows) - 1 - i
            for row, dy, filled in ((paired(b, a, c, subset), 0.17, False), (robust(b, a, c, subset), -0.17, True)):
                if row is None:
                    continue
                gm, lo, hi = row["ratio"], 10 ** row["ci"][0], 10 ** row["ci"][1]
                ax.plot([lo, hi], [y + dy] * 2, color="black", linewidth=0.8)
                ax.plot(
                    gm, y + dy, marker="o", markersize=3.5, color="black", markerfacecolor="black" if filled else "white",
                    markeredgewidth=0.8, linestyle="none",
                    label=None if i else ("Reported: capped, failures counted, clusters" if filled else "Sensitivity: uncapped, failures left out"),
                )  # fmt: skip
        ax.axvline(1, color="#555555", linewidth=0.6, linestyle="--")
        ax.set_xscale("log")
        ax.set_yticks(
            range(len(rows)),
            [f"{NAMES[a]} / {NAMES[c]}, {'A' if s == 'A' else 'A and B'}" for a, c, s in reversed(rows)],
        )
        ax.set_xlabel("Ratio of errors (below 1: the first method has the smaller error)")
        ax.set_ylim(-0.6, len(rows) - 0.4)
        ax.grid(axis="y", visible=False)
        ax.legend(loc="upper center", bbox_to_anchor=(0.35, -0.22), ncol=1, frameon=False)
        save(fig, "forest_primary")

    # The triangulation experiment: the two quadratures on a spinning disk.
    if b.triangulation:
        import ablation_triangulation

        ablation_triangulation.figure(b.triangulation, out_dir / "ablation_triangulation.pdf", WIDTH)

    def series_panels(name, kind, scenarios, keys, xlabel):
        scenarios = [s for s in scenarios if s in b.cls]
        cols = 3
        nrows = math.ceil(len(scenarios) / cols)
        fig, axes = plt.subplots(nrows, cols, figsize=(WIDTH, 1.55 * nrows + 0.5), squeeze=False, sharex=True)
        for ax, s in zip(axes.flat, scenarios):
            for k in keys:
                pts = [(h, show(e)) for h, e in b.series(s, k, kind)]
                if pts:
                    ax.plot([p[0] for p in pts], [p[1] for p in pts], **line_style(k, markersize=3))
            ax.set_xscale("log")
            ax.set_yscale("log")
            ax.set_title(f"{s} ({b.cls[s]})", pad=2)
            ticks = [h for _, h in (DT_SERIES if kind == "dt" else MESH_SERIES)]
            ax.set_xticks(ticks, [f"{t:g}" for t in ticks])
            ax.minorticks_off()
            if kind == "dt":
                ax.invert_xaxis()
        for ax in axes.flat[len(scenarios) :]:
            ax.axis("off")
        for ax in axes[-1]:
            ax.set_xlabel(xlabel)
        for ax in axes[:, 0]:
            ax.set_ylabel("Error [%]")
        handles, labels = axes.flat[0].get_legend_handles_labels()
        fig.subplots_adjust(hspace=0.32, wspace=0.32)
        fig.legend(handles, labels, ncol=4, loc="upper center", bbox_to_anchor=(0.5, 0.11 - 0.1 * nrows), frameon=False)
        save(fig, name)

    shown = ["gonthier", "distributed", "pcm", "hydro", "nsc", "soft"]
    series_panels("convergence_dt", "dt", ["drop", "block", "energy", "tower", "massratio", "disk"], shown, "Time step [ms]")
    series_panels("convergence_mesh", "mesh", ["disk", "bearing", "cone", "journal", "bowl", "energy"], shown, "Segments per turn")

    # Friction-quadrature ablation: refinement of the curved meshes, and decomposition of the ground.
    quad = ["distributed", "vertex", "centroid", "pcm"]
    fig, axes = plt.subplots(1, 3, figsize=(WIDTH, 1.9), sharey=True)
    for ax, s in zip(axes[:2], ("disk", "contensou")):
        for k in quad:
            pts = [(h, show(e)) for h, e in b.series(s, k, "mesh")]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], **line_style(k))
        ax.set_xscale("log")
        ax.set_xticks([32, 64, 128], ["32", "64", "128"])
        ax.minorticks_off()
        ax.set_title(s, pad=2)
        ax.set_xlabel("Segments per turn")
    pieces = [(n, f"pieces{n}") for n in (1, 2, 8, 64) if f"pieces{n}" in b.cls]
    for k in [*quad, "gonthier"]:
        axes[2].plot([n for n, _ in pieces], [show(b.e(s, k)) for _, s in pieces], **line_style(k))
    axes[2].set_xscale("log")
    axes[2].set_xticks([n for n, _ in pieces], [str(n) for n, _ in pieces])
    axes[2].minorticks_off()
    axes[2].set_title("pieces", pad=2)
    axes[2].set_xlabel("Pieces of the ground")
    axes[0].set_yscale("log")
    axes[0].set_ylabel("Error [%]")
    handles, labels = axes[2].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.06), frameon=False)
    save(fig, "ablation")

    # Engine independence: the error of a law on Drake and on MuJoCo against its error on Chrono.
    fig, axes = plt.subplots(1, 2, figsize=(WIDTH, 2.35), sharey=True)
    for ax, law in zip(axes, ("gonthier", "distributed")):
        for engine, marker, filled in (("drake", "o", False), ("mujoco", "x", True)):
            xs, ys = [], []
            for s in b.scenarios:
                x, y = b.e(s, f"chrono-{law}"), b.e(s, f"{engine}-{law}")
                if x is not None and y is not None and math.isfinite(x) and math.isfinite(y):
                    xs.append(100 * max(x, 1e-5))
                    ys.append(100 * max(y, 1e-5))
            ax.plot(
                xs, ys, linestyle="none", marker=marker, markersize=4.5 if marker == "o" else 3.5, color="black",
                markerfacecolor="none", markeredgewidth=0.7, label={"drake": "Drake", "mujoco": "MuJoCo"}[engine],
            )  # fmt: skip
        ax.plot([1e-3, 1e7], [1e-3, 1e7], color="#777777", linewidth=0.5)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlim(1e-3, 1e7)
        ax.set_ylim(1e-3, 1e7)
        ax.set_title(NAMES[law], pad=2)
        ax.set_xlabel("Error on Chrono [%]")
    axes[0].set_ylabel("Error on the other engine [%]")
    axes[0].legend(loc="upper left", frameon=False)
    save(fig, "independence")

    # Stiffness series: the error at K and at 10 K, both at 0.2 ms.
    scen = [s for cls in "AB" for s in b.of_class(cls) if b.e(s, "distributed", "stiff") is not None]
    fig, axes = plt.subplots(2, 1, figsize=(WIDTH, 3.6), sharex=True)
    for ax, k in zip(axes, ("distributed", "hydro")):
        for i, s in enumerate(scen):
            soft, stiff = b.e(s, k, "dt0.2"), b.e(s, k, "stiff")
            if soft is None or stiff is None or not (math.isfinite(soft) and math.isfinite(stiff)):
                ax.text(i, 1e-2, "×", ha="center", va="center", fontsize=7)
                continue
            ax.plot([i, i], [show(soft), show(stiff)], color="#777777", linewidth=0.8)
            ax.plot(i, show(soft), marker="o", markersize=4, color="black", markerfacecolor="white", markeredgewidth=0.8, linestyle="none", label="$K$" if i == 0 else None)
            ax.plot(i, show(stiff), marker="o", markersize=4, color="black", linestyle="none", label="$10\\,K$" if i == 0 else None)
        ax.set_yscale("log")
        ax.set_ylabel(f"Error [%], {NAMES[k]}")
        n_a = len([s for s in scen if b.cls[s] == "A"])
        ax.axvline(n_a - 0.5, color="#555555", linewidth=0.5, linestyle=":")
    axes[0].legend(loc="upper left", ncol=2, frameon=False)
    axes[1].set_xticks(range(len(scen)), scen, rotation=90)
    save(fig, "stiffness")

    # Work against precision: median cost of a method and share of class A and B scenarios within 5 %.
    fig, ax = plt.subplots(figsize=(WIDTH, 2.7))
    for k in ALL:
        walls = sorted(w for w in (b.wall(s, k) for s in b.of_class("AB")) if math.isfinite(w))
        if not walls:
            continue
        n, total = pass_rate(b, k, "AB", 0.05)
        med = statistics.median(walls)
        q = lambda f, walls=walls: walls[min(len(walls) - 1, int(f * len(walls)))]
        ax.plot([q(0.1), q(0.9)], [n / total] * 2, color=STYLE[k][0], linewidth=0.7)
        ax.plot(med, n / total, **line_style(k, linestyle="none", markersize=5))
    ax.axvline(1, color="#555555", linewidth=0.5, linestyle="--")
    ax.set_xscale("log")
    ax.set_xlabel("Wall-clock time over simulated time (median; bar: 10th to 90th percentile of the scenarios)")
    ax.set_ylabel("Share of class A and B scenarios within 5 %")
    ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.2), frameon=False)
    save(fig, "work_precision")


# ---- main ----------------------------------------------------------------------------------------


def generate(results, paper, with_figures=True):
    b = Battery(results, paper)
    out = Path(paper) / "generated"
    (out / "tables").mkdir(parents=True, exist_ok=True)
    (out / "numbers.tex").write_text(numbers_tex(b, used_keys(paper)))
    for name, text in tables(b).items():
        (out / "tables" / f"{name}.tex").write_text(
            "% Generated by tools/paper_results.py; do not edit.\n" + text
        )
    if with_figures:
        figures(b, out / "figures")
    return b


def main(argv=None):
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--results", default=root / "results")
    ap.add_argument("--paper", default=root / "paper")
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--list-keys", action="store_true", help="print the keys the paper uses with their values")
    a = ap.parse_args(argv)
    if a.list_keys:
        numbers = Numbers(Battery(a.results, a.paper))
        for key in used_keys(a.paper):
            try:
                print(f"{key}\t{numbers.resolve(key)}")
            except (KeyError, TypeError, ValueError):
                print(f"{key}\tUNDEFINED")
        return 0
    generate(a.results, a.paper, not a.no_figures)
    return 0


if __name__ == "__main__":
    sys.exit(main())
