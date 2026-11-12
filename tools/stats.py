# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "matplotlib"]
# ///
"""Statistics over the scenario battery (docs/scenarios.md §2, docs/research/scenario-audit.md §4).

Reads a battery CSV (engine,contact,scenario,variant,status,error,realtime_ratio,wall_s,steps,dt,
segments,stiffness_scale,class; older files with fewer columns are accepted) and the terms CSV next
to it if present, and writes stats.md, stats.json and figures/*.pdf|png next to it.

Method = engine-contact. Laws: gonthier, gonthier-thesis, distributed, its friction-quadrature
ablations distributed-vertex and distributed-centroid, and the baseline pcm. Natives: contacts starting
with `native`. Only Drake's hydroelastic `native` is a Winkler-type model, so class C (large
deformation, contact switching) compares laws and Winkler natives only.

Rules: an error below FLOOR is a practical tie and counts as FLOOR in every ratio and profile; nan,
timeout and failed count as infinite error (a failure to pass), and paired comparisons leave out the
pairs with a non-finite value, reporting how many. Duplicate rows (resumed runs): the last wins.

Robust comparisons (the ones to draw conclusions from): every error is capped at CAP, and a run that
is nan, timeout or failed counts as CAP for whichever method it belongs to, so no pair is left out and
the size of a catastrophic error carries no weight; scenarios that are the same problem under another
name (`pieces1`, `pieces2`, ...: a common stem and a number) form one cluster that enters once, with
the mean of its log ratios; each ratio comes with its leave-one-out range, with wins, ties and losses
under a tie band (a ratio within TIE of 1 is a tie) and a sign test, and with a bootstrap p-value. The
PRIMARY comparisons (distributed against gonthier, and each of the two against the default native
model of each engine, on class A and on classes A and B) are the only ones tested, with Holm's
correction over that family; every other pair is exploratory. The p-values of that family are
not sampled: bootstrap_p() computes the bootstrap distribution of the mean itself (the convolution
of the units' values on a fine grid, rounded down and up, which bounds the p-value from both sides),
so they depend on no seed and no B. The smallest are about 0.002 and 0.004 before Holm multiplies
them by 14 and 13; sampled with B = 10^4 their Monte Carlo error was of the size of the margin to
0.05 (the second comparison survived with some seeds and not with others), and with B = 10^7 the
adjusted values still moved in the third decimal (0.0213 to 0.0217 for the first). On the
committed battery the bounds leave less than 1e-5 open in the two adjusted values below 0.05
(0.02147 and 0.04564). `A-hK` is class A without the scenarios
whose rigid reference no damped compliant model can meet at the chosen h and K (PARAMETER_BOUND).

Outputs: pass rates at 1/5/20 % per class; paired log10 error ratios with stratified bootstrap
intervals (resampling scenarios within class), uncapped and robust; performance profiles; convergence of the error with
the time step and the mesh (log-log slope, flags); the stiffness series; independence of the engines;
spread over the eight perturbations; the dominant error term of every base run (JSON only).

usage: uv run tools/stats.py results/battery.csv [--no-figures] [--seed N] [--bootstrap B]
"""

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

LAWS = (
    "gonthier",
    "gonthier-thesis",
    "distributed",
    "distributed-vertex",
    "distributed-centroid",
    "pcm",
)
FLOOR = 1e-3
CAP = 1.0  # robust comparisons: an error above 100 %, or a run without an error, counts as 100 %
TIE = 1.25  # robust comparisons: two errors within this factor of each other are a tie
# Class A scenarios whose rigid-body reference lacks the rolling resistance (bowl, trough,
# turntable) or the compliance (edge) that any damped foundation has at finite h and K: their error
# measures those two parameters more than the contact law. `A-hK` is class A without them.
PARAMETER_BOUND = ("bowl", "trough", "turntable", "edge")
# The comparisons that are tested (Holm over the family); the others are exploratory.
PRIMARY_LAWS = ("gonthier", "distributed")
PRIMARY_NATIVE = "native"  # the default native model of each engine
PRIMARY_SUBSETS = ("A", "A+B")
THRESHOLDS = (0.01, 0.05, 0.20)
DT_SERIES = (
    ("dt2", 2.0),
    ("base", 1.0),
    ("dt0.5", 0.5),
    ("dt0.2", 0.2),
    ("dt0.1", 0.1),
)
MESH_SERIES = (("mesh32", 32), ("base", 64), ("mesh128", 128))
SLOPE_TOL = 0.2
# fallback when the CSV has no class column
CLASSES = {
    "A": [
        "block",
        "cylinder",
        "push",
        "billiard",
        "vgroove",
        "twoblocks",
        "sphere",
        "bowl",
        "trough",
        "journal",
        "wedge",
        "tower",
        "massratio",
        "turntable",
        "energy",
        "oblique",
        "edge",
        "pieces1",
        "pieces2",
        "pieces8",
        "pieces64",
    ],
    "B": ["disk", "contensou", "ring", "bearing", "cone"],
    "C": ["tip", "drop", "hole", "corner", "offset", "stack", "restitution"],
}
SUBSETS = {"A": ("A",), "A+B": ("A", "B"), "C": ("C",)}
ROBUST_SUBSETS = {"A": ("A",), "A+B": ("A", "B"), "A-hK": ("A",)}
COLOURS = {  # Okabe–Ito
    "gonthier": "#0072B2",
    "gonthier-thesis": "#56B4E9",
    "distributed": "#D55E00",
    "distributed-vertex": "#E69F00",
    "distributed-centroid": "#F0E442",
    "pcm": "#CC79A7",
    "native": "#009E73",
}
MARKERS = {"drake": "o", "chrono": "s", "mujoco": "^"}
LINESTYLES = {"drake": "-", "chrono": "--", "mujoco": ":"}


def method(engine, contact):
    return f"{engine}-{contact}"


def is_law(contact):
    return contact in LAWS


def is_winkler_native(engine, contact):
    return engine == "drake" and contact == "native"


def colour(contact):
    return COLOURS.get(contact, COLOURS["native"])


def fnum(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return math.nan


def class_of(scenario):
    for c, names in CLASSES.items():
        if scenario in names:
            return c
    return "?"


def load(path):
    """Rows as dicts with error as a float (inf for nan/timeout/failed); the last of duplicates wins."""
    unique = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            status = r.get("status") or (
                "nan" if not math.isfinite(fnum(r.get("error"))) else "ok"
            )
            err = fnum(r.get("error"))
            if status != "ok" or not math.isfinite(err):
                err = math.inf
            row = {
                "engine": r["engine"],
                "contact": r["contact"],
                "scenario": r["scenario"],
                "variant": r["variant"],
                "status": status,
                "error": err,
                "wall": fnum(r.get("wall_s")),
                "dt": fnum(r.get("dt")),
                "segments": int(fnum(r.get("segments")) or 0)
                if r.get("segments")
                else 0,
                "scale": fnum(r.get("stiffness_scale"))
                if r.get("stiffness_scale")
                else 1.0,
                "class": r.get("class") or class_of(r["scenario"]),
            }
            unique[(row["engine"], row["contact"], row["scenario"], row["variant"])] = (
                row
            )
    return list(unique.values())


def load_terms(path):
    """(scenario, method, variant) -> {term: value}."""
    out = defaultdict(dict)
    if not Path(path).exists():
        return out
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            out[(r["scenario"], method(r["engine"], r["contact"]), r["variant"])][
                r["term"]
            ] = fnum(r["value"])
    return out


class Data:
    def __init__(self, rows):
        self.rows = rows
        self.err = {}  # (scenario, method, variant) -> error
        self.wall = {}
        self.cls = {}
        self.methods = {}  # method -> (engine, contact)
        for r in rows:
            m = method(r["engine"], r["contact"])
            self.methods[m] = (r["engine"], r["contact"])
            self.err[(r["scenario"], m, r["variant"])] = r["error"]
            self.wall[(r["scenario"], m, r["variant"])] = r["wall"]
            self.cls[r["scenario"]] = r["class"]
        self.scenarios = sorted(self.cls, key=lambda s: (self.cls[s], s))
        order = {c: i for i, c in enumerate(LAWS)}
        self.method_list = sorted(
            self.methods,
            key=lambda m: (self.methods[m][0], order.get(self.methods[m][1], 9), m),
        )
        self.series = {  # (scenario, method, variant) -> row, for dt/segments
            (r["scenario"], method(r["engine"], r["contact"]), r["variant"]): r
            for r in rows
        }

    def base(self, s, m):
        return self.err.get((s, m, "base"))

    def eligible(self, m, cls):
        """Whether method m takes part in class cls (class C: laws and Winkler natives only)."""
        engine, contact = self.methods[m]
        return cls != "C" or is_law(contact) or is_winkler_native(engine, contact)

    def scenarios_of(self, classes):
        return [s for s in self.scenarios if self.cls[s] in classes]


def eff(e):
    return max(e, FLOOR) if math.isfinite(e) else math.inf


# ---- pass rates --------------------------------------------------------------------------------


def pass_rates(d):
    out = {}
    for cls in "ABC":
        scens = d.scenarios_of(cls)
        for m in d.method_list:
            if not scens or not d.eligible(m, cls):
                continue
            errs = [d.base(s, m) for s in scens]
            errs = [e for e in errs if e is not None]
            if not errs:
                continue
            finite = [e for e in errs if math.isfinite(e)]
            out[(cls, m)] = {
                "n": len(errs),
                "pass": {
                    f"{int(t * 100)}%": sum(e <= t for e in errs) / len(errs)
                    for t in THRESHOLDS
                },
                "median": float(np.median(finite)) if finite else math.nan,
                "geomean": float(
                    np.exp(np.mean(np.log([max(e, FLOOR) for e in finite])))
                )
                if finite
                else math.nan,
                "failed": len(errs) - len(finite),
            }
    return out


# ---- paired comparisons ------------------------------------------------------------------------


def paired(d, a, b, classes):
    """log10(error a / error b) per scenario of the classes where both are finite, with the class."""
    vals, strata, excluded = [], [], 0
    for s in d.scenarios_of(classes):
        ea, eb = d.base(s, a), d.base(s, b)
        if ea is None or eb is None:
            continue
        if not (math.isfinite(ea) and math.isfinite(eb)):
            excluded += 1
            continue
        vals.append(math.log10(eff(ea) / eff(eb)))
        strata.append(d.cls[s])
    return np.array(vals), strata, excluded


def bootstrap(vals, strata, B, rng):
    """Mean and 95 % interval, resampling scenarios within each class."""
    if len(vals) == 0:
        return math.nan, math.nan, math.nan
    groups = [
        np.array([v for v, c in zip(vals, strata) if c == k])
        for k in sorted(set(strata))
    ]
    means = np.empty(B)
    for i in range(B):
        means[i] = np.concatenate(
            [g[rng.integers(0, len(g), len(g))] for g in groups]
        ).mean()
    lo, hi = np.percentile(means, [2.5, 97.5])
    return float(vals.mean()), float(lo), float(hi)


def bootstrap_p(vals, strata, bins=1 << 22):
    """Two-sided bootstrap p-value of "the mean is 0" under the resampling of bootstrap() (units
    drawn with replacement within each class), computed and not sampled: (p, half-width).

    The sum of the draws of a class is the n-fold convolution of the distribution of its values. The
    values go on a grid of `bins` steps over the range of the sum, once rounded down and once rounded
    up, and the convolutions are powers in Fourier space; the two roundings bound P(sum <= 0) and
    P(sum >= 0) from both sides. The p-value is twice the smaller of the two midpoints, and the
    half-width is what the bounds leave open (a sum of exactly zero falls between them)."""
    vals = np.asarray(vals, dtype=float)
    groups = [
        np.array([v for v, c in zip(vals, strata) if c == k])
        for k in sorted(set(strata))
    ]
    if len(vals) == 0:
        return math.nan, math.nan
    n = len(vals)
    low = sum(len(g) * g.min() for g in groups)
    span = sum(len(g) * (g.max() - g.min()) for g in groups)
    if span <= 0:  # every resample has the same sum
        return (1.0 if low == 0 else 0.0), 0.0
    step = span / (bins - n - 2)
    slack = 1e-7  # in grid steps: keeps the bounds valid under the rounding of the divisions

    def distribution(rounding):
        spectrum = None
        for g in groups:
            index = rounding((g - g.min()) / step).astype(np.int64).clip(0)
            f = np.fft.rfft(np.bincount(index, minlength=bins) / len(g)) ** len(g)
            spectrum = f if spectrum is None else spectrum * f
        return np.clip(np.fft.irfft(spectrum, bins), 0.0, None)

    down = distribution(lambda x: np.floor(x - slack))  # sum >= low + step * index
    up = distribution(lambda x: np.ceil(x + slack))  # sum <= low + step * index
    zero = -low / step  # the index of a sum of zero
    j = np.arange(bins)
    le = (up[j <= zero - slack].sum(), down[j <= zero + slack].sum())
    ge = (down[j >= zero + slack].sum(), up[j >= zero - slack].sum())
    side = min(le, ge, key=lambda b: b[0] + b[1])
    return float(min(1.0, side[0] + side[1])), float(min(1.0, abs(side[1] - side[0])))


def comparisons(d):
    """[(a, b, label)]: law vs law, and each law vs each native of its own engine."""
    out = []
    for engine in sorted({e for e, _ in d.methods.values()}):
        have = lambda c, engine=engine: method(engine, c) in d.methods
        for a, b in (
            ("distributed", "gonthier"),
            ("gonthier-thesis", "gonthier"),
            ("distributed", "gonthier-thesis"),
            ("distributed", "distributed-vertex"),
            ("distributed", "distributed-centroid"),
            ("distributed", "pcm"),
        ):
            if have(a) and have(b):
                out.append((method(engine, a), method(engine, b)))
        for m, (e, c) in d.methods.items():
            if e == engine and not is_law(c):
                for law in LAWS:
                    if have(law):
                        out.append((method(engine, law), m))
    return out


def paired_table(d, B, seed):
    rng = np.random.default_rng(seed)
    rows = []
    for a, b in comparisons(d):
        for name, classes in SUBSETS.items():
            if name == "C" and not (d.eligible(a, "C") and d.eligible(b, "C")):
                continue
            vals, strata, excl = paired(d, a, b, classes)
            if len(vals) == 0:
                continue
            mean, lo, hi = bootstrap(vals, strata, B, rng)
            rows.append(
                {
                    "a": a,
                    "b": b,
                    "subset": name,
                    "n": len(vals),
                    "excluded": excl,
                    "log10_ratio": mean,
                    "ci": [lo, hi],
                    "ratio": 10**mean,
                    "wins": int((vals < 0).sum()),
                    "losses": int((vals > 0).sum()),
                }
            )
    return rows


# ---- robust paired comparisons -----------------------------------------------------------------


def capped(e):
    """An error for the robust comparisons: floored, capped, and CAP for a run without one."""
    if e is None or not math.isfinite(e):
        return CAP
    return min(max(e, FLOOR), CAP)


def cluster_of(scenario):
    """`pieces8` -> `pieces`: scenarios that differ in a trailing number are one problem."""
    stem = scenario.rstrip("0123456789")
    return stem if stem and stem != scenario else scenario


def robust_units(d, a, b, subset, cap=True):
    """[(unit, class, log10 ratio)]: one entry per cluster of scenarios of the subset. With cap,
    every error is capped and a failure counts as the cap; without, pairs with a failure are left
    out as in paired()."""
    groups = defaultdict(list)
    for s in d.scenarios_of(ROBUST_SUBSETS[subset]):
        if subset == "A-hK" and s in PARAMETER_BOUND:
            continue
        ea, eb = d.base(s, a), d.base(s, b)
        if ea is None or eb is None:
            continue
        if cap:
            value = math.log10(capped(ea) / capped(eb))
        elif math.isfinite(ea) and math.isfinite(eb):
            value = math.log10(eff(ea) / eff(eb))
        else:
            continue
        groups[(cluster_of(s), d.cls[s])].append(value)
    return [(unit, cls, float(np.mean(v))) for (unit, cls), v in groups.items()]


def sign_test(wins, losses):
    """Two-sided exact binomial p-value of wins against losses (ties left out)."""
    n = wins + losses
    if n == 0:
        return 1.0
    k = min(wins, losses)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2**n
    return min(1.0, 2 * tail)


def leave_one_out(units):
    """(lowest ratio, unit removed, highest ratio, unit removed) of the geometric mean."""
    if len(units) < 2:
        return math.nan, "", math.nan, ""
    total = sum(v for _, _, v in units)
    means = [((total - v) / (len(units) - 1), u) for u, _, v in units]
    lo, hi = min(means), max(means)
    return 10 ** lo[0], lo[1], 10 ** hi[0], hi[1]


def holm(pvalues):
    """Holm's step-down adjustment of a family of p-values (same order as given)."""
    order = sorted(range(len(pvalues)), key=lambda i: pvalues[i])
    out, running = [0.0] * len(pvalues), 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (len(pvalues) - rank) * pvalues[i]))
        out[i] = running
    return out


def is_primary(d, a, b, subset):
    (ea, ca), (eb, cb) = d.methods[a], d.methods[b]
    if subset not in PRIMARY_SUBSETS or ea != eb:
        return False
    if (ca, cb) == ("distributed", "gonthier"):
        return (
            ea == "chrono"
        )  # the laws give the same errors on every engine: one engine, once
    return ca in PRIMARY_LAWS and cb == PRIMARY_NATIVE


def robust_table(d, B, seed):
    rng = np.random.default_rng(seed + 1)
    rows = []
    for a, b in comparisons(d):
        for subset in ROBUST_SUBSETS:
            units = robust_units(d, a, b, subset)
            if not units:
                continue
            vals = np.array([v for _, _, v in units])
            strata = [c for _, c, _ in units]
            mean, lo, hi = bootstrap(vals, strata, B, rng)
            # Two-sided bootstrap p-value of "the geometric mean ratio is 1", from the same resampling
            # scheme, with the +1 that keeps it away from zero.
            groups = [
                np.array([v for v, c in zip(vals, strata) if c == k])
                for k in sorted(set(strata))
            ]
            means = np.empty(B)
            for i in range(B):
                means[i] = np.concatenate(
                    [g[rng.integers(0, len(g), len(g))] for g in groups]
                ).mean()
            p_boot = min(
                1.0, 2 * (min((means <= 0).sum(), (means >= 0).sum()) + 1) / (B + 1)
            )
            band = math.log10(TIE)
            wins, losses = int((vals < -band).sum()), int((vals > band).sum())
            loo_lo, loo_lo_unit, loo_hi, loo_hi_unit = leave_one_out(units)
            raw = robust_units(d, a, b, subset, cap=False)
            raw_lo, raw_lo_unit, raw_hi, raw_hi_unit = leave_one_out(raw)
            rows.append(
                {
                    "a": a,
                    "b": b,
                    "subset": subset,
                    "primary": is_primary(d, a, b, subset),
                    "n": len(units),
                    "log10_ratio": mean,
                    "ratio": 10**mean,
                    "ci": [lo, hi],
                    "p_boot": p_boot,
                    "p_holm": None,
                    "wins": wins,
                    "ties": len(units) - wins - losses,
                    "losses": losses,
                    "p_sign": sign_test(wins, losses),
                    "loo": [loo_lo, loo_hi],
                    "loo_units": [loo_lo_unit, loo_hi_unit],
                    "uncapped_ratio": 10 ** float(np.mean([v for _, _, v in raw]))
                    if raw
                    else math.nan,
                    "uncapped_n": len(raw),
                    "uncapped_loo": [raw_lo, raw_hi],
                    "uncapped_loo_units": [raw_lo_unit, raw_hi_unit],
                }
            )
    # The tested family: p-values computed, not sampled (bootstrap_p), so that Holm's adjustment
    # does not multiply a Monte Carlo error; the intervals above keep their resampling.
    family = [r for r in rows if r["primary"]]
    for r in family:
        units = robust_units(d, r["a"], r["b"], r["subset"])
        r["p_boot"], r["p_boot_width"] = bootstrap_p(
            [v for _, _, v in units], [c for _, c, _ in units]
        )
    for r, p in zip(family, holm([r["p_boot"] for r in family])):
        r["p_holm"] = p
    return rows


# ---- performance profiles ----------------------------------------------------------------------


def profiles(d, classes, methods):
    """method -> (taus, rho); rho(tau) = share of scenarios with error <= tau * best (floored)."""
    scens = d.scenarios_of(classes)
    table = {m: [d.base(s, m) for s in scens] for m in methods}
    n = len(scens)
    if n == 0 or not methods:
        return {}
    ratios = {m: [] for m in methods}
    for i in range(n):
        col = [
            eff(table[m][i]) if table[m][i] is not None else math.inf for m in methods
        ]
        best = min(col)
        if not math.isfinite(best):
            continue
        for m, e in zip(methods, col):
            ratios[m].append(e / best)
    count = len(ratios[methods[0]])
    taus = np.logspace(0, 3, 121)
    return {
        m: (
            taus,
            np.array([sum(r <= t for r in ratios[m]) / count for t in taus])
            if count
            else taus * 0,
        )
        for m in methods
    }


def profile_summary(prof):
    out = {}
    for m, (taus, rho) in prof.items():
        out[m] = {
            "best": float(rho[0]),
            "tau2": float(np.interp(2.0, taus, rho)),
            "tau10": float(np.interp(10.0, taus, rho)),
            "auc": float(np.trapezoid(rho, np.log10(taus)) / 3.0),
        }
    return out


# ---- convergence -------------------------------------------------------------------------------


def convergence_series(d, s, m, kind):
    """[(h, error)] along the dt or mesh series; h in the sense that smaller is finer."""
    pts = []
    if kind == "dt":
        for v, h in DT_SERIES:
            if (s, m, v) in d.err:
                pts.append((h, d.err[(s, m, v)]))
    else:
        for v, seg in MESH_SERIES:
            if (s, m, v) in d.err:
                pts.append((1.0 / seg, d.err[(s, m, v)]))
    return sorted(pts, reverse=True)  # coarse to fine


def classify(pts, n_expected):
    """(flag, slope, relative change of the two finest). Slope of log error against log h:
    positive means the error falls as h falls."""
    if len(pts) < n_expected or any(not math.isfinite(e) for _, e in pts):
        return "failed", math.nan, math.nan
    h = np.log([p[0] for p in pts])
    e = np.log([max(p[1], FLOOR) for p in pts])
    slope = float(np.polyfit(h, e, 1)[0])
    last, prev = pts[-1][1], pts[-2][1]
    change = abs(last - prev) / max(last, prev, FLOOR)
    errs = [max(p[1], FLOOR) for p in pts]
    if max(errs) <= 2 * FLOOR:
        return "flat", slope, change
    if slope < -SLOPE_TOL:
        return "diverging", slope, change
    if abs(slope) <= SLOPE_TOL:
        return "flat", slope, change
    if any(errs[i + 1] > 1.1 * errs[i] for i in range(len(errs) - 1)):
        return "non-monotone", slope, change
    return "converging", slope, change


def convergence(d):
    out = {}
    for kind, n_expected in (("dt", len(DT_SERIES)), ("mesh", len(MESH_SERIES))):
        for m in d.method_list:
            for s in d.scenarios:
                pts = convergence_series(d, s, m, kind)
                if len(pts) <= 1:
                    continue  # series not run for this scenario
                out[(kind, s, m)] = classify(pts, n_expected)
    return out


def convergence_summary(conv, d):
    """(kind, class, method) -> {flag: count, median_slope}."""
    groups = defaultdict(list)
    for (kind, s, m), (flag, slope, change) in conv.items():
        groups[(kind, d.cls[s], m)].append((flag, slope))
    out = {}
    for key, items in groups.items():
        counts = defaultdict(int)
        for f, _ in items:
            counts[f] += 1
        slopes = [sl for _, sl in items if math.isfinite(sl)]
        out[key] = {
            "flags": dict(counts),
            "median_slope": float(np.median(slopes)) if slopes else math.nan,
            "n": len(items),
        }
    return out


def stiffness(d):
    out = []
    for s in d.scenarios:
        for m in d.method_list:
            a, b = d.err.get((s, m, "dt0.2")), d.err.get((s, m, "stiff"))
            if a is not None and b is not None:
                out.append((s, m, a, b))
    return out


# ---- engines and perturbations -----------------------------------------------------------------


def independence(d):
    """law -> {engine pair: median |e1 - e2| / max(e1, e2, floor)} over base errors."""
    engines = sorted({e for e, _ in d.methods.values()})
    out = {}
    for law in LAWS:
        for i, e1 in enumerate(engines):
            for e2 in engines[i + 1 :]:
                m1, m2 = method(e1, law), method(e2, law)
                diffs = []
                for s in d.scenarios:
                    a, b = d.base(s, m1), d.base(s, m2)
                    if (
                        a is None
                        or b is None
                        or not (math.isfinite(a) and math.isfinite(b))
                    ):
                        continue
                    diffs.append(abs(a - b) / max(a, b, FLOOR))
                if diffs:
                    out[(law, f"{e1}/{e2}")] = (float(np.median(diffs)), len(diffs))
    return out


def perturbations(d):
    """(scenario, method) -> (min, median, max) over the finite perturbed draws, and the draw count."""
    out = {}
    for s in d.scenarios:
        for m in d.method_list:
            errs = [d.err.get((s, m, f"perturbed{k}")) for k in range(1, 9)]
            have = [e for e in errs if e is not None]
            fin = [e for e in have if math.isfinite(e)]
            if have:
                out[(s, m)] = (
                    (min(fin), float(np.median(fin)), max(fin), len(have))
                    if fin
                    else (math.nan,) * 3 + (len(have),)
                )
    return out


def perturbation_summary(pert):
    groups = defaultdict(list)
    for (s, m), (lo, med, hi, n) in pert.items():
        if math.isfinite(med):
            groups[m].append((hi - lo) / max(med, FLOOR))
    return {m: float(np.median(v)) for m, v in groups.items()}


def dominant_terms(d, terms):
    out = {}
    for s in d.scenarios:
        for m in d.method_list:
            t = terms.get((s, m, "base"))
            if t:
                top = max(t, key=lambda k: t[k] if math.isfinite(t[k]) else -1)
                out[f"{s}/{m}"] = {"term": top, "value": t[top]}
    return out


# ---- analysis and rendering --------------------------------------------------------------------


def analyse(rows, terms=None, B=10000, seed=0):
    d = Data(rows)
    rates = pass_rates(d)
    pair = paired_table(d, B, seed)
    robust = robust_table(d, B, seed)
    prof = {
        "A+B": profiles(d, ("A", "B"), d.method_list),
        "C": profiles(d, ("C",), [m for m in d.method_list if d.eligible(m, "C")]),
    }
    conv = convergence(d)
    return {
        "d": d,
        "rates": rates,
        "paired": pair,
        "robust": robust,
        "profiles": prof,
        "convergence": conv,
        "convergence_summary": convergence_summary(conv, d),
        "stiffness": stiffness(d),
        "independence": independence(d),
        "perturbations": perturbations(d),
        "dominant_terms": dominant_terms(d, terms or {}),
    }


def fmt(x, digits=3):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "n/a"
    if isinstance(x, float) and math.isinf(x):
        return "inf"
    return f"{x:.{digits}g}"


def md_table(header, rows):
    out = [
        "| " + " | ".join(header) + " |",
        "|" + "|".join("---" for _ in header) + "|",
    ]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return out


def render(res, source):
    d = res["d"]
    L = [
        "# Battery statistics\n",
        (
            f"Source: `{source}`. {len(d.rows)} runs, {len(d.scenarios)} scenarios, "
            f"{len(d.method_list)} methods. Error floor {FLOOR:g} (a practical tie); nan, timeout and failed "
            "count as infinite error; the uncapped paired comparisons leave those pairs out (column `excl`), "
            "the robust ones count them at the cap.\n"
        ),
    ]
    status = defaultdict(int)
    for r in d.rows:
        status[r["status"]] += 1
    L.append(
        "Runs by status: "
        + ", ".join(f"{k} {v}" for k, v in sorted(status.items()))
        + ".\n"
    )

    L += [
        "## Pass rates (base variant)\n",
        (
            "Share of scenarios with error at or below 1 / 5 / 20 %. Class C: laws and the "
            "Winkler native only.\n"
        ),
    ]
    for cls in "ABC":
        rows = [
            [
                m,
                v["n"],
                *[f"{v['pass'][k]:.0%}" for k in v["pass"]],
                fmt(v["median"]),
                fmt(v["geomean"]),
                v["failed"],
            ]
            for (c, m), v in res["rates"].items()
            if c == cls
        ]
        if rows:
            L += [
                f"### Class {cls}\n",
                *md_table(
                    [
                        "method",
                        "n",
                        "≤1 %",
                        "≤5 %",
                        "≤20 %",
                        "median",
                        "geo-mean",
                        "failed",
                    ],
                    rows,
                ),
                "",
            ]

    L += [
        "## Paired error ratios, uncapped\n",
        (
            "Ratio = error of the first over the second (below 1 favours the first), geometric "
            "mean over scenarios, 95 % interval from a stratified bootstrap (scenarios resampled within class).\n"
        ),
    ]
    rows = [
        [
            f"{p['a']} / {p['b']}",
            p["subset"],
            p["n"],
            p["excluded"],
            fmt(p["ratio"]),
            f"{fmt(10 ** p['ci'][0])} – {fmt(10 ** p['ci'][1])}",
            f"{p['wins']}/{p['losses']}",
        ]
        for p in res["paired"]
    ]
    L += [
        *md_table(
            ["pair", "subset", "n", "excl", "ratio", "95 % CI", "wins/losses"], rows
        ),
        "",
    ]

    def robust_rows(rows):
        return [
            [
                f"{p['a']} / {p['b']}",
                p["subset"],
                p["n"],
                fmt(p["ratio"]),
                f"{fmt(10 ** p['ci'][0])} – {fmt(10 ** p['ci'][1])}",
                fmt(p["p_holm"]) if p["p_holm"] is not None else fmt(p["p_boot"]),
                f"{fmt(p['loo'][0])} – {fmt(p['loo'][1])} ({p['loo_units'][0]}, {p['loo_units'][1]})",
                f"{p['wins']}/{p['ties']}/{p['losses']}",
                fmt(p["p_sign"]),
                f"{fmt(p['uncapped_ratio'])} [{fmt(p['uncapped_loo'][0])} – {fmt(p['uncapped_loo'][1])}]",
            ]
            for p in rows
        ]

    head = [
        "pair",
        "subset",
        "units",
        "ratio",
        "95 % CI",
        "p",
        "leave-one-out (unit removed)",
        "W/T/L",
        "sign p",
        "uncapped ratio [leave-one-out]",
    ]
    L += [
        "## Robust paired ratios\n",
        (
            f"Errors capped at {CAP:.0%}; nan, timeout and failed count as {CAP:.0%} for the method they "
            "belong to (no pair is left out); scenarios with a common stem and a number (`pieces*`) are one "
            f"unit; a ratio within a factor {TIE:g} of 1 is a tie. `A-hK` is class A without "
            + ", ".join(f"`{s}`" for s in PARAMETER_BOUND)
            + ". Leave-one-out: range of the ratio when one unit is removed. The last column is the ratio "
            "without the cap and with failed pairs left out, and its leave-one-out range.\n"
        ),
        "### Primary comparisons (p: bootstrap, computed without sampling, Holm-adjusted over this family)\n",
        *md_table(head, robust_rows([p for p in res["robust"] if p["primary"]])),
        "",
        "### Exploratory comparisons (p: bootstrap, unadjusted; not tests)\n",
        *md_table(head, robust_rows([p for p in res["robust"] if not p["primary"]])),
        "",
    ]

    L += [
        "## Performance profiles\n",
        (
            "Share of scenarios where a method is within a factor τ of the best "
            "(floored errors). `auc` is the area under the profile over τ = 1…1000 on a log axis.\n"
        ),
    ]
    for name, prof in res["profiles"].items():
        summ = profile_summary(prof)
        if summ:
            L += [
                f"### {name}\n",
                *md_table(
                    ["method", "best", "τ≤2", "τ≤10", "auc"],
                    [
                        [
                            m,
                            f"{v['best']:.0%}",
                            f"{v['tau2']:.0%}",
                            f"{v['tau10']:.0%}",
                            f"{v['auc']:.2f}",
                        ]
                        for m, v in summ.items()
                    ],
                ),
                "",
            ]

    L += [
        "## Convergence\n",
        (
            "Log-log slope of the error against the step (dt 2, 1, 0.5, 0.2, 0.1 ms) and against the "
            "mesh size (32, 64, 128 segments); positive means the error falls as the discretisation refines. "
            "Flags: converging, flat (|slope| ≤ 0.2 or errors at the floor), diverging, non-monotone, failed "
            "(a run is missing or not finite). Series cover only scenarios where they apply.\n"
        ),
    ]
    for kind in ("dt", "mesh"):
        rows = [
            [
                m,
                cls,
                v["n"],
                fmt(v["median_slope"]),
                ", ".join(f"{k} {n}" for k, n in sorted(v["flags"].items())),
            ]
            for (k, cls, m), v in sorted(
                res["convergence_summary"].items(), key=lambda kv: (kv[0][1], kv[0][2])
            )
            if k == kind
        ]
        if rows:
            L += [
                f"### By {'time step' if kind == 'dt' else 'mesh'}\n",
                *md_table(["method", "class", "n", "median slope", "flags"], rows),
                "",
            ]
    bad = [
        (k, v)
        for k, v in res["convergence"].items()
        if v[0] in ("diverging", "failed", "non-monotone")
    ]
    if bad:
        L += [
            "### Series to look at\n",
            *md_table(
                ["series", "scenario", "method", "flag", "slope", "change last two"],
                [
                    [k[0], k[1], k[2], v[0], fmt(v[1]), fmt(v[2])]
                    for k, v in sorted(bad)
                ],
            ),
            "",
        ]

    if res["stiffness"]:
        L += [
            "## Stiffness series\n",
            "Error at dt 0.2 ms against the same step with K × 10 (class C is not run).\n",
            *md_table(
                ["scenario", "method", "dt 0.2 ms", "K × 10"],
                [[s, m, fmt(a), fmt(b)] for s, m, a, b in res["stiffness"]],
            ),
            "",
        ]

    if res["independence"]:
        L += [
            "## Engine independence\n",
            (
                "Median of |e₁ − e₂| / max(e₁, e₂, floor) between engines running the same law: "
                "0 means the engine does not matter, 1 means they disagree entirely.\n"
            ),
            *md_table(
                ["law", "engines", "median", "n"],
                [
                    [k[0], k[1], fmt(v[0]), v[1]]
                    for k, v in sorted(res["independence"].items())
                ],
            ),
            "",
        ]

    summ = perturbation_summary(res["perturbations"])
    if summ:
        L += [
            "## Perturbations\n",
            "Median over scenarios of (max − min) / median error across the eight seeded draws.\n",
            *md_table(
                ["method", "relative spread"],
                [[m, fmt(v)] for m, v in sorted(summ.items())],
            ),
            "",
        ]
    return "\n".join(L) + "\n"


def jsonable(res):
    d = res["d"]
    return {
        "rates": {f"{c}/{m}": v for (c, m), v in res["rates"].items()},
        "paired": res["paired"],
        "robust": res["robust"],
        "robust_rules": {
            "cap": CAP,
            "tie": TIE,
            "floor": FLOOR,
            "parameter_bound": list(PARAMETER_BOUND),
            "primary": {
                "laws": list(PRIMARY_LAWS),
                "native": PRIMARY_NATIVE,
                "subsets": list(PRIMARY_SUBSETS),
            },
        },
        "profiles": {k: profile_summary(v) for k, v in res["profiles"].items()},
        "convergence": {
            f"{k[0]}/{k[1]}/{k[2]}": {"flag": v[0], "slope": v[1], "change": v[2]}
            for k, v in res["convergence"].items()
        },
        "convergence_summary": {
            f"{k[0]}/{k[1]}/{k[2]}": v for k, v in res["convergence_summary"].items()
        },
        "stiffness": [
            {"scenario": s, "method": m, "dt0.2": a, "stiff": b}
            for s, m, a, b in res["stiffness"]
        ],
        "independence": {
            f"{k[0]}/{k[1]}": {"median": v[0], "n": v[1]}
            for k, v in res["independence"].items()
        },
        "perturbations": {
            f"{k[0]}/{k[1]}": {"min": v[0], "median": v[1], "max": v[2], "n": v[3]}
            for k, v in res["perturbations"].items()
        },
        "dominant_terms": res["dominant_terms"],
        "scenarios": {s: d.cls[s] for s in d.scenarios},
    }


def figures(res, out_dir):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm

    d = res["d"]
    out_dir.mkdir(parents=True, exist_ok=True)

    def save(fig, name):
        extra = getattr(fig, "_legends", [])
        fig.savefig(out_dir / f"{name}.pdf", bbox_inches="tight", bbox_extra_artists=extra)
        fig.savefig(
            out_dir / f"{name}.png", dpi=150, bbox_inches="tight", bbox_extra_artists=extra
        )
        plt.close(fig)

    def style(m):
        e, c = d.methods[m]
        return {
            "color": colour(c),
            "linestyle": LINESTYLES.get(e, "-"),
            "marker": MARKERS.get(e, "o"),
            "markersize": 4,
            "label": m,
        }

    def grouped_legend(ax):
        """Two short legends outside the axes: colour = contact law, line style and marker = engine."""
        from matplotlib.lines import Line2D

        contacts = []
        for e, c in d.methods.values():
            key = "native" if c.startswith("native") else c
            if key not in contacts:
                contacts.append(key)
        engines = sorted({e for e, _ in d.methods.values()})
        first = ax.legend(
            [Line2D([], [], color=colour(c), lw=2.5) for c in contacts],
            contacts,
            title="contact (colour)",
            fontsize=8,
            title_fontsize=8,
            loc="upper left",
            bbox_to_anchor=(1.02, 1.0),
            frameon=False,
        )
        ax.add_artist(first)
        second = ax.legend(
            [
                Line2D([], [], color="0.3", ls=LINESTYLES.get(e, "-"), marker=MARKERS.get(e, "o"), ms=4)
                for e in engines
            ],
            engines,
            title="engine (line, marker)",
            fontsize=8,
            title_fontsize=8,
            loc="upper left",
            bbox_to_anchor=(1.02, 0.45),
            frameon=False,
        )
        ax.figure._legends = [first, second]

    # error heatmap per class
    for cls in "ABC":
        scens = d.scenarios_of(cls)
        meths = [m for m in d.method_list if d.eligible(m, cls)]
        if not scens or not meths:
            continue
        grid = np.array(
            [
                [d.base(s, m) if d.base(s, m) is not None else np.nan for s in scens]
                for m in meths
            ],
            float,
        )
        shown = np.where(np.isfinite(grid), np.maximum(grid, FLOOR), np.nan)
        fig, ax = plt.subplots(
            figsize=(max(4, 0.45 * len(scens) + 2), 0.3 * len(meths) + 1.5)
        )
        cmap = plt.get_cmap("viridis_r").with_extremes(bad="#999999")
        im = ax.imshow(
            shown, aspect="auto", cmap=cmap, norm=LogNorm(vmin=FLOOR, vmax=1.0)
        )
        ax.set_xticks(range(len(scens)), scens, rotation=60, ha="right")
        ax.set_yticks(range(len(meths)), meths)
        ax.set_title(f"Class {cls}: error (grey: failed)")
        fig.colorbar(im, ax=ax, label="error")
        save(fig, f"heatmap_{cls}")

    # performance profiles
    for name, prof in res["profiles"].items():
        if not prof:
            continue
        fig, ax = plt.subplots(figsize=(7.5, 3.8))
        for m, (taus, rho) in prof.items():
            e, c = d.methods[m]
            ax.step(
                taus,
                rho,
                where="post",
                color=colour(c),
                linestyle=LINESTYLES.get(e, "-"),
                label=m,
            )
        ax.set_xscale("log")
        ax.set_xlabel("τ (factor over the best error)")
        ax.set_ylabel("share of scenarios")
        ax.set_title(f"Performance profile, {name}")
        grouped_legend(ax)
        save(fig, f"profile_{name.replace('+', '')}")

    # convergence: error normalised to the coarsest point, median over scenarios
    for kind, xlabel in (("dt", "time step [ms]"), ("mesh", "segments")):
        fig, ax = plt.subplots(figsize=(7.5, 3.8))
        plotted = False
        for m in d.method_list:
            curves = defaultdict(list)
            for s in d.scenarios:
                pts = convergence_series(d, s, m, kind)
                if len(pts) <= 1 or any(not math.isfinite(e) for _, e in pts):
                    continue
                base = max(pts[0][1], FLOOR)
                for h, e in pts:
                    curves[h].append(max(e, FLOOR) / base)
            if curves:
                hs = sorted(curves, reverse=True)
                xs = hs if kind == "dt" else [1 / h for h in hs]
                ax.plot(xs, [np.median(curves[h]) for h in hs], **style(m))
                plotted = True
        if plotted:
            ax.set_xscale("log")
            ax.set_yscale("log")
            ax.set_xlabel(xlabel)
            ax.set_ylabel("error / error at the coarsest (median)")
            grouped_legend(ax)
            save(fig, f"convergence_{kind}")
        else:
            plt.close(fig)

    # work-precision over the time step: median wall time against median error
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    plotted = False
    for m in d.method_list:
        xs, ys = [], []
        for v, _ in DT_SERIES:
            walls = [
                d.wall[(s, m, v)]
                for s in d.scenarios
                if (s, m, v) in d.wall
                and math.isfinite(d.err[(s, m, v)])
                and math.isfinite(d.wall[(s, m, v)])
            ]
            errs = [
                max(d.err[(s, m, v)], FLOOR)
                for s in d.scenarios
                if (s, m, v) in d.err and math.isfinite(d.err[(s, m, v)])
            ]
            if walls and errs:
                xs.append(np.median(walls))
                ys.append(np.median(errs))
        if len(xs) > 1:
            ax.plot(xs, ys, **style(m))
            plotted = True
    if plotted:
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("median wall time per run [s]")
        ax.set_ylabel("median error")
        grouped_legend(ax)
        save(fig, "work_precision")
    else:
        plt.close(fig)

    # forest plot of the paired ratios on A+B
    rows = [p for p in res["paired"] if p["subset"] == "A+B"]
    if rows:
        fig, ax = plt.subplots(figsize=(5.5, 0.22 * len(rows) + 1.2))
        for i, p in enumerate(rows):
            ax.errorbar(
                10 ** p["log10_ratio"],
                i,
                xerr=[
                    [10 ** p["log10_ratio"] - 10 ** p["ci"][0]],
                    [10 ** p["ci"][1] - 10 ** p["log10_ratio"]],
                ],
                fmt="o",
                color="#0072B2",
                ms=3,
                capsize=2,
            )
        ax.axvline(1, color="k", lw=0.6)
        ax.set_xscale("log")
        ax.set_yticks(
            range(len(rows)), [f"{p['a']} / {p['b']}" for p in rows], fontsize=7
        )
        ax.set_xlabel("error ratio (below 1 favours the first), classes A+B")
        save(fig, "forest_AB")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv")
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--bootstrap", type=int, default=10000)
    args = ap.parse_args(argv)
    path = Path(args.csv)
    rows = load(path)
    if not rows:
        sys.exit(f"{path}: no rows")
    res = analyse(
        rows, load_terms(path.with_name("terms.csv")), args.bootstrap, args.seed
    )
    text = render(res, path.name)
    (path.parent / "stats.md").write_text(text)
    (path.parent / "stats.json").write_text(
        json.dumps(jsonable(res), indent=1, default=float)
    )
    if not args.no_figures:
        figures(res, path.parent / "figures")
    print(text)


if __name__ == "__main__":
    main()
