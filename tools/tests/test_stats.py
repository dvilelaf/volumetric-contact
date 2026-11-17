"""tools/stats.py on synthetic batteries, read back from stats.json."""

import csv
import itertools
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import stats

HEADER = [
    "engine",
    "contact",
    "scenario",
    "variant",
    "status",
    "error",
    "realtime_ratio",
    "wall_s",
    "steps",
    "dt",
    "segments",
    "stiffness_scale",
    "class",
]
AB = [(f"a{i:02d}", "A") for i in range(10)] + [(f"b{i:02d}", "B") for i in range(4)]
CLS_C = [(f"c{i:02d}", "C") for i in range(4)]
ENGINES = ("drake", "chrono")
CONTACTS = ("gonthier", "gonthier-thesis", "distributed", "native")


def write(
    tmp_path, error, variants=("base",), scenarios=AB + CLS_C, status=lambda *a: "ok"
):
    """battery.csv from error(engine, contact, scenario, variant)."""
    path = tmp_path / "battery.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        for e in ENGINES:
            for c in CONTACTS:
                for s, cls in scenarios:
                    for v in variants:
                        st = status(e, c, s, v)
                        err = error(e, c, s, v)
                        w.writerow(
                            [
                                e,
                                c,
                                s,
                                v,
                                st,
                                "" if st in ("timeout", "failed") else err,
                                0.1,
                                1.0,
                                1000,
                                0.001,
                                0,
                                1,
                                cls,
                            ]
                        )
    return path


def run(tmp_path, error, **kw):
    path = write(tmp_path, error, **kw)
    stats.main([str(path), "--no-figures", "--bootstrap", "300"])
    return json.loads((tmp_path / "stats.json").read_text())


def test_pass_rates_per_class(tmp_path):
    def error(e, c, s, v):
        return 0.5 if (c == "native" and s == "a00") else 0.004

    res = run(tmp_path, error)
    assert res["rates"]["A/drake-gonthier"]["pass"]["1%"] == 1.0
    assert res["rates"]["A/drake-native"]["pass"]["1%"] == 0.9
    assert res["rates"]["A/drake-native"]["pass"]["20%"] == 0.9


def test_class_c_leaves_out_non_winkler_natives(tmp_path):
    res = run(tmp_path, lambda *a: 0.01)
    assert "C/drake-native" in res["rates"]
    assert "C/chrono-native" not in res["rates"]
    assert "C/chrono-gonthier" in res["rates"]


def test_paired_ratio_and_exclusions(tmp_path):
    def error(e, c, s, v):
        if c == "distributed":
            return math.nan if s == "a00" else 0.02
        return 0.2

    res = run(tmp_path, error)
    p = next(
        p
        for p in res["paired"]
        if p["a"] == "drake-distributed"
        and p["b"] == "drake-gonthier"
        and p["subset"] == "A+B"
    )
    assert abs(p["ratio"] - 0.1) < 1e-9
    assert p["excluded"] == 1 and p["n"] == 13
    assert p["ci"][0] <= p["log10_ratio"] <= p["ci"][1]


def test_bootstrap_is_reproducible(tmp_path):
    def error(e, c, s, v):
        return (
            0.01
            * (1 + (hash(s) % 7) * 0.0 + int(s[1:]) % 3)
            * (2 if c == "distributed" else 1)
        )

    first = run(tmp_path, error)
    second = run(tmp_path, error)
    assert first["paired"] == second["paired"]


def test_floor_makes_tiny_errors_tie(tmp_path):
    res = run(tmp_path, lambda e, c, s, v: 1e-6 if c == "gonthier" else 5e-4)
    p = next(
        p
        for p in res["paired"]
        if p["a"] == "drake-distributed"
        and p["b"] == "drake-gonthier"
        and p["subset"] == "A+B"
    )
    assert abs(p["ratio"] - 1.0) < 1e-9


def test_convergence_order_two_in_dt(tmp_path):
    dts = {"dt2": 2.0, "base": 1.0, "dt0.5": 0.5, "dt0.2": 0.2, "dt0.1": 0.1}
    res = run(
        tmp_path,
        lambda e, c, s, v: 0.2 * dts[v] ** 2,
        variants=tuple(dts),
        scenarios=AB[:3],
    )
    item = res["convergence"]["dt/a00/drake-gonthier"]
    assert item["flag"] == "converging"
    assert abs(item["slope"] - 2.0) < 1e-6


def test_convergence_flags(tmp_path):
    dts = {"dt2": 2.0, "base": 1.0, "dt0.5": 0.5, "dt0.2": 0.2, "dt0.1": 0.1}

    def error(e, c, s, v):
        if s == "a00":
            return 0.05 / dts[v]  # diverging
        if s == "a01":
            return 0.05  # flat
        return 0.05 * dts[v] ** 2

    def status(e, c, s, v):
        return "timeout" if (s == "a02" and v == "dt0.1") else "ok"

    res = run(tmp_path, error, variants=tuple(dts), scenarios=AB[:3], status=status)
    assert res["convergence"]["dt/a00/drake-gonthier"]["flag"] == "diverging"
    assert res["convergence"]["dt/a01/drake-gonthier"]["flag"] == "flat"
    assert res["convergence"]["dt/a02/drake-gonthier"]["flag"] == "failed"


def test_mesh_series_uses_segments(tmp_path):
    seg = {"mesh32": 32, "base": 64, "mesh128": 128}
    res = run(
        tmp_path, lambda e, c, s, v: 0.5 / seg[v], variants=tuple(seg), scenarios=AB[:2]
    )
    item = res["convergence"]["mesh/a00/chrono-distributed"]
    assert item["flag"] == "converging" and abs(item["slope"] - 1.0) < 1e-6


def test_profile_and_independence(tmp_path):
    res = run(
        tmp_path,
        lambda e, c, s, v: 0.01 if c == "gonthier" else 0.1 if e == "drake" else 0.2,
    )
    assert res["profiles"]["A+B"]["drake-gonthier"]["best"] == 1.0
    assert res["profiles"]["A+B"]["drake-native"]["best"] == 0.0
    assert res["independence"]["distributed/chrono/drake"]["median"] == 0.5


def test_old_format_without_status_is_accepted(tmp_path):
    path = tmp_path / "battery.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["engine", "contact", "scenario", "variant", "error", "realtime_ratio"]
        )
        w.writerow(["drake", "gonthier", "block", "base", 0.02, 0.1])
        w.writerow(["drake", "gonthier", "disk", "base", "nan", 0.1])
    stats.main([str(path), "--no-figures", "--bootstrap", "10"])
    res = json.loads((tmp_path / "stats.json").read_text())
    assert res["rates"]["A/drake-gonthier"]["pass"]["5%"] == 1.0
    assert res["rates"]["B/drake-gonthier"]["failed"] == 1


def test_figures_are_written(tmp_path):
    path = write(
        tmp_path, lambda *a: 0.02, variants=("base", "dt2", "dt0.5", "dt0.2", "dt0.1")
    )
    stats.main([str(path), "--bootstrap", "10"])
    assert (tmp_path / "figures" / "heatmap_A.pdf").exists()
    assert (tmp_path / "figures" / "profile_AB.png").exists()


# ---- robust comparisons ----------------------------------------------------------------------


# Names without a trailing number: scenarios that differ only in one are a single cluster.
LETTERS = [(f"s{chr(97 + i)}", "A") for i in range(10)] + [
    (f"t{chr(97 + i)}", "B") for i in range(4)
]


def robust_row(res, a, b, subset):
    return next(
        p
        for p in res["robust"]
        if p["a"] == a and p["b"] == b and p["subset"] == subset
    )


def test_robust_caps_errors_and_counts_failures_on_both_sides(tmp_path):
    # distributed: lost (6000 %) in a00, no value in a01; native: no value in a02.
    def error(e, c, s, v):
        if c == "distributed" and s == "sa":
            return 60.0
        return 0.01

    def status(e, c, s, v):
        if (c == "distributed" and s == "sb") or (c == "native" and s == "sc"):
            return "timeout"
        return "ok"

    res = run(tmp_path, error, status=status, scenarios=LETTERS)
    p = robust_row(res, "drake-distributed", "drake-native", "A")
    # ten units: +2 decades (capped at 100 % against 1 %), +2 (failure of a), -2 (failure of b)
    assert p["n"] == 10
    assert abs(p["log10_ratio"] - 2.0 / 10) < 1e-9
    assert (p["wins"], p["ties"], p["losses"]) == (1, 7, 2)
    # the uncapped ratio leaves the two failed pairs out and keeps the 6000 %
    old = next(
        q
        for q in res["paired"]
        if q["a"] == "drake-distributed"
        and q["b"] == "drake-native"
        and q["subset"] == "A"
    )
    assert old["excluded"] == 2 and old["log10_ratio"] > p["log10_ratio"]


def test_robust_counts_a_family_of_scenarios_once(tmp_path):
    scenarios = [("block", "A"), ("push", "A")] + [
        (f"pieces{n}", "A") for n in (1, 2, 8, 64)
    ]

    def error(e, c, s, v):
        return 0.1 if (c == "distributed" and s.startswith("pieces")) else 0.01

    res = run(tmp_path, error, scenarios=scenarios)
    p = robust_row(res, "drake-distributed", "drake-gonthier", "A")
    assert p["n"] == 3  # block, push and the cluster
    assert abs(p["log10_ratio"] - 1.0 / 3) < 1e-9
    assert (
        stats.cluster_of("pieces64") == "pieces"
        and stats.cluster_of("block") == "block"
    )


def test_tie_band_and_sign_test(tmp_path):
    def error(e, c, s, v):
        if c == "distributed":
            return (
                0.012 if s[1] < "g" else 0.002
            )  # within 1.25 of 0.01, or five times smaller
        return 0.01

    res = run(tmp_path, error, scenarios=LETTERS)
    p = robust_row(res, "drake-distributed", "drake-gonthier", "A")
    assert (p["wins"], p["ties"], p["losses"]) == (4, 6, 0)
    assert abs(p["p_sign"] - 2 * 0.5**4) < 1e-12
    assert stats.sign_test(0, 0) == 1.0 and stats.sign_test(5, 5) == 1.0


def test_leave_one_out_names_the_scenario_that_drives_a_ratio(tmp_path):
    def error(e, c, s, v):
        return 1.0 if (c == "distributed" and s == "sd") else 0.01

    res = run(tmp_path, error, scenarios=LETTERS)
    p = robust_row(res, "drake-distributed", "drake-gonthier", "A")
    assert abs(p["ratio"] - 10 ** (2 / 10)) < 1e-9
    assert abs(p["loo"][0] - 1.0) < 1e-9 and p["loo_units"][0] == "sd"
    assert abs(p["loo"][1] - 10 ** (2 / 9)) < 1e-9


def test_parameter_bound_scenarios_are_left_out_of_the_reduced_class_a(tmp_path):
    scenarios = [("block", "A"), ("push", "A"), ("bowl", "A"), ("edge", "A")]
    res = run(
        tmp_path,
        lambda e, c, s, v: (
            0.3 if (c == "distributed" and s in ("bowl", "edge")) else 0.03
        ),
        scenarios=scenarios,
    )
    assert robust_row(res, "drake-distributed", "drake-gonthier", "A")["n"] == 4
    reduced = robust_row(res, "drake-distributed", "drake-gonthier", "A-hK")
    assert reduced["n"] == 2 and abs(reduced["ratio"] - 1.0) < 1e-9


def test_only_the_primary_family_is_holm_adjusted(tmp_path):
    res = run(
        tmp_path,
        lambda e, c, s, v: 0.002 if c == "distributed" else 0.02,
        scenarios=LETTERS,
    )
    primary = [p for p in res["robust"] if p["primary"]]
    # distributed/gonthier once (chrono) and the two laws against each engine's `native`, A and A+B
    assert len(primary) == 2 * (1 + 2 * 2)
    assert all(p["p_holm"] is not None and p["p_holm"] >= p["p_boot"] for p in primary)
    assert all(p["p_holm"] is None for p in res["robust"] if not p["primary"])
    assert stats.holm([0.01, 0.04, 0.03]) == [0.03, 0.06, 0.06]


def test_primary_p_values_are_the_bootstrap_distribution_itself():
    vals, strata = [-0.9, -0.2, 0.35, -1.4, 0.6, 0.11], list("AAAABB")
    sums = [
        sum(a) + sum(b)
        for a in itertools.product(vals[:4], repeat=4)
        for b in itertools.product(vals[4:], repeat=2)
    ]
    exact = 2 * min(sum(x <= 0 for x in sums), sum(x >= 0 for x in sums)) / len(sums)
    p, width = stats.bootstrap_p(vals, strata)
    assert abs(p - exact) <= width + 1e-12 and width < 1e-9
    assert stats.bootstrap_p([0.3, 0.3], ["A", "A"]) == (0.0, 0.0)
    assert stats.bootstrap_p([1.0, -1.0], ["A", "B"])[0] == 1.0  # one unit per class: one sum, zero


def test_primary_p_values_do_not_depend_on_the_seed(tmp_path):
    def error(e, c, s, v):
        return 0.01 * (1 + int(s[1:]) % 3) * (2 if c == "distributed" else 1)

    rows = stats.load(write(tmp_path, error))
    a, b = (
        [(p["a"], p["b"], p["subset"], p["p_holm"]) for p in stats.analyse(rows, B=50, seed=k)["robust"] if p["primary"]]
        for k in (0, 1)
    )
    assert a and a == b
