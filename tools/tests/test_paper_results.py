import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import paper_results as pr

ROOT = Path(__file__).resolve().parents[2]
HEADER = "engine,contact,scenario,variant,status,error,realtime_ratio,wall_s,steps,dt,segments,stiffness_scale,class"


def write_battery(tmp_path, timing=False):
    """A battery of three scenarios: the laws on three engines, two natives, one failure of each kind."""
    results, paper = tmp_path / "results", tmp_path / "paper"
    (paper / "sections").mkdir(parents=True)
    results.mkdir()
    rows = []
    errors = {"block": ("A", 0.004), "disk": ("B", 0.0012), "tower": ("A", 0.035)}
    for scenario, (cls, err) in errors.items():
        for i, engine in enumerate(pr.ENGINES):
            for law, factor in (("gonthier", 2.0), ("distributed", 1.0), ("pcm", 30.0)):
                for variant in ("base", "dt2", "dt0.5", "dt0.2", "dt0.1", "stiff"):
                    e = err * factor * (1 + 1e-4 * i)
                    rows.append(f"{engine},{law},{scenario},{variant},ok,{e},0.01,0.01,1000,0.001,0,1,{cls}")
        rows.append(f"drake,native,{scenario},base,ok,{err * 3},2.5,2.5,1000,0.001,0,1,{cls}")
    rows.append("chrono,native,block,base,ok,0.0005,0.02,0.02,1000,0.001,0,1,A")
    rows.append("chrono,native,disk,base,nan,nan,0.02,0.02,1000,0.001,0,1,B")
    rows.append("chrono,native,tower,base,timeout,nan,0.02,0.02,1000,0.001,0,1,A")
    rows.append("mujoco,native,tower,base,failed,nan,0.02,0.02,1000,0.001,0,1,A")
    (results / "battery.csv").write_text(HEADER + "\n" + "\n".join(rows) + "\n")
    (results / "terms.csv").write_text(
        "scenario,engine,contact,variant,term,value\nblock,chrono,distributed,base,velocity,0.004\n"
    )
    stats = {
        "paired": [
            {"a": "chrono-distributed", "b": "chrono-gonthier", "subset": "A+B", "n": 3, "excluded": 0,
             "log10_ratio": -0.301, "ci": [-0.5, -0.1], "ratio": 0.5, "wins": 3, "losses": 0},
            {"a": "drake-distributed", "b": "drake-native", "subset": "A", "n": 2, "excluded": 0,
             "log10_ratio": -0.477, "ci": [-1.0, 0.2], "ratio": 0.333, "wins": 2, "losses": 0},
        ],
        "convergence": {"dt/block/chrono-distributed": {"flag": "flat", "slope": 0.01, "change": 0.0}},
    }  # fmt: skip
    (results / "stats.json").write_text(json.dumps(stats))
    provenance = {
        "commit": "abc1234", "commit_baselines": "def5678", "date": "1 January 2026", "spot_runs": 18,
        "spot_max_relative_difference": 3e-7, "heldout_first_run": {"tower": {"gonthier": 840.0, "distributed": 0.5}},
    }  # fmt: skip
    (paper / "provenance.json").write_text(json.dumps(provenance))
    if timing:
        (results / "timing.csv").write_text(
            HEADER + "\n" + "\n".join(f"chrono,distributed,block,base,ok,0.004,{w},1,1000,0.001,0,1,A" for w in (0.5, 0.3, 0.4))
            + "\nchrono,native,disk,base,nan,nan,0.07,1,1000,0.001,0,1,B\n"
        )
    return results, paper


def test_numbers_are_rounded_for_print():
    assert [pr.pct(e) for e in (0.00004, 0.0012, 0.035, 0.146, 4.2, 6300)] == [
        "$<$0.01", "0.12", "3.5", "15", "420", "$>$1000",
    ]  # fmt: skip
    assert [pr.sig(x) for x in (0.484, 5.14, 20.6, 0.0353, 0.997)] == ["0.48", "5.1", "21", "0.035", "1.00"]
    assert pr.sci(3e-7) == "$3\\times10^{-7}$"
    assert pr.sci(1e-5) == "$10^{-5}$"


def test_only_the_keys_the_paper_uses_are_written(tmp_path):
    results, paper = write_battery(tmp_path)
    (paper / "sections" / "05-results.tex").write_text(
        "\\result{battery/runs} \\result{battery/failed} \\result{err/block/distributed}\n"
        "\\result{err/disk/smc} \\result{ratio/distributed-vs-gonthier/A+B/full}\n"
        "\\result{ratio/distributed-vs-hydro/A/hi} \\result{status/natives/notok}\n"
        "\\result{pass/A/distributed/5} \\result{first/tower/gonthier} \\result{term/block/distributed/velocity}\n"
        "% \\result{not/a/key} in a comment is not a use\n"
    )
    pr.generate(results, paper, with_figures=False)
    text = (paper / "generated" / "numbers.tex").read_text()
    lines = text.splitlines()
    assert "abc1234" in lines[0] and pr.Battery(results, paper).sha256 in lines[0]
    assert len(lines) == 11
    assert "\\setresult{battery/failed}{3}" in text
    assert "\\setresult{err/block/distributed}{0.40\\,\\%}" in text
    assert "\\setresult{err/disk/smc}{\\textit{nan}}" in text  # a status, never a number
    assert "\\setresult{ratio/distributed-vs-gonthier/A+B/full}{0.50 (0.32--0.79)}" in text
    assert "\\setresult{ratio/distributed-vs-hydro/A/hi}{1.6}" in text  # the law on the native's engine
    assert "\\setresult{status/natives/notok}{3}" in text
    assert "\\setresult{pass/A/distributed/5}{2 of 2}" in text
    assert "\\setresult{first/tower/gonthier}{$>$1000\\,\\%}" in text
    assert "\\setresult{term/block/distributed/velocity}{0.40\\,\\%}" in text


def test_a_key_that_does_not_exist_stops_the_build(tmp_path):
    results, paper = write_battery(tmp_path)
    (paper / "sections" / "05-results.tex").write_text("\\result{err/nowhere/distributed}")
    with pytest.raises(SystemExit, match="err/nowhere/distributed"):
        pr.generate(results, paper, with_figures=False)


def test_tables_print_the_status_of_a_run_that_did_not_finish(tmp_path):
    results, paper = write_battery(tmp_path)
    pr.generate(results, paper, with_figures=False)
    tables = paper / "generated" / "tables"
    status = (tables / "status.tex").read_text()
    for needle in ("Chrono SMC & nan & 1 & \\scn{disk} 1/1", "Chrono SMC & timeout & 1", "MuJoCo soft & failed & 1"):
        assert needle in status
    row = next(line for line in (tables / "errors_B.tex").read_text().splitlines() if "scn{disk}" in line)
    assert "\\textit{nan}" in row and "0.12" in row
    assert "\\textbf{0.50}" in (tables / "ratios.tex").read_text()  # an interval that excludes 1
    assert "0.33 & 0.10--1.6" in (tables / "ratios.tex").read_text()  # one that does not


def test_engine_independence_is_the_spread_over_the_engines(tmp_path):
    results, paper = write_battery(tmp_path)
    spread = pr.independence(pr.Battery(results, paper), "distributed")
    assert spread["tower"][0] == pytest.approx(0.035 * 2e-4)
    assert spread["tower"][1] == pytest.approx(2e-4, rel=1e-3)


def test_costs_come_from_the_timing_pass_when_it_exists(tmp_path):
    results, paper = write_battery(tmp_path)
    numbers = pr.Numbers(pr.Battery(results, paper))
    assert "shared machine" in numbers.resolve("cost/source")
    assert numbers.resolve("cost/block/distributed") == "0.010"
    results, paper = write_battery(tmp_path / "timed", timing=True)
    numbers = pr.Numbers(pr.Battery(results, paper))
    assert "idle machine" in numbers.resolve("cost/source")
    assert numbers.resolve("cost/block/distributed") == "0.40"  # the median of the repeats
    assert numbers.resolve("cost/disk/smc") == "0.070"  # a run without an error value still has a cost


def test_a_run_stopped_by_its_limit_gives_a_lower_bound_of_its_cost(tmp_path):
    results, paper = write_battery(tmp_path)
    battery = results / "battery.csv"
    battery.write_text(battery.read_text().replace(
        "chrono,native,tower,base,timeout,nan,0.02,0.02", "chrono,native,tower,base,timeout,,,3600"))
    numbers = pr.Numbers(pr.Battery(results, paper))
    assert numbers.resolve("cost/bound/tower-smc") == "3600"  # 3600 s of wall for 1000 steps of 1 ms
    with pytest.raises(KeyError):
        numbers.resolve("cost/bound/block-smc")


def test_the_output_is_deterministic(tmp_path):
    results, paper = write_battery(tmp_path)
    pr.generate(results, paper)
    first = {p.name: p.read_bytes() for p in (paper / "generated").rglob("*") if p.is_file()}
    pr.generate(results, paper)
    second = {p.name: p.read_bytes() for p in (paper / "generated").rglob("*") if p.is_file()}
    assert first == second
    assert {"heatmap_A.pdf", "profile_AB.pdf", "ablation.pdf", "numbers.tex", "errors_A.tex"} <= set(first)


def test_every_key_of_the_paper_resolves_on_the_committed_results():
    if not (ROOT / "results" / "battery.csv").exists():
        pytest.skip("no battery in this checkout")
    battery = pr.Battery(ROOT / "results", ROOT / "paper")
    text = pr.numbers_tex(battery, pr.used_keys(ROOT / "paper"))
    committed = ROOT / "paper" / "generated" / "numbers.tex"
    assert committed.read_text() == text, "paper/generated is stale: run uv run tools/paper_results.py"


# ---- the revision: robust comparisons, band, perturbations, triangulation ---------------------------


def robust_row(a, b, subset, primary, p_holm):
    return {
        "a": a, "b": b, "subset": subset, "primary": primary, "n": 3, "log10_ratio": -0.301, "ratio": 0.5,
        "ci": [-0.6, -0.1], "p_boot": 0.01, "p_holm": p_holm, "wins": 2, "ties": 1, "losses": 0, "p_sign": 0.5,
        "loo": [0.4, 0.7], "loo_units": ["block", "tower"], "uncapped_ratio": 0.5, "uncapped_n": 3,
        "uncapped_loo": [0.4, 0.7], "uncapped_loo_units": ["block", "tower"],
    }  # fmt: skip


def with_robust(tmp_path):
    results, paper = write_battery(tmp_path)
    stats = json.loads((results / "stats.json").read_text())
    stats["robust"] = [
        robust_row("chrono-distributed", "chrono-gonthier", "A+B", True, 0.03),
        robust_row("drake-distributed", "drake-native", "A", True, 0.4),
    ]
    stats["robust_rules"] = {"cap": 1.0, "tie": 1.25, "floor": 1e-3}
    (results / "stats.json").write_text(json.dumps(stats))
    return results, paper


def test_robust_keys_and_the_primary_table(tmp_path):
    results, paper = with_robust(tmp_path)
    numbers = pr.Numbers(pr.Battery(results, paper))
    assert numbers.resolve("robust/distributed-vs-gonthier/A+B/full") == "0.50 (0.25--0.79)"
    assert numbers.resolve("robust/distributed-vs-gonthier/A+B/holm") == "0.030"
    assert numbers.resolve("robust/distributed-vs-hydro/A/wtl") == "2/1/0"
    assert numbers.resolve("robust/distributed-vs-hydro/A/loo") == "0.40--0.70"
    assert numbers.resolve("primary/n") == "2" and numbers.resolve("primary/significant") == "1"
    assert numbers.resolve("rules/cap") == "100\\,\\%" and numbers.resolve("rules/tie") == "1.25"
    with pytest.raises(KeyError):
        numbers.resolve("robust/distributed-vs-smc/A/full")  # not in stats.json: no number
    table = pr.table_ratios_primary(pr.Battery(results, paper))
    assert "\\textbf{0.50} (0.25--0.79) & 0.030" in table  # significant after Holm: bold
    assert "Dis / Dr-h & 0.33 (0.10--1.6) & 0.50 (0.25--0.79) & 0.40" in table  # the uncapped estimate next to it


def test_compact_primary_table_and_the_error_tables_by_class(tmp_path):
    results, paper = with_robust(tmp_path)
    b = pr.Battery(results, paper)
    table = pr.table_primary_compact(b)
    assert "Dis / Gon & -- & -- & \\textbf{0.50} (0.25--0.79) & 0.030" in table  # significant after Holm: bold
    assert "Dis / Dr-h & 0.50 (0.25--0.79) & 0.40 & -- & --" in table  # a pair missing in a subset prints no number
    pr.generate(results, paper, with_figures=False)
    by_class = (paper / "generated" / "tables" / "errors_class_A.tex").read_text()
    assert "\\scn{block}" in by_class and "\\scn{tower}" in by_class and "\\scn{disk}" not in by_class
    assert "\\textit{timeout}" in by_class  # a run that did not finish prints its status
    assert (paper / "generated" / "tables" / "primary_compact.tex").read_text().endswith(table)


def test_band_position_needs_the_sign_only_between_the_two_sides():
    class B:
        def __init__(self, e):
            self.error = e

        def e(self, scenario, key):
            return self.error

    # disk: band −15 % / +33 %
    assert pr.band_status(B(0.10), "disk", "gonthier") == "inside"
    assert pr.band_status(B(0.20), "disk", "gonthier") == "undetermined"
    assert pr.band_status(B(0.40), "disk", "gonthier") == "outside"
    assert pr.band_status(B(float("inf")), "disk", "gonthier") == "none"
    assert pr.band_status(B(0.01), "ring", "gonthier") == "undetermined"  # a side of zero width


def test_reduced_class_a_and_perturbed_runs(tmp_path):
    results, paper = write_battery(tmp_path)
    rows = (results / "battery.csv").read_text().rstrip("\n").split("\n")
    rows += ["chrono,native,bowl,base,ok,0.3,0.02,0.02,1000,0.001,0,1,A"]
    rows += [f"chrono,native,block,perturbed{k},ok,{0.01 * k},0.02,0.02,1000,0.001,0,1,A" for k in range(1, 8)]
    rows += ["chrono,native,block,perturbed8,nan,nan,0.02,0.02,1000,0.001,0,1,A"]
    (results / "battery.csv").write_text("\n".join(rows) + "\n")
    b = pr.Battery(results, paper)
    assert "bowl" in b.of_class("A") and "bowl" not in b.of_class("Ar")
    numbers = pr.Numbers(b)
    assert numbers.resolve("pertrange/block/smc/range") == "1.0--7.0\\,\\%"
    assert numbers.resolve("pertrange/block/smc/median") == "4.0\\,\\%"
    assert numbers.resolve("pertrange/block/smc/lost") == "1"
    assert ("block", "smc") in pr.scattered(b, ["smc"])
    assert "\\scn{block} & Chrono SMC & 0.05 & 4.0 & 1.0--7.0 & 1" in pr.table_perturbations(b)


def test_triangulation_keys_come_from_the_experiment(tmp_path):
    results, paper = write_battery(tmp_path)
    (results / "ablation_triangulation.csv").write_text(
        "shape,case,triangulation,parameter,triangles,min_angle_deg,rule,moment_error,force_error\n"
        "disk,spin,rim-fan,64,62,2.8,closed,-1.0e-16,2.0e-17\n"
        "disk,spin,rim-fan,64,62,2.8,centroid,-1.791e-01,3.0e-17\n"
    )
    numbers = pr.Numbers(pr.Battery(results, paper))
    assert numbers.resolve("tri/disk/spin/rim-fan/64/centroid/moment") == "$-$18\\,\\%"
    assert numbers.resolve("tri/disk/spin/rim-fan/64/centroid/triangles") == "62"
    assert numbers.resolve("tri/closed-spread") == "$10^{-16}$"
    with pytest.raises(KeyError):
        numbers.resolve("tri/disk/spin/centre-fan/64/centroid/moment")


def test_a_count_of_a_group_needs_every_listed_method_within_the_threshold(tmp_path):
    results, paper = write_battery(tmp_path)
    numbers = pr.Numbers(pr.Battery(results, paper))
    assert numbers.resolve("within/oneplane/distributed/1") == "1 of 7"  # block, 0.4 %
    assert numbers.resolve("within/oneplane/distributed/0.3") == "0 of 7"
    assert numbers.resolve("within/oneplane/distributed+gonthier/0.5") == "0 of 7"  # gonthier: 0.8 %
