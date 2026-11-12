# The paper

Sources of the paper, a technical report published on the project's site and not submitted to a
journal (the class is the Springer Nature template `sn-jnl` with the reference style
`sn-mathphys-num`, kept as it is).

```
scripts/paper.sh          build paper/main.pdf and paper/supplement.pdf (not committed)
scripts/paper.sh check    numerical checks of the derivations (paper/checks/check_*.py)
scripts/paper.sh refs     re-verify references.bib online and print the report
scripts/paper.sh clean
```

The build uses [Tectonic](https://tectonic-typesetting.github.io) 0.17.0, one static binary that
the script downloads once to `~/.cache/volcontact-paper` (checksum pinned; set `VC_PAPER_CACHE` to
move it, or `TECTONIC` to use your own binary). Nothing is installed on the host, and the same
script runs in GitHub Actions on `ubuntu-latest`. The first build downloads the TeX packages it
needs (about two minutes). The build fails on an undefined citation or reference and prints the
number of `\todo{}` markers left (none at present).

## Layout

| Path | Content |
|---|---|
| `main.tex`, `preamble.tex` | Document, packages, notation macros, `\todo`, the result macros |
| `supplement.tex` | Supplementary material (sections S1 to S5, files `sections/S*.tex`): S1 geometry and constructions of the laws, S2 stability of the step and step bounds, S3 native configurations and the results of those configurations, S4 complete tables and figures, S5 statistical inference and sensitivity. Same preamble, numbers and bibliography. The paper cites its subsections by number (S1.4, S3.5, ...): keep their order or change the citations |
| `sections/` | One file per section: `00` abstract to `09` declarations, appendices `A` and `B`, supplement `S1` to `S5`. The main text leads with the per-scenario tables by class and the mechanism findings; every caveat is stated once, in `07-limitations.tex`. `fem-reference.tex` (Section 5.4, input by `05-results.tex`) is the finite-element reference of `disk` and `bearing`; its table is typed from `results/fem/` (written by `uv run tools/fem/run.py`) and marked so |
| `figures/` | TikZ sources of the schematic figures |
| `generated/` | Tables, figures and numbers produced from the battery by `tools/paper_results.py` (see below) |
| `provenance.json` | Commits and dates of the runs and the first run of the second set, read by that script |
| `references.bib` | Only verified entries; `references-verification.md` lists the URL checked for each |
| `checks/` | `check_polar_moments.py` (closed forms against quadrature), `verify_references.py` |
| `template/` | `sn-jnl.cls` and `sn-mathphys-num.bst`, unmodified, from Springer Nature's template package of December 2024 (LPPL) |

## Rules

- **No number that comes from a run is typed by hand.** Results enter through `generated/`.
  Parameters of the design (stiffness of a scenario, constants of a law) are typed, next to a
  comment that names the file of the repository they are transcribed from.
- **No claim about results that do not exist.** A missing result is a `\todo{}` or a `\result{}`.
- **Every reference is verified before it is added**: run `scripts/paper.sh refs`, and add what a
  script cannot read to `MANUAL` in `checks/verify_references.py`. The paper states only what its
  sources support; `docs/research/literature.md` records which were read in full.
- The methods sections describe the code at the commit named in their header comments. If the
  laws, the native rules or the scenarios change, change the text with them.

## `generated/`: how results enter the paper

```
uv run tools/paper_results.py            # writes paper/generated/ from results/
uv run tools/paper_results.py --list-keys   # every \result{} key the paper uses, with its value
```

`tools/paper_results.py` (tests: `tools/tests/test_paper_results.py`) reads

- `results/battery.csv` and `results/terms.csv` (formats in `docs/scenarios.md` §2);
- `results/stats.json` from `tools/stats.py`, for the bootstrap intervals of the paired ratios,
  the robust comparisons (capped errors, failures counted, clusters, tie band, leave-one-out,
  Holm over the primary family) and the convergence flags;
- `results/ablation_triangulation.csv` from `tools/ablation_triangulation.py`: the friction
  quadratures on one flat face for several triangulations;
- `paper/provenance.json`: what the CSV files do not carry (the commits and dates of the runs, the
  spot check of the baselines, and the first run of the second set transcribed from
  `docs/history.md`);
- `results/timing.csv`, when it exists (the timing pass on an idle machine). Without it the costs
  are the wall times of the accuracy pass and every place that shows them says "indicative,
  measured under load"; with it the same table, figure and `cost/*` keys switch to the timing pass
  and say so. Nothing in the text needs editing.

and writes, deterministically (the same inputs give the same bytes):

1. `generated/numbers.tex`: one `\setresult{key}{value}` per key **that the paper uses**. The
   script scans `main.tex`, `supplement.tex` and `sections/*.tex` for `\result{...}`; a key that it cannot resolve
   stops it, so the paper cannot cite a number that does not exist, and a test fails when the
   committed file is stale. The first line is a comment with the commit, the date and the
   checksum of `battery.csv`. Keys (methods: `gonthier`, `thesis`, `distributed`, `vertex`,
   `centroid`, `pcm` on Chrono, or `<engine>-<law>`; natives `hydro`, `dpoint`, `smc`, `nsc`,
   `soft`, `mjdefault`):

   | Key | Meaning |
   |---|---|
   | `battery/{commit,commit-baselines,date,runs,ok,failed,nan,timeout,crashed,scenarios,heldout,dev,methods,variants,sha256}`, `battery/n/<class>[-dev\|-heldout]` | The battery |
   | `status/<method\|laws\|natives\|baselines>/<nan\|timeout\|failed\|notok>` | Runs without an error value |
   | `err/<scenario>/<method>[/<variant>]` | Error in percent, or the status of the run |
   | `term/<scenario>/<method>/<term>[/<variant>]` | One error term |
   | `pass/<class>/<method>/<1\|5\|20>[/dev\|heldout]` ("16 of 21"; class `Ar` is A without `bowl`, `trough`, `turntable`, `edge`), `passn/...` (the count), `median/<class>/<method>` | Pass rates |
   | `ratio/<a>-vs-<b>/<A\|A+B\|C>/<gm\|lo\|hi\|n\|excl\|wins\|losses\|full>` | Paired ratio of errors, uncapped (a sensitivity analysis), failed pairs left out (a law against a native: on the native's engine) |
   | `robust/<a>-vs-<b>/<A\|A+B\|A-hK>/<gm\|lo\|hi\|full\|holm\|p\|wins\|ties\|losses\|wtl\|loo\|loo-unit\|n>`, `primary/<n\|significant\|excluding-one>[/<subset>]`, `rules/<cap\|tie\|floor>` | Robust ratio (the one conclusions use), the family of primary comparisons, the rules |
   | `band/<method>/<inside\|undetermined\|outside\|none>` | Class B scenarios by position against the band (the error has no sign) |
   | `pertrange/<scenario>/<method>/<min\|median\|max\|range\|lost>` | The eight perturbed runs of one cell |
   | `tri/<shape>/<case>/<triangulation>/<parameter>/<rule>/<moment\|force\|triangles>`, `tri/closed-spread` | The triangulation experiment |
   | `ind/<law>/<maxabs\|maxrel\|median\|worst-scenario\|n>[/<variant>]`, `independence/worst` | Spread over the engines |
   | `conv/<dt\|mesh>/<scenario>/<method>/<flag\|slope>`, `convn/<dt\|mesh>/<method>/<flag\|n>` | Convergence series |
   | `stiffn/<method>/<better\|worse\|n>[/<classes>]`, `pert/<method>` | Stiffness series, perturbation spread |
   | `cost/<method>/<median\|min\|max\|slow\|n>`, `cost/<scenario>/<method>`, `cost/source`, `cost/label` | Wall time over simulated time, and where it comes from |
   | `first/<scenario>/<gonthier\|distributed>` | First run of the second set |

2. `generated/tables/<name>.tex`: complete `tabular` environments (the float, caption and label
   stay in the section file): `status`, `independence`, `errors_A`, `errors_B`, `errors_C`,
   `errors_heldout`, `errors_class_A` and `errors_class_C` (both sets together, the tables of the main
   text), `heldout_first`, `pass_rates`, `ratios` (uncapped), `ratios_primary`, `primary_compact` (the
   small table of the main text), `ratios_robust`, `reduced`, `band`, `perturbations`, `ablation`, `triangulation`,
   `convergence`, `stiffness`, `cost`. A run that did not finish prints its status, never a number.

3. `generated/figures/<name>.pdf`: vector figures in one style (12.5 cm wide, colour-blind-safe
   colours for the laws and greys for the native models, a marker and a line style per method so
   that they read without colour): `heatmap_A`, `heatmap_BC`, `profile_AB`, `forest_AB`,
   `forest_primary`, `convergence_dt`, `convergence_mesh`, `ablation`,
   `ablation_triangulation`, `independence`, `stiffness`, `work_precision` (`profile_AB` and
   `forest_AB` are generated and no longer used by the text).

`generated/` is committed, so that the paper builds without rerunning anything; it is
regenerated, never edited. After a new battery: `uv run tools/stats.py results/battery.csv`,
update `provenance.json`, `uv run tools/paper_results.py`, and reread Sections 5 to 8, the
abstract and the supplement against the new tables (the prose states findings, and a number that changes can change a finding).

## Open points

- The byline lists the AI system Claude Fable 5.2 as first author and David Vilela as second,
  directing and corresponding author (the owner's decision; the report is not submitted to a
  journal).
- External reference: a static finite-element solution for `disk` and `bearing` (Section 5.4,
  `results/fem/README.md`); no experimental one. Its numbers are typed from `results/fem/` in
  `sections/fem-reference.tex` and repeated (the 18 % of `disk`) in the abstract, Section 7 and
  Section 8: rerun `tools/fem/run.py` and retype them if the battery's `disk` or `bearing` change.
- The costs come from `results/timing.csv`; the two friction quadratures were not timed.
- What the paper leaves open, and the runs that would settle it, are in its Section 7 and in
  Section S5.5 of the supplement.
- The main text is 29 pages up to the conclusions (about 25 were aimed at); the supplement has 40.
- Hasegawa and Sato's Eqs. 13 to 16 were not re-checked against their paper (Sections 7 and S1.13
  say so).
