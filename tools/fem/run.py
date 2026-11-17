# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "scikit-fem", "matplotlib"]
# ///
"""Write the finite-element reference of `disk` and `bearing` to results/fem/.

    uv run tools/fem/run.py [--traj-dir DIR]

Outputs (deterministic; about four minutes on one core):

    results/fem/fem_reference.csv   f for every case and refinement level, and a `converged` row
                                    per case (Richardson extrapolation when the three finest
                                    levels converge monotonically, else the finest value; the
                                    spread of the three finest levels is the discretisation error)
    results/fem/stop_times.csv      the battery's base-variant stop times of `disk` and `bearing`
                                    against the nominal theory and against the FE reference,
                                    t_FE = t_theory · f_nominal / f_FE (the scenarios' own formula
                                    t = I ω0 / τ with τ ∝ f)
    results/fem/figures/*.pdf       the FE pressure distributions against the band's

The signed stop times come from the viewer's trajectory export (`build/site/data/traj`, written by
the site export from the committed battery); battery.csv keeps only |error|, which is used to check
that the export matches the committed battery.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import bearing
import disk
import solver

OUT = ROOT / "results" / "fem"
LEVELS = range(4)
DISK_H_OVER_R = (0.05, 0.2, 1.0, 4.0)
POISSON = (0.3, 0.45)
COLUMNS = [
    "case", "H_over_R", "nu", "load_N_per_m", "E_Pa", "level", "dofs", "ratio", "observed_order",
    "spread", "stiffness_vs_winkler", "contact_half_angle_deg", "ratio_nominal",
]


def converged(values: list[float]) -> tuple[float, float, float]:
    """(value, observed order, spread of the three finest levels)."""
    order, extrapolated = solver.observed_order(values)
    spread = max(values[-3:]) - min(values[-3:])
    if math.isfinite(order) and 0.5 < order < 8:
        return extrapolated, order, spread
    return values[-1], float("nan"), spread


def fmt(x) -> str:
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return ""
    return f"{x:.6g}" if isinstance(x, float) else str(x)


def run_disk(rows: list[dict], finest: dict):
    for nu in POISSON:
        for hr in DISK_H_OVER_R:
            results = [disk.solve(hr, nu, level) for level in LEVELS]
            for res in results:
                rows.append({"case": "disk", "H_over_R": hr, "nu": nu, "level": res.level, "dofs": res.dofs,
                             "ratio": res.ratio, "stiffness_vs_winkler": res.stiffness_vs_winkler,
                             "ratio_nominal": 2 / 3})
            value, order, spread = converged([r.ratio for r in results])
            rows.append({"case": "disk", "H_over_R": hr, "nu": nu, "level": "converged", "ratio": value,
                         "observed_order": order, "spread": spread,
                         "stiffness_vs_winkler": results[-1].stiffness_vs_winkler, "ratio_nominal": 2 / 3})
            finest[("disk", nu, hr)] = (results[-1], value)
            print(f"disk nu={nu} H/R={hr}: f = {value:.5f} (order {order:.2f}, spread {spread:.1e})", flush=True)


def run_bearing(rows: list[dict], finest: dict):
    for nu in POISSON:
        for load in (1e-3 * bearing.LOAD_BATTERY, bearing.LOAD_BATTERY):
            results = [bearing.solve(nu, level, load) for level in LEVELS]
            common = {"case": "bearing", "nu": nu, "load_N_per_m": load, "E_Pa": bearing.winkler_modulus(nu),
                      "ratio_nominal": 4 / math.pi}
            for res in results:
                rows.append({**common, "level": res.level, "dofs": res.dofs, "ratio": res.ratio,
                             "stiffness_vs_winkler": res.stiffness_vs_winkler,
                             "contact_half_angle_deg": math.degrees(res.contact_half_angle)})
            value, order, spread = converged([r.ratio for r in results])
            rows.append({**common, "level": "converged", "ratio": value, "observed_order": order, "spread": spread,
                         "stiffness_vs_winkler": results[-1].stiffness_vs_winkler,
                         "contact_half_angle_deg": math.degrees(results[-1].contact_half_angle)})
            finest[("bearing", nu, load)] = (results[-1], value)
            print(f"bearing nu={nu} load={load:.4g}: f = {value:.5f} (order {order:.2f}, spread {spread:.1e})", flush=True)


def figures(finest: dict):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    (OUT / "figures").mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.6), sharey=True)
    rho = np.linspace(0.005, 0.995, 400)
    for ax, nu in zip(axes, POISSON):
        ax.plot(rho, np.ones_like(rho), "k-", lw=1.2, label="uniform (nominal, 2/3)")
        ax.plot(rho, 0.5 / np.sqrt(1 - rho**2), "k--", lw=1.0, label=r"half-space punch ($\pi/4$)")
        ax.plot(rho, 0.5 / rho, "k:", lw=1.0, label="uniform wear (1/2)")
        for hr in DISK_H_OVER_R:
            res, value = finest[("disk", nu, hr)]
            mean = res.force / (math.pi * disk.R**2)
            keep = res.r < disk.R * 0.995
            ax.plot(res.r[keep] / disk.R, res.pressure[keep] / mean, lw=1.4, label=f"FE, H/R = {hr:g}: f = {value:.3f}")
        ax.set_title(rf"disk, $\nu$ = {nu}")
        ax.set_xlabel("r / R")
        ax.set_ylim(0, 3)
        ax.legend(fontsize=6, loc="upper center", ncol=2, frameon=False)
    axes[0].set_ylabel(r"p / p$_\mathrm{mean}$")
    fig.tight_layout()
    fig.savefig(OUT / "figures" / "disk_pressure.pdf")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(4.2, 3.2))
    phi = np.linspace(-math.pi / 2, math.pi / 2, 401)
    ax.plot(np.degrees(phi), 2 / math.pi * np.cos(phi), "k-", lw=1.2, label=r"Winkler cosine (nominal, 4/$\pi$)")
    sqrt_norm = np.trapezoid(np.cos(phi) ** 1.5, phi)
    ax.plot(np.degrees(phi), np.sqrt(np.cos(phi)) / sqrt_norm, "k--", lw=1.0, label=r"$\sqrt{\cos}$ (1.371)")
    ax.plot(np.degrees(phi), 0.5 * np.ones_like(phi), "k:", lw=1.0, label=r"uniform (lower half, $\pi$/2)")
    for nu in POISSON:
        res, value = finest[("bearing", nu, bearing.LOAD_BATTERY)]
        # p r / N: the normalisation with ∫ p̂ cos φ dφ = 1 over the whole bore.
        p_hat = res.pressure * bearing.RADIUS / res.load
        ax.plot(np.degrees(np.concatenate([-res.phi[::-1], res.phi[1:]])), np.concatenate([p_hat[::-1], p_hat[1:]]),
                lw=1.4, label=rf"FE, $\nu$ = {nu}: f = {value:.3f}")
    ax.set_xlim(-120, 120)
    ax.set_ylim(0, 0.95)
    ax.set_xlabel(r"$\varphi$ from the load line (deg)")
    ax.set_ylabel(r"p r / N")
    ax.set_title("bearing (line contact: f = 1)")
    ax.legend(fontsize=6, loc="upper center", ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "figures" / "bearing_pressure.pdf")
    plt.close(fig)


def stop_times(rows: list[dict], traj_dir: Path):
    """The battery's base-variant stop times against the nominal theory and the FE reference."""
    with open(ROOT / "results" / "battery.csv") as f:
        battery = {(r["engine"], r["contact"], r["scenario"]): r for r in csv.DictReader(f) if r["variant"] == "base"}
    fe = {(r["case"], r["nu"]): r["ratio"] for r in rows if r["level"] == "converged"
          and (r["case"] == "bearing" and r["load_N_per_m"] == bearing.LOAD_BATTERY
               or r["case"] == "disk" and r["H_over_R"] == disk.H_BATTERY / disk.R)}
    nominal = {"disk": 2 / 3, "bearing": 4 / math.pi}
    out = []
    for (engine, contact, scenario), row in sorted(battery.items()):
        if scenario not in nominal:
            continue
        path = traj_dir / scenario / "base" / f"{engine}-{contact}.json"
        scalars = json.loads(path.read_text())["scalars"]
        t, t_theory = scalars["stop_time"], scalars["stop_time_theory"]
        entry = {"scenario": scenario, "engine": engine, "contact": contact, "stop_time": t, "stop_time_theory": t_theory}
        if t is not None:
            if abs(abs(t / t_theory - 1) - float(row["error"])) > 1e-4:
                raise SystemExit(f"{path} does not match results/battery.csv")
            entry["error_vs_nominal"] = t / t_theory - 1
        for nu in POISSON:
            t_fe = t_theory * nominal[scenario] / fe[(scenario, nu)]
            entry[f"stop_time_fe_nu{nu}"] = t_fe
            if t is not None:
                entry[f"error_vs_fe_nu{nu}"] = t / t_fe - 1
        out.append(entry)
    cols = ["scenario", "engine", "contact", "stop_time", "stop_time_theory", "error_vs_nominal"]
    for nu in POISSON:
        cols += [f"stop_time_fe_nu{nu}", f"error_vs_fe_nu{nu}"]
    with open(OUT / "stop_times.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for e in out:
            w.writerow({k: fmt(e.get(k)) for k in cols})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj-dir", type=Path, default=ROOT / "build" / "site" / "data" / "traj")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    finest: dict = {}
    run_disk(rows, finest)
    run_bearing(rows, finest)
    with open(OUT / "fem_reference.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(r.get(k)) for k in COLUMNS})
    figures(finest)
    if args.traj_dir.is_dir():
        stop_times(rows, args.traj_dir)
    else:
        print(f"{args.traj_dir} not found: stop_times.csv not written (run the site export first)")


if __name__ == "__main__":
    main()
