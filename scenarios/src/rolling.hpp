#pragma once

// Rolling from rest down an incline, shared by `cylinder` and `sphere`.

#include "common.hpp"

namespace volcontact::scenarios {

// The run settles on its contact before the measurement starts [s].
constexpr double kRollingSettle = 0.2;

// A body of radius r and inertia factor β = I/(m r²) rolling without slipping. The rigid theory
// is v̇ = a0 = g sin θ/(1 + β). A Winkler pressure with damping, K·ℓ·(1 − h·vn), is larger on the
// leading half of the patch than on the trailing half: it resists the rolling with the moment
// M = N·h·ω·a²/k (a the half-width of the patch along the motion; k = 6 for a spherical cap, 5 for
// a cylinder), so v̇ = a0 − c·v with c = N·h·a²/(k·m·r²·(1 + β)). That term is the law's own
// (it has h and, through a, K) and vanishes at rest, so the scenario measures the acceleration
// at zero speed, which the rigid theory fixes, and reports the rate c next to its prediction.
struct RollingTheory {
  double accel = 0.0;        // a0 [m/s²].
  double winkler_rate = 0.0; // c [1/s], predicted for the Winkler laws.

  [[nodiscard]] double velocity(double t) const { return accel * t; }
  [[nodiscard]] double winkler_velocity(double t) const {
    return winkler_rate > 0.0 ? accel / winkler_rate * (1.0 - std::exp(-winkler_rate * t)) : accel * t;
  }
};

// Metric of a rolling run: t, the velocity v of the centre along the slope, the rolling speed
// ω·r and the lateral position y, one entry per sample.
inline MetricReport rolling_metrics(const RollingTheory& th, const std::vector<double>& t, Series v, Series wr,
                                    Series y) {
  MetricReport r;
  Series v_th{"v_theory", {}}, wr_th{"wr_theory", {}};
  std::vector<double> slip;
  for (std::size_t i = 0; i < t.size(); ++i) {
    v_th.values.push_back(th.velocity(t[i]));
    wr_th.values.push_back(th.velocity(t[i]));
    slip.push_back(v.values[i] - wr.values[i]);
  }
  // v(t) = p0 + p1·t + p2·t² after the settling: p1 is the acceleration at t = 0, where the body
  // is at rest, and −2·p2/p1 the rate of a resistance proportional to the speed.
  const std::array<double, 3> p = fit_parabola(t, v.values, kRollingSettle);
  const double v_end = v.values.back(), v_th_end = v_th.values.back();
  const double slip_max = max_abs_after(t, slip, kRollingSettle);
  ErrorTerms terms;
  terms.add("acceleration", relative_error(p[1], th.accel));
  terms.add("slip", slip_max / v_th_end);
  r.scalars = {{"acceleration", p[1]},
               {"acceleration_theory", th.accel},
               {"rolling_resistance_rate", -2.0 * p[2] / p[1]},
               {"rolling_resistance_rate_winkler", th.winkler_rate},
               {"velocity_end", v_end},
               {"velocity_end_theory", v_th_end},
               {"velocity_end_winkler", th.winkler_velocity(t.back())},
               {"slip_max_mps", slip_max},
               {"slip_max_mps_theory", 0.0},
               {"lateral_drift_max_m", max_abs(y.values)},
               {"lateral_drift_max_m_theory", 0.0}};
  terms.finish(r);
  r.series = {std::move(v), std::move(v_th), std::move(wr), std::move(wr_th), std::move(y)};
  return r;
}

}  // namespace volcontact::scenarios
