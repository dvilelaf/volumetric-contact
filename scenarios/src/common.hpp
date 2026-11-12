#pragma once

// Helpers shared by the scenario implementations.

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "volcontact/scenarios.hpp"

namespace volcontact::scenarios {

constexpr double kG = 9.81;
constexpr double kDamping = 5.0;  // h [s/m], common to every scenario but the impacts below.
// Damping of the impact scenarios (`restitution`, `oblique`). The rebound of a contact force
// f(δ)·(1 + h·δ̇) depends on h·v only and saturates at 1/h: with h = 5 s/m nothing rebounds
// faster than 0.2 m/s and a sphere does not even leave the plane. With 0.5 s/m, impacts between
// 0.5 and 4 m/s span h·v from 0.25 to 2, where the restitution goes from 0.86 to 0.41.
constexpr double kImpactDamping = 0.5;
// Bodies that start resting penetrate 1 µm: it keeps every engine away from
// vertices lying exactly on the contact plane.
constexpr double kInitialPenetration = 1e-6;

// Gravity of an incline tilted `angle_deg`; downhill is +x.
inline Vec3 inclined_gravity(double angle_deg) {
  const double a = angle_deg * std::numbers::pi / 180.0;
  return {kG * std::sin(a), 0.0, -kG * std::cos(a)};
}

// Central inertia of the solids, in the body frame (axes of revolution along z).
inline Mat3 box_inertia(double m, double lx, double ly, double lz) {
  return Vec3(m * (ly * ly + lz * lz) / 12, m * (lx * lx + lz * lz) / 12, m * (lx * lx + ly * ly) / 12).asDiagonal();
}
inline Mat3 cylinder_inertia(double m, double r, double length) {
  const double t = m * (3 * r * r + length * length) / 12;
  return Vec3(t, t, m * r * r / 2).asDiagonal();
}
inline Mat3 sphere_inertia(double m, double r) { return Mat3::Identity() * (0.4 * m * r * r); }

// Penetration δ of a sphere of radius r resting on a plane under the normal load m·g_n:
// m·g_n = K·π·δ²·(r − δ/3), the volume of the cap of depth δ.
inline double sphere_static_penetration(double m, double g_n, double K, double r) {
  double delta = 0.0;
  for (int i = 0; i < 50; ++i) delta = std::sqrt(m * g_n / (K * std::numbers::pi * (r - delta / 3)));
  return delta;
}

// Name of a curved asset at the options' resolution: `<base>_seg<N>`.
inline std::string curved(const ScenarioOptions& o, const std::string& base) {
  return base + "_seg" + std::to_string(o.segments());
}

// Contact of §4 between a and b: K, h, μ; Gonthier with the bristle defaults,
// friction at the volume centroid and μ_spin = μ (used by the thesis form only); distributed with its defaults.
inline ContactSpec standard_contact(BodyRef a, BodyRef b, double K, double h, double mu) {
  GonthierParams g{.stiffness = K, .damping = h, .mu_spin = mu};
  g.friction = BristleFriction{.mu_dynamic = mu, .mu_static = mu};
  return {.a = a,
          .b = b,
          .friction = mu,
          .gonthier = g,
          .distributed = {.stiffness = K, .damping = h, .mu_dynamic = mu, .mu_static = mu}};
}

// --- Metric helpers (t and x have one entry per trajectory sample) -------------

// First time x falls below `level`, interpolated linearly between samples; NaN if never.
inline double crossing_time(const std::vector<double>& t, const std::vector<double>& x, double level) {
  for (std::size_t i = 1; i < x.size(); ++i) {
    if (x[i] < level) {
      const double f = (x[i - 1] - level) / (x[i - 1] - x[i]);
      return t[i - 1] + f * (t[i] - t[i - 1]);
    }
  }
  return std::nan("");
}

// Mean of x over the samples with t >= t0.
inline double mean_after(const std::vector<double>& t, const std::vector<double>& x, double t0) {
  double sum = 0.0;
  std::size_t n = 0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    if (t[i] >= t0) {
      sum += x[i];
      ++n;
    }
  }
  return n ? sum / static_cast<double>(n) : std::nan("");
}

// Mean period of an oscillation about zero, from its zero crossings (linearly
// interpolated); consecutive crossings are half a period apart. NaN with fewer than two.
inline double mean_period(const std::vector<double>& t, const std::vector<double>& x) {
  std::vector<double> crossings;
  for (std::size_t i = 1; i < x.size(); ++i) {
    if ((x[i - 1] < 0.0) != (x[i] < 0.0) && x[i - 1] != x[i]) {
      crossings.push_back(t[i - 1] + (t[i] - t[i - 1]) * x[i - 1] / (x[i - 1] - x[i]));
    }
  }
  if (crossings.size() < 2) return std::nan("");
  return 2.0 * (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1);
}

inline double relative_error(double measured, double theory) { return std::abs(measured - theory) / std::abs(theory); }

// Angle between body axis k and world axis k.
inline double axis_deviation(const Pose& X, int k) { return std::acos(std::clamp(X.R(k, k), -1.0, 1.0)); }

// The larger of a and b; NaN if either is (std::max would drop it, and a run that blew up
// would score as if it had not moved).
inline double worse(double a, double b) { return std::isnan(a) || std::isnan(b) ? std::nan("") : std::max(a, b); }

inline double max_abs(const std::vector<double>& v) {
  double m = 0.0;
  for (double x : v) m = worse(m, std::abs(x));
  return m;
}

// Largest |x| over the samples with t >= t0.
inline double max_abs_after(const std::vector<double>& t, const std::vector<double>& x, double t0) {
  double m = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    if (t[i] >= t0) m = worse(m, std::abs(x[i]));
  }
  return m;
}

// x at time `at`, interpolated linearly between the samples around it; NaN outside the samples.
inline double value_at(const std::vector<double>& t, const std::vector<double>& x, double at) {
  if (t.empty() || std::isnan(at) || at < t.front()) return std::nan("");
  for (std::size_t i = 1; i < t.size(); ++i) {
    if (t[i] >= at) return x[i - 1] + (at - t[i - 1]) / (t[i] - t[i - 1]) * (x[i] - x[i - 1]);
  }
  return std::nan("");
}

// Least-squares parabola p0 + p1·t + p2·t² through the samples with t >= t0.
inline std::array<double, 3> fit_parabola(const std::vector<double>& t, const std::vector<double>& x, double t0) {
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < t.size(); ++i) {
    if (t[i] < t0) continue;
    const Eigen::Vector3d row(1.0, t[i], t[i] * t[i]);
    A += row * row.transpose();
    b += row * x[i];
  }
  const Eigen::Vector3d p = A.ldlt().solve(b);
  return {p(0), p(1), p(2)};
}

// Time and |x| of the largest |x| over the last `window` of the samples, refined by the
// parabola through it and its neighbours. With `window` half a period of an oscillation about
// zero, this is its last extreme.
inline std::pair<double, double> last_peak(const std::vector<double>& t, const std::vector<double>& x,
                                           double window) {
  if (x.empty()) return {std::nan(""), std::nan("")};
  std::size_t j = x.size() - 1;
  for (std::size_t i = x.size(); i-- > 0 && t[i] >= t.back() - window;) {
    if (std::abs(x[i]) > std::abs(x[j])) j = i;
  }
  if (j == 0 || j + 1 == x.size()) return {t[j], std::abs(x[j])};
  const double sign = x[j] < 0 ? -1.0 : 1.0;
  const double a = sign * x[j - 1], b = sign * x[j], c = sign * x[j + 1];
  const double curvature = a - 2 * b + c;
  const double shift = curvature < 0.0 ? 0.5 * (a - c) / curvature : 0.0;
  return {t[j] + shift * (t[j + 1] - t[j]), b - 0.25 * (a - c) * shift};
}

// Arithmetic–geometric mean: the exact period of a pendulum released at θ0 is its
// small-amplitude period divided by agm(1, cos(θ0/2)).
inline double agm(double a, double b) {
  for (int i = 0; i < 40; ++i) {
    const double m = 0.5 * (a + b);
    b = std::sqrt(a * b);
    a = m;
  }
  return a;
}

// θ(t) of the pendulum θ̈ = −w2·sin θ released at rest at θ0, at the times t (RK4, steps ≤ 0.1 ms).
inline std::vector<double> pendulum_angle(const std::vector<double>& t, double theta0, double w2) {
  std::vector<double> out;
  out.reserve(t.size());
  double th = theta0, om = 0.0, now = t.empty() ? 0.0 : t.front();
  for (const double target : t) {
    const int n = std::max(1, static_cast<int>(std::ceil((target - now) / 1e-4)));
    const double h = (target - now) / n;
    for (int k = 0; k < n && h > 0.0; ++k) {
      const double k1 = -w2 * std::sin(th), l1 = om;
      const double k2 = -w2 * std::sin(th + h / 2 * l1), l2 = om + h / 2 * k1;
      const double k3 = -w2 * std::sin(th + h / 2 * l2), l3 = om + h / 2 * k2;
      const double k4 = -w2 * std::sin(th + h * l3), l4 = om + h * k3;
      th += h / 6 * (l1 + 2 * l2 + 2 * l3 + l4);
      om += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    }
    now = target;
    out.push_back(th);
  }
  return out;
}

// Penetration δ of a cylinder of radius r and length l pressed on a plane (or, with r the
// relative radius, in a conforming cavity) by the load N: N = K·l·(4/3)·√(2r)·δ^(3/2).
inline double cylinder_static_penetration(double N, double K, double r, double l) {
  return std::pow(N / (K * l * (4.0 / 3.0) * std::sqrt(2.0 * r)), 2.0 / 3.0);
}

// --- Error terms ---------------------------------------------------------------

// The normalized deviations of the quantities a theory fixes. `error` is the worst of
// them; each one is kept as the scalar `err_<name>`. A NaN term makes the error NaN.
class ErrorTerms {
 public:
  void add(const std::string& name, double value) { terms_.emplace_back("err_" + name, std::abs(value)); }
  // Deviation of a length (creep, slip, slide) over a length of the body.
  void add_length(const std::string& name, double length, double body_length) { add(name, length / body_length); }
  // A residual speed, as the drift it would add up to over the run, over a length of the body.
  void add_residual_speed(double speed, double duration, double body_length) {
    add("residual_speed", speed * duration / body_length);
  }
  void finish(MetricReport& r) const {
    double worst = 0.0;
    for (const auto& [name, value] : terms_) {
      worst = std::isnan(value) || std::isnan(worst) ? std::nan("") : std::max(worst, value);
      r.scalars.emplace_back(name, value);
    }
    r.error = worst;
  }

 private:
  std::vector<std::pair<std::string, double>> terms_;
};

// Class B: the answers of the other admissible pressure distributions, as the scalars
// `<name>_<label>`, the band they span with the nominal theory (`<name>_band_min`,
// `<name>_band_max`) and `band_error`, the distance of the measured value to that band over
// the nominal value (0 inside the band).
inline void add_band(MetricReport& r, const std::string& name, double measured, double nominal,
                     std::initializer_list<std::pair<const char*, double>> others) {
  double lo = nominal, hi = nominal;
  for (const auto& [label, value] : others) {
    r.scalars.emplace_back(name + "_" + label, value);
    lo = std::min(lo, value);
    hi = std::max(hi, value);
  }
  r.scalars.emplace_back(name + "_band_min", lo);
  r.scalars.emplace_back(name + "_band_max", hi);
  r.scalars.emplace_back("band_error", std::isnan(measured)
                                           ? std::nan("")
                                           : std::max({0.0, lo - measured, measured - hi}) / std::abs(nominal));
}

}  // namespace volcontact::scenarios
