#include <array>
#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr int kBalls = 4;
constexpr std::array<double, kBalls> kImpactSpeed = {0.5, 1.0, 2.0, 4.0};  // [m/s]
constexpr double kMass = 1.0, kRadius = 0.1, kMu = 0.5, kK = 1e7, kDuration = 0.6;
constexpr double kSpacing = 0.3;

}  // namespace

// Rebound speed of a body that hits at speed u_i a contact whose force is f(δ)·(1 + h·δ̇), for any
// elastic f (Hunt–Crossley), with no other force: h·u_i − ln(1 + h·u_i) = −h·u_o − ln(1 − h·u_o).
// It does not depend on f or on the mass, and tends to 1/h for fast impacts.
double hunt_crossley_rebound(double impact, double h) {
  const double lhs = h * impact - std::log1p(h * impact);
  double lo = 0.0, hi = 1.0;  // x = h·u_o in [0, 1): −x − ln(1 − x) grows from 0 to ∞.
  for (int i = 0; i < 200; ++i) {
    const double mid = (lo + hi) / 2;
    (-mid - std::log1p(-mid) < lhs ? lo : hi) = mid;
  }
  return (lo + hi) / 2 / h;
}

// The same impact for a sphere on the Winkler foundation, with gravity: m·δ̈ = m·g − F,
// F = K·V(δ)·(1 + h·δ̇) ≥ 0, V the cap of depth δ, from δ = 0 at speed `impact` (RK4 at 1 µs)
// until it leaves the plane. Returns the largest upward speed.
double winkler_sphere_rebound(double impact, double m, double r, double K, double h) {
  const auto accel = [&](double d, double dd) {
    const double cap = std::numbers::pi * d * d * (r - d / 3);
    return kG - (d > 0.0 ? std::max(0.0, K * cap * (1.0 + h * dd)) : 0.0) / m;
  };
  double d = 0.0, dd = impact, best = 0.0;
  const double step = 1e-6;
  for (long long n = 0; n < 2'000'000 && d >= 0.0; ++n) {
    const double a1 = dd, b1 = accel(d, dd);
    const double a2 = dd + step / 2 * b1, b2 = accel(d + step / 2 * a1, a2);
    const double a3 = dd + step / 2 * b2, b3 = accel(d + step / 2 * a2, a3);
    const double a4 = dd + step * b3, b4 = accel(d + step * a3, a4);
    d += step / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
    dd += step / 6 * (b1 + 2 * b2 + 2 * b3 + b4);
    best = std::max(best, -dd);
  }
  return best;
}

// Four spheres dropped from the heights that give 0.5, 1, 2 and 4 m/s at the plane. With
// K = 1e7 the impacts last 18 to 30 ms, at least 17 steps of 1 ms.
Scenario restitution(const ScenarioOptions& o) {
  Scenario s;
  s.name = "restitution";
  s.description = "Spheres dropped on a plane at 0.5, 1, 2 and 4 m/s: rebound speed and energy";
  s.theory_class = TheoryClass::kC;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  s.grounds.push_back(planar_ground(o.data_dir));
  std::array<double, kBalls> theory{}, closed{};
  for (int k = 0; k < kBalls; ++k) {
    BodySpec& body = s.bodies.emplace_back();
    body.name = "ball" + std::to_string(k + 1);
    body.shape = shape_meshes(o.data_dir, curved(o, "sphere_r100"));
    body.mass = kMass;
    body.inertia = sphere_inertia(kMass, kRadius);
    const double gap = kImpactSpeed[k] * kImpactSpeed[k] / (2.0 * kG);
    body.X_W0.p = Vec3((k - 0.5 * (kBalls - 1)) * kSpacing, 0.0, kRadius + gap);
    body.primitive = Primitive{.kind = Primitive::Kind::kSphere, .size = Vec3(kRadius, 0.0, 0.0), .X_BG = {}};
    s.contacts.push_back(standard_contact(BodyRef::body(k), BodyRef::ground(0), kK, kImpactDamping, kMu));
    theory[k] = winkler_sphere_rebound(kImpactSpeed[k], kMass, kRadius, kK, kImpactDamping);
    closed[k] = hunt_crossley_rebound(kImpactSpeed[k], kImpactDamping);
  }
  s.native = {.stiffness = kK, .damping = kImpactDamping};

  s.metrics = [theory, closed](const Trajectory& traj) {
    MetricReport r;
    ErrorTerms terms;
    double gain_worst = 0.0;
    for (int k = 0; k < kBalls; ++k) {
      const std::string idx = std::to_string(k + 1);
      Series up{"v_" + idx, {}}, energy{"energy_" + idx, {}};
      // Rebound: the largest upward speed after the first contact. Energy (kinetic plus
      // gravitational, from the plane) is compared between samples in the air only, where no
      // contact stores any: the largest rise above an earlier value, over the impact energy.
      double rebound = std::nan(""), lowest = std::numeric_limits<double>::infinity(), gain = 0.0;
      bool touched = false;
      for (std::size_t i = 0; i < traj.size(); ++i) {
        const Pose& X = traj.pose(i, k);
        const Twist& V = traj.twist(i, k);
        up.values.push_back(V.v.z());
        energy.values.push_back(0.5 * kMass * V.v.squaredNorm() + 0.2 * kMass * kRadius * kRadius * V.w.squaredNorm() +
                                kMass * kG * (X.p.z() - kRadius));
        if (X.p.z() < kRadius) {
          touched = true;
        } else {
          gain = std::max(gain, energy.values.back() - lowest);
          lowest = std::min(lowest, energy.values.back());
        }
        if (touched) rebound = std::isnan(rebound) ? V.v.z() : std::max(rebound, V.v.z());
      }
      gain /= 0.5 * kMass * kImpactSpeed[k] * kImpactSpeed[k];
      gain_worst = std::max(gain_worst, gain);
      terms.add("rebound_" + idx, relative_error(rebound, theory[k]));
      r.scalars.emplace_back("rebound_" + idx, rebound);
      r.scalars.emplace_back("rebound_" + idx + "_theory", theory[k]);
      r.scalars.emplace_back("rebound_" + idx + "_hunt_crossley", closed[k]);
      r.scalars.emplace_back("energy_gain_" + idx, gain);
      r.series.push_back(std::move(up));
      r.series.push_back(std::move(energy));
    }
    // Class A inside a class C scenario: whatever the rebound, energy must not appear.
    terms.add("energy_gain", gain_worst);
    r.scalars.emplace_back("energy_gain_theory", 0.0);
    terms.finish(r);
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
