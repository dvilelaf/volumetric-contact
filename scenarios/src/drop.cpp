#include <algorithm>
#include <limits>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 10.0, kSide = 0.4, kThickness = 0.2, kGap = 0.02, kMu = 0.5, kK = 1e6;
constexpr double kArea = kSide * kSide;  // Flat face.
constexpr double kDuration = 0.5;
constexpr double kTheoryDt = 1e-6;  // RK4 step of the theory.
constexpr long long kStoreEvery = 10;  // The series keep every 10 µs.
constexpr double kStoreDt = kTheoryDt * kStoreEvery;

// The 1-D law of §4 for a flat face: m·δ̈ = m·g − F, F = K·A·δ·(1 + h·δ̇), F ≥ 0.
// δ is the penetration (positive into the plane), δ̇ its rate (positive downwards).
// In the air F = 0.
struct DropTheory {
  double rebound = 0.0;          // Max upward speed after the first impact [m/s].
  double max_penetration = 0.0;  // [m]
  std::vector<double> delta;     // Penetration, every kStoreDt.
  std::vector<double> up;        // Upward speed, every kStoreDt.
};

DropTheory integrate_drop() {
  const auto accel = [](double delta, double ddelta) {
    const double force = delta > 0.0 ? std::max(0.0, kK * kArea * delta * (1.0 + kDamping * ddelta)) : 0.0;
    return kG - force / kMass;
  };
  const auto rate = [&](double delta, double ddelta) { return std::pair{ddelta, accel(delta, ddelta)}; };

  DropTheory th;
  th.rebound = -std::numeric_limits<double>::infinity();
  th.max_penetration = -std::numeric_limits<double>::infinity();
  double delta = -kGap, ddelta = 0.0;  // At rest, bottom face 20 mm above the plane.
  bool impacted = false;
  const long long steps = std::llround(kDuration / kTheoryDt);
  for (long long n = 0; n <= steps; ++n) {
    if (n % kStoreEvery == 0) {
      th.delta.push_back(delta);
      th.up.push_back(-ddelta);
    }
    impacted = impacted || delta > 0.0;
    if (impacted) th.rebound = std::max(th.rebound, -ddelta);
    th.max_penetration = std::max(th.max_penetration, delta);

    const double h = kTheoryDt;
    const auto [a1, b1] = rate(delta, ddelta);
    const auto [a2, b2] = rate(delta + h / 2 * a1, ddelta + h / 2 * b1);
    const auto [a3, b3] = rate(delta + h / 2 * a2, ddelta + h / 2 * b2);
    const auto [a4, b4] = rate(delta + h * a3, ddelta + h * b3);
    delta += h / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
    ddelta += h / 6 * (b1 + 2 * b2 + 2 * b3 + b4);
  }
  return th;
}

// Value of a series sampled every kStoreDt, at time t (linear, clamped to its span).
double sample(const std::vector<double>& y, double t) {
  const double pos = std::clamp(t / kStoreDt, 0.0, static_cast<double>(y.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(pos), y.size() - 2);
  return y[i] + (pos - static_cast<double>(i)) * (y[i + 1] - y[i]);
}

// Largest value of x from index `first` on, refined by the parabola through it and its
// neighbours, so that the result does not depend on the sampling. NaN if there is none.
double peak(const std::vector<double>& x, std::size_t first) {
  if (first >= x.size()) return std::nan("");
  std::size_t j = first;
  for (std::size_t i = first; i < x.size(); ++i) {
    if (x[i] > x[j]) j = i;
  }
  if (j == first || j + 1 == x.size()) return x[j];
  const double a = x[j - 1], b = x[j], c = x[j + 1];
  const double curvature = a - 2 * b + c;
  return curvature < 0.0 ? b - (c - a) * (c - a) / (8 * curvature) : b;
}

}  // namespace

// A box dropped flat from 20 mm above the plane. Its rebound is the main result.
Scenario drop(const ScenarioOptions& o) {
  Scenario s;
  s.name = "drop";
  s.description = "Box dropped flat from 20 mm above a plane, 10 kg";
  s.theory_class = TheoryClass::kC;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "box";
  body.shape = shape_meshes(o.data_dir, "box_400x400x200");
  body.mass = kMass;
  body.inertia = box_inertia(kMass, kSide, kSide, kThickness);
  body.X_W0.p = Vec3(0.0, 0.0, kThickness / 2 + kGap);
  body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kSide, kSide, kThickness), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  const DropTheory theory = integrate_drop();
  s.metrics = [theory](const Trajectory& traj) {
    MetricReport r;
    Series delta{"delta", {}}, delta_th{"delta_theory", {}}, up{"v", {}}, up_th{"v_theory", {}};
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const double t = traj.times()[i];
      delta.values.push_back(kThickness / 2 - traj.pose(i, 0).p.z());
      delta_th.values.push_back(sample(theory.delta, t));
      up.values.push_back(traj.twist(i, 0).v.z());
      up_th.values.push_back(sample(theory.up, t));
    }
    // First impact: the first sample with the bottom face below the plane.
    const auto first = static_cast<std::size_t>(
        std::find_if(delta.values.begin(), delta.values.end(), [](double d) { return d > 0.0; }) -
        delta.values.begin());
    const double rebound = peak(up.values, first), penetration = peak(delta.values, first);
    ErrorTerms terms;
    terms.add("rebound", relative_error(rebound, theory.rebound));
    r.scalars = {{"rebound_velocity", rebound},
                 {"rebound_velocity_theory", theory.rebound},
                 {"penetration_max_m", penetration},
                 {"penetration_max_theory_m", theory.max_penetration}};
    terms.finish(r);
    r.series = {std::move(delta), std::move(delta_th), std::move(up), std::move(up_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
