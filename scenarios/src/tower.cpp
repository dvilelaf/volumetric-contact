#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kSide = 0.4, kHeight = 0.1, kMu = 0.5;

// Tilt θ(t) of the gravity [rad]: a half-cosine ramp from 0 to `angle` over `ramp`, then constant.
double tilt(double t, double angle, double ramp) {
  return t >= ramp ? angle : angle * (1.0 - std::cos(std::numbers::pi * t / ramp)) / 2.0;
}

Vec3 gravity_at(double theta) { return {kG * std::sin(theta), 0.0, -kG * std::cos(theta)}; }

// Blocks 400 × 400 × 100 mm stacked on the plane, the lowest first, with the masses given.
void stack_blocks(Scenario& s, const ScenarioOptions& o, const std::vector<double>& masses, double K) {
  s.grounds.push_back(planar_ground(o.data_dir));
  for (std::size_t k = 0; k < masses.size(); ++k) {
    BodySpec& body = s.bodies.emplace_back();
    body.name = "block" + std::to_string(k + 1);
    body.shape = shape_meshes(o.data_dir, "box_400x400x100");
    body.mass = masses[k];
    body.inertia = box_inertia(masses[k], kSide, kSide, kHeight);
    // Each block sinks 1 µm more than the one below, the lowest one into the ground.
    body.X_W0.p = Vec3(0.0, 0.0, (static_cast<double>(k) + 0.5) * kHeight - static_cast<double>(k + 1) * kInitialPenetration);
    body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kSide, kSide, kHeight), .X_BG = {}};
    const int i = static_cast<int>(k);
    s.contacts.push_back(
        standard_contact(BodyRef::body(i), k == 0 ? BodyRef::ground(0) : BodyRef::body(i - 1), K, kDamping, kMu));
  }
  s.native = {.stiffness = K, .damping = kDamping};
}

// Metric of a stack that must not move after `settle`: the largest displacement of any block
// from its pose at `settle` (the top one is reported apart), and the residual speed.
MetricReport rest_metrics(const Trajectory& traj, double settle, double duration) {
  MetricReport r;
  const std::vector<double>& t = traj.times();
  const int top = traj.num_bodies() - 1;
  std::size_t i0 = 0;
  while (i0 + 1 < t.size() && t[i0] < settle) ++i0;
  Series creep{"creep_top", {}}, lean{"displacement_top", {}};
  std::vector<double> speed;
  double creep_max = 0.0;
  for (std::size_t i = 0; i < traj.size(); ++i) {
    lean.values.push_back((traj.pose(i, top).p - traj.pose(0, top).p).norm());
    creep.values.push_back(i < i0 ? 0.0 : (traj.pose(i, top).p - traj.pose(i0, top).p).norm());
    speed.push_back(0.0);
    for (int k = 0; k <= top; ++k) {
      speed.back() = worse(speed.back(), traj.twist(i, k).v.norm());
      if (i >= i0) creep_max = worse(creep_max, (traj.pose(i, k).p - traj.pose(i0, k).p).norm());
    }
  }
  const double residual = mean_after(t, speed, duration - 0.5);
  ErrorTerms terms;
  terms.add_length("creep", creep_max, kSide);
  terms.add_residual_speed(residual, duration, kSide);
  r.scalars = {{"creep_max_m", creep_max},
               {"creep_max_m_theory", 0.0},
               {"creep_top_m", max_abs(creep.values)},
               {"displacement_top_settled_m", lean.values[i0]},
               {"residual_speed_mps", residual},
               {"residual_speed_mps_theory", 0.0}};
  terms.finish(r);
  r.series = {std::move(creep), std::move(lean)};
  return r;
}

}  // namespace

// Ten blocks stacked on a plane tilted to 5° for 10 s. The blocks are 400 mm wide and the tower
// 1 m tall, so that the resultant of every interface stays in the middle third
// (h_c·tan θ = 44 mm < a/6 = 67 mm): the contact is full everywhere and holding the tower does
// not depend on the stiffness; ten cubes at 5° would stand on a strip a fifth of their side.
Scenario tower(const ScenarioOptions& o) {
  constexpr int blocks = 10;
  constexpr double m = 1.0, K = 1e6, angle = 5.0 * std::numbers::pi / 180.0, ramp = 1.0, settle = 3.0, duration = 10.0;
  Scenario s;
  s.name = "tower";
  s.description = "Ten blocks stacked on a plane tilted to 5 deg over 1 s, held for 10 s, mu = 0.5";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = duration;
  stack_blocks(s, o, std::vector<double>(blocks, m), K);
  for (BodySpec& body : s.bodies) {
    // Keeps |g_eff| = g: the total force on the COM is m·g(θ(t)).
    body.applied_force = [=](double t) -> Vec3 { return m * (gravity_at(tilt(t, angle, ramp)) - gravity_at(0.0)); };
  }
  s.metrics = [=](const Trajectory& traj) { return rest_metrics(traj, settle, duration); };
  return s;
}

// A 1000 kg block resting on a 1 kg block of the same size. The stiffness puts the static
// penetration of the heavy block at 0.6 mm, under 1 % of its height.
Scenario massratio(const ScenarioOptions& o) {
  constexpr double K = 1e8, settle = 0.5, duration = 2.0;
  Scenario s;
  s.name = "massratio";
  s.description = "A 1000 kg block resting on a 1 kg block on a plane";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = duration;
  stack_blocks(s, o, {1.0, 1000.0}, K);
  s.metrics = [=](const Trajectory& traj) { return rest_metrics(traj, settle, duration); };
  return s;
}

}  // namespace volcontact::scenarios
