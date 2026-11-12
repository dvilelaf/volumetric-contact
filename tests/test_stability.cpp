#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <doctest/doctest.h>

#include "test_util.hpp"
#include "volcontact/contact_model.hpp"

using namespace volcontact;
using volcontact::test::make_box;

namespace {

constexpr double kG = 9.81;

// Boxes stacked on a fixed ground and stepped the way the engines couple the laws: the wrenches
// of the state at the start of the step, then v += dt·a and x += dt·v (semi-implicit Euler).
struct Stack {
  ContactModel model;
  std::vector<double> mass;
  std::vector<Mat3> inertia;  // Body frame.
  std::vector<Pose> poses;    // The ground is the last one.
  std::vector<Twist> twists;
  std::vector<FrictionState> friction;
  ContactModel::Evaluation eval;

  [[nodiscard]] int blocks() const { return static_cast<int>(mass.size()); }

  void step(double dt, const Vec3& gravity) {
    model.evaluate(poses, twists, friction, &eval);
    friction.swap(eval.next_friction);
    for (int i = 0; i < blocks(); ++i) {
      Pose& X = poses[i];
      Twist& V = twists[i];
      const Wrench& w = eval.wrenches[i];
      const Mat3 I = X.R * inertia[i] * X.R.transpose();
      V.v += dt * (gravity + w.force / mass[i]);
      V.w += dt * I.ldlt().solve(Vec3(w.torque - V.w.cross(I * V.w)));
      X.p += dt * V.v;
      const double angle = dt * V.w.norm();
      if (angle > 0.0) X.R = Eigen::AngleAxisd(angle, V.w.normalized()).toRotationMatrix() * X.R;
    }
  }
};

struct StackOptions {
  Vec3 size = Vec3(0.4, 0.4, 0.1);
  std::vector<double> masses;  // The lowest block first.
  double stiffness = 1e6, damping = 5.0, mu = 0.5;
  bool distributed = false;
  bool body_point = false;
  bool anchored = false;
  double dt = 0.0;  // Explicit coupling with this step; 0 for none.
};

Stack make_stack(const StackOptions& o) {
  Stack s;
  const auto box = std::make_shared<const CollisionShape>(make_box(-0.5 * o.size, 0.5 * o.size));
  const auto ground = std::make_shared<const CollisionShape>(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));
  const int n = static_cast<int>(o.masses.size());
  ContactLaw law;
  if (o.distributed) {
    law = DistributedParams{.stiffness = o.stiffness, .damping = o.damping, .mu_dynamic = o.mu, .mu_static = o.mu};
  } else {
    GonthierParams g{.stiffness = o.stiffness, .damping = o.damping, .mu_spin = o.mu};
    g.friction = BristleFriction{.mu_dynamic = o.mu, .mu_static = o.mu};
    if (o.anchored) g.friction->deflection = BristleFriction::Deflection::kAnchored;
    if (o.body_point) g.friction_point = {.kind = FrictionPoint::Kind::kBodyPoint, .p_A = Vec3(0, 0, -0.5 * o.size.z())};
    law = g;
  }
  double load = 0.0;
  for (const double m : o.masses) load += m * kG;
  double z = 0.0;
  for (int k = 0; k < n; ++k) {
    const double m = o.masses[k];
    const Vec3 d = o.size.cwiseProduct(o.size);
    s.mass.push_back(m);
    s.inertia.push_back(Vec3(m * (d.y() + d.z()) / 12, m * (d.x() + d.z()) / 12, m * (d.x() + d.y()) / 12).asDiagonal());
    // Each interface starts at the penetration that carries the blocks above it.
    z += (k == 0 ? 0.5 : 1.0) * o.size.z() - load / (o.stiffness * o.size.x() * o.size.y());
    load -= m * kG;
    s.poses.push_back({Mat3::Identity(), Vec3(0, 0, z)});
    s.model.add_pair({.body_a = k, .body_b = k == 0 ? n : k - 1, .shape_a = box, .shape_b = k == 0 ? ground : box, .law = law});
  }
  s.poses.emplace_back();
  if (o.dt > 0.0) {
    std::vector<ContactModel::BodyInertia> bodies;
    for (int k = 0; k < n; ++k) bodies.push_back({1.0 / s.mass[k], s.inertia[k].inverse()});
    bodies.emplace_back();
    s.model.set_explicit_coupling(o.dt, std::move(bodies));
  }
  s.twists.resize(s.poses.size());
  s.friction = s.model.initial_friction();
  return s;
}

struct Rest {
  double creep = 0.0;  // Largest displacement of any block after the settling [m].
  double speed = 0.0;  // Fastest block at the end [m/s].
};

// Tilts the gravity to `angle` over `ramp` seconds (half-cosine), holds it until `duration`, and
// measures what moves after `settle`.
Rest hold(Stack& s, double dt, double angle, double ramp, double settle, double duration) {
  Rest r;
  std::vector<Vec3> settled;
  const auto steps = std::llround(duration / dt);
  for (long long i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double th = t >= ramp ? angle : angle * (1.0 - std::cos(std::numbers::pi * t / ramp)) / 2.0;
    s.step(dt, Vec3(kG * std::sin(th), 0.0, -kG * std::cos(th)));
    if (t + dt >= settle) {
      if (settled.empty()) {
        for (int k = 0; k < s.blocks(); ++k) settled.push_back(s.poses[k].p);
      }
      for (int k = 0; k < s.blocks(); ++k) {
        const double d = (s.poses[k].p - settled[k]).norm();
        r.creep = std::isnan(d) ? d : std::max(r.creep, d);
      }
    }
  }
  for (int k = 0; k < s.blocks(); ++k) r.speed = std::max(r.speed, s.twists[k].v.norm());
  return r;
}

}  // namespace

TEST_CASE("a tower of ten blocks tilted to 5 deg holds at 1 ms, with either law") {
  // The lowest interface carries ten times the weight of its block and the next one nine: their
  // stick dampers, c = μ·Fn·τ/d, take 2·c·dt/m = 0.245·19 = 4.7 of the block's velocity per step,
  // beyond the 4 an explicit step integrates. Bounded to what the step resolves, it holds.
  const double angle = 5.0 * std::numbers::pi / 180.0, dt = 1e-3;
  for (const bool distributed : {false, true}) {
    CAPTURE(distributed);
    Stack bounded = make_stack({.masses = std::vector<double>(10, 1.0), .distributed = distributed, .dt = dt});
    const Rest held = hold(bounded, dt, angle, 1.0, 2.5, 4.0);
    CHECK(held.creep < 1e-3);  // The sway of the tower on its foundations, 0.5 mm; nothing slides.
    CHECK(held.speed < 5e-3);

    Stack plain = make_stack({.masses = std::vector<double>(10, 1.0), .distributed = distributed});
    const Rest lost = hold(plain, dt, angle, 1.0, 2.5, 4.0);
    CHECK(lost.creep > 10.0 * held.creep);
  }
}

TEST_CASE("Gonthier's law holds the tower at any step only with the bristle on the body's slip") {
  // At 0.1 ms nothing is near a stability limit. The overlap of a block that rocks by 1e-5 rad
  // moves its centroid by millimetres, and a bristle anchored to it never rests.
  const double angle = 5.0 * std::numbers::pi / 180.0, dt = 1e-4;
  Stack integrated = make_stack({.masses = std::vector<double>(10, 1.0)});
  const Rest held = hold(integrated, dt, angle, 1.0, 2.0, 2.5);
  CHECK(held.creep < 1e-3);
  Stack anchored = make_stack({.masses = std::vector<double>(10, 1.0), .anchored = true});
  const Rest lost = hold(anchored, dt, angle, 1.0, 2.0, 2.5);
  CHECK(lost.creep > 5e-3);
  CHECK(lost.creep > 10.0 * held.creep);
}

TEST_CASE("a 1000 kg block on a 1 kg block stays at the step the normal stiffness allows") {
  // The light block sits between two foundations of stiffness K·A = 1.6e7 N/m: with the dampers
  // at their bounds the explicit step needs Σ K·A·w·dt² = 2·K·A·dt²/m ≤ 2, dt ≤ 0.25 ms. As
  // given, the normal dampers (h·Fn = 49 kN·s/m on 1 kg) need dt < 0.02 ms, and sliding friction
  // (4.9 kN on 1 kg, blended in over 25 mm/s) a step under 5 µs.
  const double dt = 2.5e-4;
  for (const bool distributed : {false, true}) {
    CAPTURE(distributed);
    Stack s = make_stack({.masses = {1.0, 1000.0}, .stiffness = 1e8, .distributed = distributed, .dt = dt});
    s.twists[0].v = Vec3(1e-3, 0.0, -1e-3);  // A push on each block.
    s.twists[1].v = Vec3(0.0, 1e-3, 0.0);
    const Rest r = hold(s, dt, 0.0, 1.0, 0.5, 1.0);
    // The heavy block swings on the stick of the light one, whose damper the step bounds with
    // the light block's mass (1 % of critical for the heavy one): it dies out over seconds.
    CHECK(r.creep < 1e-4);
    CHECK(r.speed < 5e-4);

    // As given, the same push throws the blocks apart.
    Stack plain = make_stack({.masses = {1.0, 1000.0}, .stiffness = 1e8, .distributed = distributed});
    plain.twists[0].v = Vec3(1e-3, 0.0, -1e-3);
    plain.twists[1].v = Vec3(0.0, 1e-3, 0.0);
    const Rest lost = hold(plain, dt, 0.0, 1.0, 0.5, 1.0);
    CHECK_FALSE(lost.speed < 1e-2);
  }
}

TEST_CASE("the bounds of the explicit coupling leave a body under its own weight as it was") {
  // A 1 kg block sliding and spinning on the ground at 1 ms: far from every bound, same wrench.
  for (const bool distributed : {false, true}) {
    Stack plain = make_stack({.masses = {1.0}, .distributed = distributed});
    Stack bounded = make_stack({.masses = {1.0}, .distributed = distributed, .dt = 1e-3});
    for (Stack* s : {&plain, &bounded}) {
      s->twists[0] = {Vec3(0.02, -0.01, 0.3), Vec3(0.004, 0.002, -0.01)};
      for (int i = 0; i < 20; ++i) s->step(1e-3, Vec3(1.0, 0.0, -kG));
    }
    CHECK(plain.eval.wrenches[0].force == bounded.eval.wrenches[0].force);
    CHECK(plain.eval.wrenches[0].torque == bounded.eval.wrenches[0].torque);
    // The load acceleration of the block: its normal force over its mass at the contact, 50 mm under the centre.
    CHECK(bounded.friction[0].load_tangent ==
          doctest::Approx(bounded.eval.pairs[0].force.normal_force * (1.0 + 0.05 * 0.05 * 12.0 / 0.17)).epsilon(0.01));
  }
}

TEST_CASE("the stick of a contact that carries many times the weight of its body is as stiff as the step resolves") {
  // 1 kg pressed by 20 times its weight: μ·Fn/d = 9.8e5 N/m, against m_eff/dt² = 2.5e5/1.17 at 2 ms
  // (the stick acts at the overlap, 49 mm under the centre of the block, which also turns).
  const double dt = 2e-3, m = 1.0, load = 20.0 * m * kG, slip = 1e-6;
  const double lever = 0.05 - 0.5 * load / (1e6 * 0.16);
  const double inv_mass = 1.0 / m + lever * lever / (m * 0.17 / 12.0);
  for (const bool distributed : {false, true}) {
    CAPTURE(distributed);
    double stiffness[2];
    for (const bool bounded : {false, true}) {
      Stack s = make_stack({.masses = {m}, .damping = 0.0, .distributed = distributed, .dt = bounded ? dt : 0.0});
      s.poses[0].p.z() = 0.05 - load / (1e6 * 0.16);
      s.model.evaluate(s.poses, s.twists, s.friction, &s.eval);
      s.friction.swap(s.eval.next_friction);
      s.poses[0].p.x() += slip;
      s.model.evaluate(s.poses, s.twists, s.friction, &s.eval);
      stiffness[bounded] = -s.eval.wrenches[0].force.x() / slip;
    }
    CHECK(stiffness[0] == doctest::Approx(0.5 * load / 1e-4).epsilon(1e-3));
    CHECK(stiffness[1] == doctest::Approx(1.0 / (inv_mass * dt * dt)).epsilon(1e-3));
  }
}

TEST_CASE("the normal damper takes at most the relative velocity of a step") {
  // 1 kg pressed by 1000 times its weight and closing at 1 mm/s: h·Fn = 49 kN·s/m as given.
  const double dt = 1e-3, load = 1000.0 * kG, v = 1e-3;
  for (const bool distributed : {false, true}) {
    CAPTURE(distributed);
    Stack s = make_stack({.masses = {1.0}, .distributed = distributed, .dt = dt});
    s.poses[0].p.z() = 0.05 - load / (1e6 * 0.16);
    s.twists[0].v.z() = -v;
    s.model.evaluate(s.poses, s.twists, s.friction, &s.eval);
    const double damper = s.eval.wrenches[0].force.z() - load;
    CHECK(damper > 0.0);
    CHECK(damper * dt <= 1.0 * v * (1.0 + 1e-9));  // The impulse of the step against m·v.
    CHECK(damper * dt > 0.2 * v);                  // Not thrown away either (rocking shares the bound).
  }
}

TEST_CASE("the explicit coupling needs the inertia of every body") {
  Stack s = make_stack({.masses = {1.0, 1.0}});
  s.model.set_explicit_coupling(1e-3, {ContactModel::BodyInertia{}});
  CHECK_THROWS_AS(s.model.evaluate(s.poses, s.twists, s.friction, &s.eval), std::invalid_argument);
  CHECK_THROWS_AS(s.model.set_explicit_coupling(-1.0, {}), std::invalid_argument);
}

TEST_CASE("the pair state keeps the load accelerations through its flat form") {
  FrictionState state{.active = true, .load_normal = 3.5, .load_tangent = 7.25};
  double z[FrictionState::kSize];
  state.pack(z);
  const FrictionState back = FrictionState::unpack(z);
  CHECK(back.load_normal == 3.5);
  CHECK(back.load_tangent == 7.25);
}
