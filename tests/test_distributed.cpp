#include <cmath>
#include <numbers>

#include <doctest/doctest.h>

#include "test_util.hpp"
#include "volcontact/contact_model.hpp"
#include "volcontact/distributed.hpp"
#include "volcontact/detail/polar_moments.hpp"

using namespace volcontact;
using volcontact::test::make_box;
using volcontact::test::make_step;

namespace {

const auto kCube = std::make_shared<const CollisionShape>(make_box(Vec3::Constant(-0.5), Vec3::Constant(0.5)));
const auto kGround = std::make_shared<const CollisionShape>(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));

struct Result {
  bool hit = false;
  IntersectionProperties geo;
  ContactForce force;
  FrictionState next;
};

Result evaluate(const CollisionShape& a, const Pose& X_WA, const Twist& V_WA, const DistributedParams& p,
                const FrictionState& state = {}) {
  IntersectionWorkspace iws;
  ContactSurface surface;
  DistributedWorkspace dws;
  Result r;
  const Pose X_WG;
  const Twist V_WG;
  r.hit = intersect(a, X_WA, *kGround, X_WG, iws, &r.geo, &surface);
  if (r.hit) r.force = distributed_force(r.geo, surface, {a, X_WA, V_WA}, {*kGround, X_WG, V_WG}, p, state, &r.next, dws);
  return r;
}

DistributedParams kinetic_only(double mu) {
  DistributedParams p{.stiffness = 1e6, .damping = 0.0, .mu_dynamic = mu, .mu_static = mu};
  p.stick.reset();
  return p;
}

// Mean distance to the centre over a regular n-gon of circumradius R.
double ngon_mean_radius(int n, double R) {
  const double half = std::numbers::pi / n, apothem = R * std::cos(half);
  const int steps = 20000;
  double integral = 0.0;  // ∫∫ r·r dr dθ over one triangle.
  for (int i = 0; i < steps; ++i) {
    const double theta = -half + (i + 0.5) * 2.0 * half / steps;
    const double rmax = apothem / std::cos(theta);
    integral += rmax * rmax * rmax / 3.0 * (2.0 * half / steps);
  }
  const double area = 0.5 * n * R * R * std::sin(2.0 * half);
  return n * integral / area;
}

// Brute-force midpoint sums over a fine grid of the triangle.
PolarMoments numeric_moments(const Vec3& q, const Vec3& a, const Vec3& b, const Vec3& c) {
  PolarMoments m;
  const int n = 600;
  const double cell = 0.5 * (b - a).cross(c - a).norm() / (n * n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n - i; ++j) {
      for (int up = 0; up < (j < n - i - 1 ? 2 : 1); ++up) {
        const double u = (i + (up ? 2.0 : 1.0) / 3.0) / n, v = (j + (up ? 2.0 : 1.0) / 3.0) / n;
        const Vec3 r = a + u * (b - a) + v * (c - a) - q;
        const double d = r.norm();
        m.distance += cell * d;
        m.weighted_r += cell * d * r;
        m.unit_r += cell / d * r;
        m.outer_unit_r += cell / d * r * r.transpose();
      }
    }
  }
  return m;
}

}  // namespace

TEST_CASE("closed-form polar moments match brute-force integration") {
  const Vec3 a(0, 0, 0), b(2, 0.3, 0), c(0.4, 1.5, 0), m = Vec3::UnitZ();
  for (const Vec3& q : {Vec3(0.7, 0.5, 0), Vec3(3, -1, 0), Vec3(-0.5, 2, 0), a}) {
    const PolarMoments exact = polar_moments(q, a, b, c, m), approx = numeric_moments(q, a, b, c);
    CHECK(exact.distance == doctest::Approx(approx.distance).epsilon(1e-4));
    CHECK(exact.weighted_r.isApprox(approx.weighted_r, 1e-4));
    CHECK(exact.unit_r.isApprox(approx.unit_r, 1e-3));
    CHECK(exact.outer_unit_r.isApprox(approx.outer_unit_r, 1e-3));
  }
}

TEST_CASE("a flat contact carries K·V centred under the volume") {
  const Pose X{Mat3::Identity(), Vec3(0.1, -0.2, 0.5 - 1e-4)};
  const Result r = evaluate(*kCube, X, Twist{}, kinetic_only(0.5));
  REQUIRE(r.hit);
  CHECK(r.force.normal_force == doctest::Approx(1e6 * r.geo.volume));
  CHECK(r.force.force.isApprox(Vec3(0, 0, r.force.normal_force), 1e-9));
  CHECK(r.force.point.head<2>().isApprox(r.geo.centroid.head<2>(), 1e-9));
  CHECK(r.force.torque.norm() == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("sliding friction is mu·Fn against the slip") {
  const Pose X{Mat3::Identity(), Vec3(0, 0, 0.5 - 1e-4)};
  const Twist V{Vec3::Zero(), Vec3(0.3, 0.4, 0)};
  const Result r = evaluate(*kCube, X, V, kinetic_only(0.5));
  REQUIRE(r.hit);
  // The thin side strips of V (depth/size = 1e-4) add their own small friction.
  const Vec3 tangential(r.force.force.x(), r.force.force.y(), 0.0);
  CHECK(tangential.isApprox(-0.5 * r.force.normal_force * Vec3(0.6, 0.8, 0), 2e-4));
}

TEST_CASE("spinning friction integrates mu·p·|r| over the patch") {
  const auto disk = CollisionShape::from_obj(VOLCONTACT_DATA_DIR "/bodies/cylinder_r250_h50_seg64/surface.obj");
  const double depth = 1e-4, mu = 0.6;
  const Pose X{Mat3::Identity(), Vec3(0, 0, 0.025 - depth)};
  const Twist spin{Vec3(0, 0, 10.0), Vec3::Zero()};
  const Result r = evaluate(*disk, X, spin, kinetic_only(mu));
  REQUIRE(r.hit);
  const double expected = mu * r.force.normal_force * ngon_mean_radius(64, 0.25);
  CHECK(-r.force.torque.z() == doctest::Approx(expected).epsilon(5e-3));
}

TEST_CASE("stick holds up to the static limit and slips exactly at it") {
  DistributedParams p{.stiffness = 1e6, .damping = 0.0, .mu_dynamic = 0.5, .mu_static = 0.5};
  p.stick = StickParams{.v_stick = 0.01, .k_stick = 400.0, .c_stick = 0.0, .travel = 0.0};  // Constant mode.
  // 10 µm deep: Fn = K·V = 10 N, so the static limit is 5 N.
  const Pose X0{Mat3::Identity(), Vec3(0, 0, 0.5 - 1e-5)};
  const Result first = evaluate(*kCube, X0, Twist{}, p);
  REQUIRE(first.next.active);

  // 1 mm away from the anchor: spring force k·d, anchor kept.
  const Pose X1{Mat3::Identity(), Vec3(1e-3, 0, 0.5 - 1e-5)};
  const Result hold = evaluate(*kCube, X1, Twist{}, p, first.next);
  CHECK(hold.force.force.x() == doctest::Approx(-0.4));
  CHECK(hold.next.deflection_B.isApprox(Vec3(1e-3, 0, 0)));

  // 10 cm away: the spring saturates at μ·Fn and the anchor follows to keep it there.
  const Pose X2{Mat3::Identity(), Vec3(0.1, 0, 0.5 - 1e-5)};
  const Result slip = evaluate(*kCube, X2, Twist{}, p, first.next);
  const double limit = 0.5 * slip.force.normal_force;
  CHECK(slip.force.force.x() == doctest::Approx(-limit));
  const Result after = evaluate(*kCube, X2, Twist{}, p, slip.next);
  CHECK(after.force.force.x() == doctest::Approx(-limit));
}

TEST_CASE("stick reaches the static limit after the same travel whatever the load") {
  DistributedParams p{.stiffness = 1e6, .damping = 0.0, .mu_dynamic = 0.5, .mu_static = 0.5};
  const double d = p.stick->travel;
  for (const double load : {1.0, 1000.0}) {  // Fn = K·V = 1e6·depth for the unit cube.
    const double z = 0.5 - load / 1e6;
    const Result first = evaluate(*kCube, Pose{Mat3::Identity(), Vec3(0, 0, z)}, Twist{}, p);
    REQUIRE(first.next.active);
    const double limit = 0.5 * first.force.normal_force;
    const auto at = [&](double x) {
      return evaluate(*kCube, Pose{Mat3::Identity(), Vec3(x, 0, z)}, Twist{}, p, first.next);
    };
    CHECK(at(0.3 * d).force.force.x() == doctest::Approx(-0.3 * limit).epsilon(1e-6));
    CHECK(at(0.3 * d).next.deflection_B.x() == doctest::Approx(0.3 * d).epsilon(1e-6));
    CHECK(at(0.01).force.force.x() == doctest::Approx(-limit).epsilon(1e-6));  // Slips at the limit.
    CHECK(at(0.01).next.deflection_B.x() == doctest::Approx(d).epsilon(1e-6));
  }
}

TEST_CASE("bodies of 1 kg and 100 kg held by stiction are stable with explicit Euler at 1 ms") {
  DistributedParams p{.stiffness = 1e6, .damping = 0.0, .mu_dynamic = 0.5, .mu_static = 0.5};
  const double d = p.stick->travel, dt = 1e-3;
  for (const double mass : {1.0, 100.0}) {
    const double weight = mass * 9.81, z = 0.5 - weight / 1e6;
    const double pull = 0.5 * 0.5 * weight;  // Half the static limit.
    double x = 0.0, v = 0.0, x_mid = 0.0, peak = 0.0;
    FrictionState state;
    for (int i = 0; i < 5000; ++i) {
      const Result r =
          evaluate(*kCube, Pose{Mat3::Identity(), Vec3(x, 0, z)}, Twist{Vec3::Zero(), Vec3(v, 0, 0)}, p, state);
      state = r.next;
      const double a = (pull + r.force.force.x()) / mass;
      x += dt * v;
      v += dt * a;
      peak = std::max(peak, std::abs(x));
      if (i == 2499) x_mid = x;
    }
    CHECK(x == doctest::Approx(0.5 * d).epsilon(1e-3));  // F/(μN)·d.
    CHECK(x == doctest::Approx(x_mid).epsilon(1e-4));    // No creep.
    CHECK(peak < 0.7 * d);                               // Well damped, never reaches the limit.
    CHECK(std::abs(v) < 1e-6);
  }
}

TEST_CASE("a block on an incline slides at g(sinθ − μcosθ) above the friction angle and holds below it") {
  DistributedParams p{.stiffness = 1e6, .damping = 0.0, .mu_dynamic = 0.5, .mu_static = 0.5};
  const double dt = 1e-3, g = 9.81, mu = 0.5, mass = 1.0;
  for (const double deg : {30.0, 20.0}) {
    const double th = deg * std::numbers::pi / 180.0;
    const double pull = mass * g * std::sin(th), z = 0.5 - mass * g * std::cos(th) / 1e6;
    double x = 0.0, v = 0.0, x_mid = 0.0;
    FrictionState state;
    for (int i = 0; i < 1000; ++i) {
      const Result r =
          evaluate(*kCube, Pose{Mat3::Identity(), Vec3(x, 0, z)}, Twist{Vec3::Zero(), Vec3(v, 0, 0)}, p, state);
      state = r.next;
      x += dt * v;
      v += dt * (pull + r.force.force.x()) / mass;
      if (i == 499) x_mid = x;
    }
    if (deg > 26.6) {
      CHECK(v == doctest::Approx(g * (std::sin(th) - mu * std::cos(th))).epsilon(0.02));
    } else {
      CHECK(x < 2.0 * p.stick->travel);
      CHECK(x == doctest::Approx(x_mid).epsilon(1e-4));
    }
  }
}

TEST_CASE("the distributed law rejects non-convex shapes") {
  ContactModel model;
  const auto step = std::make_shared<const CollisionShape>(make_step());
  CHECK_FALSE(step->convex());
  CHECK_THROWS(model.add_pair({.body_a = 0, .body_b = 1, .shape_a = step, .shape_b = kGround,
                               .law = DistributedParams{}}));
  CHECK_NOTHROW(model.add_pair({.body_a = 0, .body_b = 1, .shape_a = step, .shape_b = kGround,
                                .law = GonthierParams{}}));
}

TEST_CASE("convex pieces of a body add up to the whole") {
  // The step solid as two convex boxes against a slab: same volume as the
  // non-convex mesh.
  const auto lower = std::make_shared<const CollisionShape>(make_box(Vec3(0, 0, 0), Vec3(2, 1, 1)));
  const auto upper = std::make_shared<const CollisionShape>(make_box(Vec3(1, 0, 1), Vec3(3, 1, 2)));
  const auto slab = std::make_shared<const CollisionShape>(make_box(Vec3(-1, -1, 0.5), Vec3(4, 2, 1)));
  ContactModel model;
  for (const auto& piece : {lower, upper}) {
    model.add_pair({.body_a = 0, .body_b = 1, .shape_a = piece, .shape_b = slab, .law = DistributedParams{}});
  }
  const std::vector<Pose> X(2);
  const std::vector<Twist> V(2);
  ContactModel::Evaluation eval;
  model.evaluate(X, V, model.initial_friction(), &eval);
  double volume = 0.0;
  for (const auto& r : eval.pairs) volume += r.in_contact ? r.geometry.volume : 0.0;
  CHECK(volume == doctest::Approx(1.0));
}

TEST_CASE("parallel evaluation matches serial evaluation") {
  ContactModel model;
  const int n = 24;
  std::vector<Pose> X(n + 1);
  std::vector<Twist> V(n + 1);
  for (int i = 0; i < n; ++i) {
    model.add_pair({.body_a = i, .body_b = n, .shape_a = kCube, .shape_b = kGround,
                    .law = (i % 2) ? ContactLaw(DistributedParams{}) : ContactLaw(GonthierParams{})});
    X[i].p = Vec3(0.1 * i - 1.0, 0.0, 0.49 - 0.001 * i);
    V[i].v = Vec3(0.1, 0.0, -0.01);
  }
  ContactModel::Evaluation serial, parallel;
  model.set_parallel_threshold(1 << 30);
  model.evaluate(X, V, model.initial_friction(), &serial);
  model.set_parallel_threshold(1);
  model.evaluate(X, V, model.initial_friction(), &parallel);
  for (int i = 0; i <= n; ++i) {
    CHECK(parallel.wrenches[i].force == serial.wrenches[i].force);
    CHECK(parallel.wrenches[i].torque == serial.wrenches[i].torque);
  }
}
