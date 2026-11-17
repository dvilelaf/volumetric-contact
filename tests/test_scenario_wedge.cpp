#include <numbers>

#include <doctest/doctest.h>

#include "test_scenario_heldout.hpp"

namespace volcontact::scenarios {
std::vector<double> wedge_slide_velocity(const std::vector<double>& t);  // wedge.cpp; not in the public header.
}

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

const double kAngle = 20.0 * std::numbers::pi / 180.0;

// The block that holds at rest (displaced by `creep` at a steady rate) and the other at `scale`
// times the theory's velocity.
Trajectory theory_trajectory(const Scenario& s, double dt, double creep = 0.0, double scale = 1.0) {
  const std::vector<double> t = sample_times(dt, s.duration), v = wedge_slide_velocity(t);
  Trajectory traj(2);
  for (std::size_t i = 0; i < t.size(); ++i) {
    std::vector<Pose> X = {s.bodies[0].X_W0, s.bodies[1].X_W0};
    std::vector<Twist> V(2);
    X[0].p.x() += creep * t[i] / s.duration;
    V[0].v.x() = creep / s.duration;
    V[1].v.x() = scale * v[i];
    traj.push(t[i], X, V);
  }
  return traj;
}

}  // namespace

TEST_CASE("wedge: two 1 kg blocks, one with mu 10 % above tan 20 deg and one 10 % below; class A") {
  const Scenario s = make_scenario("wedge", kHeldOutOptions);
  CHECK(s.theory_class == TheoryClass::kA);
  REQUIRE(s.bodies.size() == 2);
  REQUIRE(s.contacts.size() == 2);
  CHECK(s.contacts[0].friction == doctest::Approx(0.40));
  CHECK(s.contacts[1].friction == doctest::Approx(0.33));
  CHECK(s.contacts[0].friction / std::tan(kAngle) == doctest::Approx(1.1).epsilon(0.01));
  CHECK(s.contacts[1].friction / std::tan(kAngle) == doctest::Approx(0.9).epsilon(0.01));
  CHECK(s.contacts[1].distributed.mu_static == doctest::Approx(0.33));
  CHECK(s.gravity.isApprox(Vec3(0.0, 0.0, -9.81)));
  CHECK(s.grounds[0].planar);
  for (const BodySpec& b : s.bodies) {
    CHECK(b.inertia.isApprox(box_inertia(1.0, 0.2, 0.2, 0.1)));
    CHECK(0.05 - b.X_W0.p.z() == doctest::Approx(1e-6).epsilon(1e-6));
    CHECK(b.V_W0.v.isZero());
  }
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, law).pairs().size() == 2);
    CHECK(initial_overlap(s, law) == doctest::Approx(2 * 0.04 * 1e-6).epsilon(1e-3));
  }
}

TEST_CASE("wedge: the applied force tilts gravity to 20 deg over 1 s at constant |g|") {
  const Scenario s = make_scenario("wedge", kHeldOutOptions);
  for (const BodySpec& b : s.bodies) {
    REQUIRE(static_cast<bool>(b.applied_force));
    CHECK(b.applied_force(0.0).isZero());
    for (const double t : {0.3, 1.0, 2.5}) {
      const Vec3 g = s.gravity + b.applied_force(t) / b.mass;
      CHECK(g.norm() == doctest::Approx(9.81));
      if (t >= 1.0) CHECK(std::atan2(g.x(), -g.z()) == doctest::Approx(kAngle));
    }
  }
}

TEST_CASE("wedge: the sliding block starts when tan(tilt) = mu and then accelerates at g (sin a - mu cos a)") {
  const std::vector<double> t = sample_times(1e-3, 3.0), v = wedge_slide_velocity(t);
  // Half-cosine ramp: the tilt reaches atan(0.33) at t0.
  const double t0 = std::acos(1.0 - 2.0 * std::atan(0.33) / kAngle) / std::numbers::pi;
  CHECK(t0 > 0.5);
  CHECK(t0 < 1.0);
  CHECK(v[static_cast<std::size_t>(t0 * 1000) - 1] == 0.0);
  CHECK(v[static_cast<std::size_t>(t0 * 1000) + 5] > 0.0);
  const double a = 9.81 * (std::sin(kAngle) - 0.33 * std::cos(kAngle));
  CHECK(v[3000] - v[2000] == doctest::Approx(a).epsilon(1e-9));
  CHECK(v[1000] < a * (1.0 - t0));  // During the ramp the slope is still below 20°.
}

TEST_CASE("wedge: error is within 1e-3 on the theory, and counts the creep of the held block and the velocity") {
  const Scenario s = make_scenario("wedge", kHeldOutOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(theory_trajectory(s, dt));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
  }
  // 20 mm of creep over the 0.2 m side, at 6.7 mm/s kept for the 3 s: both 0.1.
  const MetricReport crept = s.metrics(theory_trajectory(s, 1e-3, 0.02));
  CHECK(scalar_of(crept, "err_hold_creep") == doctest::Approx(0.1).epsilon(1e-6));
  CHECK(scalar_of(crept, "err_residual_speed") == doctest::Approx(0.1).epsilon(1e-6));
  CHECK(crept.error == doctest::Approx(0.1).epsilon(1e-6));
  const MetricReport slow = s.metrics(theory_trajectory(s, 1e-3, 0.0, 0.8));
  CHECK(scalar_of(slow, "err_slide_velocity") == doctest::Approx(0.2).epsilon(1e-6));
  // A block that does not slide at all scores 1.
  CHECK(s.metrics(theory_trajectory(s, 1e-3, 0.0, 0.0)).error == doctest::Approx(1.0));
}
