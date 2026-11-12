#include <numbers>

#include <doctest/doctest.h>

#include "test_scenario_heldout.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

// Every block at its initial pose; the top one moves `lean` during the first 3 s·`settle_share`
// and creeps `creep` more, at a steady rate, from then to the end.
Trajectory rest_trajectory(const Scenario& s, double settle, double lean, double creep) {
  const int n = static_cast<int>(s.bodies.size());
  Trajectory traj(n);
  for (const double t : sample_times(1e-3, s.duration)) {
    std::vector<Pose> X;
    std::vector<Twist> V(n);
    for (const BodySpec& b : s.bodies) X.push_back(b.X_W0);
    X.back().p.x() += lean * std::min(1.0, t / settle);
    if (t > settle) {
      X.back().p.x() += creep * (t - settle) / (s.duration - settle);
      V.back().v.x() = creep / (s.duration - settle);
    }
    traj.push(t, X, V);
  }
  return traj;
}

}  // namespace

TEST_CASE("tower: ten 1 kg blocks 400 x 400 x 100 mm, tilted to 5 deg for 10 s, resultant in the middle third") {
  const Scenario s = make_scenario("tower", kHeldOutOptions);
  CHECK(s.theory_class == TheoryClass::kA);
  CHECK(s.duration == doctest::Approx(10.0));
  REQUIRE(s.bodies.size() == 10);
  REQUIRE(s.contacts.size() == 10);
  for (int k = 0; k < 10; ++k) {
    CHECK(s.bodies[k].mass == doctest::Approx(1.0));
    CHECK(s.bodies[k].inertia.isApprox(box_inertia(1.0, 0.4, 0.4, 0.1)));
    CHECK(s.bodies[k].X_W0.p.z() == doctest::Approx((k + 0.5) * 0.1 - (k + 1) * 1e-6));
    const Vec3 g = s.gravity + s.bodies[k].applied_force(5.0) / s.bodies[k].mass;
    CHECK(g.norm() == doctest::Approx(9.81));
    CHECK(std::atan2(g.x(), -g.z()) == doctest::Approx(5.0 * std::numbers::pi / 180.0));
    CHECK(s.bodies[k].applied_force(0.0).isZero());
  }
  // The whole tower's centre of mass is 0.5 m up: its weight crosses the base 44 mm off centre.
  CHECK(0.5 * std::tan(5.0 * std::numbers::pi / 180.0) < 0.4 / 6);
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, law).pairs().size() == 10);
    CHECK(initial_overlap(s, law) == doctest::Approx(10 * 0.16 * 1e-6).epsilon(1e-3));
  }
}

TEST_CASE("massratio: a 1000 kg block on a 1 kg block of the same size; class A") {
  const Scenario s = make_scenario("massratio", kHeldOutOptions);
  CHECK(s.theory_class == TheoryClass::kA);
  REQUIRE(s.bodies.size() == 2);
  CHECK(s.bodies[0].mass == doctest::Approx(1.0));
  CHECK(s.bodies[1].mass == doctest::Approx(1000.0));
  CHECK(s.bodies[1].inertia.isApprox(box_inertia(1000.0, 0.4, 0.4, 0.1)));
  CHECK(s.bodies[1].X_W0.p.z() > s.bodies[0].X_W0.p.z());
  CHECK_FALSE(static_cast<bool>(s.bodies[0].applied_force));
  // Static penetration of the heavy block under 1 % of its height.
  CHECK(1001 * 9.81 / (s.native.stiffness * 0.16) < 0.01 * 0.1);
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, law).pairs().size() == 2);
  }
}

TEST_CASE("tower and massratio: a stack at rest has error 0; the elastic lean before settling is not charged") {
  for (const auto& [name, settle] : {std::pair{"tower", 3.0}, std::pair{"massratio", 0.5}}) {
    const Scenario s = make_scenario(name, kHeldOutOptions);
    const MetricReport rest = s.metrics(rest_trajectory(s, settle, 0.0, 0.0));
    CHECK(rest.error < 1e-12);
    const MetricReport lean = s.metrics(rest_trajectory(s, settle, 4e-3, 0.0));
    CHECK(lean.error < 1e-9);
    CHECK(scalar_of(lean, "displacement_top_settled_m") == doctest::Approx(4e-3).epsilon(1e-3));
  }
}

TEST_CASE("tower: the error counts the creep after the settling over the 0.4 m side, and the residual speed") {
  const Scenario s = make_scenario("tower", kHeldOutOptions);
  // 14 mm in the 7 s after the settling: 2 mm/s, which over the 10 s of the run is 20 mm.
  const MetricReport r = s.metrics(rest_trajectory(s, 3.0, 0.0, 0.014));
  CHECK(scalar_of(r, "creep_top_m") == doctest::Approx(0.014).epsilon(1e-3));
  CHECK(scalar_of(r, "err_creep") == doctest::Approx(0.035).epsilon(1e-3));
  CHECK(scalar_of(r, "err_residual_speed") == doctest::Approx(0.05).epsilon(1e-3));
  CHECK(r.error == doctest::Approx(0.05).epsilon(1e-3));
}

TEST_CASE("tower: a run that blew up has no result, not a perfect score") {
  const Scenario s = make_scenario("tower", kHeldOutOptions);
  Trajectory traj = rest_trajectory(s, 3.0, 0.0, 0.0);
  std::vector<Pose> X;
  for (const BodySpec& b : s.bodies) X.push_back(b.X_W0);
  X[4].p = Vec3::Constant(std::nan(""));
  traj.push(s.duration + 1e-3, X, std::vector<Twist>(10));
  CHECK(std::isnan(s.metrics(traj).error));
  CHECK(std::isnan(max_abs({0.0, std::nan(""), 1.0})));
  CHECK(std::isnan(worse(std::nan(""), 1.0)));
  CHECK(worse(2.0, 1.0) == 2.0);
}

