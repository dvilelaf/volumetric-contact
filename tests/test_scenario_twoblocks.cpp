#include <algorithm>
#include <cmath>
#include <numbers>
#include <string_view>
#include <vector>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};
const double kTheoryAccel = 9.81 * (std::sin(std::numbers::pi / 6) - 0.2 * std::cos(std::numbers::pi / 6));

std::vector<double> uniform_times(double dt, double duration) {
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  return t;
}

// Both blocks at their initial heights, the lower one at x(t) and the upper one at x(t) + slip_rate t.
Trajectory pair_trajectory(const std::vector<double>& times, const std::vector<double>& x, double slip_rate = 0.0) {
  Trajectory traj(2);
  const Pose lower{Mat3::Identity(), Vec3(0.0, 0.0, 0.05)}, upper{Mat3::Identity(), Vec3(0.0, 0.0, 0.15)};
  for (std::size_t i = 0; i < times.size(); ++i) {
    const double v = kTheoryAccel * times[i];
    Pose Xl = lower, Xu = upper;
    Xl.p.x() = x[i];
    Xu.p.x() = x[i] + slip_rate * times[i];
    Twist Vl{Vec3::Zero(), Vec3(v, 0.0, 0.0)}, Vu = Vl;
    traj.push(times[i], std::vector{Xl, Xu}, std::vector{Vl, Vu});
  }
  return traj;
}

// x(t) = a t² / 2: the pair slides from rest at the theory's acceleration.
std::vector<double> theory_positions(const std::vector<double>& times) {
  std::vector<double> x;
  for (double t : times) x.push_back(0.5 * kTheoryAccel * t * t);
  return x;
}

double find_scalar(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

}  // namespace

TEST_CASE("twoblocks: a 2 kg lower block and a 1 kg upper block of the true size, on a 30 degree incline") {
  const Scenario s = make_scenario("twoblocks", kOptions);
  REQUIRE(s.bodies.size() == 2);
  CHECK(s.name == "twoblocks");
  CHECK(s.gravity.isApprox(Vec3(9.81 * 0.5, 0.0, -9.81 * std::sqrt(3.0) / 2)));
  CHECK(s.duration == doctest::Approx(1.0));
  CHECK(s.bodies[0].mass == doctest::Approx(2.0));
  CHECK(s.bodies[0].inertia.isApprox(box_inertia(2.0, 0.4, 0.4, 0.1)));
  CHECK(s.bodies[1].mass == doctest::Approx(1.0));
  CHECK(s.bodies[1].inertia.isApprox(box_inertia(1.0, 0.2, 0.2, 0.1)));
  CHECK(s.bodies[0].X_W0.p.z() == doctest::Approx(0.05 - 1e-6));
  CHECK(s.bodies[1].X_W0.p.z() == doctest::Approx(0.15 - 2e-6));
  CHECK(s.bodies[1].X_W0.p.head<2>().isZero());
  CHECK(s.bodies[0].V_W0.v.isZero());
  CHECK(s.bodies[1].V_W0.v.isZero());
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
}

TEST_CASE("twoblocks: the ground pair has mu 0.2 and the interface pair mu 0.6") {
  const Scenario s = make_scenario("twoblocks", kOptions);
  REQUIRE(s.contacts.size() == 2);
  CHECK(s.contacts[0].a.kind == BodyRef::Kind::kBody);
  CHECK(s.contacts[0].a.index == 0);
  CHECK(s.contacts[0].b.kind == BodyRef::Kind::kGround);
  CHECK(s.contacts[0].friction == doctest::Approx(0.2));
  CHECK(s.contacts[1].a.index == 1);
  CHECK(s.contacts[1].b.kind == BodyRef::Kind::kBody);
  CHECK(s.contacts[1].b.index == 0);
  CHECK(s.contacts[1].friction == doctest::Approx(0.6));
}

TEST_CASE("twoblocks builds, and its contact model has one pair per contact, for both laws") {
  const Scenario s = make_scenario("twoblocks", kOptions);
  CHECK(static_cast<bool>(s.metrics));
  for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, model).pairs().size() == 2);
  }
}

TEST_CASE("twoblocks: at t = 0 the overlaps are the 1 µm layers, under the lower block and under the upper one") {
  const Scenario s = make_scenario("twoblocks", kOptions);
  const std::vector<Pose> poses = {s.bodies[0].X_W0, s.bodies[1].X_W0, s.grounds[0].X_WG};
  const std::vector<Twist> twists = {s.bodies[0].V_W0, s.bodies[1].V_W0, Twist{}};
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    const ContactModel model = make_contact_model(s, law);
    ContactModel::Evaluation eval;
    model.evaluate(poses, twists, model.initial_friction(), &eval);
    REQUIRE(eval.pairs.size() == 2);
    CHECK(eval.pairs[0].in_contact);
    CHECK(eval.pairs[1].in_contact);
    CHECK(eval.pairs[0].geometry.volume == doctest::Approx(0.16 * kInitialPenetration).epsilon(0.01));
    CHECK(eval.pairs[1].geometry.volume == doctest::Approx(0.04 * kInitialPenetration).epsilon(0.01));
  }
}

TEST_CASE("twoblocks: a pair that slides together at the theory has error below 1e-3, and no slip") {
  const Scenario s = make_scenario("twoblocks", kOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const std::vector<double> times = uniform_times(dt, s.duration);
    const MetricReport r = s.metrics(pair_trajectory(times, theory_positions(times)));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(find_scalar(r, "velocity_end_theory") == doctest::Approx(kTheoryAccel));
    CHECK(find_scalar(r, "velocity_upper_end") == doctest::Approx(kTheoryAccel));
    CHECK(find_scalar(r, "relative_slip_end_m") == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(r.series.size() == 5);
  }
}

TEST_CASE("twoblocks: an upper block that creeps 1 mm forward is charged that slip over its 0.2 m side") {
  const Scenario s = make_scenario("twoblocks", kOptions);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  const MetricReport r = s.metrics(pair_trajectory(times, theory_positions(times), 1e-3));
  CHECK(find_scalar(r, "relative_slip_end_m") == doctest::Approx(1e-3));
  CHECK(find_scalar(r, "err_slip") == doctest::Approx(5e-3));
  CHECK(find_scalar(r, "err_velocity") <= 1e-3);
  CHECK(r.error == doctest::Approx(5e-3));
  CHECK(s.theory_class == TheoryClass::kA);
}
