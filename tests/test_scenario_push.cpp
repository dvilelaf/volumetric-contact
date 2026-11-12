#include <algorithm>
#include <cmath>
#include <string_view>
#include <vector>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};

// Theory of §4: sliding from t* = μmg/c, then v = c(t − t*)²/(2m).
constexpr double kSlideStart = 0.5 * 1.0 * 9.81 / 2.0;

double theory_speed(double t) { return t > kSlideStart ? 2.0 * (t - kSlideStart) * (t - kSlideStart) / 2.0 : 0.0; }

std::vector<double> uniform_times(double dt, double duration) {
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  return t;
}

// A box on the plane, at rest in x until t*, then moving at the theory's speed.
Trajectory theory_trajectory(const std::vector<double>& times) {
  Trajectory traj(1);
  for (const double t : times) {
    Pose X;
    X.p = Vec3(0.0, 0.0, 0.1 - 1e-6);
    Twist V;
    V.v = Vec3(theory_speed(t), 0.0, 0.0);
    traj.push(t, std::vector{X}, std::vector{V});
  }
  return traj;
}

double find_scalar(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

}  // namespace

TEST_CASE("push is a 1 kg box of the true size, resting on the plane, pushed by a 2 N/s ramp") {
  const Scenario s = make_scenario("push", kOptions);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& box = s.bodies[0];
  CHECK(s.name == "push");
  CHECK(box.mass == doctest::Approx(1.0));
  CHECK(box.inertia.isApprox(box_inertia(1.0, 0.4, 0.4, 0.2)));
  CHECK(s.gravity.isApprox(Vec3(0.0, 0.0, -9.81)));
  CHECK(s.duration == doctest::Approx(4.0));
  CHECK(box.V_W0.v.isZero());
  CHECK(box.V_W0.w.isZero());
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
}

TEST_CASE("push's applied force is the ramp (2 N/s · t, 0, 0) at the centre of mass") {
  const Scenario s = make_scenario("push", kOptions);
  REQUIRE(static_cast<bool>(s.bodies[0].applied_force));
  for (const double t : {0.0, 1.0, 2.4525, 4.0}) {
    CHECK(s.bodies[0].applied_force(t).isApprox(Vec3(2.0 * t, 0.0, 0.0)));
  }
}

TEST_CASE("push starts 1 µm into the plane") {
  const Scenario s = make_scenario("push", kOptions);
  const BodySpec& box = s.bodies[0];
  const double overlap = 0.1 - box.X_W0.p.z();  // Below the bottom face, z = h/2 − 1 µm.
  CHECK(overlap == doctest::Approx(1e-6));
  CHECK(box.X_W0.R.isIdentity());
}

TEST_CASE("push builds at both resolutions, and its contact model builds for both laws") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("push", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    CHECK(static_cast<bool>(s.metrics));
    for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, model).pairs().size() == 1);
    }
  }
}

TEST_CASE("push's error is within 1e-3 on a trajectory that follows the theory") {
  const Scenario s = make_scenario("push", kOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(theory_trajectory(uniform_times(dt, s.duration)));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(find_scalar(r, "creep") == doctest::Approx(0.0));
    CHECK(std::abs(find_scalar(r, "onset_time") - find_scalar(r, "onset_time_theory")) <= dt);
  }
}

TEST_CASE("push's onset is the threshold crossing of the theory, a few tens of ms after t*") {
  const Scenario s = make_scenario("push", kOptions);
  const MetricReport r = s.metrics(theory_trajectory(uniform_times(1e-3, s.duration)));
  const double onset = find_scalar(r, "onset_time_theory");
  CHECK(onset > kSlideStart);
  CHECK(onset < kSlideStart + 0.05);
  CHECK(theory_speed(onset) == doctest::Approx(1e-3));
}

TEST_CASE("push's error counts the creep before the onset over the 0.4 m side, and a wrong onset time") {
  const Scenario s = make_scenario("push", kOptions);
  CHECK(s.theory_class == TheoryClass::kA);
  // The theory's velocity, on a box that crept 35 mm at a steady rate before t*.
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  Trajectory traj(1);
  for (const double t : times) {
    Pose X;
    X.p = Vec3(0.035 * std::min(t, kSlideStart) / kSlideStart, 0.0, 0.1 - 1e-6);
    Twist V;
    V.v = Vec3(theory_speed(t), 0.0, 0.0);
    traj.push(t, std::vector{X}, std::vector{V});
  }
  const MetricReport crept = s.metrics(traj);
  CHECK(find_scalar(crept, "err_creep") == doctest::Approx(0.035 / 0.4).epsilon(1e-3));
  CHECK(crept.error == doctest::Approx(0.035 / 0.4).epsilon(1e-3));

  // A box that moves at 2 mm/s from 0.05 s on: the onset is 98 % early.
  Trajectory early(1);
  for (const double t : times) {
    Pose X;
    X.p = Vec3(0.0, 0.0, 0.1 - 1e-6);
    Twist V;
    V.v = Vec3(std::max(theory_speed(t), t >= 0.05 ? 2e-3 : 0.0), 0.0, 0.0);
    early.push(t, std::vector{X}, std::vector{V});
  }
  const MetricReport r = s.metrics(early);
  CHECK(find_scalar(r, "err_onset_time") == doctest::Approx(1.0 - 0.05 / find_scalar(r, "onset_time_theory")));
  CHECK(r.error == doctest::Approx(find_scalar(r, "err_onset_time")));
}

TEST_CASE("push's onset is the time from which the box keeps moving: a passing transient is not charged") {
  const Scenario s = make_scenario("push", kOptions);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  Trajectory traj(1);
  for (const double t : times) {
    Pose X;
    X.p = Vec3(0.0, 0.0, 0.1 - 1e-6);
    Twist V;
    V.v = Vec3(t > 0.3 && t < 0.31 ? 5e-3 : theory_speed(t), 0.0, 0.0);  // 10 ms at 5 mm/s.
    traj.push(t, std::vector{X}, std::vector{V});
  }
  const MetricReport r = s.metrics(traj);
  CHECK(find_scalar(r, "err_onset_time") < 1e-3);
  CHECK(r.error < 1e-3);
}
