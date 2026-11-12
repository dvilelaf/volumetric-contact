#include <algorithm>
#include <cmath>
#include <numbers>
#include <string_view>
#include <vector>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

namespace volcontact::scenarios {
double tip_equilibrium_tilt(double K);  // tip.cpp; not in the public header.
}

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};
const double kDeg = std::numbers::pi / 180.0;
const double kRigid = std::atan(1.0 / 3.0);  // Rigid tip, tan θ = w/h = 0.2/0.6.
const double kTheta = tip_equilibrium_tilt(1e7);  // θ* of §4, row 5 [rad].

std::vector<double> uniform_times(double dt, double duration) {
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  return t;
}

// The tilt of the gravity at time t, read back from the applied force: F_x = m g sin θ.
double tilt_at(const Scenario& s, double t) {
  return std::asin(s.bodies[0].applied_force(t).x() / (s.bodies[0].mass * 9.81));
}

// Time at which the tilt reaches theta, by bisection on tilt_at.
double time_at_tilt(const Scenario& s, double theta) {
  double lo = 0.0, hi = s.duration;
  for (int i = 0; i < 100; ++i) {
    const double mid = (lo + hi) / 2;
    if (tilt_at(s, mid) < theta) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return (lo + hi) / 2;
}

// The Winkler condition of §4, row 5, with the lean φ maximised on a log-spaced grid
// (a/2 = 0.1 m, h_c = 0.3 m): positive while the block holds.
double brute_margin(double theta, double K) {
  const double N = 9.81 * std::cos(theta);
  double best = -1.0;
  for (int i = 0; i <= 200000; ++i) {
    const double phi = 1e-6 * std::pow(1e5, i / 200000.0);
    best = std::max(best, 0.1 - 0.3 * phi - std::sqrt(2 * N / (K * 0.2 * phi)) / 3);
  }
  return best - 0.3 * std::tan(theta);
}

// The block at rest until the tilt reaches θ*, then turning about its downhill edge
// (x = a/2, z = 0) by 0.05 rad within 0.01 s.
Trajectory tipping_trajectory(const std::vector<double>& times, double tip_time) {
  Trajectory traj(1);
  const Vec3 edge_body(0.1, 0.0, -0.3), edge_world(0.1, 0.0, -1e-6);
  for (const double t : times) {
    const double phi = t < tip_time ? 0.0 : 0.05 * std::min(1.0, (t - tip_time) / 0.01);
    Pose X;
    X.R = Eigen::AngleAxisd(phi, Vec3::UnitY()).toRotationMatrix();
    X.p = edge_world - X.R * edge_body;
    traj.push(t, std::vector{X}, std::vector{Twist{}});
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

TEST_CASE("tip is a 1 kg box of the true size, standing on its face on the plane") {
  const Scenario s = make_scenario("tip", kOptions);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& box = s.bodies[0];
  CHECK(s.name == "tip");
  CHECK(box.mass == doctest::Approx(1.0));
  CHECK(box.inertia.isApprox(box_inertia(1.0, 0.2, 0.2, 0.6)));
  CHECK(s.gravity.isApprox(Vec3(0.0, 0.0, -9.81)));
  CHECK(s.duration == doctest::Approx(15.0));
  CHECK(box.V_W0.v.isZero());
  CHECK(box.V_W0.w.isZero());
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
}

TEST_CASE("tip's applied force is zero at t = 0 and keeps |g_eff| = g throughout") {
  const Scenario s = make_scenario("tip", kOptions);
  REQUIRE(static_cast<bool>(s.bodies[0].applied_force));
  CHECK(s.bodies[0].applied_force(0.0).norm() == doctest::Approx(0.0).epsilon(1e-12));
  for (const double t : {0.5, 1.0, 3.0, 5.0, 10.0, 15.0}) {
    const Vec3 g_eff = s.gravity + s.bodies[0].applied_force(t) / s.bodies[0].mass;
    CHECK(g_eff.norm() == doctest::Approx(9.81));
  }
  // At t = 15 s the tilt is 12° + 0.5°/s · 12 s = 18°.
  const Vec3 g_end = s.gravity + s.bodies[0].applied_force(15.0) / s.bodies[0].mass;
  CHECK(g_end.isApprox(9.81 * Vec3(std::sin(18.0 * kDeg), 0.0, -std::cos(18.0 * kDeg)), 1e-9));
}

TEST_CASE("tip's tilt is continuous, with a continuous rate, where the ramp ends at 3 s") {
  const Scenario s = make_scenario("tip", kOptions);
  const double h = 1e-4;
  const double value = tilt_at(s, 3.0);
  CHECK(value == doctest::Approx(12.0 * kDeg));
  const double left = (value - tilt_at(s, 3.0 - h)) / h;
  const double right = (tilt_at(s, 3.0 + h) - value) / h;
  CHECK(left / kDeg == doctest::Approx(0.5).epsilon(2e-3));
  CHECK(right / kDeg == doctest::Approx(0.5).epsilon(2e-3));
}

TEST_CASE("tip's equilibrium tilt solves its equation and tends to the rigid limit for a large K") {
  CHECK(kTheta / kDeg == doctest::Approx(16.2).epsilon(1e-3));
  CHECK(brute_margin(kTheta, 1e7) >= -1e-8);
  CHECK(brute_margin(kTheta + 1e-4, 1e7) < 0.0);
  CHECK(tip_equilibrium_tilt(1e18) == doctest::Approx(kRigid).epsilon(1e-5));
}

TEST_CASE("tip starts 1 µm into the plane") {
  const Scenario s = make_scenario("tip", kOptions);
  const BodySpec& box = s.bodies[0];
  const double overlap = 0.3 - box.X_W0.p.z();  // Below the bottom face, z = h/2 − 1 µm.
  CHECK(overlap == doctest::Approx(1e-6));
  CHECK(box.X_W0.R.isIdentity());
}

TEST_CASE("tip builds at both resolutions, and its contact model builds for both laws") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("tip", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    CHECK(static_cast<bool>(s.metrics));
    for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, model).pairs().size() == 1);
    }
  }
}

TEST_CASE("tip's error is within 1e-3 on a trajectory that tips at the equilibrium tilt") {
  const Scenario s = make_scenario("tip", kOptions);
  const double tip_time = time_at_tilt(s, kTheta);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(tipping_trajectory(uniform_times(dt, s.duration), tip_time));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(find_scalar(r, "tip_angle_theory") == doctest::Approx(kTheta));
    CHECK(find_scalar(r, "tip_angle_rigid") == doctest::Approx(kRigid));
    CHECK(find_scalar(r, "tip_time_theory") == doctest::Approx(tip_time).epsilon(1e-6));
    CHECK(std::abs(find_scalar(r, "tip_time") - tip_time) <= 0.02);
    CHECK(find_scalar(r, "slide") <= 1e-3);
  }
}

TEST_CASE("tip's error is NaN when the block never rotates 0.05 rad") {
  const Scenario s = make_scenario("tip", kOptions);
  Trajectory still(1);
  for (const double t : uniform_times(1e-2, s.duration)) {
    Pose X;
    X.p = Vec3(0.0, 0.0, 0.3 - 1e-6);
    still.push(t, std::vector{X}, std::vector{Twist{}});
  }
  CHECK(std::isnan(s.metrics(still).error));
}

TEST_CASE("tip's error counts the sliding before the tip over the 0.2 m side; class C") {
  const Scenario s = make_scenario("tip", kOptions);
  CHECK(s.theory_class == TheoryClass::kC);
  const double tip_time = time_at_tilt(s, kTheta);
  // The tipping trajectory, shifted steadily by 98 mm along the slope before it tips.
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  const Trajectory tipping = tipping_trajectory(times, tip_time);
  Trajectory slid(1);
  for (std::size_t i = 0; i < times.size(); ++i) {
    Pose X = tipping.pose(i, 0);
    X.p.x() += 0.098 * std::min(times[i], tip_time) / tip_time;
    slid.push(times[i], std::vector{X}, std::vector{Twist{}});
  }
  const MetricReport r = s.metrics(slid);
  CHECK(find_scalar(r, "err_tip_angle") <= 1e-3);
  CHECK(find_scalar(r, "err_slide") == doctest::Approx(0.49).epsilon(0.01));
  CHECK(r.error == doctest::Approx(find_scalar(r, "err_slide")));
}
