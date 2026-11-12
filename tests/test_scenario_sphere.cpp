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
const double kAngle = 20.0 * std::numbers::pi / 180.0;

std::vector<double> uniform_times(double dt, double duration) {
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  return t;
}

// A ball of radius 0.1 m at height z[i], with centre speed vx[i] and rolling speed wr[i] = ω·r.
Trajectory ball_trajectory(const std::vector<double>& times, double z, const std::vector<double>& vx,
                           const std::vector<double>& wr) {
  Trajectory traj(1);
  for (std::size_t i = 0; i < times.size(); ++i) {
    Pose X;
    X.p = Vec3(0.0, 0.0, z);
    Twist V;
    V.v = Vec3(vx[i], 0.0, 0.0);
    V.w = Vec3(0.0, wr[i] / 0.1, 0.0);
    traj.push(times[i], std::vector{X}, std::vector{V});
  }
  return traj;
}

const Series& find_series(const MetricReport& r, std::string_view name) {
  return *std::find_if(r.series.begin(), r.series.end(), [&](const Series& s) { return s.name == name; });
}

double find_scalar(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

}  // namespace

TEST_CASE("sphere is a 1 kg sphere of the true size, at rest on a 20 deg incline, 1 um into the plane") {
  const Scenario s = make_scenario("sphere", kOptions);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& ball = s.bodies[0];
  CHECK(s.name == "sphere");
  CHECK(ball.mass == doctest::Approx(1.0));
  CHECK(ball.inertia.isApprox(sphere_inertia(1.0, 0.1)));
  CHECK(s.gravity.isApprox(inclined_gravity(20.0)));
  CHECK(s.duration == doctest::Approx(1.5));
  CHECK(ball.V_W0.v.isZero());
  CHECK(ball.V_W0.w.isZero());
  // No contact model's static penetration: 1 µm, as every resting body.
  CHECK(0.1 - ball.X_W0.p.z() == doctest::Approx(1e-6).epsilon(1e-6));
  CHECK(s.theory_class == TheoryClass::kA);
  REQUIRE(ball.primitive.has_value());
  CHECK(ball.primitive->kind == Primitive::Kind::kSphere);
  CHECK(ball.primitive->size.x() == doctest::Approx(0.1));
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
}

TEST_CASE("sphere builds at both resolutions, and its contact model builds for both laws") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("sphere", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    CHECK(static_cast<bool>(s.metrics));
    for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, model).pairs().size() == 1);
    }
  }
}

TEST_CASE("at t = 0 the overlap with the plane is the intended cap, within the tessellation error") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("sphere", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    const ContactModel contacts = make_contact_model(s, ForceModel::kGonthier);
    const std::vector<Pose> poses{s.bodies[0].X_W0, s.grounds[0].X_WG};
    const std::vector<Twist> twists(poses.size());
    ContactModel::Evaluation out;
    contacts.evaluate(poses, twists, contacts.initial_friction(), &out);
    REQUIRE(out.pairs.size() == 1);
    REQUIRE(out.pairs[0].in_contact);

    const double delta = 0.1 - s.bodies[0].X_W0.p.z();
    const double cap = std::numbers::pi * delta * delta * (0.1 - delta / 3);
    // The polygonal sphere lies inside the true one, so its cap is no larger.
    CHECK_MESSAGE(out.pairs[0].geometry.volume <= cap * (1 + 1e-9),
                  "volume " << out.pairs[0].geometry.volume << ", cap " << cap);
    CHECK(out.pairs[0].geometry.volume > 0.0);
  }
}

TEST_CASE("the theory rolls from rest with a = 5/7 g sin θ and no slip") {
  const Scenario s = make_scenario("sphere", kOptions);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  const std::vector<double> none(times.size(), 0.0);
  const MetricReport theory = s.metrics(ball_trajectory(times, 0.1, none, none));
  const double a = 5.0 / 7.0 * 9.81 * std::sin(kAngle);
  CHECK(find_scalar(theory, "velocity_end_theory") == doctest::Approx(a * 1.5));
  CHECK(find_scalar(theory, "slip_max_mps_theory") == 0.0);
  CHECK(find_scalar(theory, "lateral_drift_max_m_theory") == 0.0);
  const Series& v = find_series(theory, "v_theory");
  const Series& wr = find_series(theory, "wr_theory");
  CHECK(v.values[1000] == doctest::Approx(a * 1.0));
  CHECK(v.values == wr.values);
}

TEST_CASE("sphere's error is within 1e-3 on a trajectory that follows the theory") {
  const Scenario s = make_scenario("sphere", kOptions);
  const double z = s.bodies[0].X_W0.p.z();
  for (const double dt : {1e-3, 2e-4}) {
    const std::vector<double> times = uniform_times(dt, s.duration);
    const std::vector<double> none(times.size(), 0.0);
    const MetricReport theory = s.metrics(ball_trajectory(times, z, none, none));
    const MetricReport r = s.metrics(
        ball_trajectory(times, z, find_series(theory, "v_theory").values, find_series(theory, "wr_theory").values));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(find_scalar(r, "slip_max_mps") < 1e-12);
    CHECK(find_scalar(r, "lateral_drift_max_m") < 1e-12);
  }
}

TEST_CASE("sphere measures the acceleration at rest: a resistance proportional to the speed is reported apart") {
  const Scenario s = make_scenario("sphere", kOptions);
  const double a = 5.0 / 7.0 * 9.81 * std::sin(kAngle), c = 0.06;
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  std::vector<double> v;
  for (const double t : times) v.push_back(a / c * (1.0 - std::exp(-c * t)));  // v̇ = a − c·v.
  const MetricReport r = s.metrics(ball_trajectory(times, 0.1, v, v));
  CHECK(find_scalar(r, "velocity_end") < 0.96 * find_scalar(r, "velocity_end_theory"));
  CHECK(r.error <= 2e-3);
  CHECK(find_scalar(r, "err_acceleration") == doctest::Approx(r.error));
  CHECK(find_scalar(r, "rolling_resistance_rate") == doctest::Approx(c).epsilon(0.05));
  // The Winkler prediction of that rate: N·h·a²/(6·m·r²·(7/5)) with a² = 2·r·δ.
  const double N = 9.81 * std::cos(kAngle), delta = sphere_static_penetration(1.0, N, 1e8, 0.1);
  CHECK(find_scalar(r, "rolling_resistance_rate_winkler") ==
        doctest::Approx(N * 5.0 * 2.0 * 0.1 * delta / (6.0 * 0.01 * 1.4)));
}

TEST_CASE("sphere's error counts a wrong acceleration and a slipping contact") {
  const Scenario s = make_scenario("sphere", kOptions);
  const double a = 5.0 / 7.0 * 9.81 * std::sin(kAngle);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  std::vector<double> slow, th;
  for (const double t : times) {
    slow.push_back(0.9 * a * t);
    th.push_back(a * t);
  }
  CHECK(s.metrics(ball_trajectory(times, 0.1, slow, slow)).error == doctest::Approx(0.1).epsilon(1e-6));
  const MetricReport slip = s.metrics(ball_trajectory(times, 0.1, th, slow));
  CHECK(find_scalar(slip, "err_slip") == doctest::Approx(0.1).epsilon(1e-6));
  CHECK(slip.error == doctest::Approx(0.1).epsilon(1e-6));
}
