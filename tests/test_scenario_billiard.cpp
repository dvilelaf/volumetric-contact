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

TEST_CASE("billiard is a 1 kg sphere of the true size, sliding at 2 m/s from 1 um into the plane") {
  const Scenario s = make_scenario("billiard", kOptions);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& ball = s.bodies[0];
  CHECK(s.name == "billiard");
  CHECK(ball.mass == doctest::Approx(1.0));
  CHECK(ball.inertia.isApprox(sphere_inertia(1.0, 0.1)));
  CHECK(s.gravity.isApprox(Vec3(0.0, 0.0, -9.81)));
  CHECK(s.duration == doctest::Approx(0.6));
  CHECK(s.theory_class == TheoryClass::kA);
  CHECK(ball.V_W0.v.isApprox(Vec3(2.0, 0.0, 0.0)));
  CHECK(ball.V_W0.w.isZero());
  // No contact model's static penetration: 1 µm, as every resting body.
  CHECK(0.1 - ball.X_W0.p.z() == doctest::Approx(1e-6).epsilon(1e-6));
  REQUIRE(ball.primitive.has_value());
  CHECK(ball.primitive->kind == Primitive::Kind::kSphere);
  CHECK(ball.primitive->size.x() == doctest::Approx(0.1));
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
}

TEST_CASE("billiard builds at both resolutions, and its contact model builds for both laws") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("billiard", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    CHECK(static_cast<bool>(s.metrics));
    for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, model).pairs().size() == 1);
    }
  }
}

TEST_CASE("at t = 0 the overlap with the plane is the intended cap, within the tessellation error") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("billiard", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
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

TEST_CASE("the theory slides, then rolls at t* = 2 v0 / (7 mu g) with v = 5 v0 / 7") {
  const Scenario s = make_scenario("billiard", kOptions);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  const std::vector<double> none(times.size(), 0.0);
  const MetricReport theory = s.metrics(ball_trajectory(times, 0.1, none, none));
  const double t_star = 2.0 * 2.0 / (7.0 * 0.2 * 9.81);
  CHECK(find_scalar(theory, "full_rolling_time_theory") == doctest::Approx(t_star));
  // The measured onset is the slip falling below 1e-3 v0, which the theory reaches at 0.999 t*.
  CHECK(find_scalar(theory, "rolling_start_time_theory") == doctest::Approx(0.999 * t_star));
  CHECK(find_scalar(theory, "rolling_start_velocity_theory") == doctest::Approx(2.0 - 0.2 * 9.81 * 0.999 * t_star));
  CHECK(find_scalar(theory, "velocity_end_theory") == doctest::Approx(5.0 * 2.0 / 7.0));
  const Series& v = find_series(theory, "v_theory");
  const Series& wr = find_series(theory, "wr_theory");
  const std::size_t sliding = static_cast<std::size_t>(0.5 * t_star / 1e-3);
  CHECK(v.values[sliding] == doctest::Approx(2.0 - 0.2 * 9.81 * times[sliding]));
  CHECK(v.values[sliding] > 5.0 * 2.0 / 7.0);
  CHECK(wr.values[sliding] < 5.0 * 2.0 / 7.0);
  CHECK(v.values.back() == doctest::Approx(wr.values.back()));
}

TEST_CASE("billiard's error is within 1e-3 on a trajectory that follows the theory") {
  const Scenario s = make_scenario("billiard", kOptions);
  const double z = s.bodies[0].X_W0.p.z();
  for (const double dt : {1e-3, 2e-4}) {
    const std::vector<double> times = uniform_times(dt, s.duration);
    const std::vector<double> none(times.size(), 0.0);
    const MetricReport theory = s.metrics(ball_trajectory(times, z, none, none));
    const MetricReport r = s.metrics(
        ball_trajectory(times, z, find_series(theory, "v_theory").values, find_series(theory, "wr_theory").values));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
  }
}

TEST_CASE("billiard's error is the onset: a ball that loses 5 % of its speed after it rolls is not charged") {
  const Scenario s = make_scenario("billiard", kOptions);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  const std::vector<double> none(times.size(), 0.0);
  const MetricReport theory = s.metrics(ball_trajectory(times, 0.1, none, none));
  std::vector<double> v = find_series(theory, "v_theory").values, wr = find_series(theory, "wr_theory").values;
  const double t_star = 2.0 * 2.0 / (7.0 * 0.2 * 9.81);
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (times[i] <= t_star) continue;
    const double loss = 1.0 - 0.05 * (times[i] - t_star) / (s.duration - t_star);
    v[i] *= loss;
    wr[i] *= loss;
  }
  const MetricReport r = s.metrics(ball_trajectory(times, 0.1, v, wr));
  CHECK(r.error <= 2e-3);
  CHECK(find_scalar(r, "velocity_end") == doctest::Approx(0.95 * 5.0 * 2.0 / 7.0));
}

TEST_CASE("billiard's error counts a late onset, and is NaN for a ball that never rolls") {
  const Scenario s = make_scenario("billiard", kOptions);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  std::vector<double> v, wr;
  // Friction 10 % weaker: the same rolling speed, reached 11 % later.
  const double mu = 0.18, t_star = 2.0 * 2.0 / (7.0 * mu * 9.81);
  for (const double t : times) {
    v.push_back(std::max(2.0 - mu * 9.81 * t, 10.0 / 7.0));
    wr.push_back(std::min(2.5 * mu * 9.81 * t, 10.0 / 7.0));
  }
  const MetricReport late = s.metrics(ball_trajectory(times, 0.1, v, wr));
  CHECK(late.error == doctest::Approx(1.0 / 9.0).epsilon(0.01));
  CHECK(find_scalar(late, "err_onset_time") == doctest::Approx(late.error));
  CHECK(find_scalar(late, "err_onset_velocity") < 2e-3);
  CHECK(find_scalar(late, "rolling_start_time") == doctest::Approx(0.999 * t_star).epsilon(1e-3));
  const std::vector<double> fast(times.size(), 2.0), none(times.size(), 0.0);
  CHECK(std::isnan(s.metrics(ball_trajectory(times, 0.1, fast, none)).error));
}
