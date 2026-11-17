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

constexpr double kR = 0.3, kSphere = 0.1, kTheta0 = 0.4;
constexpr double kW2 = 5.0 * 9.81 / (7.0 * (kR - kSphere));

double find_scalar(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

// The sphere's centre at the angle `scale(t)·θ(t)` of the rolling pendulum.
template <typename Scale>
Trajectory swing(double dt, double duration, Scale scale) {
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  const std::vector<double> th = pendulum_angle(t, kTheta0, kW2);
  Trajectory traj(1);
  for (std::size_t i = 0; i < t.size(); ++i) {
    const double a = scale(t[i]) * th[i];
    Pose X;
    X.p = Vec3((kR - kSphere) * std::sin(a), 0.0, kR - (kR - kSphere) * std::cos(a));
    traj.push(t[i], std::vector{X}, std::vector{Twist{}});
  }
  return traj;
}

}  // namespace

TEST_CASE("bowl builds at both resolutions, and its contact model builds for both laws") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("bowl", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    CHECK(static_cast<bool>(s.metrics));
    CHECK(s.theory_class == TheoryClass::kA);
    CHECK(s.bodies[0].shape.pieces.size() == 1);
    CHECK(s.grounds[0].shape.pieces.size() > 100);
    for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, model).pairs().size() == s.grounds[0].shape.pieces.size());
    }
  }
}

TEST_CASE("bowl starts at rest, 0.4 rad up +x, 1 um into the true wall of a bowl with R - r = 200 mm") {
  const Scenario s = make_scenario("bowl", {.data_dir = VOLCONTACT_DATA_DIR});
  const BodySpec& b = s.bodies[0];
  const Vec3 centre = b.X_W0.p - Vec3(0.0, 0.0, kR);
  CHECK(centre.norm() + kSphere - kR == doctest::Approx(1e-6).epsilon(1e-3));
  CHECK(std::atan2(centre.x(), -centre.z()) == doctest::Approx(kTheta0));
  CHECK(centre.y() == 0.0);
  CHECK(b.V_W0.v.isZero());
  CHECK(b.V_W0.w.isZero());
  CHECK(b.inertia.isApprox(sphere_inertia(1.0, kSphere)));
  CHECK(kR - kSphere >= 0.05);
}

TEST_CASE("bowl: the period is the exact one of the rolling pendulum at 0.4 rad, 1 % above the small-amplitude one") {
  const Scenario s = make_scenario("bowl", {.data_dir = VOLCONTACT_DATA_DIR});
  const MetricReport r = s.metrics(swing(1e-3, s.duration, [](double) { return 1.0; }));
  const double small = 2.0 * std::numbers::pi * std::sqrt(7.0 * (kR - kSphere) / (5.0 * 9.81));
  // Series of the pendulum period: 1 + θ0²/16 + 11 θ0⁴/3072.
  const double series = small * (1.0 + kTheta0 * kTheta0 / 16 + 11 * std::pow(kTheta0, 4) / 3072);
  CHECK(find_scalar(r, "period_theory") == doctest::Approx(series).epsilon(1e-5));
  CHECK(find_scalar(r, "period") == doctest::Approx(series).epsilon(1e-4));
}

TEST_CASE("bowl's error is within 1e-3 on a trajectory that follows the theory") {
  const Scenario s = make_scenario("bowl", {.data_dir = VOLCONTACT_DATA_DIR});
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(swing(dt, s.duration, [](double) { return 1.0; }));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
  }
}

TEST_CASE("bowl's error counts the amplitude a swing loses, even with the right period") {
  const Scenario s = make_scenario("bowl", {.data_dir = VOLCONTACT_DATA_DIR});
  const MetricReport r = s.metrics(swing(1e-3, s.duration, [](double t) { return std::exp(-0.3 * t); }));
  const double t_peak = find_scalar(r, "amplitude_last_time");
  CHECK(t_peak > 2.5);
  CHECK(find_scalar(r, "err_period") < 5e-3);
  CHECK(find_scalar(r, "err_amplitude") == doctest::Approx(1.0 - std::exp(-0.3 * t_peak)).epsilon(0.02));
  CHECK(r.error == doctest::Approx(find_scalar(r, "err_amplitude")));
  CHECK(find_scalar(r, "amplitude_last_winkler") < kTheta0);
}
