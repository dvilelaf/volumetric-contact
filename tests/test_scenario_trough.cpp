#include <algorithm>
#include <cmath>
#include <string_view>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

constexpr double kR = 0.1, kCavity = 0.3, kTheta0 = 0.4;
constexpr double kW2 = 2.0 * 9.81 / (3.0 * (kCavity - kR));

Scenario make(Resolution res) { return trough({.data_dir = VOLCONTACT_DATA_DIR, .resolution = res}); }

double find_scalar(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

// The cylinder following the theory: angle θ(t) about the cavity axis, rolling without slipping.
Trajectory theory_trajectory(double dt, double duration, double decay = 0.0) {
  Trajectory traj(1);
  const double d = kCavity - kR;
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  const std::vector<double> angle = pendulum_angle(t, kTheta0, kW2);
  for (std::size_t i = 0; i < t.size(); ++i) {
    const double th = angle[i] * std::exp(-decay * t[i]);
    Pose X;
    X.p = Vec3(d * std::sin(th), 0.0, kCavity - d * std::cos(th));
    traj.push(t[i], std::vector{X}, std::vector{Twist{}});
  }
  return traj;
}

}  // namespace

TEST_CASE("trough: builds at both resolutions, with the inertia of the true cylinder") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    CHECK(s.name == "trough");
    REQUIRE(s.bodies.size() == 1);
    CHECK(s.bodies[0].mass == doctest::Approx(1.0));
    CHECK(s.bodies[0].inertia.isApprox(cylinder_inertia(1.0, kR, 0.4)));
    CHECK(s.duration == doctest::Approx(3.5));
    CHECK(s.theory_class == TheoryClass::kA);
    CHECK_FALSE(s.grounds[0].planar);
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, law).pairs().size() >= 1);
    }
  }
}

TEST_CASE("trough: the cylinder axis starts on the circle R - r, 1 µm into the true cavity wall") {
  const Scenario s = make(Resolution::kBase);
  const Vec3 p = s.bodies[0].X_W0.p;
  const double dist = std::hypot(p.x(), p.z() - kCavity);
  CHECK(dist + kR - kCavity == doctest::Approx(kInitialPenetration).epsilon(1e-6));
  CHECK(std::atan2(p.x(), kCavity - p.z()) == doctest::Approx(kTheta0));
  CHECK(kCavity - kR >= 0.05);
  CHECK((s.bodies[0].X_W0.R * Vec3::UnitZ()).isApprox(Vec3::UnitY()));
}

TEST_CASE("trough: no overlap at t = 0 beyond the intended 1 µm, in both laws") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      const ContactModel model = make_contact_model(s, law);
      const std::vector<Pose> poses = {s.bodies[0].X_W0, s.grounds[0].X_WG};
      const std::vector<Twist> twists = {s.bodies[0].V_W0, Twist{}};
      ContactModel::Evaluation eval;
      model.evaluate(poses, twists, model.initial_friction(), &eval);
      double volume = 0.0;
      for (const auto& pair : eval.pairs) {
        if (pair.in_contact) volume += pair.geometry.volume;
      }
      // Footprint at most the 0.4 m length by a 0.05 m chord, 1 µm deep.
      CHECK_MESSAGE(volume <= 0.4 * 0.05 * kInitialPenetration * 2.0, "volume = " << volume);
    }
  }
}

TEST_CASE("trough: error is within 1e-3 on a trajectory that follows the theory") {
  const Scenario s = make(Resolution::kBase);
  // Exact pendulum period at 0.4 rad: the small-amplitude one times 1 + θ0²/16 + 11 θ0⁴/3072.
  const double period = 2.0 * std::numbers::pi * std::sqrt(3.0 * (kCavity - kR) / (2.0 * 9.81)) *
                        (1.0 + kTheta0 * kTheta0 / 16 + 11 * std::pow(kTheta0, 4) / 3072);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(theory_trajectory(dt, s.duration));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(find_scalar(r, "period_theory") == doctest::Approx(period).epsilon(1e-5));
  }
}

TEST_CASE("trough: error counts the amplitude a swing loses, even with the right period") {
  const Scenario s = make(Resolution::kBase);
  const MetricReport r = s.metrics(theory_trajectory(1e-3, s.duration, 0.3));
  const double t_peak = find_scalar(r, "amplitude_last_time");
  CHECK(find_scalar(r, "err_period") < 5e-3);
  CHECK(find_scalar(r, "err_amplitude") == doctest::Approx(1.0 - std::exp(-0.3 * t_peak)).epsilon(0.02));
  CHECK(r.error == doctest::Approx(find_scalar(r, "err_amplitude")));
}
