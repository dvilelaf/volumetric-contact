#include <cmath>
#include <numbers>
#include <span>
#include <string>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};
const double kAngle = 30.0 * std::numbers::pi / 180.0;
const double kRoll = std::numbers::pi / 4;
const double kAccel = 9.81 * (std::sin(kAngle) - std::sqrt(2.0) * 0.2 * std::cos(kAngle));
const double kHeight = std::sqrt(2.0) * (0.05 - 1e-6);

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// The bar following the theory from x = −1.5 m, rolled by 45° about x.
Trajectory theory_trajectory(double dt) {
  const Mat3 R = Eigen::AngleAxisd(kRoll, Vec3::UnitX()).toRotationMatrix();
  Trajectory traj(1);
  for (int i = 0; i <= std::lround(1.0 / dt); ++i) {
    const double t = dt * i;
    Pose X{R, Vec3(-1.5 + 0.5 * kAccel * t * t, 0.0, kHeight)};
    Twist V;
    V.v = Vec3(kAccel * t, 0.0, 0.0);
    traj.push(t, std::span(&X, 1), std::span(&V, 1));
  }
  return traj;
}

}  // namespace

TEST_CASE("vgroove is the 1 kg bar of the true size, rolled 45° about x, on the groove at x = -1.5 m") {
  const Scenario s = make_scenario("vgroove", kOptions);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& bar = s.bodies[0];
  CHECK(s.name == "vgroove");
  CHECK(bar.mass == doctest::Approx(1.0));
  CHECK(bar.inertia.isApprox(box_inertia(1.0, 0.4, 0.1, 0.1)));
  CHECK(s.gravity.isApprox(inclined_gravity(30.0)));
  CHECK(s.duration == doctest::Approx(1.0));
  CHECK(bar.V_W0.v.isZero());
  CHECK(bar.V_W0.w.isZero());
  CHECK(bar.X_W0.R.isApprox(Eigen::AngleAxisd(std::numbers::pi / 4, Vec3::UnitX()).toRotationMatrix()));
  CHECK(bar.X_W0.p.x() == doctest::Approx(-1.5));
  CHECK(bar.X_W0.p.y() == doctest::Approx(0.0));
  REQUIRE(s.grounds.size() == 1);
  CHECK_FALSE(s.grounds[0].planar);
  CHECK(s.grounds[0].shape.pieces.size() == 2);
}

TEST_CASE("vgroove's contact model builds for both laws, with one pair per groove face") {
  const Scenario s = make_scenario("vgroove", kOptions);
  for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, model).pairs().size() == 2);
  }
}

TEST_CASE("at t = 0 each face overlaps its groove face by the 1 µm layer, and nothing else") {
  const Scenario s = make_scenario("vgroove", kOptions);
  for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    const ContactModel contacts = make_contact_model(s, model);
    const std::vector<Pose> poses{s.bodies[0].X_W0, s.grounds[0].X_WG};
    const std::vector<Twist> twists(poses.size());
    ContactModel::Evaluation out;
    contacts.evaluate(poses, twists, contacts.initial_friction(), &out);
    REQUIRE(out.pairs.size() == 2);
    // Each layer is the face (0.4 m × 0.1 m) times 1 µm.
    for (const auto& pair : out.pairs) {
      REQUIRE(pair.in_contact);
      CHECK(pair.geometry.volume == doctest::Approx(0.04 * 1e-6).epsilon(0.01));
    }
  }
}

TEST_CASE("the theory slides from rest with a = g (sin θ − √2 μ cos θ), with no lateral drift or roll") {
  const Scenario s = make_scenario("vgroove", kOptions);
  const MetricReport theory = s.metrics(theory_trajectory(1e-3));
  CHECK(scalar(theory, "velocity_end_theory") == doctest::Approx(kAccel * 1.0));
  CHECK(kAccel == doctest::Approx(2.5).epsilon(0.01));
  CHECK(scalar(theory, "lateral_drift_max_m") == doctest::Approx(0.0));
  CHECK(scalar(theory, "roll_max_rad") < 1e-12);
}

TEST_CASE("vgroove's error is within 1e-3 on a trajectory that follows the theory") {
  const Scenario s = make_scenario("vgroove", kOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(theory_trajectory(dt));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(scalar(r, "lateral_drift_max_m") < 1e-12);
    CHECK(scalar(r, "roll_max_rad") < 1e-12);
    CHECK(r.series.size() == 2);
  }
}
