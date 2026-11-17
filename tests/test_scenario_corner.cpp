#include <cmath>
#include <span>
#include <string>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const double kA = 0.1, kMu = 0.5, kK = 1e7, kM = 1.0, kDuration = 1.0;
const Vec3 kDir = Vec3(-1.0, -1.0, -2.0).normalized();

Scenario make() { return corner({.data_dir = VOLCONTACT_DATA_DIR}); }

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// Overlap volume of the cube with each of the three ground pieces, at its starting pose.
std::vector<double> overlaps(const Scenario& s, ForceModel law) {
  const ContactModel model = make_contact_model(s, law);
  const std::vector<Pose> poses = {s.bodies[0].X_W0, s.grounds[0].X_WG};
  const std::vector<Twist> twists = {Twist{}, Twist{}};
  ContactModel::Evaluation eval;
  model.evaluate(poses, twists, model.initial_friction(), &eval);
  std::vector<double> v;
  for (const auto& pair : eval.pairs) v.push_back(pair.in_contact ? pair.geometry.volume : 0.0);
  return v;
}

// Cube at rest with each face penetrating by the theoretical depth m g |dᵢ|/(K a²).
Trajectory theory_trajectory(double dt) {
  Trajectory traj(1);
  Pose X;
  for (int k = 0; k < 3; ++k) {
    const double pen = kM * kG * std::abs(kDir(k)) / (kK * kA * kA);
    X.p(k) = kA / 2 - pen;
  }
  Twist V;
  for (int i = 0; i <= std::lround(kDuration / dt); ++i) {
    traj.push(dt * i, std::span(&X, 1), std::span(&V, 1));
  }
  return traj;
}

}  // namespace

TEST_CASE("corner: builds with the true cube, and its contact model builds for both laws") {
  const Scenario s = make();
  CHECK(s.name == "corner");
  REQUIRE(s.bodies.size() == 1);
  CHECK(s.bodies[0].mass == doctest::Approx(1.0));
  CHECK(s.bodies[0].inertia.isApprox(box_inertia(1.0, kA, kA, kA)));
  CHECK(s.gravity.normalized().isApprox(kDir));
  CHECK(s.gravity.norm() == doctest::Approx(kG));
  CHECK(s.duration == doctest::Approx(kDuration));
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, law).pairs().size() == 3);
  }
}

TEST_CASE("corner: at t = 0 each face overlaps its ground by a 1 um layer") {
  const Scenario s = make();
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    const std::vector<double> v = overlaps(s, law);
    REQUIRE(v.size() == 3);
    for (const double volume : v) CHECK(volume == doctest::Approx(kA * kA * 1e-6).epsilon(1e-3));
  }
}

TEST_CASE("corner: a cube at the theoretical penetrations has error below 1e-3") {
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = make().metrics(theory_trajectory(dt));
    CHECK(r.error <= 1e-3);
    CHECK(scalar(r, "penetration_x_m_theory") == doctest::Approx(kM * kG * std::abs(kDir(0)) / (kK * kA * kA)));
    CHECK(scalar(r, "penetration_z_m_theory") == doctest::Approx(80.1e-6).epsilon(1e-3));
    CHECK(scalar(r, "penetration_x_m") == doctest::Approx(scalar(r, "penetration_x_m_theory")));
    CHECK(scalar(r, "residual_speed_mps") == doctest::Approx(0.0).scale(1.0));
    CHECK(r.series.size() == 6);
  }
}

TEST_CASE("corner: frictionless, class C; a cube still drifting at 10 mm/s is charged that drift") {
  const Scenario s = make();
  CHECK(s.theory_class == TheoryClass::kC);
  REQUIRE(s.contacts.size() == 1);
  CHECK(s.contacts[0].friction == 0.0);
  CHECK(s.contacts[0].distributed.mu_static == 0.0);
  REQUIRE(s.contacts[0].gonthier.friction.has_value());
  CHECK(s.contacts[0].gonthier.friction->mu_dynamic == 0.0);
  Trajectory traj(1);
  Pose X;
  for (int k = 0; k < 3; ++k) X.p(k) = kA / 2 - kM * kG * std::abs(kDir(k)) / (kK * kA * kA);
  const Twist V{Vec3::Zero(), Vec3(0.0, 0.0, 0.01)};
  for (int i = 0; i <= 1000; ++i) traj.push(1e-3 * i, std::span(&X, 1), std::span(&V, 1));
  const MetricReport r = s.metrics(traj);
  // 10 mm/s for the 1 s of the run, over the 0.1 m side.
  CHECK(scalar(r, "err_residual_speed") == doctest::Approx(0.1));
  CHECK(r.error == doctest::Approx(0.1));
}
