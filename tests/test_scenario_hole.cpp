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

const double kA = 0.1, kDelta = 1e-4, kMu = 0.5, kBigK = 1e8, kM = 1.0;
const double kStiff = 4 * kMu * kBigK * kDelta * kA;

Scenario make() { return hole({.data_dir = VOLCONTACT_DATA_DIR}); }

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// Total overlap volume of the cube, centred with its bottom face `depth` below the entry, per wall.
std::vector<double> overlaps(const Scenario& s, ForceModel law, double depth) {
  const ContactModel model = make_contact_model(s, law);
  std::vector<Pose> poses = {s.bodies[0].X_W0, s.grounds[0].X_WG};
  poses[0].p.z() = kA / 2 - depth;
  const std::vector<Twist> twists = {Twist{}, Twist{}};
  ContactModel::Evaluation eval;
  model.evaluate(poses, twists, model.initial_friction(), &eval);
  std::vector<double> v;
  for (const auto& pair : eval.pairs) v.push_back(pair.in_contact ? pair.geometry.volume : 0.0);
  return v;
}

// Cube following the theory: z = z_stop (1 − cos ωt)/2 up to the half period, then held.
Trajectory theory_trajectory(double dt) {
  const double w = std::sqrt(kStiff / kM), z_stop = 2 * kM * kG / kStiff;
  Trajectory traj(1);
  for (int i = 0; i <= std::lround(0.5 / dt); ++i) {
    const double t = dt * i, tc = std::min(t, std::numbers::pi / w);
    Pose X;
    X.p = Vec3(0, 0, kA / 2 - 0.5 * z_stop * (1 - std::cos(w * tc)));
    Twist V;
    V.v = Vec3(0, 0, t < std::numbers::pi / w ? -0.5 * z_stop * w * std::sin(w * t) : 0.0);
    traj.push(t, std::span(&X, 1), std::span(&V, 1));
  }
  return traj;
}

}  // namespace

TEST_CASE("hole: builds with the true cube, and its contact model builds for both laws") {
  const Scenario s = make();
  CHECK(s.name == "hole");
  REQUIRE(s.bodies.size() == 1);
  CHECK(s.bodies[0].mass == doctest::Approx(1.0));
  CHECK(s.bodies[0].inertia.isApprox(box_inertia(1.0, kA, kA, kA)));
  CHECK(s.bodies[0].X_W0.p.z() == doctest::Approx(kA / 2));
  CHECK(s.duration == doctest::Approx(0.5));
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, law).pairs().size() == 4);
  }
}

TEST_CASE("hole: no overlap at the entry; 5 mm deep, delta*a*5 mm per wall") {
  const Scenario s = make();
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    for (const double v : overlaps(s, law, 0.0)) CHECK(v == doctest::Approx(0.0).epsilon(1e-9));
    const std::vector<double> deep = overlaps(s, law, 5e-3);
    REQUIRE(deep.size() == 4);
    for (const double v : deep) CHECK(v == doctest::Approx(kDelta * kA * 5e-3).epsilon(0.01));
  }
}

TEST_CASE("hole: a cube that follows the theory has error below 1e-3") {
  const double z_stop = 2 * kM * kG / kStiff;
  CHECK(z_stop == doctest::Approx(9.81e-3));
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = make().metrics(theory_trajectory(dt));
    CHECK(r.error <= 1e-3);
    CHECK(scalar(r, "depth_final_m_theory") == doctest::Approx(z_stop));
    CHECK(scalar(r, "time_max_depth") == doctest::Approx(scalar(r, "time_max_depth_theory")).epsilon(1e-2));
    CHECK(scalar(r, "drift_max_m") == doctest::Approx(0.0).scale(1.0));
    CHECK(r.series.size() == 2);
  }
}

TEST_CASE("hole: class C; a cube that keeps sinking at 2 mm/s after the stop is charged that drift") {
  const Scenario s = make();
  CHECK(s.theory_class == TheoryClass::kC);
  const Trajectory theory = theory_trajectory(1e-3);
  Trajectory traj(1);
  for (std::size_t i = 0; i < theory.size(); ++i) {
    const Pose X = theory.pose(i, 0);
    Twist V = theory.twist(i, 0);
    if (theory.times()[i] > 0.3) V.v.z() = -2e-3;
    traj.push(theory.times()[i], std::span(&X, 1), std::span(&V, 1));
  }
  const MetricReport r = s.metrics(traj);
  // 2 mm/s for the 0.5 s of the run, over the 0.1 m side.
  CHECK(scalar(r, "err_residual_speed") == doctest::Approx(0.01));
  CHECK(r.error == doctest::Approx(0.01));
}
