#include <numbers>

#include <doctest/doctest.h>
#include <Eigen/Geometry>

#include "test_scenario_heldout.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

constexpr double kMu = 0.3, kSpeed = 1.5, kHeading = 25.0 * std::numbers::pi / 180.0;

// The four boxes of `pieces`: the two that slide follow Coulomb's theory from their initial poses;
// the two that spin share one motion, the one on the cut ground shifted by `cut_shift` and turned
// by `cut_turn` at the end.
Trajectory theory_trajectory(const Scenario& s, double dt, double cut_shift = 0.0, double cut_turn = 0.0,
                             double mu = kMu) {
  const Vec3 heading(std::cos(kHeading), std::sin(kHeading), 0.0);
  const double t_stop = kSpeed / (mu * 9.81);
  Trajectory traj(4);
  for (const double t : sample_times(dt, s.duration)) {
    const double tc = std::min(t, t_stop), f = t / s.duration;
    std::vector<Pose> X(4);
    std::vector<Twist> V(4);
    for (int k = 0; k < 4; ++k) {
      X[k] = s.bodies[k].X_W0;
      if (k % 2 == 0) {
        X[k].p += heading * (kSpeed * tc - 0.5 * mu * 9.81 * tc * tc);
        V[k].v = heading * (kSpeed - mu * 9.81 * tc);
      } else {
        X[k].p += Vec3(0.3 * f, 0.1 * f, 0.0);
        X[k].R = Eigen::AngleAxisd(2.0 * f, Vec3::UnitZ()).toRotationMatrix();
      }
    }
    X[3].p.x() += cut_shift * f;
    X[3].R = X[3].R * Eigen::AngleAxisd(cut_turn * f, Vec3::UnitZ()).toRotationMatrix();
    traj.push(t, X, V);
  }
  return traj;
}

}  // namespace

TEST_CASE("pieces: two boxes on the whole ground and two on the same ground cut into 1, 2, 8 or 64 pieces") {
  const std::pair<const char*, std::size_t> cases[] = {{"pieces1", 1}, {"pieces2", 2}, {"pieces8", 8}, {"pieces64", 64}};
  for (const auto& [name, count] : cases) {
    const Scenario s = make_scenario(name, kHeldOutOptions);
    CHECK(s.name == name);
    CHECK(s.theory_class == TheoryClass::kA);
    REQUIRE(s.bodies.size() == 4);
    REQUIRE(s.grounds.size() == 2);
    CHECK(s.grounds[0].shape.pieces.size() == 1);
    CHECK(s.grounds[1].shape.pieces.size() == count);
    CHECK(s.grounds[0].planar);
    CHECK(s.grounds[1].planar);
    // The same initial conditions on both grounds.
    const Vec3 gap = s.grounds[1].X_WG.p - s.grounds[0].X_WG.p;
    for (int k = 0; k < 2; ++k) {
      CHECK((s.bodies[k + 2].X_W0.p - s.bodies[k].X_W0.p).isApprox(gap));
      CHECK(s.bodies[k + 2].X_W0.R.isApprox(s.bodies[k].X_W0.R));
      CHECK(s.bodies[k + 2].V_W0.v.isApprox(s.bodies[k].V_W0.v));
      CHECK(s.bodies[k + 2].V_W0.w.isApprox(s.bodies[k].V_W0.w));
      CHECK(s.bodies[k].inertia.isApprox(box_inertia(1.0, 0.3, 0.2, 0.1)));
    }
    CHECK(s.bodies[0].V_W0.w.isZero());
    CHECK(s.bodies[1].V_W0.w.z() == doctest::Approx(5.0));
    CHECK(s.bodies[0].V_W0.v.norm() == doctest::Approx(kSpeed));
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, law).pairs().size() == 2 + 2 * count);
      // Each box starts with its 0.3 × 0.2 m face 1 µm into its ground, whatever the cut.
      CHECK(initial_overlap(s, law) == doctest::Approx(4 * 0.06 * 1e-6).epsilon(1e-3));
    }
  }
}

TEST_CASE("pieces: the boxes stay on their tables and cross the seam x = 0 of every decomposition") {
  const Scenario s = make_scenario("pieces64", kHeldOutOptions);
  const double travel = kSpeed * kSpeed / (2 * kMu * 9.81);
  for (int k = 0; k < 2; ++k) {
    const Vec3 start = s.bodies[k + 2].X_W0.p - s.grounds[1].X_WG.p;
    const Vec3 end = start + travel * Vec3(std::cos(kHeading), std::sin(kHeading), 0.0);
    CHECK(start.x() < -0.2);
    CHECK(end.x() > 0.0);
    // Half a diagonal of the box (0.18 m) inside the 1.6 m table at both ends.
    CHECK(std::max({std::abs(start.x()), std::abs(start.y()), std::abs(end.x()), std::abs(end.y())}) < 0.8 - 0.18);
  }
  // The two boxes of a table are apart by more than their diagonals.
  CHECK((s.bodies[1].X_W0.p - s.bodies[0].X_W0.p).norm() > 2 * 0.18 + 0.3);
}

TEST_CASE("pieces: error is within 1e-3 on Coulomb's sliding with the same motion on both grounds") {
  const Scenario s = make_scenario("pieces8", kHeldOutOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(theory_trajectory(s, dt));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(scalar_of(r, "travel_theory") == doctest::Approx(kSpeed * kSpeed / (2 * kMu * 9.81)));
    CHECK(scalar_of(r, "cut_spin_shift_m") == doctest::Approx(0.0).scale(1.0));
  }
}

TEST_CASE("pieces: the error counts a difference between the cut ground and the whole one, and a wrong friction") {
  const Scenario s = make_scenario("pieces8", kHeldOutOptions);
  // The spinning box ends 30 mm away and 0.1 rad off on the cut ground: 0.1 and 0.05.
  const MetricReport cut = s.metrics(theory_trajectory(s, 1e-3, 0.03, 0.1));
  CHECK(scalar_of(cut, "err_cut_spin") == doctest::Approx(0.1).epsilon(1e-6));
  CHECK(scalar_of(cut, "err_cut_spin_rotation") == doctest::Approx(0.05).epsilon(1e-6));
  CHECK(scalar_of(cut, "err_cut_slide") < 1e-9);
  CHECK(cut.error == doctest::Approx(0.1).epsilon(1e-6));
  // Friction 10 % high on both grounds: 9 % less travel, no difference between the grounds.
  const MetricReport mu = s.metrics(theory_trajectory(s, 1e-3, 0.0, 0.0, 1.1 * kMu));
  CHECK(scalar_of(mu, "err_travel") == doctest::Approx(1.0 - 1.0 / 1.1).epsilon(1e-3));
  CHECK(scalar_of(mu, "err_stop_time") == doctest::Approx(1.0 - 1.0 / 1.1).epsilon(2e-3));
  CHECK(scalar_of(mu, "err_cut_slide") < 1e-9);
}
