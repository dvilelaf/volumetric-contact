#include <array>

#include <doctest/doctest.h>
#include <Eigen/Geometry>

#include "test_scenario_heldout.hpp"

namespace volcontact::scenarios {
// edge.cpp; not in the public header.
std::vector<double> edge_pitch(const std::vector<double>& t);
std::array<double, 2> edge_winkler_pitch_times(double K, double h);
}  // namespace volcontact::scenarios

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

// The box pitching as the theory says, `delay` seconds late.
Trajectory theory_trajectory(const Scenario& s, double dt, double delay = 0.0) {
  const std::vector<double> t = sample_times(dt, s.duration);
  std::vector<double> shifted;
  for (const double ti : t) shifted.push_back(std::max(0.0, ti - delay));
  const std::vector<double> pitch = edge_pitch(shifted);
  Trajectory traj(1);
  for (std::size_t i = 0; i < t.size(); ++i) {
    Pose X = s.bodies[0].X_W0;
    X.R = Eigen::AngleAxisd(pitch[i], Vec3::UnitY()).toRotationMatrix();
    traj.push(t[i], std::vector{X}, std::vector{Twist{}});
  }
  return traj;
}

}  // namespace

TEST_CASE("edge: a 1 kg box sliding at 1 m/s, frictionless, 0.3 m before the edge of the table; class A") {
  const Scenario s = make_scenario("edge", kHeldOutOptions);
  CHECK(s.theory_class == TheoryClass::kA);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& b = s.bodies[0];
  CHECK(b.inertia.isApprox(box_inertia(1.0, 0.3, 0.2, 0.1)));
  CHECK(b.V_W0.v.isApprox(Vec3(1.0, 0.0, 0.0)));
  CHECK(b.X_W0.p.x() == doctest::Approx(0.5 - 0.3));  // The table's edge is at x = 0.5.
  CHECK(0.05 - b.X_W0.p.z() == doctest::Approx(1e-6).epsilon(1e-6));
  CHECK(s.contacts[0].friction == 0.0);
  CHECK_FALSE(s.grounds[0].planar);
  CHECK(s.grounds[0].shape.volume.has_value());
  for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, law).pairs().size() == 1);
    CHECK(initial_overlap(s, law) == doctest::Approx(0.06 * 1e-6).epsilon(1e-3));  // Wholly on the table.
  }
}

TEST_CASE("edge: the theory stays flat until the centre of mass is over the edge, then pitches") {
  const std::vector<double> t = sample_times(1e-3, 0.6), pitch = edge_pitch(t);
  for (std::size_t i = 0; i <= 300; ++i) CHECK(pitch[i] == 0.0);
  CHECK(pitch[310] > 0.0);
  // Values of the independent Newton–Euler integration (docs/scenarios.md §4.4): 0.1 rad at
  // 92.92 ms and 0.25 rad at 142.43 ms after the onset; the rear end leaves at 156.7 ms with 0.300 rad.
  CHECK(value_at(t, pitch, 0.3 + 0.09292) == doctest::Approx(0.1).epsilon(1e-3));
  CHECK(value_at(t, pitch, 0.3 + 0.14243) == doctest::Approx(0.25).epsilon(1e-3));
  CHECK(value_at(t, pitch, 0.3 + 0.1567) == doctest::Approx(0.300).epsilon(2e-3));
  // Just after the onset the torque is m·g·v·t about the edge: φ = m·g·v·t³/(6·(I + m·h_c²)).
  const double inertia = (0.09 + 0.01) / 12 + 0.05 * 0.05;
  CHECK(value_at(t, pitch, 0.32) == doctest::Approx(9.81 * 1.0 * 8e-6 / (6 * inertia)).epsilon(0.02));
  // In flight the pitch rate is constant.
  CHECK(pitch[600] - pitch[550] == doctest::Approx(pitch[550] - pitch[500]).epsilon(1e-9));
}

TEST_CASE("edge: error is within 1e-3 on the theory, and counts a late or early pitch") {
  const Scenario s = make_scenario("edge", kHeldOutOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(theory_trajectory(s, dt));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(scalar_of(r, "pitch_time_1_theory") == doctest::Approx(0.39292).epsilon(1e-4));
    CHECK(scalar_of(r, "pitch_time_2_theory") == doctest::Approx(0.44243).epsilon(1e-4));
  }
  // 10 ms late: over the 92.9 ms and the 142.4 ms the theory takes from the onset.
  const MetricReport late = s.metrics(theory_trajectory(s, 1e-3, 0.01));
  CHECK(scalar_of(late, "err_pitch_time_1") == doctest::Approx(0.01 / 0.09292).epsilon(1e-3));
  CHECK(scalar_of(late, "err_pitch_time_2") == doctest::Approx(0.01 / 0.14243).epsilon(1e-3));
  CHECK(late.error == doctest::Approx(0.01 / 0.09292).epsilon(1e-3));
  // A box that never pitches has no result.
  Trajectory flat(1);
  for (const double t : sample_times(1e-2, 0.6)) flat.push(t, std::vector{s.bodies[0].X_W0}, std::vector{Twist{}});
  CHECK(std::isnan(s.metrics(flat).error));
}

TEST_CASE("edge: a Winkler table lets the box pitch early; the times tend to the rigid ones as K grows") {
  // Values of the independent strip model (docs/scenarios.md §4.4), counted from the start.
  const auto soft = edge_winkler_pitch_times(1e7, 5.0), stiff = edge_winkler_pitch_times(1e10, 5.0);
  CHECK(soft[0] == doctest::Approx(0.3 + 0.0749).epsilon(2e-3));
  CHECK(soft[1] == doctest::Approx(0.3 + 0.1245).epsilon(2e-3));
  CHECK(stiff[0] == doctest::Approx(0.3 + 0.0908).epsilon(2e-3));
  CHECK(stiff[0] < 0.39292);
  const Scenario s = make_scenario("edge", kHeldOutOptions);
  const MetricReport r = s.metrics(theory_trajectory(s, 1e-3));
  CHECK(scalar_of(r, "pitch_time_1_winkler") == doctest::Approx(soft[0]));
}
