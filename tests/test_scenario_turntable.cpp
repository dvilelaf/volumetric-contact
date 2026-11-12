#include <numbers>

#include <doctest/doctest.h>

#include "test_scenario_heldout.hpp"

namespace volcontact::scenarios {
double turntable_orbit_ratio(double ball_mass, double table_mass);  // turntable.cpp; not in the public header.
}

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

// The table turning at 7 rad/s about a fixed axis and the ball on a circle of radius `radius(t)`
// about it at `rate` rad/s.
template <typename Radius>
Trajectory orbit(const Scenario& s, double dt, double rate, Radius radius) {
  Trajectory traj(2);
  for (const double t : sample_times(dt, s.duration)) {
    std::vector<Pose> X = {s.bodies[0].X_W0, s.bodies[1].X_W0};
    std::vector<Twist> V(2);
    X[0].p = Vec3(0.3, -0.1, 0.05);
    V[0].w = Vec3(0.0, 0.0, 7.0);
    X[1].p = X[0].p + Vec3(radius(t) * std::cos(rate * t), radius(t) * std::sin(rate * t), 0.15);
    traj.push(t, X, V);
  }
  return traj;
}

}  // namespace

TEST_CASE("turntable: the orbit rate is (2/7) of the table's for a fixed table, a little more for one that recoils") {
  CHECK(turntable_orbit_ratio(1.0, 1e12) == doctest::Approx(2.0 / 7.0).epsilon(1e-9));
  CHECK(turntable_orbit_ratio(1.0, 100.0) == doctest::Approx(0.28775).epsilon(1e-4));
  // Equal masses: the reduced mass is half, the rate Ω/(1 + 5/4).
  CHECK(turntable_orbit_ratio(1.0, 1.0) == doctest::Approx(4.0 / 9.0));
}

TEST_CASE("turntable: a 100 kg table free on a frictionless plane and a 1 kg ball rolling on it; class A") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("turntable", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = res});
    CHECK(s.theory_class == TheoryClass::kA);
    REQUIRE(s.bodies.size() == 2);
    const BodySpec& table = s.bodies[0];
    const BodySpec& ball = s.bodies[1];
    CHECK(table.inertia.isApprox(cylinder_inertia(100.0, 0.5, 0.1)));
    CHECK(ball.inertia.isApprox(sphere_inertia(1.0, 0.1)));
    CHECK(table.V_W0.w.isApprox(Vec3(0.0, 0.0, 7.0)));
    REQUIRE(s.contacts.size() == 2);
    CHECK(s.contacts[0].friction <= 1e-5);  // Table on the plane: as good as frictionless.
    CHECK(s.contacts[1].friction == doctest::Approx(0.5));
    // The pair's centre of mass is at the origin, at rest.
    CHECK((100.0 * table.X_W0.p + ball.X_W0.p).head<2>().norm() < 1e-12);
    CHECK((100.0 * table.V_W0.v + ball.V_W0.v).norm() < 1e-12);
    // 0.2 m from the table's axis, moving around it at the theory's rate.
    const Vec3 rel = ball.X_W0.p - table.X_W0.p, u = ball.V_W0.v - table.V_W0.v;
    const double ratio = turntable_orbit_ratio(1.0, 100.0);
    CHECK(rel.head<2>().norm() == doctest::Approx(0.2));
    CHECK(u.isApprox(ratio * 7.0 * Vec3::UnitZ().cross(rel.head<2>().homogeneous().cwiseProduct(Vec3(1, 1, 0)))));
    // No slip: the ball's contact point moves with the table's surface under it.
    const Vec3 contact = Vec3(rel.x(), rel.y(), 0.0);
    const Vec3 ball_point = ball.V_W0.v + ball.V_W0.w.cross(Vec3(0.0, 0.0, -0.1));
    const Vec3 table_point = table.V_W0.v + table.V_W0.w.cross(contact);
    CHECK((ball_point - table_point).norm() < 1e-12);
    CHECK(ball.V_W0.w.z() == doctest::Approx(7.0));  // No pivoting either.
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, law).pairs().size() == 2);
      CHECK(initial_overlap(s, law) > 0.0);
      CHECK(initial_overlap(s, law) < 1.01 * std::numbers::pi * 0.25 * 1e-6);
    }
  }
}

TEST_CASE("turntable: error is within 1e-3 on the theory's orbit, and counts its rate and its radius") {
  const Scenario s = make_scenario("turntable", kHeldOutOptions);
  const double rate = 7.0 * turntable_orbit_ratio(1.0, 100.0);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(orbit(s, dt, rate, [](double) { return 0.2; }));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(scalar_of(r, "table_rate") == doctest::Approx(7.0));
  }
  const MetricReport slow = s.metrics(orbit(s, 1e-3, 0.9 * rate, [](double) { return 0.2; }));
  CHECK(scalar_of(slow, "err_orbit_rate") == doctest::Approx(0.1).epsilon(1e-6));
  CHECK(slow.error == doctest::Approx(0.1).epsilon(1e-6));
  // An orbit that spirals out by 30 mm over the run.
  const MetricReport spiral = s.metrics(orbit(s, 1e-3, rate, [&](double t) { return 0.2 + 0.03 * t / s.duration; }));
  CHECK(scalar_of(spiral, "err_radius") == doctest::Approx(0.15).epsilon(1e-6));
  CHECK(scalar_of(spiral, "err_orbit_rate") < 1e-6);
  CHECK(spiral.error == doctest::Approx(0.15).epsilon(1e-6));
}
