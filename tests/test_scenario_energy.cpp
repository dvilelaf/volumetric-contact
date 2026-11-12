#include <numbers>

#include <doctest/doctest.h>

#include "test_scenario_heldout.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

constexpr double kR = 0.3, kSphere = 0.1, kTheta0 = 0.4;
constexpr double kW2 = 9.81 / (kR - kSphere);

// The centre of the sphere as a point-mass pendulum of length R − r; the speed is `gain(t)` times
// the one that conserves energy.
template <typename Gain>
Trajectory swing(double dt, double duration, Gain gain) {
  const std::vector<double> t = sample_times(dt, duration), th = pendulum_angle(t, kTheta0, kW2);
  Trajectory traj(1);
  for (std::size_t i = 0; i < t.size(); ++i) {
    const double rate = std::sqrt(std::max(0.0, 2.0 * kW2 * (std::cos(th[i]) - std::cos(kTheta0))));
    const double speed = gain(t[i]) * (kR - kSphere) * rate;
    Pose X;
    X.p = Vec3((kR - kSphere) * std::sin(th[i]), 0.0, kR - (kR - kSphere) * std::cos(th[i]));
    Twist V;
    V.v = speed * Vec3(std::cos(th[i]), 0.0, std::sin(th[i]));
    traj.push(t[i], std::vector{X}, std::vector{V});
  }
  return traj;
}

}  // namespace

TEST_CASE("energy: the sphere of bowl without friction and without damping, in all three methods; class A") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("energy", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = res});
    CHECK(s.theory_class == TheoryClass::kA);
    REQUIRE(s.contacts.size() == 1);
    CHECK(s.contacts[0].friction == 0.0);
    CHECK(s.contacts[0].gonthier.damping == 0.0);
    CHECK(s.contacts[0].distributed.damping == 0.0);
    CHECK(s.contacts[0].distributed.mu_static == 0.0);
    CHECK(s.native.damping == 0.0);
    const Vec3 centre = s.bodies[0].X_W0.p - Vec3(0.0, 0.0, kR);
    CHECK(centre.norm() + kSphere - kR == doctest::Approx(1e-6).epsilon(1e-3));
    CHECK(std::atan2(centre.x(), -centre.z()) == doctest::Approx(kTheta0));
    CHECK(s.bodies[0].V_W0.v.isZero());
  }
}

TEST_CASE("energy: error is within 1e-3 on the sliding pendulum, whose period is 2 pi sqrt((R - r)/g) at small swings") {
  const Scenario s = make_scenario("energy", kHeldOutOptions);
  const double small = 2.0 * std::numbers::pi * std::sqrt((kR - kSphere) / 9.81);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(swing(dt, s.duration, [](double) { return 1.0; }));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(scalar_of(r, "period_theory") ==
          doctest::Approx(small * (1.0 + kTheta0 * kTheta0 / 16 + 11 * std::pow(kTheta0, 4) / 3072)).epsilon(1e-5));
    CHECK(std::abs(scalar_of(r, "energy_drift")) < 1e-3);
  }
}

TEST_CASE("energy: the error counts energy that appears or disappears, over the energy of the swing") {
  const Scenario s = make_scenario("energy", kHeldOutOptions);
  // The speed grows 5 % over the run: the kinetic energy of the last period is about 10 % higher,
  // and on average over a period half of the swing's energy is kinetic.
  const MetricReport r = s.metrics(swing(1e-3, s.duration, [&](double t) { return 1.0 + 0.05 * t / s.duration; }));
  CHECK(scalar_of(r, "energy_drift") > 0.02);
  CHECK(scalar_of(r, "energy_drift") < 0.06);
  CHECK(scalar_of(r, "err_energy_drift") == doctest::Approx(scalar_of(r, "energy_drift")));
  CHECK(r.error == doctest::Approx(scalar_of(r, "energy_drift")));
  const MetricReport loss = s.metrics(swing(1e-3, s.duration, [&](double t) { return 1.0 - 0.05 * t / s.duration; }));
  CHECK(scalar_of(loss, "energy_drift") < -0.02);
  CHECK(scalar_of(loss, "err_energy_drift") == doctest::Approx(-scalar_of(loss, "energy_drift")));
}
