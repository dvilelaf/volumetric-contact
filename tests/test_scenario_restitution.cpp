#include <doctest/doctest.h>

#include "test_scenario_heldout.hpp"

namespace volcontact::scenarios {
// restitution.cpp; not in the public header.
double hunt_crossley_rebound(double impact, double h);
double winkler_sphere_rebound(double impact, double m, double r, double K, double h);
}  // namespace volcontact::scenarios

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

constexpr double kSpeeds[4] = {0.5, 1.0, 2.0, 4.0};

// Each ball falls from rest, touches the plane for one or two samples and leaves at `rebound[k]`
// at a sample time.
Trajectory bounce_trajectory(const Scenario& s, double dt, const double rebound[4]) {
  Trajectory traj(4);
  for (const double t : sample_times(dt, s.duration)) {
    std::vector<Pose> X(4);
    std::vector<Twist> V(4);
    for (int k = 0; k < 4; ++k) {
      const double t_hit = kSpeeds[k] / 9.81, gap = kSpeeds[k] * kSpeeds[k] / (2 * 9.81);
      const double t_leave = (std::floor(t_hit / dt) + 2.0) * dt;
      X[k] = s.bodies[k].X_W0;
      if (t < t_hit) {
        X[k].p.z() = 0.1 + gap - 0.5 * 9.81 * t * t;
        V[k].v.z() = -9.81 * t;
      } else if (t < t_leave - 0.5 * dt) {
        X[k].p.z() = 0.1 - 1e-4;  // In contact.
      } else {
        const double u = t - t_leave;
        X[k].p.z() = 0.1 + 1e-9 + rebound[k] * u - 0.5 * 9.81 * u * u;
        V[k].v.z() = rebound[k] - 9.81 * u;
        if (X[k].p.z() < 0.1) {  // Back on the plane: at rest.
          X[k].p.z() = 0.1 - 1e-4;
          V[k].v.z() = 0.0;
        }
      }
    }
    traj.push(t, X, V);
  }
  return traj;
}

}  // namespace

TEST_CASE("restitution: four 1 kg spheres at rest at the heights that give 0.5, 1, 2 and 4 m/s; class C") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("restitution", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = res});
    CHECK(s.theory_class == TheoryClass::kC);
    REQUIRE(s.bodies.size() == 4);
    for (int k = 0; k < 4; ++k) {
      const BodySpec& b = s.bodies[k];
      CHECK(b.V_W0.v.isZero());
      CHECK(std::sqrt(2 * 9.81 * (b.X_W0.p.z() - 0.1)) == doctest::Approx(kSpeeds[k]));
      CHECK(b.inertia.isApprox(sphere_inertia(1.0, 0.1)));
      if (k > 0) CHECK(b.X_W0.p.x() - s.bodies[k - 1].X_W0.p.x() > 0.2);  // More than a diameter apart.
      CHECK(s.contacts[k].gonthier.damping == doctest::Approx(0.5));
      CHECK(s.contacts[k].distributed.damping == doctest::Approx(0.5));
    }
    CHECK(s.native.damping == doctest::Approx(0.5));
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, law).pairs().size() == 4);
      CHECK(initial_overlap(s, law) == 0.0);
    }
  }
}

TEST_CASE("restitution: the Hunt-Crossley rebound solves its equation, tends to 1/h and to the impact speed") {
  for (const double h : {0.5, 5.0}) {
    for (const double u : {0.01, 0.5, 2.0}) {
      const double out = hunt_crossley_rebound(u, h);
      CHECK(h * u - std::log1p(h * u) == doctest::Approx(-h * out - std::log1p(-h * out)).epsilon(1e-9));
      CHECK(out < u);
      CHECK(out < 1.0 / h);
    }
    CHECK(hunt_crossley_rebound(1e-3 / h, h) == doctest::Approx(1e-3 / h).epsilon(1e-3));  // Elastic when slow.
    CHECK(hunt_crossley_rebound(100.0 / h, h) == doctest::Approx(1.0 / h).epsilon(1e-6));
  }
  CHECK(hunt_crossley_rebound(1.0, 0.5) == doctest::Approx(0.74843).epsilon(1e-4));
}

TEST_CASE("restitution: the theory is the law's impact with gravity, a few percent under the closed form") {
  // Values of the independent integration summarized in docs/scenarios.md §4.4.
  const double expected[4] = {0.41970, 0.71937, 1.14401, 1.59327};
  for (int k = 0; k < 4; ++k) {
    const double v = winkler_sphere_rebound(kSpeeds[k], 1.0, 0.1, 1e7, 0.5);
    CHECK(v == doctest::Approx(expected[k]).epsilon(1e-4));
    CHECK(v < hunt_crossley_rebound(kSpeeds[k], 0.5));
    CHECK(v > 0.95 * hunt_crossley_rebound(kSpeeds[k], 0.5));
  }
  // The weight matters as (m/K)^(1/3): a ball a thousand times lighter is ten times closer.
  CHECK(winkler_sphere_rebound(4.0, 1e-3, 0.1, 1e7, 0.5) == doctest::Approx(hunt_crossley_rebound(4.0, 0.5)).epsilon(5e-3));
}

TEST_CASE("restitution: error is within 1e-3 on balls that leave at the theory's speed; energy never rises") {
  const Scenario s = make_scenario("restitution", kHeldOutOptions);
  double theory[4];
  for (int k = 0; k < 4; ++k) theory[k] = winkler_sphere_rebound(kSpeeds[k], 1.0, 0.1, 1e7, 0.5);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(bounce_trajectory(s, dt, theory));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(scalar_of(r, "err_energy_gain") < 1e-6);
    CHECK(scalar_of(r, "rebound_2_hunt_crossley") == doctest::Approx(0.74843).epsilon(1e-4));
  }
}

TEST_CASE("restitution: the error counts each rebound and any energy that appears in the bounce") {
  const Scenario s = make_scenario("restitution", kHeldOutOptions);
  double rebound[4];
  for (int k = 0; k < 4; ++k) rebound[k] = winkler_sphere_rebound(kSpeeds[k], 1.0, 0.1, 1e7, 0.5);
  rebound[2] *= 1.3;
  const MetricReport r = s.metrics(bounce_trajectory(s, 1e-3, rebound));
  CHECK(scalar_of(r, "err_rebound_3") == doctest::Approx(0.3).epsilon(1e-6));
  CHECK(scalar_of(r, "err_rebound_1") < 1e-9);
  CHECK(r.error == doctest::Approx(0.3).epsilon(1e-6));
  // The first ball leaves 20 % faster than it came: 44 % more energy than its impact had.
  rebound[2] /= 1.3;
  rebound[0] = 1.2 * kSpeeds[0];
  const MetricReport gain = s.metrics(bounce_trajectory(s, 1e-3, rebound));
  CHECK(scalar_of(gain, "energy_gain_1") == doctest::Approx(0.44).epsilon(0.05));
  CHECK(scalar_of(gain, "err_energy_gain") == doctest::Approx(scalar_of(gain, "energy_gain_1")));
  // A ball that never touches the plane has no rebound.
  Trajectory air(4);
  for (const double t : sample_times(1e-2, 0.1)) {
    std::vector<Pose> X(4);
    for (int k = 0; k < 4; ++k) X[k].p.z() = 1.0;
    air.push(t, X, std::vector<Twist>(4));
  }
  CHECK(std::isnan(s.metrics(air).error));
}
