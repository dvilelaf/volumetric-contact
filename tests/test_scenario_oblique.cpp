#include <doctest/doctest.h>

#include "test_scenario_heldout.hpp"

namespace volcontact::scenarios {
double oblique_velocity_loss(double p, double tangential, double mu);  // oblique.cpp; not in the public header.
}

using namespace volcontact;
using namespace volcontact::scenarios;
using namespace volcontact::test;

namespace {

constexpr double kTangential[2] = {3.0, 0.35};

// Each ball flies for 5 ms, touches the plane for 10 ms and leaves at `rebound` with the
// tangential velocity loss `loss[k]` and the rolling speed `spin[k]`.
Trajectory impact_trajectory(const Scenario& s, double rebound, const double loss[2], const double spin[2]) {
  Trajectory traj(2);
  for (const double t : sample_times(1e-3, s.duration)) {
    std::vector<Pose> X(2);
    std::vector<Twist> V(2);
    for (int k = 0; k < 2; ++k) {
      X[k] = s.bodies[k].X_W0;
      V[k] = s.bodies[k].V_W0;
      if (t >= 0.0155) {  // After the impact.
        const double u = t - 0.0155;
        X[k].p.z() = 0.1 + 1e-6 + rebound * u - 0.5 * 9.81 * u * u;
        V[k].v = Vec3(kTangential[k] - loss[k], 0.0, rebound - 9.81 * u);
        V[k].w = Vec3(0.0, spin[k] / 0.1, 0.0);
      } else if (t >= 0.0045) {  // In contact.
        X[k].p.z() = 0.1 - 1e-3;
        V[k].v.z() = -0.5;
      } else {
        X[k].p.z() -= t;
        V[k].v.z() = -1.0 - 9.81 * t;
      }
    }
    traj.push(t, X, V);
  }
  return traj;
}

}  // namespace

TEST_CASE("oblique: two 1 kg spheres thrown at the plane without spin, at 3 and 0.35 m/s along it; class A") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("oblique", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = res});
    CHECK(s.theory_class == TheoryClass::kA);
    REQUIRE(s.bodies.size() == 2);
    for (int k = 0; k < 2; ++k) {
      const BodySpec& b = s.bodies[k];
      CHECK(b.V_W0.v.isApprox(Vec3(kTangential[k], 0.0, -1.0)));
      CHECK(b.V_W0.w.isZero());
      CHECK(b.X_W0.p.z() - 0.1 == doctest::Approx(0.005));
      CHECK(s.contacts[k].friction == doctest::Approx(0.2));
      CHECK(s.contacts[k].distributed.damping == doctest::Approx(0.5));
    }
    // For any restitution in [0, 1] the first slides throughout and the second ends sticking.
    CHECK(kTangential[0] / 1.0 > 7 * 0.2 * 1.1);
    CHECK(kTangential[1] / 1.1 < 3.5 * 0.2);
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, law).pairs().size() == 2);
      CHECK(initial_overlap(s, law) == 0.0);
    }
  }
}

TEST_CASE("oblique: the tangential loss is mu times the normal impulse, capped where the contact point stops") {
  CHECK(oblique_velocity_loss(1.6, 3.0, 0.2) == doctest::Approx(0.32));
  CHECK(oblique_velocity_loss(1.6, 0.35, 0.2) == doctest::Approx(0.1));
  // At the cap the ball rolls: v = (5/7) v_t and ω·r = (5/2)·loss = v.
  CHECK(0.35 - 0.1 == doctest::Approx(2.5 * 0.1));
  CHECK(oblique_velocity_loss(0.0, 3.0, 0.2) == 0.0);
}

TEST_CASE("oblique: error is within 1e-3 on the rigid impact, with the normal impulse taken from the run") {
  const Scenario s = make_scenario("oblique", kHeldOutOptions);
  for (const double rebound : {0.2, 0.7}) {
    // P/m = Δv_z + g·Δt between the first sample and the first one after the impact (16 ms).
    const double impulse = rebound + 1.0 + 9.81 * 0.016 - 9.81 * 0.0005;
    const double loss[2] = {0.2 * impulse, 0.1}, spin[2] = {2.5 * 0.2 * impulse, 0.25};
    const MetricReport r = s.metrics(impact_trajectory(s, rebound, loss, spin));
    CHECK_MESSAGE(r.error <= 1e-3, "rebound = " << rebound << ", error = " << r.error);
    CHECK(scalar_of(r, "slide_normal_impulse") == doctest::Approx(impulse).epsilon(1e-9));
    CHECK(scalar_of(r, "stick_velocity_theory") == doctest::Approx(0.25));
    CHECK(scalar_of(r, "slide_rebound") == doctest::Approx(rebound - 9.81 * 0.0005));
  }
}

TEST_CASE("oblique: the error counts the velocity and the spin of each ball; NaN without an impact") {
  const Scenario s = make_scenario("oblique", kHeldOutOptions);
  const double impulse = 0.7 + 1.0 + 9.81 * 0.016 - 9.81 * 0.0005;
  // The sliding ball gets half the friction impulse; the sticking one keeps slipping.
  const double loss[2] = {0.1 * impulse, 0.05}, spin[2] = {2.5 * 0.1 * impulse, 0.125};
  const MetricReport r = s.metrics(impact_trajectory(s, 0.7, loss, spin));
  CHECK(scalar_of(r, "err_slide_spin") == doctest::Approx(0.5).epsilon(1e-6));
  CHECK(scalar_of(r, "err_slide_velocity") == doctest::Approx(0.1 * impulse / (3.0 - 0.2 * impulse)).epsilon(1e-6));
  CHECK(scalar_of(r, "err_stick_velocity") == doctest::Approx(0.05 / 0.25).epsilon(1e-6));
  CHECK(scalar_of(r, "err_stick_spin") == doctest::Approx(0.5).epsilon(1e-6));
  CHECK(r.error == doctest::Approx(0.5).epsilon(1e-6));
  Trajectory air(2);
  for (const double t : sample_times(1e-2, 0.1)) {
    std::vector<Pose> X(2);
    X[0].p.z() = X[1].p.z() = 1.0;
    air.push(t, X, std::vector<Twist>(2));
  }
  CHECK(std::isnan(s.metrics(air).error));
}
