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

const double kOuter = 0.25, kInner = 0.15, kH = 0.05, kMu = 0.6, kM = 1.0;
const double kSumSq = kOuter * kOuter + kInner * kInner;
const double kW0 = 5.0 * std::numbers::pi;

// τ = (2/3)μmg(R³ − r³)/(R² − r²) and α = τ/I_z, from the formulas of docs/scenarios.md §4.
double theory_alpha() {
  const double tau = 2.0 / 3.0 * kMu * kM * kG * (std::pow(kOuter, 3) - std::pow(kInner, 3)) /
                     (kOuter * kOuter - kInner * kInner);
  return tau / (kM * kSumSq / 2);
}

Scenario make(Resolution res) { return ring({.data_dir = VOLCONTACT_DATA_DIR, .resolution = res}); }

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// Spin about z decaying linearly at `alpha` from w0, sampled every dt for 1 s.
Trajectory decay(double alpha, double dt) {
  Trajectory traj(1);
  for (int i = 0; i <= std::lround(1.0 / dt); ++i) {
    const double t = dt * i;
    const Pose X;
    const Twist V{Vec3(0, 0, std::max(0.0, kW0 - alpha * t)), Vec3::Zero()};
    traj.push(t, std::span(&X, 1), std::span(&V, 1));
  }
  return traj;
}

}  // namespace

TEST_CASE("ring: builds at both resolutions, with the inertia of the true annulus") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    CHECK(s.name == "ring");
    REQUIRE(s.bodies.size() == 1);
    CHECK(s.duration == doctest::Approx(1.0));
    CHECK(s.bodies[0].mass == doctest::Approx(kM));
    CHECK(s.bodies[0].inertia(2, 2) == doctest::Approx(kM * kSumSq / 2));
    CHECK(s.bodies[0].inertia(0, 0) == doctest::Approx(kM * (3 * kSumSq + kH * kH) / 12));
    const std::size_t pieces = res == Resolution::kBase ? 64 : 128;
    CHECK(s.bodies[0].shape.pieces.size() == pieces);
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      const ContactModel model = make_contact_model(s, law);
      CHECK(model.pairs().size() == pieces);
    }
  }
}

TEST_CASE("ring: the annulus starts overlapping the ground by 1 µm in total, in both laws") {
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
      const double expected = std::numbers::pi * (kOuter * kOuter - kInner * kInner) * kInitialPenetration;
      CHECK(volume == doctest::Approx(expected).epsilon(0.01));
    }
  }
}

TEST_CASE("ring theory: uniform pressure over the annulus stops the spin at I w0 / tau") {
  // τ = 1.2017 N m, I_z = 0.0425 kg m², so t = 0.5556 s; the 1 % stop comes at 0.99 of it.
  const double t_full = kW0 / theory_alpha();
  CHECK(t_full == doctest::Approx(0.5556).epsilon(1e-3));
  const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha(), 1e-3));
  CHECK(scalar(rep, "full_stop_theory") == doctest::Approx(t_full));
  CHECK(scalar(rep, "stop_time_theory") == doctest::Approx(0.99 * t_full));
}

TEST_CASE("ring: a spin that follows the theory has error below 1e-3") {
  const Scenario s = make(Resolution::kBase);
  const MetricReport rep = s.metrics(decay(theory_alpha(), 1e-3));
  CHECK(rep.error <= 1e-3);
  CHECK(scalar(rep, "stop_time") == doctest::Approx(scalar(rep, "stop_time_theory")).epsilon(1e-6));
  CHECK(scalar(rep, "omega_error_max_radps") < 1e-9);
  CHECK(rep.series.size() == 2);
}

TEST_CASE("ring: a spin that stops 10 % late has error 0.1") {
  const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha() / 1.1, 1e-3));
  CHECK(rep.error == doctest::Approx(0.1).epsilon(1e-6));
}
