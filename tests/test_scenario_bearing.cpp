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

const double kR = 0.05, kMu = 0.2, kM = 1.0, kW0 = 40.0;
const double kInertia = kM * kR * kR / 2;

// τ = (4/π)μmgr/√(1 + μ²), from the formulas of docs/scenarios.md §4.
double theory_alpha() { return 4.0 / std::numbers::pi * kMu * kM * kG * kR / std::sqrt(1 + kMu * kMu) / kInertia; }

Scenario make(Resolution res) { return bearing({.data_dir = VOLCONTACT_DATA_DIR, .resolution = res}); }

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// Spin about y decaying linearly at `alpha`, the centre sunk at the theory's angle and depth.
Trajectory decay(double alpha, double dt) {
  const double e = kM * kG / (1e6 * kR * 0.2 * std::numbers::pi / 2), th = std::atan(kMu);
  Trajectory traj(1);
  for (int i = 0; i <= std::lround(0.8 / dt); ++i) {
    const double t = dt * i;
    Pose X;
    X.p = Vec3(e * std::sin(th), 0.0, -e * std::cos(th));
    const Twist V{Vec3(0, std::max(0.0, kW0 - alpha * t), 0), Vec3::Zero()};
    traj.push(t, std::span(&X, 1), std::span(&V, 1));
  }
  return traj;
}

}  // namespace

TEST_CASE("bearing: builds at both resolutions, with the inertia of the true cylinder") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    CHECK(s.name == "bearing");
    REQUIRE(s.bodies.size() == 1);
    CHECK(s.duration == doctest::Approx(0.8));
    CHECK(s.bodies[0].inertia(2, 2) == doctest::Approx(kInertia));
    // The body axis z is the world y axis, and the spin is about it.
    CHECK((s.bodies[0].X_W0.R * Vec3::UnitZ()).y() == doctest::Approx(1.0));
    CHECK(s.bodies[0].V_W0.w.y() == doctest::Approx(kW0));
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      const ContactModel model = make_contact_model(s, law);
      CHECK(!model.pairs().empty());
    }
  }
}

TEST_CASE("bearing: the shaft starts concentric, without overlap, in both laws") {
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
      CHECK(volume == doctest::Approx(0.0).scale(1e-9));
    }
  }
}

TEST_CASE("bearing: a spin that follows the theory has error below 1e-3") {
  const double t_full = kW0 / theory_alpha();
  const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha(), 1e-3));
  CHECK(scalar(rep, "full_stop_theory") == doctest::Approx(t_full));
  CHECK(rep.error <= 1e-3);
  CHECK(scalar(rep, "angle") == doctest::Approx(std::atan(kMu)).epsilon(1e-6));
  CHECK(scalar(rep, "sink") == doctest::Approx(scalar(rep, "sink_theory")).epsilon(1e-6));
  CHECK(rep.series.size() == 4);
}

TEST_CASE("bearing: a spin that stops 10 % late has error 0.1") {
  const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha() / 1.1, 1e-3));
  CHECK(rep.error == doctest::Approx(0.1).epsilon(1e-6));
}

TEST_CASE("bearing: class B; the band runs from the uniform pressure to the line contact") {
  const Scenario s = make(Resolution::kBase);
  CHECK(s.theory_class == TheoryClass::kB);
  const MetricReport rep = s.metrics(decay(theory_alpha(), 1e-3));
  const double nominal = scalar(rep, "stop_time_theory");
  CHECK(scalar(rep, "stop_time_line_contact") == doctest::Approx(nominal * 4.0 / std::numbers::pi));
  CHECK(scalar(rep, "stop_time_uniform") == doctest::Approx(nominal * 8.0 / (std::numbers::pi * std::numbers::pi)));
  CHECK(scalar(rep, "stop_time_band_max") == scalar(rep, "stop_time_line_contact"));
  CHECK(scalar(rep, "stop_time_band_min") == scalar(rep, "stop_time_uniform"));
  CHECK(scalar(rep, "band_error") == 0.0);
  // A line contact's torque (factor 1 instead of 4/π) is inside the band, 27 % from the nominal.
  const MetricReport line = s.metrics(decay(theory_alpha() * std::numbers::pi / 4.0, 1e-3));
  CHECK(line.error == doctest::Approx(4.0 / std::numbers::pi - 1.0).epsilon(1e-3));
  CHECK(scalar(line, "band_error") < 1e-6);
}
