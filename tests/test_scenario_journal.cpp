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

const double kR = 0.05, kHole = 0.06, kMu = 0.2, kM = 1.0, kW0 = 40.0;
const double kInertia = kM * kR * kR / 2;

// Line contact: τ = μ·m·g·r/√(1 + μ²).
double theory_alpha() { return kMu * kM * kG * kR / std::sqrt(1 + kMu * kMu) / kInertia; }

Scenario make(Resolution res) { return journal({.data_dir = VOLCONTACT_DATA_DIR, .resolution = res}); }

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// Spin about y decaying linearly at `alpha`, the centre on the wall at `angle` from the bottom.
Trajectory decay(double alpha, double angle) {
  Trajectory traj(1);
  for (int i = 0; i <= 800; ++i) {
    const double t = 1e-3 * i;
    Pose X;
    X.p = (kHole - kR) * Vec3(std::sin(angle), 0.0, -std::cos(angle));
    const Twist V{Vec3(0, std::max(0.0, kW0 - alpha * t), 0), Vec3::Zero()};
    traj.push(t, std::span(&X, 1), std::span(&V, 1));
  }
  return traj;
}

}  // namespace

TEST_CASE("journal: the shaft of bearing in a hole with 10 mm of clearance, class A") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    CHECK(s.name == "journal");
    CHECK(s.theory_class == TheoryClass::kA);
    REQUIRE(s.bodies.size() == 1);
    CHECK(s.duration == doctest::Approx(0.8));
    CHECK(s.bodies[0].inertia(2, 2) == doctest::Approx(kInertia));
    CHECK((s.bodies[0].X_W0.R * Vec3::UnitZ()).y() == doctest::Approx(1.0));
    CHECK(s.bodies[0].V_W0.w.y() == doctest::Approx(kW0));
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(!make_contact_model(s, law).pairs().empty());
    }
  }
}

TEST_CASE("journal: the shaft starts on the wall at atan(mu) towards +x, 1 um into the true wall") {
  const Scenario s = make(Resolution::kBase);
  const Vec3 p = s.bodies[0].X_W0.p;
  CHECK(p.y() == 0.0);
  CHECK(std::atan2(p.x(), -p.z()) == doctest::Approx(std::atan(kMu)));
  CHECK(std::hypot(p.x(), p.z()) + kR - kHole == doctest::Approx(1e-6).epsilon(1e-6));
  // The polygons do not overlap by more than that layer over a few facets.
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
    CHECK(volume <= 0.2 * 0.02 * 1e-6);
  }
}

TEST_CASE("journal: a spin that follows the line-contact theory at the climb angle has error below 1e-3") {
  const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha(), std::atan(kMu)));
  CHECK(scalar(rep, "full_stop_theory") == doctest::Approx(kW0 / theory_alpha()));
  CHECK(rep.error <= 1e-3);
  CHECK(scalar(rep, "angle") == doctest::Approx(std::atan(kMu)).epsilon(1e-9));
  // The finite arc of a Winkler contact shortens the stop by less than 1 %.
  CHECK(scalar(rep, "stop_time_winkler_arc") < scalar(rep, "stop_time_theory"));
  CHECK(scalar(rep, "stop_time_winkler_arc") > 0.99 * scalar(rep, "stop_time_theory"));
}

TEST_CASE("journal: the error counts the stop time and the angle at which the shaft rides") {
  const Scenario s = make(Resolution::kBase);
  // The zero-clearance Winkler torque, 4/π times the line value, stops 21 % early.
  const MetricReport fast = s.metrics(decay(theory_alpha() * 4.0 / std::numbers::pi, std::atan(kMu)));
  CHECK(fast.error == doctest::Approx(1.0 - std::numbers::pi / 4.0).epsilon(1e-3));
  CHECK(scalar(fast, "err_stop_time") == doctest::Approx(fast.error));
  const MetricReport low = s.metrics(decay(theory_alpha(), 0.5 * std::atan(kMu)));
  CHECK(scalar(low, "err_angle") == doctest::Approx(0.5).epsilon(1e-6));
  CHECK(low.error == doctest::Approx(0.5).epsilon(1e-6));
}
