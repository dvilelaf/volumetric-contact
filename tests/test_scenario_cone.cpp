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

const double kSmall = 0.05, kBig = 0.1, kH = 0.1, kMu = 0.3, kM = 1.0, kW0 = 60.0;
const double kSinA = (kBig - kSmall) / std::hypot(kBig - kSmall, kH);

// I_z and α = τ/I_z from the formulas of docs/scenarios.md §4.
double inertia_z() {
  return 0.3 * kM * (std::pow(kBig, 5) - std::pow(kSmall, 5)) / (std::pow(kBig, 3) - std::pow(kSmall, 3));
}
double theory_alpha() {
  const double tau = 2.0 / 3.0 * kMu * kM * kG * (std::pow(kBig, 3) - std::pow(kSmall, 3)) /
                     ((kBig * kBig - kSmall * kSmall) * kSinA);
  return tau / inertia_z();
}
double theory_sink() { return kM * kG / (1e6 * std::numbers::pi * (kBig * kBig - kSmall * kSmall) * kSinA); }

Scenario make(Resolution res) { return cone({.data_dir = VOLCONTACT_DATA_DIR, .resolution = res}); }

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// Spin decaying linearly at `alpha` from w0, sunk `sink` below the seat, sampled every dt for 0.8 s.
Trajectory decay(double alpha, double sink, double dt) {
  Trajectory traj(1);
  for (int i = 0; i <= std::lround(0.8 / dt); ++i) {
    const double t = dt * i;
    Pose X;
    X.p = Vec3(0, 0, -sink);
    const Twist V{Vec3(0, 0, std::max(0.0, kW0 - alpha * t)), Vec3::Zero()};
    traj.push(t, std::span(&X, 1), std::span(&V, 1));
  }
  return traj;
}

}  // namespace

TEST_CASE("cone: builds at both resolutions, with the inertia of the true frustum") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    CHECK(s.name == "cone");
    REQUIRE(s.bodies.size() == 1);
    CHECK(s.duration == doctest::Approx(0.8));
    CHECK(s.bodies[0].inertia(2, 2) == doctest::Approx(inertia_z()));
    // Transverse inertia about the centroid, 2.403e-3 kg m² (numerical integration).
    CHECK(s.bodies[0].inertia(0, 0) == doctest::Approx(s.bodies[0].inertia(1, 1)));
    CHECK(s.bodies[0].inertia(0, 0) == doctest::Approx(2.403e-3).epsilon(1e-3));
    CHECK(s.bodies[0].V_W0.w.z() == doctest::Approx(kW0));
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      const ContactModel model = make_contact_model(s, law);
      CHECK(model.pairs().size() >= 1);
    }
  }
}

TEST_CASE("cone: the frustum starts overlapping the seat by a 1 µm layer, in both laws") {
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
      const double expected = std::numbers::pi * (kBig * kBig - kSmall * kSmall) / kSinA * kInitialPenetration;
      CHECK_MESSAGE(volume == doctest::Approx(expected).epsilon(0.05), "volume " << volume << " vs " << expected);
    }
  }
}

TEST_CASE("cone theory: uniform pressure stops the spin at I w0 / tau, sinking mg/(K pi (R² - r²) sin a)") {
  // τ = 0.5118 N m, I_z = 3.321e-3 kg m², so t = 0.3893 s.
  CHECK(kW0 / theory_alpha() == doctest::Approx(0.3893).epsilon(1e-3));
  const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha(), theory_sink(), 1e-3));
  CHECK(scalar(rep, "stop_time_theory") == doctest::Approx(0.99 * kW0 / theory_alpha()));
  CHECK(scalar(rep, "sinking_theory") == doctest::Approx(theory_sink()));
}

TEST_CASE("cone: a spin and a sinking that follow the theory have error below 1e-3") {
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha(), theory_sink(), dt));
    CHECK_MESSAGE(rep.error <= 1e-3, "dt = " << dt << ", error = " << rep.error);
    CHECK(scalar(rep, "sinking") == doctest::Approx(theory_sink()));
    CHECK(rep.series.size() == 2);
  }
}

TEST_CASE("cone: a spin that stops 10 % late has error 0.1") {
  const MetricReport rep = make(Resolution::kBase).metrics(decay(theory_alpha() / 1.1, 0.0, 1e-3));
  CHECK(rep.error == doctest::Approx(0.1).epsilon(1e-3));
}
