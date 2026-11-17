#include <cmath>
#include <numbers>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

namespace volcontact::scenarios {
// Defined in scenarios/src/contensou.cpp.
struct CoulombDisk {
  double force, torque;
};
struct ContensouTheory {
  std::vector<double> t, v, wr;
};
CoulombDisk coulomb_disk(double v, double w);
ContensouTheory integrate_contensou(double v0, double w0, double r, double mu);
}  // namespace volcontact::scenarios

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const double kR = 0.1, kMu = 0.3;

Scenario make(Resolution res) { return contensou({.data_dir = VOLCONTACT_DATA_DIR, .resolution = res}); }

}  // namespace

TEST_CASE("contensou: builds at both resolutions, with the inertia of the true cylinder") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    CHECK(s.name == "contensou");
    REQUIRE(s.bodies.size() == 1);
    CHECK(s.bodies[0].inertia(2, 2) == doctest::Approx(0.5 * kR * kR));
    CHECK(s.bodies[0].inertia(0, 0) == doctest::Approx((3 * kR * kR + 0.02 * 0.02) / 12));
    CHECK(s.duration == doctest::Approx(1.2));
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      const ContactModel model = make_contact_model(s, law);
      CHECK(model.pairs().size() == 1);
    }
  }
}

TEST_CASE("contensou: the disk starts overlapping the ground by 1 µm, in both laws") {
  for (const Resolution res : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make(res);
    for (const ForceModel law : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      const ContactModel model = make_contact_model(s, law);
      const std::vector<Pose> poses = {s.bodies[0].X_W0, s.grounds[0].X_WG};
      const std::vector<Twist> twists = {s.bodies[0].V_W0, Twist{}};
      ContactModel::Evaluation eval;
      model.evaluate(poses, twists, model.initial_friction(), &eval);
      REQUIRE(eval.pairs.size() == 1);
      CHECK(eval.pairs[0].in_contact);
      const double expected = std::numbers::pi * kR * kR * kInitialPenetration;
      CHECK(eval.pairs[0].geometry.volume == doctest::Approx(expected).epsilon(0.01));
    }
  }
}

TEST_CASE("contensou theory: pure sliding decelerates at mu g, pure spinning at 4 mu g / (3 r)") {
  const CoulombDisk slide = coulomb_disk(1.0, 0.0);
  CHECK(slide.force == doctest::Approx(1.0));
  CHECK(slide.torque == doctest::Approx(0.0).epsilon(1e-9));
  const CoulombDisk spin = coulomb_disk(0.0, 1.0);
  CHECK(spin.force == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(spin.torque == doctest::Approx(2.0 / 3.0).epsilon(1e-6));
  // Nearly pure sliding, either side of the disk edge, tends to the same limit.
  CHECK(coulomb_disk(1.0, 1e-4).force == doctest::Approx(1.0).epsilon(1e-6));
  CHECK(coulomb_disk(1.0, 1.0).force == doctest::Approx(coulomb_disk(1.0, 1.0 + 1e-9).force).epsilon(1e-4));

  const ContensouTheory slide_run = integrate_contensou(1.0, 0.0, kR, kMu);
  CHECK(slide_run.t.back() == doctest::Approx(1.0 / (kMu * kG)).epsilon(1e-3));
  const ContensouTheory spin_run = integrate_contensou(0.0, 10.0, kR, kMu);
  CHECK(spin_run.t.back() == doctest::Approx(10.0 * 3.0 * kR / (4.0 * kMu * kG)).epsilon(1e-3));
}

TEST_CASE("contensou theory: v and w r stop together, with v/(w r) near 0.653") {
  const ContensouTheory th = integrate_contensou(1.0, 10.0, kR, kMu);
  // At the end both are tiny; the ratio of their remaining values is the asymptote.
  const std::size_t i = th.t.size() - 40;
  CHECK(th.v[i] / th.wr[i] == doctest::Approx(0.653).epsilon(5e-3));
}

TEST_CASE("contensou: a trajectory that follows the theory has error below 1e-3") {
  const Scenario s = make(Resolution::kBase);
  const ContensouTheory th = integrate_contensou(1.0, 10.0, kR, kMu);
  Trajectory traj(1);
  for (int i = 0; i <= 1200; ++i) {
    const double t = 1e-3 * i;
    const std::size_t j = std::upper_bound(th.t.begin(), th.t.end(), t) - th.t.begin();
    double v = 0.0, wr = 0.0;
    if (j < th.t.size()) {
      const double f = (t - th.t[j - 1]) / (th.t[j] - th.t[j - 1]);
      v = th.v[j - 1] + f * (th.v[j] - th.v[j - 1]);
      wr = th.wr[j - 1] + f * (th.wr[j] - th.wr[j - 1]);
    }
    const Pose X;
    const Twist V{Vec3(0, 0, wr / kR), Vec3(v, 0, 0)};
    traj.push(t, std::span(&X, 1), std::span(&V, 1));
  }
  const MetricReport rep = s.metrics(traj);
  CHECK(rep.error <= 1e-3);
  CHECK(s.theory_class == TheoryClass::kB);
  // Class B: the stop times of uniform wear and of the rigid punch bound the nominal one.
  double wear = 0.0, punch = 0.0, nominal = 0.0, band = 1.0;
  for (const auto& [name, value] : rep.scalars) {
    if (name == "stop_time_uniform_wear") wear = value;
    if (name == "stop_time_rigid_punch") punch = value;
    if (name == "stop_time_theory") nominal = value;
    if (name == "band_error") band = value;
  }
  CHECK(wear > nominal);
  CHECK(punch < nominal);
  CHECK(wear < 1.5 * nominal);
  CHECK(punch > 0.7 * nominal);
  CHECK(band == 0.0);
  CHECK(rep.series.size() == 4);
  double ratio = 0.0, ratio_th = 1.0;
  for (const auto& [name, value] : rep.scalars) {
    if (name == "ratio") ratio = value;
    if (name == "ratio_theory") ratio_th = value;
  }
  CHECK(ratio == doctest::Approx(ratio_th).epsilon(1e-3));
}
