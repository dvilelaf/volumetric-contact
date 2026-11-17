#include <array>
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

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};
const double kAngle = 5.0 * std::numbers::pi / 180.0;
const double kSide = 0.2;

double scalar(const MetricReport& rep, const std::string& key) {
  for (const auto& [name, value] : rep.scalars) {
    if (name == key) return value;
  }
  return std::nan("");
}

// n m g cos θ / (K a²) for the interfaces ground–c1, c1–c2, c2–c3 (n = 3, 2, 1), at the 5° tilt.
std::array<double, 3> theory_compressions() {
  std::array<double, 3> d{};
  for (int i = 0; i < 3; ++i) d[i] = (3 - i) * 1.0 * 9.81 * std::cos(kAngle) / (1e6 * kSide * kSide);
  return d;
}

// The stack at rest, each interface compressed by its theoretical amount, for 3 s.
Trajectory theory_trajectory(double dt) {
  const auto d = theory_compressions();
  const double z0 = kSide / 2 - d[0];
  const double z1 = z0 + kSide - d[1];
  const double z2 = z1 + kSide - d[2];
  const std::array<Pose, 3> X{Pose{Mat3::Identity(), Vec3(0.0, 0.0, z0)}, Pose{Mat3::Identity(), Vec3(0.0, 0.0, z1)},
                              Pose{Mat3::Identity(), Vec3(0.0, 0.0, z2)}};
  const std::array<Twist, 3> V{};
  Trajectory traj(3);
  for (int i = 0; i <= std::lround(3.0 / dt); ++i) traj.push(dt * i, X, V);
  return traj;
}

}  // namespace

TEST_CASE("stack is three 1 kg cubes of 0.2 m, aligned and at rest, under vertical gravity") {
  const Scenario s = make_scenario("stack", kOptions);
  REQUIRE(s.bodies.size() == 3);
  CHECK(s.name == "stack");
  CHECK(s.gravity.isApprox(Vec3(0.0, 0.0, -9.81)));
  CHECK(s.duration == doctest::Approx(3.0));
  for (const BodySpec& cube : s.bodies) {
    CHECK(cube.mass == doctest::Approx(1.0));
    CHECK(cube.inertia.isApprox(box_inertia(1.0, kSide, kSide, kSide)));
    CHECK(cube.X_W0.R.isApprox(Mat3::Identity()));
    CHECK(cube.X_W0.p.x() == doctest::Approx(0.0));
    CHECK(cube.X_W0.p.y() == doctest::Approx(0.0));
    CHECK(cube.V_W0.v.isZero());
    CHECK(cube.V_W0.w.isZero());
  }
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
  CHECK(s.contacts.size() == 3);
}

TEST_CASE("stack's contact model builds for both laws, with one pair per contact") {
  const Scenario s = make_scenario("stack", kOptions);
  for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    CHECK(make_contact_model(s, model).pairs().size() == 3);
  }
}

TEST_CASE("at t = 0 each interface overlaps by its intended 1 µm layer, a² × 1 µm") {
  const Scenario s = make_scenario("stack", kOptions);
  for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
    const ContactModel contacts = make_contact_model(s, model);
    const std::vector<Pose> poses{s.bodies[0].X_W0, s.bodies[1].X_W0, s.bodies[2].X_W0, s.grounds[0].X_WG};
    const std::vector<Twist> twists(poses.size());
    ContactModel::Evaluation out;
    contacts.evaluate(poses, twists, contacts.initial_friction(), &out);
    REQUIRE(out.pairs.size() == 3);
    for (const auto& pair : out.pairs) {
      REQUIRE(pair.in_contact);
      CHECK(pair.geometry.volume == doctest::Approx(kSide * kSide * 1e-6).epsilon(0.01));
    }
  }
}

TEST_CASE("the theory compressions are n m g cos θ / (K a²), with n = 3, 2, 1 cubes above each interface") {
  const Scenario s = make_scenario("stack", kOptions);
  const MetricReport theory = s.metrics(theory_trajectory(1e-3));
  CHECK(scalar(theory, "compression_1_m_theory") == doctest::Approx(7.3295e-4).epsilon(1e-4));
  CHECK(scalar(theory, "compression_2_m_theory") == doctest::Approx(2.0 / 3.0 * 7.3295e-4).epsilon(1e-4));
  CHECK(scalar(theory, "compression_3_m_theory") == doctest::Approx(1.0 / 3.0 * 7.3295e-4).epsilon(1e-4));
}

TEST_CASE("the applied forces are zero at t = 0 and keep |g_eff| = g while the gravity tilts") {
  const Scenario s = make_scenario("stack", kOptions);
  for (const BodySpec& cube : s.bodies) {
    REQUIRE(static_cast<bool>(cube.applied_force));
    CHECK(cube.applied_force(0.0).isZero());
    for (const double t : {0.25, 0.5, 1.0, 2.0, 3.0}) {
      const Vec3 g_eff = s.gravity + cube.applied_force(t) / cube.mass;
      CHECK(g_eff.norm() == doctest::Approx(9.81));
    }
  }
}

TEST_CASE("the tilt ramps from 0 to 5° over 1 s and is constant after") {
  const Scenario s = make_scenario("stack", kOptions);
  const Vec3 g_end = s.gravity + s.bodies[0].applied_force(3.0) / s.bodies[0].mass;
  CHECK(std::atan2(g_end.x(), -g_end.z()) == doctest::Approx(kAngle));
  const Vec3 g_mid = s.gravity + s.bodies[0].applied_force(0.5) / s.bodies[0].mass;
  CHECK(std::atan2(g_mid.x(), -g_mid.z()) == doctest::Approx(kAngle / 2));
  const Vec3 g_1 = s.gravity + s.bodies[0].applied_force(1.0) / s.bodies[0].mass;
  CHECK(std::atan2(g_1.x(), -g_1.z()) == doctest::Approx(kAngle));
}

TEST_CASE("a stack at rest with the theoretical compressions has no creep and error 0") {
  const Scenario s = make_scenario("stack", kOptions);
  const MetricReport r = s.metrics(theory_trajectory(1e-3));
  CHECK(r.error < 1e-12);
  for (int i = 1; i <= 3; ++i) {
    const std::string idx = std::to_string(i);
    CHECK(scalar(r, "compression_" + idx + "_m") == doctest::Approx(scalar(r, "compression_" + idx + "_m_theory")));
    CHECK(std::abs(scalar(r, "creep_c" + idx + "_m")) < 1e-12);
  }
}

TEST_CASE("stack's error is within 1e-3 on a trajectory that sits at the theoretical compressions") {
  const Scenario s = make_scenario("stack", kOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const MetricReport r = s.metrics(theory_trajectory(dt));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
    CHECK(r.series.size() == 9);
  }
}

TEST_CASE("stack's error counts the creep of any cube over the 0.2 m side and the residual speed; class C") {
  const Scenario s = make_scenario("stack", kOptions);
  CHECK(s.theory_class == TheoryClass::kC);
  // The theoretical compressions, with the top cube creeping 50 mm in the 3 s.
  const Trajectory rest = theory_trajectory(1e-3);
  Trajectory traj(3);
  for (std::size_t i = 0; i < rest.size(); ++i) {
    std::array<Pose, 3> X{rest.pose(i, 0), rest.pose(i, 1), rest.pose(i, 2)};
    std::array<Twist, 3> V{};
    X[2].p.x() = 0.05 * rest.times()[i] / 3.0;
    V[2].v.x() = 0.05 / 3.0;
    traj.push(rest.times()[i], X, V);
  }
  const MetricReport r = s.metrics(traj);
  CHECK(scalar(r, "err_compression") < 1e-9);
  CHECK(scalar(r, "err_creep") == doctest::Approx(0.25));
  // 16.7 mm/s kept for the 3 s of the run, over the side: the same 0.25.
  CHECK(scalar(r, "err_residual_speed") == doctest::Approx(0.25));
  CHECK(r.error == doctest::Approx(0.25));
}

TEST_CASE("stack's compression term is the worst of the three interfaces") {
  const Scenario s = make_scenario("stack", kOptions);
  const auto d = theory_compressions();
  // The top interface compressed 30 % too much, the others right.
  const double z0 = kSide / 2 - d[0], z1 = z0 + kSide - d[1], z2 = z1 + kSide - 1.3 * d[2];
  const std::array<Pose, 3> X{Pose{Mat3::Identity(), Vec3(0.0, 0.0, z0)}, Pose{Mat3::Identity(), Vec3(0.0, 0.0, z1)},
                              Pose{Mat3::Identity(), Vec3(0.0, 0.0, z2)}};
  const std::array<Twist, 3> V{};
  Trajectory traj(3);
  for (int i = 0; i <= 3000; ++i) traj.push(1e-3 * i, X, V);
  CHECK(s.metrics(traj).error == doctest::Approx(0.3).epsilon(1e-6));
}
