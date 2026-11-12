#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include <doctest/doctest.h>

#include "test_util.hpp"
#include "volcontact/catalog.hpp"
#include "volcontact/engine.hpp"

using namespace volcontact;

namespace {

const std::filesystem::path kData = VOLCONTACT_DATA_DIR;
constexpr ContactMethod kMethods[] = {ContactMethod::kGonthier, ContactMethod::kDistributed, ContactMethod::kNative};

std::unique_ptr<Engine> engine_named(std::string_view name) {
  for (auto& e : available_engines()) {
    if (e->name() == name) return std::move(e);
  }
  return nullptr;
}

// A cube of side `side` [m] and density 1000 kg/m³ at `p`.
BodySpec cube(const std::string& asset, double side, const Vec3& p) {
  BodySpec b;
  b.shape = shape_meshes(kData, asset);
  b.mass = 1000.0 * side * side * side;
  b.inertia = b.mass * side * side / 6.0 * Mat3::Identity();
  b.X_W0.p = p;
  return b;
}

// A 0.2 m cube high above a planar ground it never reaches (the contact model
// needs a pair).
Scenario flying_cube() {
  Scenario s;
  s.duration = 1.0;
  s.grounds.push_back(planar_ground(kData));
  s.bodies.push_back(cube("box_200x200x200", 0.2, Vec3(0, 0, 100)));
  s.contacts.push_back({BodyRef::body(0), BodyRef::ground(0), 0.5, {}, {}});
  return s;
}

Scenario incline_with_two_blocks(double theta, double mu_a, double mu_b) {
  Scenario s;
  s.duration = 1.5;
  s.gravity = 9.81 * Vec3(std::sin(theta), 0.0, -std::cos(theta));
  s.grounds.push_back(planar_ground(kData));
  for (const double y : {-0.5, 0.5}) s.bodies.push_back(cube("box_200x200x200", 0.2, Vec3(0.0, y, 0.1 - 1e-6)));
  for (int i = 0; i < 2; ++i) s.contacts.push_back({BodyRef::body(i), BodyRef::ground(0), i == 0 ? mu_a : mu_b, {}, {}});
  return s;
}

// Mean acceleration along x between two instants.
double acceleration(const Trajectory& tr, int body, double t0, double t1) {
  const auto at = [&](double t) { return tr.twist(static_cast<std::size_t>(std::llround(t / 1e-3)), body).v.x(); };
  return (at(t1) - at(t0)) / (t1 - t0);
}

double scalar(const MetricReport& r, const std::string& name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

const Pose& last_pose(const RunResult& r, int body = 0) { return r.trajectory.pose(r.trajectory.size() - 1, body); }
const Twist& last_twist(const RunResult& r, int body = 0) { return r.trajectory.twist(r.trajectory.size() - 1, body); }

}  // namespace

TEST_CASE("mujoco: free fall follows g·t²/2 along a tilted gravity, in the three methods") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  Scenario s = flying_cube();
  s.gravity = Vec3(1.0, -2.0, -9.0);
  s.bodies[0].X_W0.R = test::rot(0.7, Vec3(1, 2, 3));
  for (const ContactMethod m : kMethods) {
    const RunResult r = engine->run(s, {.contact = m});
    CAPTURE(to_string(m));
    REQUIRE(r.trajectory.size() == 1001);
    CHECK(r.trajectory.times().back() == doctest::Approx(1.0));
    const Vec3 fall = last_pose(r).p - s.bodies[0].X_W0.p;
    // Semi-implicit Euler overshoots g·t²/2 by dt/t.
    CHECK((fall - 0.5 * s.gravity).norm() < 2e-3 * 0.5 * s.gravity.norm());
    CHECK((last_twist(r).v - s.gravity).norm() < 1e-9);
    CHECK((last_pose(r).R - s.bodies[0].X_W0.R).norm() < 1e-9);
  }
}

// Catches a mistake in the frames of the free joint (angular velocity in the
// body frame) or of the inertia tensor.
TEST_CASE("mujoco: a spinning body with products of inertia keeps its angular momentum") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  Scenario s = flying_cube();
  s.gravity = Vec3::Zero();
  BodySpec& b = s.bodies[0];
  b.mass = 2.0;
  const Mat3 P = test::rot(0.6, Vec3(1, -1, 2));
  b.inertia = P * Vec3(0.02, 0.05, 0.06).asDiagonal() * P.transpose();  // Principal axes off the body axes.
  REQUIRE(std::abs(b.inertia(0, 1)) > 1e-3);
  b.X_W0.R = test::rot(1.1, Vec3(3, 1, -2));
  b.V_W0.w = Vec3(3.0, -2.0, 4.0);
  b.V_W0.v = Vec3(0.3, 0.1, -0.2);

  const RunResult r = engine->run(s, {});
  const auto momentum = [&](std::size_t i) {
    const Mat3& R = r.trajectory.pose(i, 0).R;
    return Vec3(R * b.inertia * R.transpose() * r.trajectory.twist(i, 0).w);
  };
  const Vec3 L0 = b.X_W0.R * b.inertia * b.X_W0.R.transpose() * b.V_W0.w;
  CHECK((r.trajectory.twist(0, 0).w - b.V_W0.w).norm() < 1e-12);
  CHECK((momentum(0) - L0).norm() < 1e-12);
  double drift = 0.0, wobble = 0.0;
  for (std::size_t i = 0; i < r.trajectory.size(); ++i) {
    drift = std::max(drift, (momentum(i) - L0).norm() / L0.norm());
    wobble = std::max(wobble, (r.trajectory.twist(i, 0).w - b.V_W0.w).norm());
  }
  std::printf("mujoco free spin: angular momentum drift %.3g, change of angular velocity %.3g rad/s\n", drift, wobble);
  CHECK(drift < 5e-3);
  CHECK(wobble > 1.0);  // The body does precess: the test is not trivial.
  CHECK((last_pose(r).p - (b.X_W0.p + b.V_W0.v)).norm() < 1e-9);
}

// The same law gives the same motion on both engines, to the accuracy of their
// integrators.
TEST_CASE("mujoco: gonthier and distributed block and disk agree with chrono") {
  const auto mujoco = engine_named("mujoco"), chrono = engine_named("chrono");
  if (!mujoco || !chrono) return;
  for (const ContactMethod m : {ContactMethod::kGonthier, ContactMethod::kDistributed}) {
    CAPTURE(to_string(m));
    {
      const Scenario s = make_scenario("block", {.data_dir = kData});
      const RunResult a = mujoco->run(s, {.contact = m}), b = chrono->run(s, {.contact = m});
      const double va = scalar(s.metrics(a.trajectory), "velocity_end"), vb = scalar(s.metrics(b.trajectory), "velocity_end");
      std::printf("block %s: velocity_end mujoco %.6f, chrono %.6f; final x %.6f, %.6f\n", std::string(to_string(m)).c_str(),
                  va, vb, last_pose(a).p.x(), last_pose(b).p.x());
      CHECK(va == doctest::Approx(vb).epsilon(1e-4));
      CHECK(last_pose(a).p.x() == doctest::Approx(last_pose(b).p.x()).epsilon(1e-4));
      CHECK(last_pose(a).p.z() == doctest::Approx(last_pose(b).p.z()).epsilon(1e-6));
    }
    {
      const Scenario s = make_scenario("disk", {.data_dir = kData});
      const RunResult a = mujoco->run(s, {.contact = m}), b = chrono->run(s, {.contact = m});
      const double ta = scalar(s.metrics(a.trajectory), "stop_time"), tb = scalar(s.metrics(b.trajectory), "stop_time");
      std::printf("disk %s: stop_time mujoco %.6f, chrono %.6f\n", std::string(to_string(m)).c_str(), ta, tb);
      // Gonthier's disk drifts sideways as it stops (its friction point moves), which
      // amplifies the difference between the integrators.
      CHECK(ta == doctest::Approx(tb).epsilon(m == ContactMethod::kGonthier ? 1e-2 : 1e-4));
    }
  }
}

TEST_CASE("mujoco native: each contact has its own mu, and a block slides at g(sin θ − μ cos θ)") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  const double theta = std::numbers::pi / 6.0;
  const Scenario s = incline_with_two_blocks(theta, 0.2, 0.6);
  const RunResult r = engine->run(s, {.contact = ContactMethod::kNative});
  const double expected = 9.81 * (std::sin(theta) - 0.2 * std::cos(theta));
  std::printf("mujoco native incline: a = %.4f (theory %.4f), held block %.2g m/s²\n",
              acceleration(r.trajectory, 0, 0.5, 1.5), expected, acceleration(r.trajectory, 1, 0.5, 1.5));
  CHECK(acceleration(r.trajectory, 0, 0.5, 1.5) == doctest::Approx(expected).epsilon(0.03));
  CHECK(std::abs(acceleration(r.trajectory, 1, 0.5, 1.5)) < 0.01 * 9.81 * std::sin(theta));
  // The held block slips only while it settles 2 mm into the foundation at the start.
  CHECK(std::abs(last_pose(r, 1).p.x()) < 2e-3);
}

// The native mapping: at rest a flat contact carries K times the reference area
// of the pair (a sixth of the cube's surface, its face) times the penetration.
TEST_CASE("mujoco native: a resting cube stays at rest on four points, at the Winkler penetration of its face") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  Scenario s;
  s.duration = 2.0;
  s.grounds.push_back(planar_ground(kData));
  s.bodies.push_back(cube("box_200x200x200", 0.2, Vec3(0, 0, 0.1 - 1e-6)));
  s.contacts.push_back({BodyRef::body(0), BodyRef::ground(0), 0.5, {}, {}});
  const RunResult r = engine->run(s, {.contact = ContactMethod::kNative});
  const double sag = 0.1 - last_pose(r).p.z();
  const double expected = s.bodies[0].mass * 9.81 / (s.native.stiffness * 0.04);
  std::printf("mujoco native rest: penetration %.4g m (mapping %.4g m), speed %.2g m/s\n", sag, expected,
              last_twist(r).v.norm());
  CHECK(sag == doctest::Approx(expected).epsilon(0.02));
  CHECK(r.stats.native_contact_points == doctest::Approx(4.0));
  CHECK(last_twist(r).v.norm() < 1e-6);
  CHECK(last_twist(r).w.norm() < 1e-6);
  CHECK(last_pose(r).p.head<2>().norm() < 1e-6);
}

TEST_CASE("mujoco: an applied force F = (t, 0, 0) gives v = t²/(2m), in the three methods") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  Scenario s = flying_cube();
  s.gravity = Vec3::Zero();
  s.bodies[0].X_W0.p = Vec3(0, 0, 1);
  s.bodies[0].applied_force = [](double t) { return Vec3(t, 0, 0); };
  for (const ContactMethod m : kMethods) {
    const RunResult r = engine->run(s, {.contact = m});
    const double expected = 1.0 / (2.0 * s.bodies[0].mass);
    CHECK_MESSAGE(last_twist(r).v.x() == doctest::Approx(expected).epsilon(2e-3), to_string(m));
  }
}

TEST_CASE("mujoco: push (applied force) and vgroove (two-piece ground) in the three methods") {
  const auto mujoco = engine_named("mujoco"), chrono = engine_named("chrono");
  if (!mujoco) return;
  for (const char* name : {"push", "vgroove"}) {
    const Scenario s = make_scenario(name, {.data_dir = kData});
    for (const ContactMethod m : kMethods) {
      CAPTURE(name);
      CAPTURE(to_string(m));
      const RunResult a = mujoco->run(s, {.contact = m});
      const MetricReport report = s.metrics(a.trajectory);
      // The final velocity is what tells whether the engine runs the scenario; `error` also counts
      // the creep and the early onset of the Gonthier law in `push` (docs/scenarios.md §4).
      const double error = scalar(report, "err_velocity");
      std::printf("mujoco %s %s: error %.4f\n", name, std::string(to_string(m)).c_str(), error);
      // The native box hops as it slides (MuJoCo's cone couples sliding into the
      // normal direction), which the two-plane groove amplifies.
      CHECK(error < (m == ContactMethod::kNative ? 0.05 : 0.03));
      if (m == ContactMethod::kNative || !chrono) continue;
      const RunResult b = chrono->run(s, {.contact = m});
      CHECK(scalar(report, "velocity_end") == doctest::Approx(scalar(s.metrics(b.trajectory), "velocity_end")).epsilon(2e-3));
    }
  }
}

TEST_CASE("mujoco native: a cube rests in corner_s300") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  Scenario s;
  s.duration = 1.0;
  GroundSpec corner;
  corner.shape = shape_meshes(kData, "corner_s300");
  s.grounds.push_back(corner);
  s.bodies.push_back(cube("box_100x100x100", 0.1, Vec3(0.05, 0.05, 0.05 - 1e-6)));
  s.contacts.push_back({BodyRef::body(0), BodyRef::ground(0), 0.5, {}, {}});
  const RunResult r = engine->run(s, {.contact = ContactMethod::kNative});
  const Vec3 drift = last_pose(r).p - Vec3(0.05, 0.05, 0.05);
  std::printf("mujoco corner rest: displacement (%.3g, %.3g, %.3g) m, speed %.3g m/s\n", drift.x(), drift.y(), drift.z(),
              last_twist(r).v.norm());
  CHECK(drift.norm() < 1e-3);
  CHECK(last_twist(r).v.norm() < 1e-4);
}

// Regression: a rolling body collided as the convex hull of its mesh, a prism
// or a faceted ball that rolls on its edges (cylinder 7 %, sphere 4 %, billiard 11 %).
TEST_CASE("mujoco native: a body with an analytic shape collides as that shape") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  const struct {
    const char* name;
    double points;
  } cases[] = {{"cylinder", 2.0}, {"sphere", 1.0}, {"billiard", 1.0}};
  for (const auto& c : cases) {
    CAPTURE(c.name);
    const Scenario s = make_scenario(c.name, {.data_dir = kData});
    REQUIRE(s.bodies[0].primitive);
    const RunResult r = engine->run(s, {.contact = ContactMethod::kNative});
    // `billiard` also charges the onset time (7 % late with MuJoCo's soft contact while the ball
    // settles from 1 µm of penetration); the rolling itself is what this regression is about.
    const MetricReport report = s.metrics(r.trajectory);
    CHECK((std::string_view(c.name) == "billiard" ? scalar(report, "err_onset_velocity") : report.error) < 0.01);
    CHECK(r.stats.native_contact_points == doctest::Approx(c.points));
  }
  // A box keeps its four-point manifold.
  const Scenario s = make_scenario("block", {.data_dir = kData});
  CHECK(s.metrics(engine->run(s, {.contact = ContactMethod::kNative}).trajectory).error < 0.03);
}

// Regression: with MuJoCo's default solver tolerance the sliding block wobbled
// on unequal corner forces at dt = 0.2 ms and ended 10 % fast.
TEST_CASE("mujoco native: the sliding block converges with the time step") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  const Scenario s = make_scenario("block", {.data_dir = kData});
  const RunResult r = engine->run(s, {.dt = 2e-4, .contact = ContactMethod::kNative});
  CHECK(s.metrics(r.trajectory).error < 0.01);
}

// Regression: with a quarter of the surface as reference area the compressions
// of a stack of cubes were two thirds of the foundation's.
TEST_CASE("mujoco native: stacked cubes compress as a Winkler foundation under their faces") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  const Scenario s = make_scenario("stack", {.data_dir = kData});
  CHECK(s.metrics(engine->run(s, {.contact = ContactMethod::kNative}).trajectory).error < 0.02);
}

// `default`: what MuJoCo does with its own contact parameters. K does not
// enter, the block still slides at Coulomb's acceleration.
TEST_CASE("mujoco native default: MuJoCo's own parameters, independent of K") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  CHECK(engine->native_models() == std::vector<std::string>{"soft", "default"});
  Scenario s;
  s.duration = 1.0;
  s.grounds.push_back(planar_ground(kData));
  s.bodies.push_back(cube("box_200x200x200", 0.2, Vec3(0, 0, 0.1 - 1e-6)));
  s.contacts.push_back({BodyRef::body(0), BodyRef::ground(0), 0.5, {}, {}});
  double sag[2][2];
  for (int k = 0; k < 2; ++k) {
    s.native.stiffness = k == 0 ? 1e6 : 1e8;
    for (int model = 0; model < 2; ++model) {
      const RunResult r = engine->run(s, {.contact = ContactMethod::kNative, .native_model = model == 0 ? "soft" : "default"});
      sag[model][k] = 0.1 - last_pose(r).p.z();
      CHECK(last_twist(r).v.norm() < 1e-5);
      CHECK(r.stats.native_contact_points == doctest::Approx(4.0));
    }
  }
  std::printf("mujoco native rest: soft %.3g, %.3g m; default %.3g, %.3g m\n", sag[0][0], sag[0][1], sag[1][0], sag[1][1]);
  CHECK(sag[0][1] == doctest::Approx(sag[0][0] / 100.0).epsilon(0.05));
  CHECK(sag[1][1] == doctest::Approx(sag[1][0]).epsilon(1e-9));
  CHECK(sag[1][0] > 0.0);

  const Scenario block = make_scenario("block", {.data_dir = kData});
  const RunResult r = engine->run(block, {.contact = ContactMethod::kNative, .native_model = "default"});
  CHECK(block.metrics(r.trajectory).error < 0.03);
}

TEST_CASE("mujoco: a run that goes numerically bad throws instead of resetting") {
  const auto engine = engine_named("mujoco");
  if (!engine) return;
  Scenario s = flying_cube();
  s.bodies[0].applied_force = [](double t) {
    return Vec3(t > 0.5 ? std::numeric_limits<double>::quiet_NaN() : 0.0, 0, 0);
  };
  for (const ContactMethod m : kMethods) CHECK_THROWS_AS(static_cast<void>(engine->run(s, {.contact = m})), std::runtime_error);
}
