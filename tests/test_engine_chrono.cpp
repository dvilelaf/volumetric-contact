#include <cmath>
#include <cstdio>
#include <numbers>

#include <doctest/doctest.h>

#include "volcontact/engine.hpp"

using namespace volcontact;

namespace {

const std::filesystem::path kData = VOLCONTACT_DATA_DIR;

std::unique_ptr<Engine> chrono_engine() {
  for (auto& e : available_engines()) {
    if (e->name() == "chrono") return std::move(e);
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

Scenario incline_with_two_blocks(double theta, double mu_a, double mu_b) {
  Scenario s;
  s.duration = 1.5;
  s.gravity = 9.81 * Vec3(std::sin(theta), 0.0, -std::cos(theta));
  s.grounds.push_back(planar_ground(kData));
  const Mat3 R = Eigen::AngleAxisd(0.0, Vec3::UnitZ()).toRotationMatrix();
  for (const double y : {-0.5, 0.5}) {
    BodySpec b = cube("box_200x200x200", 0.2, Vec3(0.0, y, 0.1 - 1e-6));
    b.X_W0.R = R;
    s.bodies.push_back(b);
  }
  for (int i = 0; i < 2; ++i) s.contacts.push_back({BodyRef::body(i), BodyRef::ground(0), i == 0 ? mu_a : mu_b, {}, {}});
  return s;
}

// Mean acceleration along x between two instants.
double acceleration(const Trajectory& tr, int body, double t0, double t1) {
  const auto at = [&](double t) { return tr.twist(static_cast<std::size_t>(std::llround(t / 1e-3)), body).v.x(); };
  return (at(t1) - at(t0)) / (t1 - t0);
}

}  // namespace

TEST_CASE("chrono native: each contact has its own mu") {
  const auto engine = chrono_engine();
  if (!engine) return;
  const double theta = std::numbers::pi / 6.0;
  const Scenario s = incline_with_two_blocks(theta, 0.2, 0.6);
  const RunResult r = engine->run(s, {.contact = ContactMethod::kNative});
  const double expected = 9.81 * (std::sin(theta) - 0.2 * std::cos(theta));
  CHECK(acceleration(r.trajectory, 0, 0.5, 1.5) == doctest::Approx(expected).epsilon(0.05));
  CHECK(std::abs(acceleration(r.trajectory, 1, 0.5, 1.5)) < 0.05 * 9.81 * std::sin(theta));
}

TEST_CASE("chrono: an applied force F = (t, 0, 0) gives v = t²/(2m), in the three methods") {
  const auto engine = chrono_engine();
  if (!engine) return;
  Scenario s;
  s.gravity = Vec3::Zero();
  s.duration = 1.0;
  s.grounds.push_back(planar_ground(kData));  // Far below: the contact model needs a pair.
  s.bodies.push_back(cube("box_200x200x200", 0.2, Vec3(0, 0, 1)));
  s.contacts.push_back({BodyRef::body(0), BodyRef::ground(0), 0.5, {}, {}});
  s.bodies[0].applied_force = [](double t) { return Vec3(t, 0, 0); };
  for (const ContactMethod m : {ContactMethod::kGonthier, ContactMethod::kDistributed, ContactMethod::kNative}) {
    const RunResult r = engine->run(s, {.contact = m});
    const double expected = 1.0 / (2.0 * s.bodies[0].mass);
    CHECK_MESSAGE(r.trajectory.twist(r.trajectory.size() - 1, 0).v.x() == doctest::Approx(expected).epsilon(2e-3),
                  to_string(m));
  }
}

TEST_CASE("chrono native: a cube rests in corner_s300") {
  const auto engine = chrono_engine();
  if (!engine) return;
  Scenario s;
  s.duration = 1.0;
  GroundSpec corner;
  corner.shape = shape_meshes(kData, "corner_s300");
  s.grounds.push_back(corner);
  s.bodies.push_back(cube("box_100x100x100", 0.1, Vec3(0.05, 0.05, 0.05 - 1e-6)));
  s.contacts.push_back({BodyRef::body(0), BodyRef::ground(0), 0.5, {}, {}});
  const RunResult r = engine->run(s, {.contact = ContactMethod::kNative});
  const Pose& end = r.trajectory.pose(r.trajectory.size() - 1, 0);
  const Vec3 drift = end.p - Vec3(0.05, 0.05, 0.05);
  std::printf("corner rest: displacement (%.3g, %.3g, %.3g) m, speed %.3g m/s\n", drift.x(), drift.y(), drift.z(),
              r.trajectory.twist(r.trajectory.size() - 1, 0).v.norm());
  // It sinks m g / (K A) = 0.98 mm on its face (K = 1e6 N/m³) and nowhere else.
  CHECK(drift.head<2>().norm() < 1e-5);
  CHECK(-drift.z() == doctest::Approx(9.81 / (1e6 * 0.01)).epsilon(0.05));
  CHECK(r.trajectory.twist(r.trajectory.size() - 1, 0).v.norm() < 1e-2);
}
