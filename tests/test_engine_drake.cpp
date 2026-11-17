#include <cmath>
#include <stdexcept>

#include <doctest/doctest.h>

#include "../engines/drake_backend/native_materials.hpp"
#include "volcontact/engine.hpp"

using namespace volcontact;

namespace {

const std::filesystem::path kData = VOLCONTACT_DATA_DIR;
constexpr double kStiffness = 1e6;

const Engine* drake_engine() {
  static const auto engines = available_engines();
  for (const auto& e : engines) {
    if (e->name() == "drake") return e.get();
  }
  return nullptr;
}

// A 100 mm cube of 1 kg centred at `p`, in contact with ground 0.
Scenario cube_scenario(const Vec3& p) {
  Scenario s;
  s.duration = 1.0;
  s.native = {kStiffness, 5.0};
  BodySpec b;
  b.name = "cube";
  b.shape = shape_meshes(kData, "box_100x100x100");
  b.inertia = Mat3::Identity() / 600.0;
  b.X_W0.p = p;
  s.bodies.push_back(b);
  s.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::ground(0), .friction = 0.5});
  return s;
}

// Static penetration of the cube below its resting height.
double settled_penetration(const Scenario& s, double resting_z) {
  const RunResult r = drake_engine()->run(s, {.dt = 1e-3, .contact = ContactMethod::kNative});
  return resting_z - r.trajectory.pose(r.trajectory.size() - 1, 0).p.z();
}

constexpr double kExpected = 9.81 / (kStiffness * 0.1 * 0.1);  // mg / (K A).

double combined(double a, double b) { return 2.0 * a * b / (a + b); }

}  // namespace

TEST_CASE("native contact of a cube on a compliant VTK floor: static penetration is mg/(K A)") {
  if (!drake_engine()) return;
  Scenario s = cube_scenario(Vec3(0.2, 0.2, 0.05 - 1e-6));
  s.grounds.push_back({.shape = shape_meshes(kData, "corner_s300")});
  CHECK(settled_penetration(s, 0.05) == doctest::Approx(kExpected).epsilon(0.03));
}

TEST_CASE("native contact of a cube on a compliant cube: static penetration is mg/(K A)") {
  if (!drake_engine()) return;
  Scenario s = cube_scenario(Vec3(0.0, 0.0, 0.05 - 1e-6));
  GroundSpec g{.shape = shape_meshes(kData, "box_200x200x200")};
  g.X_WG.p = Vec3(0.0, 0.0, -0.1);
  s.grounds.push_back(g);
  CHECK(settled_penetration(s, 0.05) == doctest::Approx(kExpected).epsilon(0.03));
}

TEST_CASE("friction solver reproduces the pair coefficients of a chain") {
  Scenario s;
  s.bodies.resize(2);
  s.grounds.resize(1);
  s.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::ground(0), .friction = 0.2});
  s.contacts.push_back({.a = BodyRef::body(1), .b = BodyRef::body(0), .friction = 0.6});
  const auto m = drake_backend::solve_native_materials(s);
  CHECK(combined(m.friction[0], m.friction[2]) == doctest::Approx(0.2));
  CHECK(combined(m.friction[1], m.friction[0]) == doctest::Approx(0.6));
  CHECK(m.compliant == std::vector<bool>{false, true, true});  // Rigid, compliant, ground.
}

TEST_CASE("friction solver rejects a triangle and an inconsistent cycle") {
  Scenario triangle;
  triangle.bodies.resize(2);
  triangle.grounds.resize(1);
  triangle.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::ground(0), .friction = 0.5});
  triangle.contacts.push_back({.a = BodyRef::body(1), .b = BodyRef::ground(0), .friction = 0.5});
  triangle.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::body(1), .friction = 0.5});
  CHECK_THROWS_AS(drake_backend::solve_native_materials(triangle), std::runtime_error);

  Scenario square;
  square.bodies.resize(2);
  square.grounds.resize(2);
  const double mu[4] = {0.2, 0.3, 0.4, 0.5};
  square.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::ground(0), .friction = mu[0]});
  square.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::ground(1), .friction = mu[1]});
  square.contacts.push_back({.a = BodyRef::body(1), .b = BodyRef::ground(1), .friction = mu[2]});
  square.contacts.push_back({.a = BodyRef::body(1), .b = BodyRef::ground(0), .friction = mu[3]});
  CHECK_THROWS_AS(drake_backend::solve_native_materials(square), std::runtime_error);
}

TEST_CASE("applied force F = (t, 0, 0) gives v = t²/(2m) in every method") {
  if (!drake_engine()) return;
  Scenario s;
  s.gravity = Vec3::Zero();
  s.duration = 1.0;
  BodySpec b;
  b.name = "free";
  b.mass = 2.0;
  b.applied_force = [](double t) { return Vec3(t, 0.0, 0.0); };
  s.bodies.push_back(b);
  for (ContactMethod method : {ContactMethod::kGonthier, ContactMethod::kDistributed, ContactMethod::kNative}) {
    CAPTURE(to_string(method));
    const RunResult r = drake_engine()->run(s, {.dt = 1e-3, .contact = method});
    const std::size_t last = r.trajectory.size() - 1;
    const double t = r.trajectory.times()[last];
    CHECK(r.trajectory.twist(last, 0).v.x() == doctest::Approx(t * t / 4.0).epsilon(0.01));
  }
}

TEST_CASE("native contact of three stacked cubes: rigid cubes 1 and 3 never pair, compressions are n m g/(K a²)") {
  if (!drake_engine()) return;
  constexpr double a = 0.2;
  Scenario s;
  s.duration = 1.0;
  s.native = {kStiffness, 5.0};
  for (int k = 0; k < 3; ++k) {
    BodySpec b;
    b.name = "cube" + std::to_string(k + 1);
    b.shape = shape_meshes(kData, "box_200x200x200");
    b.inertia = Mat3::Identity() / 150.0;
    b.X_W0.p = Vec3(0.0, 0.0, (k + 0.5) * a - (k + 1) * 1e-6);
    b.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3::Constant(a), .X_BG = {}};
    s.bodies.push_back(b);
  }
  s.grounds.push_back({.shape = shape_meshes(kData, "slab_40000x40000x1000"), .planar = true});
  s.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::ground(0), .friction = 0.5});
  s.contacts.push_back({.a = BodyRef::body(1), .b = BodyRef::body(0), .friction = 0.5});
  s.contacts.push_back({.a = BodyRef::body(2), .b = BodyRef::body(1), .friction = 0.5});

  RunResult r;
  REQUIRE_NOTHROW(r = drake_engine()->run(s, {.dt = 1e-3, .contact = ContactMethod::kNative,
                                                .native_primitives = true}));
  const auto z = [&](int k) { return r.trajectory.pose(r.trajectory.size() - 1, k).p.z(); };
  const double unit = 9.81 / (kStiffness * a * a);
  CHECK(0.5 * a - z(0) == doctest::Approx(3 * unit).epsilon(0.03));
  CHECK(z(0) - z(1) + a == doctest::Approx(2 * unit).epsilon(0.03));
  CHECK(z(1) - z(2) + a == doctest::Approx(1 * unit).epsilon(0.03));
}
