// The native-contact contract of the engines (docs/scenarios.md §3): the models
// each engine offers, how they are selected, and the geometry helpers behind them.

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <doctest/doctest.h>

#include "../engines/native_geometry.hpp"
#include "test_util.hpp"
#include "volcontact/engine.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};

std::unique_ptr<Engine> engine_named(std::string_view name) {
  for (auto& e : available_engines()) {
    if (e->name() == name) return std::move(e);
  }
  return nullptr;
}

std::vector<std::string> labels(const Engine& engine, std::string_view argument) {
  std::vector<std::string> names;
  for (const ContactChoice& c : contact_choices(engine, argument)) names.push_back(c.label);
  return names;
}

double edge_max(const TriMesh& m) {
  double longest = 0.0;
  for (const auto& f : m.triangles) {
    for (int k = 0; k < 3; ++k) longest = std::max(longest, (m.vertices[f[k]] - m.vertices[f[(k + 1) % 3]]).norm());
  }
  return longest;
}

}  // namespace

TEST_CASE("native models: every engine lists its models, the default first") {
  const auto engines = available_engines();
  for (const auto& e : engines) {
    CAPTURE(e->name());
    const std::vector<std::string> models = e->native_models();
    REQUIRE(!models.empty());
    CHECK(native_model(*e, {}) == models.front());
    CHECK(native_model(*e, {.native_model = models.back()}) == models.back());
    CHECK_THROWS_AS((void)native_model(*e, {.native_model = "no-such-model"}), std::invalid_argument);
  }
  if (const auto drake = engine_named("drake")) {
    CHECK(drake->native_models() == std::vector<std::string>{"hydroelastic", "point"});
  }
  if (const auto chrono = engine_named("chrono")) {
    CHECK(chrono->native_models() == std::vector<std::string>{"smc", "nsc"});
  }
  if (const auto mujoco = engine_named("mujoco")) CHECK(mujoco->native_models() == std::vector<std::string>{"soft", "default"});
}

TEST_CASE("native models: a run with a model the engine lacks throws") {
  Scenario s;
  s.duration = 0.01;
  s.bodies.emplace_back();
  for (const auto& e : available_engines()) {
    CAPTURE(e->name());
    CHECK_THROWS_AS((void)e->run(s, {.contact = ContactMethod::kNative, .native_model = "no-such-model"}),
                    std::invalid_argument);
  }
}

TEST_CASE("--contact syntax: native is the default model, native:<model> one, native:all every one") {
  const auto chrono = engine_named("chrono");
  if (!chrono) return;
  CHECK(labels(*chrono, "all") == std::vector<std::string>{"gonthier", "distributed", "native"});
  CHECK(labels(*chrono, "gonthier") == std::vector<std::string>{"gonthier"});
  CHECK(labels(*chrono, "native") == std::vector<std::string>{"native"});
  CHECK(labels(*chrono, "native:smc") == std::vector<std::string>{"native"});
  CHECK(labels(*chrono, "native:nsc") == std::vector<std::string>{"native-nsc"});
  CHECK(labels(*chrono, "native:all") == std::vector<std::string>{"native", "native-nsc"});
  CHECK(labels(*chrono, "native:point").empty());  // Another engine's model.
  CHECK(labels(*chrono, "nonsense").empty());

  const std::vector<ContactChoice> nsc = contact_choices(*chrono, "native:nsc");
  REQUIRE(nsc.size() == 1);
  CHECK(nsc[0].contact == ContactMethod::kNative);
  CHECK(nsc[0].native_model == "nsc");
  CHECK(contact_choices(*chrono, "native")[0].native_model.empty());
}

TEST_CASE("refined: every edge within the limit, same surface, still closed") {
  const TriMesh box = test::make_box(Vec3(0, 0, 0), Vec3(1.0, 0.5, 0.05));
  const TriMesh fine = native::refined(box, 0.125);
  CHECK(edge_max(fine) <= 0.125 + 1e-12);
  CHECK(native::surface_area(fine) == doctest::Approx(native::surface_area(box)).epsilon(1e-12));
  CHECK(signed_volume(fine) == doctest::Approx(0.025).epsilon(1e-12));
  CHECK(fine.triangles.size() >= 2 * 8 * 4 * 2);  // The large faces alone: 8 x 4 cells of two triangles.
  // Nothing to do: the mesh comes back unchanged.
  CHECK(native::refined(box, 2.0).triangles.size() == box.triangles.size());
}

TEST_CASE("reference area: the face of a cube, the mean face of a box") {
  CHECK(native::reference_area(test::make_box(Vec3::Zero(), Vec3::Constant(0.1))) == doctest::Approx(0.01));
  CHECK(native::reference_area(test::make_box(Vec3::Zero(), Vec3(1.0, 2.0, 3.0))) == doctest::Approx(11.0 / 3.0));
}

TEST_CASE("as_box: recognises a rotated box, rejects a wedge") {
  TriMesh box = test::make_box(Vec3(-0.1, -0.2, -0.3), Vec3(0.1, 0.2, 0.3));
  const Mat3 R = test::rot(0.7, Vec3(1.0, 2.0, 3.0));
  const Vec3 p(0.5, -1.0, 2.0);
  for (Vec3& v : box.vertices) v = R * v + p;
  const auto found = native::as_box(box);
  REQUIRE(found.has_value());
  CHECK(found->X.p.isApprox(p, 1e-12));
  CHECK(found->X.R.determinant() == doctest::Approx(1.0));
  // The box frame reproduces the corners.
  for (const Vec3& v : box.vertices) {
    const Vec3 local = found->X.inverse().apply(v);
    for (int k = 0; k < 3; ++k) CHECK(std::abs(local[k]) == doctest::Approx(0.5 * found->size[k]));
  }
  Vec3 sorted = found->size;
  std::sort(sorted.data(), sorted.data() + 3);
  CHECK(sorted.isApprox(Vec3(0.2, 0.4, 0.6), 1e-12));

  TriMesh wedge = test::make_box(Vec3::Zero(), Vec3::Ones());
  wedge.vertices[7] = Vec3(1.0, 1.0, 0.5);
  wedge.vertices[6] = Vec3(0.0, 1.0, 0.5);
  CHECK(!native::as_box(wedge).has_value());
}

TEST_CASE("drake hydroelastic: a box on the half-space has the support to resist tilting (offset)") {
  const auto drake = engine_named("drake");
  if (!drake) return;
  const Scenario s = make_scenario("offset", kOptions);
  const RunResult r = drake->run(s, {.contact = ContactMethod::kNative});
  // Two contact points (a 12-triangle box) gave 113 % here.
  CHECK(s.metrics(r.trajectory).error < 0.05);
  CHECK(r.stats.native_contact_points > 100.0);
}

TEST_CASE("drake point: block slides at g(sin θ − μ cos θ) on its corner spheres") {
  const auto drake = engine_named("drake");
  if (!drake) return;
  const Scenario s = make_scenario("block", kOptions);
  const RunResult r = drake->run(s, {.contact = ContactMethod::kNative, .native_model = "point"});
  CHECK(s.metrics(r.trajectory).error < 0.02);
  CHECK(r.stats.native_contact_points >= 4.0);
}

TEST_CASE("chrono smc and nsc: a box on a plane has a manifold of at least 3 points, and block is right") {
  const auto chrono = engine_named("chrono");
  if (!chrono) return;
  const Scenario s = make_scenario("block", kOptions);
  for (const std::string model : {"smc", "nsc"}) {
    CAPTURE(model);
    const RunResult r = chrono->run(s, {.contact = ContactMethod::kNative, .native_model = model});
    CHECK(r.stats.native_contact_points >= 3.0);
    CHECK(s.metrics(r.trajectory).error < 0.02);
  }
}

TEST_CASE("chrono smc and nsc: a box without a primitive (convex piece) also gets a manifold") {
  const auto chrono = engine_named("chrono");
  if (!chrono) return;
  Scenario s = make_scenario("block", kOptions);
  s.bodies[0].primitive.reset();
  for (const std::string model : {"smc", "nsc"}) {
    CAPTURE(model);
    const RunResult r = chrono->run(s, {.contact = ContactMethod::kNative, .native_model = model});
    CHECK(r.stats.native_contact_points >= 3.0);
    CHECK(s.metrics(r.trajectory).error < 0.02);
  }
}

TEST_CASE("chrono smc: a resting cube sinks m g / (K A) and a held one creeps below the bound of its viscous stick") {
  const auto chrono = engine_named("chrono");
  if (!chrono) return;
  // 1 kg cube of 100 mm on a 10° incline, μ = 0.5: it must hold.
  constexpr double K = 1e6, a = 0.1, theta = 10.0 * 3.14159265358979 / 180.0;
  Scenario s;
  s.duration = 1.0;
  s.gravity = 9.81 * Vec3(std::sin(theta), 0.0, -std::cos(theta));
  s.native = {K, 5.0};
  s.grounds.push_back(planar_ground(kOptions.data_dir));
  BodySpec b;
  b.shape = shape_meshes(kOptions.data_dir, "box_100x100x100");
  b.inertia = Mat3::Identity() / 600.0;
  b.X_W0.p = Vec3(0.0, 0.0, 0.5 * a);
  s.bodies.push_back(b);
  s.contacts.push_back({.a = BodyRef::body(0), .b = BodyRef::ground(0), .friction = 0.5});
  const RunResult r = chrono->run(s, {.contact = ContactMethod::kNative});
  const std::size_t last = r.trajectory.size() - 1;
  CHECK(0.5 * a - r.trajectory.pose(last, 0).p.z() == doctest::Approx(9.81 * std::cos(theta) / (K * a * a)).epsilon(0.05));
  // Stick is viscous in Chrono's SMC: a full manifold takes a quarter of the
  // momentum per step, so the creep speed stays below 4·g·sin θ·Δt (Δt = 1 ms here).
  CHECK(std::abs(r.trajectory.twist(last, 0).v.x()) < 4.0 * 9.81 * std::sin(theta) * 1e-3);
  CHECK(r.stats.native_contact_points >= 3.0);
}
