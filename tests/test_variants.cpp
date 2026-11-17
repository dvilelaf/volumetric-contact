// The run matrix of the convergence series (docs/scenarios.md §2): variants, seeded
// perturbations, the stiffness scale and the wall-clock limit of a run.

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

#include <doctest/doctest.h>

#include "volcontact/engine.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};

const Variant& variant(const std::string& name) {
  for (const Variant& v : variants()) {
    if (v.name == name) return v;
  }
  throw std::out_of_range(name);
}

}  // namespace

TEST_CASE("the variants are the base, the mesh and step series, the stiff limit and eight perturbations") {
  std::set<std::string> names;
  for (const Variant& v : variants()) names.insert(v.name);
  CHECK(names.size() == variants().size());
  CHECK(variants().size() == 16);
  CHECK(variant("base").resolution == Resolution::kBase);
  CHECK(variant("base").dt == doctest::Approx(1e-3));
  CHECK(variant("mesh32").resolution == Resolution::kCoarse);
  CHECK(variant("mesh128").resolution == Resolution::kFine);
  for (const auto& [name, dt] : {std::pair{"dt2", 2e-3}, {"dt0.5", 5e-4}, {"dt0.2", 2e-4}, {"dt0.1", 1e-4}}) {
    CHECK(variant(name).dt == doctest::Approx(dt));
  }
  CHECK(variant("stiff").stiffness_scale == doctest::Approx(10.0));
  CHECK(variant("stiff").dt == doctest::Approx(2e-4));
  for (int k = 1; k <= 8; ++k) CHECK(variant("perturbed" + std::to_string(k)).perturbation == k);
}

TEST_CASE("a seeded perturbation has the magnitudes of the fixed one, a fixed direction per seed") {
  const auto perturbed = [](int seed) {
    Scenario s;
    s.bodies.resize(2);
    perturb(s, seed);
    return s;
  };
  const Scenario a = perturbed(3), again = perturbed(3), b = perturbed(4);
  CHECK(a.bodies[0].V_W0.v.norm() == doctest::Approx(1e-3));
  CHECK(a.bodies[0].V_W0.w.norm() == doctest::Approx(1e-2));
  CHECK(a.bodies[1].V_W0.v.isApprox(a.bodies[0].V_W0.v));
  CHECK(a.bodies[0].V_W0.v.isApprox(again.bodies[0].V_W0.v, 0.0));
  CHECK_FALSE(a.bodies[0].V_W0.v.isApprox(b.bodies[0].V_W0.v));
  // Seed 0 is the fixed perturbation.
  Scenario fixed;
  fixed.bodies.resize(1);
  perturb(fixed);
  CHECK(perturbed(0).bodies[0].V_W0.v.isApprox(fixed.bodies[0].V_W0.v));
}

TEST_CASE("the directions of the eight perturbations spread over the sphere") {
  Vec3 mean = Vec3::Zero();
  for (int k = 1; k <= 8; ++k) {
    Scenario s;
    s.bodies.resize(1);
    perturb(s, k);
    mean += s.bodies[0].V_W0.v.normalized() / 8.0;
  }
  CHECK(mean.norm() < 0.8);  // Eight identical directions would give 1.
}

TEST_CASE("only the scenarios with a curved asset depend on the mesh resolution") {
  for (const char* name : {"block", "push", "hole", "twoblocks", "pieces64", "edge"}) {
    CAPTURE(name);
    CHECK_FALSE(depends_on_resolution(name, kOptions.data_dir));
  }
  for (const char* name : {"disk", "cylinder", "billiard", "bowl", "bearing", "cone", "ring", "turntable"}) {
    CAPTURE(name);
    CHECK(depends_on_resolution(name, kOptions.data_dir));
  }
}

TEST_CASE("the coarse resolution resolves every curved asset the scenarios use") {
  const ScenarioOptions coarse{.data_dir = VOLCONTACT_DATA_DIR, .resolution = Resolution::kCoarse};
  for (const CatalogEntry& e : catalog()) {
    CAPTURE(e.name);
    if (!depends_on_resolution(e.name, kOptions.data_dir)) continue;
    const Scenario s = make_scenario(e.name, coarse);
    for (const BodySpec& b : s.bodies) CHECK(b.shape.surface.string().find("_seg32") != std::string::npos);
  }
}

TEST_CASE("the stiffness scale multiplies K of the laws and of the native models, not the damping") {
  const Scenario base = make_scenario("block", kOptions);
  const Scenario stiff = make_scenario("block", {.data_dir = VOLCONTACT_DATA_DIR, .stiffness_scale = 10.0});
  REQUIRE(base.contacts.size() == stiff.contacts.size());
  for (std::size_t i = 0; i < base.contacts.size(); ++i) {
    CHECK(stiff.contacts[i].gonthier.stiffness == doctest::Approx(10.0 * base.contacts[i].gonthier.stiffness));
    CHECK(stiff.contacts[i].distributed.stiffness == doctest::Approx(10.0 * base.contacts[i].distributed.stiffness));
    CHECK(stiff.contacts[i].gonthier.damping == doctest::Approx(base.contacts[i].gonthier.damping));
  }
  CHECK(stiff.native.stiffness == doctest::Approx(10.0 * base.native.stiffness));
  CHECK(stiff.native.damping == doctest::Approx(base.native.damping));
}

TEST_CASE("a run past its wall-clock limit throws RunTimeout, on every engine") {
  const Scenario s = make_scenario("block", kOptions);
  for (const auto& engine : available_engines()) {
    CAPTURE(engine->name());
    RunOptions options{.record = false, .time_limit_seconds = 1e-6};
    CHECK_THROWS_AS((void)engine->run(s, options), RunTimeout);
    options.time_limit_seconds = 0.0;
    Scenario brief = s;
    brief.duration = 0.02;
    CHECK(engine->run(brief, options).stats.steps > 0);
  }
}
