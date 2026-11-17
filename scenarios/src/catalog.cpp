#include "volcontact/scenarios.hpp"

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <stdexcept>

namespace volcontact {

const std::vector<CatalogEntry>& catalog() {
  static const std::vector<CatalogEntry> entries = {
      {"block", scenarios::block},       {"disk", scenarios::disk},         {"cylinder", scenarios::cylinder},
      {"push", scenarios::push},         {"tip", scenarios::tip},           {"drop", scenarios::drop},
      {"billiard", scenarios::billiard}, {"contensou", scenarios::contensou}, {"vgroove", scenarios::vgroove},
      {"ring", scenarios::ring},         {"hole", scenarios::hole},         {"bowl", scenarios::bowl},
      {"trough", scenarios::trough},     {"bearing", scenarios::bearing},   {"corner", scenarios::corner},
      {"offset", scenarios::offset},     {"twoblocks", scenarios::twoblocks}, {"stack", scenarios::stack},
      {"cone", scenarios::cone},         {"sphere", scenarios::sphere},     {"journal", scenarios::journal},
      {"pieces1", scenarios::pieces1},   {"pieces2", scenarios::pieces2},   {"pieces8", scenarios::pieces8},
      {"pieces64", scenarios::pieces64}, {"wedge", scenarios::wedge},       {"restitution", scenarios::restitution},
      {"oblique", scenarios::oblique},   {"edge", scenarios::edge},         {"turntable", scenarios::turntable},
      {"energy", scenarios::energy},
      {"tower", scenarios::tower},       {"massratio", scenarios::massratio},
  };
  return entries;
}

Scenario make_scenario(const std::string& name, const ScenarioOptions& options) {
  for (const CatalogEntry& e : catalog()) {
    if (e.name != name) continue;
    Scenario s = e.make(options);
    if (options.stiffness_scale != 1.0) {
      for (ContactSpec& c : s.contacts) {
        c.gonthier.stiffness *= options.stiffness_scale;
        c.distributed.stiffness *= options.stiffness_scale;
      }
      s.native.stiffness *= options.stiffness_scale;
    }
    return s;
  }
  throw std::out_of_range("unknown scenario " + name);
}

bool depends_on_resolution(const std::string& name, const std::filesystem::path& data_dir) {
  const Scenario base = make_scenario(name, {.data_dir = data_dir, .resolution = Resolution::kBase});
  const Scenario fine = make_scenario(name, {.data_dir = data_dir, .resolution = Resolution::kFine});
  for (std::size_t i = 0; i < base.bodies.size(); ++i) {
    if (base.bodies[i].shape.surface != fine.bodies[i].shape.surface) return true;
  }
  for (std::size_t i = 0; i < base.grounds.size(); ++i) {
    if (base.grounds[i].shape.surface != fine.grounds[i].shape.surface) return true;
  }
  return false;
}

const std::vector<Variant>& variants() {
  static const std::vector<Variant> list = [] {
    std::vector<Variant> v = {
        {.name = "base"},
        {.name = "mesh32", .resolution = Resolution::kCoarse},
        {.name = "mesh128", .resolution = Resolution::kFine},
        {.name = "dt2", .dt = 2e-3},
        {.name = "dt0.5", .dt = 5e-4},
        {.name = "dt0.2", .dt = 2e-4},
        {.name = "dt0.1", .dt = 1e-4},
        {.name = "stiff", .dt = 2e-4, .stiffness_scale = 10.0},
    };
    for (int k = 1; k <= 8; ++k) v.push_back({.name = "perturbed" + std::to_string(k), .perturbation = k});
    return v;
  }();
  return list;
}

void perturb(Scenario& scenario) {
  const Vec3 dv = 1e-3 * Vec3(1.0, 2.0, 3.0) / std::sqrt(14.0);
  const Vec3 dw = 1e-2 * Vec3(3.0, -1.0, 2.0) / std::sqrt(14.0);
  for (BodySpec& body : scenario.bodies) {
    body.V_W0.v += dv;
    body.V_W0.w += dw;
  }
}

namespace {

// A uniformly distributed unit vector from the raw output of a mt19937_64 (Box–Muller on 53-bit
// uniforms: <random>'s distributions are not specified to give the same numbers everywhere).
Vec3 random_direction(std::mt19937_64& rng) {
  const auto uniform = [&] { return (static_cast<double>(rng() >> 11) + 0.5) * 0x1p-53; };
  const auto normal = [&] { return std::sqrt(-2.0 * std::log(uniform())) * std::cos(2.0 * std::numbers::pi * uniform()); };
  Vec3 d(normal(), normal(), normal());
  return d.normalized();
}

}  // namespace

void perturb(Scenario& scenario, int seed) {
  if (seed <= 0) return perturb(scenario);
  std::mt19937_64 rng(static_cast<std::uint64_t>(seed));
  const Vec3 dv = 1e-3 * random_direction(rng);
  const Vec3 dw = 1e-2 * random_direction(rng);
  for (BodySpec& body : scenario.bodies) {
    body.V_W0.v += dv;
    body.V_W0.w += dw;
  }
}

}  // namespace volcontact
