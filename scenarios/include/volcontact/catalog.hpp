#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "volcontact/scenario.hpp"

namespace volcontact {

// Mesh resolution of the curved assets: segments per full turn.
enum class Resolution { kCoarse = 32, kBase = 64, kFine = 128 };

struct ScenarioOptions {
  std::filesystem::path data_dir = "data";
  Resolution resolution = Resolution::kBase;
  // Factor on the foundation stiffness K of every contact (the volumetric laws and the native
  // models), damping unchanged. `make_scenario` applies it. A theory that contains K (class C)
  // does not follow it: use it on classes A and B only.
  double stiffness_scale = 1.0;

  [[nodiscard]] int segments() const { return static_cast<int>(resolution); }
};

struct CatalogEntry {
  std::string name;
  Scenario (*make)(const ScenarioOptions&);
};

// The scenarios of docs/scenarios.md §4, in table order. A scenario that is
// not written yet throws std::logic_error("not implemented") when made.
[[nodiscard]] const std::vector<CatalogEntry>& catalog();

// Throws std::out_of_range for an unknown name.
[[nodiscard]] Scenario make_scenario(const std::string& name, const ScenarioOptions& options);

// Whether the scenario's assets depend on the mesh resolution (it has a curved asset).
// Throws what make_scenario throws.
[[nodiscard]] bool depends_on_resolution(const std::string& name, const std::filesystem::path& data_dir = "data");

// A way of running a scenario: the experiments of the convergence series and the
// perturbed starts (docs/scenarios.md §2).
struct Variant {
  std::string name;
  Resolution resolution = Resolution::kBase;
  double dt = 1e-3;
  double stiffness_scale = 1.0;
  int perturbation = 0;  // 0: none; k >= 1: the k-th seeded perturbation.
};

// base (N = 64, dt 1 ms); mesh32, mesh128; dt2, dt0.5, dt0.2, dt0.1 (ms); stiff (K × 10 at
// dt 0.2 ms); perturbed1 … perturbed8.
[[nodiscard]] const std::vector<Variant>& variants();

// Adds Δv = 1 mm/s along (1, 2, 3)/√14 and Δω = 0.01 rad/s about (3, −1, 2)/√14
// to the initial velocity of every dynamic body.
void perturb(Scenario& scenario);

// The same magnitudes with directions drawn from a generator seeded with `seed` (>= 1; the
// same draw on every platform), equal for every body. `seed` 0 is the fixed perturbation above.
void perturb(Scenario& scenario, int seed);

}  // namespace volcontact
