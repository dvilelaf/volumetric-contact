#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "volcontact/scenario.hpp"

namespace volcontact {

enum class ContactMethod {
  kGonthier,     // Gonthier's lumped volumetric model (the thesis) on the exact mesh intersection.
  kDistributed,  // Distributed elastic-foundation model on the same intersection.
  kPcm,          // Per-face elastic foundation of the literature (PCM, Simbody) on the same intersection.
  kNative,       // One of the engine's own contact models (Engine::native_models()).
};

// The force model of the volumetric methods.
[[nodiscard]] ForceModel force_model(ContactMethod method);

[[nodiscard]] std::string_view to_string(ContactMethod method);

struct RunOptions {
  double dt = 1e-3;
  ContactMethod contact = ContactMethod::kGonthier;
  bool record = true;              // Record the trajectory (off for timing runs).
  // Native contact model, one of Engine::native_models(); empty selects the engine's default.
  std::string native_model;
  bool native_primitives = false;  // Native contact on analytic shapes instead of the meshes.
  double native_resolution = 0.05; // Tessellation hint for native primitives [m].
  // Wall-clock budget of the stepping loop [s]; 0 means none. A run that exceeds it
  // throws RunTimeout (Drake checks it every 100 steps).
  double time_limit_seconds = 0.0;
};

// Thrown by Engine::run when RunOptions::time_limit_seconds ran out.
class RunTimeout : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct RunStats {
  double wall_seconds = 0.0;     // Time spent advancing the simulation.
  double contact_seconds = 0.0;  // Part of it spent in this library's contact model.
  std::int64_t steps = 0;
  // Native contact, recorded runs: mean number of contact points the engine
  // used per step, over the steps that had any (0 if it never had contact).
  double native_contact_points = 0.0;
};

struct RunResult {
  Trajectory trajectory;
  RunStats stats;
};

// A multibody engine able to run a Scenario.
class Engine {
 public:
  virtual ~Engine() = default;
  [[nodiscard]] virtual std::string_view name() const = 0;
  // Names of the engine's own contact models (docs/scenarios.md §3); the first is its default.
  [[nodiscard]] virtual std::vector<std::string> native_models() const = 0;
  [[nodiscard]] virtual RunResult run(const Scenario& scenario, const RunOptions& options) const = 0;
};

// The native model `options` selects on `engine`: its default when
// `native_model` is empty. Throws std::invalid_argument for a name the engine lacks.
[[nodiscard]] std::string native_model(const Engine& engine, const RunOptions& options);

// A contact method as run and reported: the label is the method's name, with
// `native` for the engine's default native model and `native-<model>` for another.
struct ContactChoice {
  ContactMethod contact = ContactMethod::kGonthier;
  std::string native_model;  // Empty: the engine's default.
  std::string label;
};

// The methods a `--contact` argument asks of an engine: `gonthier`,
// `distributed`, `pcm`, `native` (its default native model), `native:<model>`
// (that one; none if the engine lacks it), `native:all` (every one) or `all`
// (`gonthier`, `distributed` and the default native model). Empty for anything else.
[[nodiscard]] std::vector<ContactChoice> contact_choices(const Engine& engine, std::string_view argument);

// Engines compiled into this build.
[[nodiscard]] std::vector<std::unique_ptr<Engine>> available_engines();

}  // namespace volcontact
