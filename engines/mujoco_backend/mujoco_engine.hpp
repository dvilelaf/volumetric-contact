#pragma once

#include <memory>

#include "volcontact/engine.hpp"

namespace volcontact {

// MuJoCo (free joints, fixed step, implicitfast integrator). Volumetric contact
// enters through xfrc_applied, evaluated at the state each step starts from, with
// MuJoCo's own contacts disabled; native contact is MuJoCo's soft constraint
// between explicit pairs of convex geoms (docs/scenarios.md §3).
[[nodiscard]] std::unique_ptr<Engine> make_mujoco_engine();

}  // namespace volcontact
