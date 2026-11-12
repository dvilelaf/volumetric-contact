#pragma once

#include <memory>

#include "volcontact/engine.hpp"

namespace volcontact {

// Drake MultibodyPlant (discrete, fixed step). Volumetric contact enters as
// externally applied spatial forces; native contact (docs/scenarios.md §3) is
// `hydroelastic` or `point`.
[[nodiscard]] std::unique_ptr<Engine> make_drake_engine();

}  // namespace volcontact
