#pragma once

#include <memory>

#include "volcontact/engine.hpp"

namespace volcontact {

// Project Chrono (fixed step, implicit linearized Euler). Volumetric contact
// enters through body force accumulators, evaluated once per step, in a
// ChSystemSMC without collision. Native contact (docs/scenarios.md §3) is `smc`,
// Chrono's smooth (penalty) contact, sub-stepped because it is explicit, or
// `nsc`, its complementarity contact.
[[nodiscard]] std::unique_ptr<Engine> make_chrono_engine();

}  // namespace volcontact
