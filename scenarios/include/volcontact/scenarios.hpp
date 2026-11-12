#pragma once

#include "volcontact/catalog.hpp"

// One factory per scenario of docs/scenarios.md §4. The incline is modelled by
// tilting gravity, so every planar ground has its top at z = 0.
namespace volcontact::scenarios {

// Thesis tests (chapter 5).
[[nodiscard]] Scenario block(const ScenarioOptions& options);
[[nodiscard]] Scenario disk(const ScenarioOptions& options);
[[nodiscard]] Scenario cylinder(const ScenarioOptions& options);

// The block test on an incline of `angle_deg`, for the sliding-onset search.
[[nodiscard]] Scenario sliding_block(const ScenarioOptions& options, double angle_deg = 30.0, double duration = 4.0);

[[nodiscard]] Scenario push(const ScenarioOptions& options);
[[nodiscard]] Scenario tip(const ScenarioOptions& options);
[[nodiscard]] Scenario drop(const ScenarioOptions& options);
[[nodiscard]] Scenario billiard(const ScenarioOptions& options);
[[nodiscard]] Scenario contensou(const ScenarioOptions& options);
[[nodiscard]] Scenario vgroove(const ScenarioOptions& options);
[[nodiscard]] Scenario ring(const ScenarioOptions& options);
[[nodiscard]] Scenario hole(const ScenarioOptions& options);
[[nodiscard]] Scenario bowl(const ScenarioOptions& options);
[[nodiscard]] Scenario trough(const ScenarioOptions& options);
[[nodiscard]] Scenario bearing(const ScenarioOptions& options);
[[nodiscard]] Scenario corner(const ScenarioOptions& options);
[[nodiscard]] Scenario offset(const ScenarioOptions& options);
[[nodiscard]] Scenario twoblocks(const ScenarioOptions& options);
[[nodiscard]] Scenario stack(const ScenarioOptions& options);
[[nodiscard]] Scenario cone(const ScenarioOptions& options);
[[nodiscard]] Scenario sphere(const ScenarioOptions& options);
[[nodiscard]] Scenario journal(const ScenarioOptions& options);

// Held-out set (docs/scenarios.md §4.3): written with their theory before any contact law
// was run on them.
[[nodiscard]] Scenario pieces1(const ScenarioOptions& options);
[[nodiscard]] Scenario pieces2(const ScenarioOptions& options);
[[nodiscard]] Scenario pieces8(const ScenarioOptions& options);
[[nodiscard]] Scenario pieces64(const ScenarioOptions& options);
[[nodiscard]] Scenario wedge(const ScenarioOptions& options);
[[nodiscard]] Scenario restitution(const ScenarioOptions& options);
[[nodiscard]] Scenario oblique(const ScenarioOptions& options);
[[nodiscard]] Scenario edge(const ScenarioOptions& options);
[[nodiscard]] Scenario turntable(const ScenarioOptions& options);
[[nodiscard]] Scenario energy(const ScenarioOptions& options);
[[nodiscard]] Scenario tower(const ScenarioOptions& options);
[[nodiscard]] Scenario massratio(const ScenarioOptions& options);

}  // namespace volcontact::scenarios
