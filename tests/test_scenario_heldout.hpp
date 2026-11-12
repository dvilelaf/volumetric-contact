#pragma once

// Helpers shared by the tests of the held-out scenarios.

#include <cmath>
#include <string_view>
#include <vector>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

namespace volcontact::test {

inline const ScenarioOptions kHeldOutOptions{.data_dir = VOLCONTACT_DATA_DIR};

inline std::vector<double> sample_times(double dt, double duration) {
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  return t;
}

inline double scalar_of(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

// Total overlap volume of the scenario's pairs at its initial poses, for one law.
inline double initial_overlap(const Scenario& s, ForceModel law) {
  const ContactModel model = make_contact_model(s, law);
  std::vector<Pose> poses;
  std::vector<Twist> twists;
  for (const BodySpec& b : s.bodies) {
    poses.push_back(b.X_W0);
    twists.push_back(b.V_W0);
  }
  for (const GroundSpec& g : s.grounds) {
    poses.push_back(g.X_WG);
    twists.push_back({});
  }
  ContactModel::Evaluation eval;
  model.evaluate(poses, twists, model.initial_friction(), &eval);
  double volume = 0.0;
  for (const auto& pair : eval.pairs) {
    if (pair.in_contact) volume += pair.geometry.volume;
  }
  return volume;
}

}  // namespace volcontact::test
