#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "volcontact/scenario.hpp"

namespace volcontact::drake_backend {

// Which geometries of a scenario are compliant in Drake's hydroelastic contact,
// and the Coulomb μ each one must carry (docs/scenarios.md §3). Entries follow
// the contact model's body order: dynamic bodies, then grounds.
struct NativeMaterials {
  std::vector<bool> compliant;
  std::vector<double> friction;
};

// The contact graph is 2-coloured from the grounds (compliant); a body touching
// a compliant one is rigid and vice versa. Drake combines two geometries' μ as
// 2μᵢμⱼ/(μᵢ+μⱼ), i.e. 1/μᵢ + 1/μⱼ = 2/μ of the pair: linear in u = 1/μ. In each
// connected component the colour classes are u = ±a + dᵢ with one free a; it is
// set midway in the range keeping every μ positive. Throws if the graph is not
// bipartite or the pairs' μ are inconsistent.
inline NativeMaterials solve_native_materials(const Scenario& s) {
  const int n = static_cast<int>(s.bodies.size() + s.grounds.size());
  struct Edge {
    int to;
    double sum;  // 2/μ of the pair.
  };
  std::vector<std::vector<Edge>> adjacent(n);
  for (const ContactSpec& c : s.contacts) {
    const int a = contact_body_index(s, c.a), b = contact_body_index(s, c.b);
    adjacent[a].push_back({b, 2.0 / c.friction});
    adjacent[b].push_back({a, 2.0 / c.friction});
  }

  NativeMaterials m;
  m.compliant.assign(n, false);
  m.friction.assign(n, 0.0);
  std::vector<int> side(n, 0);  // 0 unvisited, +1 compliant, -1 rigid.
  std::vector<double> offset(n, 0.0);

  // Components with a ground start at it; the others at their first body (rigid).
  std::vector<int> roots;
  for (int i = static_cast<int>(s.bodies.size()); i < n; ++i) roots.push_back(i);
  for (int i = 0; i < static_cast<int>(s.bodies.size()); ++i) roots.push_back(i);
  const auto fail = [](const std::string& why) { throw std::runtime_error("native contact: " + why); };

  for (int root : roots) {
    if (side[root] != 0) continue;
    std::vector<int> component{root};
    side[root] = root >= static_cast<int>(s.bodies.size()) ? 1 : -1;
    for (std::size_t head = 0; head < component.size(); ++head) {
      const int i = component[head];
      for (const Edge& e : adjacent[i]) {
        if (side[e.to] == 0) {
          side[e.to] = -side[i];
          offset[e.to] = e.sum - offset[i];  // u_to = e.sum − u_i; the a terms cancel.
          component.push_back(e.to);
        } else if (side[e.to] == side[i]) {
          fail("the contact graph is not bipartite, a body cannot be rigid or compliant consistently");
        } else if (std::abs(offset[i] + offset[e.to] - e.sum) > 1e-9 * e.sum) {
          fail("friction coefficients of the contact pairs are inconsistent");
        }
      }
    }
    // u_i = side_i·a + offset_i > 0 for every member.
    double lo = -INFINITY, hi = INFINITY;
    for (int i : component) {
      if (side[i] > 0) lo = std::max(lo, -offset[i]);
      else hi = std::min(hi, offset[i]);
    }
    if (lo >= hi) fail("no positive friction coefficients reproduce the pairs");
    const double a = std::isinf(lo) ? (std::isinf(hi) ? 1.0 : hi - 1.0) : std::isinf(hi) ? lo + 1.0 : 0.5 * (lo + hi);
    for (int i : component) {
      m.compliant[i] = side[i] > 0;
      m.friction[i] = 1.0 / (side[i] * a + offset[i]);
    }
  }
  return m;
}

}  // namespace volcontact::drake_backend
