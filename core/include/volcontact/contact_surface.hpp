#pragma once

#include <cstdint>
#include <vector>

#include "volcontact/types.hpp"

namespace volcontact {

// The boundary of V = A ∩ B as produced by clipping, in world: convex polygons
// that are pieces of A's faces inside B, and pieces of B's faces inside A.
struct ContactSurface {
  struct Polygon {
    std::uint32_t begin = 0, count = 0;  // Range in `vertices`.
    Vec3 normal = Vec3::Zero();          // Outward normal of the face it comes from.
    bool on_a = true;                    // Piece of A's surface (inside B), else of B's.
    bool interior = false;               // On a seam between pieces of its body: not contact surface.
    std::uint32_t plane = 0;             // Index of its face plane in its (convex) shape.
  };

  std::vector<Vec3> vertices;
  std::vector<Polygon> polygons;

  void clear() {
    vertices.clear();
    polygons.clear();
  }
};

}  // namespace volcontact
