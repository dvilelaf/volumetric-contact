#pragma once

#include <Eigen/Geometry>

#include "volcontact/mesh.hpp"

namespace volcontact::test {

// Axis-aligned box [lo, hi] with outward, counter-clockwise triangles.
inline TriMesh make_box(const Vec3& lo, const Vec3& hi) {
  TriMesh m;
  for (int i = 0; i < 8; ++i) m.vertices.emplace_back(i & 1 ? hi.x() : lo.x(), i & 2 ? hi.y() : lo.y(), i & 4 ? hi.z() : lo.z());
  const std::uint32_t quads[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
  for (const auto& q : quads) {
    m.triangles.push_back({q[0], q[1], q[2]});
    m.triangles.push_back({q[0], q[2], q[3]});
  }
  return m;
}

// Non-convex "step" solid: the profile below in the xz plane, extruded along
// y ∈ [0, 1]. The plane z = 1 holds an up-facing face (x ∈ [0, 1]) and a
// down-facing face (x ∈ [2, 3]) of the same mesh.
//
//   z=2        +-------+
//              |       |
//   z=1  +-----+   +---+
//        |         |
//   z=0  +---------+
//        x=0   1   2   3
inline TriMesh make_step() {
  const double px[8] = {0, 2, 2, 3, 3, 1, 1, 0}, pz[8] = {0, 0, 1, 1, 2, 2, 1, 1};
  TriMesh m;
  for (int side = 0; side < 2; ++side) {
    for (int i = 0; i < 8; ++i) m.vertices.emplace_back(px[i], side, pz[i]);
  }
  // Profile triangles (counter-clockwise seen from -y, i.e. outward for y = 0).
  const std::uint32_t caps[6][3] = {{0, 1, 2}, {0, 2, 6}, {0, 6, 7}, {2, 3, 4}, {2, 4, 5}, {2, 5, 6}};
  for (const auto& t : caps) {
    m.triangles.push_back({t[0], t[2], t[1]});          // y = 0, normal -y.
    m.triangles.push_back({t[0] + 8, t[1] + 8, t[2] + 8});  // y = 1, normal +y.
  }
  for (std::uint32_t i = 0; i < 8; ++i) {
    const std::uint32_t j = (i + 1) % 8;
    m.triangles.push_back({i, j, j + 8});
    m.triangles.push_back({i, j + 8, i + 8});
  }
  return m;
}

inline Mat3 rot(double angle, const Vec3& axis) { return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix(); }

}  // namespace volcontact::test
