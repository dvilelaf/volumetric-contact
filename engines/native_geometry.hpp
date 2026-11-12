#pragma once

// Geometry helpers shared by the backends' native contact (docs/scenarios.md §3).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "volcontact/mesh.hpp"
#include "volcontact/types.hpp"

namespace volcontact::native {

inline double surface_area(const TriMesh& mesh) {
  double area = 0.0;
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto [a, b, c] = mesh.triangle(t);
    area += 0.5 * (b - a).cross(c - a).norm();
  }
  return area;
}

// Reference area of a solid for models whose stiffness is per contact point
// [N/m] instead of per area: a sixth of its surface (the face of a cube, the
// mean face of a box). K·A is then the stiffness of a Winkler foundation K
// under a face of that size.
inline double reference_area(const TriMesh& surface) { return surface_area(surface) / 6.0; }

// Largest side of the mesh's bounding box.
inline double largest_extent(const TriMesh& mesh) {
  Vec3 lo = mesh.vertices.front(), hi = lo;
  for (const Vec3& v : mesh.vertices) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  return (hi - lo).maxCoeff();
}

// The same surface with every edge at most `max_edge` long: each pass splits
// every longer edge at its midpoint (1, 2 or 3 per triangle), so neighbours
// stay conforming and long thin triangles are cut across, not into slivers.
inline TriMesh refined(TriMesh mesh, double max_edge) {
  using Edge = std::pair<std::uint32_t, std::uint32_t>;
  for (;;) {
    std::map<Edge, std::uint32_t> midpoint;
    const auto mid = [&](std::uint32_t a, std::uint32_t b) -> std::optional<std::uint32_t> {
      if ((mesh.vertices[a] - mesh.vertices[b]).norm() <= max_edge) return std::nullopt;
      const Edge key{std::min(a, b), std::max(a, b)};
      const auto [it, fresh] = midpoint.try_emplace(key, static_cast<std::uint32_t>(mesh.vertices.size()));
      if (fresh) mesh.vertices.push_back(0.5 * (mesh.vertices[a] + mesh.vertices[b]));
      return it->second;
    };
    std::vector<std::array<std::uint32_t, 3>> triangles;
    triangles.reserve(mesh.triangles.size() * 2);
    for (const auto& f : mesh.triangles) {
      const std::optional<std::uint32_t> m[3] = {mid(f[0], f[1]), mid(f[1], f[2]), mid(f[2], f[0])};
      const int count = (m[0] ? 1 : 0) + (m[1] ? 1 : 0) + (m[2] ? 1 : 0);
      if (count == 0) {
        triangles.push_back(f);
      } else if (count == 3) {
        triangles.push_back({f[0], *m[0], *m[2]});
        triangles.push_back({*m[0], f[1], *m[1]});
        triangles.push_back({*m[2], *m[1], f[2]});
        triangles.push_back({*m[0], *m[1], *m[2]});
      } else {
        // Rotate so that edge (a, b) is split; with two, (b, c) is the other one.
        int k = 0;
        while (!(m[k] && (count == 1 || m[(k + 1) % 3]))) ++k;
        const std::uint32_t a = f[k], b = f[(k + 1) % 3], c = f[(k + 2) % 3], ab = *m[k];
        if (count == 1) {
          triangles.push_back({a, ab, c});
          triangles.push_back({ab, b, c});
        } else {
          const std::uint32_t bc = *m[(k + 1) % 3];
          triangles.push_back({a, ab, c});
          triangles.push_back({ab, bc, c});
          triangles.push_back({ab, b, bc});
        }
      }
    }
    if (midpoint.empty()) return mesh;
    mesh.triangles = std::move(triangles);
  }
}

// Wavefront OBJ text of the mesh, with 17 significant digits.
inline std::string to_obj(const TriMesh& mesh) {
  std::ostringstream out;
  out.precision(17);
  for (const Vec3& v : mesh.vertices) out << "v " << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
  for (const auto& f : mesh.triangles) out << "f " << f[0] + 1 << ' ' << f[1] + 1 << ' ' << f[2] + 1 << '\n';
  return out.str();
}

// The box a convex piece is, if it is one: 8 vertices that are the corners of
// a (possibly rotated) box. Engines collide boxes exactly and with full manifolds.
struct OrientedBox {
  Vec3 size;  // Side lengths.
  Pose X;     // Box frame (centre, axes along the sides) in the mesh frame.
};

inline std::optional<OrientedBox> as_box(const TriMesh& mesh) {
  if (mesh.vertices.size() != 8) return std::nullopt;
  Vec3 centre = Vec3::Zero();
  for (const Vec3& v : mesh.vertices) centre += v / 8.0;
  const double scale = (mesh.vertices[0] - centre).norm();
  // The edges at a corner: three mutually orthogonal vectors to other vertices
  // such that every corner o + a·e0 + b·e1 + c·e2, a, b, c ∈ {0, 1}, is a vertex.
  const Vec3& o = mesh.vertices[0];
  const double tol = 1e-9 * scale;
  const auto is_vertex = [&](const Vec3& x) {
    return std::any_of(mesh.vertices.begin(), mesh.vertices.end(),
                       [&](const Vec3& v) { return (v - x).norm() < tol; });
  };
  Vec3 e[3];
  bool found = false;
  for (int i = 1; i < 8 && !found; ++i) {
    for (int j = i + 1; j < 8 && !found; ++j) {
      for (int k = j + 1; k < 8 && !found; ++k) {
        e[0] = mesh.vertices[i] - o;
        e[1] = mesh.vertices[j] - o;
        e[2] = mesh.vertices[k] - o;
        found = std::abs(e[0].dot(e[1])) < tol * scale && std::abs(e[1].dot(e[2])) < tol * scale &&
                std::abs(e[2].dot(e[0])) < tol * scale && is_vertex(o + e[0] + e[1]) && is_vertex(o + e[1] + e[2]) &&
                is_vertex(o + e[2] + e[0]) && is_vertex(o + e[0] + e[1] + e[2]);
      }
    }
  }
  if (!found) return std::nullopt;
  if (e[0].cross(e[1]).dot(e[2]) < 0.0) std::swap(e[0], e[1]);
  OrientedBox box;
  box.size = Vec3(e[0].norm(), e[1].norm(), e[2].norm());
  for (int k = 0; k < 3; ++k) box.X.R.col(k) = e[k].normalized();
  box.X.p = centre;
  return box;
}

}  // namespace volcontact::native
