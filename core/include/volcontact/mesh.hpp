#pragma once

#include <array>
#include <filesystem>
#include <vector>

#include "volcontact/types.hpp"

namespace volcontact {

// Plane {x : n·x = d} with unit normal n.
struct Plane {
  Vec3 n = Vec3::UnitZ();
  double d = 0.0;

  [[nodiscard]] double distance(const Vec3& x) const { return n.dot(x) - d; }
};

// Closed triangle mesh with outward-facing, counter-clockwise triangles.
struct TriMesh {
  std::vector<Vec3> vertices;
  std::vector<std::array<std::uint32_t, 3>> triangles;

  [[nodiscard]] std::array<Vec3, 3> triangle(std::size_t t) const {
    const auto& f = triangles[t];
    return {vertices[f[0]], vertices[f[1]], vertices[f[2]]};
  }
  [[nodiscard]] Aabb bounds() const;
};

// Reads the vertices and faces of a Wavefront OBJ file. Polygons are
// fan-triangulated; normals, texture coordinates and materials are ignored.
[[nodiscard]] TriMesh load_obj(const std::filesystem::path& path);

// Signed volume enclosed by the mesh (positive for outward orientation).
[[nodiscard]] double signed_volume(const TriMesh& mesh);

// Flips every triangle (reverses the orientation of the surface).
void flip_orientation(TriMesh& mesh);

}  // namespace volcontact
