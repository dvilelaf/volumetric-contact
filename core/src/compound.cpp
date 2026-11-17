#include <algorithm>
#include <cmath>
#include <utility>

#include "volcontact/intersection.hpp"

namespace volcontact {
namespace {

using Polygons = std::vector<std::vector<Vec3>>;

Aabb widened(Aabb box, double pad) {
  box.lo.array() -= pad;
  box.hi.array() += pad;
  return box;
}

// Appends the fan triangles of `polys` to the mesh, flagged `interior`. Slivers
// of no area are dropped: they change no integral and have no normal.
void append(const Polygons& polys, bool interior, double min_area2, TriMesh* mesh, std::vector<std::uint8_t>* flags) {
  for (const auto& poly : polys) {
    const auto base = static_cast<std::uint32_t>(mesh->vertices.size());
    mesh->vertices.insert(mesh->vertices.end(), poly.begin(), poly.end());
    for (std::uint32_t i = 1; i + 1 < poly.size(); ++i) {
      if ((poly[i] - poly[0]).cross(poly[i + 1] - poly[0]).norm() <= min_area2) continue;
      mesh->triangles.push_back({base, base + i, base + i + 1});
      flags->push_back(interior ? 1 : 0);
    }
  }
}

}  // namespace

CompoundShape CompoundShape::from_meshes(std::vector<TriMesh> meshes, double tolerance) {
  // First pass: each piece on its own, which orients it and builds its tree.
  std::vector<std::shared_ptr<const CollisionShape>> raw;
  raw.reserve(meshes.size());
  for (TriMesh& mesh : meshes) raw.push_back(std::make_shared<const CollisionShape>(std::move(mesh)));
  if (raw.size() < 2) return {std::move(raw)};

  CompoundShape out;
  out.pieces.reserve(raw.size());
  Polygons exterior, interior, rest;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const CollisionShape& piece = *raw[i];
    TriMesh mesh{piece.mesh().vertices, {}};
    std::vector<std::uint8_t> flags;
    bool any_interior = false;
    for (std::size_t t = 0; t < piece.mesh().triangles.size(); ++t) {
      const auto tri = piece.mesh().triangle(t);
      const Vec3& normal = piece.triangle_normal(t);
      const double area2 = (tri[1] - tri[0]).cross(tri[2] - tri[0]).norm();
      Aabb box;
      for (const Vec3& v : tri) box.grow(v);

      // What is left of the triangle outside every sibling is surface of the
      // body; the fragments with a sibling just in front of them are seams.
      exterior.assign(1, {tri[0], tri[1], tri[2]});
      interior.clear();
      if (!normal.hasNaN()) {
        for (std::size_t j = 0; j < raw.size() && !exterior.empty(); ++j) {
          if (j == i) continue;
          const CollisionShape& other = *raw[j];
          const double tol = std::max({tolerance, piece.plane_tolerance(), other.plane_tolerance()});
          if (!widened(other.bounds(), tol).overlaps(box)) continue;
          rest.clear();
          for (const auto& poly : exterior) {
            other.bsp().split(poly, normal, Coplanar::kInsideInFront, tol, &interior, &rest);
          }
          exterior.swap(rest);
        }
      }

      if (interior.empty() || exterior.empty()) {  // All of one kind: the triangle stays as it is.
        mesh.triangles.push_back(piece.mesh().triangles[t]);
        flags.push_back(interior.empty() ? 0 : 1);
      } else {
        append(exterior, false, 1e-12 * area2, &mesh, &flags);
        append(interior, true, 1e-12 * area2, &mesh, &flags);
      }
      any_interior |= !interior.empty();
    }
    out.pieces.push_back(any_interior ? std::make_shared<const CollisionShape>(std::move(mesh), std::move(flags))
                                      : raw[i]);
  }
  return out;
}

CompoundShape CompoundShape::from_objs(std::span<const std::filesystem::path> paths, double tolerance) {
  std::vector<TriMesh> meshes;
  meshes.reserve(paths.size());
  for (const auto& path : paths) meshes.push_back(load_obj(path));
  return from_meshes(std::move(meshes), tolerance);
}

}  // namespace volcontact
