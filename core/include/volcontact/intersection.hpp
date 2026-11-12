#pragma once

#include <algorithm>
#include <filesystem>
#include <limits>
#include <memory>
#include <span>
#include <vector>

#include "volcontact/detail/bsp.hpp"
#include "volcontact/detail/bvh.hpp"
#include "volcontact/contact_surface.hpp"
#include "volcontact/intersection_properties.hpp"
#include "volcontact/mesh.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// Immutable collision geometry of a rigid body: a closed mesh in the body frame
// plus the acceleration structures built from it. Shareable between pairs.
class CollisionShape {
 public:
  // Takes a closed mesh; fixes its orientation if it is inside-out. Throws if the
  // mesh encloses no volume. `interior` flags, per triangle, the faces that lie
  // inside the body the shape is a piece of (see CompoundShape); empty for none.
  explicit CollisionShape(TriMesh mesh, std::vector<std::uint8_t> interior = {});
  static std::shared_ptr<const CollisionShape> from_obj(const std::filesystem::path& path);

  [[nodiscard]] const TriMesh& mesh() const { return mesh_; }
  [[nodiscard]] const Bvh& bvh() const { return bvh_; }
  [[nodiscard]] const SolidBsp& bsp() const { return bsp_; }
  [[nodiscard]] const Aabb& bounds() const { return bounds_; }
  [[nodiscard]] const Vec3& centroid() const { return centroid_; }  // Of the enclosed volume.
  [[nodiscard]] const Vec3& triangle_normal(std::size_t t) const { return normals_[t]; }
  // The triangle is a seam: it lies inside the body, against a sibling piece.
  [[nodiscard]] bool triangle_interior(std::size_t t) const { return !interior_.empty() && interior_[t] != 0; }

  // Convex shapes know their distinct face planes (outward normals).
  [[nodiscard]] bool convex() const { return !planes_.empty(); }
  [[nodiscard]] const std::vector<Plane>& face_planes() const { return planes_; }
  // Convex shapes: index in face_planes() of the plane of a triangle, and whether
  // any part of the face on a plane belongs to the surface of the body.
  [[nodiscard]] std::uint32_t triangle_plane(std::size_t t) const { return triangle_plane_[t]; }
  [[nodiscard]] bool plane_exterior(std::size_t i) const { return plane_exterior_[i] != 0; }
  [[nodiscard]] bool has_seams() const { return !interior_.empty(); }
  // Up to this many face planes, going through all of them costs less than
  // finding the ones near a polygon (measured: a 64-segment cylinder, 66 faces,
  // is twice as fast with all; a 64-segment sphere, 2048, six times slower).
  static constexpr std::size_t kFewFaces = 200;
  [[nodiscard]] double plane_tolerance() const { return bsp_.tolerance(); }

 private:
  TriMesh mesh_;
  Aabb bounds_;
  Vec3 centroid_;
  std::vector<Vec3> normals_;  // Unit outward normal per triangle.
  Bvh bvh_;
  SolidBsp bsp_;
  std::vector<std::uint32_t> triangle_plane_;  // Filled while planes_ is built.
  std::vector<Plane> planes_;                  // Distinct face planes if convex, else empty.
  std::vector<std::uint8_t> interior_, plane_exterior_;
};

// A rigid body given as convex pieces that add up to it without overlapping,
// all in the body frame. Building it finds the seams: the parts of a piece's
// faces that lie against a sibling piece, hence inside the body. They are found
// geometrically (each face is clipped against the other pieces, so partial
// overlaps of coplanar faces are handled; such faces are re-triangulated) and
// flagged in the pieces (CollisionShape::triangle_interior). Seams bound the
// overlap of a piece with another body, so they stay in its volume integrals,
// but they are not contact surface: the distributed law puts no pressure there.
struct CompoundShape {
  std::vector<std::shared_ptr<const CollisionShape>> pieces;

  // `tolerance` is the distance within which faces of two pieces count as
  // coplanar; by default the plane thickness of the pieces (1e-10 of their
  // size), which suits pieces written with full precision.
  static CompoundShape from_meshes(std::vector<TriMesh> meshes, double tolerance = 0.0);
  static CompoundShape from_objs(std::span<const std::filesystem::path> paths, double tolerance = 0.0);
};

// Scratch memory reused across intersection queries.
struct IntersectionWorkspace {
  ClipWorkspace clip;
  std::vector<Vec3> fragment, polygon, clipped;
  std::vector<double> distance;
  std::vector<std::uint32_t> faces;
  std::vector<std::uint64_t> plane_seen;  // Per face plane: last tick it was taken.
  std::uint64_t tick = 0;
  // For tests: clip against convex shapes with their BSP tree, like non-convex
  // ones, instead of only against their faces near the polygon. Same result.
  bool bsp_only = false;
};

// Computes the exact intersection of two closed meshes. Returns false when the
// solids do not overlap (or the overlap is numerically empty). If `surface` is
// given, it also receives the polygons of ∂(A∩B).
bool intersect(const CollisionShape& a, const Pose& X_WA, const CollisionShape& b, const Pose& X_WB,
               IntersectionWorkspace& ws, IntersectionProperties* out, ContactSurface* surface = nullptr);

// Direction that pushes a body out of B when the overlap itself defines none
// (intersect() returned a zero normal and there is no earlier normal to keep):
// the outward normal of the face of a convex B nearest to `point` (a face of the
// body's surface if B is a piece of it), i.e. the shortest way out; for a
// non-convex B, the direction from the centre of its box to `point`. In world.
[[nodiscard]] Vec3 escape_direction(const CollisionShape& b, const Pose& X_WB, const Vec3& point);

}  // namespace volcontact
