#include "volcontact/intersection.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "volcontact/detail/volume_integrator.hpp"

namespace volcontact {
namespace {

TriMesh outward(TriMesh mesh) {
  const double v = signed_volume(mesh);
  const double scale = mesh.bounds().half_extent().norm();
  if (!(std::abs(v) > 1e-12 * scale * scale * scale)) {
    throw std::invalid_argument("CollisionShape: mesh encloses no volume (is it closed?)");
  }
  if (v < 0.0) flip_orientation(mesh);
  return mesh;
}

std::vector<Vec3> unit_normals(const TriMesh& mesh) {
  std::vector<Vec3> normals(mesh.triangles.size());
  for (std::size_t t = 0; t < normals.size(); ++t) {
    const auto v = mesh.triangle(t);
    normals[t] = (v[1] - v[0]).cross(v[2] - v[0]).normalized();
  }
  return normals;
}

// Plane thickness relative to the size of the shape.
double bsp_tolerance(const Aabb& bounds) { return 1e-10 * (bounds.hi - bounds.lo).norm(); }

Aabb triangle_box(const Vec3 (&v)[3]) {
  return {v[0].cwiseMin(v[1]).cwiseMin(v[2]), v[0].cwiseMax(v[1]).cwiseMax(v[2])};
}

// Distinct face planes of a convex mesh, or none if the mesh is not convex.
// `triangle_plane` receives the index of each triangle's plane.
std::vector<Plane> convex_planes(const TriMesh& mesh, const std::vector<Vec3>& normals, double tol,
                                 std::vector<std::uint32_t>* triangle_plane) {
  std::vector<Plane> planes;
  triangle_plane->assign(mesh.triangles.size(), 0);
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    if (normals[t].hasNaN()) continue;  // Degenerate triangle.
    const Plane p{normals[t], normals[t].dot(mesh.vertices[mesh.triangles[t][0]])};
    const auto known = std::find_if(planes.begin(), planes.end(), [&](const Plane& q) {
      return q.n.dot(p.n) > 1.0 - 1e-9 && std::abs(q.d - p.d) <= tol;
    });
    (*triangle_plane)[t] = static_cast<std::uint32_t>(known - planes.begin());
    if (known == planes.end()) planes.push_back(p);
  }
  for (const Plane& p : planes) {
    for (const Vec3& v : mesh.vertices) {
      if (p.distance(v) > tol) {
        triangle_plane->clear();
        return {};
      }
    }
  }
  return planes;
}

// Unit vector along a vector area (given doubled) when it is well conditioned
// for a region of volume `volume`, else zero.
Vec3 conditioned_normal(const Vec3& twice_area, double volume) {
  const double s2 = 0.25 * twice_area.squaredNorm();
  constexpr double kMinRatio = 0.1;  // |S| / V^(2/3); a compact overlap bounded by B on one side has about 2 or more.
  if (!(s2 > 0.0) || s2 < kMinRatio * kMinRatio * std::pow(volume, 4.0 / 3.0)) return Vec3::Zero();
  return twice_area.normalized();
}

Aabb widened(Aabb box, double pad) {
  box.lo.array() -= pad;
  box.hi.array() += pad;
  return box;
}

Vec3 volume_centroid(const TriMesh& mesh, const Aabb& bounds) {
  VolumeIntegrator integrator;
  const Vec3 origin = bounds.center();
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto v = mesh.triangle(t);
    integrator.add_triangle(v[0] - origin, v[1] - origin, v[2] - origin);
  }
  return integrator.result().centroid + origin;
}

// Does the segment from a to b touch the box?
bool segment_touches(const Aabb& box, const Vec3& a, const Vec3& b) {
  double t0 = 0.0, t1 = 1.0;
  for (int k = 0; k < 3; ++k) {
    const double d = b[k] - a[k];
    if (d == 0.0) {
      if (a[k] < box.lo[k] || a[k] > box.hi[k]) return false;
      continue;
    }
    double u0 = (box.lo[k] - a[k]) / d, u1 = (box.hi[k] - a[k]) / d;
    if (u0 > u1) std::swap(u0, u1);
    t0 = std::max(t0, u0);
    t1 = std::min(t1, u1);
    if (t0 > t1) return false;
  }
  return true;
}

// SolidBsp::clip_inside for a convex shape, with the same rules and the same
// result. The tree of a convex mesh is a chain of all its faces, and a polygon
// inside walks all of it; here it is cut only by the faces near it:
//  - The polygon P is first cut to `region`, the box where the overlap can be,
//    with a margin, so that the cuts it leaves fall clearly outside C.
//  - For convex P and solid C that meet, P ∩ C is P cut by the planes of the
//    faces of C that touch P: a point of P outside C is joined to one inside by
//    a segment of P that leaves C through such a face, and lies beyond its
//    plane. The faces whose boxes overlap P's are a superset of them.
//  - If P and C do not meet, what is left of P is outside C, which one point
//    tells: a point is outside C when it lies beyond the plane of a face that
//    the segment from the centroid of C to it goes through.
template <class Emit>
void clip_inside_convex(const CollisionShape& s, std::span<const Vec3> poly, const Vec3& normal, Coplanar rule,
                        double tol, const Aabb& region, IntersectionWorkspace& ws, Emit&& emit) {
  auto& cur = ws.polygon;
  auto& out = ws.clipped;
  auto& dist = ws.distance;
  cur.assign(poly.begin(), poly.end());
  // Keeps the side of `cur` where the distances are at most `keep` (crossings where they change sign past ±cross).
  const auto cut = [&](double keep, double cross) {
    out.clear();
    const std::size_t n = cur.size();
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t j = i + 1 == n ? 0 : i + 1;
      const double di = dist[i], dj = dist[j];
      if (di <= keep) out.push_back(cur[i]);
      if ((di > cross && dj < -cross) || (di < -cross && dj > cross)) {
        out.push_back(cur[i] + (di / (di - dj)) * (cur[j] - cur[i]));
      }
    }
    cur.swap(out);
  };

  const double margin = 0.05 * (region.hi - region.lo).norm() + 100.0 * tol;
  for (int k = 0; k < 3; ++k) {
    for (const double side : {-1.0, 1.0}) {
      const double bound = (side < 0.0 ? -region.lo[k] : region.hi[k]) + margin;
      dist.resize(cur.size());
      bool beyond = false;
      for (std::size_t i = 0; i < cur.size(); ++i) {
        dist[i] = side * cur[i][k] - bound;
        beyond |= dist[i] > 0.0;
      }
      if (!beyond) continue;
      cut(0.0, 0.0);
      if (cur.size() < 3) return;
    }
  }

  Aabb box;
  for (const Vec3& v : cur) box.grow(v);
  // One triangle per face plane.
  ws.faces.clear();
  if (ws.plane_seen.size() < s.face_planes().size()) ws.plane_seen.resize(s.face_planes().size());
  const std::uint64_t tick = ++ws.tick;
  s.bvh().for_each_overlapping(widened(box, tol), [&](std::uint32_t t) {
    std::uint64_t& seen = ws.plane_seen[s.triangle_plane(t)];
    if (seen == tick || s.triangle_normal(t).hasNaN()) return;
    seen = tick;
    ws.faces.push_back(t);
  });
  const auto& mesh = s.mesh();
  for (const std::uint32_t t : ws.faces) {
    const Vec3& n = s.triangle_normal(t);
    const double d = n.dot(mesh.vertices[mesh.triangles[t][0]]);
    dist.resize(cur.size());
    bool any_front = false, any_back = false;
    for (std::size_t i = 0; i < cur.size(); ++i) {
      dist[i] = n.dot(cur[i]) - d;
      any_front |= dist[i] > tol;
      any_back |= dist[i] < -tol;
    }
    if (!any_front && !any_back) {  // On the plane of the face: the rule decides (see Coplanar).
      const bool same = normal.dot(n) > 0.0;
      const bool inside = rule == Coplanar::kInsideBehind ? same : rule == Coplanar::kInsideInFront ? !same : false;
      if (!inside) return;
      continue;
    }
    if (!any_front) continue;
    if (!any_back) return;
    cut(tol, tol);  // Like SolidBsp::clip_to_side: points on the thick plane stay.
    if (cur.size() < 3) return;
  }

  Vec3 point = Vec3::Zero();
  for (const Vec3& v : cur) point += v;
  point /= static_cast<double>(cur.size());
  const Vec3& centre = s.centroid();
  bool outside = false;
  s.bvh().for_each_accepted(
      [&](const Aabb& b) { return segment_touches(widened(b, tol), centre, point); },
      [&](std::uint32_t t) {
        const Vec3& n = s.triangle_normal(t);
        if (!n.hasNaN() && n.dot(point - mesh.vertices[mesh.triangles[t][0]]) > tol) outside = true;
      });
  if (!outside) emit(cur.data(), static_cast<int>(cur.size()));
}

// The fragments of a polygon inside a shape, both in the shape's frame.
template <class Emit>
void clip_inside(const CollisionShape& s, std::span<const Vec3> poly, const Vec3& normal, Coplanar rule, double tol,
                 const Aabb& region, IntersectionWorkspace& ws, Emit&& emit) {
  if (s.convex() && !ws.bsp_only && s.face_planes().size() > CollisionShape::kFewFaces) {
    clip_inside_convex(s, poly, normal, rule, tol, region, ws, emit);
  } else {
    s.bsp().clip_inside(poly, normal, rule, tol, ws.clip, emit);
  }
}

}  // namespace

CollisionShape::CollisionShape(TriMesh mesh, std::vector<std::uint8_t> interior)
    : mesh_(outward(std::move(mesh))),
      bounds_(mesh_.bounds()),
      centroid_(volume_centroid(mesh_, bounds_)),
      normals_(unit_normals(mesh_)),
      bvh_(mesh_),
      bsp_(mesh_, bsp_tolerance(bounds_)),
      // OBJ files usually carry ~6 significant digits; accept that much non-planarity.
      planes_(convex_planes(mesh_, normals_, 1e-5 * (bounds_.hi - bounds_.lo).norm(), &triangle_plane_)),
      interior_(std::move(interior)) {
  if (!interior_.empty() && interior_.size() != mesh_.triangles.size()) {
    throw std::invalid_argument("CollisionShape: one interior flag per triangle expected");
  }
  plane_exterior_.assign(planes_.size(), interior_.empty() ? 1 : 0);
  if (!interior_.empty() && !planes_.empty()) {
    for (std::size_t t = 0; t < interior_.size(); ++t) {
      if (!interior_[t] && !normals_[t].hasNaN()) plane_exterior_[triangle_plane_[t]] = 1;
    }
  }
}

std::shared_ptr<const CollisionShape> CollisionShape::from_obj(const std::filesystem::path& path) {
  return std::make_shared<const CollisionShape>(load_obj(path));
}

bool intersect(const CollisionShape& a, const Pose& X_WA, const CollisionShape& b, const Pose& X_WB,
               IntersectionWorkspace& ws, IntersectionProperties* out, ContactSurface* surface) {
  const Pose X_AB = X_WA.inverse() * X_WB;
  const Pose X_BA = X_AB.inverse();
  // One plane thickness for both passes, so they agree on what is coplanar.
  const double tol = std::max(a.bsp().tolerance(), b.bsp().tolerance());

  // Region where V can be, as a box in each frame. Faces outside it are skipped.
  Aabb region_A = a.bounds().intersection(b.bounds().transformed(X_AB));
  if (region_A.empty()) return false;
  Aabb region_B = b.bounds().intersection(region_A.transformed(X_BA));
  if (region_B.empty()) return false;
  region_A = region_A.intersection(region_B.transformed(X_AB));
  if (region_A.empty()) return false;
  region_A = widened(region_A, tol);
  region_B = widened(region_B, tol);

  // Integrate in A's frame about a point inside the region, which keeps the
  // second moments well conditioned for thin intersection volumes.
  const Vec3 origin = region_A.center();
  if (surface) surface->clear();
  // Appends a polygon given in frame F (pose X_WF), with the outward normal of
  // its face in world.
  const auto record = [&](const Vec3* v, int n, const Pose& X_WF, const Vec3& normal_W, const CollisionShape& s,
                          std::uint32_t t, bool on_a) {
    const auto begin = static_cast<std::uint32_t>(surface->vertices.size());
    for (int i = 0; i < n; ++i) surface->vertices.push_back(X_WF.apply(v[i]));
    surface->polygons.push_back({begin, static_cast<std::uint32_t>(n), normal_W, on_a, s.triangle_interior(t),
                                 s.convex() ? s.triangle_plane(t) : 0});
  };
  VolumeIntegrator integrator;
  Vec3 area_a = Vec3::Zero(), area_b = Vec3::Zero();  // Twice the vector areas.
  Vec3 area_b_exterior = Vec3::Zero();                // Of B's faces that are not seams.
  auto& frag = ws.fragment;

  // Part of ∂V on A: faces of A inside B, clipped in B's frame.
  const auto& va = a.mesh().vertices;
  a.bvh().for_each_overlapping(region_A, [&](std::uint32_t t) {
    const auto& f = a.mesh().triangles[t];
    const Vec3 tri[3] = {X_BA.apply(va[f[0]]), X_BA.apply(va[f[1]]), X_BA.apply(va[f[2]])};
    if (!triangle_box(tri).overlaps(region_B)) return;  // The A-frame box is loose once A rotates.
    clip_inside(b, tri, X_BA.R * a.triangle_normal(t), Coplanar::kInsideBehind, tol, region_B, ws,
                [&](const Vec3* v, int n) {
                  frag.resize(n);
                  for (int i = 0; i < n; ++i) frag[i] = X_AB.apply(v[i]) - origin;
                  area_a += integrator.add_polygon(frag.data(), n);
                  if (surface) record(v, n, X_WB, X_WA.R * a.triangle_normal(t), a, t, true);
                });
  });

  // Part of ∂V on B: faces of B strictly inside A, clipped in A's frame.
  const auto& vb = b.mesh().vertices;
  b.bvh().for_each_overlapping(region_B, [&](std::uint32_t t) {
    const auto& f = b.mesh().triangles[t];
    const Vec3 tri[3] = {X_AB.apply(vb[f[0]]), X_AB.apply(vb[f[1]]), X_AB.apply(vb[f[2]])};
    if (!triangle_box(tri).overlaps(region_A)) return;
    clip_inside(a, tri, X_AB.R * b.triangle_normal(t), Coplanar::kInsideBothSides, tol, region_A, ws,
                [&](const Vec3* v, int n) {
                  frag.resize(n);
                  for (int i = 0; i < n; ++i) frag[i] = v[i] - origin;
                  const Vec3 area = integrator.add_polygon(frag.data(), n);
                  area_b += area;
                  if (!b.triangle_interior(t)) area_b_exterior += area;
                  if (surface) record(v, n, X_WA, X_WB.R * b.triangle_normal(t), b, t, false);
                });
  });

  const MassProperties mp = integrator.result();
  if (!(mp.volume > 0.0)) return false;

  out->volume = mp.volume;
  out->centroid = X_WA.apply(mp.centroid + origin);
  out->inertia = X_WA.R * mp.inertia * X_WA.R.transpose();
  // For a closed ∂V the two parts have opposite vector areas; their difference is twice B's.
  out->normal = X_WA.R * conditioned_normal(0.5 * (area_b - area_a), mp.volume);
  out->exterior_normal = X_WA.R * conditioned_normal(area_b_exterior, mp.volume);
  return true;
}

Vec3 escape_direction(const CollisionShape& b, const Pose& X_WB, const Vec3& point) {
  const Vec3 p_B = X_WB.inverse().apply(point);
  Vec3 n_B = Vec3::Zero();
  if (b.convex()) {
    // Nearest face first among those of the body's surface, then among all.
    for (const bool exterior_only : {true, false}) {
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < b.face_planes().size(); ++i) {
        if (exterior_only && !b.plane_exterior(i)) continue;
        const double depth = -b.face_planes()[i].distance(p_B);
        if (depth < best) {
          best = depth;
          n_B = b.face_planes()[i].n;
        }
      }
      if (!n_B.isZero()) break;
    }
  } else {
    n_B = p_B - b.bounds().center();
  }
  const double norm = n_B.norm();
  return norm > 0.0 ? Vec3(X_WB.R * (n_B / norm)) : Vec3::UnitZ();
}

}  // namespace volcontact
