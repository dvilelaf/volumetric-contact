#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "volcontact/mesh.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// How a polygon lying on a splitting plane (within tolerance) is classified.
// Two solids touching along a face must contribute that patch of ∂(A∩B) once or
// not at all, never twice (the consistency problem of the thesis, §4.2.1):
//  - kInsideBehind: the polygon counts where the solid is inside just behind it
//    (opposite its normal). Used for A's faces against B: a face of A that
//    coincides with a same-facing face of B is kept.
//  - kInsideBothSides: the polygon counts only where the solid is inside on both
//    sides, i.e. strictly in its interior. Used for B's faces against A, so a
//    face of B that coincides with any face of A is dropped.
//  - kInsideInFront: the polygon counts where the solid is inside just in front
//    of it (along its normal). Used between the pieces of one body: a face of a
//    piece is interior to the body where a sibling piece lies in front of it.
enum class Coplanar : std::uint8_t { kInsideBehind, kInsideBothSides, kInsideInFront };

// Scratch memory for SolidBsp::clip_inside. Reused across calls so that clipping
// does not allocate once warmed up.
class ClipWorkspace {
 public:
  ClipWorkspace() {
    pool_.reserve(1024);
    stack_.reserve(64);
  }

 private:
  friend class SolidBsp;
  struct Item {
    std::int32_t node;
    std::uint32_t begin, count;
    std::int32_t then;  // Subtree that must also accept the fragment, or kNoSubtree.
  };
  std::vector<Vec3> pool_;
  std::vector<Item> stack_;
  std::vector<double> dist_;
};

// Solid-leaf BSP tree of a closed, outward-oriented mesh, built once in the mesh
// frame. It answers point containment and splits convex polygons into the
// fragments that lie inside the solid. Planes are "thick": points within the
// tolerance of a plane are on it.
class SolidBsp {
 public:
  // `tolerance` decides which faces are coplanar while building.
  SolidBsp(const TriMesh& mesh, double tolerance);

  [[nodiscard]] bool contains(const Vec3& p) const;

  // Splits the convex polygon `poly` (outward normal `normal`) by the tree and
  // calls emit(const Vec3* vertices, int count) for every fragment inside the
  // solid. Fragments keep the input vertex order. `tol` is the plane thickness.
  template <class Emit>
  void clip_inside(std::span<const Vec3> poly, const Vec3& normal, Coplanar rule, double tol, ClipWorkspace& ws,
                   Emit&& emit) const;

  // The same split keeping both sides: the fragments inside the solid are
  // appended to `inside` and the rest to `outside`. Not for the hot path.
  void split(std::span<const Vec3> poly, const Vec3& normal, Coplanar rule, double tol,
             std::vector<std::vector<Vec3>>* inside, std::vector<std::vector<Vec3>>* outside) const;

  [[nodiscard]] std::size_t num_nodes() const { return nodes_.size(); }
  [[nodiscard]] double tolerance() const { return tol_; }

 private:
  static constexpr std::int32_t kInside = -1;
  static constexpr std::int32_t kOutside = -2;
  static constexpr std::int32_t kNoSubtree = -3;

  struct Node {
    Plane plane;
    std::int32_t front, back;  // Child node index, or kInside / kOutside.
    bool mixed;                // The mesh has faces of both orientations on this plane.
  };

  std::int32_t build(std::vector<std::vector<Vec3>> polys, std::vector<Plane> planes);
  void split_at(std::int32_t node, std::vector<Vec3> poly, const Vec3& normal, Coplanar rule, double tol,
                std::vector<std::vector<Vec3>>* inside, std::vector<std::vector<Vec3>>* outside) const;
  static std::uint32_t clip_to_side(const Vec3* in, const double* dist, std::uint32_t n, double side, double tol,
                                    Vec3* out);

  std::vector<Node> nodes_;
  double tol_;
};

// --------------------------------------------------------------------------
// Implementation of the hot path.

template <class Emit>
void SolidBsp::clip_inside(std::span<const Vec3> poly, const Vec3& normal, Coplanar rule, double tol,
                           ClipWorkspace& ws, Emit&& emit) const {
  auto& pool = ws.pool_;
  auto& stack = ws.stack_;
  auto& dist = ws.dist_;
  pool.assign(poly.begin(), poly.end());
  stack.clear();
  stack.push_back({0, 0, static_cast<std::uint32_t>(poly.size()), kNoSubtree});

  while (!stack.empty()) {
    ClipWorkspace::Item item = stack.back();
    stack.pop_back();

    // Descend while the polygon stays on one side; split only when it straddles.
    for (;;) {
      if (item.node == kInside && item.then != kNoSubtree) {
        item.node = item.then;
        item.then = kNoSubtree;
      }
      if (item.node < 0) break;

      const Node& node = nodes_[item.node];
      const std::uint32_t n = item.count;
      if (dist.size() < n) dist.resize(n);
      const Vec3* v = pool.data() + item.begin;
      bool any_front = false, any_back = false;
      for (std::uint32_t i = 0; i < n; ++i) {
        const double d = node.plane.distance(v[i]);
        dist[i] = d;
        any_front |= d > tol;
        any_back |= d < -tol;
      }

      if (!any_front && !any_back) {  // On the plane.
        if (rule == Coplanar::kInsideBehind) {
          item.node = normal.dot(node.plane.n) > 0.0 ? node.back : node.front;
        } else if (rule == Coplanar::kInsideInFront) {
          item.node = normal.dot(node.plane.n) > 0.0 ? node.front : node.back;
        } else if (!node.mixed) {
          // Every face on this plane faces the front, so inside in front implies inside behind.
          item.node = node.front;
        } else {
          item.node = node.front;
          item.then = node.back;
        }
        continue;
      }
      if (!any_back) {
        item.node = node.front;
        continue;
      }
      if (!any_front) {
        item.node = node.back;
        continue;
      }

      // Straddles: build only the halves that can still reach an inside leaf.
      // Each half has at most n + 2 vertices.
      const bool keep_front = node.front != kOutside;
      const auto base = static_cast<std::uint32_t>(pool.size());
      pool.resize(base + (keep_front ? 2 : 1) * (n + 2));
      const Vec3* in = pool.data() + item.begin;
      Vec3* out = pool.data() + base;
      const std::uint32_t num_front = keep_front ? clip_to_side(in, dist.data(), n, 1.0, tol, out) : 0;
      const std::uint32_t num_back = clip_to_side(in, dist.data(), n, -1.0, tol, out + num_front);
      pool.resize(base + num_front + num_back);

      if (num_front >= 3) stack.push_back({node.front, base, num_front, item.then});
      if (num_back < 3) {
        item.node = kOutside;
        break;
      }
      item = {node.back, base + num_front, num_back, item.then};
    }
    if (item.node == kInside) emit(pool.data() + item.begin, static_cast<int>(item.count));
  }
}

inline std::uint32_t SolidBsp::clip_to_side(const Vec3* in, const double* dist, std::uint32_t n, double side,
                                            double tol, Vec3* out) {
  // Sutherland–Hodgman against one side of the plane. Vertices on the plane
  // (within tolerance) belong to both sides.
  std::uint32_t count = 0;
  for (std::uint32_t i = 0; i < n; ++i) {
    const std::uint32_t j = (i + 1 == n) ? 0 : i + 1;
    const double di = dist[i], dj = dist[j];
    if (side * di >= -tol) out[count++] = in[i];
    if ((di > tol && dj < -tol) || (di < -tol && dj > tol)) out[count++] = in[i] + (di / (di - dj)) * (in[j] - in[i]);
  }
  return count;
}

}  // namespace volcontact
