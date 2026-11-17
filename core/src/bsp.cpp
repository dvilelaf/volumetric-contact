#include "volcontact/detail/bsp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace volcontact {
namespace {

enum class Side { kFront, kBack, kCoplanar, kStraddle };

Side classify(const std::vector<Vec3>& v, const Plane& plane, double tol) {
  bool front = false, back = false;
  for (const Vec3& p : v) {
    const double d = plane.distance(p);
    front |= d > tol;
    back |= d < -tol;
  }
  if (front && back) return Side::kStraddle;
  if (front) return Side::kFront;
  if (back) return Side::kBack;
  return Side::kCoplanar;
}

void split_polygon(const std::vector<Vec3>& v, const Plane& plane, double tol, std::vector<Vec3>& front,
           std::vector<Vec3>& back) {
  const std::size_t n = v.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec3& a = v[i];
    const Vec3& b = v[(i + 1) % n];
    const double da = plane.distance(a), db = plane.distance(b);
    if (da >= -tol) front.push_back(a);
    if (da <= tol) back.push_back(a);
    if ((da > tol && db < -tol) || (da < -tol && db > tol)) {
      const Vec3 x = a + (da / (da - db)) * (b - a);
      front.push_back(x);
      back.push_back(x);
    }
  }
}

}  // namespace

SolidBsp::SolidBsp(const TriMesh& mesh, double tolerance) : tol_(tolerance) {
  std::vector<std::vector<Vec3>> polys;
  std::vector<Plane> planes;
  polys.reserve(mesh.triangles.size());
  planes.reserve(mesh.triangles.size());
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto v = mesh.triangle(t);
    const Vec3 n = (v[1] - v[0]).cross(v[2] - v[0]);
    const double norm = n.norm();
    if (norm <= std::numeric_limits<double>::min()) continue;  // Degenerate triangle.
    const Vec3 u = n / norm;
    polys.push_back({v[0], v[1], v[2]});
    planes.push_back({u, u.dot(v[0])});
  }
  if (polys.empty()) throw std::invalid_argument("SolidBsp: mesh has no valid triangles");
  nodes_.reserve(2 * polys.size());
  build(std::move(polys), std::move(planes));
}

std::int32_t SolidBsp::build(std::vector<std::vector<Vec3>> polys, std::vector<Plane> planes) {
  // Pick the splitter among a bounded sample of the polygons' own planes,
  // minimizing splits first and imbalance second.
  constexpr std::size_t kMaxCandidates = 32;
  const std::size_t n = polys.size();
  const std::size_t stride = std::max<std::size_t>(1, n / kMaxCandidates);
  std::size_t best = 0;
  double best_score = std::numeric_limits<double>::infinity();
  for (std::size_t c = 0; c < n; c += stride) {
    long front = 0, back = 0, splits = 0;
    for (std::size_t i = 0; i < n; ++i) {
      switch (classify(polys[i], planes[c], tol_)) {
        case Side::kFront: ++front; break;
        case Side::kBack: ++back; break;
        case Side::kStraddle: ++splits; break;
        case Side::kCoplanar: break;
      }
    }
    const double score = 8.0 * static_cast<double>(splits) + std::abs(static_cast<double>(front - back));
    if (score < best_score) {
      best_score = score;
      best = c;
    }
  }

  const Plane splitter = planes[best];
  const auto index = static_cast<std::int32_t>(nodes_.size());
  nodes_.push_back({splitter, kOutside, kInside, false});

  std::vector<std::vector<Vec3>> front_polys, back_polys;
  std::vector<Plane> front_planes, back_planes;
  bool mixed = false;
  for (std::size_t i = 0; i < n; ++i) {
    switch (classify(polys[i], splitter, tol_)) {
      case Side::kCoplanar:  // Lies on this node's plane.
        mixed |= planes[i].n.dot(splitter.n) < 0.0;
        break;
      case Side::kFront:
        front_polys.push_back(std::move(polys[i]));
        front_planes.push_back(planes[i]);
        break;
      case Side::kBack:
        back_polys.push_back(std::move(polys[i]));
        back_planes.push_back(planes[i]);
        break;
      case Side::kStraddle: {
        std::vector<Vec3> f, b;
        split_polygon(polys[i], splitter, tol_, f, b);
        if (f.size() >= 3) {
          front_polys.push_back(std::move(f));
          front_planes.push_back(planes[i]);
        }
        if (b.size() >= 3) {
          back_polys.push_back(std::move(b));
          back_planes.push_back(planes[i]);
        }
        break;
      }
    }
  }
  polys.clear();
  planes.clear();

  // Indices are assigned after recursion because nodes_ may grow meanwhile.
  const std::int32_t front = front_polys.empty() ? kOutside : build(std::move(front_polys), std::move(front_planes));
  const std::int32_t back = back_polys.empty() ? kInside : build(std::move(back_polys), std::move(back_planes));
  nodes_[index].front = front;
  nodes_[index].back = back;
  nodes_[index].mixed = mixed;
  return index;
}

void SolidBsp::split(std::span<const Vec3> poly, const Vec3& normal, Coplanar rule, double tol,
                     std::vector<std::vector<Vec3>>* inside, std::vector<std::vector<Vec3>>* outside) const {
  split_at(0, {poly.begin(), poly.end()}, normal, rule, tol, inside, outside);
}

void SolidBsp::split_at(std::int32_t node, std::vector<Vec3> poly, const Vec3& normal, Coplanar rule, double tol,
                        std::vector<std::vector<Vec3>>* inside, std::vector<std::vector<Vec3>>* outside) const {
  if (node < 0) {
    (node == kInside ? inside : outside)->push_back(std::move(poly));
    return;
  }
  const Node& n = nodes_[node];
  switch (classify(poly, n.plane, tol)) {
    case Side::kFront: split_at(n.front, std::move(poly), normal, rule, tol, inside, outside); break;
    case Side::kBack: split_at(n.back, std::move(poly), normal, rule, tol, inside, outside); break;
    case Side::kCoplanar: {
      const bool same = normal.dot(n.plane.n) > 0.0;
      if (rule == Coplanar::kInsideBehind) {
        split_at(same ? n.back : n.front, std::move(poly), normal, rule, tol, inside, outside);
      } else if (rule == Coplanar::kInsideInFront) {
        split_at(same ? n.front : n.back, std::move(poly), normal, rule, tol, inside, outside);
      } else {  // Inside on both sides: what the front subtree accepts must also pass the back one.
        std::vector<std::vector<Vec3>> in_front;
        split_at(n.front, std::move(poly), normal, rule, tol, &in_front, outside);
        for (auto& f : in_front) split_at(n.back, std::move(f), normal, rule, tol, inside, outside);
      }
      break;
    }
    case Side::kStraddle: {
      std::vector<Vec3> f, b;
      split_polygon(poly, n.plane, tol, f, b);
      if (f.size() >= 3) split_at(n.front, std::move(f), normal, rule, tol, inside, outside);
      if (b.size() >= 3) split_at(n.back, std::move(b), normal, rule, tol, inside, outside);
      break;
    }
  }
}

bool SolidBsp::contains(const Vec3& p) const {
  std::int32_t node = 0;
  while (node >= 0) {
    const Node& n = nodes_[node];
    node = n.plane.distance(p) > tol_ ? n.front : n.back;
  }
  return node == kInside;
}

}  // namespace volcontact
