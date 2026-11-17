#include "volcontact/distributed.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

#include "volcontact/detail/polar_moments.hpp"

namespace volcontact {
namespace {

using Triangle = DistributedWorkspace::Triangle;

// Velocity of A relative to B.
struct RelativeMotion {
  const BodyContext& a;
  const BodyContext& b;
  [[nodiscard]] Vec3 at(const Vec3& x) const {
    return a.V_W.point_velocity(a.X_W.p, x) - b.V_W.point_velocity(b.X_W.p, x);
  }
  [[nodiscard]] Vec3 angular() const { return a.V_W.w - b.V_W.w; }
};

// Turns the polygons of ∂V into triangles with the pressure at their vertices.
//
// The pressure is p = w·K·ℓ·g, the product of two functions that are affine on
// each piece of a polygon:
//  - ℓ(x), the column length, is the minimum of affine functions, one per face
//    of A or B where the column can end; each polygon is split where the
//    minimizing face changes.
//  - g(x) = 1 − h·n·v(x), the damping factor, with v the velocity of A relative
//    to B and n the pair normal, along which the columns run. n·v is the rate
//    at which the column through x shortens (n pushes A out of B, so a separating
//    motion, n·v > 0, shortens it and lowers the pressure): its ends are material points of A
//    and of B a distance ℓ apart along n, and since the relative velocity field
//    is rigid, v(x + ℓ·n) − v(x) = ω × ℓ·n has no component along n, so n·v is
//    the same all along the column. (A face's own normal would turn the slip
//    along any face not perpendicular to n into "damping".) Without adhesion the
//    pressure is cut at g = 0 by clipping the polygon along that line.
//
// A triangle stores the linear pressure that has the same ∫p dA and ∫p·x dA as
// the exact product (the normal force and its moment are exact and independent
// of the triangulation; the friction integrals see the linear equivalent).
//
// A column runs from the point to the first face of the surface of either body
// (a seam does not end it: the body goes on behind it, so the columns are those
// of the whole bodies however they are cut into pieces).
//
// w: a column carries K·ℓ, half at each end when the ends are on the two
// bodies. When both are on the same body (an overlap deep enough to enclose a
// body, or to pierce it) the end that pushes A out carries it whole.
class Sampler {
 public:
  Sampler(const DistributedParams& p, const BodyContext& a, const BodyContext& b, const RelativeMotion& motion,
          const Vec3& normal, DistributedWorkspace& ws)
      : p_(p), a_(a), b_(b), motion_(motion), normal_(normal), ws_(ws) {}

  // Polygon (world vertices) of A's surface (`on_a`) or B's, whose columns run
  // along the unit `column` and whose pressure pushes A along `dir`.
  void polygon(const Vec3* x, int n, const Vec3& column, const Vec3& dir, bool on_a) {
    collect_exits(x, n, column);
    if (ws_.exits.empty()) return;

    // Faces that cannot be the nearest anywhere on the polygon are dropped:
    // ℓ never exceeds the smallest per-face maximum.
    double bound = std::numeric_limits<double>::infinity();
    for (const Exit& e : ws_.exits) bound = std::min(bound, e.max_v);
    std::size_t kept = 0;
    for (const Exit& e : ws_.exits) {
      if (e.min_v <= bound) ws_.exits[kept++] = e;
    }
    ws_.exits.resize(kept);

    const auto weight = [&](const Exit& e) { return e.on_a != on_a ? 0.5 : 1.0; };
    if (kept == 1) {
      emit(x, n, ws_.exits[0], dir, weight(ws_.exits[0]));
      return;
    }
    for (std::size_t k = 0; k < kept; ++k) {
      ws_.region.assign(x, x + n);
      for (std::size_t l = 0; l < kept && ws_.region.size() >= 3; ++l) {
        if (l != k) clip(ws_.exits[k], ws_.exits[l]);
      }
      if (ws_.region.size() >= 3) {
        emit(ws_.region.data(), static_cast<int>(ws_.region.size()), ws_.exits[k], dir, weight(ws_.exits[k]));
      }
    }
  }

 private:
  using Exit = DistributedWorkspace::Exit;

  // Faces where a column of the polygon can end: those of the surface of either
  // body (not seams, behind which the body goes on) that the column leaves
  // through. Looking at every face of both shapes is what costs, so only the
  // faces that can be the first are taken:
  //  - the faces that bound the overlap (the seeds), which also give an upper
  //    bound of ℓ on the polygon;
  //  - of a shape without seams, the faces within that reach of the polygon: a
  //    column starts inside the shape and leaves it through one of its faces
  //    proper, so the face is near;
  //  - of a piece of a compound, every face: past a seam the column ends on the
  //    plane of a face, anywhere along it. Also of a shape with few faces.
  // The minimum over them is the minimum over all.
  void collect_exits(const Vec3* x, int n, const Vec3& column) {
    ws_.exits.clear();
    const auto add = [&](const BodyContext& body, bool on_a, const Vec3& column_S, std::uint32_t i) {
      if (!body.shape.plane_exterior(i)) return;
      const Plane& plane = body.shape.face_planes()[i];
      const double un = plane.n.dot(column_S);
      if (un <= 1e-9) return;  // The column does not leave through this face.
      // Distance inside the face, d − n·x_S with x_S = R_WSᵀ·(x − p_WS), is
      // d + n_W·p_WS − n_W·x: affine in world x.
      const Vec3 n_W = body.X_W.R * plane.n;
      Exit e{(plane.d + n_W.dot(body.X_W.p)) / un, -n_W / un, std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(), on_a};
      for (int k = 0; k < n; ++k) {
        const double l = e.alpha + e.beta.dot(x[k]);
        e.min_v = std::min(e.min_v, l);
        e.max_v = std::max(e.max_v, l);
      }
      ws_.exits.push_back(e);
    };

    // First every face of the shapes taken whole and the seeds of the others.
    const Vec3 column_S[2] = {a_.X_W.R.transpose() * column, b_.X_W.R.transpose() * column};
    const std::uint64_t tick = ++ws_.tick;
    for (const bool on_a : {true, false}) {
      const BodyContext& body = on_a ? a_ : b_;
      if (!whole(body.shape)) continue;
      // Most faces are turned down here, without the call.
      const auto& planes = body.shape.face_planes();
      const Vec3& u = column_S[on_a ? 0 : 1];
      for (std::uint32_t i = 0; i < planes.size(); ++i) {
        if (planes[i].n.dot(u) > 1e-9) add(body, on_a, u, i);
      }
    }
    for (const auto& seed : ws_.seeds) {
      const BodyContext& body = seed.on_a ? a_ : b_;
      if (whole(body.shape)) continue;
      (seed.on_a ? ws_.stamp_a : ws_.stamp_b)[seed.plane] = tick;
      add(body, seed.on_a, column_S[seed.on_a ? 0 : 1], seed.plane);
    }
    double reach = std::numeric_limits<double>::infinity();
    for (const Exit& e : ws_.exits) reach = std::min(reach, e.max_v);

    // Then the faces of the others within reach.
    Aabb box;
    for (const bool on_a : {true, false}) {
      const BodyContext& body = on_a ? a_ : b_;
      const CollisionShape& shape = body.shape;
      if (whole(shape)) continue;
      auto& stamp = on_a ? ws_.stamp_a : ws_.stamp_b;
      const auto take = [&](std::uint32_t i) {
        if (stamp[i] == tick) return;
        stamp[i] = tick;
        add(body, on_a, column_S[on_a ? 0 : 1], i);
      };
      if (!std::isfinite(reach)) {
        for (std::uint32_t i = 0; i < shape.face_planes().size(); ++i) take(i);
        continue;
      }
      if (box.empty()) {
        for (int k = 0; k < n; ++k) box.grow(x[k]);
      }
      const double pad = std::max(reach, 0.0) * (1.0 + 1e-9) + shape.plane_tolerance();
      const Aabb near{box.lo - Vec3::Constant(pad), box.hi + Vec3::Constant(pad)};
      shape.bvh().for_each_overlapping(near.transformed(body.X_W.inverse()),
                                       [&](std::uint32_t t) { take(shape.triangle_plane(t)); });
    }
  }

  // Every face of the shape is looked at, instead of only those near the polygon.
  [[nodiscard]] bool whole(const CollisionShape& shape) const {
    return ws_.all_exits || shape.has_seams() || shape.face_planes().size() <= CollisionShape::kFewFaces;
  }

  // Keeps the part of the region where face k ends the column before face l.
  void clip(const Exit& k, const Exit& l) {
    const double a = k.alpha - l.alpha;
    const Vec3 b = k.beta - l.beta;
    auto& in = ws_.region;
    auto& out = ws_.scratch;
    out.clear();
    for (std::size_t i = 0; i < in.size(); ++i) {
      const Vec3& p = in[i];
      const Vec3& q = in[(i + 1) % in.size()];
      const double fp = a + b.dot(p), fq = a + b.dot(q);
      if (fp <= 0.0) out.push_back(p);
      if ((fp < 0.0 && fq > 0.0) || (fp > 0.0 && fq < 0.0)) out.push_back(p + (fp / (fp - fq)) * (q - p));
    }
    in.swap(out);
  }

  void emit(const Vec3* x, int n, const Exit& e, const Vec3& dir, double weight) {
    // Damping factor at the vertices; where it changes sign the polygon is cut.
    auto& g = ws_.factor;
    g.resize(static_cast<std::size_t>(n));
    bool positive = false, negative = false;
    for (int i = 0; i < n; ++i) {
      g[i] = 1.0 - p_.damping * motion_.at(x[i]).dot(normal_);
      positive |= g[i] > 0.0;
      negative |= g[i] < 0.0;
    }
    if (negative && !p_.allow_adhesion) {
      if (!positive) return;
      auto& cut = ws_.cut;
      auto& cut_g = ws_.cut_factor;
      cut.clear();
      cut_g.clear();
      for (int i = 0; i < n; ++i) {
        const int j = i + 1 == n ? 0 : i + 1;
        if (g[i] >= 0.0) {
          cut.push_back(x[i]);
          cut_g.push_back(g[i]);
        }
        if ((g[i] < 0.0 && g[j] > 0.0) || (g[i] > 0.0 && g[j] < 0.0)) {
          cut.push_back(x[i] + (g[i] / (g[i] - g[j])) * (x[j] - x[i]));
          cut_g.push_back(0.0);
        }
      }
      if (cut.size() < 3) return;
      triangles(cut.data(), cut_g.data(), static_cast<int>(cut.size()), e, dir, weight);
    } else {
      triangles(x, g.data(), n, e, dir, weight);
    }
  }

  // Fan of the polygon with, per triangle, the linear pressure p̃ that matches
  // the moments of p = f·g (f = w·K·ℓ) with barycentric coordinates λ:
  //   ∫f·g·λ_k dA = A/60·(F·G + S + g_k·F + f_k·G + 2·f_k·g_k),
  // with F = Σf_i, G = Σg_i, S = Σf_i·g_i, and ∫p̃·λ_k dA = A/12·(Σp̃_i + p̃_k),
  // so p̃_k = (g_k·F + f_k·G + 2·f_k·g_k)/5 − (F·G + S)/20.
  void triangles(const Vec3* x, const double* g, int n, const Exit& e, const Vec3& dir, double weight) {
    const double scale = weight * p_.stiffness;
    const auto load = [&](const Vec3& v) { return scale * std::max(0.0, e.alpha + e.beta.dot(v)); };
    const double f0 = load(x[0]);
    double f1 = load(x[1]);
    for (int i = 2; i < n; ++i) {
      const double f2 = load(x[i]);
      const double f[3] = {f0, f1, f2}, gs[3] = {g[0], g[i - 1], g[i]};
      f1 = f2;
      const double F = f[0] + f[1] + f[2], G = gs[0] + gs[1] + gs[2];
      const double S = f[0] * gs[0] + f[1] * gs[1] + f[2] * gs[2];
      if (F <= 0.0 || (F * G + S <= 0.0 && !p_.allow_adhesion)) continue;  // No load on this triangle.
      Triangle t{{x[0], x[i - 1], x[i]}, {}, dir};
      for (int k = 0; k < 3; ++k) t.p[k] = 0.2 * (gs[k] * F + f[k] * G + 2.0 * f[k] * gs[k]) - 0.05 * (F * G + S);
      ws_.triangles.push_back(t);
    }
  }

  const DistributedParams& p_;
  const BodyContext& a_;
  const BodyContext& b_;
  const RelativeMotion& motion_;
  const Vec3 normal_;
  DistributedWorkspace& ws_;
};

using PressureIntegrals = DistributedWorkspace::Integrals;

PressureIntegrals pressure_integrals(const Triangle& t) {
  const Vec3 n = (t.x[1] - t.x[0]).cross(t.x[2] - t.x[0]);
  const double area2 = n.norm();
  if (area2 == 0.0) return {0.0, Vec3::Zero(), 0.0, Vec3::Zero()};
  const double area = 0.5 * area2;
  const double sum_p = t.p[0] + t.p[1] + t.p[2];
  const Vec3 px = area / 12.0 * (sum_p * (t.x[0] + t.x[1] + t.x[2]) + t.p[0] * t.x[0] + t.p[1] * t.x[1] +
                                 t.p[2] * t.x[2]);
  return {area2, n / area2, area * sum_p / 3.0, px};
}

// Pressure gradient in the plane of the triangle.
Vec3 pressure_gradient(const Triangle& t, const PressureIntegrals& pi) {
  const Vec3& m = pi.m;
  return (t.p[0] * m.cross(t.x[2] - t.x[1]) + t.p[1] * m.cross(t.x[0] - t.x[2]) + t.p[2] * m.cross(t.x[1] - t.x[0])) /
         pi.area2;
}

// ∫p·|x − q| dA, and ∫p·(x − q)/|x − q| dA, for q in the triangle's plane.
struct PolarPressure {
  double distance;
  Vec3 unit;
};

PolarPressure polar_pressure(const Triangle& t, const PressureIntegrals& pi, const Vec3& q) {
  const Vec3 g = pressure_gradient(t, pi);
  const double p_q = t.p[0] + g.dot(q - t.x[0]);
  const PolarMoments pm = polar_moments(q, t.x[0], t.x[1], t.x[2], pi.m);
  return {p_q * pm.distance + g.dot(pm.weighted_r), p_q * pm.unit_r + pm.outer_unit_r * g};
}

Vec3 vee_skew(const Mat3& M) { return 0.5 * Vec3(M(2, 1) - M(1, 2), M(0, 2) - M(2, 0), M(1, 0) - M(0, 1)); }

}  // namespace

ContactForce distributed_force(const IntersectionProperties& geo, const ContactSurface& surface, const BodyContext& a,
                               const BodyContext& b, const DistributedParams& given, const FrictionState& state,
                               FrictionState* next, DistributedWorkspace& ws, const StepLimits& limits) {
  DistributedParams params = given;
  params.damping = limits.damping(given.damping, given.stiffness * geo.volume);
  const RelativeMotion motion{a, b};
  ws.triangles.clear();
  Sampler sampler(params, a, b, motion, geo.normal, ws);

  // A's faces inside B push A along −n_A and B's faces inside A push it along
  // n_B. Columns start on A's faces and run along n (out of B), or start on B's
  // faces and run along −n. Seams between the pieces of a body are not contact
  // surface: they carry no pressure and no friction.
  ws.seeds.clear();
  ws.stamp_a.resize(std::max({ws.stamp_a.size(), a.shape.face_planes().size(), std::size_t{1}}));
  ws.stamp_b.resize(std::max({ws.stamp_b.size(), b.shape.face_planes().size(), std::size_t{1}}));
  ++ws.tick;
  for (const auto& poly : surface.polygons) {
    auto& stamp = poly.on_a ? ws.stamp_a : ws.stamp_b;
    if (stamp[poly.plane] == ws.tick) continue;
    stamp[poly.plane] = ws.tick;
    ws.seeds.push_back({poly.on_a, poly.plane});
  }
  for (const auto& poly : surface.polygons) {
    if (poly.interior) continue;
    const Vec3 dir = poly.on_a ? Vec3(-poly.normal) : poly.normal;
    const Vec3 column = poly.on_a ? geo.normal : Vec3(-geo.normal);
    sampler.polygon(surface.vertices.data() + poly.begin, static_cast<int>(poly.count), column, dir, poly.on_a);
  }

  // Normal force and kinetic friction, triangle by triangle. In the plane of a
  // triangle the slip of a rigid motion is a rotation about an instantaneous
  // centre q (or a translation), so Coulomb friction integrates in closed form.
  const Vec3& pA = a.X_W.p;
  const Vec3 w_rel = motion.angular();
  Vec3 normal_force = Vec3::Zero(), normal_torque = Vec3::Zero();
  Vec3 kinetic_force = Vec3::Zero(), kinetic_torque = Vec3::Zero();
  double pressure = 0.0;
  Vec3 pressure_moment = Vec3::Zero(), load_moment = Vec3::Zero();
  ws.integrals.resize(ws.triangles.size());
  for (std::size_t k = 0; k < ws.triangles.size(); ++k) {
    const Triangle& t = ws.triangles[k];
    const PressureIntegrals& pi = ws.integrals[k] = pressure_integrals(t);
    if (pi.p == 0.0 || (pi.p < 0.0 && !params.allow_adhesion)) continue;
    normal_force += pi.p * t.dir;
    normal_torque += (pi.px - pi.p * pA).cross(t.dir);
    if (pi.p < 0.0) continue;  // Tension carries no friction.
    pressure += pi.p;
    pressure_moment += pi.px;
    load_moment += t.dir.dot(geo.normal) * pi.px;

    const Vec3& m = pi.m;
    if (params.friction_quadrature == FrictionQuadrature::kVertex) {
      // Hasegawa and Sato 2004: f_i = −μ·p_i·v̂_i at the vertices (eq. 13 there), interpolated
      // with the barycentric coordinates λ (eqs. 14–16): ∫λ_i dA = A/3 and
      // ∫λ_i·λ_j dA = A/12·(1 + δ_ij) give the force A/3·Σf_i and the moment
      // A/12·(Σx_i × Σf_j + Σx_i × f_i).
      Vec3 sum = Vec3::Zero(), arm = Vec3::Zero(), moment = Vec3::Zero();
      for (int i = 0; i < 3; ++i) {
        const Vec3 v = motion.at(t.x[i]);
        const Vec3 vt = v - v.dot(m) * m;
        const double slip = vt.norm();
        arm += t.x[i] - pA;
        if (slip == 0.0) continue;
        const Vec3 f = -params.mu_dynamic * t.p[i] / slip * vt;
        sum += f;
        moment += (t.x[i] - pA).cross(f);
      }
      kinetic_force += pi.area2 / 6.0 * sum;
      kinetic_torque += pi.area2 / 24.0 * (arm.cross(sum) + moment);
      continue;
    }
    const Vec3 v0 = motion.at(t.x[0]);
    const Vec3 vt0 = v0 - v0.dot(m) * m;
    const double wm = w_rel.dot(m);
    const double size = std::sqrt(pi.area2);
    Vec3 fk;
    if (params.friction_quadrature == FrictionQuadrature::kCentroid || std::abs(wm) * size <= 1e-3 * vt0.norm()) {
      // Centre of rotation far away: uniform slip direction (kCentroid: taken as such).
      const Vec3 vc = motion.at(pi.px / pi.p);
      const Vec3 vt = vc - vc.dot(m) * m;
      if (vt.squaredNorm() == 0.0) continue;
      const Vec3 u = vt.normalized();
      fk = -params.mu_dynamic * pi.p * u;
      kinetic_torque += (pi.px - pi.p * pA).cross(-params.mu_dynamic * u);
    } else {
      if (wm == 0.0) continue;  // No slip at all.
      const Vec3 q = t.x[0] + m.cross(vt0) / wm;
      const PolarPressure pp = polar_pressure(t, pi, q);
      const double sign = wm > 0.0 ? 1.0 : -1.0;
      fk = -params.mu_dynamic * sign * m.cross(pp.unit);
      kinetic_torque += (q - pA).cross(fk) - params.mu_dynamic * sign * pp.distance * m;
    }
    kinetic_force += fk;
  }

  ContactForce out;
  *next = {};
  if (pressure <= 0.0) {  // Nothing is pressed; at most adhesion pulls, taken about A's origin.
    out.point = pA;
    out.force = normal_force;
    out.torque = normal_torque;
    out.normal_force = normal_force.dot(geo.normal);
    return out;
  }

  // The normal load is the part of the pressure along the pair normal, and the
  // stick acts at its centre with its capacity: faces along the normal (the
  // thin sides of the overlap) carry pressure but hold nothing. Both add up
  // over the pieces of a body.
  const Vec3& n = geo.normal;
  const double fn = normal_force.dot(n);
  const Vec3 c = fn > 0.0 ? Vec3(load_moment / fn) : Vec3(pressure_moment / pressure);

  Vec3 force = normal_force + kinetic_force;
  Vec3 torque = normal_torque + kinetic_torque;  // About A's origin.

  if (params.stick) {
    const StickParams& st = *params.stick;
    // Spread of the pressure around its centre: radius of gyration ρ and the
    // farthest point (which bounds the static spinning capacity from below).
    double second_moment = 0.0, r_max2 = 0.0;
    for (std::size_t k = 0; k < ws.triangles.size(); ++k) {
      const Triangle& t = ws.triangles[k];
      const PressureIntegrals& pi = ws.integrals[k];
      if (pi.p <= 0.0) continue;
      const Vec3 cq = c - (c - t.x[0]).dot(pi.m) * pi.m;  // Centre projected on the triangle's plane.
      for (int e = 0; e < 3; ++e) {  // Edge midpoints.
        const int l = (e + 1) % 3;
        second_moment += pi.area2 / 6.0 * 0.5 * (t.p[e] + t.p[l]) * (0.5 * (t.x[e] + t.x[l]) - cq).squaredNorm();
        r_max2 = std::max(r_max2, (t.x[e] - cq).squaredNorm());
      }
    }
    const double rho2 = second_moment / pressure;
    // Static spinning capacity μ_s·∫p·|r| dA, computed exactly only when the
    // torsional spring could reach it (∫p·|r| ≥ ∫p·r² / r_max).
    const auto spin_capacity = [&] {
      double capacity = 0.0;
      for (std::size_t k = 0; k < ws.triangles.size(); ++k) {
        const Triangle& t = ws.triangles[k];
        const PressureIntegrals& pi = ws.integrals[k];
        if (pi.p > 0.0) capacity += polar_pressure(t, pi, c - (c - t.x[0]).dot(pi.m) * pi.m).distance;
      }
      return params.mu_static * capacity;
    };

    const Vec3 vc = motion.at(c);
    const Vec3 vt = vc - vc.dot(n) * n;
    const double wn = w_rel.dot(n);
    const double v_stick = limits.transition(st.v_stick, params.mu_dynamic, fn);
    const double s = std::exp(-(vt.squaredNorm() + wn * wn * rho2) / (v_stick * v_stick));

    // Bristle deflections grow with the slip of A's material point at the
    // pressure centre (and A's twist about n) since the last evaluation, so
    // rolling without slip does not load them; they are kept in B's frame.
    const Pose X_BW = b.X_W.inverse();
    const Pose X_BA = X_BW * a.X_W;
    const Vec3 n_B = X_BW.R * n;
    Vec3 z = Vec3::Zero();
    double twist = 0.0;
    if (state.active) {
      const Vec3 c_B = X_BW.apply(c);
      const Vec3 slip = c_B - (state.X_BA * X_BA.inverse()).apply(c_B);
      z = state.deflection_B + slip;
      twist = state.twist + vee_skew(X_BA.R * state.X_BA.R.transpose()).dot(n_B);
    }
    z -= z.dot(n_B) * n_B;

    // The stick is a translational spring at the centre and a torsional one
    // about the normal (stiffness and damping k·ρ², c·ρ²). What they can hold
    // together is bounded by the limit surface of the patch, taken as the
    // ellipse (F / μ_s·Fn)² + (T / T_max)² ≤ 1 (Howe and Cutkosky), with T_max
    // the static spinning capacity: sliding and spinning share the same
    // friction. Beyond it both deflections are scaled back onto the limit (the
    // bristles slip). Limiting each on its own would let a patch hold up to √2
    // times what its friction can, and a patch cut into pieces, each of which
    // sees a spin of the body as slip plus spin, more than the whole patch.
    const double limit = params.mu_static * std::abs(fn);
    double k_stick = st.travel > 0.0 ? limit / st.travel : st.k_stick;
    double c_stick = st.travel > 0.0 ? k_stick * st.time : st.c_stick;
    limits.bound_stick(fn, &k_stick, &c_stick);
    const double arm2 = rho2 * limits.torsion(rho2);
    const double k_twist = k_stick * arm2;
    const double surely_held = r_max2 > 0.0 ? params.mu_static * second_moment / std::sqrt(r_max2) : 0.0;
    double capacity = -1.0;
    // Factor that brings a force and a torque back onto the limit surface (1 inside it).
    const auto onto_limit = [&](double f, double t) {
      const auto ratio = [](double value, double bound) {
        return value > 0.0 ? (bound > 0.0 ? value / bound : std::numeric_limits<double>::infinity()) : 0.0;
      };
      const double rf = ratio(f, limit);
      // surely_held ≤ T_max: inside the smaller ellipse nothing slips.
      if (rf * rf + ratio(t, surely_held) * ratio(t, surely_held) <= 1.0) return 1.0;
      if (t > 0.0 && capacity < 0.0) {
        // While spinning fast the stick has no weight (s ≈ 0); the cheap upper
        // bound μ_s·Fn·ρ ≥ μ_s·∫p·|r| dA then suffices to keep the twist bounded.
        capacity = s > 1e-9 ? spin_capacity() : params.mu_static * std::abs(fn) * std::sqrt(rho2);
      }
      const double rt = ratio(t, capacity);
      const double q = rf * rf + rt * rt;
      return q > 1.0 ? 1.0 / std::sqrt(q) : 1.0;
    };
    const double held = onto_limit(k_stick * z.norm(), std::abs(k_twist * twist));
    z *= held;
    twist *= held;
    Vec3 f_stick = -k_stick * (b.X_W.R * z) - c_stick * vt;
    double t_stick = -k_twist * twist - c_stick * arm2 * wn;
    // The dampers alone reach the limit at v = d/τ: the total is brought back onto it (load-proportional mode).
    if (st.travel > 0.0) {
      const double total = onto_limit(f_stick.norm(), std::abs(t_stick));
      f_stick *= total;
      t_stick *= total;
    }

    force = normal_force + s * f_stick + (1.0 - s) * kinetic_force;
    torque = normal_torque + s * ((c - pA).cross(f_stick) + t_stick * n) + (1.0 - s) * kinetic_torque;
    *next = {.active = true, .deflection_B = z, .twist = twist, .X_BA = X_BA};
  }

  out.point = c;
  out.force = force;
  out.torque = torque - (c - pA).cross(force);
  out.normal_force = fn;
  return out;
}

}  // namespace volcontact
