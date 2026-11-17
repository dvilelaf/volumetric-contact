#include "volcontact/pcm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace volcontact {

ContactForce pcm_force(const IntersectionProperties& geo, const ContactSurface& surface, const BodyContext& a,
                       const BodyContext& b, const PcmParams& params, PcmWorkspace& ws, const StepLimits& limits) {
  const double damping = limits.damping(params.damping, params.stiffness * geo.volume);
  const double k = 0.5 * params.stiffness;  // The springs of each body carry half (Simbody, two meshes).
  const bool nearest = params.penetration == PcmParams::Penetration::kNearestPoint;
  const Vec3& pA = a.X_W.p;
  const auto velocity = [&](const Vec3& x) -> Vec3 {  // Of A relative to B.
    return a.V_W.point_velocity(a.X_W.p, x) - b.V_W.point_velocity(b.X_W.p, x);
  };
  const Pose X_AW = a.X_W.inverse(), X_BW = b.X_W.inverse();

  // Normal forces, element by element: the springs of A's faces inside B, then those of B's
  // faces inside A.
  ws.elements.clear();
  double fn = 0.0, total = 0.0;
  Vec3 centre = Vec3::Zero();
  for (const auto& poly : surface.polygons) {
    if (poly.interior) continue;  // A seam is not surface: it has no springs.
    const BodyContext& target = poly.on_a ? b : a;
    const Pose& X_TW = poly.on_a ? X_BW : X_AW;
    const auto& planes = target.shape.face_planes();
    const Vec3 line_T = X_TW.R * -poly.normal;  // PCM's penetration line: into the element's own body.
    const Vec3* x = surface.vertices.data() + poly.begin;
    for (std::uint32_t i = 2; i < poly.count; ++i) {
      const double area = 0.5 * (x[i - 1] - x[0]).cross(x[i] - x[0]).norm();
      if (area == 0.0) continue;
      const Vec3 c = (x[0] + x[i - 1] + x[i]) / 3.0;
      const Vec3 c_T = X_TW.apply(c);
      // The target is convex: its nearest surface point is on the nearest face plane, and a
      // line leaves it through the first plane it meets.
      double depth = std::numeric_limits<double>::infinity();
      std::size_t face = 0;
      for (std::size_t j = 0; j < planes.size(); ++j) {
        if (!target.shape.plane_exterior(static_cast<std::uint32_t>(j))) continue;
        double d = -planes[j].distance(c_T);
        if (!nearest) {
          const double along = planes[j].n.dot(line_T);
          if (along <= 1e-9) continue;
          d /= along;
        }
        if (d < depth) {
          depth = d;
          face = j;
        }
      }
      if (!(depth > 0.0) || !std::isfinite(depth)) continue;
      // Direction that pushes the element's body out of the target, and the point of the force.
      const Vec3 out = nearest ? Vec3(target.X_W.R * planes[face].n) : Vec3(-poly.normal);
      const Vec3 point = nearest ? Vec3(c + depth * out) : c;
      const Vec3 dir = poly.on_a ? out : Vec3(-out);
      const double rate = -velocity(point).dot(dir);  // ẋ: positive while the penetration grows.
      const double f = k * area * depth * (1.0 + damping * rate);
      if (f <= 0.0) continue;
      ws.elements.push_back({point, dir, f});
      fn += f * dir.dot(geo.normal);
      total += f;
      centre += f * point;
    }
  }

  ContactForce result;
  result.point = pA;
  if (ws.elements.empty()) return result;

  const double v_t = limits.transition(params.transition_velocity, params.mu_dynamic, fn);
  Vec3 force = Vec3::Zero(), torque = Vec3::Zero();  // About A's origin.
  for (const auto& e : ws.elements) {
    Vec3 f = e.force * e.dir;
    const Vec3 v = velocity(e.point);
    const Vec3 vt = v - v.dot(e.dir) * e.dir;
    const double slip = vt.norm();
    if (slip > 0.0) {
      const double r = slip / v_t;
      const double mu =
          std::min(r, 1.0) * (params.mu_dynamic + 2.0 * (params.mu_static - params.mu_dynamic) / (1.0 + r * r));
      f -= mu * e.force / slip * vt;
    }
    force += f;
    torque += (e.point - pA).cross(f);
  }
  result.point = centre / total;
  result.force = force;
  result.torque = torque - (result.point - pA).cross(force);
  result.normal_force = fn;
  return result;
}

}  // namespace volcontact
