#include "volcontact/gonthier.hpp"

#include <algorithm>
#include <cmath>

namespace volcontact {
namespace {

// Stick-slip weight and Contensou factors of one evaluation (docs/gonthier-model.md).
struct Coupling {
  double s = 1.0;       // Sticking-state function.
  double force = 1.0;   // C_v,s: fraction of the friction budget taken by the force.
  double torque = 0.0;  // C_ω,s: fraction taken by the spinning torque.
};

// speed = |v_t| at the friction point, spin = r_gyr·|ω_n|.
Coupling coupling(double v_stick, bool published, double speed, double spin) {
  Coupling c;
  if (!published) {
    const double ratio = speed / v_stick;
    c.s = std::exp(-ratio * ratio);
    return c;
  }
  const double v_avg = std::hypot(speed, spin);
  const double ratio = v_avg / v_stick;
  c.s = std::exp(-ratio * ratio);
  const double denominator = std::max(v_avg, 0.01 * v_stick);
  c.force = c.s + (1.0 - c.s) * speed / denominator;
  c.torque = c.s + (1.0 - c.s) * spin / denominator;
  return c;
}

// Tangential force at point p (velocity vp on A, vb on B) for normal force fn.
Vec3 bristle_force(const BristleFriction& f, bool anchored, const Coupling& share, const Vec3& n, double fn,
                   const Vec3& p, const Vec3& vp, const Vec3& vb, const Pose& X_WA, const Pose& X_WB,
                   const StepLimits& limits, const FrictionState& state, FrictionState* next) {
  const Vec3 dv = vp - vb;
  const Vec3 vt = dv - n.dot(dv) * n;
  const double speed = vt.norm();
  const Vec3 dir = speed > 0.0 ? Vec3(vt / speed) : Vec3(Vec3::Zero());
  const Vec3 f_slide = -(f.mu_dynamic * fn * share.force) * dir;

  const double static_limit = f.mu_static * std::abs(fn);
  const double limit = static_limit * share.force;
  const bool preslide = f.preslide_travel > 0.0;
  double k = preslide ? static_limit / f.preslide_travel : f.k_stick;
  double c = preslide ? k * f.preslide_time : f.c_stick;
  limits.bound_stick(fn, &k, &c);
  next->active = true;

  if (!anchored) {
    // The deflection grows with the slip of A's material point at p relative to B since the
    // last evaluation; it is kept in B's frame, in the tangent plane.
    const Pose X_BW = X_WB.inverse();
    const Pose X_BA = X_BW * X_WA;
    const Vec3 n_B = X_BW.R * n;
    Vec3 z = Vec3::Zero();
    if (state.active) {
      const Vec3 p_B = X_BW.apply(p);
      z = state.deflection_B + p_B - (state.X_BA * X_BA.inverse()).apply(p_B);
    }
    z -= z.dot(n_B) * n_B;
    Vec3 elastic = -k * (X_WB.R * z);
    const Vec3 viscous = -c * vt;
    Vec3 f_stick = elastic + viscous;
    const double magnitude = (f.limit_includes_damping ? f_stick : elastic).norm();
    if (magnitude > limit) {
      f_stick = f.limit_includes_damping ? Vec3(f_stick * (limit / magnitude))
                                         : Vec3(elastic * (limit / magnitude) + viscous);
      // The bristle slips: it keeps the deflection of the limit, along itself.
      if (z.norm() > 0.0) z *= f.anchor_update * limit / (k * z.norm());
    }
    if (preslide && f_stick.norm() > limit) f_stick *= limit / f_stick.norm();
    next->deflection_B = z;
    next->X_BA = X_BA;
    return share.s * f_stick + (1.0 - share.s) * f_slide - f.mu_viscous * vt;
  }

  // A new contact anchors its bristle at the current point.
  Vec3 p0 = state.active ? X_WB.apply(state.anchor_B) : p;

  // Spring-damper along the bristle (as in the thesis), projected on the tangent plane.
  Vec3 elastic = Vec3::Zero(), viscous = Vec3::Zero();
  const Vec3 d = p - p0;
  const double len = d.norm();
  if (len > 0.0) {
    const Vec3 u = d / len;
    elastic = -k * len * u;
    viscous = -c * u.dot(dv) * u;
    elastic -= n.dot(elastic) * n;
    viscous -= n.dot(viscous) * n;
  }
  Vec3 f_stick = elastic + viscous;
  const double magnitude = (f.limit_includes_damping ? f_stick : elastic).norm();
  if (magnitude > limit) {
    f_stick = f.limit_includes_damping ? Vec3(f_stick * (limit / magnitude))
                                       : Vec3(elastic * (limit / magnitude) + viscous);
    // The anchor trails the point along the slip; along the bristle itself when there is no slip
    // or the slip runs against the bristle (it is unloading), which would otherwise flip the spring.
    const Vec3 trail = speed > 0.0 && dir.dot(elastic) <= 0.0 ? dir : Vec3(-elastic.normalized());
    p0 = p - (f.anchor_update * limit / k) * trail;
  }
  // With the load-proportional stiction the damper alone reaches the static limit at v = d/τ: the
  // total stick force is capped there so that a body that must slide is not braked by it.
  if (preslide && f_stick.norm() > limit) f_stick *= limit / f_stick.norm();
  next->anchor_B = X_WB.R.transpose() * (p0 - X_WB.p);
  return share.s * f_stick + (1.0 - share.s) * f_slide - f.mu_viscous * vt;
}

// Spinning friction torque about n of the published model: the torsional
// counterpart of bristle_force, with lever arm r_gyr. wn = n·ω_rel.
double spinning_torque(const BristleFriction& f, const Coupling& share, const Vec3& n, double fn, double r_gyr,
                       double wn, const Pose& X_WA, const Pose& X_WB, const StepLimits& limits,
                       const FrictionState& state, FrictionState* next) {
  // The bristle angle grows with the rotation of A relative to B about n since the last evaluation.
  const Pose X_BA = X_WB.inverse() * X_WA;
  double twist = 0.0;
  if (state.active) {
    const Mat3 dR = X_BA.R * state.X_BA.R.transpose();
    const Vec3 rotation = 0.5 * Vec3(dR(2, 1) - dR(1, 2), dR(0, 2) - dR(2, 0), dR(1, 0) - dR(0, 1));
    twist = state.twist + rotation.dot(X_WB.R.transpose() * n);
  }
  const double sense = wn > 0.0 ? 1.0 : (wn < 0.0 ? -1.0 : 0.0);
  const double t_slide = -f.mu_dynamic * fn * r_gyr * share.torque * sense;

  const double static_limit = f.mu_static * std::abs(fn);
  const double limit = static_limit * r_gyr * share.torque;
  const bool preslide = f.preslide_travel > 0.0;
  double k = preslide ? static_limit / f.preslide_travel : f.k_stick;
  double c = preslide ? k * f.preslide_time : f.c_stick;
  limits.bound_stick(fn, &k, &c);
  const double arm2 = r_gyr * r_gyr * limits.torsion(r_gyr * r_gyr);
  k *= arm2;
  c *= arm2;
  const double elastic = -k * twist, viscous = -c * wn;
  double t_stick = elastic + viscous;
  const double magnitude = std::abs(f.limit_includes_damping ? t_stick : elastic);
  if (magnitude > limit) {
    t_stick = f.limit_includes_damping ? t_stick * (limit / magnitude) : elastic * (limit / magnitude) + viscous;
    // Along the spin, or along the bristle itself when it is unloading.
    twist = (f.anchor_update * limit / k) * (sense * twist > 0.0 ? sense : std::copysign(1.0, twist));
  }
  if (preslide) t_stick = std::clamp(t_stick, -limit, limit);
  next->twist = twist;
  next->X_BA = X_BA;
  return share.s * t_stick + (1.0 - share.s) * t_slide - f.mu_viscous * r_gyr * r_gyr * wn;
}

}  // namespace

void FrictionState::pack(double* z) const {
  z[0] = active ? 1.0 : 0.0;
  Eigen::Map<Vec3>(z + 1) = anchor_B;
  Eigen::Map<Vec3>(z + 4) = deflection_B;
  z[7] = twist;
  Eigen::Map<Mat3>(z + 8) = X_BA.R;
  Eigen::Map<Vec3>(z + 17) = X_BA.p;
  z[20] = has_normal ? 1.0 : 0.0;
  Eigen::Map<Vec3>(z + 21) = normal_B;
  z[24] = load_normal;
  z[25] = load_tangent;
}

FrictionState FrictionState::unpack(const double* z) {
  return {.active = z[0] > 0.5,
          .anchor_B = Eigen::Map<const Vec3>(z + 1),
          .deflection_B = Eigen::Map<const Vec3>(z + 4),
          .twist = z[7],
          .X_BA = {Eigen::Map<const Mat3>(z + 8), Eigen::Map<const Vec3>(z + 17)},
          .has_normal = z[20] > 0.5,
          .normal_B = Eigen::Map<const Vec3>(z + 21),
          .load_normal = z[24],
          .load_tangent = z[25]};
}

ContactForce gonthier_force(const IntersectionProperties& geo, const Pose& X_WA, const Twist& V_WA, const Pose& X_WB,
                            const Twist& V_WB, const GonthierParams& params, const FrictionState& state,
                            FrictionState* next, const StepLimits& limits) {
  *next = state;
  ContactForce out;
  const Vec3& n = geo.normal;
  const Vec3& c = geo.centroid;
  out.point = c;

  const Vec3 v_rel = V_WA.point_velocity(X_WA.p, c) - V_WB.point_velocity(X_WB.p, c);
  const double damping = limits.damping(params.damping, params.stiffness * geo.volume);
  const double pressure_factor = 1.0 - damping * n.dot(v_rel);
  const bool loaded = params.allow_adhesion || pressure_factor > 0.0;
  const double fn = loaded ? params.stiffness * geo.volume * pressure_factor : 0.0;
  out.normal_force = fn;
  out.force = fn * n;

  const Vec3 w_rel = V_WA.w - V_WB.w;
  const double wn = n.dot(w_rel);
  const Vec3 w_n = wn * n;
  const bool published = params.friction_model == GonthierParams::FrictionModel::kPublished;
  // Radius of gyration of the overlap about the normal.
  const double r_gyr = std::sqrt(std::max(n.dot(geo.inertia * n) / geo.volume, 0.0));

  if (params.friction) {
    const bool at_body_point = params.friction_point.kind == FrictionPoint::Kind::kBodyPoint;
    const Vec3 p = at_body_point ? X_WA.apply(params.friction_point.p_A) : c;
    const Vec3 vp = V_WA.point_velocity(X_WA.p, p), vb = V_WB.point_velocity(X_WB.p, p);
    const Vec3 dv = vp - vb;
    const double v_stick = limits.transition(params.friction->v_stick, params.friction->mu_dynamic, fn);
    const Coupling share =
        coupling(v_stick, published, (dv - n.dot(dv) * n).norm(), published ? r_gyr * std::abs(wn) : 0.0);
    // Read the state before bristle_force marks it active.
    if (published) {
      out.torque = spinning_torque(*params.friction, share, n, fn, r_gyr, wn, X_WA, X_WB, limits, state, next) * n;
    }
    const bool anchored = !published || params.friction->deflection == BristleFriction::Deflection::kAnchored;
    out.force += bristle_force(*params.friction, anchored, share, n, fn, p, vp, vb, X_WA, X_WB, limits, state, next);
  }
  if (!published) out.torque = -(params.mu_spin * fn / geo.volume) * (geo.inertia * w_n);

  // The rolling resistance is the moment of the foundation pressure: none while it is clamped at zero.
  if (params.rolling_resistance && loaded) {
    out.torque -= params.stiffness * damping * (geo.inertia * (w_rel - w_n));
  }
  return out;
}

}  // namespace volcontact
