#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "volcontact/intersection_properties.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// Stick-slip friction with bristles (Dopico et al., "Dealing with multiple
// contacts in a human-in-the-loop application", MSD 2011):
//   Ft = s·F_stick + (1 − s)·F_slide − μ_visc·v_t,   s = exp(−(|v_t| / v_stick)²)
// F_stick is a spring-damper on the bristle's deflection (see `deflection`),
// which slips whenever the stick force exceeds μ_static·|Fn|. In the
// published formulation of GonthierParams v_avg replaces |v_t| in s, and
// F_slide and the stick limit carry the share of the friction budget that the
// spin leaves to the force.
//
// The defaults hold a body still up to the static limit. The thesis used
// anchor_update = 0.8 and limit_includes_damping = true, which leave the spring
// below the limit after every slip and let a loaded body creep slowly.
struct BristleFriction {
  double mu_dynamic = 0.5;
  double mu_static = 0.5;
  double mu_viscous = 0.0;
  double v_stick = 0.5 * 9.81 * 5e-3;     // Transition velocity [m/s].
  double k_stick = 1e-2 / (5e-3 * 5e-3);  // Bristle stiffness [N/m].
  double c_stick = 2.0 * 20.0;            // Bristle damping [N·s/m] (2·√k_stick).
  double anchor_update = 1.0;             // Fraction of the limit the spring keeps when the anchor slips.
  bool limit_includes_damping = false;    // Compare spring + damper (instead of the spring) with the limit.
  // Pre-sliding: k = μ_static·|Fn| / preslide_travel and c = k·preslide_time, so
  // every contact reaches its static limit after the same displacement whatever
  // its load (Dahl, LuGre). With preslide_travel = 0, k_stick and c_stick above
  // are used instead (the thesis' constant bristle). For a body at rest on its
  // contact ω² = μ·g/travel whatever its mass, so explicit coupling at dt = 1 ms
  // needs ω²·dt·(dt + 2·time) < 4; 5 ms gives ζ ≈ 0.55 and holds up to a load of
  // ~7× the weight. Beyond that the step bounds them (StepLimits).
  double preslide_travel = 1e-4;  // Displacement at the static limit [m].
  double preslide_time = 5e-3;    // Damping time constant c/k [s].
  // What the bristle's deflection is.
  //  - kIntegrated: the slip of A's material point at the friction point, relative to B,
  //    accumulated from one evaluation to the next. It is the state of Gonthier's bristle,
  //    ż = v_ct (thesis 2007, eq. 4.227, while sticking); the damper acts on v_ct.
  //  - kAnchored: the distance from the friction point to an anchor fixed in B (Vilela's thesis
  //    2018); the damper acts along the bristle. With the friction point at the centroid of the
  //    overlap, which is not a material point, the bristle also stretches when the overlap
  //    changes shape with nothing sliding: a body that rocks by microradians on a flat contact
  //    moves its centroid by millimetres. GonthierParams::kThesis2018 always uses it.
  enum class Deflection { kIntegrated, kAnchored } deflection = Deflection::kIntegrated;
};

// Point whose velocity drives the friction model and whose slip (or, for the
// anchored bristle, whose position) loads its bristle. The resulting force is applied, like the normal force, at the
// centroid of the intersection volume (as in the thesis).
struct FrictionPoint {
  enum class Kind { kVolumeCentroid, kBodyPoint } kind = Kind::kVolumeCentroid;
  Vec3 p_A = Vec3::Zero();  // Body point of A (for kBodyPoint), in A's frame.
};

// Gonthier volumetric contact model (docs/gonthier-model.md lists the equations
// and their sources):
//   Fn = K·V·(1 − h·vn)     normal force along n, at the centroid of the overlap
//   τr = −K·h·Ig·ω_t        rolling resistance (none while Fn is clamped at zero)
// with V, Ig the intersection volume and its inertia. The friction comes in two
// formulations:
//  - kPublished (Gonthier's PhD thesis 2007, §4.4): Coulomb sliding force and
//    Coulomb spinning torque that share one friction budget through the
//    average surface velocity v_avg = √(|v_t|² + r_gyr²·ω_n²), r_gyr² = n·Ig·n / V:
//      Ft = −μ·Fn·v_t / v_avg,   τs = −μ·Fn·r_gyr²·ω_n / v_avg
//    Stick-slip is driven by v_avg, and a torsional bristle (stiffness and
//    damping of the translational one times r_gyr²) holds the spin at rest; its
//    angle is kept in FrictionState::twist and X_BA. It takes every coefficient
//    from `friction`: without it there is neither force nor spinning torque.
//  - kThesis2018 (Vilela's thesis): the sliding force ignores the spin and the
//    spinning torque is viscous, τs = −μ_spin·(Fn / V)·Ig·ω_n.
struct GonthierParams {
  enum class FrictionModel { kPublished, kThesis2018 };

  double stiffness = 1e6;   // K [N/m³].
  double damping = 5.0;     // h [s/m].
  double mu_spin = 0.5;     // kThesis2018 only: coefficient of the spinning torque [s/m].
  bool rolling_resistance = true;
  // The thesis let Fn go negative when separating faster than 1/h (adhesion).
  // By default Fn is clamped at zero, which makes no difference in its tests.
  bool allow_adhesion = false;
  std::optional<BristleFriction> friction;  // No tangential force if empty.
  FrictionPoint friction_point;
  FrictionModel friction_model = FrictionModel::kPublished;
};

// What the scheme that integrates the forces can resolve, for one pair (ContactModel fills it
// from the step and the bodies' inertias; see ContactModel::set_explicit_coupling). The
// engines apply the forces of the state at the start of a step, v += dt·M⁻¹·f, x += dt·v. For a
// spring k and a damper c on an inverse mass w that scheme is stable when
//   k·w·dt² + 2·c·w·dt < 4,
// and with several contacts on a body the sums over them count. Each law keeps
//   Σ k·w·dt² ≤ 1   and   Σ c·w·dt ≤ 1
// over the contacts of each body of the pair, for its stick spring and damper, for its normal
// damper and for the slope of its sliding friction across the stick–slip transition: a spring that would bring the pair back in less than a step, or a damper that
// would take more than the whole relative velocity in a step, is softened to that. Stick
// stiffness and normal damping are proportional to the load, so the sums are the "load
// accelerations" S = Σ |Fn|·w of the bodies [m/s²], and the bounds read
//   k ≤ |Fn| / (S·dt²),   c ≤ |Fn| / (S·dt),   h ≤ 1 / (S·dt),   v_stick ≥ μ·S·dt,
// the same for every pair of a body however its contact is cut into pieces. S is the one of the
// last evaluation (kept in FrictionState), and at least the pair's own. The normal stiffness K
// is the model's and is not touched: K·A·w·dt² < 2 remains the limit of an explicit coupling.
struct StepLimits {
  double dt = 0.0;  // Step [s]; 0 for no bounds.
  // Inverse masses of the pair (both bodies) at the overlap: along the normal (with the
  // rocking that the damping of the foundation resists), the largest in the tangent plane, and
  // the inverse inertia about the normal [1/(kg·m²)].
  double inv_mass_normal = 0.0, inv_mass_tangent = 0.0, inv_inertia_normal = 0.0;
  // Load accelerations of the two bodies at the last evaluation, added [m/s²].
  double load_normal = 0.0, load_tangent = 0.0;

  // The damping factor h the step can integrate, for a contact whose elastic force is `elastic`.
  [[nodiscard]] double damping(double h, double elastic) const {
    const double s = std::max(load_normal, elastic * inv_mass_normal);
    return dt > 0.0 && s > 0.0 ? std::min(h, 1.0 / (s * dt)) : h;
  }
  // Bounds the stick spring k and damper c of a contact that carries the normal force fn.
  void bound_stick(double fn, double* k, double* c) const {
    const double s = std::max(load_tangent, std::abs(fn) * inv_mass_tangent);
    if (dt <= 0.0 || s <= 0.0) return;
    *k = std::min(*k, std::abs(fn) / (s * dt * dt));
    *c = std::min(*c, std::abs(fn) / (s * dt));
  }
  // The stick–slip transition velocity the step can resolve. Sliding friction μ·|Fn| against the
  // slip is blended in over that velocity, a damper of up to 0.86·μ·|Fn|/v_stick: in one step
  // friction changes the velocity of the body by μ·S·dt, and a transition narrower than that
  // makes it overshoot zero at every step (the chatter of explicit Coulomb friction).
  [[nodiscard]] double transition(double v_stick, double mu, double fn) const {
    const double s = std::max(load_tangent, std::abs(fn) * inv_mass_tangent);
    return dt > 0.0 ? std::max(v_stick, mu * s * dt) : v_stick;
  }
  // Factor on the torsional spring and damper k·ρ², c·ρ² (ρ: radius of gyration of the patch),
  // already bounded as translational ones: the twist sees the inverse inertia about the normal.
  [[nodiscard]] double torsion(double rho2) const {
    const double ratio = dt > 0.0 && inv_mass_tangent > 0.0 ? rho2 * inv_inertia_normal / inv_mass_tangent : 0.0;
    return ratio > 1.0 ? 1.0 / ratio : 1.0;
  }
};

// Per-pair state carried from one evaluation to the next, in B's frame so that
// it moves with B. Which law reads which field:
//   - every law (the pair evaluator in ContactModel): active, X_BA, has_normal, normal_B and the
//     load accelerations. The normal is kept while the overlap defines none (see
//     IntersectionProperties::normal); the loads feed StepLimits.
//   - Gonthier: anchor_B (anchored bristle), deflection_B (integrated bristle), twist.
//   - distributed: deflection_B (one bristle for the whole patch) and twist. It never reads anchor_B.
//   - pcm: none of the friction fields (it is stateless in friction).
// The state is reset when the pair separates.
struct FrictionState {
  bool active = false;               // False until the first evaluation of a new contact.
  Vec3 anchor_B = Vec3::Zero();      // Gonthier only, anchored bristle: its anchor point.
  Vec3 deflection_B = Vec3::Zero();  // Gonthier (integrated bristle) and distributed: tangential bristle deflection.
  double twist = 0.0;                // Gonthier and distributed: torsional bristle deflection [rad].
  Pose X_BA;                         // Gonthier and distributed: pose of A in B at the last evaluation.
  bool has_normal = false;           // ContactModel: the pair was in contact at the last evaluation.
  Vec3 normal_B = Vec3::Zero();      // ContactModel: its pair normal then.
  double load_normal = 0.0;          // ContactModel, for StepLimits: load accelerations of the two bodies [m/s²]
  double load_tangent = 0.0;         // of the normal dampers and of the stick (both laws that use StepLimits).

  // Flat representation for engines that store state as numbers. Layout (indices into z):
  //   0 active; 1-3 anchor_B; 4-6 deflection_B; 7 twist; 8-16 X_BA.R (column-major); 17-19 X_BA.p;
  //   20 has_normal; 21-23 normal_B; 24 load_normal; 25 load_tangent.
  static constexpr int kSize = 26;
  void pack(double* z) const;
  [[nodiscard]] static FrictionState unpack(const double* z);
};

// Force on A, applied at `point`, plus a pure torque. B receives the opposite.
struct ContactForce {
  Vec3 point = Vec3::Zero();
  Vec3 force = Vec3::Zero();
  Vec3 torque = Vec3::Zero();
  double normal_force = 0.0;
};

// Evaluates the model for one overlapping pair and returns the friction state
// for the next step in `next`.
ContactForce gonthier_force(const IntersectionProperties& geo, const Pose& X_WA, const Twist& V_WA, const Pose& X_WB,
                            const Twist& V_WB, const GonthierParams& params, const FrictionState& state,
                            FrictionState* next, const StepLimits& limits = {});

}  // namespace volcontact
