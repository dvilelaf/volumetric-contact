#pragma once

#include <vector>

#include "volcontact/contact_surface.hpp"
#include "volcontact/distributed.hpp"
#include "volcontact/gonthier.hpp"
#include "volcontact/intersection_properties.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// Per-face elastic foundation, the baseline of the multibody literature for conforming contact:
// Hippmann's Polygonal Contact Model (PCM; Hippmann 2004, restated in Hippmann 2024, "Polygonal
// contact model revisited", eqs. 1–4) and Simbody's ElasticFoundationForce (Sherman, Seth and
// Delp 2011; the formulas are those of its class documentation and source). One spring per
// surface element, sampled once at the element's centroid c:
//   F = k·A·x·(1 + c_d·ẋ)        Simbody; PCM eqs. 1 and 4 with the damping d_l = c_l·c_d·x
// with A the element's area, x its penetration into the other body and ẋ its rate, clamped at
// zero (no tension), plus Coulomb friction at the same point, regularized (Simbody, Hollars):
//   F_t = F·[min(v/v_t, 1)·(μ_d + 2·(μ_s − μ_d)/(1 + (v/v_t)²))]   against the slip v.
// Neither publication has stiction ("this model cannot represent stiction", Simbody), so there
// is no stick here either: a body on a slope creeps at up to v_t.
//
// Mapping from the repository's (K, h, μ): layer stiffness c_l = k = K, c_d = h. Both bodies
// are meshes and the springs of each are treated independently (Simbody, two meshes), each
// with half the stiffness, so a flat contact carries K·A·δ = K·V in all.
//
// Departures from the publications, all stated in docs/baselines.md:
//  - The elements are the triangles of the exact contact surface (the parts of each body's
//    faces inside the other, fan-triangulated as for the distributed law), not the whole faces
//    whose centroid is inside. A whole face inside the other body is the publications' element.
//    Without this the two-triangle faces of the battery's boxes and grounds would never, or
//    always wholly, be in contact; it also removes PCM's force jumps as faces switch on.
//  - The other body is convex (a piece of a compound): the penetration is measured to its
//    faces proper, not to the seams against its sibling pieces.
//  - The explicit coupling bounds the damping and the transition velocity (StepLimits), as for
//    the other laws.
struct PcmParams {
  // How the penetration of an element is measured and where its force points.
  enum class Penetration {
    // Simbody: to the nearest point of the other body's surface; the force is along that
    // displacement and is applied at the nearest point.
    kNearestPoint,
    // PCM: along the element's inward normal to the other body's surface (the "penetration
    // line"); the force is along the element's normal, at its centroid. A face of the thin
    // side of an overlap (a clipped strip of a box's side wall) has its line run through the
    // whole of the other body: usable only where every element faces the other surface.
    kFaceNormal,
  };

  double stiffness = 1e6;  // K [N/m³]: the layer stiffness c_l (PCM eq. 2), k (Simbody).
  double damping = 5.0;    // h [s/m]: Simbody's dissipation c.
  double mu_dynamic = 0.5;
  double mu_static = 0.5;
  double transition_velocity = 0.5 * 9.81 * 5e-3;  // v_t [m/s]; StickParams::v_stick, for a like comparison.
  Penetration penetration = Penetration::kNearestPoint;
};

// Scratch memory for pcm_force.
struct PcmWorkspace {
  struct Element {
    Vec3 point;    // Where its force acts.
    Vec3 dir;      // Direction of its normal force on A.
    double force;  // Its normal force.
  };
  std::vector<Element> elements;
};

// Evaluates the per-face elastic foundation on the contact surface of an overlapping pair of
// convex shapes (see intersect()). Returns the resultant on A as a force at the centre of the
// normal forces plus a torque. It keeps no state. `limits` bounds the damping and the
// transition velocity to what the step can integrate; `geo` is used for that only.
ContactForce pcm_force(const IntersectionProperties& geo, const ContactSurface& surface, const BodyContext& a,
                       const BodyContext& b, const PcmParams& params, PcmWorkspace& ws,
                       const StepLimits& limits = {});

}  // namespace volcontact
