#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "volcontact/contact_surface.hpp"
#include "volcontact/gonthier.hpp"
#include "volcontact/intersection.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// Lumped stick (bristle) for the distributed model: a translational spring at
// the centre of the normal load and a torsional spring about the pair normal,
// limited together by the static friction the pressure distribution can
// provide (the ellipse (F / μ_s·Fn)² + (T / T_max)² ≤ 1). Blended
// with the kinetic friction by s = exp(−(u / v_stick)²), with u the slip speed
// at the pressure centre combined with the spin speed at its radius of gyration.
struct StickParams {
  double v_stick = 0.5 * 9.81 * 5e-3;  // Transition velocity [m/s].
  double k_stick = 400.0;              // Translational stiffness [N/m]; torsional is k·ρ².
  double c_stick = 40.0;               // Translational damping [N·s/m]; torsional is c·ρ².
  // Pre-sliding: k = μ_static·|Fn| / travel and c = k·time, so every contact
  // reaches its static limit after the same displacement whatever its load. With
  // travel = 0, k_stick and c_stick are used instead (the constant bristle). For
  // a body at rest ω² = μ·g/travel whatever its mass, so explicit coupling at
  // dt = 1 ms needs ω²·dt·(dt + 2·time) < 4; 5 ms gives ζ ≈ 0.55 and holds up to
  // a load of ~7× the weight. Beyond that the step bounds them (StepLimits).
  double travel = 1e-4;  // Displacement at the static limit [m].
  double time = 5e-3;    // Damping time constant c/k [s].
};

// Gonthier's elastic (Winkler) foundation integrated point by point instead of
// through closed-form volume moments. The overlap V is a bundle of springs along
// the contact normal n of the pair; a point of the contact surface (∂V without
// the seams between the pieces of a compound body) carries the pressure
// p = K·ℓ·(1 − h·vn), with ℓ the length of the column of V through it along n
// and vn = n·v the rate at which that column lengthens (v: velocity of A
// relative to B), plus Coulomb friction μ·p against its own slip velocity.
// Integrating over the clipped polygons gives the normal force, the centre of
// pressure, and spinning, rolling and combined friction without closed-form
// shape factors. For a flat contact the normal force is K·V·(1 − h·vn) with vn
// at the centroid of V: Gonthier's. The two ends of a column carry half each,
// so the model is symmetric. Columns do not stop at seams, so the result does
// not depend on how a body is cut into pieces; a column with both ends on the
// same body (an overlap that encloses or pierces it) is carried whole by the
// end that pushes A out, so the force does not drop at the deepest overlaps.
//
// Integration is exact for the pressure: ℓ is the minimum of affine functions
// (one per face where a column can end), so each polygon is split where the
// minimizing face changes, cut where the damping factor changes sign, and the
// product of the two affine factors is integrated in closed form. Friction is
// integrated in closed form (see polar_moments.hpp) for the linear pressure
// with the same resultant and moment on each triangle.
//
// The other two friction quadratures are those of the literature, kept to measure
// what the closed form is worth (docs/baselines.md). They change the kinetic
// friction only: geometry, pressure, clamp and stick are the same.
enum class FrictionQuadrature {
  kClosedForm,  // ∫μ·p·v̂ dA exactly, about the instantaneous centre of the slip in the triangle's plane.
  // Hasegawa and Sato 2004, eqs. 13–16: f = −μ·p·v̂ at the three vertices (none at a
  // vertex that does not slip), interpolated linearly over the triangle.
  kVertex,
  // One point per triangle, as PCM, Simbody's elastic foundation and the discrete
  // hydroelastic model do per element: μ·∫p dA at the pressure centroid, against the slip there.
  kCentroid,
};

struct DistributedParams {
  double stiffness = 1e6;  // K [N/m³].
  double damping = 5.0;    // h [s/m].
  double mu_dynamic = 0.5;
  double mu_static = 0.5;
  std::optional<StickParams> stick = StickParams{};  // Kinetic friction only if empty.
  bool allow_adhesion = false;                        // Let p go negative when separating fast.
  FrictionQuadrature friction_quadrature = FrictionQuadrature::kClosedForm;
};

// Scratch memory for distributed_force.
struct DistributedWorkspace {
  struct Triangle {
    Vec3 x[3];    // World positions.
    double p[3];  // Pressure at the vertices (already weighted by the side's share).
    Vec3 dir;     // Direction of the pressure force on A.
  };
  // Column length as an affine function α + β·x of the position, for one face
  // where the column can end.
  struct Exit {
    double alpha;
    Vec3 beta;
    double min_v, max_v;  // Over the polygon's vertices.
    bool on_a;            // The face belongs to A.
  };
  // ∫p dA and ∫p·x dA over a triangle with linear pressure.
  struct Integrals {
    double area2;  // Twice the area.
    Vec3 m;        // Unit normal.
    double p;
    Vec3 px;
  };
  // A face that bounds the overlap: plane index in its shape.
  struct Seed {
    bool on_a;
    std::uint32_t plane;
  };
  std::vector<Triangle> triangles;
  std::vector<Exit> exits;
  std::vector<Integrals> integrals;  // Per triangle.
  std::vector<Seed> seeds;
  std::vector<std::uint64_t> stamp_a, stamp_b;  // Per face plane: last tick it was looked at.
  std::uint64_t tick = 0;
  // For tests: look at every face of both shapes for the end of a column, which
  // must give the same forces as the default (only the faces within reach).
  bool all_exits = false;
  std::vector<Vec3> region, scratch, cut;
  std::vector<double> factor, cut_factor;
};

// The two bodies of a pair: convex shapes with their poses and velocities.
struct BodyContext {
  const CollisionShape& shape;
  const Pose& X_W;
  const Twist& V_W;
};

// Evaluates the distributed model from the contact surface of an overlapping
// pair (see intersect()). Returns the resultant on A as a force at the pressure
// centre plus a torque, and the friction state for the next step in `next`.
// `limits` bounds the damping and the stick to what the step can integrate.
ContactForce distributed_force(const IntersectionProperties& geo, const ContactSurface& surface, const BodyContext& a,
                               const BodyContext& b, const DistributedParams& params, const FrictionState& state,
                               FrictionState* next, DistributedWorkspace& ws, const StepLimits& limits = {});

}  // namespace volcontact
