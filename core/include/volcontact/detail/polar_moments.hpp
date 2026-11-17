#pragma once

#include "volcontact/types.hpp"

namespace volcontact {

// Integrals over a triangle of functions of r = x − q, for a point q in the
// triangle's plane (q need not be inside the triangle). They give Coulomb
// friction in closed form when the slip is a rotation about q:
//   distance = ∫|r| dA            weighted_r   = ∫ r·|r| dA
//   unit_r   = ∫ r/|r| dA         outer_unit_r = ∫ r·rᵀ/|r| dA
// The triangle is split into the three triangles (q, Pi, Pj), each integrated in
// polar coordinates about q; contributions are signed by orientation about `m`,
// the unit normal of the plane.
struct PolarMoments {
  double distance = 0.0;
  Vec3 weighted_r = Vec3::Zero();
  Vec3 unit_r = Vec3::Zero();
  Mat3 outer_unit_r = Mat3::Zero();
};

[[nodiscard]] PolarMoments polar_moments(const Vec3& q, const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& m);

}  // namespace volcontact
