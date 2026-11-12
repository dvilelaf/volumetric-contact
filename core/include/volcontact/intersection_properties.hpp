#pragma once

#include "volcontact/types.hpp"

namespace volcontact {

// Geometric properties of the intersection volume V = A ∩ B, in world.
struct IntersectionProperties {
  double volume = 0.0;
  Vec3 centroid = Vec3::Zero();
  Mat3 inertia = Mat3::Zero();  // Of V about its centroid, unit density.
  // Unit contact normal: the direction that pushes A out of B. It is the
  // normalized vector area of the part of ∂V that belongs to B (minus that of
  // A), which for a planar contact contour equals the normal of the contour
  // plane (the least-squares plane of the thesis) without extracting the
  // contour. Zero when undefined or ill-conditioned: the vector area S of B's
  // part cancels (A entirely inside B, or piercing it), |S| < 0.1·V^(2/3).
  // ContactModel then keeps the pair's last normal (see FrictionState).
  Vec3 normal = Vec3::Zero();
  // The same from the faces of B that belong to its body's surface only, leaving
  // out the seams between the pieces of a compound body (CompoundShape). For a
  // piece of a faceted ground it is the normal of the facets in contact, however
  // the overlap straddles the piece. Equal to `normal` when B has no seams.
  Vec3 exterior_normal = Vec3::Zero();
};

}  // namespace volcontact
