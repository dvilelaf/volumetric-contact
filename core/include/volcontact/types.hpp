#pragma once

// Basic value types shared by every module.
//
// Only Eigen types without static alignment requirements (3-vectors, 3x3
// matrices) appear here, so this header is safe to include from translation
// units compiled with different SIMD flags (e.g. the core with -march=native and
// an engine backend linked against a generic binary distribution).

#include <algorithm>
#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>  // cross()

namespace volcontact {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// Rigid transform X_AB: maps coordinates in frame B to frame A, p_A = R·p_B + p.
struct Pose {
  Mat3 R = Mat3::Identity();
  Vec3 p = Vec3::Zero();

  [[nodiscard]] Vec3 apply(const Vec3& x) const { return R * x + p; }
  [[nodiscard]] Pose operator*(const Pose& other) const { return {R * other.R, R * other.p + p}; }
  [[nodiscard]] Pose inverse() const {
    Mat3 Rt = R.transpose();
    return {Rt, -(Rt * p)};
  }
};

// Spatial velocity of a body frame B measured in world: angular velocity and the
// translational velocity of the frame origin Bo, both expressed in world.
struct Twist {
  Vec3 w = Vec3::Zero();
  Vec3 v = Vec3::Zero();

  // Velocity of the material point of B currently at world position p_W.
  [[nodiscard]] Vec3 point_velocity(const Vec3& p_WBo, const Vec3& p_W) const {
    return v + w.cross(p_W - p_WBo);
  }
};

// Wrench acting on a body about its origin Bo, expressed in world.
struct Wrench {
  Vec3 torque = Vec3::Zero();
  Vec3 force = Vec3::Zero();
};

struct Aabb {
  Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity());
  Vec3 hi = Vec3::Constant(-std::numeric_limits<double>::infinity());

  [[nodiscard]] bool empty() const { return (lo.array() > hi.array()).any(); }
  [[nodiscard]] Vec3 center() const { return 0.5 * (lo + hi); }
  [[nodiscard]] Vec3 half_extent() const { return 0.5 * (hi - lo); }

  void grow(const Vec3& x) {
    lo = lo.cwiseMin(x);
    hi = hi.cwiseMax(x);
  }
  void grow(const Aabb& b) {
    lo = lo.cwiseMin(b.lo);
    hi = hi.cwiseMax(b.hi);
  }

  [[nodiscard]] bool overlaps(const Aabb& b) const {
    return (lo.array() <= b.hi.array()).all() && (b.lo.array() <= hi.array()).all();
  }

  [[nodiscard]] Aabb intersection(const Aabb& b) const { return {lo.cwiseMax(b.lo), hi.cwiseMin(b.hi)}; }

  // Smallest box, in frame A, that encloses this box (given in frame B) after X_AB.
  [[nodiscard]] Aabb transformed(const Pose& X_AB) const {
    const Vec3 c = X_AB.apply(center());
    const Vec3 e = X_AB.R.cwiseAbs() * half_extent();
    return {c - e, c + e};
  }
};

}  // namespace volcontact
