#pragma once

#include <array>

#include "volcontact/types.hpp"

namespace volcontact {

// Volume, centroid and inertia (unit density) of a solid.
struct MassProperties {
  double volume = 0.0;
  Vec3 centroid = Vec3::Zero();
  Mat3 inertia = Mat3::Zero();  // About the centroid; standard sign convention (positive diagonal).
};

// Accumulates the volume integrals of 1, x, y, z, x², y², z², xy, yz, zx over the
// region bounded by a set of outward-oriented polygons, using the divergence
// theorem (D. Eberly, "Polyhedral Mass Properties (Revisited)", 2002).
//
// The polygons need not be stored: each one is consumed as it is produced, so
// the integrator works directly on the output of polygon clipping. Accuracy is
// best when coordinates are given relative to a point close to the solid.
class VolumeIntegrator {
 public:
  // Adds the triangle (a, b, c), counter-clockwise seen from outside. Returns
  // (b - a) × (c - a), i.e. twice its vector area.
  Vec3 add_triangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 d = (b - a).cross(c - a);
    double f1[3], f2[3], f3[3], g0[3], g1[3], g2[3];
    for (int k = 0; k < 3; ++k) subexpressions(a[k], b[k], c[k], f1[k], f2[k], f3[k], g0[k], g1[k], g2[k]);
    s_[0] += d.x() * f1[0];
    s_[1] += d.x() * f2[0];
    s_[2] += d.y() * f2[1];
    s_[3] += d.z() * f2[2];
    s_[4] += d.x() * f3[0];
    s_[5] += d.y() * f3[1];
    s_[6] += d.z() * f3[2];
    s_[7] += d.x() * (a.y() * g0[0] + b.y() * g1[0] + c.y() * g2[0]);
    s_[8] += d.y() * (a.z() * g0[1] + b.z() * g1[1] + c.z() * g2[1]);
    s_[9] += d.z() * (a.x() * g0[2] + b.x() * g1[2] + c.x() * g2[2]);
    return d;
  }

  // Adds a convex polygon by fan triangulation. Returns twice its vector area.
  Vec3 add_polygon(const Vec3* v, int n) {
    Vec3 twice_area = Vec3::Zero();
    for (int i = 1; i + 1 < n; ++i) twice_area += add_triangle(v[0], v[i], v[i + 1]);
    return twice_area;
  }

  [[nodiscard]] MassProperties result() const {
    MassProperties mp;
    mp.volume = s_[0] / 6.0;
    if (mp.volume <= 0.0) return mp;
    const Vec3 first(s_[1] / 24.0, s_[2] / 24.0, s_[3] / 24.0);
    const Vec3 c = first / mp.volume;
    const double xx = s_[4] / 60.0, yy = s_[5] / 60.0, zz = s_[6] / 60.0;
    const double xy = s_[7] / 120.0, yz = s_[8] / 120.0, zx = s_[9] / 120.0;
    const double m = mp.volume;
    Mat3& I = mp.inertia;
    I(0, 0) = yy + zz - m * (c.y() * c.y() + c.z() * c.z());
    I(1, 1) = zz + xx - m * (c.z() * c.z() + c.x() * c.x());
    I(2, 2) = xx + yy - m * (c.x() * c.x() + c.y() * c.y());
    I(0, 1) = I(1, 0) = -(xy - m * c.x() * c.y());
    I(1, 2) = I(2, 1) = -(yz - m * c.y() * c.z());
    I(0, 2) = I(2, 0) = -(zx - m * c.z() * c.x());
    mp.centroid = c;
    return mp;
  }

 private:
  static void subexpressions(double w0, double w1, double w2, double& f1, double& f2, double& f3, double& g0,
                             double& g1, double& g2) {
    const double t0 = w0 + w1;
    f1 = t0 + w2;
    const double t1 = w0 * w0;
    const double t2 = t1 + w1 * t0;
    f2 = t2 + w2 * f1;
    f3 = w0 * t1 + w1 * t2 + w2 * f2;
    g0 = f2 + w0 * (f1 + w0);
    g1 = f2 + w1 * (f1 + w1);
    g2 = f2 + w2 * (f1 + w2);
  }

  std::array<double, 10> s_{};
};

}  // namespace volcontact
