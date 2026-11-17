#include "volcontact/detail/polar_moments.hpp"

#include <cmath>

namespace volcontact {
namespace {

// Antiderivatives in t = tan θ, with sec θ = √(1 + t²):
//   ∫sec = asinh t,  ∫sec·tan = sec,  ∫sec³ = (sec·t + asinh t)/2,  ∫sec³·tan = sec³/3.
struct Primitives {
  double sec, sec_tan, sec3, sec3_tan;
  explicit Primitives(double t) {
    const double s = std::sqrt(1.0 + t * t);
    const double a = std::asinh(t);
    sec = a;
    sec_tan = s;
    sec3 = 0.5 * (s * t + a);
    sec3_tan = s * s * s / 3.0;
  }
};

// Signed integral over the triangle (q, a, b): with e1 the unit direction from q
// to the line ab and h its distance, a point at angle θ from e1 is at r = h·sec θ.
void add_edge(const Vec3& q, const Vec3& a, const Vec3& b, const Vec3& m, PolarMoments& out) {
  const Vec3 ra = a - q, rb = b - q;
  const Vec3 d = b - a;
  const double len2 = d.squaredNorm();
  if (len2 == 0.0) return;
  const Vec3 foot = ra - (ra.dot(d) / len2) * d;  // From q to the line, perpendicular to it.
  const double h = foot.norm();
  if (h <= 1e-15 * std::sqrt(len2)) return;  // q on the line: no area.
  const Vec3 e1 = foot / h;
  const Vec3 e2 = m.cross(e1);
  const Primitives pa(ra.dot(e2) / h), pb(rb.dot(e2) / h);
  const double d_sec = pb.sec - pa.sec, d_sec_tan = pb.sec_tan - pa.sec_tan;
  const double d_sec3 = pb.sec3 - pa.sec3, d_sec3_tan = pb.sec3_tan - pa.sec3_tan;

  const double h2 = h * h, h3 = h2 * h;
  out.distance += h3 / 3.0 * d_sec3;
  out.weighted_r += h3 * h / 4.0 * (d_sec3 * e1 + d_sec3_tan * e2);
  out.unit_r += h2 / 2.0 * (d_sec * e1 + d_sec_tan * e2);
  out.outer_unit_r += h3 / 3.0 *
                      (d_sec * e1 * e1.transpose() + d_sec_tan * (e1 * e2.transpose() + e2 * e1.transpose()) +
                       (d_sec3 - d_sec) * e2 * e2.transpose());
}

}  // namespace

PolarMoments polar_moments(const Vec3& q, const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& m) {
  PolarMoments out;
  add_edge(q, p0, p1, m, out);
  add_edge(q, p1, p2, m, out);
  add_edge(q, p2, p0, m, out);
  return out;
}

}  // namespace volcontact
