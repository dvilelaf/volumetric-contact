#include <array>
#include <numbers>
#include <utility>

#include "common.hpp"

namespace volcontact::scenarios {

struct CoulombDisk {
  double force, torque;
};
struct ContensouTheory {
  std::vector<double> t, v, wr;
};

// Rigid disk of unit radius with uniform pressure and Coulomb friction at every
// point against its own slip velocity, for v along x and ω > 0 about z. The point
// of zero slip is c = (0, k), k = v/ω; polar coordinates about it make the
// direction of the friction force depend on the angle only. Returns
// −F_x/(μ m g) and −T_z/(μ m g r), both ≥ 0.
CoulombDisk coulomb_disk(double v, double w) {
  constexpr int kN = 4096;
  const double pi = std::numbers::pi;
  double fx = 0.0, torque = 0.0;
  if (w <= 0.0) return {1.0, 0.0};
  const double k = v / w;
  if (k < 1.0) {
    // c inside the disk: Q(θ) is the distance from c to the circle.
    for (int i = 0; i < kN; ++i) {
      const double th = 2.0 * pi * (i + 0.5) / kN, s = std::sin(th), c = std::cos(th);
      const double q = -k * s + std::sqrt(1.0 - k * k * c * c);
      fx += -s * q * q / 2.0;                              // û_x = −sin θ
      torque += -k * (-s) * q * q / 2.0 + q * q * q / 3.0;  // (c × û)_z = −k û_x
    }
    fx *= 2.0 * pi / kN;
    torque *= 2.0 * pi / kN;
  } else {
    // c outside: sin Δ = sin ψ / k, with Δ the angle from the direction towards the origin.
    for (int i = 0; i < kN; ++i) {
      const double psi = pi * ((i + 0.5) / kN - 0.5), sp = std::sin(psi), cp = std::cos(psi);
      const double cd = std::sqrt(std::max(0.0, 1.0 - sp * sp / (k * k)));  // cos Δ
      fx += 2.0 * cp * cp * cd;                                             // û = (cos Δ, sin Δ)
      torque += 2.0 * cp * cp * (-k) * cd;
      if (cd > 0.0) torque += (2.0 * cp * cp * cp + 6.0 * k * k * cd * cd * cp) / 3.0 * cp / (k * cd);
    }
    fx *= pi / kN;
    torque *= pi / kN;
  }
  return {fx / pi, torque / pi};
}

// The same disk with another axisymmetric pressure, by midpoint quadrature in polar coordinates
// about the disk's centre (unit radius, 48 × 96 points; the stop time is good to 1e-3):
// uniform wear, p ∝ 1/ρ, or the rigid punch on an elastic half-space, p ∝ 1/√(1 − ρ²). They
// bound the answers of class B; the uniform pressure above is the nominal theory.
enum class DiskPressure { kUniform, kUniformWear, kRigidPunch };

CoulombDisk coulomb_disk(double v, double w, DiskPressure pressure) {
  if (pressure == DiskPressure::kUniform) return coulomb_disk(v, w);
  constexpr int kRadial = 48, kAngular = 96;
  const double pi = std::numbers::pi;
  double fx = 0.0, torque = 0.0;
  for (int i = 0; i < kRadial; ++i) {
    // Radius and share of the load of the ring: wear, p·ρ dρ ∝ dρ; punch, with ρ = sin u,
    // p·ρ dρ ∝ sin u du.
    double rho = 0.0, weight = 0.0;
    if (pressure == DiskPressure::kUniformWear) {
      rho = (i + 0.5) / kRadial;
      weight = 1.0 / kRadial;
    } else {
      const double u = pi / 2 * (i + 0.5) / kRadial;
      rho = std::sin(u);
      weight = std::cos(pi / 2 * i / kRadial) - std::cos(pi / 2 * (i + 1) / kRadial);
    }
    for (int j = 0; j < kAngular; ++j) {
      const double phi = 2.0 * pi * (j + 0.5) / kAngular, x = rho * std::cos(phi), y = rho * std::sin(phi);
      const double ux = v - w * y, uy = w * x, norm = std::hypot(ux, uy);
      if (norm == 0.0) continue;
      fx += weight / kAngular * ux / norm;
      torque += weight / kAngular * (x * uy - y * ux) / norm;
    }
  }
  return {fx, torque};
}

// RK4 integration of m v̇ = F, I ω̇ = T (I = m r²/2) with the law above, until v and ωr
// fall below 1e-6 (or one passes zero and the other is already negligible).
ContensouTheory integrate_contensou(double v0, double w0, double r, double mu, DiskPressure pressure) {
  const auto rates = [&](double v, double w) {
    const CoulombDisk f = coulomb_disk(std::max(v, 0.0), std::max(w * r, 0.0), pressure);
    return std::pair{-mu * kG * f.force, -2.0 * mu * kG * f.torque / r};
  };
  ContensouTheory th{{0.0}, {v0}, {w0 * r}};
  double t = 0.0, v = v0, w = w0;
  while (std::max(v, w * r) > 1e-6) {
    const double dt = std::min(1e-3, 0.1 * std::max(v, w * r) / (mu * kG));
    const auto [a1, b1] = rates(v, w);
    const auto [a2, b2] = rates(v + dt / 2 * a1, w + dt / 2 * b1);
    const auto [a3, b3] = rates(v + dt / 2 * a2, w + dt / 2 * b2);
    const auto [a4, b4] = rates(v + dt * a3, w + dt * b3);
    v = std::max(0.0, v + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4));
    w = std::max(0.0, w + dt / 6 * (b1 + 2 * b2 + 2 * b3 + b4));
    t += dt;
    th.t.push_back(t);
    th.v.push_back(v);
    th.wr.push_back(w * r);
  }
  return th;
}

ContensouTheory integrate_contensou(double v0, double w0, double r, double mu) {
  return integrate_contensou(v0, w0, r, mu, DiskPressure::kUniform);
}

namespace {

// The larger of a and b at each sample: it falls below a level once both do.
std::vector<double> larger(const std::vector<double>& a, const std::vector<double>& b) {
  std::vector<double> m(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) m[i] = std::max(a[i], b[i]);
  return m;
}

// v/(ωr) the first time √(v² + (ωr)²) = level; NaN if never.
double ratio_at(const std::vector<double>& t, const std::vector<double>& v, const std::vector<double>& wr,
                double level) {
  const auto n = [&](std::size_t i) { return std::hypot(v[i], wr[i]); };
  for (std::size_t i = 1; i < t.size(); ++i) {
    if (n(i) < level) {
      const double f = (n(i - 1) - level) / (n(i - 1) - n(i));
      const double vi = v[i - 1] + f * (v[i] - v[i - 1]), wi = wr[i - 1] + f * (wr[i] - wr[i - 1]);
      return vi / wi;
    }
  }
  return std::nan("");
}

double at(const ContensouTheory& th, const std::vector<double>& x, double t) {
  if (t >= th.t.back()) return 0.0;
  const std::size_t j = std::upper_bound(th.t.begin(), th.t.end(), t) - th.t.begin();
  const double f = (t - th.t[j - 1]) / (th.t[j] - th.t[j - 1]);
  return x[j - 1] + f * (x[j] - x[j - 1]);
}

}  // namespace

Scenario contensou(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.1, h = 0.02, mu = 0.3, K = 1e7, v0 = 1.0, w0 = 10.0;
  Scenario s;
  s.name = "contensou";
  s.description = "Disk sliding and spinning on a plane (Contensou), v0 = 1 m/s, w0 = 10 rad/s, mu = 0.3";
  s.theory_class = TheoryClass::kB;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 1.2;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "disk";
  body.shape = shape_meshes(o.data_dir, curved(o, "cylinder_r100_h20"));
  body.mass = m;
  body.inertia = cylinder_inertia(m, r, h);
  body.X_W0.p = Vec3(0.0, 0.0, h / 2 - kInitialPenetration);
  body.V_W0.v = Vec3(v0, 0.0, 0.0);
  body.V_W0.w = Vec3(0.0, 0.0, w0);
  body.primitive = Primitive{.kind = Primitive::Kind::kCylinder, .size = Vec3(r, h, 0.0), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  const ContensouTheory th = integrate_contensou(v0, w0, r, mu);
  // Stop times of the other admissible pressures, computed once per process.
  static const std::array<double, 2> band = [&] {
    std::array<double, 2> t{};
    const DiskPressure others[] = {DiskPressure::kUniformWear, DiskPressure::kRigidPunch};
    for (int k = 0; k < 2; ++k) {
      const ContensouTheory b = integrate_contensou(v0, w0, r, mu, others[k]);
      t[k] = crossing_time(b.t, larger(b.v, b.wr), 0.01 * v0);
    }
    return t;
  }();
  s.metrics = [th](const Trajectory& traj) {
    MetricReport rep;
    Series v{"v", {}}, wr{"wr", {}}, v_th{"v_theory", {}}, wr_th{"wr_theory", {}};
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const double t = traj.times()[i];
      v.values.push_back(std::hypot(traj.twist(i, 0).v.x(), traj.twist(i, 0).v.y()));
      wr.values.push_back(std::abs(traj.twist(i, 0).w.z()) * r);
      v_th.values.push_back(at(th, th.v, t));
      wr_th.values.push_back(at(th, th.wr, t));
    }
    const double t_stop = crossing_time(traj.times(), larger(v.values, wr.values), 0.01 * v0);
    const double t_theory = crossing_time(th.t, larger(th.v, th.wr), 0.01 * v0);
    ErrorTerms terms;
    terms.add("stop_time", relative_error(t_stop, t_theory));
    rep.scalars = {{"stop_time", t_stop},
                   {"stop_time_theory", t_theory},
                   {"ratio", ratio_at(traj.times(), v.values, wr.values, 0.1 * v0)},
                   {"ratio_theory", ratio_at(th.t, th.v, th.wr, 0.1 * v0)}};
    add_band(rep, "stop_time", t_stop, t_theory, {{"uniform_wear", band[0]}, {"rigid_punch", band[1]}});
    terms.finish(rep);
    rep.series = {std::move(v), std::move(wr), std::move(v_th), std::move(wr_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
