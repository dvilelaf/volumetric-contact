#include <array>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

// Mass properties of the solid frustum (radius r at z = 0, R at z = H) from 3-point Gauss
// quadrature, exact for the degree-4 integrands: I_z = (3/10) m (R⁵ − r⁵)/(R³ − r³) and the
// transverse inertia about the centroid.
struct Frustum {
  double i_z, i_t;
};
Frustum frustum_inertia(double m, double r, double R, double H) {
  constexpr std::array<double, 3> x = {-0.7745966692414834, 0.0, 0.7745966692414834};
  constexpr std::array<double, 3> w = {5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0};
  const auto integrate = [&](auto f) {
    double sum = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double z = 0.5 * H * (1.0 + x[i]);
      sum += 0.5 * H * w[i] * f(z, r + (R - r) * z / H);
    }
    return sum;
  };
  const double area_int = integrate([](double, double rho) { return rho * rho; });
  const double zc = integrate([](double z, double rho) { return z * rho * rho; }) / area_int;
  const double density = m / (std::numbers::pi * area_int);  // per unit disc area·dz
  const double i_t = density * std::numbers::pi *
                     integrate([&](double z, double rho) {
                       return std::pow(rho, 4) / 4 + rho * rho * (z - zc) * (z - zc);
                     });
  return {0.3 * m * (std::pow(R, 5) - std::pow(r, 5)) / (std::pow(R, 3) - std::pow(r, 3)), i_t};
}

}  // namespace

Scenario cone(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.05, R = 0.1, H = 0.1, mu = 0.3, K = 1e6, w0 = 60.0;
  const double sin_a = (R - r) / std::hypot(R - r, H);  // half-angle α = atan((R − r)/H)
  const Frustum inertia = frustum_inertia(m, r, R, H);
  Scenario s;
  s.name = "cone";
  s.description = "Frustum spinning at 60 rad/s in a conical seat, mu = 0.3";
  s.theory_class = TheoryClass::kB;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 0.8;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "frustum";
  body.shape = shape_meshes(o.data_dir, curved(o, "frustum_r50_r100_h100"));
  body.mass = m;
  body.inertia = Vec3(inertia.i_t, inertia.i_t, inertia.i_z).asDiagonal();
  // Seated, then 1 µm along the wall normal: the axial displacement is that over sin α.
  body.X_W0.p = Vec3(0.0, 0.0, -kInitialPenetration / sin_a);
  body.V_W0.w = Vec3(0.0, 0.0, w0);

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, curved(o, "seat_r50_r100_h100_t20"))});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  // Winkler: the axial load m·g spreads as a uniform pressure p = mg/(π(R² − r²)) over the lateral
  // area π(R² − r²)/sin α, so the wall normal depth is p/K and the axial sinking that over sin α.
  // Friction is circumferential: τ = μ·p·∫ρ dA = (2/3)μmg(R³ − r³)/((R² − r²) sin α), and ω decays
  // linearly at τ/I_z; the 1 % stop comes at 0.99 of the full stop time.
  const double sink_theory = m * kG / (K * std::numbers::pi * (R * R - r * r) * sin_a);
  const double tau = 2.0 / 3.0 * mu * m * kG * (std::pow(R, 3) - std::pow(r, 3)) / ((R * R - r * r) * sin_a);
  // Uniform wear (p ∝ 1/ρ) gives τ = μ·m·g·(R + r)/(2 sin α), 3.6 % less.
  const double tau_wear = mu * m * kG * (R + r) / (2.0 * sin_a);
  const double alpha = tau / inertia.i_z;
  const double t_full = w0 / alpha, t_theory = 0.99 * t_full;
  s.metrics = [=](const Trajectory& traj) {
    MetricReport rep;
    Series w_s{"w", {}}, w_th{"w_theory", {}};
    std::vector<double> speed, sink;
    double tilt = 0.0, drift = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      w_s.values.push_back(traj.twist(i, 0).w.z());
      w_th.values.push_back(std::max(0.0, w0 - alpha * traj.times()[i]));
      speed.push_back(std::abs(w_s.values.back()));
      sink.push_back(-X.p.z());
      tilt = std::max(tilt, axis_deviation(X, 2));
      drift = std::max(drift, std::hypot(X.p.x(), X.p.y()));
    }
    const double t_stop = crossing_time(traj.times(), speed, 0.01 * speed.front());
    ErrorTerms terms;
    terms.add("stop_time", relative_error(t_stop, t_theory));
    rep.scalars = {{"stop_time", t_stop},
                   {"stop_time_theory", t_theory},
                   {"full_stop_theory", t_full},
                   {"sinking", mean_after(traj.times(), sink, 0.2)},
                   {"sinking_theory", sink_theory},
                   {"axis_tilt_max_rad", tilt},
                   {"axis_tilt_max_rad_theory", 0.0},
                   {"drift_max_m", drift}};
    add_band(rep, "stop_time", t_stop, t_theory, {{"uniform_wear", t_theory * tau / tau_wear}});
    terms.finish(rep);
    rep.series = {std::move(w_s), std::move(w_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
