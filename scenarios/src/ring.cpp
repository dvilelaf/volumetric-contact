#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {

Scenario ring(const ScenarioOptions& o) {
  constexpr double m = 1.0, R = 0.25, r = 0.15, h = 0.05, mu = 0.6, K = 1e6;
  constexpr double w0 = 5.0 * std::numbers::pi;
  constexpr double sum_sq = R * R + r * r;
  Scenario s;
  s.name = "ring";
  s.description = "Thick ring spinning on a plane, w0 = 5 pi rad/s, mu = 0.6";
  s.theory_class = TheoryClass::kB;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 1.0;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "ring";
  body.shape = shape_meshes(o.data_dir, curved(o, "ring_ro250_ri150_h50"));
  body.mass = m;
  // Annulus: I_z = m(R² + r²)/2, I_x = I_y = m(3(R² + r²) + h²)/12.
  const double t_xy = m * (3 * sum_sq + h * h) / 12;
  body.inertia = Vec3(t_xy, t_xy, m * sum_sq / 2).asDiagonal();
  body.X_W0.p = Vec3(0.0, 0.0, h / 2 - kInitialPenetration);
  body.V_W0.w = Vec3(0.0, 0.0, w0);

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  // Uniform pressure over the annulus: τ = (2/3)·μ·m·g·(R³ − r³)/(R² − r²). ω decays linearly
  // at α = τ/I_z and stops at t = I_z·ω0/τ. The speed falls below 1 % of ω0 at 0.99 of that.
  // Uniform wear (p ∝ 1/ρ) gives τ = μ·m·g·(R + r)/2, 2 % less.
  const double tau = 2.0 / 3.0 * mu * m * kG * (R * R * R - r * r * r) / (R * R - r * r);
  const double tau_wear = mu * m * kG * (R + r) / 2.0;
  const double alpha = tau / (m * sum_sq / 2);
  const double t_full = w0 / alpha, t_theory = 0.99 * t_full;
  s.metrics = [alpha, w0, t_full, t_theory, wear = tau / tau_wear](const Trajectory& traj) {
    MetricReport rep;
    Series w{"w", {}}, w_th{"w_theory", {}};
    std::vector<double> speed;
    double drift = 0.0, tilt = 0.0, w_err = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      const double t = traj.times()[i];
      w.values.push_back(traj.twist(i, 0).w.z());
      w_th.values.push_back(std::max(0.0, w0 - alpha * t));
      w_err = std::max(w_err, std::abs(w.values.back() - w_th.values.back()));
      drift = std::max(drift, std::hypot(X.p.x(), X.p.y()));
      tilt = std::max(tilt, axis_deviation(X, 2));
      speed.push_back(std::abs(w.values.back()));
    }
    const double t_stop = crossing_time(traj.times(), speed, 0.01 * speed.front());
    ErrorTerms terms;
    terms.add("stop_time", relative_error(t_stop, t_theory));
    rep.scalars = {{"stop_time", t_stop},
                   {"stop_time_theory", t_theory},
                   {"full_stop_theory", t_full},
                   {"omega_error_max_radps", w_err},
                   {"drift_max_m", drift},
                   {"axis_tilt_max_rad", tilt}};
    add_band(rep, "stop_time", t_stop, t_theory, {{"uniform_wear", t_theory * wear}});
    terms.finish(rep);
    rep.series = {std::move(w), std::move(w_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
