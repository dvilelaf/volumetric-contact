#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {

Scenario disk(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.25, h = 0.05, mu = 0.6, K = 1e6;
  constexpr double w0 = 5.0 * std::numbers::pi;
  Scenario s;
  s.name = "disk";
  s.description = "Disk spinning on a plane, w0 = 5 pi rad/s, mu = 0.6";
  s.theory_class = TheoryClass::kB;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 1.5;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "disk";
  body.shape = shape_meshes(o.data_dir, curved(o, "cylinder_r250_h50"));
  body.mass = m;
  body.inertia = cylinder_inertia(m, r, h);
  body.X_W0.p = Vec3(0.0, 0.0, h / 2 - kInitialPenetration);
  body.V_W0.w = Vec3(0.0, 0.0, w0);
  body.primitive = Primitive{.kind = Primitive::Kind::kCylinder, .size = Vec3(r, h, 0.0), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  // Uniform pressure (a rigid punch on a Winkler foundation): τ = (2/3)·μ·m·g·R, so ω stops at
  // t = 3·R·ω0 / (4·μ·g). The speed falls below 1 % of ω0 at 0.99 of that. Other admissible
  // pressures give τ = f·μ·m·g·R with f = 1/2 (uniform wear, p ∝ 1/ρ) and f = π/4 (rigid punch
  // on an elastic half-space, p ∝ 1/√(1 − ρ²/R²)); the stop time goes as 1/f.
  const double alpha = 4.0 * mu * kG / (3.0 * r);
  s.metrics = [alpha](const Trajectory& traj) {
    MetricReport rep;
    Series w{"w", {}}, w_th{"w_theory", {}};
    double drift = 0.0, tilt = 0.0, w_err = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      const double t = traj.times()[i];
      w.values.push_back(traj.twist(i, 0).w.z());
      w_th.values.push_back(std::max(0.0, w0 - alpha * t));
      w_err = std::max(w_err, std::abs(w.values.back() - w_th.values.back()));
      drift = std::max(drift, std::hypot(X.p.x(), X.p.y()));
      tilt = std::max(tilt, axis_deviation(X, 2));
    }
    std::vector<double> speed;
    for (double x : w.values) speed.push_back(std::abs(x));
    const double t_stop = crossing_time(traj.times(), speed, 0.01 * speed.front()), t_theory = 0.99 * w0 / alpha;
    ErrorTerms terms;
    terms.add("stop_time", relative_error(t_stop, t_theory));
    rep.scalars = {{"stop_time", t_stop},
                   {"stop_time_theory", t_theory},
                   {"omega_error_max_radps", w_err},
                   {"drift_max_m", drift},
                   {"axis_tilt_max_rad", tilt}};
    add_band(rep, "stop_time", t_stop, t_theory,
             {{"uniform_wear", t_theory * (2.0 / 3.0) / 0.5},
              {"rigid_punch", t_theory * (2.0 / 3.0) / (std::numbers::pi / 4.0)}});
    terms.finish(rep);
    rep.series = {std::move(w), std::move(w_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
