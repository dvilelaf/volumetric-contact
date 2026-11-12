#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {

// A 1 kg cube sinking into a through-hole 0.2 mm narrower than itself: each side face
// overlaps its wall by δ, so friction μK·δ·a·z per wall slows it down as it goes deeper.
Scenario hole(const ScenarioOptions& o) {
  constexpr double m = 1.0, a = 0.1, delta = 1e-4, mu = 0.5, K = 1e8;
  constexpr double k = 4.0 * mu * K * delta * a;  // m z̈ = m g − k z while it slides.
  constexpr double z_stop = 2.0 * m * kG / k;
  const double t_max = std::numbers::pi / std::sqrt(k / m);
  Scenario s;
  s.name = "hole";
  s.description = "Cube sinking into a square through-hole 0.2 mm narrower, mu = 0.5";
  s.theory_class = TheoryClass::kC;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 0.5;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "cube";
  body.shape = shape_meshes(o.data_dir, "box_100x100x100");
  body.mass = m;
  body.inertia = box_inertia(m, a, a, a);
  body.X_W0.p = Vec3(0.0, 0.0, a / 2);  // Bottom face at the entry.

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, "hole_s99.8_d100")});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  s.metrics = [=](const Trajectory& traj) {
    MetricReport r;
    Series depth{"depth", {}}, depth_th{"depth_theory", {}};
    std::vector<double> speed;
    double drift = 0.0, t_stop = std::nan("");
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const double t = traj.times()[i];
      const Pose& X = traj.pose(i, 0);
      depth.values.push_back(a / 2 - X.p.z());
      // Theory: z = z_stop (1 − cos ωt)/2 until the half period, then held.
      depth_th.values.push_back(0.5 * z_stop * (1.0 - std::cos(std::min(t, t_max) * std::sqrt(k / m))));
      drift = std::max(drift, std::hypot(X.p.x(), X.p.y()));
      speed.push_back(traj.twist(i, 0).v.norm());
      // First time the cube, once sinking, stops: linear crossing of vz = 0.
      if (std::isnan(t_stop) && i > 0 && depth.values.back() > 1e-3 && traj.twist(i, 0).v.z() >= 0.0) {
        const double v0 = traj.twist(i - 1, 0).v.z(), v1 = traj.twist(i, 0).v.z();
        t_stop = v0 < v1 ? traj.times()[i - 1] + (traj.times()[i] - traj.times()[i - 1]) * (-v0) / (v1 - v0) : t;
      }
    }
    const double final_depth = mean_after(traj.times(), depth.values, s.duration - 0.1);
    const double residual = mean_after(traj.times(), speed, s.duration - 0.1);
    // The theory fixes the depth at which the cube stops and that it stays there.
    ErrorTerms terms;
    terms.add("depth", relative_error(final_depth, z_stop));
    terms.add_residual_speed(residual, s.duration, a);
    r.scalars = {{"depth_final_m", final_depth},
                 {"depth_final_m_theory", z_stop},
                 {"depth_max_m", max_abs(depth.values)},
                 {"depth_max_m_theory", z_stop},
                 {"time_max_depth", t_stop},
                 {"time_max_depth_theory", t_max},
                 {"drift_max_m", drift},
                 {"residual_speed_mps", residual},
                 {"residual_speed_mps_theory", 0.0}};
    terms.finish(r);
    r.series = {std::move(depth), std::move(depth_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
