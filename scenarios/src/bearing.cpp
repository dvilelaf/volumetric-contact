#include <numbers>

#include <Eigen/Geometry>

#include "common.hpp"

namespace volcontact::scenarios {

Scenario bearing(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.05, l = 0.2, mu = 0.2, K = 1e6, w0 = 40.0;
  constexpr double half_pi = std::numbers::pi / 2;
  Scenario s;
  s.name = "bearing";
  s.description = "Shaft spinning at 40 rad/s in a bearing hole, mu = 0.2";
  s.theory_class = TheoryClass::kB;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 0.8;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "shaft";
  body.shape = shape_meshes(o.data_dir, curved(o, "cylinder_r50_h200"));
  body.mass = m;
  body.inertia = cylinder_inertia(m, r, l);
  // Body axis z lies along the world y axis, concentric with the hole.
  body.X_W0.R = Eigen::AngleAxisd(-half_pi, Vec3::UnitX()).toRotationMatrix();
  body.V_W0.w = Vec3(0.0, w0, 0.0);
  body.primitive = Primitive{.kind = Primitive::Kind::kCylinder, .size = Vec3(r, l, 0.0), .X_BG = {}};

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, curved(o, "bearing_r50_t20_l200"))});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  // Winkler: pressure p0·cos(φ) over the lower half, so the load is K·e·r·L·π/2 = mg. The contact
  // force is vertical at atan(μ) from the bottom, where the friction torque is
  // τ = (4/π)·μ·m·g·r/√(1 + μ²). ω decays linearly and stops at I·ω0/τ; it falls below 1 % of ω0
  // at 0.99 of that. A journal without clearance is statically indeterminate, so the factor 4/π
  // is the Winkler value: with the same friction circle a line contact gives 1, a Hertz-like
  // √cos pressure 1.37 and a uniform pressure over the lower half π/2. The stop time goes as
  // the inverse of the factor.
  const double tau = 4.0 / std::numbers::pi * mu * m * kG * r / std::sqrt(1.0 + mu * mu);
  const double inertia = m * r * r / 2;
  const double alpha = tau / inertia;
  const double t_full = w0 / alpha, t_theory = 0.99 * t_full;
  const double angle_theory = std::atan(mu), sink_theory = m * kG / (K * r * l * half_pi);
  s.metrics = [=](const Trajectory& traj) {
    MetricReport rep;
    Series w{"w", {}}, w_th{"w_theory", {}}, angle{"angle", {}}, angle_th{"angle_theory", {}};
    std::vector<double> speed, sink;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      const double t = traj.times()[i];
      w.values.push_back(traj.twist(i, 0).w.y());
      w_th.values.push_back(std::max(0.0, w0 - alpha * t));
      speed.push_back(std::abs(w.values.back()));
      // Climb angle of the shaft centre from the bottom of the hole, towards the side it rolls to.
      angle.values.push_back(std::atan2(std::abs(X.p.x()), -X.p.z()));
      angle_th.values.push_back(angle_theory);
      sink.push_back(std::hypot(X.p.x(), X.p.z()));
    }
    const double t_stop = crossing_time(traj.times(), speed, 0.01 * speed.front());
    // Steady state: after the transient and before the spin dies out.
    const double t0 = 0.25 * t_full, t1 = 0.9 * t_full;
    std::vector<double> t_in, angle_in, sink_in;
    for (std::size_t i = 0; i < speed.size(); ++i) {
      if (traj.times()[i] >= t0 && traj.times()[i] <= t1) {
        t_in.push_back(traj.times()[i]);
        angle_in.push_back(angle.values[i]);
        sink_in.push_back(sink[i]);
      }
    }
    ErrorTerms terms;
    terms.add("stop_time", relative_error(t_stop, t_theory));
    rep.scalars = {{"stop_time", t_stop},
                   {"stop_time_theory", t_theory},
                   {"full_stop_theory", t_full},
                   {"angle", mean_after(t_in, angle_in, 0.0)},
                   {"angle_theory", angle_theory},
                   {"sink", mean_after(t_in, sink_in, 0.0)},
                   {"sink_theory", sink_theory}};
    const double factor = 4.0 / std::numbers::pi;
    add_band(rep, "stop_time", t_stop, t_theory,
             {{"line_contact", t_theory * factor},
              {"sqrt_cosine", t_theory * factor / 1.3708},
              {"uniform", t_theory * factor / half_pi}});
    terms.finish(rep);
    rep.series = {std::move(w), std::move(w_th), std::move(angle), std::move(angle_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
