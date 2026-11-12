#include <numbers>

#include <Eigen/Geometry>

#include "common.hpp"

namespace volcontact::scenarios {

// The shaft of `bearing` in a hole 10 mm larger in radius. With clearance the contact is a
// line, so the answer does not depend on a pressure distribution: the shaft climbs the wall
// to the angle atan μ, where the contact force balances the weight, and the friction torque is
// μ·m·g·r/√(1 + μ²). Clearance and stiffness follow from the resolution: the contact arc must be
// narrow (half-angle φm = a/R ≈ 0.26 rad, so the friction torque of a parabolic pressure is
// 1 + φm²/10 = 1.007 times the line value) and the penetration (0.3 mm) larger than the sag of
// the two 64-segment polygons (0.13 mm), whose facets pass under the contact.
Scenario journal(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.05, l = 0.2, R = 0.06, mu = 0.2, K = 1e7, w0 = 40.0;
  constexpr double half_pi = std::numbers::pi / 2;
  Scenario s;
  s.name = "journal";
  s.description = "Shaft spinning at 40 rad/s in a hole with 10 mm of radial clearance, mu = 0.2";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 0.8;

  const double angle_theory = std::atan(mu);
  BodySpec& body = s.bodies.emplace_back();
  body.name = "shaft";
  body.shape = shape_meshes(o.data_dir, curved(o, "cylinder_r50_h200"));
  body.mass = m;
  body.inertia = cylinder_inertia(m, r, l);
  // Body axis z lies along the world y axis. The shaft starts where the theory keeps it: on the
  // wall at atan μ from the bottom, on the side its spin climbs to (+x), 1 µm into the true wall.
  body.X_W0.R = Eigen::AngleAxisd(-half_pi, Vec3::UnitX()).toRotationMatrix();
  body.X_W0.p = (R - r + kInitialPenetration) * Vec3(std::sin(angle_theory), 0.0, -std::cos(angle_theory));
  body.V_W0.w = Vec3(0.0, w0, 0.0);
  body.primitive = Primitive{.kind = Primitive::Kind::kCylinder, .size = Vec3(r, l, 0.0), .X_BG = {}};

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, curved(o, "bearing_r60_t20_l200"))});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  // ω decays linearly at τ/I and stops at I·ω0/τ; it falls below 1 % of ω0 at 0.99 of that.
  const double normal = m * kG / std::sqrt(1.0 + mu * mu);
  const double tau = mu * normal * r;
  const double alpha = tau / (m * r * r / 2);
  const double t_full = w0 / alpha, t_theory = 0.99 * t_full;
  // Winkler: half-width a of the contact strip, a² = 2·r_rel·δ with r_rel = rR/(R − r).
  const double r_rel = r * R / (R - r);
  const double arc = std::sqrt(2.0 * r_rel * cylinder_static_penetration(normal, K, r_rel, l)) / R;
  s.metrics = [=](const Trajectory& traj) {
    MetricReport rep;
    Series w{"w", {}}, w_th{"w_theory", {}}, angle{"angle", {}}, angle_th{"angle_theory", {}};
    std::vector<double> speed;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      const double t = traj.times()[i];
      w.values.push_back(traj.twist(i, 0).w.y());
      w_th.values.push_back(std::max(0.0, w0 - alpha * t));
      speed.push_back(std::abs(w.values.back()));
      // Angle of the shaft centre from the bottom of the hole, positive towards +x.
      angle.values.push_back(std::atan2(X.p.x(), -X.p.z()));
      angle_th.values.push_back(angle_theory);
    }
    const double t_stop = crossing_time(traj.times(), speed, 0.01 * speed.front());
    // Climb angle while the shaft slides: mean between 10 % and 90 % of the theory's stop time.
    double sum = 0.0;
    int n = 0;
    for (std::size_t i = 0; i < speed.size(); ++i) {
      if (traj.times()[i] >= 0.1 * t_full && traj.times()[i] <= 0.9 * t_full) {
        sum += angle.values[i];
        ++n;
      }
    }
    const double climb = n ? sum / n : std::nan("");
    // The theory fixes the stop time and the angle at which the shaft rides.
    ErrorTerms terms;
    terms.add("stop_time", relative_error(t_stop, t_theory));
    terms.add("angle", relative_error(climb, angle_theory));
    rep.scalars = {{"stop_time", t_stop},
                   {"stop_time_theory", t_theory},
                   {"stop_time_winkler_arc", t_theory / (1.0 + arc * arc / 10.0)},
                   {"full_stop_theory", t_full},
                   {"angle", climb},
                   {"angle_theory", angle_theory}};
    terms.finish(rep);
    rep.series = {std::move(w), std::move(w_th), std::move(angle), std::move(angle_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
