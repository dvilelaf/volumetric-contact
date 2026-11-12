#include <numbers>

#include <Eigen/Geometry>

#include "rolling.hpp"

namespace volcontact::scenarios {

Scenario cylinder(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.25, l = 1.0, angle_deg = 15.0, mu = 0.6, K = 1e6;
  Scenario s;
  s.name = "cylinder";
  s.description = "Cylinder rolling down a 15 deg incline";
  s.theory_class = TheoryClass::kA;
  s.gravity = inclined_gravity(angle_deg);
  s.duration = 3.5;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "cylinder";
  body.shape = shape_meshes(o.data_dir, curved(o, "cylinder_r250_h1000"));
  body.mass = m;
  body.inertia = cylinder_inertia(m, r, l);
  // Body axis z lies along the world y axis.
  body.X_W0.R = Eigen::AngleAxisd(-std::numbers::pi / 2, Vec3::UnitX()).toRotationMatrix();
  body.X_W0.p = Vec3(0.0, 0.0, r - kInitialPenetration);
  body.primitive = Primitive{.kind = Primitive::Kind::kCylinder, .size = Vec3(r, l, 0.0), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  // Solid cylinder rolling without slipping: a = (2/3)·g·sin(θ). Winkler rolling resistance
  // (rolling.hpp): strip of half-width a, a² = 2·r·δ, M = N·h·ω·a²/5.
  const double angle = angle_deg * std::numbers::pi / 180.0, N = m * kG * std::cos(angle);
  const double delta = cylinder_static_penetration(N, K, r, l);
  const RollingTheory theory{.accel = 2.0 * kG * std::sin(angle) / 3.0,
                             .winkler_rate = N * kDamping * 2.0 * r * delta / (5.0 * m * r * r * 1.5)};
  s.metrics = [theory](const Trajectory& traj) {
    Series v{"v", {}}, wr{"wr", {}}, y{"lateral", {}};
    double axis = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      v.values.push_back(traj.twist(i, 0).v.x());
      wr.values.push_back(traj.twist(i, 0).w.y() * r);
      y.values.push_back(X.p.y());
      axis = std::max(axis, std::acos(std::clamp(X.R(1, 2), -1.0, 1.0)));  // Body z against world y.
    }
    MetricReport rep = rolling_metrics(theory, traj.times(), std::move(v), std::move(wr), std::move(y));
    rep.scalars.emplace_back("axis_deviation_max_rad", axis);
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
