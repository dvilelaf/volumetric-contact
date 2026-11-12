#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {

Scenario sliding_block(const ScenarioOptions& o, double angle_deg, double duration) {
  constexpr double m = 1.0, lx = 1.0, ly = 0.5, lz = 0.5, mu = 0.5, K = 1e6;
  Scenario s;
  s.name = "block";
  s.description = "Block sliding on a plane inclined " + std::to_string(angle_deg) + " deg, mu = 0.5";
  s.theory_class = TheoryClass::kA;
  s.gravity = inclined_gravity(angle_deg);
  s.duration = duration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "block";
  body.shape = shape_meshes(o.data_dir, "box_1000x500x500");
  body.mass = m;
  body.inertia = box_inertia(m, lx, ly, lz);
  body.X_W0.p = Vec3(0.0, 0.0, lz / 2 - kInitialPenetration);
  body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(lx, ly, lz), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  const double a = angle_deg * std::numbers::pi / 180.0;
  const double accel = std::max(0.0, kG * (std::sin(a) - mu * std::cos(a)));
  s.metrics = [accel](const Trajectory& traj) {
    MetricReport r;
    Series v{"v", {}}, v_th{"v_theory", {}}, y{"lateral", {}};
    double angular = 0.0, v_err = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      v.values.push_back(traj.twist(i, 0).v.x());
      v_th.values.push_back(accel * traj.times()[i]);
      y.values.push_back(X.p.y());
      v_err = std::max(v_err, std::abs(v.values.back() - v_th.values.back()));
      for (int k = 0; k < 3; ++k) angular = std::max(angular, axis_deviation(X, k));
    }
    const double v_end = v.values.back(), v_th_end = v_th.values.back();
    ErrorTerms terms;
    terms.add("velocity", relative_error(v_end, v_th_end));
    r.scalars = {{"velocity_end", v_end},
                 {"velocity_end_theory", v_th_end},
                 {"lateral_error_max_m", max_abs(y.values)},
                 {"velocity_error_max_mps", v_err},
                 {"angular_deviation_max_rad", angular}};
    terms.finish(r);
    r.series = {std::move(v), std::move(v_th), std::move(y)};
    return r;
  };
  return s;
}

Scenario block(const ScenarioOptions& o) { return sliding_block(o); }

}  // namespace volcontact::scenarios
