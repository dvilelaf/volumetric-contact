#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kMu = 0.2, kK = 1e6, kAngleDeg = 30.0, kX0 = -1.5;
constexpr double kAngle = kAngleDeg * std::numbers::pi / 180.0;
// Rotation about x that puts the bar's lower edge on the groove's bottom line.
constexpr double kRoll = std::numbers::pi / 4;

}  // namespace

// A 0.4 m bar sliding down a 90° V-groove at 30° along the groove.
Scenario vgroove(const ScenarioOptions& o) {
  Scenario s;
  s.name = "vgroove";
  s.description = "Bar sliding down a 90 deg V-groove, at 30 deg along the groove, mu = 0.2";
  s.theory_class = TheoryClass::kA;
  s.gravity = inclined_gravity(kAngleDeg);
  s.duration = 1.0;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "bar";
  body.shape = shape_meshes(o.data_dir, "box_400x100x100");
  body.mass = kMass;
  body.inertia = box_inertia(kMass, 0.4, 0.1, 0.1);
  // The centre lies on the bisector, half a diagonal above the edge; moving it
  // by −√2·1 µm in z puts each lower face 1 µm into its groove face.
  body.X_W0.R = Eigen::AngleAxisd(kRoll, Vec3::UnitX()).toRotationMatrix();
  body.X_W0.p = Vec3(kX0, 0.0, std::sqrt(2.0) * (0.05 - kInitialPenetration));
  body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(0.4, 0.1, 0.1), .X_BG = {}};

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, "vgroove_w300_l4000")});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  // Each face carries m g cos θ / (2 cos 45°), so friction is √2 μ m g cos θ in total.
  const double accel = kG * (std::sin(kAngle) - std::sqrt(2.0) * kMu * std::cos(kAngle));
  s.metrics = [accel](const Trajectory& traj) {
    MetricReport r;
    Series v{"v", {}}, v_th{"v_theory", {}};
    double lateral = 0.0, roll = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      v.values.push_back(traj.twist(i, 0).v.x());
      v_th.values.push_back(accel * traj.times()[i]);
      lateral = std::max(lateral, std::abs(X.p.y()));
      roll = std::max(roll, std::abs(std::atan2(X.R(2, 1), X.R(1, 1)) - kRoll));
    }
    const double v_end = v.values.back(), v_th_end = v_th.values.back();
    ErrorTerms terms;
    terms.add("velocity", relative_error(v_end, v_th_end));
    r.scalars = {{"velocity_end", v_end},
                 {"velocity_end_theory", v_th_end},
                 {"lateral_drift_max_m", lateral},
                 {"lateral_drift_max_m_theory", 0.0},
                 {"roll_max_rad", roll},
                 {"roll_max_rad_theory", 0.0}};
    terms.finish(r);
    r.series = {std::move(v), std::move(v_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
