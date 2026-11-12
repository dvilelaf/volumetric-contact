#include <numbers>

#include "rolling.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kRadius = 0.1, kMu = 0.5, kK = 1e8, kAngleDeg = 20.0;
constexpr double kDuration = 1.5;
constexpr double kAngle = kAngleDeg * std::numbers::pi / 180.0;

}  // namespace

// A sphere rolling down a 20° incline from rest.
Scenario sphere(const ScenarioOptions& o) {
  Scenario s;
  s.name = "sphere";
  s.description = "Sphere rolling down a 20 deg incline from rest, mu = 0.5";
  s.theory_class = TheoryClass::kA;
  s.gravity = inclined_gravity(kAngleDeg);
  s.duration = kDuration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "sphere";
  body.shape = shape_meshes(o.data_dir, curved(o, "sphere_r100"));
  body.mass = kMass;
  body.inertia = sphere_inertia(kMass, kRadius);
  body.X_W0.p = Vec3(0.0, 0.0, kRadius - kInitialPenetration);
  body.primitive = Primitive{.kind = Primitive::Kind::kSphere, .size = Vec3(kRadius, 0.0, 0.0), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  // Rolling without slipping from rest: a = (5/7)·g·sin θ. Winkler rolling resistance
  // (rolling.hpp): cap of radius a, a² = 2·r·δ, M = N·h·ω·a²/6.
  const double N = kMass * kG * std::cos(kAngle);
  const double delta = sphere_static_penetration(kMass, kG * std::cos(kAngle), kK, kRadius);
  const RollingTheory theory{
      .accel = 5.0 / 7.0 * kG * std::sin(kAngle),
      .winkler_rate = N * kDamping * 2.0 * kRadius * delta / (6.0 * kMass * kRadius * kRadius * 1.4)};
  s.metrics = [theory](const Trajectory& traj) {
    Series v{"v", {}}, wr{"wr", {}}, y{"lateral", {}};
    for (std::size_t i = 0; i < traj.size(); ++i) {
      v.values.push_back(traj.twist(i, 0).v.x());
      wr.values.push_back(traj.twist(i, 0).w.y() * kRadius);
      y.values.push_back(traj.pose(i, 0).p.y());
    }
    return rolling_metrics(theory, traj.times(), std::move(v), std::move(wr), std::move(y));
  };
  return s;
}

}  // namespace volcontact::scenarios
