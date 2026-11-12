#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kBallMass = 1.0, kBallRadius = 0.1, kTableMass = 100.0, kTableRadius = 0.5, kTableHeight = 0.1;
constexpr double kMu = 0.5, kBallK = 1e8, kGroundK = 1e7;
// Friction of the table on the plane: as good as none (it takes 0.1 % of the table's spin in
// the run), but not zero, which Drake's per-geometry friction cannot combine with the ball's 0.5.
constexpr double kGroundMu = 1e-5;
constexpr double kTableRate = 7.0;  // Ω [rad/s]
constexpr double kOrbitRadius = 0.2, kDuration = 3.5, kSettle = 0.2;

}  // namespace

// A ball that rolls without slipping on a table turning at Ω moves in a circle at the rate
// Ω·I/(I + m·a²) = (2/7)·Ω, whatever its spin about the vertical. The table here is a free body
// on a frictionless plane, so it recoils: with the ball's position taken from the table's axis,
// the equations are the same with the reduced mass m·M/(m + M) in place of m, and the rate is
// Ω/(1 + (5/2)·M/(m + M)).
double turntable_orbit_ratio(double ball_mass, double table_mass) {
  return 1.0 / (1.0 + 2.5 * table_mass / (ball_mass + table_mass));
}

Scenario turntable(const ScenarioOptions& o) {
  Scenario s;
  s.name = "turntable";
  s.description = "Ball rolling without slipping on a table turning at 7 rad/s, mu = 0.5";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  // The circular orbit about the table's axis, with the pair's centre of mass at rest at the
  // origin: relative speed ω_orb·ρ, and the ball's spin such that its contact point moves with
  // the table (Ω·ρ there). The spin about the vertical is the table's, so nothing pivots.
  const double ratio = turntable_orbit_ratio(kBallMass, kTableMass), share = kTableMass / (kBallMass + kTableMass);
  const double orbit_speed = ratio * kTableRate * kOrbitRadius;

  BodySpec& table = s.bodies.emplace_back();
  table.name = "table";
  table.shape = shape_meshes(o.data_dir, curved(o, "cylinder_r500_h100"));
  table.mass = kTableMass;
  table.inertia = cylinder_inertia(kTableMass, kTableRadius, kTableHeight);
  table.X_W0.p = Vec3(-(1.0 - share) * kOrbitRadius, 0.0, kTableHeight / 2 - kInitialPenetration);
  table.V_W0.v = Vec3(0.0, -(1.0 - share) * orbit_speed, 0.0);
  table.V_W0.w = Vec3(0.0, 0.0, kTableRate);
  table.primitive = Primitive{.kind = Primitive::Kind::kCylinder, .size = Vec3(kTableRadius, kTableHeight, 0.0), .X_BG = {}};

  BodySpec& ball = s.bodies.emplace_back();
  ball.name = "ball";
  ball.shape = shape_meshes(o.data_dir, curved(o, "sphere_r100"));
  ball.mass = kBallMass;
  ball.inertia = sphere_inertia(kBallMass, kBallRadius);
  ball.X_W0.p = Vec3(share * kOrbitRadius, 0.0, kTableHeight + kBallRadius - 2 * kInitialPenetration);
  ball.V_W0.v = Vec3(0.0, share * orbit_speed, 0.0);
  ball.V_W0.w = Vec3((1.0 - ratio) * kTableRate * kOrbitRadius / kBallRadius, 0.0, kTableRate);
  ball.primitive = Primitive{.kind = Primitive::Kind::kSphere, .size = Vec3(kBallRadius, 0.0, 0.0), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kGroundK, kDamping, kGroundMu));
  s.contacts.push_back(standard_contact(BodyRef::body(1), BodyRef::body(0), kBallK, kDamping, kMu));
  s.native = {.stiffness = kBallK, .damping = kDamping};

  s.metrics = [ratio](const Trajectory& traj) {
    MetricReport r;
    const std::vector<double>& t = traj.times();
    Series angle{"orbit_angle", {}}, angle_th{"orbit_angle_theory", {}}, radius{"orbit_radius", {}};
    std::vector<double> table_rate, radius_error;
    double turns = 0.0, previous = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Vec3 rel = traj.pose(i, 1).p - traj.pose(i, 0).p;
      const double a = std::atan2(rel.y(), rel.x());
      if (i > 0 && a - previous > std::numbers::pi) turns -= 1.0;
      if (i > 0 && a - previous < -std::numbers::pi) turns += 1.0;
      previous = a;
      angle.values.push_back(a + 2.0 * std::numbers::pi * turns);
      angle_th.values.push_back(ratio * kTableRate * t[i]);
      radius.values.push_back(rel.head<2>().norm());
      radius_error.push_back(radius.values.back() - kOrbitRadius);
      table_rate.push_back(traj.twist(i, 0).w.z());
    }
    // Rate of the orbit: slope of the least-squares line through the angle after the settling
    // (the parabola's p1 with its p2 taken out would be the rate at t = 0; the line is the mean).
    double st = 0.0, sa = 0.0, stt = 0.0, sta = 0.0, n = 0.0;
    for (std::size_t i = 0; i < t.size(); ++i) {
      if (t[i] < kSettle) continue;
      st += t[i];
      sa += angle.values[i];
      stt += t[i] * t[i];
      sta += t[i] * angle.values[i];
      n += 1.0;
    }
    const double orbit = (n * sta - st * sa) / (n * stt - st * st), spin = mean_after(t, table_rate, kSettle);
    // The theory fixes the ratio of the orbit's rate to the table's and that the orbit keeps its radius.
    ErrorTerms terms;
    terms.add("orbit_rate", relative_error(orbit / spin, ratio));
    terms.add("radius", max_abs_after(t, radius_error, kSettle) / kOrbitRadius);
    r.scalars = {{"orbit_ratio", orbit / spin},
                 {"orbit_ratio_theory", ratio},
                 {"orbit_ratio_fixed_table", 2.0 / 7.0},
                 {"table_rate", spin},
                 {"table_rate_theory", kTableRate},
                 {"radius_end", radius.values.back()},
                 {"radius_end_theory", kOrbitRadius},
                 {"ball_spin_z_end", traj.twist(traj.size() - 1, 1).w.z()},
                 {"ball_spin_z_end_theory", kTableRate}};
    terms.finish(r);
    r.series = {std::move(angle), std::move(angle_th), std::move(radius)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
