#include <numbers>

#include <Eigen/Geometry>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kLx = 0.3, kLy = 0.2, kLz = 0.1, kMu = 0.3, kK = 1e6;
constexpr double kSpeed = 1.5, kSpin = 5.0, kDuration = 1.2;
constexpr double kHeading = 25.0 * std::numbers::pi / 180.0;  // Direction of v0 from the x axis.
constexpr double kYaw = 30.0 * std::numbers::pi / 180.0;      // Attitude of the sliding box.
constexpr double kTableGap = 2.0;  // Distance between the centres of the two tables along y [m].

// Coulomb sliding of a box without spin: v = v0 − μ·g·t along its heading until it stops, after
// v0²/(2μg); friction acts below the centre of mass, which moves the centre of pressure ahead
// along the heading, so there is no moment about the vertical and no lateral force.
constexpr double kStopTime = kSpeed / (kMu * kG);
constexpr double kTravel = kSpeed * kSpeed / (2.0 * kMu * kG);

// The same ground cut into `pieces` convex tiles. Two boxes cross it: one slides without spin
// (Coulomb theory), one slides and spins (no closed form). The same two boxes, with the same
// initial conditions, cross the uncut ground next to it; a contact model that does not depend
// on the decomposition gives both pairs the same motion.
Scenario pieces(const ScenarioOptions& o, int pieces) {
  Scenario s;
  s.name = "pieces" + std::to_string(pieces);
  s.description = "Boxes sliding and spinning across a ground cut into " + std::to_string(pieces) +
                  " convex pieces, against the same ground in one piece, mu = 0.3";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  const std::string table = "table_1600x1600x200";
  s.grounds.push_back({.shape = shape_meshes(o.data_dir, table), .planar = true});
  s.grounds.push_back(
      {.shape = shape_meshes(o.data_dir, pieces == 1 ? table : table + "_p" + std::to_string(pieces)),
       .planar = true});
  s.grounds[0].X_WG.p = Vec3(0.0, -kTableGap / 2, 0.0);
  s.grounds[1].X_WG.p = Vec3(0.0, kTableGap / 2, 0.0);

  // Start points on each table: both paths cross the seam x = 0 that every decomposition has,
  // the sliding box at an attitude oblique to it.
  const Vec3 heading(std::cos(kHeading), std::sin(kHeading), 0.0);
  const Vec3 starts[2] = {Vec3(-0.25, -0.6, 0.0), Vec3(-0.25, 0.25, 0.0)};
  for (int g = 0; g < 2; ++g) {
    for (int k = 0; k < 2; ++k) {
      BodySpec& body = s.bodies.emplace_back();
      body.name = std::string(k == 0 ? "slide" : "spin") + (g == 0 ? "_whole" : "_cut");
      body.shape = shape_meshes(o.data_dir, "box_300x200x100");
      body.mass = kMass;
      body.inertia = box_inertia(kMass, kLx, kLy, kLz);
      body.X_W0.p = s.grounds[g].X_WG.p + starts[k] + Vec3(0.0, 0.0, kLz / 2 - kInitialPenetration);
      body.V_W0.v = kSpeed * heading;
      if (k == 0) body.X_W0.R = Eigen::AngleAxisd(kYaw, Vec3::UnitZ()).toRotationMatrix();
      if (k == 1) body.V_W0.w = Vec3(0.0, 0.0, kSpin);
      body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kLx, kLy, kLz), .X_BG = {}};
      s.contacts.push_back(
          standard_contact(BodyRef::body(static_cast<int>(s.bodies.size()) - 1), BodyRef::ground(g), kK, kDamping, kMu));
    }
  }
  s.native = {.stiffness = kK, .damping = kDamping};

  const Vec3 gap = s.grounds[1].X_WG.p - s.grounds[0].X_WG.p;
  s.metrics = [heading, gap](const Trajectory& traj) {
    // Bodies: 0 slide and 1 spin on the whole ground, 2 slide and 3 spin on the cut one.
    MetricReport r;
    Series v{"v", {}}, v_th{"v_theory", {}}, v_whole{"v_whole", {}}, w{"w_spin", {}}, w_whole{"w_spin_whole", {}};
    for (std::size_t i = 0; i < traj.size(); ++i) {
      v.values.push_back(traj.twist(i, 2).v.head<2>().norm());
      v_whole.values.push_back(traj.twist(i, 0).v.head<2>().norm());
      v_th.values.push_back(std::max(0.0, kSpeed - kMu * kG * traj.times()[i]));
      w.values.push_back(traj.twist(i, 3).w.z());
      w_whole.values.push_back(traj.twist(i, 1).w.z());
    }
    const std::size_t last = traj.size() - 1;
    const Vec3 moved = traj.pose(last, 2).p - traj.pose(0, 2).p;
    const double travel = moved.dot(heading), lateral = moved.head<2>().norm() > 0.0 ? (moved - travel * heading).head<2>().norm() : 0.0;
    const double yaw = Eigen::AngleAxisd(traj.pose(0, 2).R.transpose() * traj.pose(last, 2).R).angle();
    const double t_stop = crossing_time(traj.times(), v.values, 0.01 * kSpeed);
    // The cut ground against the whole one: distance between the two final poses.
    const double slide_shift = (traj.pose(last, 2).p - traj.pose(last, 0).p - gap).norm();
    const double slide_turn = Eigen::AngleAxisd(traj.pose(last, 0).R.transpose() * traj.pose(last, 2).R).angle();
    const double spin_shift = (traj.pose(last, 3).p - traj.pose(last, 1).p - gap).norm();
    const double spin_turn = Eigen::AngleAxisd(traj.pose(last, 1).R.transpose() * traj.pose(last, 3).R).angle();
    // Coulomb theory on the cut ground, and the cut ground against the whole one. Lengths over
    // the box's length; a rotation as the displacement of the box's end, over that length.
    ErrorTerms terms;
    terms.add("travel", relative_error(travel, kTravel));
    terms.add("stop_time", relative_error(t_stop, 0.99 * kStopTime));
    terms.add_length("lateral", lateral, kLx);
    terms.add("rotation", yaw / 2);
    terms.add_length("cut_slide", slide_shift, kLx);
    terms.add("cut_slide_rotation", slide_turn / 2);
    terms.add_length("cut_spin", spin_shift, kLx);
    terms.add("cut_spin_rotation", spin_turn / 2);
    r.scalars = {{"travel", travel},
                 {"travel_theory", kTravel},
                 {"stop_time", t_stop},
                 {"stop_time_theory", 0.99 * kStopTime},
                 {"lateral_m", lateral},
                 {"lateral_m_theory", 0.0},
                 {"rotation_rad", yaw},
                 {"rotation_rad_theory", 0.0},
                 {"cut_slide_shift_m", slide_shift},
                 {"cut_slide_turn_rad", slide_turn},
                 {"cut_spin_shift_m", spin_shift},
                 {"cut_spin_turn_rad", spin_turn}};
    terms.finish(r);
    r.series = {std::move(v), std::move(v_th), std::move(v_whole), std::move(w), std::move(w_whole)};
    return r;
  };
  return s;
}

}  // namespace

Scenario pieces1(const ScenarioOptions& o) { return pieces(o, 1); }
Scenario pieces2(const ScenarioOptions& o) { return pieces(o, 2); }
Scenario pieces8(const ScenarioOptions& o) { return pieces(o, 8); }
Scenario pieces64(const ScenarioOptions& o) { return pieces(o, 64); }

}  // namespace volcontact::scenarios
