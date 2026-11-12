#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kM1 = 2.0, kM2 = 1.0;             // Lower and upper block [kg].
constexpr double kL1 = 0.4, kL2 = 0.2, kH = 0.1;   // Sides of the lower and upper block, and their common height [m].
constexpr double kMuGround = 0.2, kMuInterface = 0.6, kK = 1e6, kAngleDeg = 30.0, kDuration = 1.0;

}  // namespace

// A block resting on a larger one on an incline. The pair slides together: the
// friction of the ground gives the common acceleration, and the interface needs
// m2 g mu1 cos a <= mu2 m2 g cos a, which holds since mu1 < mu2: no relative slip.
Scenario twoblocks(const ScenarioOptions& o) {
  Scenario s;
  s.name = "twoblocks";
  s.description = "Block on a block on a 30 deg incline, ground mu = 0.2, interface mu = 0.6";
  s.theory_class = TheoryClass::kA;
  s.gravity = inclined_gravity(kAngleDeg);
  s.duration = kDuration;

  BodySpec& lower = s.bodies.emplace_back();
  lower.name = "lower";
  lower.shape = shape_meshes(o.data_dir, "box_400x400x100");
  lower.mass = kM1;
  lower.inertia = box_inertia(kM1, kL1, kL1, kH);
  lower.X_W0.p = Vec3(0.0, 0.0, kH / 2 - kInitialPenetration);
  lower.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kL1, kL1, kH), .X_BG = {}};

  BodySpec& upper = s.bodies.emplace_back();
  upper.name = "upper";
  upper.shape = shape_meshes(o.data_dir, "box_200x200x100");
  upper.mass = kM2;
  upper.inertia = box_inertia(kM2, kL2, kL2, kH);
  // Centred on the lower block, its bottom 1 µm into the top face of the lower block.
  upper.X_W0.p = Vec3(0.0, 0.0, 1.5 * kH - 2 * kInitialPenetration);
  upper.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kL2, kL2, kH), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMuGround));
  s.contacts.push_back(standard_contact(BodyRef::body(1), BodyRef::body(0), kK, kDamping, kMuInterface));
  s.native = {.stiffness = kK, .damping = kDamping};

  const double a = kAngleDeg * std::numbers::pi / 180.0;
  const double accel = std::max(0.0, kG * (std::sin(a) - kMuGround * std::cos(a)));
  s.metrics = [accel](const Trajectory& traj) {
    MetricReport r;
    Series v{"v_lower", {}}, v_th{"v_lower_theory", {}}, w{"v_upper", {}}, w_th{"v_upper_theory", {}};
    Series slip{"slip", {}};
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const double t = traj.times()[i];
      v.values.push_back(traj.twist(i, 0).v.x());
      v_th.values.push_back(accel * t);
      w.values.push_back(traj.twist(i, 1).v.x());
      w_th.values.push_back(accel * t);
      // Horizontal displacement of the upper block relative to the lower one.
      const Vec3 d = (traj.pose(i, 1).p - traj.pose(i, 0).p) - (traj.pose(0, 1).p - traj.pose(0, 0).p);
      slip.values.push_back(std::hypot(d.x(), d.y()));
    }
    const double v_end = v.values.back(), v_th_end = v_th.values.back();
    // The theory fixes the common velocity and no slip between the blocks.
    ErrorTerms terms;
    terms.add("velocity", relative_error(v_end, v_th_end));
    terms.add_length("slip", max_abs(slip.values), kL2);
    r.scalars = {{"velocity_end", v_end},
                 {"velocity_end_theory", v_th_end},
                 {"velocity_upper_end", w.values.back()},
                 {"velocity_upper_end_theory", w_th.values.back()},
                 {"relative_slip_end_m", slip.values.back()},
                 {"relative_slip_max_m", max_abs(slip.values)},
                 {"relative_slip_max_m_theory", 0.0}};
    terms.finish(r);
    r.series = {std::move(v), std::move(v_th), std::move(w), std::move(w_th), std::move(slip)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
