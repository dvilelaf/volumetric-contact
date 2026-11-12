#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kRadius = 0.1, kMu = 0.2, kK = 1e8, kSpeed = 2.0;
constexpr double kDuration = 0.6;
// The ball rolls once the slip of its contact point, v − ω·r, falls below this [m/s].
constexpr double kRollingSlip = 1e-3 * kSpeed;

// Sliding loses speed at μg and gains spin at 5μg/(2r), so the slip falls at 7μg/2 and
// vanishes at t* = 2 v0/(7μg) with v = 5 v0/7.
double rolling_speed() { return 5.0 * kSpeed / 7.0; }
double rolling_time() { return 2.0 * kSpeed / (7.0 * kMu * kG); }
// The same event with the measurement's criterion: the slip crosses kRollingSlip.
double onset_time() { return (kSpeed - kRollingSlip) / (3.5 * kMu * kG); }
double onset_velocity() { return kSpeed - kMu * kG * onset_time(); }

// Theory of §4: the velocity of the centre and the rolling speed ω·r, at time t.
double theory_velocity(double t) { return std::max(kSpeed - kMu * kG * t, rolling_speed()); }
double theory_rolling(double t) { return std::min(2.5 * kMu * kG * t, rolling_speed()); }

}  // namespace

// A sphere on a plane, sliding with v0 along x and no spin, then rolling. It starts 1 µm into
// the plane, whatever the contact model: the friction impulse up to time t is μ·∫N dt =
// μ·m·(g·t + v_z(t) − v_z(0)), so the settling on the contact does not change the event.
Scenario billiard(const ScenarioOptions& o) {
  Scenario s;
  s.name = "billiard";
  s.description = "Sphere sliding then rolling on a plane, v0 = 2 m/s, mu = 0.2";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "sphere";
  body.shape = shape_meshes(o.data_dir, curved(o, "sphere_r100"));
  body.mass = kMass;
  body.inertia = sphere_inertia(kMass, kRadius);
  body.X_W0.p = Vec3(0.0, 0.0, kRadius - kInitialPenetration);
  body.V_W0.v = Vec3(kSpeed, 0.0, 0.0);
  body.primitive = Primitive{.kind = Primitive::Kind::kSphere, .size = Vec3(kRadius, 0.0, 0.0), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  s.metrics = [](const Trajectory& traj) {
    MetricReport r;
    Series v{"v", {}}, v_th{"v_theory", {}}, wr{"wr", {}}, wr_th{"wr_theory", {}};
    std::vector<double> slip;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const double t = traj.times()[i];
      const double vx = traj.twist(i, 0).v.x(), rolling = traj.twist(i, 0).w.y() * kRadius;
      v.values.push_back(vx);
      v_th.values.push_back(theory_velocity(t));
      wr.values.push_back(rolling);
      wr_th.values.push_back(theory_rolling(t));
      slip.push_back(vx - rolling);
    }
    // The theory fixes when the rolling starts and the velocity at that time. After it the
    // rigid ball keeps its velocity; what a contact model takes from it then is its rolling
    // resistance, which is not part of this theory (see `sphere`), so it is only reported.
    const double t_roll = crossing_time(traj.times(), slip, kRollingSlip);
    const double v_roll = value_at(traj.times(), v.values, t_roll);
    ErrorTerms terms;
    terms.add("onset_velocity", relative_error(v_roll, onset_velocity()));
    terms.add("onset_time", relative_error(t_roll, onset_time()));
    r.scalars = {{"rolling_start_velocity", v_roll},
                 {"rolling_start_velocity_theory", onset_velocity()},
                 {"rolling_start_time", t_roll},
                 {"rolling_start_time_theory", onset_time()},
                 {"full_rolling_time_theory", rolling_time()},
                 {"velocity_end", v.values.back()},
                 {"velocity_end_theory", v_th.values.back()}};
    terms.finish(r);
    r.series = {std::move(v), std::move(v_th), std::move(wr), std::move(wr_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
