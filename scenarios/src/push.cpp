#include <cmath>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kSide = 0.4, kThickness = 0.2, kMu = 0.5, kK = 1e6;
constexpr double kRamp = 2.0;  // c [N/s]: F(t) = (c·t, 0, 0) at the centre of mass.
constexpr double kDuration = 4.0;
constexpr double kOnsetSpeed = 1e-3;  // Speed that marks the onset of sliding [m/s].

// Sliding starts when the ramp reaches the static friction μmg; then m·v̇ = c·t − μmg.
constexpr double kSlideStart = kMu * kMass * kG / kRamp;
// The threshold crossing of the theory, with the same criterion as the measured onset.
constexpr double kOnsetTheory = kSlideStart + std::sqrt(2 * kMass * kOnsetSpeed / kRamp);

// Sliding speed of §4; the box stays at rest before the onset.
double sliding_speed(double t) { return t > kSlideStart ? kRamp * (t - kSlideStart) * (t - kSlideStart) / (2 * kMass) : 0.0; }

// Time from which the speed stays above kOnsetSpeed until the end of the run (a passing
// transient above it is not the onset); NaN if the run does not end above it.
double onset_time(const std::vector<double>& t, const std::vector<double>& v) {
  std::size_t i = v.size();
  while (i > 0 && v[i - 1] > kOnsetSpeed) --i;
  return i < v.size() ? t[i] : std::nan("");
}

}  // namespace

// A 1 kg box pushed by a force ramp at its centre of mass, from rest on the plane.
Scenario push(const ScenarioOptions& o) {
  Scenario s;
  s.name = "push";
  s.description = "Box pushed by a force ramp of 2 N/s at its centre of mass, 1 kg, mu = 0.5";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "box";
  body.shape = shape_meshes(o.data_dir, "box_400x400x200");
  body.mass = kMass;
  body.inertia = box_inertia(kMass, kSide, kSide, kThickness);
  body.X_W0.p = Vec3(0.0, 0.0, kThickness / 2 - kInitialPenetration);
  body.applied_force = [](double t) { return Vec3(kRamp * t, 0.0, 0.0); };
  body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kSide, kSide, kThickness), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  s.metrics = [](const Trajectory& traj) {
    MetricReport r;
    Series v{"v", {}}, v_th{"v_theory", {}};
    std::vector<double> x;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      v.values.push_back(traj.twist(i, 0).v.x());
      v_th.values.push_back(sliding_speed(traj.times()[i]));
      x.push_back(traj.pose(i, 0).p.x());
    }
    const double v_end = v.values.back(), v_th_end = v_th.values.back();
    // Creep: the displacement before the sliding starts, which the theory keeps at zero.
    const double creep = value_at(traj.times(), x, kSlideStart) - x.front();
    const double onset = onset_time(traj.times(), v.values);
    // The theory fixes the final velocity, the time of the onset and no motion before it.
    ErrorTerms terms;
    terms.add("velocity", relative_error(v_end, v_th_end));
    terms.add("onset_time", relative_error(onset, kOnsetTheory));
    terms.add_length("creep", creep, kSide);
    r.scalars = {{"velocity_end", v_end},
                 {"velocity_end_theory", v_th_end},
                 {"onset_time", onset},
                 {"onset_time_theory", kOnsetTheory},
                 {"creep", creep},
                 {"creep_theory", 0.0}};
    terms.finish(r);
    r.series = {std::move(v), std::move(v_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
