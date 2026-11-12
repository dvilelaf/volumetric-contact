#include <numbers>

#include <Eigen/Geometry>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kRadius = 0.1, kLength = 0.4, kCavity = 0.3, kTheta0 = 0.4;
constexpr double kMu = 0.5, kK = 1e7, kDuration = 3.5;

}  // namespace

// A solid cylinder released at rest 0.4 rad up a cylindrical trough of radius 0.3 m: it rolls
// back and forth. R − r = 200 mm and 240 mm of arc per swing, as in `bowl`.
Scenario trough(const ScenarioOptions& o) {
  const double pi = std::numbers::pi;
  Scenario s;
  s.name = "trough";
  s.description = "Cylinder rolling in a cylindrical trough of radius 0.3 m, released 0.4 rad from the bottom";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  // Axis of the cylinder on the circle R − r about the cavity axis (y, z = R), 1 µm into the wall.
  const double d = kCavity - kRadius + kInitialPenetration;
  BodySpec& body = s.bodies.emplace_back();
  body.name = "cylinder";
  body.shape = shape_meshes(o.data_dir, curved(o, "cylinder_r100_h400"));
  body.mass = kMass;
  body.inertia = cylinder_inertia(kMass, kRadius, kLength);
  // Body axis z lies along the world y axis.
  body.X_W0.R = Eigen::AngleAxisd(-pi / 2, Vec3::UnitX()).toRotationMatrix();
  body.X_W0.p = Vec3(d * std::sin(kTheta0), 0.0, kCavity - d * std::cos(kTheta0));
  body.primitive = Primitive{.kind = Primitive::Kind::kCylinder, .size = Vec3(kRadius, kLength, 0.0), .X_BG = {}};

  GroundSpec& ground = s.grounds.emplace_back();
  ground.shape = shape_meshes(o.data_dir, curved(o, "trough_r300_t20_a60_l500"));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  // Rolling without slipping: θ̈ = −(2g/(3(R − r)))·sin θ; exact period at the amplitude θ0
  // through the arithmetic–geometric mean; the amplitude stays θ0.
  const double w2 = 2.0 * kG / (3.0 * (kCavity - kRadius));
  const double period = 2.0 * pi / std::sqrt(w2) / agm(1.0, std::cos(kTheta0 / 2));
  // Winkler rolling resistance (rolling.hpp) with the relative radius rR/(R − r):
  // M = N·h·ω·a²/5, a² = 2·r_rel·δ, damping the swing at γ = N·h·a²/(5·2·(3/2)·m·r²), N = m·g.
  const double r_rel = kRadius * kCavity / (kCavity - kRadius);
  const double delta = cylinder_static_penetration(kMass * kG, kK, r_rel, kLength);
  const double decay = kMass * kG * kDamping * 2.0 * r_rel * delta / (5.0 * 2.0 * 1.5 * kMass * kRadius * kRadius);
  s.metrics = [w2, period, decay](const Trajectory& traj) {
    MetricReport rep;
    Series angle{"angle", {}}, angle_th{"angle_theory", pendulum_angle(traj.times(), kTheta0, w2)};
    double axis = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      angle.values.push_back(std::atan2(X.p.x(), kCavity - X.p.z()));
      axis = std::max(axis, std::acos(std::clamp(X.R(1, 2), -1.0, 1.0)));  // Body z against world y.
    }
    const double measured = mean_period(traj.times(), angle.values);
    const auto [t_peak, amplitude] = last_peak(traj.times(), angle.values, period / 2);
    // The theory fixes the period and that the swing keeps its amplitude (last extreme).
    ErrorTerms terms;
    terms.add("period", relative_error(measured, period));
    terms.add("amplitude", (amplitude - kTheta0) / kTheta0);
    rep.scalars = {{"period", measured},
                   {"period_theory", period},
                   {"amplitude_last", amplitude},
                   {"amplitude_last_theory", kTheta0},
                   {"amplitude_last_winkler", kTheta0 * std::exp(-decay * t_peak)},
                   {"amplitude_last_time", t_peak},
                   {"axis_deviation_max_rad", axis}};
    terms.finish(rep);
    rep.series = {std::move(angle), std::move(angle_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
