#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 10.0, kSide = 0.4, kThickness = 0.05, kOffset = 0.05, kMu = 0.5, kK = 1e6;
constexpr double kDuration = 1.0, kWindow = 0.2;  // The result is the mean over the last 0.2 s.
constexpr double kArea = kSide * kSide;
// The base centre, in the body frame: the origin is the COM, kOffset from the slab's centroid.
const Vec3 kBaseCentre(-kOffset, 0.0, -kThickness / 2);

// Winkler tilt of §4: K·I_A·θ = m·g·(e + z_c·θ), the COM arm growing as the slab tilts.
// The base is wider than the COM's arm, so the uphill edge must keep its penetration.
double theory_tilt() {
  const double I_A = kSide * kSide * kSide * kSide / 12;  // ∫x² dA over the base.
  const double z_c = kThickness / 2;
  return kMass * kG * kOffset / (kK * I_A - kMass * kG * z_c);
}

}  // namespace

// A slab whose COM lies 50 mm from its centroid, resting flat on the plane: it tilts
// towards the COM on the Winkler foundation. Its tilt is the main result.
Scenario offset(const ScenarioOptions& o) {
  Scenario s;
  s.name = "offset";
  s.description = "Slab 400x400x50 mm, COM 50 mm off its centre, resting flat on a plane, 10 kg";
  s.theory_class = TheoryClass::kC;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "slab";
  body.shape = shape_meshes(o.data_dir, "box_400x400x50_o50x0x0");
  body.mass = kMass;
  // Centroid inertia of the slab, moved to the COM by the parallel-axis theorem.
  const Vec3 d(-kOffset, 0.0, 0.0);
  body.inertia = box_inertia(kMass, kSide, kSide, kThickness) +
                 kMass * (d.squaredNorm() * Mat3::Identity() - d * d.transpose());
  body.X_W0.p = Vec3(0.0, 0.0, kThickness / 2 - kInitialPenetration);
  // The box is centred on the slab's centroid, which is at d from the COM (the body origin).
  body.primitive = Primitive{.kind = Primitive::Kind::kBox,
                             .size = Vec3(kSide, kSide, kThickness),
                             .X_BG = {Mat3::Identity(), d}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  const double theta = theory_tilt();
  const double sinking = kMass * kG / (kK * kArea);
  s.metrics = [theta, sinking](const Trajectory& traj) {
    MetricReport r;
    Series tilt{"tilt", {}}, tilt_th{"tilt_theory", {}};
    std::vector<double> depth, speed;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      // Rotation about y with the +x side going down: R(2,0) = −sin θ.
      tilt.values.push_back(std::atan2(-X.R(2, 0), X.R(0, 0)));
      tilt_th.values.push_back(theta);
      depth.push_back(-(X.p + X.R * kBaseCentre).z());
      speed.push_back(traj.twist(i, 0).v.norm());
    }
    const double tilt_end = mean_after(traj.times(), tilt.values, kDuration - kWindow);
    const double sink_end = mean_after(traj.times(), depth, kDuration - kWindow);
    const double speed_end = mean_after(traj.times(), speed, kDuration - kWindow);
    // The theory fixes the tilt and that the slab rests.
    ErrorTerms terms;
    terms.add("tilt", relative_error(tilt_end, theta));
    terms.add_residual_speed(speed_end, kDuration, kSide);
    r.scalars = {{"tilt", tilt_end},
                 {"tilt_theory", theta},
                 {"sinking_m", sink_end},
                 {"sinking_m_theory", sinking},
                 {"residual_speed_mps", speed_end},
                 {"residual_speed_mps_theory", 0.0}};
    terms.finish(r);
    r.series = {std::move(tilt), std::move(tilt_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
