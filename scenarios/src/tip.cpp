#include <cmath>
#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kSide = 0.2, kHeight = 0.6, kMu = 0.8, kK = 1e7;
constexpr double kHc = kHeight / 2;    // Height of the COM above the base [m].
constexpr double kDuration = 15.0;
constexpr double kRampAngle = 12.0;    // Tilt reached at the end of the ramp [deg].
constexpr double kRampTime = 3.0;      // Duration of the half-cosine ramp [s].
constexpr double kTiltRate = 0.5;      // Tilt rate after the ramp [deg/s].
constexpr double kTipRotation = 0.05;  // Rotation from the initial attitude that marks the tip [rad].

constexpr double deg = std::numbers::pi / 180.0;

// Gravity tilt θ(t) [rad]: a half-cosine ramp to kRampAngle at kRampTime, then kTiltRate.
// The ramp r·t + c·(1 − cos(πt/T)), with c = (A − r·T)/2, has rate r at both ends, so θ and
// θ̇ are continuous at T.
double tilt(double t) {
  if (t > kRampTime) return (kRampAngle + kTiltRate * (t - kRampTime)) * deg;
  const double c = (kRampAngle - kTiltRate * kRampTime) / 2;
  return (kTiltRate * t + c * (1 - std::cos(std::numbers::pi * t / kRampTime))) * deg;
}

// Gravity of an incline θ, as inclined_gravity with the angle in radians.
Vec3 gravity_at(double theta) { return {kG * std::sin(theta), 0.0, -kG * std::cos(theta)}; }

// Time at which the tilt reaches theta [rad]; the tilt only grows, so bisection.
double tilt_time(double theta) {
  double lo = 0.0, hi = kDuration;
  for (int i = 0; i < 100; ++i) {
    const double mid = (lo + hi) / 2;
    if (tilt(mid) < theta) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return (lo + hi) / 2;
}

// Tipping about the downhill edge when the gravity passes the edge: tan θ = w/h, with w the
// base side and h the height (the COM sits at the middle of the height).
const double kTipRigid = std::atan(kSide / kHeight);

// First sample whose rotation reaches kTipRotation; NaN if none does.
std::size_t tip_index(const std::vector<double>& rotation) {
  for (std::size_t i = 0; i < rotation.size(); ++i) {
    if (rotation[i] >= kTipRotation) return i;
  }
  return rotation.size();
}

}  // namespace

// Largest tilt θ [rad] at which the Winkler base still holds the block (§4, row 5): the largest θ
// with max over lean φ of (a/2 − h_c φ − c/3) ≥ h_c tan θ, where c = √(2N/(K a φ)), N = m g cos θ.
// The maximum over φ is closed form: the derivative vanishes at φ = (C/(2 h_c))^{2/3}, with
// C = √(2N/(K a))/3 so that c/3 = C/√φ. Bisection between 0 and the rigid limit.
double tip_equilibrium_tilt(double K) {
  const auto margin = [K](double theta) {
    const double N = kMass * kG * std::cos(theta);
    const double C = std::sqrt(2 * N / (K * kSide)) / 3;
    const double phi = std::pow(C / (2 * kHc), 2.0 / 3.0);
    return kSide / 2 - kHc * phi - C / std::sqrt(phi) - kHc * std::tan(theta);
  };
  double lo = 0.0, hi = kTipRigid;
  for (int i = 0; i < 100; ++i) {
    const double mid = (lo + hi) / 2;
    if (margin(mid) >= 0) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

// A 1 kg tall box standing on its 0.2 × 0.2 m face, tilted by gravity until it tips.
Scenario tip(const ScenarioOptions& o) {
  Scenario s;
  s.name = "tip";
  s.description = "Tall box on its face, gravity tilted at constant |g| to 12 deg over 3 s, then 0.5 deg/s, mu = 0.8";
  s.theory_class = TheoryClass::kC;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "box";
  body.shape = shape_meshes(o.data_dir, "box_200x200x600");
  body.mass = kMass;
  body.inertia = box_inertia(kMass, kSide, kSide, kHeight);
  body.X_W0.p = Vec3(0.0, 0.0, kHeight / 2 - kInitialPenetration);
  // Keeps |g_eff| = g: the total force on the COM is m·g(θ(t)).
  body.applied_force = [](double t) -> Vec3 { return kMass * (gravity_at(tilt(t)) - gravity_at(tilt(0.0))); };
  body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kSide, kSide, kHeight), .X_BG = {}};

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  s.native = {.stiffness = kK, .damping = kDamping};

  const double theta_star = tip_equilibrium_tilt(kK);
  s.metrics = [theta_star](const Trajectory& traj) {
    MetricReport r;
    Series rot{"rotation", {}}, theta{"theta", {}};
    const Mat3 R0 = traj.pose(0, 0).R;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      rot.values.push_back(Eigen::AngleAxisd(R0.transpose() * traj.pose(i, 0).R).angle());
      theta.values.push_back(tilt(traj.times()[i]));
    }
    const std::size_t tip = tip_index(rot.values);
    const double tip_angle = tip < traj.size() ? theta.values[tip] : std::nan("");

    // Sliding before the tip: the bottom-face centre moves only (a/2)(1 − cos φ) when the
    // block tips about its edge, so any larger horizontal displacement is sliding.
    const Vec3 bottom0 = traj.pose(0, 0).apply(Vec3(0.0, 0.0, -kHeight / 2));
    double slide = 0.0;
    for (std::size_t i = 0; i < std::min(tip, traj.size()); ++i) {
      const Vec3 bottom = traj.pose(i, 0).apply(Vec3(0.0, 0.0, -kHeight / 2));
      slide = std::max(slide, (bottom - bottom0).head<2>().norm());
    }

    // The theory fixes the tilt at which the block tips and no sliding before it.
    ErrorTerms terms;
    terms.add("tip_angle", relative_error(tip_angle, theta_star));
    terms.add_length("slide", slide, kSide);
    r.scalars = {{"tip_angle", tip_angle},
                 {"tip_angle_theory", theta_star},
                 {"tip_angle_rigid", kTipRigid},
                 {"tip_time", tip < traj.size() ? traj.times()[tip] : std::nan("")},
                 {"tip_time_theory", tilt_time(theta_star)},
                 {"slide", slide},
                 {"slide_theory", 0.0}};
    terms.finish(r);
    r.series = {std::move(rot), std::move(theta)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
