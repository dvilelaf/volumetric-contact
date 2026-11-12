#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kSide = 0.2, kHeight = 0.1, kK = 1e6;
constexpr double kAngleDeg = 20.0, kRamp = 1.0, kDuration = 3.0;
// tan 20° = 0.364: one block 10 % above the limit, the other 10 % below.
constexpr double kMuHold = 0.40, kMuSlide = 0.33;
constexpr double kAngle = kAngleDeg * std::numbers::pi / 180.0;
constexpr double kWindow = 0.5;  // The residual speed is the mean over the last 0.5 s.

// Tilt θ(t) of the gravity [rad]: a half-cosine ramp from 0 to kAngle over kRamp, then constant.
double tilt(double t) { return t >= kRamp ? kAngle : kAngle * (1.0 - std::cos(std::numbers::pi * t / kRamp)) / 2.0; }

Vec3 gravity_at(double theta) { return {kG * std::sin(theta), 0.0, -kG * std::cos(theta)}; }

// Acceleration of the block that slides, while it moves or once tan θ exceeds μ.
double slide_accel(double t, double v) {
  const double th = tilt(t), a = kG * (std::sin(th) - kMuSlide * std::cos(th));
  return v > 0.0 || a > 0.0 ? a : 0.0;
}

}  // namespace

// Velocity of the block with μ < tan α at the times t: v̇ = g(sin θ(t) − μ cos θ(t)) from the
// time tan θ = μ on (RK4, steps ≤ 0.1 ms).
std::vector<double> wedge_slide_velocity(const std::vector<double>& t) {
  std::vector<double> out;
  double v = 0.0, now = 0.0;
  for (const double target : t) {
    const int n = std::max(1, static_cast<int>(std::ceil((target - now) / 1e-4)));
    const double h = (target - now) / n;
    for (int k = 0; k < n && h > 0.0; ++k) {
      const double tk = now + k * h;
      const double k1 = slide_accel(tk, v), k2 = slide_accel(tk + h / 2, v + h / 2 * k1);
      const double k3 = slide_accel(tk + h / 2, v + h / 2 * k2), k4 = slide_accel(tk + h, v + h * k3);
      v = std::max(0.0, v + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4));
    }
    now = target;
    out.push_back(v);
  }
  return out;
}

// Two blocks on a plane tilted to 20°: Coulomb friction holds the one with μ > tan α and lets
// the one with μ < tan α slide.
Scenario wedge(const ScenarioOptions& o) {
  Scenario s;
  s.name = "wedge";
  s.description = "Two blocks on a plane tilted to 20 deg over 1 s: mu = 0.40 holds, mu = 0.33 slides";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  s.grounds.push_back(planar_ground(o.data_dir));
  const double mus[2] = {kMuHold, kMuSlide};
  for (int k = 0; k < 2; ++k) {
    BodySpec& body = s.bodies.emplace_back();
    body.name = k == 0 ? "hold" : "slide";
    body.shape = shape_meshes(o.data_dir, "box_200x200x100");
    body.mass = kMass;
    body.inertia = box_inertia(kMass, kSide, kSide, kHeight);
    body.X_W0.p = Vec3(0.0, k == 0 ? -0.3 : 0.3, kHeight / 2 - kInitialPenetration);
    // Keeps |g_eff| = g: the total force on the COM is m·g(θ(t)).
    body.applied_force = [](double t) -> Vec3 { return kMass * (gravity_at(tilt(t)) - gravity_at(0.0)); };
    body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kSide, kSide, kHeight), .X_BG = {}};
    s.contacts.push_back(standard_contact(BodyRef::body(k), BodyRef::ground(0), kK, kDamping, mus[k]));
  }
  s.native = {.stiffness = kK, .damping = kDamping};

  s.metrics = [](const Trajectory& traj) {
    MetricReport r;
    Series creep{"creep_hold", {}}, v{"v_slide", {}}, v_th{"v_slide_theory", wedge_slide_velocity(traj.times())};
    std::vector<double> speed;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      creep.values.push_back((traj.pose(i, 0).p - traj.pose(0, 0).p).head<2>().norm());
      speed.push_back(traj.twist(i, 0).v.norm());
      v.values.push_back(traj.twist(i, 1).v.x());
    }
    const double residual = mean_after(traj.times(), speed, kDuration - kWindow);
    // The theory fixes that one block does not move and the velocity of the other.
    ErrorTerms terms;
    terms.add_length("hold_creep", max_abs(creep.values), kSide);
    terms.add_residual_speed(residual, kDuration, kSide);
    terms.add("slide_velocity", relative_error(v.values.back(), v_th.values.back()));
    r.scalars = {{"hold_creep_m", max_abs(creep.values)},
                 {"hold_creep_m_theory", 0.0},
                 {"residual_speed_mps", residual},
                 {"residual_speed_mps_theory", 0.0},
                 {"slide_velocity_end", v.values.back()},
                 {"slide_velocity_end_theory", v_th.values.back()}};
    terms.finish(r);
    r.series = {std::move(creep), std::move(v), std::move(v_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
