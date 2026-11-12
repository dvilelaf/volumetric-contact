#include <array>
#include <numbers>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr int kCubes = 3;
constexpr double kMass = 1.0, kSide = 0.2, kMu = 0.5, kK = 1e6, kAngleDeg = 5.0, kRamp = 1.0, kDuration = 3.0;
constexpr double kAverage = 0.5;  // The compressions are averaged over the last 0.5 s.
constexpr double kAngle = kAngleDeg * std::numbers::pi / 180.0;

// Tilt θ(t) of the gravity [rad]: a half-cosine ramp from 0 to kAngle over kRamp, then constant.
double tilt(double t) { return t >= kRamp ? kAngle : kAngle * (1.0 - std::cos(std::numbers::pi * t / kRamp)) / 2.0; }

// Gravity of an incline θ [rad], downhill is +x.
Vec3 gravity_at(double theta) { return {kG * std::sin(theta), 0.0, -kG * std::cos(theta)}; }

// Compression of the interfaces ground–c1, c1–c2 and c2–c3, from the heights z of the centres.
std::array<double, kCubes> compressions(const std::array<double, kCubes>& z) {
  return {kSide / 2 - z[0], z[0] - z[1] + kSide, z[1] - z[2] + kSide};
}

}  // namespace

// Three cubes stacked on a plane tilted to 5°. Interface k carries the n = 3, 2, 1 cubes above it,
// so its compression is n m g cos θ / (K a²), and the stack holds still (tan θ < μ, tan θ·h_c < a/6).
Scenario stack(const ScenarioOptions& o) {
  Scenario s;
  s.name = "stack";
  s.description = "Three 0.2 m cubes stacked, gravity tilted to 5 deg over 1 s, mu = 0.5";
  s.theory_class = TheoryClass::kC;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  for (int k = 0; k < kCubes; ++k) {
    BodySpec& body = s.bodies.emplace_back();
    body.name = "cube" + std::to_string(k + 1);
    body.shape = shape_meshes(o.data_dir, "box_200x200x200");
    body.mass = kMass;
    body.inertia = box_inertia(kMass, kSide, kSide, kSide);
    // Each cube sinks 1 µm more than the one below, the lowest one into the ground.
    body.X_W0.p = Vec3(0.0, 0.0, (k + 0.5) * kSide - (k + 1) * kInitialPenetration);
    // Keeps |g_eff| = g: the total force on the COM is m·g(θ(t)).
    body.applied_force = [](double t) -> Vec3 { return kMass * (gravity_at(tilt(t)) - gravity_at(0.0)); };
    body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3::Constant(kSide), .X_BG = {}};
  }

  s.grounds.push_back(planar_ground(o.data_dir));
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, kMu));
  for (int k = 1; k < kCubes; ++k) {
    s.contacts.push_back(standard_contact(BodyRef::body(k), BodyRef::body(k - 1), kK, kDamping, kMu));
  }
  s.native = {.stiffness = kK, .damping = kDamping};

  std::array<double, kCubes> theory{};
  for (int i = 0; i < kCubes; ++i) theory[i] = (kCubes - i) * kMass * kG * std::cos(kAngle) / (kK * kSide * kSide);

  s.metrics = [theory](const Trajectory& traj) {
    MetricReport r;
    std::vector<Series> comp, comp_th, creep;
    std::vector<double> speed;  // The fastest cube at each sample.
    for (int i = 1; i <= kCubes; ++i) {
      comp.push_back({"compression_" + std::to_string(i), {}});
      comp_th.push_back({"compression_" + std::to_string(i) + "_theory", {}});
      creep.push_back({"creep_c" + std::to_string(i), {}});
    }
    for (std::size_t n = 0; n < traj.size(); ++n) {
      std::array<double, kCubes> z{};
      speed.push_back(0.0);
      for (int k = 0; k < kCubes; ++k) {
        speed.back() = worse(speed.back(), traj.twist(n, k).v.norm());
        z[k] = traj.pose(n, k).p.z();
        creep[k].values.push_back(traj.pose(n, k).p.x() - traj.pose(0, k).p.x());
      }
      const auto d = compressions(z);
      for (int i = 0; i < kCubes; ++i) {
        comp[i].values.push_back(d[i]);
        comp_th[i].values.push_back(theory[i]);
      }
    }

    double worst = 0.0, creep_max = 0.0;
    for (int i = 0; i < kCubes; ++i) {
      const std::string idx = std::to_string(i + 1);
      const double measured = mean_after(traj.times(), comp[i].values, kDuration - kAverage);
      worst = worse(worst, relative_error(measured, theory[i]));
      creep_max = worse(creep_max, max_abs(creep[i].values));
      r.scalars.emplace_back("compression_" + idx + "_m", measured);
      r.scalars.emplace_back("compression_" + idx + "_m_theory", theory[i]);
      r.scalars.emplace_back("creep_c" + idx + "_m", creep[i].values.back());
      r.scalars.emplace_back("creep_c" + idx + "_m_theory", 0.0);
    }
    const double residual = mean_after(traj.times(), speed, kDuration - kAverage);
    r.scalars.emplace_back("residual_speed_mps", residual);
    r.scalars.emplace_back("residual_speed_mps_theory", 0.0);
    // The theory fixes the three compressions (the worst one counts) and that no cube moves:
    // the largest displacement along the slope of any cube, and the residual speed.
    ErrorTerms terms;
    terms.add("compression", worst);
    terms.add_length("creep", creep_max, kSide);
    terms.add_residual_speed(residual, kDuration, kSide);
    terms.finish(r);
    for (auto* group : {&comp, &comp_th, &creep}) {
      for (auto& series : *group) r.series.push_back(std::move(series));
    }
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
