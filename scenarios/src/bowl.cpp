#include "common.hpp"

namespace volcontact::scenarios {

// A sphere released at rest up a spherical bowl: it rolls back and forth without slipping.
// R − r = 200 mm, so that the penetration and the sag of the meshes (tenths of a millimetre)
// are 0.1–0.2 % of the pendulum's length, and the contact travels 240 mm of arc, eight facets
// of the 64-segment bowl, per swing.
Scenario bowl(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.1, R = 0.3, mu = 0.5, K = 1e8, theta0 = 0.4;
  Scenario s;
  s.name = "bowl";
  s.description = "Sphere rolling in a spherical bowl of radius 0.3 m, released 0.4 rad up, mu = 0.5";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 3.5;

  // Centre on the circle of radius R - r about the cavity centre (0, 0, R), pushed
  // 1 µm into the true wall.
  const double d = R - r + kInitialPenetration;
  BodySpec& body = s.bodies.emplace_back();
  body.name = "sphere";
  body.shape = shape_meshes(o.data_dir, curved(o, "sphere_r100"));
  body.mass = m;
  body.inertia = sphere_inertia(m, r);
  body.X_W0.p = Vec3(d * std::sin(theta0), 0.0, R - d * std::cos(theta0));
  body.primitive = Primitive{.kind = Primitive::Kind::kSphere, .size = Vec3(r, 0.0, 0.0), .X_BG = {}};

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, curved(o, "bowl_r300_t20_a60"))});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  // Rolling without slipping: θ̈ = −5g/(7(R − r))·sin θ, a pendulum whose exact period at the
  // amplitude θ0 is 2π√(7(R − r)/(5g))/agm(1, cos(θ0/2)); the amplitude stays θ0.
  const double w2 = 5.0 * kG / (7.0 * (R - r));
  const double period = 2.0 * std::numbers::pi / std::sqrt(w2) / agm(1.0, std::cos(theta0 / 2));
  // Winkler rolling resistance (rolling.hpp) in the conforming contact of relative radius
  // rR/(R − r): M = N·h·ω·a²/6 with a² = 2·r_rel·δ and ω = (R − r)·θ̇/r, which damps the swing at
  // the rate γ = N·h·a²/(6·2·(7/5)·m·r²), taking N = m·g.
  const double r_rel = r * R / (R - r), delta = std::sqrt(m * kG / (K * std::numbers::pi * r_rel));
  const double decay = m * kG * kDamping * 2.0 * r_rel * delta / (6.0 * 2.0 * 1.4 * m * r * r);
  s.metrics = [=](const Trajectory& traj) {
    MetricReport rep;
    Series angle{"angle", {}}, angle_th{"angle_theory", pendulum_angle(traj.times(), theta0, w2)};
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Vec3& p = traj.pose(i, 0).p;
      angle.values.push_back(std::atan2(p.x(), R - p.z()));
    }
    const double measured = mean_period(traj.times(), angle.values);
    const auto [t_peak, amplitude] = last_peak(traj.times(), angle.values, period / 2);
    // The theory fixes the period and that the swing keeps its amplitude (last extreme).
    ErrorTerms terms;
    terms.add("period", relative_error(measured, period));
    terms.add("amplitude", (amplitude - theta0) / theta0);
    rep.scalars = {{"period", measured},
                   {"period_theory", period},
                   {"amplitude_last", amplitude},
                   {"amplitude_last_theory", theta0},
                   {"amplitude_last_winkler", theta0 * std::exp(-decay * t_peak)},
                   {"amplitude_last_time", t_peak}};
    terms.finish(rep);
    rep.series = {std::move(angle), std::move(angle_th)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
