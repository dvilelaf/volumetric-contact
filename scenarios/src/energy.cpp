#include "common.hpp"

namespace volcontact::scenarios {

// The sphere of `bowl` without friction and without damping: it slides instead of rolling, so the
// pendulum is the point mass at its centre, and nothing may add or remove energy.
Scenario energy(const ScenarioOptions& o) {
  constexpr double m = 1.0, r = 0.1, R = 0.3, K = 1e8, theta0 = 0.4;
  Scenario s;
  s.name = "energy";
  s.description = "Sphere sliding in a spherical bowl of radius 0.3 m, frictionless and undamped";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = 3.5;

  const double d = R - r + kInitialPenetration;
  BodySpec& body = s.bodies.emplace_back();
  body.name = "sphere";
  body.shape = shape_meshes(o.data_dir, curved(o, "sphere_r100"));
  body.mass = m;
  body.inertia = sphere_inertia(m, r);
  body.X_W0.p = Vec3(d * std::sin(theta0), 0.0, R - d * std::cos(theta0));
  body.primitive = Primitive{.kind = Primitive::Kind::kSphere, .size = Vec3(r, 0.0, 0.0), .X_BG = {}};

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, curved(o, "bowl_r300_t20_a60"))});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, 0.0, 0.0));
  s.native = {.stiffness = K, .damping = 0.0};

  // θ̈ = −g/(R − r)·sin θ: exact period at the amplitude θ0 through the arithmetic–geometric mean.
  const double w2 = kG / (R - r);
  const double period = 2.0 * std::numbers::pi / std::sqrt(w2) / agm(1.0, std::cos(theta0 / 2));
  const double swing = m * kG * (R - r) * (1.0 - std::cos(theta0));  // Energy of the swing.
  s.metrics = [=](const Trajectory& traj) {
    MetricReport rep;
    const std::vector<double>& t = traj.times();
    Series angle{"angle", {}}, angle_th{"angle_theory", pendulum_angle(t, theta0, w2)}, energy{"energy", {}};
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Vec3& p = traj.pose(i, 0).p;
      const Twist& V = traj.twist(i, 0);
      angle.values.push_back(std::atan2(p.x(), R - p.z()));
      // Kinetic plus gravitational, from the lowest position of the centre, over the swing's.
      energy.values.push_back(
          (0.5 * m * V.v.squaredNorm() + 0.2 * m * r * r * V.w.squaredNorm() + m * kG * (p.z() - r)) / swing);
    }
    const double measured = mean_period(t, angle.values);
    // Mean over the first and over the last period: the same phases of the swing. The energy the
    // contact stores is not in the sum above, and it does not cancel in the difference: with h = 0
    // the bounce on the contact spring is never damped, and sinking to the static penetration
    // alone moves about 3 % of the swing's energy, so this term varies irregularly with the step
    // and the start for every law (docs/research/anomalies.md §2). It bounds the drift; it does
    // not measure a steady gain or loss.
    double first = 0.0, last = 0.0;
    int n_first = 0, n_last = 0;
    for (std::size_t i = 0; i < t.size(); ++i) {
      if (t[i] <= period) {
        first += energy.values[i];
        ++n_first;
      }
      if (t[i] >= t.back() - period) {
        last += energy.values[i];
        ++n_last;
      }
    }
    const double drift = last / n_last - first / n_first;
    // The theory fixes the period and that the energy stays.
    ErrorTerms terms;
    terms.add("period", relative_error(measured, period));
    terms.add("energy_drift", drift);
    rep.scalars = {{"period", measured},
                   {"period_theory", period},
                   {"energy_drift", drift},
                   {"energy_drift_theory", 0.0},
                   {"spin_max_radps", [&] {
                      double w = 0.0;
                      for (std::size_t i = 0; i < traj.size(); ++i) w = std::max(w, traj.twist(i, 0).w.norm());
                      return w;
                    }()}};
    terms.finish(rep);
    rep.series = {std::move(angle), std::move(angle_th), std::move(energy)};
    return rep;
  };
  return s;
}

}  // namespace volcontact::scenarios
