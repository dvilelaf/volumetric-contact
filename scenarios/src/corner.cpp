#include <array>

#include "common.hpp"

namespace volcontact::scenarios {

// A 1 kg cube in the concave corner of corner_s300 under gravity g(−1, −1, −2)/√6, without
// friction. Each face carries the gravity component along its normal, so the cube sinks into the
// floor and the two walls by m g |dᵢ|/(K a²) and stays there. With friction the statics are
// indeterminate (any wall force in a wide interval is an equilibrium), so μ = 0.
Scenario corner(const ScenarioOptions& o) {
  constexpr double m = 1.0, a = 0.1, mu = 0.0, K = 1e7, duration = 1.0;
  constexpr double kTail = 0.2;  // The result is the mean over the last 0.2 s.
  const Vec3 d = Vec3(-1.0, -1.0, -2.0).normalized();  // Direction of gravity.
  Scenario s;
  s.name = "corner";
  s.description = "Cube resting in a 300 mm corner under g(-1,-1,-2)/sqrt(6), frictionless";
  s.theory_class = TheoryClass::kC;
  s.gravity = kG * d;
  s.duration = duration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "cube";
  body.shape = shape_meshes(o.data_dir, "box_100x100x100");
  body.mass = m;
  body.inertia = box_inertia(m, a, a, a);
  body.X_W0.p = Vec3::Constant(a / 2 - kInitialPenetration);  // 1 µm into each face.

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, "corner_s300")});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), K, kDamping, mu));
  s.native = {.stiffness = K, .damping = kDamping};

  s.metrics = [=](const Trajectory& traj) {
    // Depth of the face whose normal is world axis k: the cube's extent along that axis
    // (half side times the sum of |R| along the row) minus its centre, below the plane.
    const auto penetration = [&](const Pose& X, int k) { return a / 2 * X.R.row(k).cwiseAbs().sum() - X.p(k); };
    const std::vector<double>& t = traj.times();
    std::array<std::vector<double>, 3> pen;
    std::vector<double> speed;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      for (int k = 0; k < 3; ++k) pen[k].push_back(penetration(traj.pose(i, 0), k));
      speed.push_back(traj.twist(i, 0).v.norm());
    }

    MetricReport r;
    constexpr std::array<char, 3> axis = {'x', 'y', 'z'};
    double worst = 0.0;
    for (int k = 0; k < 3; ++k) {
      const double theory = m * kG * std::abs(d(k)) / (K * a * a);
      const double measured = mean_after(t, pen[k], duration - kTail);
      const std::string name = std::string("penetration_") + axis[k];
      worst = worse(worst, relative_error(measured, theory));
      r.scalars.push_back({name + "_m", measured});
      r.scalars.push_back({name + "_m_theory", theory});
      r.series.push_back({name, pen[k]});
      r.series.push_back({name + "_theory", std::vector<double>(t.size(), theory)});
    }
    const double residual = mean_after(t, speed, duration - kTail);
    r.scalars.push_back({"residual_speed_mps", residual});
    r.scalars.push_back({"residual_speed_mps_theory", 0.0});
    // The theory fixes the three penetrations (the worst one counts) and that the cube rests.
    ErrorTerms terms;
    terms.add("penetration", worst);
    terms.add_residual_speed(residual, duration, a);
    terms.finish(r);
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
