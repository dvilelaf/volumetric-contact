#include <array>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kRadius = 0.1, kMu = 0.2, kK = 1e7, kDuration = 0.1;
constexpr double kNormalSpeed = 1.0, kGap = 0.005;
// Tangential speeds: with any restitution e in [0, 1] the first impact slides throughout
// (v_t/v_n > 7μ = 1.4) and the second ends sticking (v_t/v_n < 3.5μ = 0.7).
constexpr std::array<double, 2> kTangentialSpeed = {3.0, 0.35};
constexpr std::array<const char*, 2> kNames = {"slide", "stick"};

}  // namespace

// Rigid impact of a sphere without spin on a plane with Coulomb friction: the tangential
// impulse is μ times the normal one, P, unless that would reverse the slip of the contact
// point, which stops after the impulse (2/7)·m·v_t. Returns the loss of tangential velocity
// for the normal impulse per unit mass p = P/m; the spin gained is 5/(2r) times that.
double oblique_velocity_loss(double p, double tangential, double mu) {
  return std::min(mu * p, 2.0 / 7.0 * tangential);
}

// Two spheres thrown at a plane, one at a grazing angle and one steeply.
Scenario oblique(const ScenarioOptions& o) {
  Scenario s;
  s.name = "oblique";
  s.description = "Oblique impact of two spheres on a plane, mu = 0.2: one slides through, one sticks";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  s.grounds.push_back(planar_ground(o.data_dir));
  for (int k = 0; k < 2; ++k) {
    BodySpec& body = s.bodies.emplace_back();
    body.name = kNames[k];
    body.shape = shape_meshes(o.data_dir, curved(o, "sphere_r100"));
    body.mass = kMass;
    body.inertia = sphere_inertia(kMass, kRadius);
    body.X_W0.p = Vec3(0.0, k == 0 ? -0.3 : 0.3, kRadius + kGap);
    body.V_W0.v = Vec3(kTangentialSpeed[k], 0.0, -kNormalSpeed);
    body.primitive = Primitive{.kind = Primitive::Kind::kSphere, .size = Vec3(kRadius, 0.0, 0.0), .X_BG = {}};
    s.contacts.push_back(standard_contact(BodyRef::body(k), BodyRef::ground(0), kK, kImpactDamping, kMu));
  }
  s.native = {.stiffness = kK, .damping = kImpactDamping};

  s.metrics = [](const Trajectory& traj) {
    MetricReport r;
    ErrorTerms terms;
    const std::vector<double>& t = traj.times();
    for (int k = 0; k < 2; ++k) {
      const std::string name = kNames[k];
      Series vx{"v_" + name, {}}, wr{"wr_" + name, {}}, vz{"vz_" + name, {}};
      std::size_t first = traj.size(), after = traj.size();
      for (std::size_t i = 0; i < traj.size(); ++i) {
        vx.values.push_back(traj.twist(i, k).v.x());
        wr.values.push_back(traj.twist(i, k).w.y() * kRadius);
        vz.values.push_back(traj.twist(i, k).v.z());
        if (first == traj.size() && traj.pose(i, k).p.z() < kRadius) first = i;
        // The impact ends at the largest upward speed after the first contact.
        if (first < traj.size() && (after == traj.size() || vz.values[i] > vz.values[after])) after = i;
      }
      double v_after = std::nan(""), wr_after = std::nan(""), v_theory = std::nan(""), wr_theory = std::nan("");
      double impulse = std::nan("");
      if (after < traj.size()) {
        // Momentum along the normal between the start (in the air) and the end of the impact
        // gives the normal impulse, whatever the contact model: P/m = Δv_z + g·Δt.
        impulse = vz.values[after] - vz.values[0] + kG * (t[after] - t[0]);
        const double loss = oblique_velocity_loss(impulse, kTangentialSpeed[k], kMu);
        v_after = vx.values[after];
        wr_after = wr.values[after];
        v_theory = kTangentialSpeed[k] - loss;
        wr_theory = 2.5 * loss;
      }
      terms.add(name + "_velocity", relative_error(v_after, v_theory));
      terms.add(name + "_spin", relative_error(wr_after, wr_theory));
      r.scalars.emplace_back(name + "_velocity", v_after);
      r.scalars.emplace_back(name + "_velocity_theory", v_theory);
      r.scalars.emplace_back(name + "_rolling_speed", wr_after);
      r.scalars.emplace_back(name + "_rolling_speed_theory", wr_theory);
      r.scalars.emplace_back(name + "_normal_impulse", impulse);
      r.scalars.emplace_back(name + "_rebound", after < traj.size() ? vz.values[after] : std::nan(""));
      r.series.push_back(std::move(vx));
      r.series.push_back(std::move(wr));
      r.series.push_back(std::move(vz));
    }
    terms.finish(r);
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
