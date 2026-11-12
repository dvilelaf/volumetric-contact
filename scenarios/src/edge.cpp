#include <array>

#include "common.hpp"

namespace volcontact::scenarios {
namespace {

constexpr double kMass = 1.0, kLength = 0.3, kWidth = 0.2, kHeight = 0.1, kK = 1e7;
constexpr double kSpeed = 1.0, kStart = 0.3, kDuration = 0.6;  // The COM starts kStart before the edge.
constexpr double kTableHalf = 0.5;                             // The edge is at x = +0.5 m.
constexpr std::array<double, 2> kPitch = {0.1, 0.25};           // Pitch angles whose times are compared [rad].
constexpr double kOnset = kStart / kSpeed;                     // The COM is over the edge.
constexpr double kHc = kHeight / 2;
constexpr double kInertia = kMass * (kLength * kLength + kHeight * kHeight) / 12;

// First time x rises above `level`, interpolated linearly; NaN if never.
double rising_time(const std::vector<double>& t, const std::vector<double>& x, double level) {
  for (std::size_t i = 1; i < x.size(); ++i) {
    if (x[i] >= level && x[i - 1] < level) return t[i - 1] + (level - x[i - 1]) / (x[i] - x[i - 1]) * (t[i] - t[i - 1]);
  }
  return std::nan("");
}

}  // namespace

// Pitch φ(t) of a rigid box that slides without friction off the edge of a table. It stays flat
// until its centre of mass is over the edge; then its bottom face rests on the edge only. With
// s the position of the edge along the bottom face, measured from under the centre of mass
// (s < 0 behind it), and h_c the height of the centre of mass above that face, Lagrange's
// equations for (s, φ) are
//   s̈ = h_c·φ̈ + s·φ̇² − g·sin φ,   (m·s² + I)·φ̈ = m·s·(h_c·φ̇² − 2·ṡ·φ̇ − g·cos φ),
// until the rear end reaches the edge (s = −L/2); then the box flies with φ̇ constant.
std::vector<double> edge_pitch(const std::vector<double>& t) {
  std::vector<double> out;
  using State = std::array<double, 4>;  // s, ṡ, φ, φ̇
  const auto rate = [](const State& y) -> State {
    if (y[0] > 0.0) return {y[1], 0.0, 0.0, 0.0};          // Flat on the table.
    if (y[0] < -kLength / 2) return {y[1], 0.0, y[3], 0.0};  // Past the edge.
    const double phi_dd = kMass * y[0] * (kHc * y[3] * y[3] - 2.0 * y[1] * y[3] - kG * std::cos(y[2])) /
                          (kMass * y[0] * y[0] + kInertia);
    return {y[1], kHc * phi_dd + y[0] * y[3] * y[3] - kG * std::sin(y[2]), y[3], phi_dd};
  };
  State y = {kStart, -kSpeed, 0.0, 0.0};
  double now = 0.0;
  for (const double target : t) {
    const int n = std::max(1, static_cast<int>(std::ceil((target - now) / 1e-5)));
    const double h = (target - now) / n;
    for (int k = 0; k < n && h > 0.0; ++k) {
      const auto step = [&](const State& d, double f) {
        State z;
        for (int i = 0; i < 4; ++i) z[i] = y[i] + f * d[i];
        return z;
      };
      const State k1 = rate(y), k2 = rate(step(k1, h / 2)), k3 = rate(step(k2, h / 2)), k4 = rate(step(k3, h));
      for (int i = 0; i < 4; ++i) y[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    now = target;
    out.push_back(y[2]);
  }
  return out;
}

// The same box on a Winkler foundation of stiffness K with the damping h, in the plane of the
// motion: the bottom face is cut into 300 strips, each pressed by K·depth·(1 − h·v_n) while it is
// over the table, the force normal to the table. Returns the times at which the pitch reaches
// kPitch (RK4 at 20 µs). A soft table lets the box lean before its centre of mass reaches the
// edge, so it pitches earlier than the rigid one; the times tend to the rigid ones as K grows
// (−19 % at 1e7, −9 % at 1e8, −5 % at 1e9 for the first).
std::array<double, 2> edge_winkler_pitch_times(double K, double h) {
  constexpr int kStrips = 300;
  using State = std::array<double, 6>;  // x (from the edge), z, φ and their rates.
  const auto rate = [&](const State& y) -> State {
    const double c = std::cos(y[2]), sn = std::sin(y[2]);
    double force = 0.0, moment = 0.0;
    for (int i = 0; i < kStrips; ++i) {
      const double xi = ((i + 0.5) / kStrips - 0.5) * kLength;
      const double px = y[0] + c * xi - sn * kHc, pz = y[1] - sn * xi - c * kHc;
      if (px > 0.0 || pz >= 0.0) continue;
      const double vz = y[4] - y[5] * (c * xi - sn * kHc);
      const double f = K * kWidth * (-pz) * std::max(0.0, 1.0 + h * vz) * kLength / kStrips;
      force += f;
      moment += f * (px - y[0]);
    }
    return {y[3], y[4], y[5], 0.0, force / kMass - kG, -moment / kInertia};
  };
  State y = {-kStart, kHc - kMass * kG / (K * kWidth * kLength), 0.0, kSpeed, 0.0, 0.0};
  std::array<double, 2> times = {std::nan(""), std::nan("")};
  const double h_step = 2e-5;
  for (double t = 0.0; t < kDuration && std::isnan(times[1]); t += h_step) {
    const auto step = [&](const State& d, double f) {
      State z;
      for (int i = 0; i < 6; ++i) z[i] = y[i] + f * d[i];
      return z;
    };
    const double before = y[2];
    const State k1 = rate(y), k2 = rate(step(k1, h_step / 2)), k3 = rate(step(k2, h_step / 2)), k4 = rate(step(k3, h_step));
    for (int i = 0; i < 6; ++i) y[i] += h_step / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    for (int k = 0; k < 2; ++k) {
      if (std::isnan(times[k]) && y[2] >= kPitch[k]) times[k] = t + h_step * (kPitch[k] - before) / (y[2] - before);
    }
  }
  return times;
}

// A box sliding without friction off the edge of a table. The theory is the rigid one, so a
// foundation of finite stiffness is charged for the lean it allows; what the Winkler foundation
// of this K predicts is reported next to it (`pitch_time_<k>_winkler`).
Scenario edge(const ScenarioOptions& o) {
  Scenario s;
  s.name = "edge";
  s.description = "Box sliding at 1 m/s off the edge of a table, frictionless";
  s.theory_class = TheoryClass::kA;
  s.gravity = Vec3(0.0, 0.0, -kG);
  s.duration = kDuration;

  BodySpec& body = s.bodies.emplace_back();
  body.name = "box";
  body.shape = shape_meshes(o.data_dir, "box_300x200x100");
  body.mass = kMass;
  body.inertia = box_inertia(kMass, kLength, kWidth, kHeight);
  body.X_W0.p = Vec3(kTableHalf - kStart, 0.0, kHeight / 2 - kInitialPenetration);
  body.V_W0.v = Vec3(kSpeed, 0.0, 0.0);
  body.primitive = Primitive{.kind = Primitive::Kind::kBox, .size = Vec3(kLength, kWidth, kHeight), .X_BG = {}};

  s.grounds.push_back({.shape = shape_meshes(o.data_dir, "table_1000x600x200")});
  s.contacts.push_back(standard_contact(BodyRef::body(0), BodyRef::ground(0), kK, kDamping, 0.0));
  s.native = {.stiffness = kK, .damping = kDamping};

  // The theory's own crossing times, from its pitch every 0.1 ms.
  std::vector<double> fine;
  for (int i = 0; i * 1e-4 <= kDuration + 1e-12; ++i) fine.push_back(i * 1e-4);
  const std::vector<double> fine_pitch = edge_pitch(fine);
  std::array<double, 2> theory{};
  for (int k = 0; k < 2; ++k) theory[k] = rising_time(fine, fine_pitch, kPitch[k]);

  static const std::array<double, 2> winkler = edge_winkler_pitch_times(kK, kDamping);

  s.metrics = [theory](const Trajectory& traj) {
    MetricReport r;
    Series pitch{"pitch", {}}, pitch_th{"pitch_theory", edge_pitch(traj.times())};
    double lateral = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Pose& X = traj.pose(i, 0);
      pitch.values.push_back(std::atan2(-X.R(2, 0), X.R(0, 0)));  // Nose down is positive.
      lateral = std::max(lateral, std::abs(X.p.y()));
    }
    // The theory fixes when the box starts to pitch and how fast: the times at which the pitch
    // reaches 0.1 and 0.25 rad, counted from the time the centre of mass is over the edge.
    ErrorTerms terms;
    for (int k = 0; k < 2; ++k) {
      const std::string name = "pitch_time_" + std::to_string(k + 1);
      const double measured = rising_time(traj.times(), pitch.values, kPitch[k]);
      terms.add(name, (measured - theory[k]) / (theory[k] - kOnset));
      r.scalars.emplace_back(name, measured);
      r.scalars.emplace_back(name + "_theory", theory[k]);
      r.scalars.emplace_back(name + "_winkler", winkler[k]);
    }
    r.scalars.emplace_back("onset_time_theory", kOnset);
    r.scalars.emplace_back("lateral_drift_max_m", lateral);
    r.scalars.emplace_back("lateral_drift_max_m_theory", 0.0);
    terms.finish(r);
    r.series = {std::move(pitch), std::move(pitch_th)};
    return r;
  };
  return s;
}

}  // namespace volcontact::scenarios
