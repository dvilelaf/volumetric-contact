#include <algorithm>
#include <cmath>
#include <string_view>
#include <vector>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};

std::vector<double> uniform_times(double dt, double duration) {
  std::vector<double> t;
  for (int i = 0; i * dt <= duration + 1e-12; ++i) t.push_back(i * dt);
  return t;
}

// A single box at height z[i] moving upwards at vz[i] at times[i].
Trajectory box_trajectory(const std::vector<double>& times, const std::vector<double>& z,
                          const std::vector<double>& vz) {
  Trajectory traj(1);
  for (std::size_t i = 0; i < times.size(); ++i) {
    Pose X;
    X.p = Vec3(0.0, 0.0, z[i]);
    Twist V;
    V.v = Vec3(0.0, 0.0, vz[i]);
    traj.push(times[i], std::vector{X}, std::vector{V});
  }
  return traj;
}

const Series& find_series(const MetricReport& r, std::string_view name) {
  return *std::find_if(r.series.begin(), r.series.end(), [&](const Series& s) { return s.name == name; });
}

double find_scalar(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

}  // namespace

TEST_CASE("drop is a 10 kg box of the true size, dropped flat from 20 mm, at rest") {
  const Scenario s = make_scenario("drop", kOptions);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& box = s.bodies[0];
  CHECK(s.name == "drop");
  CHECK(box.mass == doctest::Approx(10.0));
  CHECK(box.inertia.isApprox(box_inertia(10.0, 0.4, 0.4, 0.2)));
  CHECK(s.gravity.isApprox(Vec3(0.0, 0.0, -9.81)));
  CHECK(s.duration == doctest::Approx(0.5));
  CHECK(box.X_W0.R.isIdentity());
  CHECK(box.X_W0.p.z() - 0.1 == doctest::Approx(0.02));  // Bottom face, above the plane z = 0.
  CHECK(box.V_W0.v.isZero());
  CHECK(box.V_W0.w.isZero());
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
}

TEST_CASE("drop builds at both resolutions, and its contact model builds for both laws") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("drop", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    CHECK(static_cast<bool>(s.metrics));
    for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, model).pairs().size() == 1);
    }
  }
}

TEST_CASE("the theory rebounds below the impact speed, after a penetration above the static one") {
  const Scenario s = make_scenario("drop", kOptions);
  const std::vector<double> times = uniform_times(1e-3, s.duration);
  const MetricReport theory =
      s.metrics(box_trajectory(times, std::vector<double>(times.size(), 0.12), std::vector<double>(times.size(), 0.0)));
  const double v_in = std::sqrt(2.0 * 9.81 * 0.02);
  const double rebound = find_scalar(theory, "rebound_velocity_theory");
  const double penetration = find_scalar(theory, "penetration_max_theory_m");
  const double static_penetration = 10.0 * 9.81 / (1e6 * 0.16);
  CHECK(rebound > 0.0);
  CHECK(rebound < v_in);
  CHECK(penetration > static_penetration);
  CHECK(penetration < static_penetration + v_in / std::sqrt(1e6 * 0.16 / 10.0));
}

TEST_CASE("drop's error is within 1e-3 on a trajectory that follows the theory") {
  const Scenario s = make_scenario("drop", kOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const std::vector<double> times = uniform_times(dt, s.duration);
    // The theory series, read at the sample times of a box at rest in the air.
    const MetricReport rest = s.metrics(box_trajectory(times, std::vector<double>(times.size(), 0.12),
                                                       std::vector<double>(times.size(), 0.0)));
    const Series& delta = find_series(rest, "delta_theory");
    const Series& up = find_series(rest, "v_theory");

    std::vector<double> z, vz;
    for (std::size_t i = 0; i < times.size(); ++i) {
      z.push_back(0.1 - delta.values[i]);
      vz.push_back(up.values[i]);
    }
    const MetricReport r = s.metrics(box_trajectory(times, z, vz));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
  }
}
