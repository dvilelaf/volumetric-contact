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

// The slab with its origin at (0, 0, z) and rotated by `tilt` about y, +x side down.
Trajectory slab_trajectory(const std::vector<double>& times, double z, double tilt) {
  const double c = std::cos(tilt), s = std::sin(tilt);
  Mat3 R;
  R << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
  Trajectory traj(1);
  for (const double t : times) {
    Pose X;
    X.p = Vec3(0.0, 0.0, z);
    X.R = R;
    traj.push(t, std::vector{X}, std::vector{Twist{}});
  }
  return traj;
}

double find_scalar(const MetricReport& r, std::string_view name) {
  for (const auto& [key, value] : r.scalars) {
    if (key == name) return value;
  }
  return std::nan("");
}

}  // namespace

TEST_CASE("offset is a 10 kg slab with its COM 50 mm from its centroid, flat and resting") {
  const Scenario s = make_scenario("offset", kOptions);
  REQUIRE(s.bodies.size() == 1);
  const BodySpec& slab = s.bodies[0];
  CHECK(s.name == "offset");
  CHECK(slab.mass == doctest::Approx(10.0));
  // Parallel axis from the centroid, 0.05 m away along x: m·e² added to yy and zz.
  const Mat3 centroid = box_inertia(10.0, 0.4, 0.4, 0.05);
  const Mat3 parallel = Vec3(0.0, 0.025, 0.025).asDiagonal();
  CHECK(slab.inertia.isApprox(centroid + parallel));
  CHECK(s.gravity.isApprox(Vec3(0.0, 0.0, -9.81)));
  CHECK(s.duration == doctest::Approx(1.0));
  CHECK(slab.X_W0.R.isIdentity());
  CHECK(slab.V_W0.v.isZero());
  CHECK(slab.V_W0.w.isZero());
  // The slab's bottom face is 1 µm into the plane, as every resting body starts.
  CHECK(slab.X_W0.p.z() - 0.025 == doctest::Approx(-1e-6));
  REQUIRE(s.grounds.size() == 1);
  CHECK(s.grounds[0].planar);
}

TEST_CASE("offset builds at both resolutions, and its contact model builds for both laws") {
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const Scenario s = make_scenario("offset", {.data_dir = VOLCONTACT_DATA_DIR, .resolution = r});
    CHECK(static_cast<bool>(s.metrics));
    for (const ForceModel model : {ForceModel::kGonthier, ForceModel::kDistributed}) {
      CHECK(make_contact_model(s, model).pairs().size() == 1);
    }
  }
}

TEST_CASE("the theory tilts the slab towards its COM, and its uphill edge stays in contact") {
  const Scenario s = make_scenario("offset", kOptions);
  const MetricReport flat = s.metrics(slab_trajectory(uniform_times(1e-3, s.duration), 0.025, 0.0));
  const double tilt = find_scalar(flat, "tilt_theory");
  const double sinking = find_scalar(flat, "sinking_m_theory");
  // θ = m·g·e/(K·I_A − m·g·z_c) with I_A = 0.4⁴/12 and z_c = 0.025 m.
  CHECK(tilt == doctest::Approx(9.81 * 0.5 / (1e6 * 0.4 * 0.4 * 0.4 * 0.4 / 12 - 10.0 * 9.81 * 0.025)));
  CHECK(tilt > 0.0);
  CHECK(sinking == doctest::Approx(10.0 * 9.81 / (1e6 * 0.16)));
  CHECK(sinking - tilt * 0.2 > 0.0);  // Uphill edge, at x = −0.2 m, still in contact.
}

TEST_CASE("offset's error is within 1e-3 on a trajectory that sits at the theoretical tilt") {
  const Scenario s = make_scenario("offset", kOptions);
  for (const double dt : {1e-3, 2e-4}) {
    const std::vector<double> times = uniform_times(dt, s.duration);
    const MetricReport theory = s.metrics(slab_trajectory(times, 0.025, 0.0));
    const double tilt = find_scalar(theory, "tilt_theory");
    const MetricReport r = s.metrics(slab_trajectory(times, 0.025, tilt));
    CHECK_MESSAGE(r.error <= 1e-3, "dt = " << dt << ", error = " << r.error);
  }
}

TEST_CASE("offset's error is 1 for a slab that stays flat") {
  const Scenario s = make_scenario("offset", kOptions);
  const MetricReport r = s.metrics(slab_trajectory(uniform_times(1e-3, s.duration), 0.025, 0.0));
  CHECK(r.error == doctest::Approx(1.0));
}

TEST_CASE("offset: class C; a slab still moving at 4 mm/s is charged that drift over its 0.4 m side") {
  const Scenario s = make_scenario("offset", kOptions);
  CHECK(s.theory_class == TheoryClass::kC);
  const double theta = find_scalar(s.metrics(slab_trajectory(uniform_times(1e-2, s.duration), 0.025, 0.0)), "tilt_theory");
  const Trajectory rest = slab_trajectory(uniform_times(1e-3, s.duration), 0.025, theta);
  Trajectory traj(1);
  for (std::size_t i = 0; i < rest.size(); ++i) {
    const Twist V{Vec3::Zero(), Vec3(4e-3, 0.0, 0.0)};
    traj.push(rest.times()[i], std::vector{rest.pose(i, 0)}, std::vector{V});
  }
  const MetricReport r = s.metrics(traj);
  CHECK(find_scalar(r, "err_tilt") < 1e-9);
  CHECK(find_scalar(r, "err_residual_speed") == doctest::Approx(0.01));
  CHECK(r.error == doctest::Approx(0.01));
}
