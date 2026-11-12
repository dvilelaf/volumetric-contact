#include <cmath>
#include <map>
#include <numbers>
#include <set>
#include <stdexcept>

#include <doctest/doctest.h>

#include "common.hpp"
#include "volcontact/scenarios.hpp"

using namespace volcontact;
using namespace volcontact::scenarios;

namespace {

const ScenarioOptions kOptions{.data_dir = VOLCONTACT_DATA_DIR};

std::vector<double> times(std::size_t n, double dt) {
  std::vector<double> t(n);
  for (std::size_t i = 0; i < n; ++i) t[i] = static_cast<double>(i) * dt;
  return t;
}

}  // namespace

TEST_CASE("crossing_time interpolates the first downward crossing of a level") {
  const auto t = times(101, 0.01);
  std::vector<double> x;
  for (double ti : t) x.push_back(2.0 - 2.0 * ti);  // Linear, crosses 0.505 at t = 0.7475.
  CHECK(crossing_time(t, x, 0.505) == doctest::Approx(0.7475).epsilon(1e-12));
  CHECK(crossing_time(t, x, 0.02) == doctest::Approx(0.99).epsilon(1e-12));
  CHECK(std::isnan(crossing_time(t, std::vector<double>(t.size(), 1.0), 0.5)));
}

TEST_CASE("sphere_static_penetration satisfies m g_n = K pi delta^2 (r - delta/3)") {
  const double m = 1.0, g_n = 9.81 * std::cos(0.35), K = 1e8, r = 0.1;
  const double delta = sphere_static_penetration(m, g_n, K, r);
  CHECK(delta > 0.0);
  CHECK(delta < r);
  CHECK(K * std::numbers::pi * delta * delta * (r - delta / 3) == doctest::Approx(m * g_n).epsilon(1e-9));
}

TEST_CASE("mean_period of a sine, with interpolated zero crossings") {
  const double period = 0.7;
  const auto t = times(400, 0.0137);  // Sampling not commensurate with the period.
  std::vector<double> x;
  for (double ti : t) x.push_back(std::sin(2.0 * std::numbers::pi * ti / period + 0.3));
  CHECK(mean_period(t, x) == doctest::Approx(period).epsilon(1e-3));
  CHECK(std::isnan(mean_period(t, std::vector<double>(t.size(), 1.0))));
}

TEST_CASE("mean_after averages the samples from t0 on") {
  const auto t = times(11, 1.0);
  std::vector<double> x(11, 1.0);
  for (std::size_t i = 6; i < 11; ++i) x[i] = 3.0;
  CHECK(mean_after(t, x, 6.0) == doctest::Approx(3.0));
  CHECK(relative_error(1.1, 1.0) == doctest::Approx(0.1));
}

TEST_CASE("perturb adds the prescribed velocity to every dynamic body") {
  Scenario s;
  s.bodies.resize(2);
  s.bodies[1].V_W0 = {Vec3(0, 0, 1), Vec3(1, 0, 0)};
  perturb(s);
  const Vec3 dv = 1e-3 * Vec3(1, 2, 3) / std::sqrt(14.0), dw = 1e-2 * Vec3(3, -1, 2) / std::sqrt(14.0);
  CHECK(s.bodies[0].V_W0.v.isApprox(dv));
  CHECK(s.bodies[0].V_W0.w.isApprox(dw));
  CHECK(s.bodies[1].V_W0.v.isApprox(Vec3(1, 0, 0) + dv));
  CHECK(s.bodies[1].V_W0.w.isApprox(Vec3(0, 0, 1) + dw));
  CHECK(dv.norm() == doctest::Approx(1e-3));
}

TEST_CASE("the catalog has the 21 scenarios of the battery and the 12 of the held-out set, with unique names") {
  std::set<std::string> names;
  for (const auto& e : catalog()) names.insert(e.name);
  CHECK(catalog().size() == 33);
  CHECK(names.size() == 33);
  CHECK_THROWS_AS((void)make_scenario("nothing", kOptions), std::out_of_range);
}

TEST_CASE("every asset of docs/scenarios.md §1 resolves, at both resolutions") {
  const char* fixed[] = {"slab_40000x40000x1000", "box_1000x500x500", "box_400x400x200", "box_200x200x600",
                         "box_400x100x100", "box_100x100x100", "box_400x400x50_o50x0x0", "box_400x400x100",
                         "box_200x200x100", "box_200x200x200", "vgroove_w300_l4000", "hole_s99.8_d100",
                         "corner_s300", "box_300x200x100", "table_1000x600x200", "table_1600x1600x200",
                         "table_1600x1600x200_p2", "table_1600x1600x200_p8", "table_1600x1600x200_p64"};
  const char* round[] = {"cylinder_r250_h50", "cylinder_r250_h1000", "cylinder_r100_h20", "cylinder_r100_h400",
                         "cylinder_r50_h200", "cylinder_r500_h100", "sphere_r100", "frustum_r50_r100_h100", "ring_ro250_ri150_h50",
                         "bowl_r300_t20_a60", "trough_r300_t20_a60_l500", "bearing_r50_t20_l200",
                         "bearing_r60_t20_l200",
                         "seat_r50_r100_h100_t20"};
  for (const char* name : fixed) {
    const ShapeMeshes m = shape_meshes(kOptions.data_dir, name);
    CHECK_MESSAGE(std::filesystem::exists(m.surface), name);
    CHECK_MESSAGE(!m.pieces.empty(), name);
  }
  for (const Resolution r : {Resolution::kBase, Resolution::kFine}) {
    const ScenarioOptions o{.data_dir = VOLCONTACT_DATA_DIR, .resolution = r};
    for (const char* name : round) {
      const ShapeMeshes m = shape_meshes(o.data_dir, curved(o, name));
      CHECK_MESSAGE(std::filesystem::exists(m.surface), name);
      CHECK_MESSAGE(!m.pieces.empty(), name);
    }
  }
  CHECK(shape_meshes(kOptions.data_dir, "vgroove_w300_l4000").pieces.size() == 2);
  CHECK(shape_meshes(kOptions.data_dir, "box_1000x500x500").pieces.size() == 1);
  CHECK(shape_meshes(kOptions.data_dir, "hole_s99.8_d100").pieces.size() == 4);
  CHECK(shape_meshes(kOptions.data_dir, "table_1600x1600x200").pieces.size() == 1);
  CHECK(shape_meshes(kOptions.data_dir, "table_1600x1600x200_p64").pieces.size() == 64);
  CHECK(shape_meshes(kOptions.data_dir, "box_1000x500x500").volume.has_value());
  CHECK_FALSE(shape_meshes(kOptions.data_dir, "slab_40000x40000x1000").volume.has_value());
  CHECK_THROWS_AS((void)shape_meshes(kOptions.data_dir, "nothing"), std::invalid_argument);
}

TEST_CASE("thesis scenarios are built on the new assets") {
  for (const char* name : {"block", "disk", "cylinder"}) {
    const Scenario s = make_scenario(name, kOptions);
    CHECK(s.name == name);
    REQUIRE(s.grounds.size() == 1);
    CHECK(s.grounds[0].planar);
    CHECK(s.contacts[0].friction > 0.0);
  }
}

TEST_CASE("every scenario states the class of its theory (docs/scenarios.md §4)") {
  const std::map<std::string, TheoryClass> expected = {
      {"block", TheoryClass::kA},    {"disk", TheoryClass::kB},      {"cylinder", TheoryClass::kA},
      {"push", TheoryClass::kA},     {"tip", TheoryClass::kC},       {"drop", TheoryClass::kC},
      {"billiard", TheoryClass::kA}, {"contensou", TheoryClass::kB}, {"vgroove", TheoryClass::kA},
      {"ring", TheoryClass::kB},     {"hole", TheoryClass::kC},      {"bowl", TheoryClass::kA},
      {"trough", TheoryClass::kA},   {"bearing", TheoryClass::kB},   {"corner", TheoryClass::kC},
      {"offset", TheoryClass::kC},   {"twoblocks", TheoryClass::kA}, {"stack", TheoryClass::kC},
      {"cone", TheoryClass::kB},     {"sphere", TheoryClass::kA},    {"journal", TheoryClass::kA},
      {"pieces1", TheoryClass::kA},  {"pieces2", TheoryClass::kA},   {"pieces8", TheoryClass::kA},
      {"pieces64", TheoryClass::kA}, {"wedge", TheoryClass::kA},     {"restitution", TheoryClass::kC},
      {"oblique", TheoryClass::kA},  {"edge", TheoryClass::kA},      {"energy", TheoryClass::kA},
      {"turntable", TheoryClass::kA},
      {"tower", TheoryClass::kA},    {"massratio", TheoryClass::kA},
  };
  for (const auto& e : catalog()) {
    const auto it = expected.find(e.name);
    REQUIRE_MESSAGE(it != expected.end(), e.name);
    CHECK_MESSAGE(e.make(kOptions).theory_class == it->second, e.name);
  }
  CHECK(to_string(TheoryClass::kA) == "A");
  CHECK(to_string(TheoryClass::kB) == "B");
  CHECK(to_string(TheoryClass::kC) == "C");
}

TEST_CASE("ErrorTerms: the error is the worst term, each kept as err_<name>; a NaN term makes it NaN") {
  MetricReport r;
  ErrorTerms terms;
  terms.add("velocity", -0.02);
  terms.add_length("creep", 0.035, 0.4);
  terms.add_residual_speed(1e-3, 2.0, 0.1);
  terms.finish(r);
  CHECK(r.error == doctest::Approx(0.0875));
  REQUIRE(r.scalars.size() == 3);
  CHECK(r.scalars[0] == std::pair<std::string, double>("err_velocity", 0.02));
  CHECK(r.scalars[1].first == "err_creep");
  CHECK(r.scalars[2].second == doctest::Approx(0.02));
  terms.add("stop_time", std::nan(""));
  MetricReport bad;
  terms.finish(bad);
  CHECK(std::isnan(bad.error));
}

TEST_CASE("add_band: band_error is the distance to the band of admissible answers over the nominal value") {
  for (const auto& [measured, expected] : {std::pair{0.9, 0.0}, std::pair{1.2, 0.0}, std::pair{1.5, 0.2},
                                           std::pair{0.6, 0.2}}) {
    MetricReport r;
    add_band(r, "stop_time", measured, 1.0, {{"uniform_wear", 1.3}, {"rigid_punch", 0.8}});
    REQUIRE(r.scalars.size() == 5);
    CHECK(r.scalars[0] == std::pair<std::string, double>("stop_time_uniform_wear", 1.3));
    CHECK(r.scalars[2] == std::pair<std::string, double>("stop_time_band_min", 0.8));
    CHECK(r.scalars[3] == std::pair<std::string, double>("stop_time_band_max", 1.3));
    CHECK(r.scalars[4].first == "band_error");
    CHECK(r.scalars[4].second == doctest::Approx(expected));
  }
}

TEST_CASE("fit_parabola, last_peak and agm") {
  std::vector<double> t, x, osc;
  for (int i = 0; i <= 2000; ++i) {
    t.push_back(1e-3 * i);
    x.push_back(0.3 + 2.0 * t.back() - 0.5 * t.back() * t.back());
    osc.push_back(std::exp(-0.2 * t.back()) * std::cos(2.0 * std::numbers::pi * t.back() / 0.7));
  }
  const auto p = fit_parabola(t, x, 0.2);
  CHECK(p[0] == doctest::Approx(0.3));
  CHECK(p[1] == doctest::Approx(2.0));
  CHECK(p[2] == doctest::Approx(-0.5));
  // Extremes every 0.35 s: the last one in the run is at 1.75 s.
  const auto [when, peak] = last_peak(t, osc, 0.35);
  CHECK(when == doctest::Approx(1.75).epsilon(0.01));
  CHECK(peak == doctest::Approx(std::exp(-0.35)).epsilon(1e-3));
  // Pendulum at 60°: T/T0 = 1.07318.
  CHECK(1.0 / agm(1.0, std::cos(std::numbers::pi / 6)) == doctest::Approx(1.073182).epsilon(1e-6));
}
