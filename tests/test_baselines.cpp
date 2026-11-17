// The baselines of the literature inside the core (docs/baselines.md): the friction quadratures
// of the distributed law (closed form, Hasegawa–Sato's vertex interpolation, one point per
// triangle) and the per-face elastic foundation (PCM, Simbody).
#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <doctest/doctest.h>

#include "test_util.hpp"
#include "volcontact/contact_model.hpp"
#include "volcontact/pcm.hpp"

using namespace volcontact;
using volcontact::test::make_box;

namespace {

constexpr FrictionQuadrature kQuadratures[] = {FrictionQuadrature::kClosedForm, FrictionQuadrature::kVertex,
                                               FrictionQuadrature::kCentroid};

std::shared_ptr<const CollisionShape> shape(TriMesh mesh) {
  return std::make_shared<const CollisionShape>(std::move(mesh));
}

DistributedParams kinetic(FrictionQuadrature quadrature, double mu = 0.5, double h = 0.0) {
  DistributedParams p{.stiffness = 1e6, .damping = h, .mu_dynamic = mu, .mu_static = mu};
  p.stick.reset();
  p.friction_quadrature = quadrature;
  return p;
}

// Wrench on body 0 (one shape) about its origin, against the static pieces of body 1.
Wrench wrench(const std::shared_ptr<const CollisionShape>& a, const CompoundShape& ground, const ContactLaw& law,
              const Pose& X, const Twist& V, double* normal_force = nullptr) {
  ContactModel model;
  for (const auto& piece : ground.pieces) {
    model.add_pair({.body_a = 0, .body_b = 1, .shape_a = a, .shape_b = piece, .law = law});
  }
  const std::vector<Pose> poses{X, Pose{}};
  const std::vector<Twist> twists{V, Twist{}};
  ContactModel::Evaluation eval;
  model.evaluate(poses, twists, model.initial_friction(), &eval);
  if (normal_force) {
    *normal_force = 0.0;
    for (const auto& r : eval.pairs) *normal_force += r.in_contact ? r.force.normal_force : 0.0;
  }
  return eval.wrenches[0];
}

// The slab [-5, 5]² × [-1, 0] whole, cut in two (x), or cut in eight (x, y and z).
CompoundShape slab(int pieces) {
  const double xs[3] = {-5, 0.05, 5}, ys[3] = {-5, -0.03, 5}, zs[3] = {-1, -0.3, 0};
  const int nx = pieces >= 2 ? 2 : 1, nyz = pieces == 8 ? 2 : 1;
  std::vector<TriMesh> meshes;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < nyz; ++j) {
      for (int k = 0; k < nyz; ++k) {
        meshes.push_back(make_box(Vec3(xs[nx == 2 ? i : 0], ys[nyz == 2 ? j : 0], zs[nyz == 2 ? k : 0]),
                                  Vec3(xs[nx == 2 ? i + 1 : 2], ys[nyz == 2 ? j + 1 : 2], zs[nyz == 2 ? k + 1 : 2])));
      }
    }
  }
  return CompoundShape::from_meshes(std::move(meshes));
}

const auto kBox = shape(make_box(Vec3(-0.2, -0.2, -0.1), Vec3(0.2, 0.2, 0.1)));
const Pose kResting{Mat3::Identity(), Vec3(0, 0, 0.1 - 1e-3)};  // 1 mm into the slab.

// Prism over a regular n-gon of circumradius R, between z0 and z1, with both caps meshed as a
// polar grid of `rings` rings: refining n and rings together shrinks every triangle.
TriMesh make_disk(int n, int rings, double R, double z0, double z1) {
  TriMesh m;
  const auto per_cap = static_cast<std::uint32_t>(1 + rings * n);
  const auto vertex = [&](int cap, int ring, int i) {
    return static_cast<std::uint32_t>(cap) * per_cap + 1 + static_cast<std::uint32_t>((ring - 1) * n + i % n);
  };
  for (int cap = 0; cap < 2; ++cap) {
    const double z = cap ? z1 : z0;
    m.vertices.emplace_back(0, 0, z);
    for (int ring = 1; ring <= rings; ++ring) {
      for (int i = 0; i < n; ++i) {
        const double angle = 2.0 * std::numbers::pi * i / n, r = R * ring / rings;
        m.vertices.emplace_back(r * std::cos(angle), r * std::sin(angle), z);
      }
    }
  }
  for (int cap = 0; cap < 2; ++cap) {
    // Counter-clockwise seen from above on the top cap, the opposite on the bottom one.
    const auto add = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
      m.triangles.push_back(cap ? std::array<std::uint32_t, 3>{a, b, c} : std::array<std::uint32_t, 3>{a, c, b});
    };
    for (int i = 0; i < n; ++i) {
      add(static_cast<std::uint32_t>(cap) * per_cap, vertex(cap, 1, i), vertex(cap, 1, i + 1));
      for (int ring = 1; ring < rings; ++ring) {
        add(vertex(cap, ring, i), vertex(cap, ring + 1, i), vertex(cap, ring + 1, i + 1));
        add(vertex(cap, ring, i), vertex(cap, ring + 1, i + 1), vertex(cap, ring, i + 1));
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    m.triangles.push_back({vertex(0, rings, i), vertex(0, rings, i + 1), vertex(1, rings, i + 1)});
    m.triangles.push_back({vertex(0, rings, i), vertex(1, rings, i + 1), vertex(1, rings, i)});
  }
  return m;
}

// Mean distance to the centre over a regular n-gon of circumradius R: with α = π/n and the
// apothem a = R·cos α, ∫r dA over one of its triangles is a³/3·(sec α·tan α + asinh(tan α)).
double ngon_mean_radius(int n, double R) {
  const double alpha = std::numbers::pi / n, a = R * std::cos(alpha), t = std::tan(alpha);
  return a / 3.0 * (1.0 / std::cos(alpha) + std::asinh(t) / t);
}

// One polygon of a hand-made contact surface.
void add_polygon(ContactSurface& s, std::initializer_list<Vec3> x, const Vec3& outward, bool on_a) {
  s.polygons.push_back({.begin = static_cast<std::uint32_t>(s.vertices.size()),
                        .count = static_cast<std::uint32_t>(x.size()),
                        .normal = outward,
                        .on_a = on_a});
  s.vertices.insert(s.vertices.end(), x);
}

}  // namespace

TEST_CASE("on a flat contact with uniform slip the three friction quadratures give the same μ·N") {
  const Twist V{Vec3::Zero(), Vec3(0.3, 0.4, 0)};
  double fn = 0.0;
  const Wrench closed = wrench(kBox, slab(1), kinetic(FrictionQuadrature::kClosedForm), kResting, V, &fn);
  // The thin side strips of the overlap (depth/size = 1/400) carry a little friction of their own.
  CHECK(closed.force.head<2>().isApprox(-0.5 * fn * Eigen::Vector2d(0.6, 0.8), 5e-3));
  for (const auto q : kQuadratures) {
    const Wrench w = wrench(kBox, slab(1), kinetic(q), kResting, V);
    CHECK((w.force - closed.force).norm() <= 1e-13 * closed.force.norm());
    CHECK((w.torque - closed.torque).norm() <= 1e-13 * closed.torque.norm());
  }
}

TEST_CASE("the closed form gives the spinning torque of the faceted disk; the other quadratures converge to it") {
  // A prism over a regular n-gon spinning flat on a ground meshed alike, 10 µm deep. Error of the
  // torque against μ·N·(mean radius of the n-gon), with n segments and n/8 rings:
  //   n    closed form   vertex (Hasegawa–Sato)   one point
  //   16   +3.0e-5       −4.8e-2                  −4.5e-3
  //   32   +3.0e-5       −7.6e-3                  −1.0e-3
  //   64   +3.0e-5       −1.3e-3                  −2.4e-4
  // The closed form's rest is the friction of the thin side strips of the overlap (depth/R = 4e-5),
  // which is part of the model. Both others underestimate: the one-point rule because |r| is
  // convex, the vertex rule mostly in the triangles at the centre of rotation, where the vertex
  // that does not slip carries no friction and the interpolated force grows from zero (a fan
  // triangle about the centre gives ½·μ·p·A·R instead of ⅔).
  const double R = 0.25, depth = 1e-5, mu = 0.6;
  const Pose X{Mat3::Identity(), Vec3(0, 0, 0.025 - depth)};
  const Twist spin{Vec3(0, 0, 10.0), Vec3::Zero()};
  double previous[3] = {0.0, 0.0, 0.0};
  for (const int n : {16, 32, 64}) {
    CAPTURE(n);
    const auto disk = shape(make_disk(n, n / 8, R, -0.025, 0.025));
    const CompoundShape ground{{shape(make_disk(n, n / 4, 2.2 * R, -1.0, 0.0))}};
    double error[3];
    for (int q = 0; q < 3; ++q) {
      double fn = 0.0;
      const Wrench w = wrench(disk, ground, kinetic(kQuadratures[q], mu), X, spin, &fn);
      REQUIRE(fn > 0.0);
      error[q] = std::abs(-w.torque.z() / (mu * fn * ngon_mean_radius(n, R)) - 1.0);
    }
    CHECK(error[0] < 5e-5);
    CHECK(error[2] > 4.0 * error[0]);  // One point.
    CHECK(error[1] > 4.0 * error[2]);  // Vertex.
    if (previous[1] > 0.0) {
      CHECK(error[1] < previous[1] / 4.0);
      CHECK(error[2] < previous[2] / 4.0);
    }
    std::copy(error, error + 3, previous);
  }
}

TEST_CASE("only the closed form is independent of how a spinning contact is cut into pieces") {
  // A box sliding at 1 m/s on a slab whole, cut in two and cut in eight: every quadrature gives
  // the same wrench, since a uniform slip direction needs no quadrature. Spinning at 10 rad/s as
  // well (the centre of rotation is under the box), the wrench changes with the cut by
  //   closed form 8e-16, 6e-16;   vertex 0.17, 0.29;   one point 0.42, 0.75
  // (two and eight pieces, largest relative change of force or torque): the two-triangle faces
  // are all the quadrature has, and a cut gives it other triangles.
  const Twist slide{Vec3::Zero(), Vec3(1.0, 0.3, -0.02)}, spin{Vec3(0, 0, 10.0), Vec3(1.0, 0.3, -0.02)};
  const auto difference = [](const Wrench& a, const Wrench& b) {
    return std::max((a.force - b.force).norm() / b.force.norm(), (a.torque - b.torque).norm() / b.torque.norm());
  };
  for (const auto q : kQuadratures) {
    CAPTURE(static_cast<int>(q));
    const ContactLaw law = kinetic(q, 0.5, 5.0);
    const Wrench sliding = wrench(kBox, slab(1), law, kResting, slide);
    const Wrench spinning = wrench(kBox, slab(1), law, kResting, spin);
    for (const int pieces : {2, 8}) {
      CHECK(difference(wrench(kBox, slab(pieces), law, kResting, slide), sliding) < 1e-9);
      const double change = difference(wrench(kBox, slab(pieces), law, kResting, spin), spinning);
      if (q == FrictionQuadrature::kClosedForm) {
        CHECK(change < 1e-9);
      } else {
        CHECK(change > 0.1);
      }
    }
  }
}

TEST_CASE("the per-face foundation carries K·A·δ on a flat face") {
  // A 0.4 m square face 1 mm deep: K/2·A·δ from its springs and K/2·A·δ from the ground's. The
  // strips of the four side walls inside the ground (P·δ of area, at a mean depth δ/2) find the
  // top of the ground nearest and add K/2·P·δ·δ/2.
  const double K = 1e6, A = 0.16, P = 1.6, d = 1e-3;
  const Wrench w = wrench(kBox, slab(1), PcmParams{.stiffness = K, .damping = 0.0}, kResting, Twist{});
  CHECK(w.force.z() == doctest::Approx(K * A * d + 0.25 * K * P * d * d).epsilon(1e-9));
  CHECK(w.force.head<2>().norm() < 1e-9 * w.force.z());
  // One sample per triangle of a strip does not centre its force: at most the strips' force at the box's edge.
  CHECK(w.torque.norm() < 0.25 * K * P * d * d * 0.2);
  // Closing at 0.1 m/s with h = 2 s/m: (1 + h·ẋ) = 1.2 on every spring.
  const Wrench damped =
      wrench(kBox, slab(1), PcmParams{.stiffness = K, .damping = 2.0}, kResting, Twist{Vec3::Zero(), Vec3(0, 0, -0.1)});
  CHECK(damped.force.z() == doctest::Approx(1.2 * w.force.z()).epsilon(1e-9));
  // Separating faster than 1/h: no tension.
  const Wrench leaving =
      wrench(kBox, slab(1), PcmParams{.stiffness = K, .damping = 2.0}, kResting, Twist{Vec3::Zero(), Vec3(0, 0, 0.6)});
  CHECK(leaving.force.norm() == 0.0);
}

TEST_CASE("the per-face foundation gives Simbody's and Hippmann's forces on two triangles worked by hand") {
  // Two faces of A inside the ground (top at z = 0), K = 1e6 N/m³ (half per body: 5e5):
  //  1. tilted, outward normal (−0.6, 0, −0.8), area 0.0015 m², centroid (0.04/3, 0.02, −0.03);
  //  2. flat, outward normal −z, area 0.005 m², centroid (0.4/3, 0.1/3, −0.01).
  // Simbody (nearest point of the ground, straight up): x₁ = 0.03, x₂ = 0.01, so F₁ = 22.5 N and
  // F₂ = 25 N along z, applied on the ground's top above the centroids.
  // Hippmann (along the face's inward normal (0.6, 0, 0.8)): x₁ = 0.03/0.8 = 0.0375, F₁ = 28.125 N
  // along that normal at the centroid; face 2 as before.
  ContactSurface surface;
  add_polygon(surface, {Vec3(0, 0, -0.02), Vec3(0.04, 0, -0.05), Vec3(0, 0.06, -0.02)}, Vec3(-0.6, 0, -0.8), true);
  add_polygon(surface, {Vec3(0.1, 0, -0.01), Vec3(0.2, 0, -0.01), Vec3(0.1, 0.1, -0.01)}, Vec3(0, 0, -1), true);
  const CollisionShape body(make_box(Vec3::Constant(-0.5), Vec3::Constant(0.5)));
  const CollisionShape ground(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));
  const Pose X_A{Mat3::Identity(), Vec3(0.3, -0.2, 0.4)}, X_B;
  const Twist still;
  IntersectionProperties geo;
  geo.normal = Vec3::UnitZ();
  PcmWorkspace ws;
  const auto evaluate = [&](PcmParams p, const Twist& V_A) {
    return pcm_force(geo, surface, {body, X_A, V_A}, {ground, X_B, still}, p, ws);
  };
  // About the origin.
  const auto moment = [](const ContactForce& f) { return Vec3(f.torque + f.point.cross(f.force)); };
  using Penetration = PcmParams::Penetration;

  PcmParams p{.stiffness = 1e6, .damping = 2.0, .mu_dynamic = 0.5, .mu_static = 0.5};
  SUBCASE("Simbody, at rest") {
    const ContactForce f = evaluate(p, still);
    CHECK(f.force.isApprox(Vec3(0, 0, 47.5), 1e-12));
    CHECK(f.normal_force == doctest::Approx(47.5));
    // Σ x × F with x on z = 0: (y·F, −x·F, 0).
    CHECK(moment(f).isApprox(Vec3(0.02 * 22.5 + 0.1 / 3 * 25, -(0.04 / 3 * 22.5 + 0.4 / 3 * 25), 0), 1e-12));
  }
  SUBCASE("Simbody, sliding at 0.3 m/s and closing at 0.1 m/s") {
    // ẋ = 0.1: F·(1 + 2·0.1) = 27 N and 30 N; friction μ·F against x.
    const ContactForce f = evaluate(p, Twist{Vec3::Zero(), Vec3(0.3, 0, -0.1)});
    CHECK(f.force.isApprox(Vec3(-28.5, 0, 57.0), 1e-12));
  }
  SUBCASE("Simbody, friction below the transition velocity") {
    // v = v_t/2: μ·(v/v_t) = 0.25; with μ_s = 0.7, 0.5·(0.5 + 2·0.2/1.25) = 0.41.
    const Twist slow{Vec3::Zero(), Vec3(0.5 * p.transition_velocity, 0, 0)};
    CHECK(evaluate(p, slow).force.isApprox(Vec3(-0.25 * 47.5, 0, 47.5), 1e-12));
    p.mu_static = 0.7;
    CHECK(evaluate(p, slow).force.isApprox(Vec3(-0.41 * 47.5, 0, 47.5), 1e-12));
  }
  p.penetration = Penetration::kFaceNormal;
  SUBCASE("Hippmann, at rest") {
    const ContactForce f = evaluate(p, still);
    CHECK(f.force.isApprox(Vec3(28.125 * 0.6, 0, 28.125 * 0.8 + 25.0), 1e-12));
  }
  SUBCASE("Hippmann, sliding at 0.3 m/s and closing at 0.1 m/s") {
    // Face 1: ẋ = −v·d = −(0.18 − 0.08) = −0.1, F₁ = 28.125·0.8 = 22.5 N; slip (0.24, 0, −0.18), 0.3 m/s:
    // friction −0.5·22.5·(0.8, 0, −0.6) = (−9, 0, 6.75). Face 2: 30 N and (−15, 0, 0).
    const ContactForce f = evaluate(p, Twist{Vec3::Zero(), Vec3(0.3, 0, -0.1)});
    CHECK(f.force.isApprox(Vec3(13.5 - 9.0 - 15.0, 0, 18.0 + 6.75 + 30.0), 1e-12));
  }
}

TEST_CASE("the per-face foundation needs convex shapes and keeps no state") {
  ContactModel model;
  const auto step = shape(volcontact::test::make_step());
  const auto ground = shape(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));
  CHECK_THROWS(model.add_pair({.body_a = 0, .body_b = 1, .shape_a = step, .shape_b = ground, .law = PcmParams{}}));
  model.add_pair({.body_a = 0, .body_b = 1, .shape_a = kBox, .shape_b = ground, .law = PcmParams{}});
  const std::vector<Pose> poses{kResting, Pose{}};
  const std::vector<Twist> twists(2);
  ContactModel::Evaluation eval;
  model.evaluate(poses, twists, model.initial_friction(), &eval);
  CHECK(eval.pairs[0].in_contact);
  CHECK_FALSE(eval.next_friction[0].active);
  CHECK(eval.next_friction[0].has_normal);
}

TEST_CASE("the bounds of the explicit coupling apply to the baselines as to the laws") {
  // 1 kg pressed by 1000 times its weight, closing at 1 mm/s and sliding at 10 mm/s, at 1 ms: the
  // normal damper takes at most the relative velocity of the step, and the friction of a step
  // at most the sliding velocity (the transition widens to μ·S·dt).
  const double dt = 1e-3, m = 1.0, load = 1000.0 * 9.81, v = 1e-3, slide = 1e-2;
  const auto ground = shape(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));
  DistributedParams vertex, centroid;
  vertex.friction_quadrature = FrictionQuadrature::kVertex;
  centroid.friction_quadrature = FrictionQuadrature::kCentroid;
  const ContactLaw laws[] = {vertex, centroid, PcmParams{}};
  for (const ContactLaw& law : laws) {
    CAPTURE(law.index());
    double normal[2][2], friction[2];  // [bounded][moving].
    for (const bool bounded : {false, true}) {
      ContactModel model;
      model.add_pair({.body_a = 0, .body_b = 1, .shape_a = kBox, .shape_b = ground, .law = law});
      const Mat3 inertia = Vec3(m * 0.17 / 12, m * 0.17 / 12, m * 0.32 / 12).asDiagonal();
      if (bounded) model.set_explicit_coupling(dt, {{1.0 / m, inertia.inverse()}, {}});
      const std::vector<Pose> poses{Pose{Mat3::Identity(), Vec3(0, 0, 0.1 - load / (1e6 * 0.16))}, Pose{}};
      ContactModel::Evaluation eval;
      for (const bool moving : {false, true}) {
        const std::vector<Twist> twists{moving ? Twist{Vec3::Zero(), Vec3(slide, 0, -v)} : Twist{}, Twist{}};
        model.evaluate(poses, twists, model.initial_friction(), &eval);
        normal[bounded][moving] = eval.wrenches[0].force.z();
        if (moving) friction[bounded] = -eval.wrenches[0].force.x();
      }
    }
    const double damper[2] = {normal[0][1] - normal[0][0], normal[1][1] - normal[1][0]};
    CHECK(damper[0] * dt > 10.0 * m * v);  // As given: h·Fn = 49 kN·s/m.
    CHECK(damper[1] > 0.0);
    CHECK(damper[1] * dt <= m * v * (1.0 + 1e-6));
    CHECK(friction[0] * dt > 10.0 * m * slide);
    CHECK(friction[1] > 0.0);
    CHECK(friction[1] * dt <= m * slide * 1.2);  // The block also turns about its centre, 50 mm above.
  }
}
