// Invariants of the distributed law and of the intersection (docs/research/core-audit.md §7):
// the result does not depend on how a body is cut into convex pieces nor on how its faces are
// triangulated, the elastic force is conservative, and a deep overlap keeps its force.
#include <cmath>
#include <filesystem>
#include <numbers>
#include <utility>
#include <vector>

#include <doctest/doctest.h>

#include "test_util.hpp"
#include "volcontact/contact_model.hpp"
#include "volcontact/intersection.hpp"

using namespace volcontact;
using volcontact::test::make_box;

namespace {

// Body 0 (one shape) against the static body 1 (a compound), with one law.
struct Rig {
  ContactModel model;
  ContactModel::Evaluation eval;
  std::vector<FrictionState> state;

  Rig(const std::shared_ptr<const CollisionShape>& a, const CompoundShape& ground, const ContactLaw& law) {
    for (const auto& piece : ground.pieces) {
      model.add_pair({.body_a = 0, .body_b = 1, .shape_a = a, .shape_b = piece, .law = law});
    }
    state = model.initial_friction();
  }

  // Wrench on body 0 about its origin; the friction state advances.
  Wrench step(const Pose& X, const Twist& V = {}) {
    const std::vector<Pose> poses{X, Pose{}};
    const std::vector<Twist> twists{V, Twist{}};
    model.evaluate(poses, twists, state, &eval);
    state = eval.next_friction;
    return eval.wrenches[0];
  }
  // The same from a fresh state, which is left untouched.
  Wrench at(const Pose& X, const Twist& V = {}) {
    const std::vector<FrictionState> saved = std::exchange(state, model.initial_friction());
    const Wrench w = step(X, V);
    state = saved;
    return w;
  }
};

std::shared_ptr<const CollisionShape> shape(TriMesh mesh) {
  return std::make_shared<const CollisionShape>(std::move(mesh));
}

DistributedParams kinetic(double mu, double h) {
  DistributedParams p{.stiffness = 1e6, .damping = h, .mu_dynamic = mu, .mu_static = mu};
  p.stick.reset();
  return p;
}

// The slab [-5, 5]² × [-1, 0] whole, cut in two (x), and cut in eight (x, y and z).
CompoundShape slab(int pieces) {
  const double xs[3] = {-5, 0.05, 5}, ys[3] = {-5, -0.03, 5}, zs[3] = {-1, -0.3, 0};
  std::vector<TriMesh> meshes;
  if (pieces == 1) meshes.push_back(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));
  if (pieces == 2) {
    for (int i = 0; i < 2; ++i) meshes.push_back(make_box(Vec3(xs[i], -5, -1), Vec3(xs[i + 1], 5, 0)));
  }
  if (pieces == 8) {
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        for (int k = 0; k < 2; ++k) {
          meshes.push_back(make_box(Vec3(xs[i], ys[j], zs[k]), Vec3(xs[i + 1], ys[j + 1], zs[k + 1])));
        }
      }
    }
  }
  return CompoundShape::from_meshes(std::move(meshes));
}

// The same slab cut in two by a plane at 45° through the line x = 0.05, z = 0.
CompoundShape slab_cut_obliquely() {
  TriMesh left = make_box(Vec3(-5, -5, -1), Vec3(0.05, 5, 0)), right = make_box(Vec3(0.05, -5, -1), Vec3(5, 5, 0));
  for (Vec3& v : left.vertices) {
    if (v.x() > 0.0 && v.z() < -0.5) v.x() = 1.05;
  }
  for (Vec3& v : right.vertices) {
    if (v.x() < 1.0 && v.z() < -0.5) v.x() = 1.05;
  }
  return CompoundShape::from_meshes({left, right});
}

// make_box with the other diagonal on every face.
TriMesh make_box_other_diagonals(const Vec3& lo, const Vec3& hi) {
  TriMesh m = make_box(lo, hi);
  for (std::size_t q = 0; q < 6; ++q) {
    const auto a = m.triangles[2 * q], b = m.triangles[2 * q + 1];  // (q0, q1, q2) and (q0, q2, q3).
    m.triangles[2 * q] = {a[0], a[1], b[2]};
    m.triangles[2 * q + 1] = {a[1], a[2], b[2]};
  }
  return m;
}

double interior_area(const CollisionShape& s) {
  double area = 0.0;
  for (std::size_t t = 0; t < s.mesh().triangles.size(); ++t) {
    const auto v = s.mesh().triangle(t);
    if (s.triangle_interior(t)) area += 0.5 * (v[1] - v[0]).cross(v[2] - v[0]).norm();
  }
  return area;
}

double area(const TriMesh& m) {
  double sum = 0.0;
  for (std::size_t t = 0; t < m.triangles.size(); ++t) {
    const auto v = m.triangle(t);
    sum += 0.5 * (v[1] - v[0]).cross(v[2] - v[0]).norm();
  }
  return sum;
}

// The convex pieces of an asset of the battery.
CompoundShape compound(const std::filesystem::path& pieces) {
  std::vector<std::filesystem::path> paths;
  for (const auto& e : std::filesystem::directory_iterator(pieces)) paths.push_back(e.path());
  return CompoundShape::from_objs(paths);
}

double relative_difference(const Wrench& a, const Wrench& b) {
  return std::max((a.force - b.force).norm() / b.force.norm(), (a.torque - b.torque).norm() / b.torque.norm());
}

const auto kBox = shape(make_box(Vec3(-0.2, -0.2, -0.1), Vec3(0.2, 0.2, 0.1)));
const Pose kResting{Mat3::Identity(), Vec3(0, 0, 0.1 - 1e-3)};  // 1 mm into the slab.

}  // namespace

TEST_CASE("a compound flags the faces between its pieces, also where they overlap in part") {
  // A cube with a half-height box on its +x face and a full box on its −x face.
  const CompoundShape c = CompoundShape::from_meshes({make_box(Vec3(0, 0, 0), Vec3(1, 1, 1)),
                                                      make_box(Vec3(1, 0, 0), Vec3(2, 1, 0.5)),
                                                      make_box(Vec3(-1, 0, 0), Vec3(0, 1, 1))});
  REQUIRE(c.pieces.size() == 3);
  CHECK(interior_area(*c.pieces[0]) == doctest::Approx(1.5));
  CHECK(interior_area(*c.pieces[1]) == doctest::Approx(0.5));
  CHECK(interior_area(*c.pieces[2]) == doctest::Approx(1.0));
  // Re-triangulating the partly covered face keeps the pieces closed and convex.
  CHECK(signed_volume(c.pieces[0]->mesh()) == doctest::Approx(1.0));
  for (const auto& piece : c.pieces) CHECK(piece->convex());
  // The plane of a face that is partly seam still belongs to the surface; a full seam does not.
  int exterior_planes = 0;
  for (std::size_t i = 0; i < c.pieces[0]->face_planes().size(); ++i) exterior_planes += c.pieces[0]->plane_exterior(i);
  CHECK(exterior_planes == 5);

  // A lone piece and pieces that do not touch have no seams.
  const CompoundShape apart = CompoundShape::from_meshes({make_box(Vec3(0, 0, 0), Vec3(1, 1, 1)),
                                                          make_box(Vec3(1.5, 0, 0), Vec3(2, 1, 1))});
  for (const auto& piece : apart.pieces) CHECK(interior_area(*piece) == 0.0);
}

TEST_CASE("the faces of the pieces that are not seams add up to the surface of the body") {
  for (const char* asset : {"grounds/bearing_r50_t20_l200_seg64", "grounds/bowl_r105_t20_a60_seg64",
                            "grounds/bowl_r300_t20_a60_seg64", "grounds/seat_r50_r100_h100_t20_seg64",
                            "grounds/trough_r300_t20_a60_l500_seg64",
                            "grounds/hole_s99.8_d100", "grounds/corner_s300", "grounds/vgroove_w300_l4000",
                            "bodies/ring_ro250_ri150_h50_seg64"}) {
    const std::filesystem::path folder = std::filesystem::path(VOLCONTACT_DATA_DIR) / asset;
    const CompoundShape body = compound(folder / "pieces");
    double exterior = 0.0, seams = 0.0;
    for (const auto& piece : body.pieces) {
      seams += interior_area(*piece);
      exterior += area(piece->mesh()) - interior_area(*piece);
    }
    CAPTURE(asset);
    CHECK(seams > 0.0);
    CHECK(exterior == doctest::Approx(area(load_obj(folder / "surface.obj"))).epsilon(1e-9));
  }
}

TEST_CASE("the wrench on a sliding, spinning box does not depend on how the slab is cut") {
  const Twist V{Vec3(0, 0, 10.0), Vec3(1.0, 0.3, -0.02)};
  Rig whole(kBox, slab(1), kinetic(0.5, 5.0));
  const Wrench reference = whole.at(kResting, V);
  for (const int pieces : {2, 8}) {
    Rig cut(kBox, slab(pieces), kinetic(0.5, 5.0));
    CHECK(relative_difference(cut.at(kResting, V), reference) < 1e-9);
  }
}

TEST_CASE("the normal force of a rocking box does not depend on how the slab is cut") {
  // The damping factor 1 − h·vn changes sign inside the patch (vn = ±2 m/s at its edges).
  const Twist V{Vec3(10.0, -4.0, 0), Vec3(0, 0, -0.05)};
  Rig whole(kBox, slab(1), kinetic(0.0, 5.0));
  const Wrench reference = whole.at(kResting, V);
  for (const int pieces : {2, 8}) {
    Rig cut(kBox, slab(pieces), kinetic(0.0, 5.0));
    CHECK(relative_difference(cut.at(kResting, V), reference) < 1e-9);
  }
}

TEST_CASE("columns cut by an oblique seam still carry their whole load") {
  // The columns run along z and the seam is at 45°: near it a column starts in
  // one piece and ends in the other.
  const Twist V{Vec3::Zero(), Vec3(1.0, 0.3, -0.02)};
  Rig whole(kBox, slab(1), kinetic(0.5, 5.0)), cut(kBox, slab_cut_obliquely(), kinetic(0.5, 5.0));
  const Wrench a = cut.at(kResting, V), b = whole.at(kResting, V);
  CHECK((a.force - b.force).norm() < 1e-9 * b.force.norm());
  // Friction acts at each end of a column in proportion to the load it carries there.
  CHECK((a.torque - b.torque).norm() < 1e-4 * b.torque.norm());
}

TEST_CASE("stiction at slow sliding does not depend on how the slab is cut") {
  const Twist V{Vec3::Zero(), Vec3(0.01, 0.004, 0)};
  const DistributedParams law{.stiffness = 1e6, .damping = 5.0, .mu_dynamic = 0.5, .mu_static = 0.6};
  Rig whole(kBox, slab(1), law), cut(kBox, slab(8), law);
  Pose X = kResting;
  for (int i = 0; i < 5; ++i) {  // The bristles load as the box moves.
    X.p += 1e-3 * V.v;
    const Wrench a = cut.step(X, V), b = whole.step(X, V);
    CHECK(relative_difference(a, b) < 1e-9);
  }
}

TEST_CASE("a centred box spinning in place gets no lateral force") {
  Rig rig(kBox, slab(1), kinetic(0.5, 5.0));
  const Wrench w = rig.at(kResting, Twist{Vec3(0, 0, 10.0), Vec3::Zero()});
  CHECK(std::abs(w.force.x()) < 1e-9 * w.force.z());
  CHECK(std::abs(w.force.y()) < 1e-9 * w.force.z());
  CHECK(w.force.z() == doctest::Approx(160.0));
}

TEST_CASE("the normal force does not depend on the triangulation of the faces") {
  const auto other = shape(make_box_other_diagonals(Vec3(-0.2, -0.2, -0.1), Vec3(0.2, 0.2, 0.1)));
  const Twist V{Vec3(10.0, -4.0, 3.0), Vec3(0.2, 0, -0.05)};
  Pose X = kResting;
  X.R = volcontact::test::rot(0.002, Vec3(1, 2, 0));  // Tilted: the column length varies too.
  for (const bool adhesion : {false, true}) {
    DistributedParams law = kinetic(0.0, 5.0);
    law.allow_adhesion = adhesion;
    Rig a(kBox, slab(1), law), b(other, slab(1), law);
    CHECK(relative_difference(a.at(X, V), b.at(X, V)) < 1e-10);
  }
}

TEST_CASE("a flat contact is damped by the rate at which its columns shorten: K·V·(1 − h·vn)") {
  // vn at the centroid of V, whatever the slip and the rocking, while no point separates faster than 1/h.
  const double K = 1e6, h = 5.0, volume = 0.16 * 1e-3;
  const Twist V{Vec3(0.3, -0.2, 7.0), Vec3(2.0, -1.0, -0.03)};
  Rig rig(kBox, slab(2), kinetic(0.0, h));
  const Wrench w = rig.at(kResting, V);
  CHECK(w.force.z() == doctest::Approx(K * volume * (1.0 + h * 0.03)).epsilon(1e-9));
  // Sliding and spinning alone damp nothing, on any face.
  const Wrench slide = rig.at(kResting, Twist{Vec3(0, 0, 7.0), Vec3(2.0, -1.0, 0)});
  CHECK(slide.force.isApprox(Vec3(0, 0, K * volume), 1e-9));
}

TEST_CASE("a shaft spinning in a bearing of 64 pieces is braked by friction only") {
  const auto shaft = CollisionShape::from_obj(VOLCONTACT_DATA_DIR "/bodies/cylinder_r50_h200_seg64/surface.obj");
  const CompoundShape bearing = compound(VOLCONTACT_DATA_DIR "/grounds/bearing_r50_t20_l200_seg64/pieces");

  // The shaft's axis along y, 0.64 mm below the centre of the hole, at 40 rad/s.
  const double mu = 0.2, r = 0.05;
  const Pose X{volcontact::test::rot(-std::numbers::pi / 2, Vec3::UnitX()), Vec3(0, 0, -0.64e-3)};
  const Twist V{Vec3(0, 40.0, 0), Vec3::Zero()};
  Rig smooth(shaft, bearing, kinetic(0.0, 5.0)), rough(shaft, bearing, kinetic(mu, 5.0));
  const Wrench n = smooth.at(X, V), f = rough.at(X, V);
  // Pressure on a cylinder gives no torque about its axis. On a faceted one the damping loads the
  // leading half of every facet more: a rolling resistance of order h·ω·w²/(12·μ·r) of the friction
  // torque (w: facet width), 4 % here. With pressure and damping on the seams it was 123 %.
  const double friction_torque = f.torque.y() - n.torque.y();
  CHECK(std::abs(n.torque.y()) < 0.05 * std::abs(friction_torque));
  // Winkler: p ∝ cos φ over the lower half, so ∫p dA = (4/π)·N and the friction torque is μ·r·(4/π)·N.
  CHECK(std::abs(friction_torque) == doctest::Approx(mu * r * 4.0 / std::numbers::pi * n.force.z()).epsilon(0.02));
  // The friction resultant is μ times the normal one, turned 90°.
  CHECK((f.force - n.force).norm() == doctest::Approx(mu * n.force.norm()).epsilon(0.02));
}

TEST_CASE("the elastic force of a sphere in the 704-piece bowl does no work over a closed cycle") {
  const auto sphere = CollisionShape::from_obj(VOLCONTACT_DATA_DIR "/bodies/sphere_r100_seg64/surface.obj");
  Rig rig(sphere, compound(VOLCONTACT_DATA_DIR "/grounds/bowl_r105_t20_a60_seg64/pieces"), kinetic(0.0, 0.0));

  // Work over the rectangle in (x, z) from (a0, b0) to (a0 + da, b0 + db), as a fraction of Σ|Q·dq|.
  const auto cycle = [&](double a0, double da, double b0, double db) {
    const int n = 100;
    std::vector<std::pair<double, double>> path;
    for (int i = 0; i <= n; ++i) path.push_back({a0 + da * i / n, b0});
    for (int i = 1; i <= n; ++i) path.push_back({a0 + da, b0 + db * i / n});
    for (int i = 1; i <= n; ++i) path.push_back({a0 + da * (n - i) / n, b0 + db});
    for (int i = 1; i <= n; ++i) path.push_back({a0, b0 + db * (n - i) / n});
    std::vector<Vec3> force;
    for (const auto& [a, b] : path) force.push_back(rig.at(Pose{Mat3::Identity(), Vec3(a, 0.0, 0.1 + b)}).force);
    double work = 0.0, total = 0.0;
    for (std::size_t k = 0; k + 1 < path.size(); ++k) {
      const Vec3 mean = 0.5 * (force[k] + force[k + 1]);
      const double wx = mean.x() * (path[k + 1].first - path[k].first);
      const double wz = mean.z() * (path[k + 1].second - path[k].second);
      work += wx + wz;
      total += std::abs(wx) + std::abs(wz);
    }
    REQUIRE(total > 1e-4);
    return std::abs(work) / total;
  };
  // Resting on the bottom, 0.1 to 0.3 mm deep, 0.5 to 2 mm off-centre: 1.5e-5 (1.2e-2 with the seams loaded).
  CHECK(cycle(0.0005, 0.0015, -1e-4, -2e-4) < 1e-4);
  // 4 to 8 mm off-centre the sphere is wedged 3 mm into the edge of the rim, where the normal of a
  // piece turns with the overlap: 1.3e-4 (3e-3 with the seams loaded).
  CHECK(cycle(0.004, 0.004, -1e-4, -2e-4) < 5e-4);
}

TEST_CASE("a small box pushed fully inside a larger one keeps its restoring force") {
  const auto small = shape(make_box(Vec3::Constant(-0.05), Vec3::Constant(0.05)));
  const CompoundShape big = slab(1);
  const double K = 1e6, volume = 1e-3;
  for (const bool distributed : {true, false}) {
    const ContactLaw law = distributed ? ContactLaw(kinetic(0.0, 0.0)) : ContactLaw(GonthierParams{.damping = 0.0});
    Rig rig(small, big, law);
    // Sinking through the top face: 9 cm in (still poking out), then enclosed, down to 30 cm deep.
    for (const double z : {-0.04, -0.049, -0.051, -0.06, -0.3}) {
      const Wrench w = rig.step(Pose{Mat3::Identity(), Vec3(0.3, 0.1, z)});
      const double expected = K * 0.01 * std::min(0.05 - z, 0.1);  // K·V, along the way it came in.
      CHECK(w.force.z() == doctest::Approx(expected).epsilon(1e-9));
      CHECK(std::hypot(w.force.x(), w.force.y()) < 1e-9 * expected);
    }
    // Starting enclosed, with no history: the shortest way out, here through the −x face.
    const Wrench w = rig.at(Pose{Mat3::Identity(), Vec3(-4.9, 0.1, -0.5)});
    CHECK(w.force.isApprox(Vec3(-K * volume, 0, 0), 1e-9));
  }
}

TEST_CASE("a pin through a plate is pushed back the way it came") {
  // Once through, the plate's two faces inside the pin cancel: the overlap defines no normal.
  const auto pin = shape(make_box(Vec3(-0.01, -0.01, -0.2), Vec3(0.01, 0.01, 0.2)));
  const CompoundShape plate = CompoundShape::from_meshes({make_box(Vec3(-1, -1, -0.05), Vec3(1, 1, 0))});
  Rig rig(pin, plate, kinetic(0.0, 0.0));
  for (const double z : {0.19, 0.16, 0.1}) {  // Tip 1 cm in, then through.
    const Wrench w = rig.step(Pose{Mat3::Identity(), Vec3(0, 0, z)});
    CHECK(w.force.z() == doctest::Approx(1e6 * 4e-4 * std::min(0.2 - z, 0.05)).epsilon(1e-9));
  }
}

TEST_CASE("the fast paths give the same overlap and the same forces as the plain ones") {
  // Plain: convex shapes clipped with their BSP tree, and every face of both shapes taken as a
  // possible end of a column. Fast (default): only the faces near the polygon, in both.
  const auto sphere = CollisionShape::from_obj(VOLCONTACT_DATA_DIR "/bodies/sphere_r100_seg64/surface.obj");
  const auto disk = CollisionShape::from_obj(VOLCONTACT_DATA_DIR "/bodies/cylinder_r250_h50_seg64/surface.obj");
  const CompoundShape bowl = compound(VOLCONTACT_DATA_DIR "/grounds/bowl_r105_t20_a60_seg64/pieces");
  const CompoundShape ground = slab(8);
  const DistributedParams law{.stiffness = 1e6, .damping = 5.0, .mu_dynamic = 0.5, .mu_static = 0.5};
  const Twist V{Vec3(3.0, -2.0, 5.0), Vec3(0.2, 0.1, -0.03)}, still;
  const Pose X_WB;

  struct Case {
    const CollisionShape* a;
    const CompoundShape* b;
    Pose X;
  };
  const Case cases[] = {
      {sphere.get(), &bowl, {volcontact::test::rot(0.3, Vec3(1, 2, 3)), Vec3(0.001, 0.0005, 0.1 - 2e-4)}},
      {sphere.get(), &bowl, {Mat3::Identity(), Vec3(0.006, 0, 0.1 - 2e-4)}},  // Over the edge of the rim too.
      {sphere.get(), &ground, {volcontact::test::rot(1.1, Vec3(3, 1, 2)), Vec3(0.05, -0.03, 0.1 - 1e-3)}},
      {disk.get(), &ground, {volcontact::test::rot(0.002, Vec3(1, 1, 0)), Vec3(0.05, -0.03, 0.025 - 5e-4)}},
      {kBox.get(), &ground, {volcontact::test::rot(0.4, Vec3(1, 2, 0)), Vec3(0.05, -0.03, 0.2)}},  // On a corner.
      {kBox.get(), &ground, {Mat3::Identity(), Vec3(0.05, -0.03, -0.3)}},                  // Enclosed.
  };
  // The two paths cut in a different order, so they differ by slivers as thin as the planes
  // (1e-10 of the shapes): compared on each case as a whole.
  int pairs = 0;
  for (const Case& c : cases) {
    double volume_fast = 0.0, volume_plain = 0.0, error = 0.0;
    Vec3 force_fast = Vec3::Zero(), force_plain = Vec3::Zero(), torque_fast = Vec3::Zero(), torque_plain = Vec3::Zero();
    for (const auto& piece : c.b->pieces) {
      IntersectionWorkspace fast, plain;
      plain.bsp_only = true;
      IntersectionProperties gf, gp;
      ContactSurface sf, sp;
      const bool hit_fast = intersect(*c.a, c.X, *piece, X_WB, fast, &gf, &sf);
      const bool hit_plain = intersect(*c.a, c.X, *piece, X_WB, plain, &gp, &sp);
      if (hit_fast) volume_fast += gf.volume;
      if (hit_plain) volume_plain += gp.volume;
      if (!hit_fast || !hit_plain || gp.exterior_normal.isZero() || gf.exterior_normal.isZero()) continue;
      ++pairs;
      error = std::max(error, (gf.exterior_normal - gp.exterior_normal).norm());
      gf.normal = gf.exterior_normal;
      gp.normal = gp.exterior_normal;

      DistributedWorkspace wf, wp;
      wp.all_exits = true;
      FrictionState next;
      const BodyContext a{*c.a, c.X, V}, b{*piece, X_WB, still};
      const ContactForce ff = distributed_force(gf, sf, a, b, law, {}, &next, wf);
      const ContactForce fp = distributed_force(gp, sp, a, b, law, {}, &next, wp);
      force_fast += ff.force;
      force_plain += fp.force;
      torque_fast += ff.torque + ff.point.cross(ff.force);
      torque_plain += fp.torque + fp.point.cross(fp.force);
    }
    CHECK(volume_fast == doctest::Approx(volume_plain).epsilon(1e-9));
    CHECK(error < 1e-6);
    CHECK((force_fast - force_plain).norm() <= 1e-8 * force_plain.norm());
    CHECK((torque_fast - torque_plain).norm() <= 1e-8 * torque_plain.norm());
  }
  CHECK(pairs > 100);
}

TEST_CASE("sliding and spinning share the static friction of a patch") {
  // Quasi-static moves from rest, far beyond the pre-sliding travel: alone, a slide is held by
  // μ_s·Fn and a twist by the static spinning capacity; together they stay on the limit ellipse.
  const DistributedParams law{.stiffness = 1e6, .damping = 0.0, .mu_dynamic = 0.5, .mu_static = 0.6};
  const auto held = [&](const Vec3& shift, double angle) {
    Rig rig(kBox, slab(1), law);
    rig.step(kResting);
    const Wrench w = rig.step(Pose{volcontact::test::rot(angle, Vec3::UnitZ()), kResting.p + shift});
    return std::pair{std::hypot(w.force.x(), w.force.y()), std::abs(w.torque.z())};
  };
  const double slide = held(Vec3(0.01, 0, 0), 0.0).first, spin = held(Vec3::Zero(), 0.1).second;
  CHECK(slide == doctest::Approx(0.6 * 160.0).epsilon(1e-6));
  CHECK(spin > 0.6 * 160.0 * 0.15);  // μ_s·Fn times the mean radius of a 0.4 m square, 0.153 m.
  for (const double angle : {0.002, 0.02, 0.1}) {
    const auto [force, torque] = held(Vec3(0.004, 0.003, 0), angle);
    CHECK(std::pow(force / slide, 2) + std::pow(torque / spin, 2) == doctest::Approx(1.0).epsilon(1e-6));
  }
}

TEST_CASE("a patch cut into pieces holds a twist like the whole patch") {
  // Each piece sees the twist of the box as a slip of its own centre plus a twist. With each of
  // them limited on its own the four pieces held 41 % more than the friction of the patch.
  const DistributedParams law{.stiffness = 1e6, .damping = 5.0, .mu_dynamic = 0.5, .mu_static = 0.6};
  for (const double rate : {0.001, 0.02, 0.1}) {  // rad/s: loading, at the limit, starting to slip.
    Rig whole(kBox, slab(1), law), cut(kBox, slab(8), law);
    Wrench a, b;
    for (int k = 0; k < 200; ++k) {
      const Pose X{volcontact::test::rot(rate * 1e-3 * k, Vec3::UnitZ()), kResting.p};
      const Twist V{Vec3(0, 0, rate), Vec3::Zero()};
      a = cut.step(X, V);
      b = whole.step(X, V);
    }
    CHECK(a.torque.z() == doctest::Approx(b.torque.z()).epsilon(0.06));
    CHECK(std::hypot(a.force.x(), a.force.y()) < 0.04 * 0.6 * 160.0);  // The whole patch gets none.
  }
}
