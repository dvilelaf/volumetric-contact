#include <cmath>
#include <numbers>

#include <doctest/doctest.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <random>
#include <vector>

#include "test_util.hpp"
#include "volcontact/contact_model.hpp"
#include "volcontact/intersection.hpp"
#include "volcontact/detail/volume_integrator.hpp"

using namespace volcontact;
using volcontact::test::make_box;
using volcontact::test::make_step;
using volcontact::test::rot;

namespace {

// Integrates a whole mesh, which must give its own mass properties.
MassProperties integrate(const TriMesh& m) {
  VolumeIntegrator vi;
  for (std::size_t t = 0; t < m.triangles.size(); ++t) {
    const auto v = m.triangle(t);
    vi.add_triangle(v[0], v[1], v[2]);
  }
  return vi.result();
}

IntersectionProperties overlap(const CollisionShape& a, const Pose& X_WA, const CollisionShape& b, const Pose& X_WB,
                               bool* hit) {
  IntersectionWorkspace ws;
  IntersectionProperties p;
  *hit = intersect(a, X_WA, b, X_WB, ws, &p);
  return p;
}

const CollisionShape kUnitCube(make_box(Vec3::Constant(-0.5), Vec3::Constant(0.5)));
const CollisionShape kGround(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));

}  // namespace

TEST_CASE("volume integrator reproduces box mass properties") {
  const MassProperties mp = integrate(make_box(Vec3(1, 2, 3), Vec3(3, 3, 7)));  // 2 × 1 × 4
  CHECK(mp.volume == doctest::Approx(8.0));
  CHECK(mp.centroid.isApprox(Vec3(2, 2.5, 5)));
  const Vec3 expected = 8.0 / 12.0 * Vec3(1 + 16, 4 + 16, 4 + 1);
  CHECK(mp.inertia.diagonal().isApprox(expected));
  CHECK(mp.inertia(0, 1) == doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("OBJ meshes load closed and outward") {
  const auto block = CollisionShape::from_obj(VOLCONTACT_DATA_DIR "/bodies/box_1000x500x500/surface.obj");
  CHECK(signed_volume(block->mesh()) == doctest::Approx(0.25));
  const auto cylinder = CollisionShape::from_obj(VOLCONTACT_DATA_DIR "/bodies/cylinder_r250_h1000_seg64/surface.obj");
  const double polygon_area = 32.0 * 0.25 * 0.25 * std::sin(2.0 * std::numbers::pi / 64.0);
  CHECK(signed_volume(cylinder->mesh()) == doctest::Approx(polygon_area * 1.0).epsilon(1e-6));
}

TEST_CASE("BSP point containment") {
  CHECK(kUnitCube.bsp().contains(Vec3(0.1, -0.2, 0.3)));
  CHECK_FALSE(kUnitCube.bsp().contains(Vec3(0.6, 0.0, 0.0)));
  CHECK_FALSE(kUnitCube.bsp().contains(Vec3(0.0, 0.0, -0.51)));
}

TEST_CASE("cube sinking into the ground") {
  const double depth = 0.1;
  const Pose X_WA{Mat3::Identity(), Vec3(0, 0, 0.5 - depth)};
  bool hit = false;
  const IntersectionProperties p = overlap(kUnitCube, X_WA, kGround, Pose{}, &hit);
  REQUIRE(hit);
  CHECK(p.volume == doctest::Approx(depth));
  CHECK(p.centroid.isApprox(Vec3(0, 0, -depth / 2)));
  CHECK(p.normal.isApprox(Vec3::UnitZ()));
  const Vec3 expected = depth / 12.0 * Vec3(1 + depth * depth, 1 + depth * depth, 2);
  CHECK(p.inertia.diagonal().isApprox(expected, 1e-9));
}

TEST_CASE("intersection is invariant to rigid motion of the pair") {
  const double depth = 0.05;
  const Pose X_WA{rot(0.7, Vec3::UnitZ()), Vec3(1.0, -2.0, 0.5 - depth)};
  bool hit = false;
  const IntersectionProperties p0 = overlap(kUnitCube, X_WA, kGround, Pose{}, &hit);
  REQUIRE(hit);
  const Pose X{rot(1.1, Vec3(1, 2, 3)), Vec3(3, -1, 2)};
  const IntersectionProperties p1 = overlap(kUnitCube, X * X_WA, kGround, X, &hit);
  REQUIRE(hit);
  CHECK(p1.volume == doctest::Approx(p0.volume).epsilon(1e-12));
  CHECK(p1.centroid.isApprox(X.apply(p0.centroid), 1e-12));
  CHECK(p1.normal.isApprox(X.R * p0.normal, 1e-12));
  CHECK(p1.inertia.isApprox(X.R * p0.inertia * X.R.transpose(), 1e-9));
}

TEST_CASE("tilted cube: wedge volume") {
  // Cube rotated about y by θ with its lowest edge 0.1 below the ground: a
  // triangular prism of cross-section ½·b·h.
  const double theta = 0.2, depth = 0.1;
  const Mat3 R = rot(theta, Vec3::UnitY());
  const double lowest = 0.5 * (std::cos(theta) + std::sin(theta));
  const Pose X_WA{R, Vec3(0, 0, lowest - depth)};
  bool hit = false;
  const IntersectionProperties p = overlap(kUnitCube, X_WA, kGround, Pose{}, &hit);
  REQUIRE(hit);
  // Assumes the wedge stays within the faces adjacent to the edge.
  const double area = 0.5 * (depth / std::sin(theta)) * (depth / std::cos(theta));
  CHECK(p.volume == doctest::Approx(area));
  CHECK(p.normal.isApprox(Vec3::UnitZ(), 1e-12));
}

TEST_CASE("touching faces give no volume") {
  bool hit = true;
  overlap(kUnitCube, Pose{Mat3::Identity(), Vec3(0, 0, 0.5)}, kGround, Pose{}, &hit);
  CHECK_FALSE(hit);
}

TEST_CASE("coplanar faces with the same orientation are counted once") {
  // B shares the cube's x faces; the overlap is a slab of the cube.
  const CollisionShape b(make_box(Vec3(-0.5, -3, -3), Vec3(0.5, 3, -0.4)));
  bool hit = false;
  const IntersectionProperties p = overlap(kUnitCube, Pose{}, b, Pose{}, &hit);
  REQUIRE(hit);
  CHECK(p.volume == doctest::Approx(0.1));
  CHECK(p.centroid.isApprox(Vec3(0, 0, -0.45)));

  const IntersectionProperties same = overlap(kUnitCube, Pose{}, kUnitCube, Pose{}, &hit);
  REQUIRE(hit);
  CHECK(same.volume == doctest::Approx(1.0));
}

TEST_CASE("separated bodies do not intersect") {
  bool hit = true;
  overlap(kUnitCube, Pose{Mat3::Identity(), Vec3(0, 0, 3)}, kGround, Pose{}, &hit);
  CHECK_FALSE(hit);
}

TEST_CASE("non-convex solid with opposite-facing faces on one plane") {
  const CollisionShape step(make_step());
  REQUIRE(signed_volume(step.mesh()) == doctest::Approx(4.0));
  // B's top face lies on z = 1, where the step has an up-facing face (shared,
  // counted once), interior (kept) and a down-facing face (touching, dropped).
  const CollisionShape slab(make_box(Vec3(-1, -1, 0.5), Vec3(4, 2, 1)));
  bool hit = false;
  const IntersectionProperties p = overlap(step, Pose{}, slab, Pose{}, &hit);
  REQUIRE(hit);
  CHECK(p.volume == doctest::Approx(1.0));
  CHECK(p.centroid.isApprox(Vec3(1.0, 0.5, 0.75)));
}

namespace {

struct BowlFixture {
  std::shared_ptr<const CollisionShape> sphere;
  std::vector<std::shared_ptr<const CollisionShape>> pieces;
  BowlFixture() {
    const std::filesystem::path data = VOLCONTACT_DATA_DIR;
    sphere = std::make_shared<const CollisionShape>(load_obj(data / "bodies/sphere_r100_seg64/surface.obj"));
    std::vector<std::filesystem::path> files;
    for (const auto& e : std::filesystem::directory_iterator(data / "grounds/bowl_r300_t20_a60_seg128/pieces")) {
      if (e.path().extension() == ".obj") files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
    for (const auto& f : files) pieces.push_back(std::make_shared<const CollisionShape>(load_obj(f)));
  }
  ContactModel model(const ContactLaw& law, std::size_t first, std::size_t count) const {
    ContactModel m;
    for (std::size_t i = first; i < first + count; ++i) {
      m.add_pair({.body_a = 0, .body_b = 1, .shape_a = sphere, .shape_b = pieces[i], .law = law});
    }
    return m;
  }
};

// Evaluates every pair on its own (never reaching the tree) and compares with
// the model that holds all of them. Returns how many pairs touched.
int compare_with_brute_force(const BowlFixture& f, const ContactLaw& law, const std::vector<Pose>& X,
                             const std::vector<Twist>& V) {
  const ContactModel all = f.model(law, 0, f.pieces.size());
  std::vector<ContactModel> singles;
  for (std::size_t i = 0; i < f.pieces.size(); ++i) singles.push_back(f.model(law, i, 1));

  std::vector<FrictionState> friction = all.initial_friction();
  ContactModel::Evaluation eval, one;
  int touching = 0;
  for (int step = 0; step < 2; ++step) {  // The second step starts from the friction state of the first.
    all.evaluate(X, V, friction, &eval);
    Wrench expected[2];
    for (std::size_t i = 0; i < singles.size(); ++i) {
      singles[i].evaluate(X, V, std::span<const FrictionState>(&friction[i], 1), &one);
      const auto& got = eval.pairs[i];
      REQUIRE(got.in_contact == one.pairs[0].in_contact);
      CHECK(eval.next_friction[i].anchor_B == one.next_friction[0].anchor_B);
      if (!got.in_contact) continue;
      if (step == 0) ++touching;
      CHECK(got.force.force == one.pairs[0].force.force);
      CHECK(got.force.torque == one.pairs[0].force.torque);
      for (int b = 0; b < 2; ++b) {
        expected[b].force += one.wrenches[b].force;
        expected[b].torque += one.wrenches[b].torque;
      }
    }
    for (int b = 0; b < 2; ++b) {
      CHECK(eval.wrenches[b].force.isApprox(expected[b].force, 1e-12));
      CHECK(eval.wrenches[b].torque.isApprox(expected[b].torque, 1e-12));
    }
    friction = eval.next_friction;
  }
  return touching;
}

}  // namespace

TEST_CASE("broadphase gives the brute-force result for a sphere in a bowl of convex pieces") {
  const BowlFixture f;
  REQUIRE(f.pieces.size() > 1000);
  GonthierParams gonthier;
  gonthier.friction = BristleFriction{};
  const ContactLaw laws[] = {gonthier, DistributedParams{}};

  std::mt19937 rng(7);
  std::uniform_real_distribution<double> offset(-0.01, 0.01), depth(0.0001, 0.004), angle(-1.0, 1.0);
  int touching = 0;
  for (const ContactLaw& law : laws) {
    for (int k = 0; k < 4; ++k) {
      const Vec3 axis = Vec3(angle(rng), angle(rng), angle(rng) + 2.0).normalized();
      std::vector<Pose> X(2);
      X[0].R = Eigen::AngleAxisd(angle(rng), axis).toRotationMatrix();
      X[0].p = Vec3(offset(rng), offset(rng), 0.1 - depth(rng));
      std::vector<Twist> V(2);
      V[0].v = Vec3(0.05, -0.02, -0.01);
      V[0].w = Vec3(0.3, 0.1, -0.2);
      touching += compare_with_brute_force(f, law, X, V);
    }
  }
  CHECK(touching > 0);
}

TEST_CASE("broadphase: a sphere far from the bowl touches nothing") {
  const BowlFixture f;
  const ContactModel model = f.model(GonthierParams{}, 0, f.pieces.size());
  std::vector<Pose> X(2);
  X[0].p = Vec3(0, 0, 0.5);
  const std::vector<Twist> V(2);
  ContactModel::Evaluation eval;
  // Stale state from a previous contact must be cleared.
  std::vector<FrictionState> friction = model.initial_friction();
  friction[0].anchor_B = Vec3(1, 2, 3);
  model.evaluate(X, V, friction, &eval);
  for (std::size_t i = 0; i < f.pieces.size(); ++i) {
    CHECK_FALSE(eval.pairs[i].in_contact);
    CHECK(eval.next_friction[i].anchor_B == Vec3::Zero());
  }
  CHECK(eval.wrenches[0].force == Vec3::Zero());
  CHECK(eval.wrenches[1].torque == Vec3::Zero());
}

TEST_CASE("evaluate without pairs returns one zero wrench per body") {
  const ContactModel model;
  const std::vector<Pose> X(3);
  const std::vector<Twist> V(3);
  ContactModel::Evaluation eval;
  model.evaluate(X, V, {}, &eval);
  REQUIRE(eval.wrenches.size() == 3);
  for (const Wrench& w : eval.wrenches) {
    CHECK(w.force == Vec3::Zero());
    CHECK(w.torque == Vec3::Zero());
  }
}

TEST_CASE("parallel evaluation of many pairs equals serial") {
  const BowlFixture f;
  const ContactModel serial = [&] {
    ContactModel m = f.model(GonthierParams{}, 0, f.pieces.size());
    m.set_parallel_threshold(1 << 30);
    return m;
  }();
  ContactModel parallel = f.model(GonthierParams{}, 0, f.pieces.size());
  parallel.set_parallel_threshold(1);
  std::vector<Pose> X(2);
  X[0].p = Vec3(0.0, 0.0, 0.1 - 0.004);
  const std::vector<Twist> V(2);
  const std::vector<FrictionState> friction = serial.initial_friction();
  ContactModel::Evaluation a, b;
  serial.evaluate(X, V, friction, &a);
  parallel.evaluate(X, V, friction, &b);
  CHECK(a.wrenches[0].force == b.wrenches[0].force);
  CHECK(a.wrenches[0].torque == b.wrenches[0].torque);
  for (std::size_t i = 0; i < f.pieces.size(); ++i) {
    CHECK(a.pairs[i].in_contact == b.pairs[i].in_contact);
    CHECK(a.pairs[i].force.force == b.pairs[i].force.force);
  }
}
