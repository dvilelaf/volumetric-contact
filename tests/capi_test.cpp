// The C API drives a box on a slab and gets the C++ result (tests/capi_drive.c is compiled as C).
// Needs only the core: it is built even when the benchmark is not.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include "volcontact/c_api.h"
#include "volcontact/contact_model.hpp"

extern "C" int capi_drop(int law, int coupled, double stiffness, double out[7]);

namespace {

std::shared_ptr<const volcontact::CollisionShape> box(const volcontact::Vec3& half, const volcontact::Vec3& center) {
  volcontact::TriMesh mesh;
  for (int i = 0; i < 8; ++i) {
    const volcontact::Vec3 s((i & 1) ? 1 : -1, (i & 2) ? 1 : -1, (i & 4) ? 1 : -1);
    mesh.vertices.push_back(center + half.cwiseProduct(s));
  }
  mesh.triangles = {{1, 3, 7}, {1, 7, 5}, {0, 4, 6}, {0, 6, 2}, {2, 6, 7}, {2, 7, 3},
                    {0, 1, 5}, {0, 5, 4}, {4, 5, 7}, {4, 7, 6}, {0, 2, 3}, {0, 3, 1}};
  return std::make_shared<const volcontact::CollisionShape>(std::move(mesh));
}

// The scene of capi_drive.c through the C++ API.
void drop(const volcontact::ContactLaw& law, double out[7], bool coupled = false) {
  const double side = 0.2, mass = 1.0, g = 9.81, dt = 1e-3;
  volcontact::ContactModel model;
  model.add_pair({.body_a = 0, .body_b = 1, .shape_a = box(volcontact::Vec3::Constant(side / 2), volcontact::Vec3::Zero()),
                  .shape_b = box({5, 5, 0.5}, {0, 0, -0.5}), .law = law});
  if (coupled) {
    const double inertia = mass * side * side / 6.0;
    model.set_explicit_coupling(dt, {{1.0 / mass, volcontact::Mat3::Identity() / inertia}, {}});
  }
  std::vector<volcontact::Pose> poses(2);
  std::vector<volcontact::Twist> twists(2);
  poses[0].p = {0, 0, 0.5 * side + 0.005};
  twists[0].v = {0.3, 0, 0};
  auto friction = model.initial_friction();
  volcontact::ContactModel::Evaluation eval;
  bool contact = false;
  for (int step = 0; step < 1500; ++step) {
    model.evaluate(poses, twists, friction, &eval);
    friction.swap(eval.next_friction);
    contact = eval.pairs[0].in_contact;
    twists[0].v += dt * (eval.wrenches[0].force / mass + volcontact::Vec3(0, 0, -g));
    poses[0].p += dt * twists[0].v;
  }
  for (int k = 0; k < 3; ++k) {
    out[k] = poses[0].p[k];
    out[3 + k] = twists[0].v[k];
  }
  out[6] = contact;
}

void expect_same(const double* c, const double* cpp) {
  for (int k = 0; k < 7; ++k) CHECK(c[k] == doctest::Approx(cpp[k]).epsilon(1e-9));
}

}  // namespace

TEST_CASE("C API: a sliding box on a slab, distributed law, matches the C++ API") {
  double c[7], cpp[7];
  REQUIRE(capi_drop(1, 0, 0.0, c) == VOLCONTACT_OK);
  drop(volcontact::DistributedParams{}, cpp);
  expect_same(c, cpp);
  CHECK(c[6] == 1.0);                // Still in contact,
  CHECK(std::abs(c[3]) < 1e-3);      // and it has stopped by friction.
}

TEST_CASE("C API: a sliding box on a slab, Gonthier law, matches the C++ API") {
  double c[7], cpp[7];
  REQUIRE(capi_drop(0, 0, 0.0, c) == VOLCONTACT_OK);
  volcontact::GonthierParams law;
  law.friction = volcontact::BristleFriction{};
  drop(law, cpp);
  expect_same(c, cpp);
}

TEST_CASE("C API: explicit coupling bounds a stiff contact like the C++ API") {
  volcontact::DistributedParams law;
  law.stiffness = 1e7;  // A stiff foundation for the step: K·A·dt²/m = 0.4.
  double c[7], cpp[7];
  REQUIRE(capi_drop(1, 1, law.stiffness, c) == VOLCONTACT_OK);
  drop(law, cpp, true);
  expect_same(c, cpp);
  CHECK(std::abs(c[2] - 0.1) < 1e-3);  // Rests on the slab.
}

TEST_CASE("C API: set_explicit_coupling validates its arguments") {
  volcontact_model* model = nullptr;
  REQUIRE(volcontact_model_create(&model) == VOLCONTACT_OK);
  const double m[1] = {1.0}, bad_m[1] = {-1.0}, inertia[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  CHECK(volcontact_model_set_explicit_coupling(nullptr, 1e-3, 1, m, inertia) == VOLCONTACT_INVALID_ARGUMENT);
  CHECK(volcontact_model_set_explicit_coupling(model, -1.0, 1, m, inertia) == VOLCONTACT_INVALID_ARGUMENT);
  CHECK(volcontact_model_set_explicit_coupling(model, 1e-3, 1, bad_m, inertia) == VOLCONTACT_INVALID_ARGUMENT);
  CHECK(volcontact_model_set_explicit_coupling(model, 1e-3, 1, nullptr, inertia) == VOLCONTACT_INVALID_ARGUMENT);
  CHECK(volcontact_model_set_explicit_coupling(model, 1e-3, 1, m, inertia) == VOLCONTACT_OK);
  CHECK(volcontact_model_set_explicit_coupling(model, 0.0, 0, nullptr, nullptr) == VOLCONTACT_OK);
  volcontact_model_destroy(model);
}

TEST_CASE("C API: errors are status codes with a message") {
  volcontact_shape* shape = nullptr;
  CHECK(volcontact_shape_create(nullptr, &shape) == VOLCONTACT_INVALID_ARGUMENT);
  CHECK(shape == nullptr);
  const double v[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
  const uint32_t t[3] = {0, 1, 5};  // Index out of range.
  const volcontact_mesh bad = {v, 3, t, 1};
  CHECK(volcontact_shape_create(&bad, &shape) == VOLCONTACT_INVALID_ARGUMENT);
  CHECK(std::strlen(volcontact_last_error()) > 0);
  volcontact_model* model = nullptr;
  REQUIRE(volcontact_model_create(&model) == VOLCONTACT_OK);
  CHECK(volcontact_model_add_pair_distributed(model, 0, 0, nullptr, nullptr, nullptr, nullptr) == VOLCONTACT_INVALID_ARGUMENT);
  CHECK(volcontact_model_num_pairs(model) == 0);
  volcontact_model_destroy(model);
}
