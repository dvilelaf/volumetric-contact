// A box dropped on a slab, with no physics engine: the contact core plus a few lines of explicit
// integration. It shows what an engine integration needs:
//   1. once:      build the shapes and the ContactModel (pairs, force laws), and tell it the step and
//                 the inertias of your bodies (set_explicit_coupling) when you integrate explicitly;
//   2. each step: model.evaluate(poses, twists, friction) -> one wrench per body;
//   3. then:      apply the wrenches in your integrator and keep next_friction for the next step.
// Run it as `minimal` (boxes built in code) or `minimal box.obj slab.obj` (closed OBJ meshes in the
// body frame; the slab's top must be the plane z = 0 and the box's origin its centre of mass).
#include <cmath>
#include <cstdio>
#include <memory>

#include "volcontact/contact_model.hpp"

using namespace volcontact;

// Closed, counter-clockwise box mesh of the given half sizes around `center` (body frame).
static std::shared_ptr<const CollisionShape> box(const Vec3& half, const Vec3& center) {
  TriMesh mesh;
  for (int i = 0; i < 8; ++i) {
    const Vec3 s((i & 1) ? 1 : -1, (i & 2) ? 1 : -1, (i & 4) ? 1 : -1);
    mesh.vertices.push_back(center + half.cwiseProduct(s));
  }
  mesh.triangles = {{1, 3, 7}, {1, 7, 5}, {0, 4, 6}, {0, 6, 2}, {2, 6, 7}, {2, 7, 3},
                    {0, 1, 5}, {0, 5, 4}, {4, 5, 7}, {4, 7, 6}, {0, 2, 3}, {0, 3, 1}};
  return std::make_shared<const CollisionShape>(std::move(mesh));
}

int main(int argc, char** argv) {
  // 1. Shapes (in each body's frame) and the model. Body 0 is the box, body 1 the fixed slab.
  const double side = 0.2, mass = 1.0, g = 9.81;
  std::shared_ptr<const CollisionShape> box_shape, slab_shape;
  if (argc == 3) {
    box_shape = CollisionShape::from_obj(argv[1]);
    slab_shape = CollisionShape::from_obj(argv[2]);
  } else {
    box_shape = box(Vec3::Constant(side / 2), Vec3::Zero());
    slab_shape = box({5, 5, 0.5}, {0, 0, -0.5});
  }
  DistributedParams law;  // Winkler foundation K = 1e6 N/m³, damping, Coulomb friction with stick.
  ContactModel model;
  model.add_pair({.body_a = 0, .body_b = 1, .shape_a = box_shape, .shape_b = slab_shape, .law = law});
  // The integrator below is explicit: the wrench of the state at the start of the step is applied for
  // the whole step. Say so, with the step and the inverse inertias (body 0: the box; body 1: the
  // fixed slab, all zero), and the laws bound their springs and dampers to what that step can
  // integrate (nothing changes below the bounds).
  const double dt = 1e-3, inertia = mass * side * side / 6.0;
  model.set_explicit_coupling(dt, {{1.0 / mass, Mat3::Identity() / inertia}, {}});

  // State of the integrator, kept by the caller: this is all the contact needs to know.
  std::vector<Pose> poses(2);                 // The slab stays at the identity.
  std::vector<Twist> twists(2);               // ... and at rest.
  poses[0].p = {0, 0, 0.5 * side + 0.02};     // Dropped from 2 cm above the slab.
  std::vector<FrictionState> friction = model.initial_friction();
  ContactModel::Evaluation eval;              // Reuse it: its buffers keep their capacity.

  for (int step = 0; step < 3000; ++step) {
    // 2. Contact wrenches at the current state: about each body's origin, in world.
    model.evaluate(poses, twists, friction, &eval);
    friction.swap(eval.next_friction);  // 3b. Keep the friction state (it moves with body B).

    // 3a. Apply the wrench of body 0 (any integrator does; semi-implicit Euler here).
    const Wrench& w = eval.wrenches[0];
    Twist& v = twists[0];
    v.v += dt * (w.force / mass + Vec3(0, 0, -g));
    v.w += dt * w.torque / inertia;
    poses[0].p += dt * v.v;
    const double angle = dt * v.w.norm();
    if (angle > 0) poses[0].R = Eigen::AngleAxisd(angle, v.w.normalized()).toRotationMatrix() * poses[0].R;
  }

  // At rest the foundation carries the weight: K·A·δ = m·g.
  const double expected = mass * g / (law.stiffness * side * side);
  const double sunk = 0.5 * side - poses[0].p.z();
  std::printf("penetration %.4f mm, theory m·g/(K·A) = %.4f mm, speed %.2e m/s\n", 1e3 * sunk, 1e3 * expected,
              twists[0].v.norm());
  const bool ok = argc == 3 || (std::abs(sunk - expected) < 0.05 * expected && twists[0].v.norm() < 1e-3);
  std::puts(ok ? "ok" : "FAILED");
  return ok ? 0 : 1;
}
