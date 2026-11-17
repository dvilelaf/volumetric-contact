/* Compiled as C (see tests/CMakeLists.txt): a box that slides on a slab until it stops, driven
 * through the C API only. capi_test.cpp runs the same scene with the C++ API and compares. */
#include <string.h>

#include "volcontact/c_api.h"

/* Closed box mesh, 8 vertices and 12 triangles, counter-clockwise outward. */
static void make_box(const double half[3], const double center[3], double vertices[24], uint32_t triangles[36]) {
  static const uint32_t t[36] = {1, 3, 7, 1, 7, 5, 0, 4, 6, 0, 6, 2, 2, 6, 7, 2, 7, 3,
                                 0, 1, 5, 0, 5, 4, 4, 5, 7, 4, 7, 6, 0, 2, 3, 0, 3, 1};
  for (int i = 0; i < 8; ++i) {
    vertices[3 * i] = center[0] + ((i & 1) ? half[0] : -half[0]);
    vertices[3 * i + 1] = center[1] + ((i & 2) ? half[1] : -half[1]);
    vertices[3 * i + 2] = center[2] + ((i & 4) ? half[2] : -half[2]);
  }
  memcpy(triangles, t, sizeof t);
}

/* law 0: Gonthier, 1: distributed. coupled: declare the explicit coupling of the step (the box is
 * a cube of mass 1, the slab fixed). stiffness > 0 replaces the distributed law's K. out = final
 * position (3), velocity (3), pair in contact (1). Returns the status of the first call that
 * failed, VOLCONTACT_OK otherwise. */
int capi_drop(int law, int coupled, double stiffness, double out[7]) {
  const double side = 0.2, mass = 1.0, g = 9.81, dt = 1e-3;
  double bv[24], sv[24];
  uint32_t bt[36], st[36];
  const double box_half[3] = {side / 2, side / 2, side / 2}, zero[3] = {0, 0, 0};
  const double slab_half[3] = {5, 5, 0.5}, slab_center[3] = {0, 0, -0.5};
  make_box(box_half, zero, bv, bt);
  make_box(slab_half, slab_center, sv, st);
  const volcontact_mesh box_mesh = {bv, 8, bt, 12}, slab_mesh = {sv, 8, st, 12};

  volcontact_shape *box = NULL, *slab = NULL;
  volcontact_model* model = NULL;
  int status;
  if ((status = volcontact_shape_create(&box_mesh, &box)) != VOLCONTACT_OK) return status;
  if ((status = volcontact_shape_create(&slab_mesh, &slab)) != VOLCONTACT_OK) return status;
  if ((status = volcontact_model_create(&model)) != VOLCONTACT_OK) return status;
  if (law == 0) {
    const volcontact_gonthier_params p = volcontact_gonthier_params_default();
    status = volcontact_model_add_pair_gonthier(model, 0, 1, box, slab, &p, NULL);
  } else {
    volcontact_distributed_params p = volcontact_distributed_params_default();
    if (stiffness > 0.0) p.stiffness = stiffness;
    status = volcontact_model_add_pair_distributed(model, 0, 1, box, slab, &p, NULL);
  }
  if (status != VOLCONTACT_OK) return status;
  volcontact_shape_destroy(box); /* The model keeps its own references. */
  volcontact_shape_destroy(slab);
  if (coupled) {
    const double inv_masses[2] = {1.0 / mass, 0.0};
    const double inertia = mass * side * side / 6.0; /* Cube. */
    const double inv_inertias[18] = {1.0 / inertia, 0, 0, 0, 1.0 / inertia, 0, 0, 0, 1.0 / inertia,
                                     0,             0, 0, 0, 0,             0, 0, 0, 0};
    if ((status = volcontact_model_set_explicit_coupling(model, dt, 2, inv_masses, inv_inertias)) != VOLCONTACT_OK)
      return status;
  }

  double poses[2 * VOLCONTACT_POSE_SIZE] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0.5 * side + 0.005,
                                    1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  double twists[2 * VOLCONTACT_TWIST_SIZE] = {0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0, 0, 0};
  double friction[2][VOLCONTACT_FRICTION_STATE_SIZE]; /* [step parity]: evaluate must not alias in and out. */
  double wrenches[2 * VOLCONTACT_WRENCH_SIZE];
  int in_contact = 0, cur = 0;
  volcontact_friction_state_init(friction[0]);

  for (int step = 0; step < 1500; ++step) {
    status = volcontact_model_evaluate(model, 2, poses, twists, friction[cur], wrenches, friction[1 - cur], &in_contact);
    if (status != VOLCONTACT_OK) break;
    cur = 1 - cur;
    for (int k = 0; k < 3; ++k) {  /* Translation only: the box stays level. */
      twists[3 + k] += dt * (wrenches[3 + k] / mass + (k == 2 ? -g : 0.0));
      poses[9 + k] += dt * twists[3 + k];
    }
  }
  for (int k = 0; k < 3; ++k) {
    out[k] = poses[9 + k];
    out[3 + k] = twists[3 + k];
  }
  out[6] = in_contact;
  volcontact_model_destroy(model);
  return status;
}
