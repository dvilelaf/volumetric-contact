/* Plain C interface of the volumetric contact core, for engines and language bindings that cannot
 * use the C++ API. Opaque handles, plain arrays, status codes instead of exceptions.
 *
 * Workflow (the same as the C++ ContactModel, see docs/integration.md):
 *   1. once: volcontact_shape_create (or volcontact_compound_create) for each body's geometry, volcontact_model_create,
 *      volcontact_model_add_pair_* for each pair of shapes that can touch;
 *   2. every step: read the poses and twists of the bodies, volcontact_model_evaluate with the friction
 *      state of the last step, apply the wrenches, keep the new friction state.
 *
 * Conventions
 *   - Pose of a body, 12 doubles: the rotation matrix R in row-major order (9), then the position p
 *     of the body origin (3), both of the body frame in world: x_W = R·x_B + p.
 *   - Twist, 6 doubles: angular velocity w, then the velocity v of the body origin, in world.
 *   - Wrench, 6 doubles: torque, then force, acting on the body about its origin, in world.
 *   - Bodies are numbered 0..N-1 by the caller; pairs are numbered in the order they were added.
 *   - A pair (A, B) yields the force on A; B gets the opposite. Shapes are in their body's frame.
 *   - Friction state: VOLCONTACT_FRICTION_STATE_SIZE doubles per pair, opaque to the caller. Start every
 *     pair from volcontact_friction_state_init and pass what the last evaluate returned.
 *
 * Thread safety: distinct handles are independent. One model handle must not be used by two threads
 * at once (it owns its scratch memory); shape handles may be shared by any number of models.
 * Every function returns a volcontact_status; on failure volcontact_last_error() describes it (per thread).
 */
#ifndef VOLCONTACT_C_API_H
#define VOLCONTACT_C_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
#define VOLCONTACT_API __attribute__((visibility("default")))
#else
#define VOLCONTACT_API
#endif

/* Version of the C interface, bumped when it changes incompatibly. */
#define VOLCONTACT_C_API_VERSION 1

#define VOLCONTACT_POSE_SIZE 12
#define VOLCONTACT_TWIST_SIZE 6
#define VOLCONTACT_WRENCH_SIZE 6
#define VOLCONTACT_FRICTION_STATE_SIZE 26

typedef enum volcontact_status {
  VOLCONTACT_OK = 0,
  VOLCONTACT_INVALID_ARGUMENT = 1, /* Null pointer, bad index, size mismatch, unusable mesh or pair. */
  VOLCONTACT_ERROR = 2             /* Any other failure; see volcontact_last_error(). */
} volcontact_status;

typedef struct volcontact_shape volcontact_shape; /* Immutable collision geometry; reference counted by the models. */
typedef struct volcontact_model volcontact_model; /* A set of pairs, evaluated together. */

/* Message of the last failure on this thread; valid until the next call on this thread. */
VOLCONTACT_API const char* volcontact_last_error(void);

/* ---- Geometry ---------------------------------------------------------------------------------- */

/* A closed triangle mesh in the body frame: `vertices` holds 3·num_vertices doubles (x, y, z),
 * `triangles` 3·num_triangles indices. Counter-clockwise outward faces are expected; an inside-out
 * mesh is flipped. The arrays are copied. */
typedef struct volcontact_mesh {
  const double* vertices;
  size_t num_vertices;
  const uint32_t* triangles;
  size_t num_triangles;
} volcontact_mesh;

/* One shape from one mesh. Convex or not; the distributed law needs convex shapes. */
VOLCONTACT_API volcontact_status volcontact_shape_create(const volcontact_mesh* mesh, volcontact_shape** out);

/* The shapes of a rigid body given as `num_pieces` convex pieces that add up to it without
 * overlapping (all in the body frame): finds the seams between pieces, so the distributed law puts
 * no pressure there. Writes one handle per piece in `out_pieces` (an array of num_pieces pointers).
 * `tolerance` 0 picks the default. Use one pair per piece in the model. */
VOLCONTACT_API volcontact_status volcontact_compound_create(const volcontact_mesh* pieces, size_t num_pieces, double tolerance, volcontact_shape** out_pieces);

/* Releases the caller's reference; models that use the shape keep theirs. Null is allowed. */
VOLCONTACT_API void volcontact_shape_destroy(volcontact_shape* shape);

/* ---- Force laws -------------------------------------------------------------------------------- */

/* Parameters are filled with the library defaults by volcontact_*_params_default; change what you need. */
typedef struct volcontact_gonthier_params {
  double stiffness;     /* K [N/m³]. */
  double damping;       /* h [s/m]. */
  double mu_spin;       /* Only for friction_model 1. */
  int rolling_resistance;
  int allow_adhesion;
  int friction_model;   /* 0: published (Gonthier 2007), 1: Vilela 2018 thesis. */
  int has_friction;     /* 0: no tangential force at all; the fields below are then unused. */
  double mu_dynamic, mu_static, mu_viscous;
  double v_stick, k_stick, c_stick, anchor_update;
  int limit_includes_damping;
  double preslide_travel, preslide_time;
  int friction_at_body_point; /* 0: at the centroid of the overlap; 1: at friction_point_a. */
  double friction_point_a[3]; /* In A's frame. */
} volcontact_gonthier_params;

typedef struct volcontact_distributed_params {
  double stiffness;   /* K [N/m³]. */
  double damping;     /* h [s/m]. */
  double mu_dynamic, mu_static;
  int allow_adhesion;
  int has_stick;      /* 0: kinetic friction only; the fields below are then unused. */
  double v_stick, k_stick, c_stick, travel, time;
} volcontact_distributed_params;

VOLCONTACT_API volcontact_gonthier_params volcontact_gonthier_params_default(void);
VOLCONTACT_API volcontact_distributed_params volcontact_distributed_params_default(void);

/* ---- Model ------------------------------------------------------------------------------------- */

VOLCONTACT_API volcontact_status volcontact_model_create(volcontact_model** out);
VOLCONTACT_API void volcontact_model_destroy(volcontact_model* model);

/* Adds the pair (shape_a on body_a, shape_b on body_b) with its law. Shapes are shared, not copied.
 * Returns the pair's index in `out_index` when it is not null. */
VOLCONTACT_API volcontact_status volcontact_model_add_pair_gonthier(volcontact_model* model, int body_a, int body_b, const volcontact_shape* shape_a,
                                            const volcontact_shape* shape_b, const volcontact_gonthier_params* params,
                                            size_t* out_index);
VOLCONTACT_API volcontact_status volcontact_model_add_pair_distributed(volcontact_model* model, int body_a, int body_b, const volcontact_shape* shape_a,
                                               const volcontact_shape* shape_b, const volcontact_distributed_params* params,
                                               size_t* out_index);

VOLCONTACT_API size_t volcontact_model_num_pairs(const volcontact_model* model);
VOLCONTACT_API int volcontact_model_num_bodies(const volcontact_model* model); /* One more than the largest body index. */

/* Optional: builds the broadphase now instead of at the first evaluate. */
VOLCONTACT_API volcontact_status volcontact_model_finalize(volcontact_model* model);

/* Optional, mirrors ContactModel::set_explicit_coupling: tells the model that its forces are integrated
 * explicitly with the step `dt` [s] (the wrench of the state at the start of the step, applied for
 * the whole step), on bodies numbered like the poses. The laws then bound their stick spring and
 * damper and their normal damper to what that step can integrate; below those bounds nothing
 * changes. Call it once after the pairs are added, with `dt` the step you use.
 *   inv_masses     num_bodies doubles: 1/m of each body (0 for a body that does not move)
 *   inv_inertias   9 · num_bodies doubles: the inverse inertia tensor of each body, row-major, in the
 *                  body frame about its origin (which must be the centre of mass); zeros if fixed
 * dt = 0 switches the bounds off (the default); dt < 0 or a negative mass is an error. */
VOLCONTACT_API volcontact_status volcontact_model_set_explicit_coupling(volcontact_model* model, double dt, size_t num_bodies,
                                                                       const double* inv_masses,
                                                                       const double* inv_inertias);

/* Writes the initial friction state of one pair (VOLCONTACT_FRICTION_STATE_SIZE doubles). */
VOLCONTACT_API void volcontact_friction_state_init(double* state);

/* Evaluates every pair.
 *   poses            VOLCONTACT_POSE_SIZE · num_bodies doubles (num_bodies >= volcontact_model_num_bodies)
 *   twists           VOLCONTACT_TWIST_SIZE · num_bodies doubles
 *   friction         VOLCONTACT_FRICTION_STATE_SIZE · volcontact_model_num_pairs doubles, from the last step
 *   wrenches_out     VOLCONTACT_WRENCH_SIZE · num_bodies doubles: the total wrench on each body, about its
 *                    origin, in world (zero for bodies without contact)
 *   next_friction_out  same layout as `friction`; must not overlap it
 *   in_contact_out   optional, one int per pair: 1 if the pair is in contact
 * The wrench of the step belongs to the poses and twists given: apply it to the same step. */
VOLCONTACT_API volcontact_status volcontact_model_evaluate(volcontact_model* model, size_t num_bodies, const double* poses, const double* twists,
                                   const double* friction, double* wrenches_out, double* next_friction_out,
                                   int* in_contact_out);

#ifdef __cplusplus
}
#endif

#endif /* VOLCONTACT_C_API_H */
