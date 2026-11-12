# Using the contact core in your engine

The core (`core/`, target `volcontact::core`) is a library with one job: given the pose and twist of
every body, return the contact wrench on every body. It depends on Eigen (and OpenMP if enabled) and
knows nothing about Drake, Chrono, MuJoCo or the benchmark.

## Get it

```cmake
# Installed:  cmake -S volumetric-contact -B b -DVOLCONTACT_BUILD_BENCHMARK=OFF -DVOLCONTACT_BUILD_TESTS=OFF
#             cmake --build b && cmake --install b --prefix /opt/volcontact
find_package(volcontact CONFIG REQUIRED)       # -DCMAKE_PREFIX_PATH=/opt/volcontact
target_link_libraries(my_engine PRIVATE volcontact::core)

# Or from source (no benchmark, tests or engines are built when consumed this way):
include(FetchContent)
FetchContent_Declare(volcontact GIT_REPOSITORY <url> GIT_TAG <tag>)
FetchContent_MakeAvailable(volcontact)
```

`examples/minimal/` is a complete program (about 100 lines) that drops a box on a slab with a
five-line integrator and checks the resting penetration against the theory `m·g/(K·A)`;
`scripts/check-install.sh` builds and runs it against an installed package (static and shared) and
through FetchContent.

Options: `VOLCONTACT_NATIVE` (default OFF: `-march=native` makes binaries that only run on CPUs like
the build host's; turn it ON for a build that runs where it is compiled, about 8 % faster),
`VOLCONTACT_OPENMP` (parallel pairs), `BUILD_SHARED_LIBS`, `VOLCONTACT_PIC`.

## The three calls

```cpp
// 1. Once. Shapes are in each body's frame; a Pair is (shape on body A, shape on body B, law).
volcontact::ContactModel model;
model.add_pair({.body_a = 0, .body_b = 1, .shape_a = box, .shape_b = slab, .law = volcontact::DistributedParams{}});
model.set_explicit_coupling(dt, {{1.0 / m, I.inverse()}, {}});  // explicit integration: step and inverse inertias
auto friction = model.initial_friction();          // one FrictionState per pair
volcontact::ContactModel::Evaluation eval;         // reuse: no allocation after the first call

// 2. Every step: poses and twists of all bodies, indexed as in the pairs.
model.evaluate(poses, twists, friction, &eval);    // eval.wrenches[i]: on body i, about its origin, world

// 3. Apply the wrenches, then keep the state for the next step.
friction.swap(eval.next_friction);
```

Geometry comes from `CollisionShape(TriMesh)` (vertex and index arrays) or `from_obj`. A body made
of several convex pieces (needed by the distributed law, which wants convex shapes) goes through
`CompoundShape::from_meshes`: one pair per piece, and the seams between pieces are found, so
decomposition does not change the force. Gonthier's law accepts any closed mesh.

The same from C: `core/include/volcontact/c_api.h` (opaque handles, plain arrays, status codes,
friction state as `VOLCONTACT_FRICTION_STATE_SIZE` doubles per pair; `volcontact_model_set_explicit_coupling`
as below). `tests/capi_drive.c` is a complete
driver. Bindings for other languages wrap this header.

## How to add a backend

A backend in this repository implements `volcontact::Engine` (`engines/include/volcontact/engine.hpp`):
`run(const Scenario&, const RunOptions&)` builds the engine's world from the scenario's bodies
(mass, inertia, initial pose and twist, `applied_force`) and grounds, steps it for `duration` with
`dt`, and returns the sampled trajectory and timings. Register it in `engines/registry.cpp` and add
it to `engines/CMakeLists.txt` behind its own `VOLCONTACT_WITH_<ENGINE>` option. For the volumetric
methods it calls `make_contact_model(scenario, force_model(options.contact))` (bodies numbered
dynamic first, then the static grounds) and does, in this order, **every step**:

1. read the pose (`R`, `p` of the body origin) and twist (`w`, `v` of the origin, both in world) of
   each dynamic body into `poses`/`twists` (grounds are set once);
2. `model.evaluate(poses, twists, friction, &eval)`, then `friction.swap(eval.next_friction)`;
3. write `eval.wrenches[i]` (plus any applied force) into the engine's external-force slot of body
   `i`; 4. advance the engine one step.

Before the first step, if the engine integrates explicitly (all three backends do), tell the model
the step and the bodies' inverse masses and inertias: `model.set_explicit_coupling(dt, bodies)`
(`make_contact_model(scenario, model, dt)` does it for the scenarios), or in C
`volcontact_model_set_explicit_coupling(model, dt, num_bodies, inv_masses, inv_inertias)` (inverse
inertia tensors row-major, 9 doubles per body, in the body frame about the origin, which must be
the centre of mass; zeros for bodies that do not move). The laws then bound their stick spring and
damper and their normal damper to what that step can integrate (`StepLimits`: roughly
K·A·Σw·dt² ≤ 2 for a body held between stiff contacts); below the bounds nothing changes, and
without the call, or with `dt = 0`, they are evaluated as given. Use the engine's real step and
call it again if the step changes.

The three backends differ only in step 3 and in where the friction state lives:

| Engine | Where the wrench goes | Friction state |
|---|---|---|
| Chrono (`chrono_backend/`) | `ChBody` accumulator, then `UpdateForces` so it acts this step | local vector, swapped each step |
| MuJoCo (`mujoco_backend/`) | `xfrc_applied` (force, torque, at the CoM, world), rewritten for every body | local vector |
| Drake (`drake_backend/`) | a `LeafSystem` output port of spatial forces; evaluation is a cache entry | discrete state of the system, updated by a periodic event |

Pitfalls, all met while writing those:

- **One-step lag.** The wrench belongs to the state it was evaluated at. Evaluate at the start of
  the step and apply it within that step; applying it a step later (Chrono's accumulators are only
  gathered when the body updates, hence `UpdateForces`) costs accuracy and, for stiff contacts,
  stability. The coupling is explicit: with `dt = 1 ms` very stiff contacts need smaller steps or
  an implicit integrator (the force Jacobians are not provided yet).
- **Evaluate once per step.** Engines with several stages per step (Runge–Kutta, Drake's event
  ordering) must evaluate on the step's initial state only, or the friction state advances twice.
  Drake's backend recognizes the second request in its cache entry and serves the first result.
- **Friction state is per pair and must persist** while the pair exists. It stores the bristle in
  body B's frame, so it follows B. Always pass back `next_friction`, never the old state; a pair
  that separates is reset by the model. Initial state: `initial_friction()` /
  `volcontact_friction_state_init`. Engines that only store numbers use `FrictionState::pack/unpack`
  (`kSize` doubles).
- **Frames.** Poses are the body frame's pose in world (not the centre of mass); twists are the
  velocity of the *frame origin*, not of the CoM. Wrenches come back about the body origin. If your
  engine's body frame is at the CoM, nothing to convert; if not, convert the wrench to the CoM
  before applying it (`τ_com = τ_o + (p_o − p_com) × F`).
- **Shapes** are in the body frame, closed, with outward counter-clockwise triangles (an inside-out
  mesh is flipped). Units are SI; `K` is in N/m³.
- **Multiple pairs on a body** are summed by the model: apply `wrenches[i]` once per body, not once
  per pair. Bodies with no contact get a zero wrench.
- **Threads.** `evaluate` may run pairs in parallel (OpenMP); one `Evaluation` per concurrent
  caller. `finalize()` (or the first `evaluate`) builds the broadphase and is safe to race.
