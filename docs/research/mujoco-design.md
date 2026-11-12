# MuJoCo backend: design for volumetric-contact

Status: design only. Nothing was built or run. API facts were checked against the
MuJoCo 4.0.0 headers and docs downloaded to `/tmp/vc-research/mj/`. Section 5 was written
from background knowledge and was NOT re-verified online (marked there).

Version: **MuJoCo 4.0.0** (2026-11-06, latest stable; previous 3.17.0).
Release: https://github.com/google-deepmind/mujoco/releases/tag/4.0.0
Tarball: https://github.com/google-deepmind/mujoco/releases/download/4.0.0/mujoco-4.0.0-linux-x86_64.tar.gz
sha256 `3d5d2d96b6d286b758c75198aa7862590c960256626d3aac53ac5721b0a697f3` (downloaded and matched).
Docs: https://mujoco.readthedocs.io/en/stable/ (computation/index, modeling, XMLreference,
programming/simulation, overview). Headers: https://github.com/google-deepmind/mujoco/tree/4.0.0/include/mujoco

## 1. Volumetric laws on MuJoCo

**Force application.** Same pattern as Chrono, with no lag: right before every `mj_step`,
read `d->qpos`/`d->qvel` of each free joint, evaluate the contact model at that state, write the
wrench into `d->xfrc_applied[6*body_id .. +5]` (force xyz, then torque xyz, world frame, applied
at the body CoM), then call `mj_step`. `mj_step` = `mj_forward` + integrator, and `xfrc_applied`
enters `qfrc_smooth` through J^T. Our `Wrench` is about Bo = CoM, so it maps 1:1.
- Never read `d->xpos/xmat` between steps for the model input: derived fields describe the state
  before the last step. Use `qpos/qvel` (they are the integrated state).
- Explicit coupling at the current state: no callbacks, no plugin, no global state (`mjcb_*` are
  global and thread-unsafe; avoid). `mj_step1/mj_step2` is not needed; it only supports the
  single-step integrators and RK4 would need `mjcb_control`.
- `xfrc_applied` has no derivatives, so with implicit integrators it is still explicit coupling
  (the same limit as open problem 4 in AGENTS.md). RK4 holds `xfrc` constant over its stages.
- `xfrc_applied` is NOT cleared by `mj_step`; overwrite it every step (including to 0 for bodies
  with no contact and for the `applied_force(t)` case: add the applied force to the force part).

**Free joints and quaternions.** `qpos` = x,y,z + quaternion (w,x,y,z), world frame. `qvel` =
linear velocity in WORLD frame (of the body-frame origin), then angular velocity in the LOCAL
body frame (overview.rst). So `Twist.w = R(q) * qvel[3:6]`, and for initial conditions
`qvel[3:6] = R^T * w`. Set `freejoint align=false` (spec: `mjs_addFreeJoint` then
`joint->align = mjALIGNFREE_FALSE`; XML `alignfree="false"`); the default would move the body
frame to the inertial frame and break the identification of `qpos` with our body pose.
Quaternion gets renormalised by the integrator; still normalise on input.

**Inertia.** Use an explicit inertial: `body->mass`, `body->ipos = 0`, `body->fullinertia = {Ixx,
Iyy, Izz, Ixy, Ixz, Iyz}` taken from `BodySpec.inertia` (about the CoM, body frame), with
`body->explicitinertial = 1`. MuJoCo diagonalises it and sets `iquat`. With an explicit inertial
the mesh geoms contribute no mass (`inertiafromgeom=auto` only uses geoms when no inertial given).
Never rely on mesh inertia (default `legacy`).

**Integrator.** Primary: `mjINT_IMPLICITFAST` (doc-recommended default; for free bodies it keeps
the gyroscopic derivatives, so spinning bodies in contensou, disk, ring, bearing, cone and
billiard stay stable). Sensitivity variants: `mjINT_EULER` (closest to Chrono's linearised Euler;
docs warn it "quickly diverges" on gyroscopic cases, so expect failures that are real) and
`mjINT_RK4` (4.0: 4th order also for free-joint orientation). New `discrete` integrator
(Sep 2026, "subject to change") is not used. Expose via `RunOptions` extension or an env var
`VOLCONTACT_MUJOCO_INTEGRATOR` for the study; default is fixed in code.

**Disabling MuJoCo contacts for the volumetric runs.** Do not add any collision geoms/pairs
(volumetric mode needs only the meshes in our own code). Belt and braces:
`spec->option.disableflags |= mjDSBL_CONTACT`. Also `|= mjDSBL_AUTORESET` so a numerical issue
cannot silently reset the state; instead test `d->warning[mjWARN_BADQPOS/BADQVEL/BADQACC].number`
after each step and report failure in `RunStats`/`RunResult`. Also disable `mjDSBL_LIMIT`,
`mjDSBL_EQUALITY`, `mjDSBL_FRICTIONLOSS` is unnecessary (no joints besides free). Gravity is
`spec->option.gravity` (tilt for incline scenarios = rotate gravity vector, same as the other
engines' scenario API; grounds stay in the world).

**Time.** `spec->option.timestep = dt`; loop `steps = llround(duration/dt)` as in Chrono.
Time used for `applied_force(t)` is `d->time` before the step.

## 2. Native MuJoCo contact: one fixed generic mapping

**Structure.** All geoms: `contype = conaffinity = 0`. For every `ContactSpec` create explicit
pairs (`mjs_addPair`) between each convex piece of body A and each convex piece of body B
(same as the Chrono backend). Explicit pairs bypass contype/conaffinity and parent-child
filtering and ignore geom parameters, so per pair we set exactly: `condim`, `friction[5]`
(tangential x2, torsional, rolling x2), `solref`, `solreffriction`, `solimp`, `margin`, `gap`.
This avoids MuJoCo's geom mixing rule (elementwise max of friction) and gives per-contact mu,
as the Chrono `PairFriction` does.

**Geometry.** Mesh geoms collide as their convex hulls (qhull), so concave grounds use the
convex `pieces/` as separate geoms, as in Chrono. Planar grounds use `mjGEOM_PLANE`
(plane-mesh/box/cylinder up to 4 contacts, plane-sphere 1). Mesh-mesh: `nativeccd` (default on)
with `multiccd` (default enabled) gives up to 4 points per pair; legacy libccd is MPR with 5 runs.
`mjMAXCONPAIR` = 50. `ccd_tolerance` 1e-6, `ccd_iterations` 35. With `native_primitives`,
`BodySpec.primitive` maps to box/cylinder(+z)/sphere; this gives analytic and multi-point plane
contacts. SDF not used (needs plugin/mesh SDF, contacts are point-based anyway).

**Contact model.** `condim = 4` default (normal, 2 tangential, torsional) to include the
spin friction scenario families (contensou, disk), with a torsional coefficient in metres
(`friction[2]`) that is NOT a free parameter: derive it as for Chrono (see native_materials
mapping) or set to 0 and document. `cone = mjCONE_ELLIPTIC`, `impratio = 10`, `noslip_iterations
= 3` (noslip removes slow slippage; docs state friction dimensions are purely damped with no
position residual, so exact stiction is not guaranteed), `solver = Newton`, tolerance default.
Pyramidal and elliptic define different soft-contact dynamics; record the choice in docs.

**K, h, mu -> solref/solimp.** Facts: standard format (timeconst, dampratio): b = 2/(dmax*timeconst),
k = d(r)/(dmax^2 timeconst^2 dampratio^2). Direct format (negative numbers, -stiffness,
-damping): b = damping/dmax, k = stiffness*d(r)/dmax^2. These act on the constraint-space
acceleration, i.e. stiffness is **per unit effective mass** (1/s^2), and the constraint force is
the effective mass times that. The Winkler target is force = K*V*(1 - h*vn), with V the overlap
volume, so the equivalent point stiffness is K*A_c (A_c = contact area). MuJoCo has no area, so
a generic mapping must pick A_c and m_eff from geometry, not from a scenario:
- `A_c` = area of the contact patch estimated generically per pair: the smaller of the two
  pieces' largest projected cross-section along the pair's centre-to-centre axis (computed
  once from the hull at setup).
- `m_eff` = reduced mass of the pair (`m1*m2/(m1+m2)`, grounds count as infinite mass).
- `solref = (-K*A_c/m_eff, -c/m_eff)` with c = h * K*A_c * delta_ref, delta_ref = m_eff*g/(K*A_c)
  (the static sag), so c = h*m_eff*g. Pass `solimp = (dmax, dmax, 0.001, 0.5, 2)` with
  dmax = 0.99 (stiff, near constant impedance so the direct-format formulas hold).
- `refsafe` forces timeconst >= 2*dt. Effective timeconst is about 2*pi/sqrt(k) /...: at
  dt = 1 ms the maximum representable stiffness per mass is about (1/(2 dt))^2 = 2.5e5 s^-2;
  at 0.2 ms it is 6.25e6 s^-2. Stiffer requests are clamped silently; log and report the
  clamped fraction in stats (this is a real limitation, comparable to open problem 4).
- This mapping is a RECOMMENDATION and unvalidated; it is the weakest part of the design.
  Fallback with no geometry assumption: use MuJoCo defaults (solref 0.02/1, solimp
  0.9/0.95/0.001/0.5/2) and treat MuJoCo native as "MuJoCo default soft contact", stating
  clearly that K and h do not apply (like docs for the Drake point contact rules).
- mu: `friction[0..1] = mu` per pair (min `mjMINMU` 1e-5), no stiction cap logic needed
  beyond what the cone gives.
- `o_solref/o_solimp/o_friction` with `mjENBL_OVERRIDE` is a global override; not used.

**dt.** At dt = 1 ms, `timeconst >= 2 ms` limits the mapping above; at 0.2 ms it relaxes. Report
both in the battery. MuJoCo default integration supports 1 ms comfortably.

## 3. Build

**Important finding:** the Linux tarball contains `lib/libmujoco.so(.4.0.0)` and `include/`,
but NO `lib/cmake/mujoco/`, so `find_package(mujoco CONFIG)` fails on it. Either build from
source (`cmake` of the repo ships `mujocoConfig.cmake.in`; slower image, brings deps via
FetchContent: needs network at image build), or (recommended) a small `cmake/FindMuJoCo.cmake`:

```cmake
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS ${MUJOCO_ROOT} /opt/mujoco PATH_SUFFIXES include)
find_library(MUJOCO_LIBRARY mujoco HINTS ${MUJOCO_ROOT} /opt/mujoco PATH_SUFFIXES lib)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MuJoCo DEFAULT_MSG MUJOCO_INCLUDE_DIR MUJOCO_LIBRARY)
if(MuJoCo_FOUND AND NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
endif()
```
The core library has no GL/GLFW dependency; no GL packages needed (the tarball ships SDL2
headers for the samples only). Not checked: `readelf` of runtime deps and the glibc floor
(the check was interrupted); verify in Docker with `ldd /opt/mujoco/lib/libmujoco.so`.

Dockerfile addition:
```dockerfile
ARG MUJOCO_VERSION=4.0.0
RUN curl -fsSL -o /tmp/mj.tgz https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
 && echo "3d5d2d96b6d286b758c75198aa7862590c960256626d3aac53ac5721b0a697f3  /tmp/mj.tgz" | sha256sum -c - \
 && mkdir /opt/mujoco && tar -xzf /tmp/mj.tgz -C /opt/mujoco --strip-components=1 && rm /tmp/mj.tgz
ENV MUJOCO_ROOT=/opt/mujoco LD_LIBRARY_PATH=/opt/mujoco/lib:$LD_LIBRARY_PATH
```
Bump `IMAGE` in `scripts/dev.sh` (now `...drake-1.58.0-chrono-10.0.0`, e.g. add `-mujoco-4.0.0`).

Top-level `CMakeLists.txt`: `option(VOLCONTACT_WITH_MUJOCO "Build the MuJoCo backend" ON)` next to the Chrono option;
`engines/CMakeLists.txt`: `list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)`,
`find_package(MuJoCo REQUIRED)`, `target_sources(... mujoco_backend/mujoco_engine.cpp)`,
`target_link_libraries(... PRIVATE mujoco::mujoco)`, `target_compile_definitions(... PUBLIC VOLCONTACT_WITH_MUJOCO)`.
`engines/registry.cpp`: `#ifdef VOLCONTACT_WITH_MUJOCO` include `mujoco_backend/mujoco_engine.hpp` and register
`"mujoco"` -> `make_mujoco_engine()`.

## 4. File-level plan

Add: `engines/mujoco_backend/mujoco_engine.{hpp,cpp}`, `cmake/FindMuJoCo.cmake`,
`tests/test_engine_mujoco.cpp`. Change: Dockerfile, `scripts/dev.sh`, top-level and engines CMake,
`engines/registry.cpp`, `tests/CMakeLists.txt`, bench engine list (`scripts/bench.sh`, `tools/stats.py` engine names),
`docs/scenarios.md` §3 (native mapping rules), `docs/history.md` (dated entry), `README.md`.
Per AGENTS.md: no force-law changes; the mapping is generic.

`mujoco_engine.hpp`: `std::unique_ptr<Engine> make_mujoco_engine();` (same pattern as chrono).

Skeleton (names follow the verified mjspec.h/mujoco.h; double-check `mjsGeom`/`mjsPair` field names when compiling):

```cpp
#include <mujoco/mujoco.h>

namespace volcontact {
namespace {

struct Model {
  mjSpec* spec = nullptr; mjModel* m = nullptr; mjData* d = nullptr;
  std::vector<int> body_id;      // dynamic bodies, scenario order
  ~Model() { if (d) mj_deleteData(d); if (m) mj_deleteModel(m); if (spec) mj_deleteSpec(spec); }
};

void add_mesh_geom(mjSpec* s, mjsBody* b, const std::string& name, const MeshData& hull,
                   const Pose& local) {
  mjsMesh* me = mjs_addMesh(s, nullptr);
  mjs_setString(me->name, name.c_str());
  std::vector<float> v = flatten_vertices(hull);          // float
  mjs_setFloat(me->uservert, v.data(), (int)v.size());    // faces optional for hulls
  mjsGeom* g = mjs_addGeom(b, nullptr);
  g->type = mjGEOM_MESH;
  mjs_setString(g->meshname, name.c_str());
  copy_pose(g->pos, g->quat, local);                      // quat (w,x,y,z)
  g->contype = g->conaffinity = 0;                        // explicit pairs only
  g->mass = 0;                                            // inertia is explicit
}

}  // namespace

class MujocoEngine : public Engine {
 public:
  RunResult run(const Scenario& sc, const RunOptions& opt) override {
    Model M;
    M.spec = mj_makeSpec();
    M.spec->option.timestep   = opt.dt;
    M.spec->option.integrator = mjINT_IMPLICITFAST;
    M.spec->option.cone       = mjCONE_ELLIPTIC;
    M.spec->option.impratio   = 10;
    M.spec->option.noslip_iterations = 3;
    M.spec->option.disableflags |= mjDSBL_AUTORESET;
    if (opt.contact == ContactMethod::Native) { /* contact on */ }
    else M.spec->option.disableflags |= mjDSBL_CONTACT;
    set_vec3(M.spec->option.gravity, sc.gravity);

    mjsBody* world = mjs_findBody(M.spec, "world");
    for (std::size_t i = 0; i < sc.bodies.size(); ++i) {
      const BodySpec& bs = sc.bodies[i];
      mjsBody* b = mjs_addBody(world, nullptr);
      mjs_setString(b->name, ("b" + std::to_string(i)).c_str());
      mjsJoint* j = mjs_addFreeJoint(b);
      j->align = mjALIGNFREE_FALSE;
      b->mass = bs.mass; b->explicitinertial = 1;
      b->ipos[0] = b->ipos[1] = b->ipos[2] = 0;
      set_fullinertia(b->fullinertia, bs.inertia);        // xx yy zz xy xz yz
      b->pos / b->quat = bs.initial_pose;                 // initial pose via body frame
      for (auto& piece : pieces_of(bs)) add_mesh_geom(M.spec, b, ..., piece, Pose{});
    }
    for (auto& g : sc.grounds) add_ground(M.spec, world, g, opt);  // plane or convex pieces on world
    if (opt.contact == ContactMethod::Native)
      for (auto& cs : sc.contacts) add_pairs(M.spec, cs, opt);     // condim, friction, solref, solimp

    M.m = mj_compile(M.spec, nullptr);
    if (!M.m) throw std::runtime_error(mjs_getError(M.spec));
    M.d = mj_makeData(M.m);
    for (i...) { body_id[i] = mj_name2id(M.m, mjOBJ_BODY, ...);
                 write_initial_state(M, i, bs); }            // qpos (pos, quat), qvel (v world, R^T w local)
    mj_forward(M.m, M.d);

    RunResult out; RunStats stats;
    auto record = [&](double t) { out.push(t, poses(M), twists(M)); };
    const long steps = std::llround(sc.duration / opt.dt);
    record(0.0);
    for (long k = 0; k < steps; ++k) {
      const double t = M.d->time;
      auto poses = poses_from_qpos(M);                       // from qpos, not xpos
      auto twists = twists_from_qvel(M);                     // w_world = R * w_local
      std::vector<Wrench> W = evaluate_model(sc, poses, twists, t, stats);  // dynamic first, then grounds
      std::fill(M.d->xfrc_applied, M.d->xfrc_applied + 6 * M.m->nbody, 0.0);
      for (i...) { double* x = M.d->xfrc_applied + 6 * M.body_id[i];
                   x[0..2] = W[i].force + applied_force(i, t); x[3..5] = W[i].torque; }
      mj_step(M.m, M.d);
      if (any_bad_warning(M.d)) { out.failed = true; break; }
      if ((k + 1) % opt.record_every == 0) record(M.d->time);
    }
    return out;
  }
};
}  // namespace volcontact
```
Friction-state swap (`ContactModel` internal state, same as Chrono): the model evaluates once per step
from qpos/qvel, then state advances once per step; no second evaluation. Native-mode runs skip
`evaluate_model` and let `mj_step` handle contacts; `contact_seconds` comes from `d->ncon > 0`.

**Tests** (`tests/test_engine_mujoco.cpp`, mirror `test_engine_chrono.cpp`):
1. Native per-contact mu on an incline: cube starts sliding iff tan(theta) > mu, two pair mus.
2. Applied force `F = (t,0,0)` on a free body in all three methods: `v(T) = T^2/(2m)` within 1e-3.
3. Cube rests in `corner_s300` under `gonthier` and `distributed` (no drift > a tolerance).
4. Free-joint conventions: spin about a body axis, `Twist.w` in world equals R*local; initial
   condition roundtrip; body origin vs CoM offset (use an off-centre `BodySpec` if one exists).
5. Bad-state detection: forced NaN/huge dt reports failure rather than auto-reset.

**Pitfalls.**
- Body frame vs CoM: `xpos` is the body frame, `xipos` the CoM; with explicit inertial and `ipos = 0` they coincide; keep `alignfree=false`.
- Derived fields (`xpos`, `xmat`, `cvel`) are stale after `mj_step`; call `mj_forward` after writing initial state only.
- `qvel` angular is LOCAL; `mj_objectVelocity` returns object-centred (CoM) velocity.
- Mesh recentring: the compiler recentres meshes to their inertial frame, but the geom stays where placed.
- `xfrc_applied` persists between steps; always overwrite.
- Large grounds: 704-piece `bowl` gives many explicit pairs; explicit pairs still pass a per-step bounding test
  (cost, not correctness). Consider only pairing pieces within the AABB sweep.
- `mjtNum` is double (default build); warm-start `qacc_warmstart` carries over (reset between runs via new `mjData`).
- `mjcb_*` callbacks are global; avoid for thread safety. Use one `mjData` per run.
- `autoreset` disabled on purpose; check warnings.
- 4.0.0 removed `composite` and the `cable` plugin (not used). Speculative contacts (Oct 2026) and `discrete` integrator are new and not used.
- Hull convexification of non-convex meshes silently changes the geometry if pieces are not pre-split.
- Torsional and rolling friction coefficients are lengths, not dimensionless: do not copy mu.

## 5. Candidate 4th engines (not re-verified online; knowledge as of mid-2026, check before citing)

Criterion: adds scientific value for a contact-MODEL comparison paper (different contact formulation,
supports arbitrary meshes/volumes, deterministic, free) per unit cost.

- **Bullet/PyBullet**: sequential-impulse LCP with pyramidal friction, penetration-based point contacts, convex hulls/GImpact.
  Cheap to bind (C++ library, apt `libbullet-dev`). Adds a well known baseline of the *rigid/impulse* family, but Chrono already uses
  Bullet collision, so value is modest. Cost low.
- **PhysX 5 / Newton (NVIDIA, open source, MuJoCo-Warp based in 2025-2026)**: PBD/TGS solver, GPU-oriented, convex meshes, SDF
  contact in PhysX 5. Strong industry relevance; needs CUDA/Python stack for Newton, heavy to containerise.
  Value medium-high (SDF contact is mesh-based, closest to volumetric ideas), cost high.
- **Simbody (Stanford)**: has `ElasticFoundationForce` (Winkler-like, per-face pressure on triangle meshes,
  `ContactTrackerSubsystem`) and `HuntCrossleyForce`, plus the Hertz contact family, and a Hunt-Crossley point model.
  Of all candidates this is the **closest scientific peer** of `distributed` (elastic foundation on arbitrary meshes), so it provides a
  genuine second implementation of the same physical idea. Cost medium: C++ source build, no Docker image prebuilt, its mesh
  contact needs closed manifold meshes and was not designed for stiff dynamics at 1 ms. Highest scientific value.
- **Jolt**: fast rigid-body engine, convex hulls, no soft contact. Little scientific novelty. Cost low-medium.
- **Rapier**: Rust, impulse-based, convex/trimesh; same family as Bullet. Low novelty, medium cost (FFI).
- **Genesis**: GPU/Python framework with MPM/FEM/rigid coupling; heavy dependency stack and nondeterministic setups. Low fit for C++ pipeline, high cost.
- **ODE**: classic LCP, mature and tiny, hard contacts with CFM/ERP soft parameters (ERP/CFM is a spring-damper in disguise).
  Cheap to add; value as historical baseline only.

Recommendation: MuJoCo (this design) first, then **Simbody** as the one scientifically valuable addition
(independent implementation of elastic-foundation contact), Bullet only if a cheap rigid-impulse reference is wanted.
All of Section 5 should be confirmed against current docs before a paper cites it.
