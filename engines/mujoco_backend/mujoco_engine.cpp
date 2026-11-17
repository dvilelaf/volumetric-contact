#include "mujoco_backend/mujoco_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "native_geometry.hpp"

namespace volcontact {
namespace {

struct SpecDeleter {
  void operator()(mjSpec* s) const { mj_deleteSpec(s); }
};
struct ModelDeleter {
  void operator()(mjModel* m) const { mj_deleteModel(m); }
};
struct DataDeleter {
  void operator()(mjData* d) const { mj_deleteData(d); }
};

// MuJoCo's default warning handler also appends to MUJOCO_LOG.TXT in the working
// directory; warnings go to stderr only (the run loop turns them into errors).
void warnings_to_stderr() {
  static std::once_flag once;
  std::call_once(once, [] { mju_user_warning = [](const char* msg) { std::fprintf(stderr, "mujoco: %s\n", msg); }; });
}

void set_pose(double pos[3], double quat[4], const Pose& X) {
  const Eigen::Quaterniond q(X.R);
  for (int k = 0; k < 3; ++k) pos[k] = X.p[k];
  quat[0] = q.w(), quat[1] = q.x(), quat[2] = q.y(), quat[3] = q.z();
}

// --- Native contact (docs/scenarios.md §3, "MuJoCo") ---------------------------

// `soft`: the configuration MuJoCo's documentation recommends, with the
// scenario's K mapped onto the contact stiffness. `default`: MuJoCo's own
// contact parameters and options on the same geometry.
enum class NativeModel { kSoft, kDefault };

// Constant constraint impedance d0: MuJoCo's default lower value, held at every
// depth so that the direct stiffness and damping below are exact.
constexpr double kImpedance = 0.9;
// Points of MuJoCo's contact manifold between two flat faces; each stands for
// this share of the pair's reference area. A sphere touches at one point.
constexpr double kManifoldPoints = 4.0;
// Solver tolerances: MuJoCo's manual asks for an aggressive tolerance with
// Newton. With the default 1e-8 the four corners of a sliding box get unequal
// forces and it wobbles.
constexpr double kSolverTolerance = 1e-12;
constexpr double kNoslipTolerance = 1e-10;

// A collision geom and what the mapping needs of it.
struct NativeGeom {
  std::string name;
  double area = std::numeric_limits<double>::infinity();  // Reference area [m²]; infinite for a plane.
  bool point = false;                                     // Touches at a single point (sphere).
};

// Reference area of a primitive: a sixth of its surface, as native::reference_area.
double reference_area(const Primitive& p) {
  const double pi = std::numbers::pi;
  switch (p.kind) {
    case Primitive::Kind::kBox: return (p.size.x() * p.size.y() + p.size.y() * p.size.z() + p.size.z() * p.size.x()) / 3.0;
    case Primitive::Kind::kCylinder: return pi * p.size.x() * (p.size.x() + p.size.y()) / 3.0;
    case Primitive::Kind::kSphere: return 2.0 * pi * p.size.x() * p.size.x() / 3.0;
  }
  return 0.0;
}

mjsGeom* add_geom(mjsBody* body, const std::string& name, const Pose& X) {
  mjsGeom* g = mjs_addGeom(body, nullptr);
  mjs_setName(g->element, name.c_str());
  set_pose(g->pos, g->quat, X);
  g->contype = g->conaffinity = 0;  // Only the explicit pairs collide.
  return g;
}

// One mesh geom per convex piece, at `X` in the body's frame; MuJoCo collides a
// mesh as its convex hull.
void add_pieces(mjSpec* spec, mjsBody* body, const std::string& prefix, const ShapeMeshes& shape, const Pose& X,
                std::vector<NativeGeom>& geoms) {
  for (std::size_t k = 0; k < shape.pieces.size(); ++k) {
    const TriMesh mesh = load_obj(shape.pieces[k]);
    const std::string name = prefix + "p" + std::to_string(k);
    std::vector<float> vertices;
    std::vector<int> faces;
    for (const Vec3& v : mesh.vertices) {
      for (int i = 0; i < 3; ++i) vertices.push_back(static_cast<float>(v[i]));
    }
    for (const auto& f : mesh.triangles) {
      for (int i = 0; i < 3; ++i) faces.push_back(static_cast<int>(f[i]));
    }
    mjsMesh* m = mjs_addMesh(spec, nullptr);
    mjs_setName(m->element, name.c_str());
    mjs_setFloat(m->uservert, vertices.data(), static_cast<int>(vertices.size()));
    mjs_setInt(m->userface, faces.data(), static_cast<int>(faces.size()));
    mjsGeom* g = add_geom(body, name, X);
    g->type = mjGEOM_MESH;
    mjs_setString(g->meshname, name.c_str());
    geoms.push_back({name, native::reference_area(mesh), false});
  }
}

void add_primitive(mjsBody* body, const std::string& name, const Primitive& p, std::vector<NativeGeom>& geoms) {
  mjsGeom* g = add_geom(body, name, p.X_BG);
  switch (p.kind) {
    case Primitive::Kind::kBox:
      g->type = mjGEOM_BOX;
      for (int i = 0; i < 3; ++i) g->size[i] = p.size[i] / 2.0;
      break;
    case Primitive::Kind::kCylinder:
      g->type = mjGEOM_CYLINDER;
      g->size[0] = p.size.x(), g->size[1] = p.size.y() / 2.0;
      break;
    case Primitive::Kind::kSphere:
      g->type = mjGEOM_SPHERE;
      g->size[0] = p.size.x();
      break;
  }
  geoms.push_back({name, reference_area(p), p.kind == Primitive::Kind::kSphere});
}

// The parameters of one explicit pair of geoms. `soft`: the scenario's K and μ;
// `default`: μ, the rest as MuJoCo sets a pair. Returns whether the stiffness
// was limited by the time step.
bool set_pair(mjsPair* pair, NativeModel model, const NativeGeom& a, const NativeGeom& b, double mass, const Scenario& s,
              double mu, double dt) {
  mjs_setString(pair->geomname1, a.name.c_str());
  mjs_setString(pair->geomname2, b.name.c_str());
  pair->friction[0] = pair->friction[1] = mu;
  pair->margin = pair->gap = 0.0;
  if (model == NativeModel::kDefault) {
    pair->condim = 3;  // solref (0.02, 1), solimp (0.9, 0.95, 0.001, 0.5, 2): MuJoCo's.
    return false;
  }
  // Each contact point stands for a patch of area A: the smaller reference
  // area of the two geoms, shared by the points of the manifold.
  const double area = std::min(a.area, b.area) / (a.point || b.point ? 1.0 : kManifoldPoints);
  // MuJoCo's soft constraint gives the contact the acceleration
  //   a1 = (1 − d0)·a0 + k·δ − b·δ̇   (a0 without the contact, δ the penetration),
  // so a point carries, at rest, the force k·δ / ((1 − d0)·Â), where MuJoCo's
  // Â is the inverse of the pair's reduced mass. For that force to be the
  // Winkler patch's K·A·δ:
  const double wanted = (1.0 - kImpedance) * s.native.stiffness * area / mass;
  // The stiffest reference MuJoCo's refsafe accepts has a time constant of two
  // steps: stiffness 1/(2·dt)².
  const double stiffness = std::min(wanted, 1.0 / (4.0 * dt * dt));
  // Critical damping, as MuJoCo recommends. The scenario's h does not apply:
  // the damper is linear, and h linearized at the static load leaves the
  // contact underdamped, which makes sliding bodies bounce.
  pair->solref[0] = -stiffness;
  pair->solref[1] = -2.0 * std::sqrt(stiffness);
  const double solimp[mjNIMP] = {kImpedance, kImpedance, 0.001, 0.5, 2.0};
  std::copy(solimp, solimp + mjNIMP, pair->solimp);
  // The friction rows keep MuJoCo's default reference (critically damped, 20 ms).
  // A stiffer one makes sliding bodies hop higher: the friction cone couples the
  // tangential reference acceleration, which grows with the sliding speed, into
  // the normal direction.
  pair->solreffriction[0] = 0.02;
  pair->solreffriction[1] = 1.0;
  pair->condim = 4;
  // Torsional coefficient [m]: a disk of area A under uniform pressure.
  pair->friction[2] = 2.0 / 3.0 * mu * std::sqrt(area / std::numbers::pi);
  return wanted > stiffness;
}

// --- State ---------------------------------------------------------------------

// A dynamic body's free joint: qpos = position and quaternion (w, x, y, z) of
// the body frame in world; qvel = its linear velocity in world, then its
// angular velocity in the body frame.
struct FreeBody {
  int id = 0;    // Body.
  int qpos = 0;  // Address in qpos.
  int qvel = 0;  // Address in qvel.
};

Pose pose_of(const mjData& d, const FreeBody& b) {
  const double* q = d.qpos + b.qpos;
  return {Eigen::Quaterniond(q[3], q[4], q[5], q[6]).normalized().toRotationMatrix(), Vec3(q[0], q[1], q[2])};
}

Twist twist_of(const mjData& d, const FreeBody& b, const Pose& X) {
  const double* v = d.qvel + b.qvel;
  return {X.R * Vec3(v[3], v[4], v[5]), Vec3(v[0], v[1], v[2])};
}

class MujocoEngine final : public Engine {
 public:
  [[nodiscard]] std::string_view name() const override { return "mujoco"; }
  // MuJoCo has one contact model, soft constraints between convex geoms, in
  // two configurations (docs/scenarios.md §3).
  [[nodiscard]] std::vector<std::string> native_models() const override { return {"soft", "default"}; }

  [[nodiscard]] RunResult run(const Scenario& s, const RunOptions& o) const override {
    warnings_to_stderr();
    const bool native = o.contact == ContactMethod::kNative;
    // Rejects a model MuJoCo lacks.
    const NativeModel rule = native && native_model(*this, o) == "default" ? NativeModel::kDefault : NativeModel::kSoft;
    const std::size_t nb = s.bodies.size();

    const std::unique_ptr<mjSpec, SpecDeleter> spec(mj_makeSpec());
    mjsBody* world = mjs_findBody(spec.get(), "world");
    // Collision geoms of each body, numbered as the contact model does.
    std::vector<std::vector<NativeGeom>> geoms(nb + s.grounds.size());
    for (std::size_t i = 0; i < nb; ++i) {
      const BodySpec& b = s.bodies[i];
      mjsBody* body = mjs_addBody(world, nullptr);
      mjs_setName(body->element, ("b" + std::to_string(i)).c_str());
      set_pose(body->pos, body->quat, b.X_W0);
      // The body frame stays where the scenario puts it (MuJoCo would otherwise
      // move a free body's frame to its principal axes of inertia).
      mjs_addFreeJoint(body)->align = mjALIGNFREE_FALSE;
      // Explicit mass properties about the body origin; the geoms carry none.
      body->explicitinertial = 1;
      body->mass = b.mass;
      std::fill(body->ipos, body->ipos + 3, 0.0);
      const Mat3& I = b.inertia;
      const double full[6] = {I(0, 0), I(1, 1), I(2, 2), I(0, 1), I(0, 2), I(1, 2)};
      std::copy(full, full + 6, body->fullinertia);
      if (!native) continue;
      const std::string prefix = "b" + std::to_string(i);
      // The analytic shape when the scenario gives one: exact normals and
      // MuJoCo's dedicated colliders.
      if (b.primitive) add_primitive(body, prefix, *b.primitive, geoms[i]);
      else add_pieces(spec.get(), body, prefix, b.shape, Pose{}, geoms[i]);
    }
    if (native) {
      for (std::size_t g = 0; g < s.grounds.size(); ++g) {
        const GroundSpec& ground = s.grounds[g];
        const std::string prefix = "g" + std::to_string(g);
        if (ground.planar) {
          mjsGeom* plane = add_geom(world, prefix, ground.X_WG);
          plane->type = mjGEOM_PLANE;
          plane->size[0] = plane->size[1] = 0.0;  // Infinite.
          plane->size[2] = 1.0;
          geoms[nb + g].push_back({prefix, std::numeric_limits<double>::infinity(), false});
        } else {
          add_pieces(spec.get(), world, prefix, ground.shape, ground.X_WG, geoms[nb + g]);
        }
      }
      int pairs = 0, limited = 0;
      for (const ContactSpec& c : s.contacts) {
        // Reduced mass of the pair; a ground has infinite mass.
        double inverse_mass = 0.0;
        for (const BodyRef& ref : {c.a, c.b}) {
          if (ref.kind == BodyRef::Kind::kBody) inverse_mass += 1.0 / s.bodies[static_cast<std::size_t>(ref.index)].mass;
        }
        if (inverse_mass == 0.0) continue;
        const auto& ga = geoms[static_cast<std::size_t>(contact_body_index(s, c.a))];
        const auto& gb = geoms[static_cast<std::size_t>(contact_body_index(s, c.b))];
        for (const NativeGeom& a : ga) {
          for (const NativeGeom& b : gb) {
            limited += set_pair(mjs_addPair(spec.get(), nullptr), rule, a, b, 1.0 / inverse_mass, s, c.friction, o.dt);
            ++pairs;
          }
        }
      }
      // The no-slip solver needs the dense constraint matrix, which the default
      // arena cannot hold when a body touches hundreds of pieces at once.
      spec->memory = (mjtSize{64} + pairs) << 20;
      if (limited > 0) {
        std::fprintf(stderr, "mujoco native: %s: stiffness limited by dt = %g s in %d of %d pairs\n",
                     s.name.c_str(), o.dt, limited, pairs);
      }
    }

    const std::unique_ptr<mjModel, ModelDeleter> model(mj_compile(spec.get(), nullptr));
    if (!model) throw std::runtime_error(std::string("mujoco: ") + mjs_getError(spec.get()));
    mjModel& m = *model;
    m.opt.timestep = o.dt;
    for (int k = 0; k < 3; ++k) m.opt.gravity[k] = s.gravity[k];
    // `default` keeps MuJoCo's options (Euler, pyramidal cone, impratio 1, no
    // no-slip pass); the rest use what its manual recommends.
    if (!native || rule == NativeModel::kSoft) {
      m.opt.integrator = mjINT_IMPLICITFAST;
      m.opt.cone = mjCONE_ELLIPTIC;
      m.opt.impratio = 10.0;
      m.opt.noslip_iterations = 3;
      m.opt.tolerance = kSolverTolerance;
      m.opt.noslip_tolerance = kNoslipTolerance;
    }
    // A numerical failure must end the run, not silently reset the state.
    m.opt.disableflags |= mjDSBL_AUTORESET;
    if (!native) m.opt.disableflags |= mjDSBL_CONTACT;

    const std::unique_ptr<mjData, DataDeleter> data(mj_makeData(&m));
    if (!data) throw std::runtime_error("mujoco: mj_makeData failed");
    mjData& d = *data;
    std::vector<FreeBody> bodies(nb);
    for (std::size_t i = 0; i < nb; ++i) {
      const BodySpec& b = s.bodies[i];
      FreeBody& f = bodies[i];
      f.id = mj_name2id(&m, mjOBJ_BODY, ("b" + std::to_string(i)).c_str());
      if (f.id < 0 || m.body_jntnum[f.id] != 1) throw std::runtime_error("mujoco: body without its free joint");
      const int joint = m.body_jntadr[f.id];
      f.qpos = m.jnt_qposadr[joint];
      f.qvel = m.jnt_dofadr[joint];
      const Eigen::Quaterniond q(b.X_W0.R);
      const Vec3 w_B = b.X_W0.R.transpose() * b.V_W0.w;
      const double qpos[7] = {b.X_W0.p.x(), b.X_W0.p.y(), b.X_W0.p.z(), q.w(), q.x(), q.y(), q.z()};
      const double qvel[6] = {b.V_W0.v.x(), b.V_W0.v.y(), b.V_W0.v.z(), w_B.x(), w_B.y(), w_B.z()};
      std::copy(qpos, qpos + 7, d.qpos + f.qpos);
      std::copy(qvel, qvel + 6, d.qvel + f.qvel);
    }

    // Volumetric contact: poses and twists in the contact model's body order
    // (dynamic bodies, then the static grounds).
    std::optional<ContactModel> contact;
    ContactModel::Evaluation eval;
    std::vector<Pose> poses(nb + s.grounds.size());
    std::vector<Twist> twists(poses.size());
    std::vector<FrictionState> friction;
    if (!native) {
      contact = make_contact_model(s, force_model(o.contact), o.dt);
      friction = contact->initial_friction();
      for (std::size_t g = 0; g < s.grounds.size(); ++g) poses[nb + g] = s.grounds[g].X_WG;
    }
    // The state is read from qpos and qvel: MuJoCo's derived quantities (xpos,
    // xmat, cvel) lag one step behind them after mj_step.
    const auto read_state = [&] {
      for (std::size_t i = 0; i < nb; ++i) {
        poses[i] = pose_of(d, bodies[i]);
        twists[i] = twist_of(d, bodies[i], poses[i]);
      }
    };

    const auto steps = std::llround(s.duration / o.dt);
    RunResult result{Trajectory(static_cast<int>(nb)), {}};
    const auto record = [&] {
      read_state();
      result.trajectory.push(d.time, poses, twists);
    };
    if (o.record) {
      result.trajectory.reserve(static_cast<std::size_t>(steps) + 1);
      record();
    }

    double contact_seconds = 0.0, contact_points = 0.0;
    std::int64_t contact_steps = 0;
    const auto start = std::chrono::steady_clock::now();
    for (std::int64_t k = 0; k < steps; ++k) {
      if (o.time_limit_seconds > 0.0 &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > o.time_limit_seconds) {
        throw RunTimeout(s.name + ": time limit of " + std::to_string(o.time_limit_seconds) + " s");
      }
      const double t = d.time;
      if (contact) {
        const auto t0 = std::chrono::steady_clock::now();
        read_state();
        contact->evaluate(poses, twists, friction, &eval);
        friction.swap(eval.next_friction);
        contact_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      }
      // xfrc_applied (force, then torque, at the centre of mass, in world) keeps
      // its value between steps: every body is rewritten.
      for (std::size_t i = 0; i < nb; ++i) {
        Wrench w;  // Volumetric wrench plus the applied force at the step's start time.
        if (contact) w = eval.wrenches[i];
        if (s.bodies[i].applied_force) w.force += s.bodies[i].applied_force(t);
        double* x = d.xfrc_applied + 6 * bodies[i].id;
        for (int j = 0; j < 3; ++j) x[j] = w.force[j], x[3 + j] = w.torque[j];
      }
      mj_step(&m, &d);
      if (d.ncon > 0) contact_points += d.ncon, ++contact_steps;
      for (int w = 0; w < mjNWARNING; ++w) {
        if (d.warning[w].number > 0) {
          throw std::runtime_error("mujoco: " + std::string(mju_warningText(w, d.warning[w].lastinfo)) + " (" + s.name +
                                   ", t = " + std::to_string(t) + " s)");
        }
      }
      if (o.record) record();
    }
    result.stats.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    result.stats.contact_seconds = contact_seconds;
    result.stats.steps = steps;
    if (contact_steps > 0) result.stats.native_contact_points = contact_points / static_cast<double>(contact_steps);
    return result;
  }
};

}  // namespace

std::unique_ptr<Engine> make_mujoco_engine() { return std::make_unique<MujocoEngine>(); }

}  // namespace volcontact
