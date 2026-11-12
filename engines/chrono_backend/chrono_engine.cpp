#include "chrono_backend/chrono_engine.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <utility>

#include <chrono/collision/ChCollisionModel.h>
#include <chrono/collision/ChCollisionShapeBox.h>
#include <chrono/collision/ChCollisionShapeConvexHull.h>
#include <chrono/collision/ChCollisionShapeCylinder.h>
#include <chrono/collision/ChCollisionShapeSphere.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChContactContainer.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChContactMaterialSMC.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChSolverBB.h>

#include "native_geometry.hpp"

namespace volcontact {
namespace {

using chrono::ChMatrix33;
using chrono::ChVector3d;

ChVector3d to_chrono(const Vec3& v) { return {v.x(), v.y(), v.z()}; }
Vec3 from_chrono(const ChVector3d& v) { return {v.x(), v.y(), v.z()}; }

ChMatrix33<> to_chrono(const Mat3& m) {
  ChMatrix33<> r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) r(i, j) = m(i, j);
  }
  return r;
}

Mat3 from_chrono(const ChMatrix33<>& m) {
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) r(i, j) = m(i, j);
  }
  return r;
}

Pose pose_of(const chrono::ChBody& b) { return {from_chrono(b.GetRotMat()), from_chrono(b.GetPos())}; }
Twist twist_of(const chrono::ChBody& b) { return {from_chrono(b.GetAngVelParent()), from_chrono(b.GetPosDt())}; }

chrono::ChFramed to_chrono(const Pose& X) { return chrono::ChFramed(to_chrono(X.p), to_chrono(X.R)); }

// Native collision shapes of an asset, docs/scenarios.md §3: its analytic
// primitive if the scenario gives one; otherwise its convex pieces, a box for
// a piece that is one and a convex hull for the rest.
void add_shapes(chrono::ChBody& body, const ShapeMeshes& shape, const std::optional<Primitive>& primitive,
                const std::shared_ptr<chrono::ChContactMaterial>& material) {
  if (primitive) {
    const Vec3& size = primitive->size;
    std::shared_ptr<chrono::ChCollisionShape> prim;
    if (primitive->kind == Primitive::Kind::kBox) {
      prim = chrono_types::make_shared<chrono::ChCollisionShapeBox>(material, size.x(), size.y(), size.z());
    } else if (primitive->kind == Primitive::Kind::kCylinder) {
      prim = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(material, size.x(), size.y());
    } else {
      prim = chrono_types::make_shared<chrono::ChCollisionShapeSphere>(material, size.x());
    }
    body.AddCollisionShape(prim, to_chrono(primitive->X_BG));
  } else {
    for (const auto& path : shape.pieces) {
      const TriMesh mesh = load_obj(path);
      if (const auto box = native::as_box(mesh)) {
        body.AddCollisionShape(
            chrono_types::make_shared<chrono::ChCollisionShapeBox>(material, box->size.x(), box->size.y(), box->size.z()),
            to_chrono(box->X));
      } else {
        std::vector<ChVector3d> points;
        points.reserve(mesh.vertices.size());
        for (const Vec3& v : mesh.vertices) points.push_back(to_chrono(v));
        body.AddCollisionShape(chrono_types::make_shared<chrono::ChCollisionShapeConvexHull>(material, points));
      }
    }
  }
  body.EnableCollision(true);
}

// Coefficients of one pair of bodies in Chrono's own contact.
struct PairCoefficients {
  float mu = 0.0f;
  float stiffness = 0.0f;  // SMC: kn = kt [N/m].
  float damping = 0.0f;    // SMC: gn [1/s], which Chrono multiplies by the pair's effective mass.
  float stick = 0.0f;      // SMC: gt [1/s], likewise.
};

// Sets each contact's coefficients from the ContactSpec of its pair of bodies
// (numbered as the contact model does); other pairs keep the material's.
class PairMaterials final : public chrono::ChContactContainer::AddContactCallback {
 public:
  explicit PairMaterials(bool smooth) : smooth_(smooth) {}
  void add_model(const chrono::ChCollisionModel* model, int index) { index_[model] = index; }
  void add_pair(int a, int b, const PairCoefficients& c) { pairs_[{std::min(a, b), std::max(a, b)}] = c; }

  void OnAddContact(const chrono::ChCollisionInfo& info, chrono::ChContactMaterialComposite* const material) override {
    const auto a = index_.find(info.modelA), b = index_.find(info.modelB);
    if (a == index_.end() || b == index_.end()) return;
    const auto pair = pairs_.find({std::min(a->second, b->second), std::max(a->second, b->second)});
    if (pair == pairs_.end()) return;
    const PairCoefficients& c = pair->second;
    if (smooth_) {
      auto* m = static_cast<chrono::ChContactMaterialCompositeSMC*>(material);
      m->mu_eff = c.mu;
      m->kn = m->kt = c.stiffness;
      m->gn = c.damping;
      m->gt = c.stick;
    } else {
      auto* m = static_cast<chrono::ChContactMaterialCompositeNSC*>(material);
      m->static_friction = m->sliding_friction = c.mu;
    }
  }

 private:
  bool smooth_;
  std::map<const chrono::ChCollisionModel*, int> index_;
  std::map<std::pair<int, int>, PairCoefficients> pairs_;
};

// SMC, docs/scenarios.md §3: Hooke's law with explicit coefficients.
// Stiffness: a full face contact has four manifold points, so each point
// carries a quarter of the reference stiffness K·A, A the smaller reference
// area of the two bodies. Damping: the Hunt–Crossley term K·A·δ·h·δ̇ linearized
// at the static penetration under the body's weight (K·A·δ = m·g), a force
// m·h·|g|·δ̇.
constexpr double kManifoldPoints = 4.0;
constexpr double kFixedMass = 1e12;  // Mass of a fixed body in the pair's effective mass [kg].
// Largest ω·Δt of the contact integration, ω² = K·A / m_eff. Chrono's SMC
// forces enter explicitly (its Jacobians are disabled in Chrono 10), so the step
// is subdivided until each contact oscillation has at least 2π/0.1 ≈ 60 steps.
constexpr double kSmcPhasePerStep = 0.1;
// Chrono 10's tangential force is viscous below the Coulomb limit (its
// multi-step displacement is not implemented): (kt·Δt + gt·m_eff)·v per point.
// gt is set so that a full manifold takes this fraction of the pair's momentum
// per sub-step, the stiffest stick an explicit step integrates without ringing.
constexpr double kSmcStickPerStep = 0.25;
constexpr int kNscIterations = 200;

class ChronoEngine final : public Engine {
 public:
  [[nodiscard]] std::string_view name() const override { return "chrono"; }
  [[nodiscard]] std::vector<std::string> native_models() const override { return {"smc", "nsc"}; }

  [[nodiscard]] RunResult run(const Scenario& s, const RunOptions& o) const override {
    const bool native = o.contact == ContactMethod::kNative;
    const std::string native_name = native ? native_model(*this, o) : "";
    const bool smooth = native_name != "nsc";
    if (native) {
      // Chrono's defaults (3 cm envelope, 1 cm margin) suit metre-sized bodies;
      // these are the values of its collision manual.
      chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(1e-3);
      chrono::ChCollisionModel::SetDefaultSuggestedMargin(5e-4);
    }

    std::unique_ptr<chrono::ChSystem> system;
    if (smooth) system = std::make_unique<chrono::ChSystemSMC>();
    else system = std::make_unique<chrono::ChSystemNSC>();
    chrono::ChSystem& sys = *system;
    sys.SetGravitationalAcceleration(to_chrono(s.gravity));
    sys.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    std::shared_ptr<chrono::ChContactMaterial> material;
    if (native) {
      sys.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
      if (smooth) {
        auto& smc = static_cast<chrono::ChSystemSMC&>(sys);
        smc.SetContactForceModel(chrono::ChSystemSMC::Hooke);
        smc.UseMaterialProperties(false);
        material = chrono_types::make_shared<chrono::ChContactMaterialSMC>();
      } else {
        // Complementarity contact needs a solver for variational inequalities;
        // Barzilai–Borwein is the one Chrono's manual advises when accuracy matters.
        auto solver = chrono_types::make_shared<chrono::ChSolverBB>();
        solver->SetMaxIterations(kNscIterations);
        solver->SetTolerance(1e-10);
        sys.SetSolver(solver);
        material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
      }
    }

    std::vector<std::shared_ptr<chrono::ChBody>> bodies;
    std::vector<unsigned int> accumulators;
    for (const BodySpec& spec : s.bodies) {
      auto body = chrono_types::make_shared<chrono::ChBody>();
      body->SetMass(spec.mass);
      body->SetInertia(to_chrono(spec.inertia));
      body->SetPos(to_chrono(spec.X_W0.p));
      body->SetRot(to_chrono(spec.X_W0.R));
      body->SetPosDt(to_chrono(spec.V_W0.v));
      body->SetAngVelParent(to_chrono(spec.V_W0.w));
      if (native) add_shapes(*body, spec.shape, spec.primitive, material);
      accumulators.push_back(body->AddAccumulator());
      sys.AddBody(body);
      bodies.push_back(std::move(body));
    }
    const std::size_t nb = bodies.size();
    int substeps = 1;
    if (native) {
      std::vector<std::shared_ptr<chrono::ChBody>> grounds;
      for (const GroundSpec& g : s.grounds) {
        auto ground = chrono_types::make_shared<chrono::ChBody>();
        ground->SetFixed(true);
        // SMC damping scales with the pair's effective mass m₁m₂/(m₁+m₂): a
        // fixed body must not enter it with ChBody's default 1 kg.
        ground->SetMass(kFixedMass);
        ground->SetPos(to_chrono(g.X_WG.p));
        ground->SetRot(to_chrono(g.X_WG.R));
        add_shapes(*ground, g.shape, std::nullopt, material);
        sys.AddBody(ground);
        grounds.push_back(std::move(ground));
      }
      auto pairs = std::make_shared<PairMaterials>(smooth);
      for (std::size_t i = 0; i < nb; ++i) pairs->add_model(bodies[i]->GetCollisionModel().get(), static_cast<int>(i));
      for (std::size_t g = 0; g < grounds.size(); ++g)
        pairs->add_model(grounds[g]->GetCollisionModel().get(), static_cast<int>(nb + g));
      const auto area = [&](BodyRef r) {
        const ShapeMeshes& shape = r.kind == BodyRef::Kind::kBody ? s.bodies[r.index].shape : s.grounds[r.index].shape;
        return native::reference_area(load_obj(shape.surface));
      };
      const auto mass = [&](BodyRef r) { return r.kind == BodyRef::Kind::kBody ? s.bodies[r.index].mass : kFixedMass; };
      double omega = 0.0;  // Fastest contact oscillation, all the manifold's points loaded.
      for (const ContactSpec& c : s.contacts) {
        const double m_eff = mass(c.a) * mass(c.b) / (mass(c.a) + mass(c.b));
        omega = std::max(omega, std::sqrt(s.native.stiffness * std::min(area(c.a), area(c.b)) / m_eff));
      }
      if (smooth) substeps = std::max(1, static_cast<int>(std::ceil(omega * o.dt / kSmcPhasePerStep)));
      const double stick = kSmcStickPerStep / (kManifoldPoints * o.dt / substeps);
      for (const ContactSpec& c : s.contacts) {
        const double stiffness = s.native.stiffness * std::min(area(c.a), area(c.b));
        pairs->add_pair(contact_body_index(s, c.a), contact_body_index(s, c.b),
                        {.mu = static_cast<float>(c.friction),
                         .stiffness = static_cast<float>(stiffness / kManifoldPoints),
                         .damping = static_cast<float>(s.native.damping * s.gravity.norm()),
                         .stick = static_cast<float>(stick)});
      }
      sys.GetContactContainer()->RegisterAddContactCallback(pairs);
    }

    // Volumetric contact: poses and twists in the contact model's body order
    // (dynamic bodies, then the static grounds).
    std::optional<ContactModel> model;
    ContactModel::Evaluation eval;
    std::vector<Pose> poses(nb + s.grounds.size());
    std::vector<Twist> twists(poses.size());
    std::vector<FrictionState> friction;
    if (!native) {
      model = make_contact_model(s, force_model(o.contact), o.dt);
      friction = model->initial_friction();
      for (std::size_t g = 0; g < s.grounds.size(); ++g) poses[nb + g] = s.grounds[g].X_WG;
    }

    const auto steps = std::llround(s.duration / o.dt);
    RunResult result{Trajectory(static_cast<int>(nb)), {}};
    const auto record = [&] {
      for (std::size_t i = 0; i < nb; ++i) {
        poses[i] = pose_of(*bodies[i]);
        twists[i] = twist_of(*bodies[i]);
      }
      result.trajectory.push(sys.GetChTime(), poses, twists);
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
      const double t = sys.GetChTime();
      if (model) {
        const auto t0 = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < nb; ++i) {
          poses[i] = pose_of(*bodies[i]);
          twists[i] = twist_of(*bodies[i]);
        }
        model->evaluate(poses, twists, friction, &eval);
        friction.swap(eval.next_friction);
        contact_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      }
      for (std::size_t i = 0; i < nb; ++i) {
        if (!model && !s.bodies[i].applied_force) continue;
        chrono::ChBody& b = *bodies[i];
        Wrench w;  // Volumetric wrench plus the applied force at the step's start time.
        if (model) w = eval.wrenches[i];
        if (s.bodies[i].applied_force) w.force += s.bodies[i].applied_force(t);
        b.EmptyAccumulator(accumulators[i]);
        b.AccumulateForce(accumulators[i], to_chrono(w.force), b.GetPos(), false);
        b.AccumulateTorque(accumulators[i], to_chrono(w.torque), false);
        // Chrono gathers accumulators when it updates the body, which last
        // happened at the end of the previous step; without this the force
        // would act one step late.
        b.UpdateForces(t, chrono::UpdateFlags::DYNAMICS);
      }
      if (substeps == 1) {
        sys.DoStepDynamics(o.dt);
      } else {
        // Native SMC: the applied forces are refreshed at every sub-step.
        const double h = o.dt / substeps;
        for (int sub = 0; sub < substeps; ++sub) {
          if (sub > 0) {
            for (std::size_t i = 0; i < nb; ++i) {
              if (!s.bodies[i].applied_force) continue;
              chrono::ChBody& b = *bodies[i];
              b.EmptyAccumulator(accumulators[i]);
              b.AccumulateForce(accumulators[i], to_chrono(s.bodies[i].applied_force(t + sub * h)), b.GetPos(), false);
              b.UpdateForces(t + sub * h, chrono::UpdateFlags::DYNAMICS);
            }
          }
          sys.DoStepDynamics(h);
        }
      }
      if (native) {
        const unsigned int points = sys.GetNumContacts();
        contact_points += points;
        contact_steps += points > 0 ? 1 : 0;
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

std::unique_ptr<Engine> make_chrono_engine() { return std::make_unique<ChronoEngine>(); }

}  // namespace volcontact
