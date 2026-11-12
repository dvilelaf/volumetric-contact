#include "drake_backend/drake_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>

#include <drake/common/memory_file.h>
#include <drake/geometry/collision_filter_declaration.h>
#include <drake/geometry/in_memory_mesh.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/multibody/plant/contact_results.h>
#include <drake/multibody/plant/externally_applied_spatial_force_multiplexer.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include "drake_backend/native_materials.hpp"
#include "native_geometry.hpp"
#include "drake_backend/volumetric_contact_system.hpp"

namespace volcontact {
namespace {

using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::systems::Context;

RigidTransformd to_drake(const Pose& X) { return RigidTransformd(RotationMatrixd(X.R), X.p); }

Pose from_drake(const RigidTransformd& X) { return {X.rotation().matrix(), X.translation()}; }

drake::multibody::SpatialInertia<double> spatial_inertia(const BodySpec& b) {
  const Mat3& I = b.inertia;
  return drake::multibody::SpatialInertia<double>::MakeFromCentralInertia(
      b.mass, Vec3::Zero(),
      drake::multibody::RotationalInertia<double>(I(0, 0), I(1, 1), I(2, 2), I(0, 1), I(0, 2), I(1, 2)));
}

// H of a mesh asset: the foundation depth in the title line of its .vtk.
double foundation_depth(const std::filesystem::path& vtk) {
  std::ifstream in(vtk);
  std::string line;
  std::getline(in, line);
  std::getline(in, line);
  const std::string tag = "foundation_depth=";
  const auto at = line.find(tag);
  if (at == std::string::npos) throw std::runtime_error("no foundation_depth in " + vtk.string());
  return std::stod(line.substr(at + tag.size()));
}

const std::filesystem::path& volume_mesh(const ShapeMeshes& shape) {
  if (!shape.volume) throw std::runtime_error("native contact needs the .vtk of " + shape.surface.string());
  return *shape.volume;
}

// Rigid hydroelastic surface of a mesh asset; refined, every edge is at most
// 1/kRigidEdgeDivisor of the mesh's largest side. Drake turns each polygon of a
// contact surface into one contact point, and a half-space adds no polygons of
// its own, so a 12-triangle box on it would stand on two points; a tetrahedral
// partner refines the surface by itself (docs/scenarios.md §3).
constexpr double kRigidEdgeDivisor = 8.0;
// Radius of the spheres on a box's corners in point contact [m], as in Drake's box models.
constexpr double kCornerSphereRadius = 1e-7;

drake::geometry::Mesh rigid_surface(const std::filesystem::path& obj, bool refine) {
  if (!refine) return drake::geometry::Mesh(obj.string());
  const TriMesh surface = load_obj(obj);
  const TriMesh fine = native::refined(surface, native::largest_extent(surface) / kRigidEdgeDivisor);
  return drake::geometry::Mesh(
      drake::geometry::InMemoryMesh{drake::MemoryFile(native::to_obj(fine), ".obj", obj.string())});
}

// Drake's native contact, docs/scenarios.md §3: `hydroelastic` or `point`.
void add_native_contact(MultibodyPlant<double>& plant, drake::geometry::SceneGraph<double>& scene_graph,
                        const Scenario& s, const RunOptions& o, const std::string& model,
                        const std::vector<const RigidBody<double>*>& bodies) {
  namespace geo = drake::geometry;
  constexpr double kSlab = 1.0;
  const bool point = model == "point";
  const double K = s.native.stiffness;
  const drake_backend::NativeMaterials materials = drake_backend::solve_native_materials(s);
  const auto friction = [&](std::size_t node) {
    return drake::multibody::CoulombFriction<double>(materials.friction[node], materials.friction[node]);
  };

  // Geometry only for what takes part in a contact.
  std::vector<bool> in_contact(bodies.size() + s.grounds.size(), false);
  for (const ContactSpec& c : s.contacts) {
    in_contact[contact_body_index(s, c.a)] = true;
    in_contact[contact_body_index(s, c.b)] = true;
  }

  // Bodies that touch a half-space, the one partner that does not refine the contact surface.
  std::vector<bool> on_half_space(bodies.size(), false);
  for (const ContactSpec& c : s.contacts) {
    for (const auto& [body, other] : {std::pair{c.a, c.b}, std::pair{c.b, c.a}}) {
      if (body.kind == BodyRef::Kind::kBody && other.kind == BodyRef::Kind::kGround && s.grounds[other.index].planar) {
        on_half_space[body.index] = true;
      }
    }
  }

  std::vector<geo::GeometrySet> sets(in_contact.size());
  const auto keep = [&](std::size_t node, geo::GeometryId id) { sets[node].Add(id); };

  // Point contact: Hunt–Crossley d = h and a point stiffness K·A of each
  // geometry, A its reference area; Drake combines two stiffnesses in series.
  const auto point_properties = [&](std::size_t node, const ShapeMeshes& shape) {
    geo::ProximityProperties props;
    geo::AddContactMaterial(s.native.damping, K * native::reference_area(load_obj(shape.surface)), friction(node),
                            &props);
    return props;
  };
  // Point contact acts between convex shapes: the convex pieces of an asset.
  const auto add_pieces = [&](std::size_t node, const RigidBody<double>& body, const RigidTransformd& X,
                              const ShapeMeshes& shape, const std::string& name) {
    const geo::ProximityProperties props = point_properties(node, shape);
    for (std::size_t k = 0; k < shape.pieces.size(); ++k) {
      keep(node, plant.RegisterCollisionGeometry(body, X, geo::Convex(shape.pieces[k].string()),
                                                 name + "_" + std::to_string(k), props));
    }
  };

  for (std::size_t i = 0; i < s.grounds.size(); ++i) {
    const std::size_t node = bodies.size() + i;
    if (!in_contact[node]) continue;
    const GroundSpec& g = s.grounds[i];
    const std::string name = "ground_" + std::to_string(i);
    if (point) {
      if (g.planar) {
        keep(node, plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(), geo::HalfSpace(), name,
                                                   point_properties(node, g.shape)));
      } else {
        add_pieces(node, plant.world_body(), to_drake(g.X_WG), g.shape, name);
      }
      continue;
    }
    geo::ProximityProperties props;
    geo::AddContactMaterial(s.native.damping, {}, friction(node), &props);
    if (g.planar) {
      geo::AddCompliantHydroelasticPropertiesForHalfSpace(kSlab, K * kSlab, &props);
      keep(node,
           plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(), geo::HalfSpace(), name, props));
    } else {
      const auto& vtk = volume_mesh(g.shape);
      geo::AddCompliantHydroelasticProperties(1.0, K * foundation_depth(vtk), &props);
      keep(node, plant.RegisterCollisionGeometry(plant.world_body(), to_drake(g.X_WG), geo::Mesh(vtk.string()), name,
                                                 props));
    }
  }

  for (std::size_t i = 0; i < bodies.size(); ++i) {
    if (!in_contact[i]) continue;
    const BodySpec& b = s.bodies[i];
    const bool compliant = materials.compliant[i];
    // The analytic shape: always for point contact (exact and convex), on request for hydroelastic.
    std::unique_ptr<geo::Shape> shape;
    double depth = 0.0;
    if (b.primitive && (point || o.native_primitives)) {
      const Primitive& p = *b.primitive;
      if (p.kind == Primitive::Kind::kBox) {
        depth = 0.5 * p.size.minCoeff();
        shape = std::make_unique<geo::Box>(p.size.x(), p.size.y(), p.size.z());
      } else if (p.kind == Primitive::Kind::kCylinder) {
        depth = std::min(p.size.x(), 0.5 * p.size.y());
        shape = std::make_unique<geo::Cylinder>(p.size.x(), p.size.y());
      } else {
        depth = p.size.x();
        shape = std::make_unique<geo::Sphere>(p.size.x());
      }
    }
    if (point) {
      if (shape) {
        keep(i, plant.RegisterCollisionGeometry(*bodies[i], to_drake(b.primitive->X_BG), *shape, b.name,
                                                point_properties(i, b.shape)));
        // One point per pair cannot hold a face: a box also gets a tiny sphere on each
        // corner, as Drake's own box models do (docs/scenarios.md §3).
        if (b.primitive->kind == Primitive::Kind::kBox) {
          for (int c = 0; c < 8; ++c) {
            const Vec3 corner = 0.5 * Vec3(c & 1 ? 1.0 : -1.0, c & 2 ? 1.0 : -1.0, c & 4 ? 1.0 : -1.0)
                                          .cwiseProduct(b.primitive->size);
            keep(i, plant.RegisterCollisionGeometry(*bodies[i], to_drake(b.primitive->X_BG) * RigidTransformd(corner),
                                                    geo::Sphere(kCornerSphereRadius),
                                                    b.name + "_corner" + std::to_string(c),
                                                    point_properties(i, b.shape)));
          }
        }
      } else {
        add_pieces(i, *bodies[i], RigidTransformd(), b.shape, b.name);
      }
      continue;
    }
    geo::ProximityProperties props;
    geo::AddContactMaterial(s.native.damping, {}, friction(i), &props);
    if (shape) {
      if (compliant) {
        geo::AddCompliantHydroelasticProperties(o.native_resolution, K * depth, &props);
      } else {
        geo::AddRigidHydroelasticProperties(o.native_resolution, &props);
      }
      keep(i, plant.RegisterCollisionGeometry(*bodies[i], to_drake(b.primitive->X_BG), *shape, b.name, props));
    } else if (compliant) {
      const auto& vtk = volume_mesh(b.shape);
      geo::AddCompliantHydroelasticProperties(1.0, K * foundation_depth(vtk), &props);
      keep(i, plant.RegisterCollisionGeometry(*bodies[i], RigidTransformd(), geo::Mesh(vtk.string()), b.name,
                                              props));
    } else {
      geo::AddRigidHydroelasticProperties(&props);
      keep(i, plant.RegisterCollisionGeometry(*bodies[i], RigidTransformd(), rigid_surface(b.shape.surface, on_half_space[i]), b.name, props));
    }
  }

  // Only the pairs a ContactSpec declares may collide: Drake throws on a rigid–rigid pair.
  const std::size_t n = in_contact.size();
  std::vector<std::vector<bool>> declared(n, std::vector<bool>(n, false));
  for (const ContactSpec& c : s.contacts) {
    declared[contact_body_index(s, c.a)][contact_body_index(s, c.b)] = true;
    declared[contact_body_index(s, c.b)][contact_body_index(s, c.a)] = true;
  }
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      if (in_contact[i] && in_contact[j] && !declared[i][j]) {
        scene_graph.collision_filter_manager().Apply(
            geo::CollisionFilterDeclaration().ExcludeBetween(sets[i], sets[j]));
      }
    }
  }
  plant.set_contact_model(point ? drake::multibody::ContactModel::kPoint
                                : drake::multibody::ContactModel::kHydroelastic);
  plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kLagged);
}

// World forces at the centre of mass of the bodies with a `BodySpec::applied_force`.
class AppliedForceSource final : public drake::systems::LeafSystem<double> {
 public:
  using SpatialForces = std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>;

  AppliedForceSource(const Scenario& s, const std::vector<drake::multibody::BodyIndex>& bodies) {
    for (std::size_t i = 0; i < bodies.size(); ++i) {
      if (s.bodies[i].applied_force) sources_.emplace_back(bodies[i], s.bodies[i].applied_force);
    }
    DeclareAbstractOutputPort("applied_forces", &AppliedForceSource::Calc);
  }

  [[nodiscard]] bool empty() const { return sources_.empty(); }

 private:
  void Calc(const drake::systems::Context<double>& context, SpatialForces* forces) const {
    forces->clear();
    for (const auto& [body, force] : sources_) {
      drake::multibody::ExternallyAppliedSpatialForce<double> f;
      f.body_index = body;
      f.p_BoBq_B.setZero();
      f.F_Bq_W = drake::multibody::SpatialForce<double>(Vec3::Zero(), force(context.get_time()));
      forces->push_back(f);
    }
  }

  std::vector<std::pair<drake::multibody::BodyIndex, std::function<Vec3(double)>>> sources_;
};

class DrakeEngine final : public Engine {
 public:
  [[nodiscard]] std::string_view name() const override { return "drake"; }
  [[nodiscard]] std::vector<std::string> native_models() const override { return {"hydroelastic", "point"}; }

  [[nodiscard]] RunResult run(const Scenario& s, const RunOptions& o) const override {
    const std::string model = o.contact == ContactMethod::kNative ? native_model(*this, o) : "";
    drake::systems::DiagramBuilder<double> builder;
    // SceneGraph is only needed by Drake's own contact model.
    MultibodyPlant<double>* plant = nullptr;
    drake::geometry::SceneGraph<double>* scene_graph = nullptr;
    if (o.contact == ContactMethod::kNative) {
      auto added = drake::multibody::AddMultibodyPlantSceneGraph(&builder, o.dt);
      plant = &added.plant;
      scene_graph = &added.scene_graph;
    } else {
      plant = builder.AddSystem<MultibodyPlant<double>>(o.dt);
    }
    plant->mutable_gravity_field().set_gravity_vector(s.gravity);

    std::vector<const RigidBody<double>*> bodies;
    std::vector<drake::multibody::BodyIndex> indices;
    for (const BodySpec& b : s.bodies) {
      bodies.push_back(&plant->AddRigidBody(b.name, spatial_inertia(b)));
      indices.push_back(bodies.back()->index());
    }
    if (o.contact == ContactMethod::kNative) add_native_contact(*plant, *scene_graph, s, o, model, bodies);
    plant->Finalize();

    drake_backend::VolumetricContactSystem* contact = nullptr;
    auto* applied = builder.AddSystem<AppliedForceSource>(s, indices);
    const auto& plant_forces = plant->get_applied_spatial_force_input_port();
    if (o.contact != ContactMethod::kNative) {
      std::vector<Pose> ground_poses;
      for (const GroundSpec& g : s.grounds) ground_poses.push_back(g.X_WG);
      contact = builder.AddSystem<drake_backend::VolumetricContactSystem>(
          make_contact_model(s, force_model(o.contact), o.dt), indices, std::move(ground_poses), o.dt);
      builder.Connect(plant->get_body_poses_output_port(), contact->poses_input_port());
      builder.Connect(plant->get_body_spatial_velocities_output_port(), contact->velocities_input_port());
      if (applied->empty()) {
        builder.Connect(contact->forces_output_port(), plant_forces);
      } else {
        const auto* mux = builder.AddSystem<drake::multibody::ExternallyAppliedSpatialForceMultiplexer>(2);
        builder.Connect(contact->forces_output_port(), mux->get_input_port(0));
        builder.Connect(applied->get_output_port(), mux->get_input_port(1));
        builder.Connect(mux->get_output_port(), plant_forces);
      }
    } else if (!applied->empty()) {
      builder.Connect(applied->get_output_port(), plant_forces);
    }

    const auto diagram = builder.Build();
    drake::systems::Simulator<double> simulator(*diagram);
    auto& plant_context = plant->GetMyMutableContextFromRoot(&simulator.get_mutable_context());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
      const BodySpec& b = s.bodies[i];
      plant->SetFreeBodyPose(&plant_context, *bodies[i], to_drake(b.X_W0));
      plant->SetFreeBodySpatialVelocity(&plant_context, *bodies[i],
                                        drake::multibody::SpatialVelocity<double>(b.V_W0.w, b.V_W0.v));
    }

    RunResult result{Trajectory(static_cast<int>(bodies.size())), {}};
    std::vector<Pose> X(bodies.size());
    std::vector<Twist> V(bodies.size());
    double contact_points = 0.0;
    std::int64_t contact_steps = 0;
    if (o.record) {
      result.trajectory.reserve(static_cast<std::size_t>(std::llround(s.duration / o.dt)) + 2);
      simulator.set_monitor([&](const Context<double>& root) {
        const auto& context = plant->GetMyContextFromRoot(root);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
          X[i] = from_drake(plant->EvalBodyPoseInWorld(context, *bodies[i]));
          const auto& Vd = plant->EvalBodySpatialVelocityInWorld(context, *bodies[i]);
          V[i] = {Vd.rotational(), Vd.translational()};
        }
        result.trajectory.push(root.get_time(), X, V);
        if (o.contact == ContactMethod::kNative) {
          const auto& contacts =
              plant->get_contact_results_output_port().Eval<drake::multibody::ContactResults<double>>(context);
          int points = contacts.num_point_pair_contacts();
          for (int k = 0; k < contacts.num_hydroelastic_contacts(); ++k) {
            points += contacts.hydroelastic_contact_info(k).contact_surface().num_faces();
          }
          contact_points += points;
          contact_steps += points > 0 ? 1 : 0;
        }
        return drake::systems::EventStatus::Succeeded();
      });
    }

    simulator.Initialize();
    const auto& root = simulator.get_context();
    const auto contact_seconds = [&] {
      return contact ? contact->contact_seconds(contact->GetMyContextFromRoot(root)) : 0.0;
    };
    const double contact_before = contact_seconds();
    const std::int64_t updates_before = simulator.get_num_discrete_updates();
    const auto start = std::chrono::steady_clock::now();
    if (o.time_limit_seconds > 0.0) {
      // In chunks of 100 steps, to look at the clock; the plant is discrete, so the result is the same.
      const double chunk = 100.0 * o.dt;
      for (std::int64_t k = 1;; ++k) {
        const double target = std::min(s.duration, static_cast<double>(k) * chunk);
        simulator.AdvanceTo(target);
        if (target >= s.duration) break;
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > o.time_limit_seconds) {
          throw RunTimeout(s.name + ": time limit of " + std::to_string(o.time_limit_seconds) + " s");
        }
      }
    } else {
      simulator.AdvanceTo(s.duration);
    }
    result.stats.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    result.stats.contact_seconds = contact_seconds() - contact_before;
    result.stats.steps = simulator.get_num_discrete_updates() - updates_before;
    if (contact_steps > 0) result.stats.native_contact_points = contact_points / static_cast<double>(contact_steps);
    return result;
  }
};

}  // namespace

std::unique_ptr<Engine> make_drake_engine() { return std::make_unique<DrakeEngine>(); }

}  // namespace volcontact
