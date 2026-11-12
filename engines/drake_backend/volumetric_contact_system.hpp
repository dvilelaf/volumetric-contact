#pragma once

#include <vector>

#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>
#include <drake/systems/framework/leaf_system.h>

#include "volcontact/contact_model.hpp"

namespace volcontact::drake_backend {

// Evaluates the volumetric contact model from a MultibodyPlant's body poses and
// velocities and returns the wrenches as externally applied spatial forces.
//
// The evaluation lives in a cache entry, so the geometry is computed once per
// step and shared by the force output and the friction-state update. Friction
// state is discrete state updated with the same period as the plant, so both
// updates see the same pre-step context.
class VolumetricContactSystem final : public drake::systems::LeafSystem<double> {
 public:
  // `bodies[i]` is the plant body for model body i; model bodies after them are
  // static grounds with the given poses.
  VolumetricContactSystem(ContactModel model, std::vector<drake::multibody::BodyIndex> bodies,
                          std::vector<Pose> ground_poses, double period);

  [[nodiscard]] const drake::systems::InputPort<double>& poses_input_port() const { return get_input_port(0); }
  [[nodiscard]] const drake::systems::InputPort<double>& velocities_input_port() const { return get_input_port(1); }
  [[nodiscard]] const drake::systems::OutputPort<double>& forces_output_port() const { return get_output_port(0); }

  // Wall time spent evaluating the contact model in this context so far.
  [[nodiscard]] double contact_seconds(const drake::systems::Context<double>& context) const;

 private:
  struct Cache {
    ContactModel::Evaluation eval;
    // Inputs of `eval`, in contact-model body order (plant bodies, then grounds),
    // and the state they were read from.
    std::vector<Pose> poses;
    std::vector<Twist> twists;
    std::vector<FrictionState> friction;
    Eigen::VectorXd evaluated_state, next_state;  // Friction state read, and the one produced.
    std::vector<double> evaluated_inputs;
    std::vector<double> inputs;  // Scratch.
    double seconds = 0.0;        // Accumulated evaluation time (instrumentation).
  };

  using SpatialForces = std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>;

  void CalcEvaluation(const drake::systems::Context<double>& context, Cache* cache) const;
  void CalcForces(const drake::systems::Context<double>& context, SpatialForces* forces) const;
  drake::systems::EventStatus UpdateFriction(const drake::systems::Context<double>& context,
                                             drake::systems::DiscreteValues<double>* next) const;
  const Cache& EvalCache(const drake::systems::Context<double>& context) const;

  static constexpr int kStateSize = FrictionState::kSize;

  ContactModel model_;
  std::vector<drake::multibody::BodyIndex> bodies_;
  std::vector<Pose> ground_poses_;
  drake::systems::CacheIndex cache_index_;
};

}  // namespace volcontact::drake_backend
