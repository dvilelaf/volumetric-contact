#include "drake_backend/volumetric_contact_system.hpp"

#include <chrono>

#include <drake/math/rigid_transform.h>
#include <drake/multibody/math/spatial_algebra.h>

namespace volcontact::drake_backend {

using drake::math::RigidTransformd;
using drake::multibody::SpatialVelocity;
using drake::systems::Context;

VolumetricContactSystem::VolumetricContactSystem(ContactModel model, std::vector<drake::multibody::BodyIndex> bodies,
                                                 std::vector<Pose> ground_poses, double period)
    : model_(std::move(model)), bodies_(std::move(bodies)), ground_poses_(std::move(ground_poses)) {
  DeclareAbstractInputPort("body_poses", drake::Value<std::vector<RigidTransformd>>());
  DeclareAbstractInputPort("body_spatial_velocities", drake::Value<std::vector<SpatialVelocity<double>>>());

  const std::size_t num_pairs = model_.pairs().size();
  DeclareDiscreteState(static_cast<int>(kStateSize * num_pairs));
  DeclarePeriodicDiscreteUpdateEvent(period, 0.0, &VolumetricContactSystem::UpdateFriction);

  // Grounds are static: their poses are set once here and never touched again.
  Cache model_value;
  model_value.poses.resize(bodies_.size() + ground_poses_.size());
  model_value.twists.resize(model_value.poses.size());
  model_value.friction.resize(num_pairs);
  std::copy(ground_poses_.begin(), ground_poses_.end(), model_value.poses.begin() + bodies_.size());
  cache_index_ = DeclareCacheEntry("volumetric contact evaluation", model_value,
                                   &VolumetricContactSystem::CalcEvaluation, {all_input_ports_ticket(), xd_ticket()})
                     .cache_index();

  DeclareAbstractOutputPort("spatial_forces", &VolumetricContactSystem::CalcForces,
                            {cache_entry_ticket(cache_index_)});
}

const VolumetricContactSystem::Cache& VolumetricContactSystem::EvalCache(const Context<double>& context) const {
  return get_cache_entry(cache_index_).Eval<Cache>(context);
}

double VolumetricContactSystem::contact_seconds(const Context<double>& context) const {
  return get_cache_entry(cache_index_).get_cache_entry_value(context).PeekValueOrThrow<Cache>().seconds;
}

void VolumetricContactSystem::CalcEvaluation(const Context<double>& context, Cache* cache) const {
  const auto start = std::chrono::steady_clock::now();

  // Within a step Drake applies this system's friction update before the plant
  // reads the forces, which invalidates this entry. The second request comes
  // with the same body states and the friction state this evaluation produced;
  // it is recognized and served the forces of the state at the step's start.
  const auto& X_all = poses_input_port().Eval<std::vector<RigidTransformd>>(context);
  const auto& V_all = velocities_input_port().Eval<std::vector<SpatialVelocity<double>>>(context);
  const auto& xd = context.get_discrete_state_vector().value();
  auto& in = cache->inputs;
  in.clear();
  for (const drake::multibody::BodyIndex& b : bodies_) {
    const auto& X = X_all[b].GetAsMatrix34();
    in.insert(in.end(), X.data(), X.data() + 12);
    const auto& V = V_all[b].get_coeffs();
    in.insert(in.end(), V.data(), V.data() + 6);
  }
  if (in == cache->evaluated_inputs && (xd == cache->evaluated_state || xd == cache->next_state)) return;

  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    const RigidTransformd& X = X_all[bodies_[i]];
    cache->poses[i] = {X.rotation().matrix(), X.translation()};
    const SpatialVelocity<double>& V = V_all[bodies_[i]];
    cache->twists[i] = {V.rotational(), V.translational()};
  }
  for (std::size_t p = 0; p < cache->friction.size(); ++p) {
    cache->friction[p] = FrictionState::unpack(xd.data() + kStateSize * p);
  }

  model_.evaluate(cache->poses, cache->twists, cache->friction, &cache->eval);
  cache->evaluated_inputs.swap(in);
  cache->evaluated_state = xd;
  cache->next_state.resize(xd.size());
  for (std::size_t p = 0; p < cache->eval.next_friction.size(); ++p) {
    cache->eval.next_friction[p].pack(cache->next_state.data() + kStateSize * p);
  }
  cache->seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

void VolumetricContactSystem::CalcForces(const Context<double>& context, SpatialForces* forces) const {
  const Cache& cache = EvalCache(context);
  forces->resize(bodies_.size());
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    auto& f = (*forces)[i];
    f.body_index = bodies_[i];
    f.p_BoBq_B.setZero();
    // A model without pairs reports no wrenches.
    const Wrench w = i < cache.eval.wrenches.size() ? cache.eval.wrenches[i] : Wrench{};
    f.F_Bq_W = drake::multibody::SpatialForce<double>(w.torque, w.force);
  }
}

drake::systems::EventStatus VolumetricContactSystem::UpdateFriction(const Context<double>& context,
                                                                    drake::systems::DiscreteValues<double>* next) const {
  const Cache& cache = EvalCache(context);
  next->get_mutable_vector().SetFromVector(cache.next_state);
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace volcontact::drake_backend
