#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <variant>
#include <vector>

#include "volcontact/detail/bvh.hpp"
#include "volcontact/contact_surface.hpp"
#include "volcontact/distributed.hpp"
#include "volcontact/gonthier.hpp"
#include "volcontact/intersection.hpp"
#include "volcontact/pcm.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// Force law of a pair: Gonthier's lumped volumetric model (the thesis), the
// distributed elastic-foundation model, or the per-face elastic foundation of the
// literature (PCM, Simbody), kept as a baseline. All use the same exact intersection.
using ContactLaw = std::variant<GonthierParams, DistributedParams, PcmParams>;

// Volumetric contact between pairs of rigid bodies. Engine agnostic: it only
// sees body poses and twists, indexed however the caller likes, and returns one
// wrench per body. Friction state is passed in and out explicitly so engines can
// keep it wherever their stepping scheme requires.
class ContactModel {
 public:
  struct Pair {
    int body_a = -1, body_b = -1;
    std::shared_ptr<const CollisionShape> shape_a, shape_b;  // In each body's frame.
    ContactLaw law;
  };

  struct PairResult {
    bool in_contact = false;
    IntersectionProperties geometry;
    ContactForce force;  // On A.
    // Explicit coupling: what the pair adds to the load accelerations of A and of B (StepLimits).
    double load_normal[2] = {0.0, 0.0}, load_tangent[2] = {0.0, 0.0};
  };

  // Inverse inertia of a body. All zero for a body that does not move.
  struct BodyInertia {
    double inv_mass = 0.0;
    Mat3 inv_inertia = Mat3::Zero();  // In the body frame, about its origin, which must be the centre of mass.
  };

  // Scratch memory of one thread.
  struct Workspace {
    IntersectionWorkspace geometry;
    ContactSurface surface;
    DistributedWorkspace distributed;
    PcmWorkspace pcm;
  };

  // Result of one evaluation. Reuse the same object across steps: its buffers
  // keep their capacity, so evaluating does not allocate after the first call.
  struct Evaluation {
    std::vector<PairResult> pairs;
    std::vector<Wrench> wrenches;              // Per body, about its origin, in world.
    std::vector<FrictionState> next_friction;  // Per pair.
    std::vector<Workspace> workspaces;         // One per thread.
    std::vector<std::uint32_t> candidates;     // Pairs that passed the broadphase, ascending.
    std::vector<double> load_normal, load_tangent;  // Per body, explicit coupling only.
  };

  // Throws if the distributed or the per-face law is requested for a non-convex
  // shape (split such bodies into convex pieces with CompoundShape, one pair per piece).
  void add_pair(Pair pair);

  // Pairs are evaluated in parallel (OpenMP) when there are at least this many.
  // Below it, thread start-up costs more than it saves.
  void set_parallel_threshold(int num_pairs) { parallel_threshold_ = num_pairs; }

  // Upper bound on the threads of one evaluation; 0 (default) means OpenMP's limit.
  void set_max_threads(int n) { max_threads_ = n; }

  // Tells the model that its forces are integrated explicitly with the step `dt` (the forces of
  // the state at the start of the step, as every backend of this repository does), on bodies
  // with these inertias, indexed like the poses. The laws then bound their stick spring and
  // damper and their normal damper to what that step can integrate (StepLimits); below those
  // bounds nothing changes. Without this call, or with dt = 0, the laws are evaluated as given.
  void set_explicit_coupling(double dt, std::vector<BodyInertia> bodies);

  [[nodiscard]] const std::vector<Pair>& pairs() const { return pairs_; }
  [[nodiscard]] int num_bodies() const { return num_bodies_; }
  [[nodiscard]] std::vector<FrictionState> initial_friction() const {
    return std::vector<FrictionState>(pairs_.size());
  }

  // Builds the broadphase structures. evaluate() does it on its first call after add_pair(), under a
  // lock, so concurrent first calls are safe; call it once after the last add_pair() to move that
  // cost out of the first step.
  void finalize() const;

  // `poses` and `twists` are indexed by body; `friction` by pair. Safe to call from several threads
  // on one model as long as each passes its own `out`.
  void evaluate(std::span<const Pose> poses, std::span<const Twist> twists, std::span<const FrictionState> friction,
                Evaluation* out) const;

 private:
  void evaluate_pair(std::size_t i, std::span<const Pose> poses, std::span<const Twist> twists,
                     const FrictionState& friction, Workspace& ws, PairResult* result, FrictionState* next) const;

  // Pairs between the same two bodies. Large groups get a tree over the boxes of
  // their B shapes (in B's frame) so a step only visits pairs near body A.
  struct Group {
    int body_a = -1, body_b = -1;
    std::vector<std::uint32_t> pairs;
    std::vector<Aabb> boxes_a, boxes_b;  // Per entry of `pairs`, in each body's frame.
    Aabb bounds_a;
    Vec3 center_a;  // Ball in frame A that contains every A shape of the group.
    double radius_a = 0.0;
    std::shared_ptr<const Bvh> tree;  // Null for small groups, whose pairs are always candidates.
  };
  void build_groups() const;
  void find_candidates(std::span<const Pose> poses, std::vector<std::uint32_t>* out) const;

  // Copyable atomic flag (the model is returned and stored by value).
  struct Flag {
    std::atomic<bool> value{true};
    Flag() = default;
    Flag(const Flag& o) : value(o.value.load()) {}
    Flag& operator=(const Flag& o) {
      value = o.value.load();
      return *this;
    }
  };
  // Built by finalize(), on the first evaluate() after add_pair().
  mutable std::vector<Group> groups_;
  mutable Flag groups_dirty_;
  std::shared_ptr<std::mutex> groups_mutex_ = std::make_shared<std::mutex>();
  std::vector<Pair> pairs_;
  int num_bodies_ = 0;
  double dt_ = 0.0;  // Explicit coupling; 0 for none.
  std::vector<BodyInertia> inertias_;
  int parallel_threshold_ = 8;
  int max_threads_ = 0;
};

}  // namespace volcontact
