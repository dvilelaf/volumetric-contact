#include "volcontact/contact_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#ifdef VOLCONTACT_OPENMP
#include <omp.h>
#endif

namespace volcontact {

void ContactModel::add_pair(Pair pair) {
  if (pair.body_a < 0 || pair.body_b < 0 || pair.body_a == pair.body_b || !pair.shape_a || !pair.shape_b) {
    throw std::invalid_argument("ContactModel::add_pair: invalid pair");
  }
  if (!std::holds_alternative<GonthierParams>(pair.law) && (!pair.shape_a->convex() || !pair.shape_b->convex())) {
    throw std::invalid_argument("ContactModel::add_pair: the distributed and per-face laws need convex shapes; "
                                "decompose the body into convex pieces");
  }
  num_bodies_ = std::max({num_bodies_, pair.body_a + 1, pair.body_b + 1});
  pairs_.push_back(std::move(pair));
  groups_dirty_.value = true;
}

void ContactModel::set_explicit_coupling(double dt, std::vector<BodyInertia> bodies) {
  if (!(dt >= 0.0)) throw std::invalid_argument("ContactModel::set_explicit_coupling: negative step");
  dt_ = dt;
  inertias_ = std::move(bodies);
}

namespace {
[[maybe_unused]] constexpr std::size_t kMinPairsPerThread = 4;  // Smallest share of candidates worth a thread.
constexpr std::size_t kTreeMinPairs = 16;  // Below this a linear scan beats a tree query.

// Inverse masses a contact at `c` with normal `n` sees of one body (StepLimits).
struct PointInertia {
  double normal = 0.0, tangent = 0.0, spin = 0.0;
};

// `gyration` is the inertia of the overlap about its centroid over its volume: the foundation's
// damping resists the rocking about the tangent axes with K·h times that inertia.
PointInertia point_inertia(const ContactModel::BodyInertia& body, const Pose& X, const Vec3& c, const Vec3& n,
                           const Mat3& gyration) {
  const Mat3 inv_inertia = X.R * body.inv_inertia * X.R.transpose();
  const Vec3 r = c - X.p;
  // u·W·v, with W = 1/m − [r]·I⁻¹·[r] the inverse mass at the point.
  const auto along = [&](const Vec3& u, const Vec3& v) {
    return body.inv_mass * u.dot(v) + r.cross(u).dot(inv_inertia * r.cross(v));
  };
  const Vec3 t1 = n.unitOrthogonal(), t2 = n.cross(t1);
  const double a = along(t1, t1), b = along(t2, t2), ab = along(t1, t2);
  const Mat3 tangent = Mat3::Identity() - n * n.transpose();
  return {.normal = along(n, n) + (inv_inertia * tangent * gyration * tangent).trace(),
          .tangent = 0.5 * (a + b) + std::sqrt(0.25 * (a - b) * (a - b) + ab * ab),  // Largest eigenvalue.
          .spin = n.dot(inv_inertia * n)};
}
}  // namespace

void ContactModel::build_groups() const {
  groups_.clear();
  for (std::size_t i = 0; i < pairs_.size(); ++i) {
    const Pair& p = pairs_[i];
    auto it = std::find_if(groups_.begin(), groups_.end(),
                           [&](const Group& g) { return g.body_a == p.body_a && g.body_b == p.body_b; });
    if (it == groups_.end()) {
      groups_.emplace_back();
      groups_.back().body_a = p.body_a;
      groups_.back().body_b = p.body_b;
      it = groups_.end() - 1;
    }
    it->pairs.push_back(static_cast<std::uint32_t>(i));
    it->boxes_a.push_back(p.shape_a->bounds());
    it->boxes_b.push_back(p.shape_b->bounds());
    it->bounds_a.grow(p.shape_a->bounds());
  }
  for (Group& g : groups_) {
    g.center_a = g.bounds_a.center();
    double r2 = 0.0;
    const CollisionShape* last = nullptr;
    for (const std::uint32_t i : g.pairs) {
      const CollisionShape* a = pairs_[i].shape_a.get();
      if (a == last) continue;
      last = a;
      for (const Vec3& v : a->mesh().vertices) r2 = std::max(r2, (v - g.center_a).squaredNorm());
    }
    g.radius_a = std::sqrt(r2);
    if (g.pairs.size() >= kTreeMinPairs) g.tree = std::make_shared<const Bvh>(g.boxes_b);
  }
  groups_dirty_.value = false;
}

void ContactModel::finalize() const {
  const std::lock_guard lock(*groups_mutex_);
  if (groups_dirty_.value) build_groups();
}

void ContactModel::find_candidates(std::span<const Pose> poses, std::vector<std::uint32_t>* out) const {
  out->clear();
  for (const Group& g : groups_) {
    if (!g.tree) {
      out->insert(out->end(), g.pairs.begin(), g.pairs.end());
      continue;
    }
    const Pose X_BA = poses[g.body_b].inverse() * poses[g.body_a];
    // A's ball in B's frame, slightly fattened, against the boxes of B's shapes.
    const Vec3 c = X_BA.apply(g.center_a);
    const double r = g.radius_a * (1.0 + 1e-9);
    const double r2 = r * r;
    const Vec3 pad = Vec3::Constant(1e-9 * g.radius_a);
    const auto near_ball = [&](const Aabb& box) {
      const Vec3 d = (box.lo - c).cwiseMax(c - box.hi).cwiseMax(0.0);
      return d.squaredNorm() <= r2;
    };
        g.tree->for_each_accepted(near_ball, [&](std::uint32_t k) {
      if (!g.boxes_a[k].transformed(X_BA).overlaps(Aabb{g.boxes_b[k].lo - pad, g.boxes_b[k].hi + pad})) return;
      // A convex B shape with a face plane the ball lies beyond is separated from A.
      const CollisionShape& b = *pairs_[g.pairs[k]].shape_b;
      const double reach = r + b.plane_tolerance();
      for (const Plane& plane : b.face_planes()) {
        if (plane.distance(c) > reach) return;
      }
      out->push_back(g.pairs[k]);
    });
  }
  std::sort(out->begin(), out->end());
}

void ContactModel::evaluate_pair(std::size_t i, std::span<const Pose> poses, std::span<const Twist> twists,
                                 const FrictionState& friction, Workspace& ws, PairResult* r,
                                 FrictionState* next) const {
  const Pair& pair = pairs_[i];
  const Pose& X_WA = poses[pair.body_a];
  const Pose& X_WB = poses[pair.body_b];
  const auto* distributed = std::get_if<DistributedParams>(&pair.law);
  const auto* pcm = std::get_if<PcmParams>(&pair.law);
  const bool on_surface = distributed || pcm;  // Laws evaluated on the contact surface.

  r->in_contact = intersect(*pair.shape_a, X_WA, *pair.shape_b, X_WB, ws.geometry, &r->geometry,
                            on_surface ? &ws.surface : nullptr);
  if (!r->in_contact) {
    r->force = {};
    *next = {};
    return;
  }
  // Pair normal. The distributed law runs its columns along the normal of B's
  // contact faces, leaving out the seams of a compound B (the per-face law takes
  // the same one, for the bounds of the step only). When the overlap
  // defines no normal (A enclosed in B or piercing it, where the vector area
  // cancels) the pair keeps its last one, which is how it got there; a pair that
  // starts that deep takes the shortest way out of B. Either way a deep overlap
  // keeps its force.
  Vec3& normal = r->geometry.normal;
  if (on_surface && !r->geometry.exterior_normal.isZero()) normal = r->geometry.exterior_normal;
  if (normal.isZero()) {
    normal = friction.has_normal ? Vec3(X_WB.R * friction.normal_B)
                                 : escape_direction(*pair.shape_b, X_WB, r->geometry.centroid);
  }
  const Twist& V_WA = twists[pair.body_a];
  const Twist& V_WB = twists[pair.body_b];
  StepLimits limits;
  PointInertia of_a, of_b;
  if (dt_ > 0.0 && r->geometry.volume > 0.0) {
    const Mat3 gyration = r->geometry.inertia / r->geometry.volume;
    of_a = point_inertia(inertias_[pair.body_a], X_WA, r->geometry.centroid, normal, gyration);
    of_b = point_inertia(inertias_[pair.body_b], X_WB, r->geometry.centroid, normal, gyration);
    limits = {.dt = dt_,
              .inv_mass_normal = of_a.normal + of_b.normal,
              .inv_mass_tangent = of_a.tangent + of_b.tangent,
              .inv_inertia_normal = of_a.spin + of_b.spin,
              .load_normal = friction.load_normal,
              .load_tangent = friction.load_tangent};
  }
  double stiffness = 0.0;
  if (distributed) {
    stiffness = distributed->stiffness;
    r->force = distributed_force(r->geometry, ws.surface, {*pair.shape_a, X_WA, V_WA}, {*pair.shape_b, X_WB, V_WB},
                                 *distributed, friction, next, ws.distributed, limits);
  } else if (pcm) {
    stiffness = pcm->stiffness;
    r->force = pcm_force(r->geometry, ws.surface, {*pair.shape_a, X_WA, V_WA}, {*pair.shape_b, X_WB, V_WB}, *pcm,
                         ws.pcm, limits);
    *next = {};
  } else {
    const GonthierParams& gonthier = std::get<GonthierParams>(pair.law);
    stiffness = gonthier.stiffness;
    r->force = gonthier_force(r->geometry, X_WA, V_WA, X_WB, V_WB, gonthier, friction, next, limits);
  }
  next->has_normal = true;
  next->normal_B = X_WB.R.transpose() * normal;
  // What the pair adds to the load accelerations of its bodies: the normal damper is
  // proportional to the elastic force, the stick to the normal force.
  const double elastic = stiffness * r->geometry.volume, load = std::abs(r->force.normal_force);
  r->load_normal[0] = elastic * of_a.normal;
  r->load_normal[1] = elastic * of_b.normal;
  r->load_tangent[0] = load * of_a.tangent;
  r->load_tangent[1] = load * of_b.tangent;
}

void ContactModel::evaluate(std::span<const Pose> poses, std::span<const Twist> twists,
                            std::span<const FrictionState> friction, Evaluation* out) const {
  const std::size_t n = pairs_.size();
  if (poses.size() < static_cast<std::size_t>(num_bodies_) || twists.size() < poses.size() || friction.size() != n) {
    throw std::invalid_argument("ContactModel::evaluate: size mismatch");
  }
  if (dt_ > 0.0 && inertias_.size() < static_cast<std::size_t>(num_bodies_)) {
    throw std::invalid_argument("ContactModel::evaluate: the explicit coupling lacks the inertia of a body");
  }
  out->pairs.resize(n);
  out->next_friction.resize(n);

  if (groups_dirty_.value) finalize();
  find_candidates(poses, &out->candidates);
  const std::size_t nc = out->candidates.size();
  // Pairs outside the broadphase are not in contact.
  for (std::size_t i = 0; i < n; ++i) {
    out->pairs[i].in_contact = false;
    out->pairs[i].force = {};
    out->next_friction[i] = {};
  }

#ifdef VOLCONTACT_OPENMP
  // A team larger than the work only adds barrier cost, and on a loaded machine
  // every extra thread is a chance for the whole team to wait on a descheduled one.
  // Default to half the hardware threads (about the physical cores): measured, a
  // team spanning all of them loses to one on half when anything else is running.
  int cap = std::min(omp_get_max_threads(), std::max(1, omp_get_num_procs() / 2));
  if (max_threads_ > 0) cap = std::min(cap, max_threads_);
  const std::size_t threads = static_cast<int>(nc) >= parallel_threshold_
                                  ? std::clamp<std::size_t>(nc / kMinPairsPerThread, 1, std::max(cap, 1))
                                  : 1;
  const bool parallel = threads > 1;
#else
  const std::size_t threads = 1;
#endif
  if (out->workspaces.size() < threads) out->workspaces.resize(threads);

#ifdef VOLCONTACT_OPENMP
  if (parallel) {
#pragma omp parallel for schedule(dynamic) num_threads(static_cast<int>(threads))
    // A signed index: OpenMP 2.0 (MSVC) does not accept an unsigned loop variable.
    for (std::ptrdiff_t c = 0; c < static_cast<std::ptrdiff_t>(nc); ++c) {
      const std::size_t i = out->candidates[c];
      evaluate_pair(i, poses, twists, friction[i], out->workspaces[omp_get_thread_num()], &out->pairs[i],
                    &out->next_friction[i]);
    }
  } else
#endif
  {
    for (std::size_t c = 0; c < nc; ++c) {
      const std::size_t i = out->candidates[c];
      evaluate_pair(i, poses, twists, friction[i], out->workspaces[0], &out->pairs[i], &out->next_friction[i]);
    }
  }

  // Accumulated serially: several pairs may act on the same body.
  out->wrenches.assign(poses.size(), Wrench{});
  for (std::size_t c = 0; c < nc; ++c) {
    const std::size_t i = out->candidates[c];
    const PairResult& r = out->pairs[i];
    if (!r.in_contact) continue;
    const Pair& pair = pairs_[i];
    const ContactForce& f = r.force;
    Wrench& wa = out->wrenches[pair.body_a];
    Wrench& wb = out->wrenches[pair.body_b];
    wa.force += f.force;
    wa.torque += f.torque + (f.point - poses[pair.body_a].p).cross(f.force);
    wb.force -= f.force;
    wb.torque -= f.torque + (f.point - poses[pair.body_b].p).cross(f.force);
  }

  // Load accelerations of the bodies, which the pairs in contact carry to the next evaluation.
  if (dt_ <= 0.0) return;
  out->load_normal.assign(poses.size(), 0.0);
  out->load_tangent.assign(poses.size(), 0.0);
  for (std::size_t c = 0; c < nc; ++c) {
    const std::size_t i = out->candidates[c];
    const PairResult& r = out->pairs[i];
    if (!r.in_contact) continue;
    const int bodies[2] = {pairs_[i].body_a, pairs_[i].body_b};
    for (int k = 0; k < 2; ++k) {
      out->load_normal[bodies[k]] += r.load_normal[k];
      out->load_tangent[bodies[k]] += r.load_tangent[k];
    }
  }
  for (std::size_t c = 0; c < nc; ++c) {
    const std::size_t i = out->candidates[c];
    if (!out->pairs[i].in_contact) continue;
    const Pair& pair = pairs_[i];
    out->next_friction[i].load_normal = out->load_normal[pair.body_a] + out->load_normal[pair.body_b];
    out->next_friction[i].load_tangent = out->load_tangent[pair.body_a] + out->load_tangent[pair.body_b];
  }
}

}  // namespace volcontact
