#include "volcontact/c_api.h"

#include <cstring>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "volcontact/contact_model.hpp"

// Handles wrap the C++ objects; nothing here adds behaviour.
struct volcontact_shape {
  std::shared_ptr<const volcontact::CollisionShape> shape;
};

struct volcontact_model {
  volcontact::ContactModel model;
  volcontact::ContactModel::Evaluation eval;
  // Converted inputs, kept between calls so that evaluating does not allocate.
  std::vector<volcontact::Pose> poses;
  std::vector<volcontact::Twist> twists;
  std::vector<volcontact::FrictionState> friction;
};

static_assert(VOLCONTACT_FRICTION_STATE_SIZE == volcontact::FrictionState::kSize, "C API friction state size out of date");

namespace {

thread_local std::string g_error;

volcontact_status fail(volcontact_status status, const char* what) {
  g_error = what;
  return status;
}

// Runs `f`, turning exceptions into status codes.
template <class F>
volcontact_status guarded(F&& f) {
  try {
    f();
    g_error.clear();
    return VOLCONTACT_OK;
  } catch (const std::invalid_argument& e) {
    return fail(VOLCONTACT_INVALID_ARGUMENT, e.what());
  } catch (const std::exception& e) {
    return fail(VOLCONTACT_ERROR, e.what());
  } catch (...) {
    return fail(VOLCONTACT_ERROR, "unknown error");
  }
}

volcontact::TriMesh to_mesh(const volcontact_mesh& m) {
  if (!m.vertices || !m.triangles || m.num_vertices == 0 || m.num_triangles == 0) {
    throw std::invalid_argument("volcontact_mesh: empty mesh");
  }
  volcontact::TriMesh mesh;
  mesh.vertices.reserve(m.num_vertices);
  for (std::size_t i = 0; i < m.num_vertices; ++i) {
    mesh.vertices.emplace_back(m.vertices[3 * i], m.vertices[3 * i + 1], m.vertices[3 * i + 2]);
  }
  mesh.triangles.reserve(m.num_triangles);
  for (std::size_t t = 0; t < m.num_triangles; ++t) {
    const std::uint32_t a = m.triangles[3 * t], b = m.triangles[3 * t + 1], c = m.triangles[3 * t + 2];
    if (a >= m.num_vertices || b >= m.num_vertices || c >= m.num_vertices) {
      throw std::invalid_argument("volcontact_mesh: triangle index out of range");
    }
    mesh.triangles.push_back({a, b, c});
  }
  return mesh;
}

volcontact::Vec3 vec3(const double* x) { return {x[0], x[1], x[2]}; }

volcontact::GonthierParams from_c(const volcontact_gonthier_params& c) {
  volcontact::GonthierParams p;
  p.stiffness = c.stiffness;
  p.damping = c.damping;
  p.mu_spin = c.mu_spin;
  p.rolling_resistance = c.rolling_resistance != 0;
  p.allow_adhesion = c.allow_adhesion != 0;
  p.friction_model = c.friction_model == 1 ? volcontact::GonthierParams::FrictionModel::kThesis2018
                                           : volcontact::GonthierParams::FrictionModel::kPublished;
  p.friction.reset();
  if (c.has_friction) {
    volcontact::BristleFriction f;
    f.mu_dynamic = c.mu_dynamic;
    f.mu_static = c.mu_static;
    f.mu_viscous = c.mu_viscous;
    f.v_stick = c.v_stick;
    f.k_stick = c.k_stick;
    f.c_stick = c.c_stick;
    f.anchor_update = c.anchor_update;
    f.limit_includes_damping = c.limit_includes_damping != 0;
    f.preslide_travel = c.preslide_travel;
    f.preslide_time = c.preslide_time;
    p.friction = f;
  }
  if (c.friction_at_body_point) {
    p.friction_point = {volcontact::FrictionPoint::Kind::kBodyPoint, vec3(c.friction_point_a)};
  }
  return p;
}

volcontact::DistributedParams from_c(const volcontact_distributed_params& c) {
  volcontact::DistributedParams p;
  p.stiffness = c.stiffness;
  p.damping = c.damping;
  p.mu_dynamic = c.mu_dynamic;
  p.mu_static = c.mu_static;
  p.allow_adhesion = c.allow_adhesion != 0;
  p.stick.reset();
  if (c.has_stick) {
    volcontact::StickParams s;
    s.v_stick = c.v_stick;
    s.k_stick = c.k_stick;
    s.c_stick = c.c_stick;
    s.travel = c.travel;
    s.time = c.time;
    p.stick = s;
  }
  return p;
}

void check(bool ok, const char* what) {
  if (!ok) throw std::invalid_argument(what);
}

template <class Params>
volcontact_status add_pair(volcontact_model* model, int a, int b, const volcontact_shape* sa, const volcontact_shape* sb, const Params* params,
                   std::size_t* out_index) {
  return guarded([&] {
    check(model && sa && sb && params, "null argument");
    volcontact::ContactModel::Pair pair;
    pair.body_a = a;
    pair.body_b = b;
    pair.shape_a = sa->shape;
    pair.shape_b = sb->shape;
    pair.law = from_c(*params);
    model->model.add_pair(std::move(pair));
    if (out_index) *out_index = model->model.pairs().size() - 1;
  });
}

}  // namespace

extern "C" {

const char* volcontact_last_error(void) { return g_error.c_str(); }

volcontact_status volcontact_shape_create(const volcontact_mesh* mesh, volcontact_shape** out) {
  return guarded([&] {
    check(mesh && out, "null argument");
    *out = nullptr;
    auto shape = std::make_shared<const volcontact::CollisionShape>(to_mesh(*mesh));
    *out = new volcontact_shape{std::move(shape)};
  });
}

volcontact_status volcontact_compound_create(const volcontact_mesh* pieces, std::size_t num_pieces, double tolerance, volcontact_shape** out_pieces) {
  return guarded([&] {
    check(pieces && out_pieces && num_pieces > 0, "null argument or no pieces");
    for (std::size_t i = 0; i < num_pieces; ++i) out_pieces[i] = nullptr;
    std::vector<volcontact::TriMesh> meshes;
    meshes.reserve(num_pieces);
    for (std::size_t i = 0; i < num_pieces; ++i) meshes.push_back(to_mesh(pieces[i]));
    volcontact::CompoundShape compound = volcontact::CompoundShape::from_meshes(std::move(meshes), tolerance);
    std::vector<std::unique_ptr<volcontact_shape>> made;
    for (auto& piece : compound.pieces) made.push_back(std::make_unique<volcontact_shape>(volcontact_shape{std::move(piece)}));
    for (std::size_t i = 0; i < num_pieces; ++i) out_pieces[i] = made[i].release();
  });
}

void volcontact_shape_destroy(volcontact_shape* shape) { delete shape; }

volcontact_gonthier_params volcontact_gonthier_params_default(void) {
  const volcontact::GonthierParams p;
  const volcontact::BristleFriction f;  // The friction of a model that enables it without changes.
  volcontact_gonthier_params c;
  std::memset(&c, 0, sizeof c);
  c.stiffness = p.stiffness;
  c.damping = p.damping;
  c.mu_spin = p.mu_spin;
  c.rolling_resistance = p.rolling_resistance;
  c.allow_adhesion = p.allow_adhesion;
  c.friction_model = p.friction_model == volcontact::GonthierParams::FrictionModel::kThesis2018 ? 1 : 0;
  c.has_friction = 1;
  c.mu_dynamic = f.mu_dynamic;
  c.mu_static = f.mu_static;
  c.mu_viscous = f.mu_viscous;
  c.v_stick = f.v_stick;
  c.k_stick = f.k_stick;
  c.c_stick = f.c_stick;
  c.anchor_update = f.anchor_update;
  c.limit_includes_damping = f.limit_includes_damping;
  c.preslide_travel = f.preslide_travel;
  c.preslide_time = f.preslide_time;
  return c;
}

volcontact_distributed_params volcontact_distributed_params_default(void) {
  const volcontact::DistributedParams p;
  const volcontact::StickParams s;
  volcontact_distributed_params c;
  std::memset(&c, 0, sizeof c);
  c.stiffness = p.stiffness;
  c.damping = p.damping;
  c.mu_dynamic = p.mu_dynamic;
  c.mu_static = p.mu_static;
  c.allow_adhesion = p.allow_adhesion;
  c.has_stick = p.stick.has_value();
  c.v_stick = s.v_stick;
  c.k_stick = s.k_stick;
  c.c_stick = s.c_stick;
  c.travel = s.travel;
  c.time = s.time;
  return c;
}

volcontact_status volcontact_model_create(volcontact_model** out) {
  return guarded([&] {
    check(out != nullptr, "null argument");
    *out = new volcontact_model();
  });
}

void volcontact_model_destroy(volcontact_model* model) { delete model; }

volcontact_status volcontact_model_add_pair_gonthier(volcontact_model* model, int body_a, int body_b, const volcontact_shape* shape_a,
                                     const volcontact_shape* shape_b, const volcontact_gonthier_params* params, std::size_t* out_index) {
  return add_pair(model, body_a, body_b, shape_a, shape_b, params, out_index);
}

volcontact_status volcontact_model_add_pair_distributed(volcontact_model* model, int body_a, int body_b, const volcontact_shape* shape_a,
                                        const volcontact_shape* shape_b, const volcontact_distributed_params* params,
                                        std::size_t* out_index) {
  return add_pair(model, body_a, body_b, shape_a, shape_b, params, out_index);
}

std::size_t volcontact_model_num_pairs(const volcontact_model* model) { return model ? model->model.pairs().size() : 0; }
int volcontact_model_num_bodies(const volcontact_model* model) { return model ? model->model.num_bodies() : 0; }

volcontact_status volcontact_model_finalize(volcontact_model* model) {
  return guarded([&] {
    check(model != nullptr, "null argument");
    model->model.finalize();
  });
}

volcontact_status volcontact_model_set_explicit_coupling(volcontact_model* model, double dt, std::size_t num_bodies,
                                                         const double* inv_masses, const double* inv_inertias) {
  return guarded([&] {
    check(model != nullptr, "null argument");
    check(num_bodies == 0 || (inv_masses && inv_inertias), "null argument");
    check(dt >= 0.0, "negative step");
    std::vector<volcontact::ContactModel::BodyInertia> bodies(num_bodies);
    for (std::size_t i = 0; i < num_bodies; ++i) {
      check(inv_masses[i] >= 0.0, "negative inverse mass");
      bodies[i].inv_mass = inv_masses[i];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) bodies[i].inv_inertia(r, c) = inv_inertias[9 * i + 3 * r + c];
      }
    }
    model->model.set_explicit_coupling(dt, std::move(bodies));
  });
}

void volcontact_friction_state_init(double* state) {
  if (state) volcontact::FrictionState{}.pack(state);
}

volcontact_status volcontact_model_evaluate(volcontact_model* model, std::size_t num_bodies, const double* poses, const double* twists,
                            const double* friction, double* wrenches_out, double* next_friction_out,
                            int* in_contact_out) {
  return guarded([&] {
    check(model && wrenches_out, "null argument");
    const std::size_t np = model->model.pairs().size();
    check(num_bodies >= static_cast<std::size_t>(model->model.num_bodies()), "fewer bodies than the model uses");
    check(num_bodies == 0 || (poses && twists), "null poses or twists");
    check(np == 0 || (friction && next_friction_out), "null friction state");

    model->poses.resize(num_bodies);
    model->twists.resize(num_bodies);
    for (std::size_t i = 0; i < num_bodies; ++i) {
      const double* x = poses + VOLCONTACT_POSE_SIZE * i;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) model->poses[i].R(r, c) = x[3 * r + c];
      }
      model->poses[i].p = vec3(x + 9);
      model->twists[i] = {vec3(twists + VOLCONTACT_TWIST_SIZE * i), vec3(twists + VOLCONTACT_TWIST_SIZE * i + 3)};
    }
    model->friction.resize(np);
    for (std::size_t i = 0; i < np; ++i) {
      model->friction[i] = volcontact::FrictionState::unpack(friction + VOLCONTACT_FRICTION_STATE_SIZE * i);
    }

    model->model.evaluate(model->poses, model->twists, model->friction, &model->eval);

    for (std::size_t i = 0; i < num_bodies; ++i) {
      const volcontact::Wrench& w = model->eval.wrenches[i];
      Eigen::Map<volcontact::Vec3>(wrenches_out + VOLCONTACT_WRENCH_SIZE * i) = w.torque;
      Eigen::Map<volcontact::Vec3>(wrenches_out + VOLCONTACT_WRENCH_SIZE * i + 3) = w.force;
    }
    for (std::size_t i = 0; i < np; ++i) {
      model->eval.next_friction[i].pack(next_friction_out + VOLCONTACT_FRICTION_STATE_SIZE * i);
      if (in_contact_out) in_contact_out[i] = model->eval.pairs[i].in_contact ? 1 : 0;
    }
  });
}

}  // extern "C"
