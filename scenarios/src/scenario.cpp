#include "volcontact/scenario.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>

#include <Eigen/LU>

namespace volcontact {

std::string_view to_string(TheoryClass c) {
  switch (c) {
    case TheoryClass::kA: return "A";
    case TheoryClass::kB: return "B";
    case TheoryClass::kC: return "C";
  }
  return "?";
}

int contact_body_index(const Scenario& scenario, BodyRef ref) {
  const int num_bodies = static_cast<int>(scenario.bodies.size());
  const int count = ref.kind == BodyRef::Kind::kBody ? num_bodies : static_cast<int>(scenario.grounds.size());
  if (ref.index < 0 || ref.index >= count) {
    throw std::invalid_argument("scenario " + scenario.name + ": contact references a missing body");
  }
  return ref.kind == BodyRef::Kind::kBody ? ref.index : num_bodies + ref.index;
}

ContactModel make_contact_model(const Scenario& scenario, ForceModel model, double dt) {
  // One compound per asset: its pieces know which of their faces are seams.
  std::map<std::vector<std::filesystem::path>, CompoundShape> cache;
  const auto pieces = [&](BodyRef ref) -> const std::vector<std::shared_ptr<const CollisionShape>>& {
    const auto& paths = ref.kind == BodyRef::Kind::kBody ? scenario.bodies[ref.index].shape.pieces
                                                         : scenario.grounds[ref.index].shape.pieces;
    auto it = cache.find(paths);
    if (it == cache.end()) it = cache.emplace(paths, CompoundShape::from_objs(paths)).first;
    return it->second.pieces;
  };

  ContactModel contacts;
  for (const ContactSpec& c : scenario.contacts) {
    const int a = contact_body_index(scenario, c.a), b = contact_body_index(scenario, c.b);
    ContactLaw law = c.distributed;
    if (model == ForceModel::kGonthier) law = c.gonthier;
    if (model == ForceModel::kPcm) {
      law = PcmParams{.stiffness = c.distributed.stiffness,
                      .damping = c.distributed.damping,
                      .mu_dynamic = c.distributed.mu_dynamic,
                      .mu_static = c.distributed.mu_static};
    }
    for (const auto& piece_a : pieces(c.a)) {
      for (const auto& piece_b : pieces(c.b)) {
        contacts.add_pair({.body_a = a, .body_b = b, .shape_a = piece_a, .shape_b = piece_b, .law = law});
      }
    }
  }
  if (dt > 0.0) {
    std::vector<ContactModel::BodyInertia> bodies;
    for (const BodySpec& b : scenario.bodies) bodies.push_back({1.0 / b.mass, b.inertia.inverse()});
    bodies.resize(scenario.bodies.size() + scenario.grounds.size());  // The grounds do not move.
    contacts.set_explicit_coupling(dt, std::move(bodies));
  }
  return contacts;
}

ShapeMeshes shape_meshes(const std::filesystem::path& data_dir, const std::string& name) {
  auto folder = data_dir / "bodies" / name;
  if (!std::filesystem::is_directory(folder)) folder = data_dir / "grounds" / name;
  ShapeMeshes m{.surface = folder / "surface.obj"};
  if (!std::filesystem::exists(m.surface)) throw std::invalid_argument("missing mesh " + m.surface.string());
  const auto pieces = folder / "pieces";
  if (std::filesystem::is_directory(pieces)) {
    for (const auto& entry : std::filesystem::directory_iterator(pieces)) {
      if (entry.path().extension() == ".obj") m.pieces.push_back(entry.path());
    }
    std::sort(m.pieces.begin(), m.pieces.end());
  }
  if (m.pieces.empty()) m.pieces = {m.surface};
  const auto volume = folder / "volume.vtk";
  if (std::filesystem::exists(volume)) m.volume = volume;
  return m;
}

GroundSpec planar_ground(const std::filesystem::path& data_dir) {
  return {.shape = shape_meshes(data_dir, "slab_40000x40000x1000"), .planar = true};
}

}  // namespace volcontact
