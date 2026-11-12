#include "volcontact/mesh.hpp"

#include <charconv>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace volcontact {
namespace {

std::string_view next_token(std::string_view& s) {
  const auto begin = s.find_first_not_of(" \t\r");
  if (begin == std::string_view::npos) {
    s = {};
    return {};
  }
  s.remove_prefix(begin);
  const auto end = std::min(s.find_first_of(" \t\r"), s.size());
  const std::string_view token = s.substr(0, end);
  s.remove_prefix(end);
  return token;
}

double parse_double(std::string_view token, const std::filesystem::path& path) {
  double value = 0.0;
  const auto [ptr, ec] = std::from_chars(token.data(), token.data() + token.size(), value);
  if (ec != std::errc{}) throw std::runtime_error("invalid number in " + path.string());
  return value;
}

// Parses the vertex index of an OBJ face token ("7", "7/1", "7//3", "-1/...").
std::uint32_t parse_index(std::string_view token, std::size_t num_vertices, const std::filesystem::path& path) {
  token = token.substr(0, token.find('/'));
  long index = 0;
  const auto [ptr, ec] = std::from_chars(token.data(), token.data() + token.size(), index);
  if (ec != std::errc{} || index == 0) throw std::runtime_error("invalid face index in " + path.string());
  const long resolved = index > 0 ? index - 1 : static_cast<long>(num_vertices) + index;
  if (resolved < 0 || resolved >= static_cast<long>(num_vertices)) {
    throw std::runtime_error("face index out of range in " + path.string());
  }
  return static_cast<std::uint32_t>(resolved);
}

}  // namespace

Aabb TriMesh::bounds() const {
  Aabb box;
  for (const Vec3& v : vertices) box.grow(v);
  return box;
}

TriMesh load_obj(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path.string());

  TriMesh mesh;
  std::vector<std::uint32_t> face;
  std::string line;
  while (std::getline(in, line)) {
    std::string_view rest(line);
    const std::string_view tag = next_token(rest);
    if (tag == "v") {
      Vec3 v;
      for (int i = 0; i < 3; ++i) v[i] = parse_double(next_token(rest), path);
      mesh.vertices.push_back(v);
    } else if (tag == "f") {
      face.clear();
      for (std::string_view tok = next_token(rest); !tok.empty(); tok = next_token(rest)) {
        face.push_back(parse_index(tok, mesh.vertices.size(), path));
      }
      if (face.size() < 3) throw std::runtime_error("degenerate face in " + path.string());
      for (std::size_t i = 1; i + 1 < face.size(); ++i) mesh.triangles.push_back({face[0], face[i], face[i + 1]});
    }
  }
  if (mesh.triangles.empty()) throw std::runtime_error("no faces in " + path.string());
  return mesh;
}

double signed_volume(const TriMesh& mesh) {
  double six_v = 0.0;
  for (const auto& f : mesh.triangles) {
    six_v += mesh.vertices[f[0]].dot(mesh.vertices[f[1]].cross(mesh.vertices[f[2]]));
  }
  return six_v / 6.0;
}

void flip_orientation(TriMesh& mesh) {
  for (auto& f : mesh.triangles) std::swap(f[1], f[2]);
}

}  // namespace volcontact
