#pragma once

#include <cstdint>
#include <vector>

#include "volcontact/mesh.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// Static bounding volume hierarchy of axis-aligned boxes over the triangles of a
// mesh, in the mesh frame. Nodes are stored depth-first in one array: the left
// child of an internal node immediately follows it.
class Bvh {
 public:
  explicit Bvh(const TriMesh& mesh, int max_leaf_size = 4);
  // Over arbitrary boxes; "triangle" indices below are then indices into `boxes`.
  explicit Bvh(std::vector<Aabb> boxes, int max_leaf_size = 4);

  // Calls visit(triangle_index) for every triangle whose box overlaps `query`.
  template <class Visit>
  void for_each_overlapping(const Aabb& query, Visit&& visit) const {
    for_each_accepted([&](const Aabb& box) { return box.overlaps(query); }, visit);
  }

  // Calls visit(triangle_index) for every triangle whose box is accepted by
  // `accept`, which must accept a box whenever it accepts any box inside it.
  template <class Accept, class Visit>
  void for_each_accepted(Accept&& accept, Visit&& visit) const {
    if (nodes_.empty()) return;
    std::uint32_t stack[kMaxDepth];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
      const Node& node = nodes_[stack[--top]];
      if (!accept(node.box)) continue;
      if (node.count > 0) {
        for (std::uint32_t i = node.offset, end = node.offset + node.count; i < end; ++i) {
          if (accept(tri_boxes_[i])) visit(tri_index_[i]);
        }
      } else {
        const auto self = static_cast<std::uint32_t>(&node - nodes_.data());
        stack[top++] = node.offset;  // Right child.
        stack[top++] = self + 1;     // Left child.
      }
    }
  }

  [[nodiscard]] const Aabb& bounds() const { return nodes_.front().box; }
  [[nodiscard]] std::size_t num_nodes() const { return nodes_.size(); }

 private:
  static constexpr int kMaxDepth = 64;

  struct Node {
    Aabb box;
    std::uint32_t offset = 0;  // Leaf: first entry in tri_index_. Internal: right child.
    std::uint32_t count = 0;   // Number of triangles; zero for internal nodes.
  };

  void init(std::vector<Aabb> boxes, const std::vector<Vec3>& centroids, int max_leaf_size);
  std::uint32_t build(std::uint32_t begin, std::uint32_t end, int depth, int max_leaf_size,
                      const std::vector<Vec3>& centroids);

  std::vector<Node> nodes_;
  std::vector<std::uint32_t> tri_index_;  // Triangle ids in leaf order.
  std::vector<Aabb> tri_boxes_;           // Boxes in leaf order (avoids an indirection while querying).
};

}  // namespace volcontact
