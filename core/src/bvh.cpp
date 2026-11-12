#include "volcontact/detail/bvh.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace volcontact {

Bvh::Bvh(const TriMesh& mesh, int max_leaf_size) {
  const auto n = static_cast<std::uint32_t>(mesh.triangles.size());
  if (n == 0) throw std::invalid_argument("Bvh: empty mesh");

  std::vector<Aabb> boxes(n);
  std::vector<Vec3> centroids(n);
  for (std::uint32_t t = 0; t < n; ++t) {
    const auto v = mesh.triangle(t);
    for (const Vec3& p : v) boxes[t].grow(p);
    centroids[t] = (v[0] + v[1] + v[2]) / 3.0;
  }
  init(std::move(boxes), centroids, max_leaf_size);
}

Bvh::Bvh(std::vector<Aabb> boxes, int max_leaf_size) {
  if (boxes.empty()) throw std::invalid_argument("Bvh: no boxes");
  std::vector<Vec3> centroids(boxes.size());
  for (std::size_t i = 0; i < boxes.size(); ++i) centroids[i] = boxes[i].center();
  init(std::move(boxes), centroids, max_leaf_size);
}

void Bvh::init(std::vector<Aabb> boxes, const std::vector<Vec3>& centroids, int max_leaf_size) {
  const auto n = static_cast<std::uint32_t>(boxes.size());
  tri_index_.resize(n);
  std::iota(tri_index_.begin(), tri_index_.end(), 0u);
  nodes_.reserve(2 * n);
  tri_boxes_ = std::move(boxes);  // Indexed by triangle id until the build reorders it below.
  build(0, n, 0, max_leaf_size, centroids);

  std::vector<Aabb> ordered(n);
  for (std::uint32_t i = 0; i < n; ++i) ordered[i] = tri_boxes_[tri_index_[i]];
  tri_boxes_ = std::move(ordered);
}

std::uint32_t Bvh::build(std::uint32_t begin, std::uint32_t end, int depth, int max_leaf_size,
                         const std::vector<Vec3>& centroids) {
  const auto index = static_cast<std::uint32_t>(nodes_.size());
  nodes_.emplace_back();

  Aabb box, centroid_box;
  for (std::uint32_t i = begin; i < end; ++i) {
    box.grow(tri_boxes_[tri_index_[i]]);
    centroid_box.grow(centroids[tri_index_[i]]);
  }
  nodes_[index].box = box;

  const std::uint32_t count = end - begin;
  const Vec3 spread = centroid_box.hi - centroid_box.lo;
  int axis = 0;
  spread.maxCoeff(&axis);
  if (count <= static_cast<std::uint32_t>(max_leaf_size) || depth + 1 >= kMaxDepth / 2 || spread[axis] <= 0.0) {
    nodes_[index].offset = begin;
    nodes_[index].count = count;
    return index;
  }

  // Median split along the axis of largest centroid spread.
  const std::uint32_t mid = begin + count / 2;
  std::nth_element(tri_index_.begin() + begin, tri_index_.begin() + mid, tri_index_.begin() + end,
                   [&](std::uint32_t a, std::uint32_t b) { return centroids[a][axis] < centroids[b][axis]; });

  build(begin, mid, depth + 1, max_leaf_size, centroids);
  const std::uint32_t right = build(mid, end, depth + 1, max_leaf_size, centroids);
  nodes_[index].offset = right;
  return index;
}

}  // namespace volcontact
