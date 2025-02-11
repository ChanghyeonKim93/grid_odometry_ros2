#include "ndt_updater.h"

#include <iostream>

namespace ndt_updater {

NdtUpdater::NdtUpdater()
    : inverse_resolution_{1.0 / parameters_.ndt_resolution} {}

NdtUpdater::~NdtUpdater() {}

void NdtUpdater::AddPoints(const std::vector<Vec2>& points) {
  const double ndt_resolution = parameters_.ndt_resolution;
  for (const auto& point : points) {
    const auto key = GetKey(point);
    if (ndt_map_.find(key) == ndt_map_.end()) {
      NdtNode new_ndt;
      new_ndt.voxel_size = ndt_resolution;
      new_ndt.left_top.x() =
          std::floor(point.x() * inverse_resolution_) * ndt_resolution;
      new_ndt.left_top.y() =
          std::floor(point.y() * inverse_resolution_) * ndt_resolution;
      ndt_map_.insert({key, new_ndt});
    }
    // Traverse the tree to the leaf node. (max_depth)
    NdtNode* node = &ndt_map_.at(key);
    while (true) {
      if (node->depth == parameters_.quadtree.max_depth) {
        ++node->count;
        node->sum += point;
        node->moment += point * point.transpose();
        break;
      }
      const auto child_index = node->GetChildIndex(point);
      if (node->child_nodes[child_index] == nullptr)
        node->GenerateChildNode(child_index);
      node = node->child_nodes[child_index];
    }
  }
}

std::unordered_map<uint64_t, NdtNode> NdtUpdater::GetNdtMap() {
  return ndt_map_;
}

void NdtUpdater::UpdateNdtMap() {
  for (auto& [key, ndt_root] : ndt_map_) UpdateNdtRecursively(&ndt_root);
}

uint64_t NdtUpdater::GetKey(const Vec2& point) {
  int x_key = std::floor(point.x() * inverse_resolution_);
  int y_key = std::floor(point.y() * inverse_resolution_);
  x_key = x_key >= 0 ? 2 * x_key : -2 * x_key - 1;
  y_key = y_key >= 0 ? 2 * y_key : -2 * y_key - 1;
  const uint64_t key = (x_key + y_key) * (x_key + y_key + 1) / 2 + x_key;
  return key;
}

void NdtUpdater::UpdateNdtRecursively(NdtNode* node) {
  if (node->depth == parameters_.quadtree.max_depth) return;
  for (int i = 0; i < 4; ++i) {
    NdtNode* child_node = node->child_nodes[i];
    if (child_node == nullptr) continue;
    UpdateNdtRecursively(child_node);
    node->count += child_node->count;
    node->sum += child_node->sum;
    node->moment += child_node->moment;
  }
}

}  // namespace ndt_updater