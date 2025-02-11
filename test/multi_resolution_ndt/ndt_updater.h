#ifndef NDT_UPDATER_H_
#define NDT_UPDATER_H_

#include <iostream>
#include <unordered_map>

#include "types.h"

namespace ndt_updater {

struct Parameters {
  double ndt_resolution{0.8};
  struct {
    int max_depth{5};  // 0.2, 0.1, 0.05, 0.025, 0.0125
  } quadtree;
};

struct NdtNode {
  double voxel_size{0.0};
  Vec2 left_top{Vec2::Zero()};
  uint8_t depth{0};

  uint16_t count{0};
  Vec2 sum{Vec2::Zero()};
  Mat2x2 moment{Mat2x2::Zero()};
  NdtNode* child_nodes[4]{nullptr, nullptr, nullptr, nullptr};

  int GetChildIndex(const Vec2& point) {
    const double half_voxel_size = voxel_size * 0.5;
    const bool is_right = point.x() > left_top.x() + half_voxel_size;
    const bool is_bottom = point.y() > left_top.y() + half_voxel_size;
    const int child_index = is_right + 2 * is_bottom;
    return child_index;
  }

  void GenerateChildNode(const int child_index) {
    const double half_voxel_size = voxel_size * 0.5;
    child_nodes[child_index] = new NdtNode;
    NdtNode* child_node = child_nodes[child_index];
    child_node->voxel_size = half_voxel_size;
    child_node->depth = depth + 1;
    child_node->left_top.x() =
        left_top.x() + (child_index % 2) * half_voxel_size;
    child_node->left_top.y() =
        left_top.y() + (child_index / 2) * half_voxel_size;
  }
};

class NdtUpdater {
 public:
  NdtUpdater();

  ~NdtUpdater();

  void AddPoints(const std::vector<Vec2>& points);

  void UpdateNdtMap();

  std::unordered_map<uint64_t, NdtNode> GetNdtMap();

 private:
  const Parameters parameters_;
  const double inverse_resolution_;

  std::unordered_map<uint64_t, NdtNode> ndt_map_;

  uint64_t GetKey(const Vec2& point);
  void UpdateNdtRecursively(NdtNode* node);
};

}  // namespace ndt_updater

#endif  // NDT_UPDATER_H_