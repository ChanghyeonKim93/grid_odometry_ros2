#ifndef GRID_MAPPER_CORE_TYPES_H_
#define GRID_MAPPER_CORE_TYPES_H_

#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace grid_mapper {

using Vec2 = Eigen::Vector2d;
using Mat2x2 = Eigen::Matrix2d;
using Pose = Eigen::Isometry2d;

struct TimedPoint {
  double time{0.0};
  Vec2 point{Vec2::Zero()};
};

struct LaserScan {
  double time{0.0};
  std::vector<TimedPoint> data;
};

struct QuadNode;
using QuadNodePtr = std::shared_ptr<QuadNode>;

struct QuadNode {
  double x{0.0};
  double y{0.0};
  double size{0.0};
  int depth{0};
  Vec2 ndt_center{Vec2::Zero()};
  Vec2 ndt_axis{Vec2::Zero()};
  double ndt_angle{0.0};
  QuadNodePtr child_node_list[4] = {
      nullptr,
  };
  std::vector<int> point_index_list;

  QuadNode(const double x, const double y, const double size,
           const double depth) {
    this->x = x;
    this->y = y;
    this->size = size;
    this->depth = depth;
  };
};

struct GridCell {
  std::vector<Vec2> point_list;
  QuadNodePtr root{nullptr};
};

}  // namespace grid_mapper

#endif  // GRID_MAPPER_CORE_TYPES_H_