#ifndef GRID_ODOMETER_CORE_TYPES_H_
#define GRID_ODOMETER_CORE_TYPES_H_

#include <deque>
#include <unordered_map>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace grid_odometer {

using Point = Eigen::Vector2d;

using Vec2 = Eigen::Vector2d;
using Mat2x2 = Eigen::Matrix2d;

struct TimedPoint {
  double time{0.0};
  Point point{Point::Zero()};
};

struct LaserScan {
  double time{0.0};
  std::vector<TimedPoint> data;
};

using Pose = Eigen::Isometry2d;

struct GridCell {
  Vec2 normal_vector{Vec2::Zero()};
  Vec2 mean{Vec2::Zero()};
  Mat2x2 moment{Mat2x2::Zero()};

  std::unordered_map<uint64_t, std::vector<Point>> point_list_per_pose;

  uint8_t depth{0};
  GridCell* child_cell_list[4] = {nullptr};
};

struct GridMap {
  int id{-1};
  Pose pose{Pose::Identity()};
  std::unordered_map<uint64_t, GridCell> cell_list;
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_TYPES_H_