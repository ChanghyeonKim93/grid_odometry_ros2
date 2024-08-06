#ifndef GRID_ODOMETER_CORE_TYPES_H_
#define GRID_ODOMETER_CORE_TYPES_H_

#include <deque>
#include <unordered_map>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace grid_odometer {

using Point = Eigen::Vector2d;

struct TimedPoint {
  double time{0.0};
  Point point{Point::Zero()};
};

struct LaserScan {
  double time{0.0};
  std::vector<TimedPoint> data;
};

struct Pose {
  double x{0.0};
  double y{0.0};
  double angle{0.0};
};

struct GridCell {
  Eigen::Vector2d normal_vector{Eigen::Vector2d::Zero()};
  Eigen::Vector2d mean{Eigen::Vector2d::Zero()};
  std::deque<Point> point_list;
  uint8_t depth{0};
  GridCell* child_cell_list[4];
};

struct GridMap {
  int id{-1};
  Pose pose;
  std::unordered_map<uint64_t, GridCell> grid_cell_list;
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_TYPES_H_