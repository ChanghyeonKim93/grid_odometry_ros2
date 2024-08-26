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
  // Vec2 normal_vector{Vec2::Zero()};
  // Vec2 mean{Vec2::Zero()};
  // Mat2x2 moment{Mat2x2::Zero()};
  int8_t hit_count{-1};
};

struct GridMap {
  int id{-1};
  Pose pose{Pose::Identity()};
  std::vector<GridCell> cell_list;

  Eigen::Vector2i origin{Eigen::Vector2i::Zero()};
  int width{1};
  int height{1};
  double resolution{0.0};
};

// For quad map
struct Quad {
  Point left_bottom;
  Point right_top;
  const std::deque<Point>* point_list_ptr = nullptr;
  std::vector<int> point_index_list;
  Quad* child_list[4] = {nullptr};
  int8_t depth{-1};

  Point mean;
  Vec2 direction;

  Quad(const std::deque<Point>* point_list_ptr_)
      : point_list_ptr(point_list_ptr_) {}
};

struct QuadBase {
  std::deque<Point> point_list;
  Quad* root = nullptr;
};

struct QuadMap {
  int id{-1};
  Pose pose{Pose::Identity()};
  std::unordered_map<uint64_t, QuadBase> quad_base_list;
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_TYPES_H_