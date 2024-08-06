#ifndef GRID_ODOMETER_CORE_BRIDGE_H_
#define GRID_ODOMETER_CORE_BRIDGE_H_

#include <vector>

namespace grid_odometer {
namespace bridge {

struct Point {
  double x{0.0};
  double y{0.0};
};

struct TimedPoint {
  double time{0.0};
  Point point;
};

struct LaserScan {
  double time{0.0};
  std::vector<TimedPoint> data;
};

struct GridCell {
  Point normal_vector;
  Point mean;
  std::vector<Point> point_list;
  uint8_t depth{0};
  GridCell* child_cell_list[4];
};

struct GridMap {
  int id{-1};
  std::unordered_map<uint64_t, GridCell> grid_cell_list;
};

}  // namespace bridge
}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_BRIDGE_H_