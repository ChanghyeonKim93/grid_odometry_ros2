#ifndef GRID_ODOMETER_CORE_BRIDGE_H_
#define GRID_ODOMETER_CORE_BRIDGE_H_

#include <vector>

namespace grid_odometer {
namespace bridge {

struct Pose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Point {
  double x{0.0};
  double y{0.0};
  Point() {}
  Point(const double x_, const double y_) : x(x_), y(y_) {}
};

struct TimedPoint {
  double time{0.0};
  Point point;
  TimedPoint() {}
  TimedPoint(const double time_, const double x_, const double y_)
      : time(time_), point(x_, y_) {}
};

struct TimedPointCloud {
  double time{0.0};
  std::vector<TimedPoint> data;
};

struct GridCell {
  int hit_count{0};
};

struct GridMap {
  int id{-1};
  Pose pose;
  std::vector<GridCell> grid_cell_list;
  Point origin;
  int height{0};
  int width{0};
  double resolution{0.0};
};

}  // namespace bridge
}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_BRIDGE_H_