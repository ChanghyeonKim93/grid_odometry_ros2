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

}  // namespace bridge
}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_BRIDGE_H_