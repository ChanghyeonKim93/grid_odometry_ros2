#ifndef GRID_ODOMETER_CORE_TYPES_H_
#define GRID_ODOMETER_CORE_TYPES_H_

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

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_TYPES_H_