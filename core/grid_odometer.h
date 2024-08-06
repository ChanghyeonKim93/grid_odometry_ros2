#ifndef GRID_ODOMETER_CORE_GRID_ODOMETER_H_
#define GRID_ODOMETER_CORE_GRID_ODOMETER_H_

#include "eigen3/Eigen/Dense"

#include "core/bridge.h"
#include "core/types.h"

namespace grid_odometer {

struct Parameters {
  double grid_resolution{0.5};  // [m]
};

class GridOdometer {
 public:
  explicit GridOdometer(const Parameters& parameters);

  void Update(const bridge::LaserScan& bridge_laser_scan);

  bridge::GridMap GetGridMap() const;

 private:
  uint64_t ComputeGridKey(const Point point);

  const Parameters parameters_;

  GridMap grid_map_;
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_GRID_ODOMETER_H_