#include "core/grid_odometer.h"

#include <iostream>

namespace grid_odometer {

GridOdometer::GridOdometer(const Parameters& parameters)
    : parameters_{parameters} {}

void GridOdometer::Update(const bridge::LaserScan& bridge_laser_scan) {
  std::cerr << "bridge_laser_scan.data.size(): "
            << bridge_laser_scan.data.size() << std::endl;
}

}  // namespace grid_odometer